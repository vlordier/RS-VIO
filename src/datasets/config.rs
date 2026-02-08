//! VIO pipeline configuration: camera, feature detection, optimization, and keyframe parameters.

use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Config {
    pub camera: CameraConfig,
    pub keyframe_management: KeyframeManagementConfig,
    pub feature_detection: FeatureDetectionConfig,
    pub optimization: OptimizationConfig,
    #[serde(default)]
    pub calibration: Option<CalibrationRefinementConfig>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CameraConfig {
    pub image_width: u32,
    pub image_height: u32,
    pub left_intrinsics: Vec<f64>,
    pub left_distortion: Vec<f64>,
    pub right_intrinsics: Vec<f64>,
    pub right_distortion: Vec<f64>,
    pub left_model: Option<String>,
    pub right_model: Option<String>,
    #[serde(rename = "T_B_Cl")]
    pub T_B_Cl: Vec<f64>,
    #[serde(rename = "T_B_Cr")]
    pub T_B_Cr: Vec<f64>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct KeyframeManagementConfig {
    pub keyframe_window_size: u32,
    pub translation_threshold: f64,
    pub rotation_threshold: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FeatureDetectionConfig {
    #[serde(rename = "grid_size")]
    pub grid_cols: u32,
    pub max_features_per_grid: u32,
    pub optical_flow_max_iterations: u32,
    pub optical_flow_convergence_threshold: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OptimizationConfig {
    pub bundle_adjustment_max_iterations: u32,
    pub pnp_max_iterations: u32,
}

/// Online intrinsics refinement configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CalibrationRefinementConfig {
    /// Enable online intrinsics refinement during VIO
    #[serde(default)]
    pub optimize_intrinsics: bool,
    /// Optimize focal length (fx, fy)
    #[serde(default = "default_true")]
    pub optimize_focal_length: bool,
    /// Optimize principal point (cx, cy)
    #[serde(default)]
    pub optimize_principal_point: bool,
    /// Optimize distortion parameters
    #[serde(default)]
    pub optimize_distortion: bool,
    /// Refine intrinsics every N keyframes
    #[serde(default = "default_five")]
    pub intrinsics_refinement_frequency: usize,
    /// Max change per update (pixels, regularization)
    #[serde(default = "default_max_change")]
    pub max_intrinsics_change_per_update: f64,
    /// Weight of regularization to original intrinsics
    #[serde(default = "default_reg_weight")]
    pub intrinsics_regularization_weight: f64,
}

impl Default for CalibrationRefinementConfig {
    fn default() -> Self {
        Self {
            optimize_intrinsics: false,
            optimize_focal_length: true,
            optimize_principal_point: false,
            optimize_distortion: false,
            intrinsics_refinement_frequency: 5,
            max_intrinsics_change_per_update: 0.5,
            intrinsics_regularization_weight: 0.01,
        }
    }
}

const fn default_true() -> bool {
    true
}

const fn default_five() -> usize {
    5
}

const fn default_max_change() -> f64 {
    0.5
}

const fn default_reg_weight() -> f64 {
    0.01
}

impl Config {
    pub fn load(path: &str) -> anyhow::Result<Self> {
        let content = std::fs::read_to_string(path)?;
        // Strip YAML directive if present (e.g., %YAML:1.0)
        let content = if content.trim_start().starts_with("%YAML") {
            content
                .lines()
                .skip_while(|line| line.trim_start().starts_with("%"))
                .collect::<Vec<_>>()
                .join("\n")
        } else {
            content
        };
        let config: Config = serde_yaml::from_str(&content)?;
        config.validate()?;
        Ok(config)
    }

    /// Validate configuration values to catch bad YAML before panicking downstream.
    fn validate(&self) -> anyhow::Result<()> {
        let cam = &self.camera;

        if cam.image_width == 0 || cam.image_height == 0 {
            anyhow::bail!(
                "Invalid image dimensions: {}x{}",
                cam.image_width,
                cam.image_height
            );
        }

        if cam.left_intrinsics.len() != 4 {
            anyhow::bail!(
                "left_intrinsics must have exactly 4 elements [fx, fy, cx, cy], got {}",
                cam.left_intrinsics.len()
            );
        }
        if cam.right_intrinsics.len() != 4 {
            anyhow::bail!(
                "right_intrinsics must have exactly 4 elements [fx, fy, cx, cy], got {}",
                cam.right_intrinsics.len()
            );
        }

        if cam.T_B_Cl.len() != 16 {
            anyhow::bail!(
                "T_B_Cl must have exactly 16 elements (4x4 row-major), got {}",
                cam.T_B_Cl.len()
            );
        }
        if cam.T_B_Cr.len() != 16 {
            anyhow::bail!(
                "T_B_Cr must have exactly 16 elements (4x4 row-major), got {}",
                cam.T_B_Cr.len()
            );
        }

        // Sanity check intrinsics are positive
        for (name, intrinsics) in [
            ("left", &cam.left_intrinsics),
            ("right", &cam.right_intrinsics),
        ] {
            if intrinsics[0] <= 0.0 || intrinsics[1] <= 0.0 {
                anyhow::bail!(
                    "{} focal length must be positive: fx={}, fy={}",
                    name,
                    intrinsics[0],
                    intrinsics[1]
                );
            }
        }

        // Sanity check principal point is within image bounds
        for (name, intrinsics) in [
            ("left", &cam.left_intrinsics),
            ("right", &cam.right_intrinsics),
        ] {
            if intrinsics[2] < 0.0
                || intrinsics[2] >= cam.image_width as f64
                || intrinsics[3] < 0.0
                || intrinsics[3] >= cam.image_height as f64
            {
                anyhow::bail!(
                    "{} principal point ({}, {}) outside image bounds ({}x{})",
                    name,
                    intrinsics[2],
                    intrinsics[3],
                    cam.image_width,
                    cam.image_height
                );
            }
        }

        // Validate camera model names and distortion param counts
        for (name, model_opt, distortion) in [
            ("left", &cam.left_model, &cam.left_distortion),
            ("right", &cam.right_model, &cam.right_distortion),
        ] {
            let model_str = model_opt.as_deref().unwrap_or("pinhole-radtan");
            if model_str.eq_ignore_ascii_case("eucm") {
                if distortion.len() < 2 {
                    anyhow::bail!(
                        "{} camera uses EUCM but distortion has {} params (need >= 2 for alpha, beta)",
                        name,
                        distortion.len()
                    );
                }
            } else if !model_str.eq_ignore_ascii_case("pinhole-radtan") {
                log::warn!(
                    "Unknown {} camera model '{}', falling back to pinhole-radtan",
                    name,
                    model_str
                );
            }
        }

        Ok(())
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use super::*;

    #[test]
    fn test_load_tum_vi_config_has_realistic_intrinsics() {
        // Validate that the real TUM-VI config loads and has physically reasonable values
        let config = Config::load("config/tum_vi.yaml").expect("TUM-VI config should load");

        // Image dimensions should be TUM-VI standard (512x512)
        assert!(
            config.camera.image_width > 0 && config.camera.image_height > 0,
            "Image dimensions must be positive"
        );
        assert!(
            config.camera.image_width <= 2048 && config.camera.image_height <= 2048,
            "Image dimensions must be reasonable for a VIO camera"
        );

        // Focal lengths should be positive and physically plausible
        let fx = config.camera.left_intrinsics[0];
        let fy = config.camera.left_intrinsics[1];
        assert!(fx > 50.0 && fx < 2000.0, "fx={fx} must be in [50, 2000]");
        assert!(fy > 50.0 && fy < 2000.0, "fy={fy} must be in [50, 2000]");

        // Principal point should be near the image center
        let cx = config.camera.left_intrinsics[2];
        let cy = config.camera.left_intrinsics[3];
        let half_w = config.camera.image_width as f64 / 2.0;
        let half_h = config.camera.image_height as f64 / 2.0;
        assert!(
            (cx - half_w).abs() < half_w,
            "cx={cx} should be within image width"
        );
        assert!(
            (cy - half_h).abs() < half_h,
            "cy={cy} should be within image height"
        );

        // Extrinsics should be 4x4 matrices (16 elements)
        assert_eq!(config.camera.T_B_Cl.len(), 16);
        assert_eq!(config.camera.T_B_Cr.len(), 16);

        // Feature detection should have reasonable grid settings
        assert!(config.feature_detection.grid_cols > 0);
        assert!(config.feature_detection.max_features_per_grid > 0);
        assert!(config.feature_detection.optical_flow_max_iterations > 0);
    }

    #[test]
    fn test_config_serde_roundtrip() {
        // Serialize a Config to YAML and deserialize back — values should survive the roundtrip
        let original = Config {
            camera: CameraConfig {
                image_width: 512,
                image_height: 512,
                left_intrinsics: vec![190.0, 190.0, 256.0, 256.0],
                left_distortion: vec![0.0, 0.0],
                right_intrinsics: vec![190.0, 190.0, 256.0, 256.0],
                right_distortion: vec![0.0, 0.0],
                left_model: Some("EUCM".to_string()),
                right_model: Some("EUCM".to_string()),
                T_B_Cl: vec![
                    1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
                ],
                T_B_Cr: vec![
                    1.0, 0.0, 0.0, -0.1, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
                ],
            },
            keyframe_management: KeyframeManagementConfig {
                keyframe_window_size: 8,
                translation_threshold: 0.2,
                rotation_threshold: 0.1,
            },
            feature_detection: FeatureDetectionConfig {
                grid_cols: 16,
                max_features_per_grid: 80,
                optical_flow_max_iterations: 30,
                optical_flow_convergence_threshold: 0.01,
            },
            optimization: OptimizationConfig {
                bundle_adjustment_max_iterations: 10,
                pnp_max_iterations: 5,
            },
            calibration: None,
        };

        let yaml = serde_yaml::to_string(&original).expect("serialize should succeed");
        let roundtripped: Config = serde_yaml::from_str(&yaml).expect("deserialize should succeed");

        assert_eq!(roundtripped.camera.image_width, original.camera.image_width);
        assert_eq!(roundtripped.camera.image_height, original.camera.image_height);
        assert!((roundtripped.camera.left_intrinsics[0] - original.camera.left_intrinsics[0]).abs() < f64::EPSILON);
        assert_eq!(roundtripped.camera.left_model, original.camera.left_model);
        assert_eq!(
            roundtripped.feature_detection.grid_cols,
            original.feature_detection.grid_cols
        );
        assert_eq!(
            roundtripped.optimization.bundle_adjustment_max_iterations,
            original.optimization.bundle_adjustment_max_iterations
        );
    }

    #[test]
    fn test_config_validate_rejects_zero_dimensions() {
        let yaml = r#"
camera:
  image_width: 0
  image_height: 480
  left_intrinsics: [500.0, 500.0, 320.0, 240.0]
  left_distortion: [0.0]
  right_intrinsics: [500.0, 500.0, 320.0, 240.0]
  right_distortion: [0.0]
  T_B_Cl: [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1]
  T_B_Cr: [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1]
keyframe_management:
  keyframe_window_size: 8
  translation_threshold: 0.2
  rotation_threshold: 0.1
feature_detection:
  grid_size: 16
  max_features_per_grid: 80
  optical_flow_max_iterations: 30
  optical_flow_convergence_threshold: 0.01
optimization:
  bundle_adjustment_max_iterations: 5
  pnp_max_iterations: 5
"#;
        let config: Config = serde_yaml::from_str(yaml).expect("YAML parsing should succeed");
        let result = config.validate();
        assert!(result.is_err(), "Zero image width should fail validation");
        assert!(
            result.unwrap_err().to_string().contains("Invalid image dimensions"),
            "Error should mention image dimensions"
        );
    }

    #[test]
    fn test_config_validate_rejects_negative_focal_length() {
        let yaml = r#"
camera:
  image_width: 640
  image_height: 480
  left_intrinsics: [-500.0, 500.0, 320.0, 240.0]
  left_distortion: [0.0]
  right_intrinsics: [500.0, 500.0, 320.0, 240.0]
  right_distortion: [0.0]
  T_B_Cl: [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1]
  T_B_Cr: [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1]
keyframe_management:
  keyframe_window_size: 8
  translation_threshold: 0.2
  rotation_threshold: 0.1
feature_detection:
  grid_size: 16
  max_features_per_grid: 80
  optical_flow_max_iterations: 30
  optical_flow_convergence_threshold: 0.01
optimization:
  bundle_adjustment_max_iterations: 5
  pnp_max_iterations: 5
"#;
        let config: Config = serde_yaml::from_str(yaml).expect("YAML parsing should succeed");
        let result = config.validate();
        assert!(result.is_err(), "Negative focal length should fail validation");
        assert!(
            result.unwrap_err().to_string().contains("focal length must be positive"),
            "Error should mention focal length"
        );
    }
}
