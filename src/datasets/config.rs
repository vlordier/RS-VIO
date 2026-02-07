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
            if intrinsics[2] <= 0.0
                || intrinsics[2] >= cam.image_width as f64
                || intrinsics[3] <= 0.0
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
mod tests {
    use super::*;

    #[test]
    fn test_calibration_refinement_config_defaults() {
        let config = CalibrationRefinementConfig::default();

        assert!(!config.optimize_intrinsics);
        assert!(config.optimize_focal_length);
        assert!(!config.optimize_principal_point);
        assert!(!config.optimize_distortion);
        assert_eq!(config.intrinsics_refinement_frequency, 5);
        assert!((config.max_intrinsics_change_per_update - 0.5).abs() < f64::EPSILON);
        assert!((config.intrinsics_regularization_weight - 0.01).abs() < f64::EPSILON);
    }

    #[test]
    fn test_default_helper_functions() {
        assert_eq!(default_five(), 5);
        assert!((default_max_change() - 0.5).abs() < f64::EPSILON);
        assert!((default_reg_weight() - 0.01).abs() < f64::EPSILON);
        assert!(default_true());
    }

    #[test]
    fn test_camera_config_intrinsics() {
        let camera = CameraConfig {
            image_width: 640,
            image_height: 480,
            left_intrinsics: vec![500.0, 500.0, 320.0, 240.0],
            left_distortion: vec![0.0, 0.0],
            right_intrinsics: vec![505.0, 505.0, 320.0, 240.0],
            right_distortion: vec![0.0, 0.0],
            left_model: Some("EUCM".to_string()),
            right_model: Some("EUCM".to_string()),
            T_B_Cl: vec![
                1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
            ],
            T_B_Cr: vec![
                1.0, 0.0, 0.0, -0.12, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
            ],
        };

        assert_eq!(camera.image_width, 640);
        assert_eq!(camera.image_height, 480);
        assert!((camera.left_intrinsics[0] - 500.0).abs() < f64::EPSILON);
        assert!((camera.right_intrinsics[0] - 505.0).abs() < f64::EPSILON);
        assert_eq!(camera.left_model, Some("EUCM".to_string()));
    }

    #[test]
    fn test_keyframe_management_config() {
        let kf_config = KeyframeManagementConfig {
            keyframe_window_size: 10,
            translation_threshold: 0.1,
            rotation_threshold: 0.05,
        };

        assert_eq!(kf_config.keyframe_window_size, 10);
        assert!(kf_config.translation_threshold > 0.0);
        assert!(kf_config.rotation_threshold > 0.0);
    }

    #[test]
    fn test_optimization_config() {
        let opt_config = OptimizationConfig {
            bundle_adjustment_max_iterations: 100,
            pnp_max_iterations: 50,
        };

        assert_eq!(opt_config.bundle_adjustment_max_iterations, 100);
        assert_eq!(opt_config.pnp_max_iterations, 50);
        assert!(opt_config.bundle_adjustment_max_iterations > opt_config.pnp_max_iterations);
    }
}
