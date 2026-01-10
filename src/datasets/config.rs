use serde::{Deserialize, Serialize};

/// Main configuration structure for the VIO system.
///
/// This struct holds all configuration parameters loaded from YAML files,
/// including camera intrinsics, keyframe management settings, feature detection
/// parameters, and optimization settings.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Config {
    pub camera: CameraConfig,
    #[serde(rename = "keyframe_management")]
    pub keyframe_management: KeyframeManagementConfig,
    #[serde(rename = "feature_detection")]
    pub feature_detection: FeatureDetectionConfig,
    pub optimization: OptimizationConfig,
}

/// Camera configuration including intrinsics, distortion, and extrinsics.
///
/// Contains parameters for both left and right cameras in a stereo setup,
/// including image dimensions, calibration parameters, and transforms.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CameraConfig {
    #[serde(rename = "image_width")]
    pub image_width: u32,
    #[serde(rename = "image_height")]
    pub image_height: u32,
    #[serde(rename = "left_intrinsics")]
    pub left_intrinsics: Vec<f64>,
    #[serde(rename = "left_distortion")]
    pub left_distortion: Vec<f64>,
    #[serde(rename = "right_intrinsics")]
    pub right_intrinsics: Vec<f64>,
    #[serde(rename = "right_distortion")]
    pub right_distortion: Vec<f64>,
    #[serde(rename = "left_model")]
    pub left_model: Option<String>,
    #[serde(rename = "right_model")]
    pub right_model: Option<String>,
    #[serde(rename = "T_B_Cl")]
    pub T_B_Cl: Vec<f64>,
    #[serde(rename = "T_B_Cr")]
    pub T_B_Cr: Vec<f64>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct KeyframeManagementConfig {
    #[serde(rename = "keyframe_window_size")]
    pub keyframe_window_size: u32,
    #[serde(rename = "translation_threshold")]
    pub translation_threshold: f64,
    #[serde(rename = "rotation_threshold")]
    pub rotation_threshold: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FeatureDetectionConfig {
    #[serde(rename = "grid_size")]
    pub grid_cols: u32,
    #[serde(rename = "max_features_per_grid")]
    pub max_features_per_grid: u32,
    #[serde(rename = "optical_flow_max_iterations")]
    pub optical_flow_max_iterations: u32,
    #[serde(rename = "optical_flow_convergence_threshold")]
    pub optical_flow_convergence_threshold: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OptimizationConfig {
    #[serde(rename = "bundle_adjustment_max_iterations")]
    pub bundle_adjustment_max_iterations: u32,
    #[serde(rename = "pnp_max_iterations")]
    pub pnp_max_iterations: u32,
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
        Ok(config)
    }
}
