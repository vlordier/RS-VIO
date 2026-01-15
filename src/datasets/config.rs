use crate::{Result, VIOError};
use serde::{Deserialize, Serialize};

#[doc(inline)]
pub use crate::optimization::marginalization::MarginalizationConfig;

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
    #[serde(default)]
    pub feature_detection: FeatureDetectionConfig,
    #[serde(default)]
    pub visualization: VisualizationConfig,
    pub optimization: OptimizationConfig,
    #[serde(rename = "marginalization")]
    #[serde(default)]
    pub marginalization: crate::optimization::marginalization::MarginalizationConfig,
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
    #[serde(default = "default_processing_timeout_ms")]
    pub processing_timeout_ms: u64,
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

impl Default for FeatureDetectionConfig {
    fn default() -> Self {
        Self {
            grid_cols: 30,
            max_features_per_grid: 200,
            optical_flow_max_iterations: 30,
            optical_flow_convergence_threshold: 0.005,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OptimizationConfig {
    #[serde(rename = "bundle_adjustment_max_iterations")]
    pub bundle_adjustment_max_iterations: u32,
    #[serde(rename = "pnp_max_iterations")]
    pub pnp_max_iterations: u32,
    #[serde(default = "default_imu_prior_enable")]
    pub imu_prior_enable: bool,
    #[serde(default = "default_imu_prior_weight_pos")]
    pub imu_prior_weight_pos: f64,
    #[serde(default = "default_imu_prior_weight_rot")]
    pub imu_prior_weight_rot: f64,
    #[serde(default = "default_imu_prior_huber_delta")]
    pub imu_prior_huber_delta: f64,
}

impl Config {
    pub fn load(path: &str) -> Result<Self> {
        let content = std::fs::read_to_string(path).map_err(VIOError::Io)?;
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
        let config: Config =
            serde_yaml::from_str(&content).map_err(|e| VIOError::Parse(e.to_string()))?;
        Ok(config)
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VisualizationConfig {
    #[serde(default = "default_enable_viewer")]
    pub enable_viewer: bool,
    #[serde(default = "default_stream_name")]
    pub stream_name: String,
    #[serde(default = "default_startup_delay_ms")]
    pub startup_delay_ms: u64,
    #[serde(default = "default_log_axes")]
    pub log_axes: bool,
    #[serde(default)]
    pub statistics_path: Option<String>,
}

impl Default for VisualizationConfig {
    fn default() -> Self {
        Self {
            enable_viewer: true,
            stream_name: "sivo_viewer".to_string(),
            startup_delay_ms: 500,
            log_axes: true,
            statistics_path: None,
        }
    }
}

fn default_enable_viewer() -> bool {
    true
}

fn default_stream_name() -> String {
    "sivo_viewer".to_string()
}

fn default_startup_delay_ms() -> u64 {
    500
}

fn default_log_axes() -> bool {
    true
}

fn default_processing_timeout_ms() -> u64 {
    100
}

fn default_imu_prior_enable() -> bool {
    false
}

fn default_imu_prior_weight_pos() -> f64 {
    1.0
}

fn default_imu_prior_weight_rot() -> f64 {
    1.0
}

fn default_imu_prior_huber_delta() -> f64 {
    0.0
}
