use crate::{Result, VIOError};
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
    #[serde(default)]
    pub feature_detection: FeatureDetectionConfig,
    #[serde(default)]
    pub visualization: VisualizationConfig,
    #[serde(default)]
    pub debug: DebugConfig,
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

impl CameraConfig {
    /// Validate camera configuration parameters.
    pub fn validate(&self) -> Result<()> {
        // Validate image dimensions
        if self.image_width == 0 || self.image_height == 0 {
            return Err(VIOError::Config(
                "Image dimensions must be positive".to_string(),
            ));
        }
        if self.image_width > 10000 || self.image_height > 10000 {
            return Err(VIOError::Config(
                "Image dimensions unreasonably large (>10000)".to_string(),
            ));
        }

        // Validate intrinsics vectors (fx, fy, cx, cy for pinhole; more for other models)
        if self.left_intrinsics.len() < 4 {
            return Err(VIOError::Config(
                "Left intrinsics must have at least 4 parameters".to_string(),
            ));
        }
        if self.right_intrinsics.len() < 4 {
            return Err(VIOError::Config(
                "Right intrinsics must have at least 4 parameters".to_string(),
            ));
        }

        // Validate transform matrices (should be 16 elements for 4x4 matrix)
        if self.T_B_Cl.len() != 16 {
            return Err(VIOError::Config(format!(
                "T_B_Cl must have 16 elements, got {}",
                self.T_B_Cl.len()
            )));
        }
        if self.T_B_Cr.len() != 16 {
            return Err(VIOError::Config(format!(
                "T_B_Cr must have 16 elements, got {}",
                self.T_B_Cr.len()
            )));
        }

        // Check for NaN/Inf in critical parameters
        for (i, &val) in self.left_intrinsics.iter().enumerate() {
            if !val.is_finite() {
                return Err(VIOError::Config(format!(
                    "Left intrinsic[{}] is not finite",
                    i
                )));
            }
        }
        for (i, &val) in self.right_intrinsics.iter().enumerate() {
            if !val.is_finite() {
                return Err(VIOError::Config(format!(
                    "Right intrinsic[{}] is not finite",
                    i
                )));
            }
        }
        for (i, &val) in self.T_B_Cl.iter().enumerate() {
            if !val.is_finite() {
                return Err(VIOError::Config(format!("T_B_Cl[{}] is not finite", i)));
            }
        }
        for (i, &val) in self.T_B_Cr.iter().enumerate() {
            if !val.is_finite() {
                return Err(VIOError::Config(format!("T_B_Cr[{}] is not finite", i)));
            }
        }

        Ok(())
    }
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

impl KeyframeManagementConfig {
    /// Validate and clamp keyframe management parameters.
    pub fn validate_and_clamp(&mut self) {
        // Clamp window size to reasonable range [2, 50]
        if self.keyframe_window_size < 2 {
            log::warn!(
                "keyframe_window_size {} too small, clamping to 2",
                self.keyframe_window_size
            );
            self.keyframe_window_size = 2;
        }
        if self.keyframe_window_size > 50 {
            log::warn!(
                "keyframe_window_size {} too large, clamping to 50",
                self.keyframe_window_size
            );
            self.keyframe_window_size = 50;
        }

        // Clamp translation threshold to [0.01, 10.0] meters
        if self.translation_threshold < 0.01 {
            log::warn!(
                "translation_threshold {} too small, clamping to 0.01",
                self.translation_threshold
            );
            self.translation_threshold = 0.01;
        }
        if self.translation_threshold > 10.0 {
            log::warn!(
                "translation_threshold {} too large, clamping to 10.0",
                self.translation_threshold
            );
            self.translation_threshold = 10.0;
        }

        // Clamp rotation threshold to [0.01, 3.14] radians
        if self.rotation_threshold < 0.01 {
            log::warn!(
                "rotation_threshold {} too small, clamping to 0.01",
                self.rotation_threshold
            );
            self.rotation_threshold = 0.01;
        }
        if self.rotation_threshold > std::f64::consts::PI {
            log::warn!(
                "rotation_threshold {} too large, clamping to π",
                self.rotation_threshold
            );
            self.rotation_threshold = std::f64::consts::PI;
        }

        // Clamp processing timeout to [1, 10000] ms
        if self.processing_timeout_ms == 0 {
            log::warn!("processing_timeout_ms cannot be 0, setting to 1");
            self.processing_timeout_ms = 1;
        }
        if self.processing_timeout_ms > 10000 {
            log::warn!(
                "processing_timeout_ms {} too large, clamping to 10000",
                self.processing_timeout_ms
            );
            self.processing_timeout_ms = 10000;
        }
    }
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
    #[serde(default = "default_subpixel_enable")]
    pub subpixel_enable: bool,
    #[serde(default = "default_subpixel_iterations")]
    pub subpixel_iterations: u32,
    #[serde(default = "default_subpixel_threshold")]
    pub subpixel_threshold: f64,
}

impl Default for FeatureDetectionConfig {
    fn default() -> Self {
        Self {
            grid_cols: 30,
            max_features_per_grid: 200,
            optical_flow_max_iterations: 30,
            optical_flow_convergence_threshold: 0.005,
            subpixel_enable: true,
            subpixel_iterations: 15,
            subpixel_threshold: 0.0005,
        }
    }
}

impl FeatureDetectionConfig {
    /// Validate and clamp feature detection parameters.
    pub fn validate_and_clamp(&mut self) {
        // Clamp grid columns to [1, 100]
        if self.grid_cols == 0 {
            log::warn!("grid_cols cannot be 0, setting to 1");
            self.grid_cols = 1;
        }
        if self.grid_cols > 100 {
            log::warn!("grid_cols {} too large, clamping to 100", self.grid_cols);
            self.grid_cols = 100;
        }

        // Clamp max features per grid to [1, 1000]
        if self.max_features_per_grid == 0 {
            log::warn!("max_features_per_grid cannot be 0, setting to 1");
            self.max_features_per_grid = 1;
        }
        if self.max_features_per_grid > 1000 {
            log::warn!(
                "max_features_per_grid {} too large, clamping to 1000",
                self.max_features_per_grid
            );
            self.max_features_per_grid = 1000;
        }

        // Clamp optical flow iterations to [1, 100]
        if self.optical_flow_max_iterations == 0 {
            log::warn!("optical_flow_max_iterations cannot be 0, setting to 1");
            self.optical_flow_max_iterations = 1;
        }
        if self.optical_flow_max_iterations > 100 {
            log::warn!(
                "optical_flow_max_iterations {} too large, clamping to 100",
                self.optical_flow_max_iterations
            );
            self.optical_flow_max_iterations = 100;
        }

        // Clamp convergence threshold to [1e-6, 1.0]
        if self.optical_flow_convergence_threshold < 1e-6 {
            log::warn!(
                "optical_flow_convergence_threshold {} too small, clamping to 1e-6",
                self.optical_flow_convergence_threshold
            );
            self.optical_flow_convergence_threshold = 1e-6;
        }
        if self.optical_flow_convergence_threshold > 1.0 {
            log::warn!(
                "optical_flow_convergence_threshold {} too large, clamping to 1.0",
                self.optical_flow_convergence_threshold
            );
            self.optical_flow_convergence_threshold = 1.0;
        }

        // Clamp subpixel iterations to [1, 50]
        if self.subpixel_iterations == 0 {
            log::warn!("subpixel_iterations cannot be 0, setting to 1");
            self.subpixel_iterations = 1;
        }
        if self.subpixel_iterations > 50 {
            log::warn!(
                "subpixel_iterations {} too large, clamping to 50",
                self.subpixel_iterations
            );
            self.subpixel_iterations = 50;
        }

        // Clamp subpixel threshold to [1e-8, 0.1]
        if self.subpixel_threshold < 1e-8 {
            log::warn!(
                "subpixel_threshold {} too small, clamping to 1e-8",
                self.subpixel_threshold
            );
            self.subpixel_threshold = 1e-8;
        }
        if self.subpixel_threshold > 0.1 {
            log::warn!(
                "subpixel_threshold {} too large, clamping to 0.1",
                self.subpixel_threshold
            );
            self.subpixel_threshold = 0.1;
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

impl OptimizationConfig {
    /// Validate and clamp optimization parameters.
    pub fn validate_and_clamp(&mut self) {
        // Clamp bundle adjustment iterations to [1, 100]
        if self.bundle_adjustment_max_iterations == 0 {
            log::warn!("bundle_adjustment_max_iterations cannot be 0, setting to 1");
            self.bundle_adjustment_max_iterations = 1;
        }
        if self.bundle_adjustment_max_iterations > 100 {
            log::warn!(
                "bundle_adjustment_max_iterations {} too large, clamping to 100",
                self.bundle_adjustment_max_iterations
            );
            self.bundle_adjustment_max_iterations = 100;
        }

        // Clamp PnP iterations to [1, 100]
        if self.pnp_max_iterations == 0 {
            log::warn!("pnp_max_iterations cannot be 0, setting to 1");
            self.pnp_max_iterations = 1;
        }
        if self.pnp_max_iterations > 100 {
            log::warn!(
                "pnp_max_iterations {} too large, clamping to 100",
                self.pnp_max_iterations
            );
            self.pnp_max_iterations = 100;
        }

        // Clamp IMU prior weights to [1e-6, 1e6]
        if self.imu_prior_weight_pos < 1e-6 {
            log::warn!(
                "imu_prior_weight_pos {} too small, clamping to 1e-6",
                self.imu_prior_weight_pos
            );
            self.imu_prior_weight_pos = 1e-6;
        }
        if self.imu_prior_weight_pos > 1e6 {
            log::warn!(
                "imu_prior_weight_pos {} too large, clamping to 1e6",
                self.imu_prior_weight_pos
            );
            self.imu_prior_weight_pos = 1e6;
        }

        if self.imu_prior_weight_rot < 1e-6 {
            log::warn!(
                "imu_prior_weight_rot {} too small, clamping to 1e-6",
                self.imu_prior_weight_rot
            );
            self.imu_prior_weight_rot = 1e-6;
        }
        if self.imu_prior_weight_rot > 1e6 {
            log::warn!(
                "imu_prior_weight_rot {} too large, clamping to 1e6",
                self.imu_prior_weight_rot
            );
            self.imu_prior_weight_rot = 1e6;
        }

        // Clamp Huber delta to [1e-3, 100.0]
        if self.imu_prior_huber_delta < 1e-3 {
            log::warn!(
                "imu_prior_huber_delta {} too small, clamping to 1e-3",
                self.imu_prior_huber_delta
            );
            self.imu_prior_huber_delta = 1e-3;
        }
        if self.imu_prior_huber_delta > 100.0 {
            log::warn!(
                "imu_prior_huber_delta {} too large, clamping to 100.0",
                self.imu_prior_huber_delta
            );
            self.imu_prior_huber_delta = 100.0;
        }
    }
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
        let mut config: Config =
            serde_yaml::from_str(&content).map_err(|e| VIOError::Parse(e.to_string()))?;
        config.validate_and_clamp()?;
        Ok(config)
    }

    /// Validate and clamp configuration parameters to safe ranges.
    /// Returns error if critical parameters are invalid.
    pub fn validate_and_clamp(&mut self) -> Result<()> {
        self.camera.validate()?;
        self.keyframe_management.validate_and_clamp();
        self.feature_detection.validate_and_clamp();
        self.optimization.validate_and_clamp();
        self.marginalization.validate_and_clamp();
        Ok(())
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

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct DebugConfig {
    #[serde(default = "default_use_imu")]
    pub use_imu: bool,
    #[serde(default = "default_use_feature_tracking")]
    pub use_feature_tracking: bool,
    #[serde(default = "default_use_triangulation")]
    pub use_triangulation: bool,
    #[serde(default = "default_use_fallback_depth")]
    pub use_fallback_depth: bool,
    #[serde(default = "default_min_stereo_matches")]
    pub min_stereo_matches: usize,
}

fn default_use_imu() -> bool {
    true
}
fn default_use_feature_tracking() -> bool {
    true
}
fn default_use_triangulation() -> bool {
    true
}
fn default_use_fallback_depth() -> bool {
    false
}
fn default_min_stereo_matches() -> usize {
    2
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
    1.0
}

fn default_subpixel_enable() -> bool {
    true
}

fn default_subpixel_iterations() -> u32 {
    15
}

fn default_subpixel_threshold() -> f64 {
    0.0005
}
