use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Config {
    pub camera: CameraConfig,
    #[serde(rename = "keyframe_management")]
    pub keyframe_management: KeyframeManagementConfig,
    #[serde(rename = "feature_detection")]
    pub feature_detection: FeatureDetectionConfig,
    pub optimization: OptimizationConfig,
    #[serde(rename = "imu", default)]
    pub imu: ImuConfig,
}

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
    /// IMU gyro RMS rate (rad/s) above which a keyframe is forced.
    /// Default 2.0 rad/s — fires only on extreme agile motion, not handheld walking.
    #[serde(rename = "imu_gyro_kf_threshold", default = "default_imu_gyro_kf_threshold")]
    pub imu_gyro_kf_threshold: f64,
    /// IMU mean accel magnitude (m/s²) above which a keyframe is forced.
    /// Default 50.0 m/s² — fires only on extreme impacts.
    #[serde(rename = "imu_accel_kf_threshold", default = "default_imu_accel_kf_threshold")]
    pub imu_accel_kf_threshold: f64,
}

fn default_imu_gyro_kf_threshold() -> f64 { 2.0 }
fn default_imu_accel_kf_threshold() -> f64 { 50.0 }

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
    /// Initial camera–IMU time offset in seconds: `t_imu = t_cam + cam_imu_td_s`.
    /// Positive means camera timestamps lag the IMU (camera sees the event later).
    /// Refined online during VIO. Defaults to 0.0 (calibrated or unknown).
    #[serde(rename = "cam_imu_td_s", default)]
    pub cam_imu_td_s: f64,
    /// Whether to refine cam_imu_td online using rotation residuals. Default: true.
    #[serde(rename = "cam_imu_td_online_refinement", default = "default_true")]
    pub cam_imu_td_online_refinement: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ImuConfig {
    /// Gyroscope white noise standard deviation [rad/s]
    #[serde(rename = "gyro_noise_std", default = "default_gyro_noise")]
    pub gyro_noise_std: f64,
    /// Gyro bias random walk [rad/s²]
    #[serde(rename = "gyro_random_walk_std", default = "default_gyro_rw")]
    pub gyro_random_walk_std: f64,
    /// Accelerometer white noise standard deviation [m/s²]
    #[serde(rename = "accel_noise_std", default = "default_accel_noise")]
    pub accel_noise_std: f64,
    /// Accel bias random walk [m/s^(5/2)]
    #[serde(rename = "accel_random_walk_std", default = "default_accel_rw")]
    pub accel_random_walk_std: f64,
    /// Initial gyro bias [rad/s]
    #[serde(rename = "initial_gyro_bias", default = "default_zero_vec3")]
    pub initial_gyro_bias: Vec<f64>,
    /// Initial accel bias [m/s²]
    #[serde(rename = "initial_accel_bias", default = "default_zero_vec3")]
    pub initial_accel_bias: Vec<f64>,
    /// IMU to body transform (4x4 row-major)
    #[serde(rename = "T_B_I", default = "default_identity_4x4")]
    pub t_b_i: Vec<f64>,
    /// Gravity magnitude [m/s²]
    #[serde(rename = "gravity_magnitude", default = "default_gravity")]
    pub gravity_magnitude: f64,
}

fn default_gyro_noise() -> f64 { 1.6968e-4 }
fn default_gyro_rw() -> f64 { 2.5e-5 }
fn default_accel_noise() -> f64 { 2.6669e-3 }
fn default_accel_rw() -> f64 { 6.4e-4 }
fn default_zero_vec3() -> Vec<f64> { vec![0.0, 0.0, 0.0] }
fn default_identity_4x4() -> Vec<f64> { vec![1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0] }
fn default_gravity() -> f64 { 9.81007 }

impl Default for ImuConfig {
    fn default() -> Self {
        Self {
            gyro_noise_std: default_gyro_noise(),
            gyro_random_walk_std: default_gyro_rw(),
            accel_noise_std: default_accel_noise(),
            accel_random_walk_std: default_accel_rw(),
            initial_gyro_bias: default_zero_vec3(),
            initial_accel_bias: default_zero_vec3(),
            t_b_i: default_identity_4x4(),
            gravity_magnitude: default_gravity(),
        }
    }
}

fn default_true() -> bool { true }

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
