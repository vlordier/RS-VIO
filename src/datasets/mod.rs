//! # Dataset Module
//!
//! Dataset players for standard VIO benchmarks and configuration management.
//!
//! ## Overview
//!
//! This module provides tools for loading and processing stereo VIO benchmark datasets:
//! - **EuRoC**: Micro Aerial Vehicle dataset with IMU
//! - **TUM-VI**: TUM Visual-Inertial dataset
//! - **4Seasons**: Large-scale long-term dataset with appearance changes
//! - **LiveCamera**: Real-time stereo camera capture
//!
//! ## Components
//!
//! - [`EurocPlayer`](crate::datasets::euroc_player::EurocPlayer) - EuRoC dataset loader
//! - [`TUMVIPlayer`](crate::datasets::tum_vi_player::TUMVIPlayer) - TUM-VI dataset loader
//! - [`FourSeasonsPlayer`](crate::datasets::fourseasons_player::FourSeasonsPlayer) - 4Seasons dataset loader
//! - [`LiveCameraPlayer`](crate::datasets::live_camera_player::LiveCameraPlayer) - Live stereo camera
//! - [`Config`] - VIO system configuration (YAML)
//!
//! ## Live Camera Usage
//!
//! ```no_run
//! use rs_vio::datasets::{LiveCameraPlayer, Config};
//!
//! # fn main() -> Result<(), Box<dyn std::error::Error>> {
//! let config = Config::load("config/euroc_vio.yaml")?;
//! let player = LiveCameraPlayer::new(rs_vio::datasets::LiveCameraConfig {
//!     camera_index_left: 0,
//!     camera_index_right: 1,
//!     frame_rate: 30,
//!     ..Default::default()
//! });
//! # Ok(())
//! # }
//! ```
//!
//! ## Dataset Formats
//!
//! Each dataset provides:
//! - Stereo image pairs with synchronized timestamps
//! - Camera intrinsics and extrinsics
//! - IMU data (gyro, accelerometer) with synchronization
//! - Ground truth trajectories for evaluation
//!
//! ## Configuration Format (YAML)
//!
//! ```yaml
//! camera:
//!   image_width: 640
//!   image_height: 480
//!   left_intrinsics: [fx, fy, cx, cy]
//!   left_distortion: [k1, k2, p1, p2]
//!   left_model: pinhole-radtan
//!
//! keyframe_management:
//!   keyframe_window_size: 5
//!
//! feature_detection:
//!   grid_size: 15
//! ```
//!
//! ## Performance Notes
//!
//! - **EuRoC**: Fast (~10-11 min sequences)
//! - **TUM-VI**: Medium (~5-10 min sequences)
//! - **4Seasons**: Large (~1.5 hour sequences)
//! - **LiveCamera**: Real-time processing at configured frame rate
//!
//! ## See Also
//! - [`crate::datasets::config::Config`] - Configuration loading
//! - [`crate::estimator::Estimator`] - VIO pipeline

pub mod config;
pub mod euroc_player;
pub mod fourseasons_player;
pub mod frame_processor_trait;
pub mod live_camera_player;
pub mod player_trait;
pub mod trajectory_eval;
pub mod tum_vi; // Real-world TUM-VI dataset loader
pub mod tum_vi_player; // ATE/RPE trajectory evaluation

// Re-export player types for convenience
pub use euroc_player::EurocPlayer;
pub use fourseasons_player::FourSeasonsPlayer;
pub use live_camera_player::{LiveCameraConfig, LiveCameraPlayer};
pub use tum_vi_player::TUMVIPlayer;

pub use crate::datasets::config::Config;
use camera_intrinsic_model::generic_model::CameraModel;
use camera_intrinsic_model::models::opencv5::OpenCVModel5;
use camera_intrinsic_model::models::EUCM;
// nalgebra 0.34 required by camera-intrinsic-model, while main codebase uses 0.33
// TODO: Consider upgrading main codebase to 0.34 or using a unified approach
use crate::unwrap_or_log;
use nalgebra034;

// Image data structure
#[derive(Debug, Clone)]
pub struct ImageData {
    pub timestamp: i64, // nanoseconds
    pub filename: String,
}

// IMU data structure (placeholder - will be implemented when IMU support is added)
#[derive(Debug, Clone)]
pub struct ImuData {
    pub timestamp: i64,
    pub gyro: [f64; 3],
    pub accel: [f64; 3],
}

// Frame context for tracking processing state
#[derive(Debug, Clone)]
pub struct FrameContext {
    pub current_idx: usize,
    pub processed_frames: usize,
    pub previous_frame_timestamp: i64,
    pub step_mode: bool,
    pub auto_play: bool,
    pub advance_frame: bool,
    // Instrumentation accumulators
    pub io_decode_time_ms_sum: f64,
    pub imu_fetch_time_ms_sum: f64,
    pub estimator_time_ms_sum: f64,
    pub frames_timed: usize,
}

impl FrameContext {
    pub fn new(step_mode: bool) -> Self {
        FrameContext {
            current_idx: 0,
            processed_frames: 0,
            previous_frame_timestamp: 0,
            step_mode,
            auto_play: !step_mode,
            advance_frame: false,
            io_decode_time_ms_sum: 0.0,
            imu_fetch_time_ms_sum: 0.0,
            estimator_time_ms_sum: 0.0,
            frames_timed: 0,
        }
    }
}

// Result structure with statistics
#[derive(Debug, Default)]
pub struct PlayerResult {
    pub success: bool,
    pub error_message: String,
    pub processed_frames: usize,
    pub frame_processing_times: Vec<f64>, // milliseconds
    pub average_processing_time_ms: f64,
}

#[derive(Debug)]
pub struct PlayerConfig {
    pub config_path: String,
    pub dataset_path: String,
    pub enable_statistics: bool,
    pub enable_console_statistics: bool,
    pub step_mode: bool,
    pub stats_output_path: Option<String>,
}

/// Enum to represent different camera model types
#[derive(Clone, Debug)]
pub enum CameraModelType {
    OpenCV5(OpenCVModel5<f64>),
    EUCM(EUCM<f64>),
}

impl CameraModelType {
    /// Get a reference to the underlying camera model as a trait object
    pub fn as_camera_model(&self) -> &dyn CameraModel<f64> {
        match self {
            CameraModelType::OpenCV5(cam) => cam,
            CameraModelType::EUCM(cam) => cam,
        }
    }
}

/// Create camera models from config.
/// This helper function creates camera models from the configuration
/// for both left and right cameras. Supports OpenCVModel5 and EUCM models.
pub fn create_camera_models_from_config(config: &Config) -> (CameraModelType, CameraModelType) {
    let cam = &config.camera;

    // Determine camera model type (assuming same for left and right)
    // TODO make this code more generic (and elegant)
    // Using unwrap_or doesn't make sense here, if we can't get the params, we should error out
    let left_model_str = unwrap_or_log!(
        cam.left_model.as_deref(),
        "pinhole-radtan",
        "Missing left_model; defaulting to pinhole-radtan"
    );

    // Helpers to log when intrinsics/distortion parameters are missing.
    let left_intrinsic = |idx: usize, label: &str, fallback: f64| {
        unwrap_or_log!(
            cam.left_intrinsics.get(idx).copied(),
            fallback,
            "Missing left {} (index {}); defaulting to {}",
            label,
            idx,
            fallback
        )
    };

    let left_distortion = |idx: usize, label: &str, fallback: f64| {
        unwrap_or_log!(
            cam.left_distortion.get(idx).copied(),
            fallback,
            "Missing left {} (index {}); defaulting to {}",
            label,
            idx,
            fallback
        )
    };

    // Create left camera model
    let left_cam = if left_model_str == "EUCM" || left_model_str == "eucm" {
        // EUCM model: [fx, fy, cx, cy, alpha, beta]
        let eucm_params_vec: Vec<f64> = vec![
            left_intrinsic(0, "fx", 500.0),
            left_intrinsic(1, "fy", 500.0),
            left_intrinsic(2, "cx", 320.0),
            left_intrinsic(3, "cy", 240.0),
            left_distortion(0, "alpha", 0.5),
            left_distortion(1, "beta", 1.0),
        ];
        let eucm_params = nalgebra034::DVector::from_vec(eucm_params_vec);
        CameraModelType::EUCM(EUCM::new(&eucm_params, cam.image_width, cam.image_height))
    } else {
        let left_opencv_params_vec = vec![
            left_intrinsic(0, "fx", 500.0),
            left_intrinsic(1, "fy", 500.0),
            left_intrinsic(2, "cx", 320.0),
            left_intrinsic(3, "cy", 240.0),
            left_distortion(0, "k1", 0.0),
            left_distortion(1, "k2", 0.0),
            left_distortion(2, "p1", 0.0),
            left_distortion(3, "p2", 0.0),
            left_distortion(4, "k3", 0.0),
        ];
        let left_params = nalgebra034::DVector::from_vec(left_opencv_params_vec);
        CameraModelType::OpenCV5(OpenCVModel5::new(
            &left_params,
            cam.image_width,
            cam.image_height,
        ))
    };

    // Create right camera model (assuming same model type as left)
    let right_model_str = unwrap_or_log!(
        cam.right_model.as_deref(),
        left_model_str,
        "Missing right_model; defaulting to {}",
        left_model_str
    );

    let right_intrinsic = |idx: usize, label: &str, fallback: f64| {
        unwrap_or_log!(
            cam.right_intrinsics.get(idx).copied(),
            fallback,
            "Missing right {} (index {}); defaulting to {}",
            label,
            idx,
            fallback
        )
    };

    let right_distortion = |idx: usize, label: &str, fallback: f64| {
        unwrap_or_log!(
            cam.right_distortion.get(idx).copied(),
            fallback,
            "Missing right {} (index {}); defaulting to {}",
            label,
            idx,
            fallback
        )
    };
    let right_cam = if right_model_str == "EUCM" || right_model_str == "eucm" {
        // EUCM model: [fx, fy, cx, cy, alpha, beta]
        let eucm_params_vec: Vec<f64> = vec![
            right_intrinsic(0, "fx", 500.0),
            right_intrinsic(1, "fy", 500.0),
            right_intrinsic(2, "cx", 320.0),
            right_intrinsic(3, "cy", 240.0),
            right_distortion(0, "alpha", 0.5),
            right_distortion(1, "beta", 1.0),
        ];
        let eucm_params = nalgebra034::DVector::from_vec(eucm_params_vec);
        CameraModelType::EUCM(EUCM::new(&eucm_params, cam.image_width, cam.image_height))
    } else {
        let right_opencv_params_vec = vec![
            right_intrinsic(0, "fx", 500.0),
            right_intrinsic(1, "fy", 500.0),
            right_intrinsic(2, "cx", 320.0),
            right_intrinsic(3, "cy", 240.0),
            right_distortion(0, "k1", 0.0),
            right_distortion(1, "k2", 0.0),
            right_distortion(2, "p1", 0.0),
            right_distortion(3, "p2", 0.0),
            right_distortion(4, "k3", 0.0),
        ];
        let right_params = nalgebra034::DVector::from_vec(right_opencv_params_vec);
        CameraModelType::OpenCV5(OpenCVModel5::new(
            &right_params,
            cam.image_width,
            cam.image_height,
        ))
    };

    (left_cam, right_cam)
}

#[cfg(test)]
#[allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::float_cmp
)]
mod tests {
    use super::*;
    use crate::datasets::config::Config;
    use serde_yaml;

    fn create_test_config() -> Config {
        let yaml = r#"
camera:
  image_width: 640
  image_height: 480
  left_intrinsics: [500.0, 500.0, 320.0, 240.0]
  left_distortion: [0.0, 0.0, 0.0, 0.0]
  right_intrinsics: [500.0, 500.0, 320.0, 240.0]
  right_distortion: [0.0, 0.0, 0.0, 0.0]
  left_model: pinhole-radtan
  T_B_Cl: [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
  T_B_Cr: [1.0, 0.0, 0.0, 0.1, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
keyframe_management:
  keyframe_window_size: 5
  translation_threshold: 0.1
  rotation_threshold: 0.1
feature_detection:
    grid_size: 10
    max_features_per_grid: 50
    optical_flow_max_iterations: 30
    optical_flow_convergence_threshold: 0.01
optimization:
    bundle_adjustment_max_iterations: 10
    pnp_max_iterations: 5
"#;
        serde_yaml::from_str(yaml).unwrap()
    }

    #[test]
    fn test_camera_model_creation_open_cv() {
        let config = create_test_config();
        let (left_cam, right_cam) = create_camera_models_from_config(&config);

        match left_cam {
            CameraModelType::OpenCV5(_) => (),
            _ => panic!("Expected OpenCV5 model"),
        }
        match right_cam {
            CameraModelType::OpenCV5(_) => (),
            _ => panic!("Expected OpenCV5 model"),
        }
    }

    #[test]
    fn test_camera_model_creation_eucm() {
        let mut config = create_test_config();
        config.camera.left_model = Some("EUCM".to_string());
        config.camera.right_model = Some("EUCM".to_string());

        let (left_cam, right_cam) = create_camera_models_from_config(&config);

        match left_cam {
            CameraModelType::EUCM(_) => (),
            _ => panic!("Expected EUCM model"),
        }
        match right_cam {
            CameraModelType::EUCM(_) => (),
            _ => panic!("Expected EUCM model"),
        }
    }

    #[test]
    fn test_frame_context() {
        let mut ctx = FrameContext::new(true);
        assert!(ctx.step_mode);
        assert!(!ctx.auto_play);
        assert_eq!(ctx.current_idx, 0);
        assert_eq!(ctx.processed_frames, 0);

        ctx.current_idx = 1;
        ctx.processed_frames = 1;
        assert_eq!(ctx.current_idx, 1);
    }

    #[test]
    fn test_imu_data_structure() {
        let imu = ImuData {
            timestamp: 1000000000,
            gyro: [0.1, 0.2, 0.3],
            accel: [1.0, 2.0, 3.0],
        };
        assert_eq!(imu.timestamp, 1000000000);
        assert_eq!(imu.gyro, [0.1, 0.2, 0.3]);
    }
}
