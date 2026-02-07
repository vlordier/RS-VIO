pub mod config;
pub mod euroc_player;
pub mod fourseasons_player;
pub mod io;
pub mod player;
pub mod tum_vi_player;

// Re-export player types for convenience
pub use euroc_player::EurocPlayer;
pub use fourseasons_player::FourSeasonsPlayer;
pub use player::DatasetPlayer;
pub use tum_vi_player::TUMVIPlayer;

use crate::datasets::config::Config;
use camera_intrinsic_model::generic_model::CameraModel;
use camera_intrinsic_model::models::opencv5::OpenCVModel5;
use camera_intrinsic_model::models::EUCM;
use nalgebra034; // TODO find a way to avoid this dependency (currently used for camera models)

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
#[derive(Debug)]
pub struct FrameContext {
    pub current_idx: usize,
    pub processed_frames: usize,
    pub previous_frame_timestamp: i64,
    pub step_mode: bool,
    pub auto_play: bool,
    pub advance_frame: bool,
}

impl FrameContext {
    pub const fn new(step_mode: bool) -> Self {
        FrameContext {
            current_idx: 0,
            processed_frames: 0,
            previous_frame_timestamp: 0,
            step_mode,
            auto_play: !step_mode,
            advance_frame: false,
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

/// Create camera models from config
/// This helper function creates camera models from the configuration
/// for both left and right cameras. Supports OpenCVModel5 and EUCM models.
///
/// # Errors
/// Returns an error if:
/// - Intrinsics vectors don't have exactly 4 elements [fx, fy, cx, cy]
/// - EUCM model is specified but distortion doesn't have at least 2 elements [alpha, beta]
pub fn create_camera_models_from_config(
    config: &Config,
) -> anyhow::Result<(CameraModelType, CameraModelType)> {
    let cam = &config.camera;

    // Validate intrinsics arrays
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

    // Determine left camera model type
    let left_model_str = cam.left_model.as_deref().unwrap_or("pinhole-radtan");
    let left_cam = if left_model_str.eq_ignore_ascii_case("eucm") {
        // EUCM model requires at least 2 distortion params: [alpha, beta]
        if cam.left_distortion.len() < 2 {
            anyhow::bail!(
                "left camera uses EUCM but distortion has {} params (need >= 2 for alpha, beta)",
                cam.left_distortion.len()
            );
        }
        // EUCM model: [fx, fy, cx, cy, alpha, beta]
        let eucm_params_vec: Vec<f64> = vec![
            cam.left_intrinsics[0], // fx
            cam.left_intrinsics[1], // fy
            cam.left_intrinsics[2], // cx
            cam.left_intrinsics[3], // cy
            cam.left_distortion[0], // alpha
            cam.left_distortion[1], // beta
        ];
        let eucm_params = nalgebra034::DVector::from_vec(eucm_params_vec);
        CameraModelType::EUCM(EUCM::new(&eucm_params, cam.image_width, cam.image_height))
    } else {
        // OpenCVModel5: [fx, fy, cx, cy, k1, k2, p1, p2, k3]
        let left_params_vec: Vec<f64> = vec![
            cam.left_intrinsics[0],                              // fx
            cam.left_intrinsics[1],                              // fy
            cam.left_intrinsics[2],                              // cx
            cam.left_intrinsics[3],                              // cy
            cam.left_distortion.first().copied().unwrap_or(0.0), // k1
            cam.left_distortion.get(1).copied().unwrap_or(0.0),  // k2
            cam.left_distortion.get(2).copied().unwrap_or(0.0),  // p1
            cam.left_distortion.get(3).copied().unwrap_or(0.0),  // p2
            cam.left_distortion.get(4).copied().unwrap_or(0.0),  // k3
        ];
        let left_params = nalgebra034::DVector::from_vec(left_params_vec);
        CameraModelType::OpenCV5(OpenCVModel5::new(
            &left_params,
            cam.image_width,
            cam.image_height,
        ))
    };

    // Determine right camera model type
    let right_model_str = cam.right_model.as_deref().unwrap_or("pinhole-radtan");
    let right_cam = if right_model_str.eq_ignore_ascii_case("eucm") {
        // EUCM model requires at least 2 distortion params: [alpha, beta]
        if cam.right_distortion.len() < 2 {
            anyhow::bail!(
                "right camera uses EUCM but distortion has {} params (need >= 2 for alpha, beta)",
                cam.right_distortion.len()
            );
        }
        // EUCM model: [fx, fy, cx, cy, alpha, beta]
        let eucm_params_vec: Vec<f64> = vec![
            cam.right_intrinsics[0], // fx
            cam.right_intrinsics[1], // fy
            cam.right_intrinsics[2], // cx
            cam.right_intrinsics[3], // cy
            cam.right_distortion[0], // alpha
            cam.right_distortion[1], // beta
        ];
        let eucm_params = nalgebra034::DVector::from_vec(eucm_params_vec);
        CameraModelType::EUCM(EUCM::new(&eucm_params, cam.image_width, cam.image_height))
    } else {
        // OpenCVModel5: [fx, fy, cx, cy, k1, k2, p1, p2, k3]
        let right_params_vec: Vec<f64> = vec![
            cam.right_intrinsics[0],                              // fx
            cam.right_intrinsics[1],                              // fy
            cam.right_intrinsics[2],                              // cx
            cam.right_intrinsics[3],                              // cy
            cam.right_distortion.first().copied().unwrap_or(0.0), // k1
            cam.right_distortion.get(1).copied().unwrap_or(0.0),  // k2
            cam.right_distortion.get(2).copied().unwrap_or(0.0),  // p1
            cam.right_distortion.get(3).copied().unwrap_or(0.0),  // p2
            cam.right_distortion.get(4).copied().unwrap_or(0.0),  // k3
        ];
        let right_params = nalgebra034::DVector::from_vec(right_params_vec);
        CameraModelType::OpenCV5(OpenCVModel5::new(
            &right_params,
            cam.image_width,
            cam.image_height,
        ))
    };

    Ok((left_cam, right_cam))
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used)]
pub(crate) mod test_utils {
    use super::ImuData;
    use std::fs;
    use std::io::Write;
    use std::path::Path;

    pub(crate) fn write_file(path: &Path, contents: &str) {
        if let Some(parent) = path.parent() {
            fs::create_dir_all(parent).expect("create parent dirs");
        }
        let mut file = fs::File::create(path).expect("create file");
        file.write_all(contents.as_bytes()).expect("write file");
    }

    pub(crate) fn sample_imu_data() -> Vec<ImuData> {
        (1..=5)
            .map(|timestamp| ImuData {
                timestamp,
                gyro: [0.0; 3],
                accel: [0.0; 3],
            })
            .collect()
    }

    pub(crate) fn timestamps(data: &[ImuData]) -> Vec<i64> {
        data.iter().map(|imu| imu.timestamp).collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::datasets::config::{
        CameraConfig, Config, FeatureDetectionConfig, KeyframeManagementConfig,
        OptimizationConfig,
    };

    fn create_test_config(
        left_intrinsics: Vec<f64>,
        left_distortion: Vec<f64>,
        right_intrinsics: Vec<f64>,
        right_distortion: Vec<f64>,
        left_model: Option<String>,
        right_model: Option<String>,
    ) -> Config {
        Config {
            camera: CameraConfig {
                image_width: 640,
                image_height: 480,
                left_intrinsics,
                left_distortion,
                right_intrinsics,
                right_distortion,
                left_model,
                right_model,
                T_B_Cl: vec![
                    1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                    1.0,
                ],
                T_B_Cr: vec![
                    1.0, 0.0, 0.0, -0.12, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                    1.0,
                ],
            },
            keyframe_management: KeyframeManagementConfig {
                keyframe_window_size: 10,
                translation_threshold: 0.1,
                rotation_threshold: 0.05,
            },
            feature_detection: FeatureDetectionConfig {
                grid_cols: 8,
                max_features_per_grid: 10,
                optical_flow_max_iterations: 30,
                optical_flow_convergence_threshold: 0.01,
            },
            optimization: OptimizationConfig {
                bundle_adjustment_max_iterations: 10,
                pnp_max_iterations: 50,
            },
            calibration: None,
        }
    }

    #[test]
    fn test_create_camera_models_valid_opencv() {
        let config = create_test_config(
            vec![500.0, 500.0, 320.0, 240.0],
            vec![0.0, 0.0, 0.0, 0.0, 0.0],
            vec![500.0, 500.0, 320.0, 240.0],
            vec![0.0, 0.0, 0.0, 0.0, 0.0],
            Some("pinhole-radtan".to_string()),
            Some("pinhole-radtan".to_string()),
        );

        let result = create_camera_models_from_config(&config);
        assert!(result.is_ok());
    }

    #[test]
    fn test_create_camera_models_valid_eucm() {
        let config = create_test_config(
            vec![500.0, 500.0, 320.0, 240.0],
            vec![0.5, 1.0],
            vec![500.0, 500.0, 320.0, 240.0],
            vec![0.5, 1.0],
            Some("EUCM".to_string()),
            Some("EUCM".to_string()),
        );

        let result = create_camera_models_from_config(&config);
        assert!(result.is_ok());
    }

    #[test]
    fn test_create_camera_models_invalid_left_intrinsics() {
        let config = create_test_config(
            vec![500.0, 500.0], // Only 2 elements instead of 4
            vec![0.0, 0.0],
            vec![500.0, 500.0, 320.0, 240.0],
            vec![0.0, 0.0],
            Some("EUCM".to_string()),
            Some("EUCM".to_string()),
        );

        let result = create_camera_models_from_config(&config);
        assert!(result.is_err());
        let err_msg = result.unwrap_err().to_string();
        assert!(err_msg.contains("left_intrinsics must have exactly 4 elements"));
    }

    #[test]
    fn test_create_camera_models_invalid_right_intrinsics() {
        let config = create_test_config(
            vec![500.0, 500.0, 320.0, 240.0],
            vec![0.0, 0.0],
            vec![500.0, 500.0, 320.0], // Only 3 elements instead of 4
            vec![0.0, 0.0],
            Some("EUCM".to_string()),
            Some("EUCM".to_string()),
        );

        let result = create_camera_models_from_config(&config);
        assert!(result.is_err());
        let err_msg = result.unwrap_err().to_string();
        assert!(err_msg.contains("right_intrinsics must have exactly 4 elements"));
    }

    #[test]
    fn test_create_camera_models_eucm_insufficient_distortion_left() {
        let config = create_test_config(
            vec![500.0, 500.0, 320.0, 240.0],
            vec![0.5], // Only 1 element, EUCM needs at least 2
            vec![500.0, 500.0, 320.0, 240.0],
            vec![0.5, 1.0],
            Some("EUCM".to_string()),
            Some("EUCM".to_string()),
        );

        let result = create_camera_models_from_config(&config);
        assert!(result.is_err());
        let err_msg = result.unwrap_err().to_string();
        assert!(err_msg.contains("left camera uses EUCM"));
        assert!(err_msg.contains("need >= 2 for alpha, beta"));
    }

    #[test]
    fn test_create_camera_models_eucm_insufficient_distortion_right() {
        let config = create_test_config(
            vec![500.0, 500.0, 320.0, 240.0],
            vec![0.5, 1.0],
            vec![500.0, 500.0, 320.0, 240.0],
            vec![0.5], // Only 1 element, EUCM needs at least 2
            Some("EUCM".to_string()),
            Some("EUCM".to_string()),
        );

        let result = create_camera_models_from_config(&config);
        assert!(result.is_err());
        let err_msg = result.unwrap_err().to_string();
        assert!(err_msg.contains("right camera uses EUCM"));
        assert!(err_msg.contains("need >= 2 for alpha, beta"));
    }

    #[test]
    fn test_create_camera_models_opencv_with_empty_distortion() {
        // OpenCV model should work with empty distortion (defaults to 0.0)
        let config = create_test_config(
            vec![500.0, 500.0, 320.0, 240.0],
            vec![],
            vec![500.0, 500.0, 320.0, 240.0],
            vec![],
            Some("pinhole-radtan".to_string()),
            Some("pinhole-radtan".to_string()),
        );

        let result = create_camera_models_from_config(&config);
        assert!(result.is_ok());
    }
}
