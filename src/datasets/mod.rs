pub mod config;
pub mod euroc_player;
pub mod fourseasons_player;
pub mod tum_vi_player;

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
pub fn create_camera_models_from_config(config: &Config) -> (CameraModelType, CameraModelType) {
    let cam = &config.camera;

    // Determine left camera model type
    // TODO make this code more generic (and elegant)
    // Using unwrap_or doesn't make sense here, if we can't get the params, we should error out
    let left_model_str = cam.left_model.as_deref().unwrap_or("pinhole-radtan");
    let left_cam = if left_model_str == "EUCM" || left_model_str == "eucm" {
        // EUCM model: [fx, fy, cx, cy, alpha, beta]
        let eucm_params_vec: Vec<f64> = vec![
            cam.left_intrinsics.first().copied().unwrap_or(500.0), // fx
            cam.left_intrinsics.get(1).copied().unwrap_or(500.0),  // fy
            cam.left_intrinsics.get(2).copied().unwrap_or(320.0),  // cx
            cam.left_intrinsics.get(3).copied().unwrap_or(240.0),  // cy
            cam.left_distortion.first().copied().unwrap_or(0.5),   // alpha
            cam.left_distortion.get(1).copied().unwrap_or(1.0),    // beta
        ];
        let eucm_params = nalgebra034::DVector::from_vec(eucm_params_vec);
        CameraModelType::EUCM(EUCM::new(&eucm_params, cam.image_width, cam.image_height))
    } else {
        // OpenCVModel5: [fx, fy, cx, cy, k1, k2, p1, p2, k3]
        let left_params_vec: Vec<f64> = vec![
            cam.left_intrinsics.first().copied().unwrap_or(500.0),
            cam.left_intrinsics.get(1).copied().unwrap_or(500.0),
            cam.left_intrinsics.get(2).copied().unwrap_or(320.0),
            cam.left_intrinsics.get(3).copied().unwrap_or(240.0),
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
    let right_cam = if right_model_str == "EUCM" || right_model_str == "eucm" {
        // EUCM model: [fx, fy, cx, cy, alpha, beta]
        let eucm_params_vec: Vec<f64> = vec![
            cam.right_intrinsics.first().copied().unwrap_or(500.0), // fx
            cam.right_intrinsics.get(1).copied().unwrap_or(500.0),  // fy
            cam.right_intrinsics.get(2).copied().unwrap_or(320.0),  // cx
            cam.right_intrinsics.get(3).copied().unwrap_or(240.0),  // cy
            cam.right_distortion.first().copied().unwrap_or(0.5),   // alpha
            cam.right_distortion.get(1).copied().unwrap_or(1.0),    // beta
        ];
        let eucm_params = nalgebra034::DVector::from_vec(eucm_params_vec);
        CameraModelType::EUCM(EUCM::new(&eucm_params, cam.image_width, cam.image_height))
    } else {
        // OpenCVModel5: [fx, fy, cx, cy, k1, k2, p1, p2, k3]
        let right_params_vec: Vec<f64> = vec![
            cam.right_intrinsics.first().copied().unwrap_or(500.0),
            cam.right_intrinsics.get(1).copied().unwrap_or(500.0),
            cam.right_intrinsics.get(2).copied().unwrap_or(320.0),
            cam.right_intrinsics.get(3).copied().unwrap_or(240.0),
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

    (left_cam, right_cam)
}

#[cfg(test)]
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
