//! Dataset loading and playback module.

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
use nalgebra034; // NOTE: Required by camera-intrinsic-model crate (nalgebra 0.34 API boundary).

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

impl ImuData {
    /// Gyroscope reading as a `nalgebra::Vector3`.
    pub fn gyro_vec3(&self) -> nalgebra::Vector3<f64> {
        nalgebra::Vector3::from(self.gyro)
    }

    /// Accelerometer reading as a `nalgebra::Vector3`.
    pub fn accel_vec3(&self) -> nalgebra::Vector3<f64> {
        nalgebra::Vector3::from(self.accel)
    }
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

/// Create a single camera model from parameters and model type string.
fn create_camera_model(
    model_str: &str,
    intrinsics: &[f64],
    distortion: &[f64],
    width: u32,
    height: u32,
) -> CameraModelType {
    if model_str.eq_ignore_ascii_case("eucm") {
        let params = nalgebra034::DVector::from_vec(vec![
            intrinsics[0], intrinsics[1], intrinsics[2], intrinsics[3],
            distortion[0], distortion[1],
        ]);
        CameraModelType::EUCM(EUCM::new(
            &params,
            width,
            height,
        ))
    } else {
        let params = nalgebra034::DVector::from_vec(vec![
            intrinsics[0], intrinsics[1], intrinsics[2], intrinsics[3],
            distortion.first().copied().unwrap_or(0.0),
            distortion.get(1).copied().unwrap_or(0.0),
            distortion.get(2).copied().unwrap_or(0.0),
            distortion.get(3).copied().unwrap_or(0.0),
            distortion.get(4).copied().unwrap_or(0.0),
        ]);
        CameraModelType::OpenCV5(OpenCVModel5::new(
            &params,
            width,
            height,
        ))
    }
}

/// Create left and right camera models from config.
pub fn create_camera_models_from_config(config: &Config) -> (CameraModelType, CameraModelType) {
    let cam = &config.camera;
    let left = create_camera_model(
        cam.left_model.as_deref().unwrap_or("pinhole-radtan"),
        &cam.left_intrinsics,
        &cam.left_distortion,
        cam.image_width,
        cam.image_height,
    );
    let right = create_camera_model(
        cam.right_model.as_deref().unwrap_or("pinhole-radtan"),
        &cam.right_intrinsics,
        &cam.right_distortion,
        cam.image_width,
        cam.image_height,
    );
    (left, right)
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
