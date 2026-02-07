use crate::datasets::CameraModelType;
use crate::datasets::ImuData;
use crate::estimator::state::State;
use crate::feature_tracker::Feature;
use crate::types::Matrix4x4;
use camera_intrinsic_model::models::opencv5::OpenCVModel5;
use nalgebra034;

/// Type of frame (only Stereo used for now; RGBD omitted).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrameType {
    Stereo,
}

/// Simplified stereo frame representation for VIO.
#[derive(Debug, Clone)]
pub struct Frame {
    pub timestamp_ns: i64,
    pub frame_id: i32,

    pub frame_type: FrameType,

    /// Left and right camera models (supports OpenCVModel5 and EUCM).
    pub left_cam: CameraModelType,
    pub right_cam: CameraModelType,

    /// Per-frame state (T_w_b, velocity, IMU biases, etc).
    pub state: State,

    /// IMU samples since last frame.
    pub imu_from_last_frame: Vec<ImuData>,

    /// Whether this frame is a keyframe.
    pub is_keyframe: bool,

    /// Per-frame 2D features (left image).
    pub left_features: Vec<Feature>,

    /// Per-frame 2D features (right image).
    pub right_features: Vec<Feature>,
}

impl Frame {
    /// Construct an empty stereo frame with default intrinsics and identity state.
    pub fn new(timestamp_ns: i64, frame_id: i32) -> Self {
        Self {
            timestamp_ns,
            frame_id,
            frame_type: FrameType::Stereo,
            // Reasonable but arbitrary defaults; real values should come from config.
            // Use nalgebra 0.34.1 (which camera-intrinsic-model uses)
            left_cam: CameraModelType::OpenCV5(OpenCVModel5::new(
                &nalgebra034::DVector::from_column_slice(&[
                    500.0, 500.0, 320.0, 240.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                ]),
                0,
                0,
            )),
            right_cam: CameraModelType::OpenCV5(OpenCVModel5::new(
                &nalgebra034::DVector::from_column_slice(&[
                    500.0, 500.0, 320.0, 240.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                ]),
                0,
                0,
            )),
            state: State::identity(),
            imu_from_last_frame: Vec::new(),
            is_keyframe: false,
            left_features: Vec::new(),
            right_features: Vec::new(),
        }
    }

    /// Construct a stereo frame from camera models.
    pub fn from_stereo_images(
        timestamp_ns: i64,
        frame_id: i32,
        left_cam: CameraModelType,
        right_cam: CameraModelType,
        T_B_Cl: Matrix4x4,
        T_B_Cr: Matrix4x4,
    ) -> Self {
        Self {
            timestamp_ns,
            frame_id,
            frame_type: FrameType::Stereo,
            left_cam,
            right_cam,
            state: State::new(T_B_Cl, T_B_Cr),
            imu_from_last_frame: Vec::new(),
            is_keyframe: true,
            left_features: Vec::new(),
            right_features: Vec::new(),
        }
    }

    /// Immutable access to left-image features.
    pub fn left_features(&self) -> &[Feature] {
        &self.left_features
    }

    /// Append a new feature to the left image.
    pub fn add_left_feature(&mut self, mut feature: Feature) {
        let undist_coord =
            self.left_cam
                .as_camera_model()
                .unproject_one(&nalgebra034::Vector2::new(
                    feature.pixel_coord[0] as f64,
                    feature.pixel_coord[1] as f64,
                ));
        feature.undistorted_coord = [undist_coord[0] as f32, undist_coord[1] as f32];
        self.left_features.push(feature);
    }

    /// Immutable access to right-image features.
    pub fn right_features(&self) -> &[Feature] {
        &self.right_features
    }

    /// Append a new feature to the right image.
    pub fn add_right_feature(&mut self, mut feature: Feature) {
        // Use nalgebra034::Vector2 since OpenCVModel5 uses nalgebra 0.34.1
        let undist_coord =
            self.right_cam
                .as_camera_model()
                .unproject_one(&nalgebra034::Vector2::new(
                    feature.pixel_coord[0] as f64,
                    feature.pixel_coord[1] as f64,
                ));
        feature.undistorted_coord = [undist_coord[0] as f32, undist_coord[1] as f32];
        self.right_features.push(feature);
    }
}
