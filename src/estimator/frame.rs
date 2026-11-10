use crate::datasets::ImuData;
use crate::estimator::state::State;
use crate::feature_tracker::{Feature, OrbDescriptor};
use crate::types::Matrix4x4;

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

    /// ORB descriptors for left camera features (for loop closure).
    /// Empty until explicitly computed.
    pub left_descriptors: Vec<OrbDescriptor>,

    /// ORB descriptors for right camera features (for loop closure).
    /// Empty until explicitly computed.
    pub right_descriptors: Vec<OrbDescriptor>,
}

impl Frame {
    /// Construct an empty stereo frame with default identity state.
    #[must_use]
    pub fn new(timestamp_ns: i64, frame_id: i32) -> Self {
        Self {
            timestamp_ns,
            frame_id,
            frame_type: FrameType::Stereo,
            state: State::identity(),
            imu_from_last_frame: Vec::new(),
            is_keyframe: false,
            left_features: Vec::new(),
            right_features: Vec::new(),
            left_descriptors: Vec::new(),
            right_descriptors: Vec::new(),
        }
    }

    /// Construct a stereo frame with camera extrinsics.
    /// `feature_capacity` pre-sizes the feature Vecs to avoid reallocations
    /// as features are pushed during tracking.
    pub fn from_stereo_images(
        timestamp_ns: i64,
        frame_id: i32,
        T_B_Cl: Matrix4x4,
        T_B_Cr: Matrix4x4,
        feature_capacity: usize,
    ) -> Self {
        Self {
            timestamp_ns,
            frame_id,
            frame_type: FrameType::Stereo,
            state: State::new(T_B_Cl, T_B_Cr),
            imu_from_last_frame: Vec::new(),
            is_keyframe: false,
            left_features: Vec::with_capacity(feature_capacity),
            right_features: Vec::with_capacity(feature_capacity),
            left_descriptors: Vec::with_capacity(feature_capacity),
            right_descriptors: Vec::with_capacity(feature_capacity),
        }
    }

    /// Immutable access to left-image features.
    pub fn left_features(&self) -> &Vec<Feature> {
        &self.left_features
    }

    /// Append a new feature to the left image (undistorted_coord set by caller).
    pub fn add_left_feature(&mut self, feature: Feature) {
        self.left_features.push(feature);
    }

    /// Immutable access to right-image features.
    pub fn right_features(&self) -> &Vec<Feature> {
        &self.right_features
    }

    /// Append a new feature to the right image (undistorted_coord set by caller).
    pub fn add_right_feature(&mut self, feature: Feature) {
        self.right_features.push(feature);
    }
}
