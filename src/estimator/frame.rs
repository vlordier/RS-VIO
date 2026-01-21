use crate::datasets::{CameraModelType, ImuData};
use crate::estimator::state::State;
use crate::feature_tracker::Feature;
use crate::types::Matrix4x4;
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

    /// IMU samples since last keyframe.
    pub imu_since_last_keyframe: Vec<ImuData>,

    /// Whether this frame is a keyframe.
    pub is_keyframe: bool,

    /// Per-frame 2D features (left image).
    pub left_features: Vec<Feature>,

    /// Per-frame 2D features (right image).
    pub right_features: Vec<Feature>,

    /// Optional left image buffer for fusion/quality metrics.
    pub left_image_plane: Option<ImagePlane>,
}

/// Lightweight owned image plane (grayscale) with dimensions.
#[derive(Debug, Clone)]
pub struct ImagePlane {
    pub data: Vec<u8>,
    pub width: u32,
    pub height: u32,
}

/// Builder for type-safe Frame construction.
///
/// This builder ensures all required fields are set before construction
/// and provides validation during the build process.
///
/// # Example
/// ```rust,ignore
/// let frame = FrameBuilder::new(timestamp_ns, frame_id)
///     .with_left_cam(left_cam)
///     .with_right_cam(right_cam)
///     .with_state(state)
///     .is_keyframe(true)
///     .build();
/// ```
#[derive(Debug, Default)]
pub struct FrameBuilder {
    timestamp_ns: Option<i64>,
    frame_id: Option<i32>,
    left_cam: Option<CameraModelType>,
    right_cam: Option<CameraModelType>,
    state: Option<State>,
    is_keyframe: bool,
    imu_from_last_frame: Vec<ImuData>,
    imu_since_last_keyframe: Vec<ImuData>,
}

impl FrameBuilder {
    /// Create a new builder with required timestamp and frame_id
    pub fn new(timestamp_ns: i64, frame_id: i32) -> Self {
        Self {
            timestamp_ns: Some(timestamp_ns),
            frame_id: Some(frame_id),
            left_cam: None,
            right_cam: None,
            state: None,
            is_keyframe: false,
            imu_from_last_frame: Vec::new(),
            imu_since_last_keyframe: Vec::new(),
        }
    }

    /// Set the left camera model
    pub fn with_left_cam(mut self, cam: CameraModelType) -> Self {
        self.left_cam = Some(cam);
        self
    }

    /// Set the right camera model
    pub fn with_right_cam(mut self, cam: CameraModelType) -> Self {
        self.right_cam = Some(cam);
        self
    }

    /// Set the state
    pub fn with_state(mut self, state: State) -> Self {
        self.state = Some(state);
        self
    }

    /// Mark as keyframe (default: false)
    pub fn is_keyframe(mut self, keyframe: bool) -> Self {
        self.is_keyframe = keyframe;
        self
    }

    /// Add IMU data from last frame
    pub fn add_imu_from_last_frame(mut self, imu: ImuData) -> Self {
        self.imu_from_last_frame.push(imu);
        self
    }

    /// Add IMU data since last keyframe
    pub fn add_imu_since_last_keyframe(mut self, imu: ImuData) -> Self {
        self.imu_since_last_keyframe.push(imu);
        self
    }

    /// Build the frame, validating all required fields
    pub fn build(self) -> Result<Frame, String> {
        let timestamp_ns = self.timestamp_ns.ok_or("timestamp_ns is required")?;
        let frame_id = self.frame_id.ok_or("frame_id is required")?;
        let left_cam = self.left_cam.unwrap_or_else(|| {
            crate::types::CameraFactory::opencv5(
                500.0, 500.0, 320.0, 240.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0,
            )
        });
        let right_cam = self.right_cam.unwrap_or_else(|| {
            crate::types::CameraFactory::opencv5(
                500.0, 500.0, 320.0, 240.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0,
            )
        });
        let state = self.state.unwrap_or_else(State::identity);

        Ok(Frame {
            timestamp_ns,
            frame_id,
            frame_type: FrameType::Stereo,
            left_cam,
            right_cam,
            state,
            imu_from_last_frame: self.imu_from_last_frame,
            imu_since_last_keyframe: self.imu_since_last_keyframe,
            is_keyframe: self.is_keyframe,
            left_features: Vec::new(),
            right_features: Vec::new(),
            left_image_plane: None,
        })
    }
}

impl Frame {
    /// Construct an empty stereo frame with default intrinsics and identity state.
    pub fn new(timestamp_ns: i64, frame_id: i32) -> Self {
        Self {
            timestamp_ns,
            frame_id,
            frame_type: FrameType::Stereo,
            // Reasonable but arbitrary defaults; real values should come from config.
            // Using CameraFactory for consistent camera creation
            left_cam: crate::types::CameraFactory::opencv5(
                500.0, 500.0, 320.0, 240.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0,
            ),
            right_cam: crate::types::CameraFactory::opencv5(
                500.0, 500.0, 320.0, 240.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0,
            ),
            state: State::identity(),
            imu_from_last_frame: Vec::new(),
            imu_since_last_keyframe: Vec::new(),
            is_keyframe: false,
            left_features: Vec::new(),
            right_features: Vec::new(),
            left_image_plane: None,
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
            imu_since_last_keyframe: Vec::new(),
            is_keyframe: true,
            left_features: Vec::new(),
            right_features: Vec::new(),
            left_image_plane: None,
        }
    }

    /// Immutable access to left-image features.
    pub fn left_features(&self) -> &Vec<Feature> {
        &self.left_features
    }

    /// Optional borrowed view of the left image plane.
    pub fn left_image_plane(&self) -> Option<(&[u8], u32, u32)> {
        self.left_image_plane
            .as_ref()
            .map(|plane| (plane.data.as_slice(), plane.width, plane.height))
    }

    /// Set/own the left image plane (cloned or transferred by caller).
    pub fn set_left_image_plane(&mut self, data: Vec<u8>, width: u32, height: u32) {
        self.left_image_plane = Some(ImagePlane { data, width, height });
    }

    /// Append a new feature to the left image.
    pub fn add_left_feature(&mut self, mut feature: Feature) {
        // Use nalgebra034::Vector2 since OpenCVModel5 uses nalgebra 0.34.1

        // Center radius around the center of the image (256, 256)
        /*
        let x = feature.pixel_coord[0] as f64 - 256.0;
        let y = feature.pixel_coord[1] as f64 - 256.0;
        let radius = (x * x + y * y).sqrt();
        if radius > 400.0 {
            return;
        } */
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
    pub fn right_features(&self) -> &Vec<Feature> {
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
