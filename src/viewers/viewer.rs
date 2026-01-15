use crate::types::Matrix4x4;
use crate::Result;

/// Viewer trait for basic visualization operations
pub trait Viewer: Send {
    /// Initialize the viewer
    fn initialize(&mut self) -> Result<()>;

    /// Log current pose T_W_B (4x4 transformation matrix: body in world)
    fn log_pose(&mut self, T_W_B: Matrix4x4, entity_path: &str);

    /// Log raw image
    fn log_image_raw(&mut self, image: &[u8], width: u32, height: u32, entity_path: &str);

    /// Log equalized/preprocessed image
    fn log_image_equalized(&mut self, image: &[u8], width: u32, height: u32, entity_path: &str);

    /// Log image with features drawn on it
    fn log_image_with_features(
        &mut self,
        image: &[u8],
        width: u32,
        height: u32,
        features: &[[f32; 2]],
        entity_path: &str,
    );

    /// Log image with features colored by feature ID
    /// Features should be provided as (feature_id, [x, y]) tuples
    fn log_image_with_features_colored(
        &mut self,
        image: &[u8],
        width: u32,
        height: u32,
        features: &[(usize, [f32; 2])],
        entity_path: &str,
    );

    /// Log 3D points
    fn log_points(&mut self, points: &[[f32; 3]], entity_path: &str);

    /// Log 3D points colored by feature ID
    /// Points should be provided as (feature_id, [x, y, z]) tuples
    fn log_points_colored(&mut self, points: &[(usize, [f32; 3])], entity_path: &str);

    /// Set the current frame/timestamp
    fn set_frame(&mut self, frame_id: i64);

    /// Log camera frustum using Pinhole model
    /// focal_length: camera focal length (typically fx from intrinsics)
    /// width: image width in pixels
    /// height: image height in pixels
    fn log_camera_frustum(
        &mut self,
        focal_length: f32,
        width: u32,
        height: u32,
        entity_path: &str,
        size: f32,
    );

    /// Log trajectory path as a continuous 3D line
    /// trajectory: vector of 4x4 transformation matrices, extracts translation (position) from each
    fn log_trajectory(&mut self, trajectory: &[Matrix4x4], entity_path: &str);
}
