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

    /// Comprehensive robustness dashboard
    fn log_robustness_dashboard(
        &mut self,
        _prosac_inliers: usize,
        _prosac_outliers: usize,
        _vibration_level: f32,
        _covariance_scale: f32,
        _feature_count: usize,
        _high_quality_features: usize,
        _loop_closures: usize,
        _entity_path: &str,
    ) {
        // Default implementation - no-op for viewers that don't support robustness visualization
    }

    /// Visualize vibration metrics and adaptive covariance
    fn log_vibration_metrics(
        &mut self,
        _gyro_rms: f32,
        _accel_rms: f32,
        _covariance_scale: f32,
        _entity_path: &str,
    ) {
        // Default implementation - no-op
    }

    /// Visualize feature quality metrics
    fn log_feature_quality(
        &mut self,
        _features: &[crate::feature_tracker::Feature],
        _entity_path: &str,
    ) {
        // Default implementation - no-op
    }

    /// Visualize loop closure constraints
    fn log_loop_closure(
        &mut self,
        _constraints: &[crate::optimization::loop_closure::LoopClosureConstraint],
        _entity_path: &str,
    ) {
        // Default implementation - no-op
    }

    /// Visualize PROSAC/MAGSAC++ geometric verification results
    fn log_robustness_verification(
        &mut self,
        _inliers: &[(f32, f32)],
        _outliers: &[(f32, f32)],
        _entity_path: &str,
    ) {
        // Default implementation - no-op
    }
}
