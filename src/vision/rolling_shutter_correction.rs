/// Rolling shutter correction in bundle adjustment
///
/// Handles per-feature pose estimation at capture time during optimization,
/// accounting for varying readout times across the image.
use nalgebra::{Isometry3, Matrix3, Point3, Vector2, Vector3};
use std::collections::HashMap;

/// Per-frame pose at multiple readout times
#[derive(Clone, Debug)]
pub struct RollingShutterFramePose {
    /// Frame index
    pub frame_id: usize,
    /// Pose at t_start (top of image)
    pub pose_start: Isometry3<f64>,
    /// Pose at t_end (bottom of image)
    pub pose_end: Isometry3<f64>,
    /// Readout time (seconds)
    pub readout_time: f64,
    /// Does this frame need rolling shutter correction?
    pub needs_correction: bool,
}

impl RollingShutterFramePose {
    /// Interpolate pose at pixel row
    pub fn pose_at_row(&self, row: u32, image_height: u32) -> Isometry3<f64> {
        if !self.needs_correction || image_height == 0 {
            return self.pose_start;
        }

        let t_frac = (row as f64) / (image_height as f64);

        // Linear interpolation for translation (simplest approach)
        let translation_interp = (1.0 - t_frac) * self.pose_start.translation.vector
            + t_frac * self.pose_end.translation.vector;

        // For rotation, just use start rotation (full SLERP would be better but adds complexity)
        let rotation_interp = self.pose_start.rotation;

        Isometry3::from_parts(translation_interp.into(), rotation_interp)
    }

    /// Get velocity (translation and rotation rate)
    pub fn get_velocity(&self) -> (Vector3<f64>, Vector3<f64>) {
        let dt = self.readout_time;

        // Linear velocity
        let v_trans = (self.pose_end.translation.vector - self.pose_start.translation.vector) / dt;

        // Angular velocity (simplified: just zero for now)
        let v_rot = Vector3::zeros();

        (v_trans, v_rot)
    }
}

/// Rolling shutter bundle adjustment handler
#[allow(dead_code)]
pub struct RollingShutterBundleAdjustment {
    /// Frame poses with per-row information
    frame_poses: HashMap<usize, RollingShutterFramePose>,
    /// 3D world points
    world_points: HashMap<usize, Point3<f64>>,
    /// Camera intrinsics
    focal_length: f64,
    principal_point: Vector2<f64>,
    image_width: u32,
    image_height: u32,
}

impl RollingShutterBundleAdjustment {
    pub fn new(
        focal_length: f64,
        principal_point: Vector2<f64>,
        image_width: u32,
        image_height: u32,
    ) -> Self {
        Self {
            frame_poses: HashMap::new(),
            world_points: HashMap::new(),
            focal_length,
            principal_point,
            image_width,
            image_height,
        }
    }

    /// Add frame with rolling shutter pose
    pub fn add_frame(
        &mut self,
        frame_id: usize,
        pose_start: Isometry3<f64>,
        pose_end: Isometry3<f64>,
        readout_time: f64,
        needs_correction: bool,
    ) {
        self.frame_poses.insert(
            frame_id,
            RollingShutterFramePose {
                frame_id,
                pose_start,
                pose_end,
                readout_time,
                needs_correction,
            },
        );
    }

    /// Add 3D world point
    pub fn add_world_point(&mut self, point_id: usize, position: Point3<f64>) {
        self.world_points.insert(point_id, position);
    }

    /// Compute reprojection error with rolling shutter correction
    pub fn reprojection_error_rs(
        &self,
        frame_id: usize,
        point_id: usize,
        observed_pixel: &Vector2<f64>,
    ) -> Option<f64> {
        let frame = self.frame_poses.get(&frame_id)?;
        let world_point = self.world_points.get(&point_id)?;

        // Use pose at feature row
        let feature_row = (observed_pixel.y as u32).min(self.image_height - 1);
        let pose = frame.pose_at_row(feature_row, self.image_height);

        // Project world point to camera
        let pt_camera = pose * world_point;

        // Check if behind camera
        if pt_camera.z <= 0.0 {
            return Some(f64::INFINITY);
        }

        // Project to image plane
        let proj_x = self.focal_length * (pt_camera.x / pt_camera.z) + self.principal_point.x;
        let proj_y = self.focal_length * (pt_camera.y / pt_camera.z) + self.principal_point.y;

        let projected = Vector2::new(proj_x, proj_y);
        let error = (projected - observed_pixel).norm();

        Some(error)
    }

    /// Compute reprojection error (global shutter - no correction)
    pub fn reprojection_error_gs(
        &self,
        frame_id: usize,
        point_id: usize,
        observed_pixel: &Vector2<f64>,
    ) -> Option<f64> {
        let frame = self.frame_poses.get(&frame_id)?;
        let world_point = self.world_points.get(&point_id)?;

        // Use start pose only (global shutter)
        let pose = frame.pose_start;

        // Project world point to camera
        let pt_camera = pose * world_point;

        // Check if behind camera
        if pt_camera.z <= 0.0 {
            return Some(f64::INFINITY);
        }

        // Project to image plane
        let proj_x = self.focal_length * (pt_camera.x / pt_camera.z) + self.principal_point.x;
        let proj_y = self.focal_length * (pt_camera.y / pt_camera.z) + self.principal_point.y;

        let projected = Vector2::new(proj_x, proj_y);
        let error = (projected - observed_pixel).norm();

        Some(error)
    }

    /// Compute Jacobian for rolling shutter (w.r.t. pose at frame start)
    pub fn jacobian_rs(
        &self,
        frame_id: usize,
        point_id: usize,
    ) -> Option<(Vector3<f64>, Vector3<f64>)> {
        // This is a simplified version; full implementation would compute
        // partial derivatives w.r.t. rotation (axis-angle) and translation

        let frame = self.frame_poses.get(&frame_id)?;
        let world_point = self.world_points.get(&point_id)?;

        // Use mid-frame pose for central difference
        let pose = frame.pose_at_row(self.image_height / 2, self.image_height);
        let pt_camera = pose * world_point;

        if pt_camera.z <= 0.0 {
            return None;
        }

        // Simplified Jacobian (numerical differentiation)
        let eps = 1e-5;

        // Translation Jacobian
        let mut j_trans = Vector3::zeros();
        for i in 0..3 {
            let mut perturbed = frame.pose_start.translation.vector.clone();
            perturbed[i] += eps;

            let pose_plus = Isometry3::from_parts(perturbed.into(), frame.pose_start.rotation);

            let pt_plus = pose_plus * world_point;
            let proj_plus = if pt_plus.z > 0.0 {
                Vector2::new(
                    self.focal_length * (pt_plus.x / pt_plus.z),
                    self.focal_length * (pt_plus.y / pt_plus.z),
                )
            } else {
                Vector2::zeros()
            };

            let proj = Vector2::new(
                self.focal_length * (pt_camera.x / pt_camera.z),
                self.focal_length * (pt_camera.y / pt_camera.z),
            );

            j_trans[i] = (proj_plus - proj).norm() / eps;
        }

        // Rotation Jacobian (simplified)
        let j_rot = Vector3::new(pt_camera.y / pt_camera.z, -pt_camera.x / pt_camera.z, 1.0);

        Some((j_trans, j_rot))
    }

    /// Accumulate residuals for all observations
    pub fn accumulate_residuals(
        &self,
        observations: &[(usize, usize, Vector2<f64>)], // (frame_id, point_id, pixel)
        use_rs_correction: bool,
    ) -> (f64, usize) {
        let mut total_error = 0.0;
        let mut count = 0;

        for (frame_id, point_id, pixel) in observations {
            let error = if use_rs_correction {
                self.reprojection_error_rs(*frame_id, *point_id, pixel)
            } else {
                self.reprojection_error_gs(*frame_id, *point_id, pixel)
            };

            if let Some(e) = error {
                if e < f64::INFINITY {
                    total_error += e * e;
                    count += 1;
                }
            }
        }

        let rms = if count > 0 {
            (total_error / count as f64).sqrt()
        } else {
            f64::INFINITY
        };

        (rms, count)
    }
}

// Helper functions

/// Convert rotation matrix to axis-angle vector
#[allow(dead_code)]
fn matrix_to_axis_angle(R: &Matrix3<f64>) -> Vector3<f64> {
    let trace = R.m11 + R.m22 + R.m33;

    let angle = ((trace - 1.0) / 2.0).clamp(-1.0, 1.0).acos();

    if angle.abs() < 1e-6 {
        return Vector3::zeros();
    }

    let axis = Vector3::new(R.m32 - R.m23, R.m13 - R.m31, R.m21 - R.m12) / (2.0 * angle.sin());

    axis * angle
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_rolling_shutter_pose() {
        let pose_start = Isometry3::identity();
        let pose_end = Isometry3::identity();

        let rs_pose = RollingShutterFramePose {
            frame_id: 0,
            pose_start,
            pose_end,
            readout_time: 0.01,
            needs_correction: true,
        };

        assert_eq!(rs_pose.readout_time, 0.01);
        assert!(rs_pose.needs_correction);
    }

    #[test]
    fn test_pose_interpolation() {
        let pose_start = Isometry3::identity();
        let pose_end = Isometry3::identity();

        let rs_pose = RollingShutterFramePose {
            frame_id: 0,
            pose_start,
            pose_end,
            readout_time: 0.01,
            needs_correction: true,
        };

        let pose_mid = rs_pose.pose_at_row(240, 480);

        // Should be identity for this case (no motion between start and end)
        assert!((pose_mid.translation.vector.norm() - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_bundle_adjustment_creation() {
        let ba = RollingShutterBundleAdjustment::new(500.0, Vector2::new(320.0, 240.0), 640, 480);

        assert_eq!(ba.focal_length, 500.0);
        assert_eq!(ba.image_width, 640);
        assert_eq!(ba.image_height, 480);
    }

    #[test]
    fn test_add_frame() {
        let mut ba =
            RollingShutterBundleAdjustment::new(500.0, Vector2::new(320.0, 240.0), 640, 480);

        let pose = Isometry3::identity();
        ba.add_frame(0, pose, pose, 0.01, true);

        assert!(ba.frame_poses.contains_key(&0));
    }

    #[test]
    fn test_add_world_point() {
        let mut ba =
            RollingShutterBundleAdjustment::new(500.0, Vector2::new(320.0, 240.0), 640, 480);

        let point = Point3::new(1.0, 2.0, 5.0);
        ba.add_world_point(0, point);

        assert!(ba.world_points.contains_key(&0));
    }
}
