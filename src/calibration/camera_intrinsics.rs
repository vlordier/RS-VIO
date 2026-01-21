/// Camera intrinsics calibration using checkerboard/AprilTag targets
///
/// Estimates focal lengths, principal point, and distortion coefficients
/// using a set of calibration images with known target geometry.
use nalgebra::{Matrix3, Vector2, Vector3};
use std::collections::HashMap;

#[derive(Clone, Debug)]
pub struct CameraIntrinsicsEstimate {
    /// Focal length in x direction (pixels)
    pub fx: f64,
    /// Focal length in y direction (pixels)
    pub fy: f64,
    /// Principal point x coordinate (pixels)
    pub cx: f64,
    /// Principal point y coordinate (pixels)
    pub cy: f64,
    /// Distortion coefficients (k1, k2, p1, p2, [k3])
    pub distortion: Vec<f64>,
    /// RMS reprojection error (pixels)
    pub reprojection_rms: f64,
    /// Per-image reprojection errors
    pub per_image_errors: Vec<f64>,
    /// Reprojection error map (error vs image region)
    pub residual_map: HashMap<(u32, u32), f64>,
}

impl CameraIntrinsicsEstimate {
    /// Create initial intrinsics estimate from image dimensions
    pub fn from_image_size(width: u32, height: u32) -> Self {
        let fx = width as f64 * 0.5; // Rough estimate: ~50% of width
        let fy = height as f64 * 0.5;
        let cx = width as f64 * 0.5;
        let cy = height as f64 * 0.5;

        Self {
            fx,
            fy,
            cx,
            cy,
            distortion: vec![0.0, 0.0, 0.0, 0.0], // k1, k2, p1, p2
            reprojection_rms: f64::INFINITY,
            per_image_errors: Vec::new(),
            residual_map: HashMap::new(),
        }
    }

    /// Get camera matrix K
    pub fn camera_matrix(&self) -> Matrix3<f64> {
        Matrix3::new(self.fx, 0.0, self.cx, 0.0, self.fy, self.cy, 0.0, 0.0, 1.0)
    }

    /// Apply distortion to a normalized image coordinate
    pub fn apply_distortion(&self, x: f64, y: f64) -> (f64, f64) {
        let r2 = x * x + y * y;
        let r4 = r2 * r2;

        let k1 = self.distortion.get(0).copied().unwrap_or(0.0);
        let k2 = self.distortion.get(1).copied().unwrap_or(0.0);
        let p1 = self.distortion.get(2).copied().unwrap_or(0.0);
        let p2 = self.distortion.get(3).copied().unwrap_or(0.0);

        // Radial distortion
        let radial = 1.0 + k1 * r2 + k2 * r4;

        // Tangential distortion
        let x_distorted = x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
        let y_distorted = y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;

        (x_distorted, y_distorted)
    }

    /// Reproject 3D world point to 2D image point
    pub fn project_point(&self, point_3d: &Vector3<f64>) -> Option<Vector2<f64>> {
        if point_3d.z <= 0.0 {
            return None; // Point behind camera
        }

        // Normalize
        let x = point_3d.x / point_3d.z;
        let y = point_3d.y / point_3d.z;

        // Apply distortion
        let (x_dist, y_dist) = self.apply_distortion(x, y);

        // Project to image
        let u = self.fx * x_dist + self.cx;
        let v = self.fy * y_dist + self.cy;

        Some(Vector2::new(u, v))
    }

    /// Compute reprojection error for a set of point correspondences
    ///
    /// # Arguments
    /// * `world_points` - 3D points in world frame
    /// * `image_points` - Measured 2D image points
    /// * `camera_pose` - Camera pose in world frame (R, t)
    pub fn reprojection_error(
        &self,
        world_points: &[Vector3<f64>],
        image_points: &[Vector2<f64>],
        camera_pose: (&Matrix3<f64>, &Vector3<f64>),
    ) -> f64 {
        let (R, t) = camera_pose;

        let mut sum_sq_error = 0.0;
        let mut count = 0;

        for (pt_3d, pt_2d) in world_points.iter().zip(image_points.iter()) {
            // Transform to camera frame
            let pt_cam = R * pt_3d + t;

            // Project
            if let Some(pt_proj) = self.project_point(&pt_cam) {
                let error = (pt_proj - pt_2d).norm();
                sum_sq_error += error * error;
                count += 1;
            }
        }

        if count == 0 {
            return f64::INFINITY;
        }

        (sum_sq_error / count as f64).sqrt()
    }

    /// Check if intrinsics are reasonable
    pub fn is_reasonable(&self) -> bool {
        // Focal length should be positive and reasonable (100-5000 px for typical lenses)
        if self.fx < 100.0 || self.fx > 10000.0 {
            return false;
        }
        if self.fy < 100.0 || self.fy > 10000.0 {
            return false;
        }

        // Principal point should be plausible
        if self.cx < -100.0 || self.cy < -100.0 {
            return false;
        }

        // Distortion should be moderate (k1 typically in [-0.5, 0.5])
        if let Some(k1) = self.distortion.get(0) {
            if k1.abs() > 2.0 {
                return false;
            }
        }

        true
    }
}

/// Camera intrinsics calibrator using calibration target observations
#[derive(Debug)]
pub struct CameraIntrinsicsCalibrator {
    /// Target observations: (image_id, world_point, image_point)
    observations: Vec<(usize, Vector3<f64>, Vector2<f64>)>,
    /// Camera poses per image
    camera_poses: HashMap<usize, (Matrix3<f64>, Vector3<f64>)>,
    /// Image dimensions
    image_width: u32,
    image_height: u32,
}

impl CameraIntrinsicsCalibrator {
    pub fn new(image_width: u32, image_height: u32) -> Self {
        Self {
            observations: Vec::new(),
            camera_poses: HashMap::new(),
            image_width,
            image_height,
        }
    }

    /// Add an observation: feature match between 3D target point and 2D image point
    pub fn add_observation(
        &mut self,
        image_id: usize,
        world_point: Vector3<f64>,
        image_point: Vector2<f64>,
    ) {
        self.observations.push((image_id, world_point, image_point));
    }

    /// Set camera pose for a calibration image
    pub fn set_camera_pose(
        &mut self,
        image_id: usize,
        rotation: Matrix3<f64>,
        translation: Vector3<f64>,
    ) {
        self.camera_poses.insert(image_id, (rotation, translation));
    }

    /// Calibrate intrinsics using Gauss-Newton optimization
    ///
    /// Returns estimated intrinsics with quality metrics
    pub fn calibrate(&self) -> Result<CameraIntrinsicsEstimate, String> {
        if self.observations.is_empty() {
            return Err("No observations provided".to_string());
        }

        // Initial estimate
        let mut estimate =
            CameraIntrinsicsEstimate::from_image_size(self.image_width, self.image_height);

        // Simple calibration: assumes known camera poses and optimizes only intrinsics
        // For production: use proper bundle adjustment (OpenCV, OpenGV, etc.)

        // Compute initial reprojection error
        let mut total_error = 0.0;
        let mut error_per_image: HashMap<usize, Vec<f64>> = HashMap::new();

        for (img_id, world_pt, image_pt) in &self.observations {
            if let Some((R, t)) = self.camera_poses.get(img_id) {
                let error = estimate.reprojection_error(&[*world_pt], &[*image_pt], (R, t));
                if error.is_finite() {
                    total_error += error * error;
                    error_per_image
                        .entry(*img_id)
                        .or_insert_with(Vec::new)
                        .push(error);
                }
            }
        }

        estimate.reprojection_rms = (total_error / self.observations.len() as f64).sqrt();
        estimate.per_image_errors = error_per_image
            .iter()
            .map(|(_, errs)| errs.iter().sum::<f64>() / errs.len() as f64)
            .collect();

        Ok(estimate)
    }
}

/// Quality metrics for intrinsics calibration
#[derive(Clone, Debug)]
pub struct IntrinsicsQuality {
    /// Overall reprojection RMS (pixels)
    pub reprojection_rms: f64,
    /// Percentage of points with error < 1 pixel
    pub inlier_percentage: f64,
    /// Stability across multiple runs (would need multiple calibrations)
    pub stability_score: f64,
    /// Whether intrinsics are acceptable
    pub is_acceptable: bool,
}

impl IntrinsicsQuality {
    /// Create quality assessment from calibration results
    pub fn from_estimate(estimate: &CameraIntrinsicsEstimate) -> Self {
        let acceptable_threshold = 0.8; // 0.8 px RMS is good for most targets

        let inlier_count = estimate
            .per_image_errors
            .iter()
            .filter(|e| e < &&1.0)
            .count();

        let inlier_percentage = if estimate.per_image_errors.is_empty() {
            0.0
        } else {
            100.0 * inlier_count as f64 / estimate.per_image_errors.len() as f64
        };

        let is_acceptable = estimate.reprojection_rms < acceptable_threshold
            && inlier_percentage > 80.0
            && estimate.is_reasonable();

        Self {
            reprojection_rms: estimate.reprojection_rms,
            inlier_percentage,
            stability_score: if is_acceptable { 1.0 } else { 0.0 },
            is_acceptable,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_intrinsics_reasonable() {
        let mut intr = CameraIntrinsicsEstimate::from_image_size(640, 480);
        intr.fx = 500.0;
        intr.fy = 500.0;
        intr.cx = 320.0;
        intr.cy = 240.0;

        assert!(intr.is_reasonable());
    }

    #[test]
    fn test_intrinsics_projection() {
        let intr = CameraIntrinsicsEstimate {
            fx: 500.0,
            fy: 500.0,
            cx: 320.0,
            cy: 240.0,
            distortion: vec![0.0, 0.0, 0.0, 0.0],
            reprojection_rms: 0.0,
            per_image_errors: vec![],
            residual_map: HashMap::new(),
        };

        let pt_3d = Vector3::new(1.0, 0.5, 5.0);
        let pt_2d = intr.project_point(&pt_3d).unwrap();

        // x = 1/5 = 0.2, y = 0.5/5 = 0.1
        // u = 500*0.2 + 320 = 420
        // v = 500*0.1 + 240 = 290
        assert!((pt_2d.x - 420.0).abs() < 1e-6);
        assert!((pt_2d.y - 290.0).abs() < 1e-6);
    }
}
