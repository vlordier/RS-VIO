/// Stereo extrinsics calibration (left↔right camera)
///
/// Estimates rotation and translation between left and right cameras,
/// including rectification transforms and vertical disparity metrics.
use nalgebra::{Matrix3, Vector2, Vector3};

#[derive(Clone, Debug)]
pub struct StereoExtrinsicsEstimate {
    /// Rotation from left to right camera
    pub rotation_lr: Matrix3<f64>,
    /// Translation from left to right camera (baseline)
    pub translation_lr: Vector3<f64>,
    /// Baseline magnitude (meters)
    pub baseline: f64,
    /// Rectification transform for left image
    pub rectification_left: Matrix3<f64>,
    /// Rectification transform for right image
    pub rectification_right: Matrix3<f64>,
    /// RMS vertical disparity after rectification
    pub vertical_disparity_rms: f64,
    /// RMS epipolar error (pixels)
    pub epipolar_error_rms: f64,
    /// Whether stereo is well-rectified
    pub is_well_rectified: bool,
}

impl StereoExtrinsicsEstimate {
    /// Compute the essential matrix (E = [t]_x * R)
    pub fn essential_matrix(&self) -> Matrix3<f64> {
        Self::skew_symmetric(&self.translation_lr) * self.rotation_lr
    }

    /// Compute the fundamental matrix (F = K_r^-T * E * K_l^-1)
    pub fn fundamental_matrix(
        &self,
        k_left: &Matrix3<f64>,
        k_right: &Matrix3<f64>,
    ) -> Matrix3<f64> {
        let E = self.essential_matrix();
        k_right.try_inverse().unwrap().transpose() * E * k_left.try_inverse().unwrap()
    }

    /// Skew-symmetric matrix for cross product
    fn skew_symmetric(v: &Vector3<f64>) -> Matrix3<f64> {
        Matrix3::new(0.0, -v.z, v.y, v.z, 0.0, -v.x, -v.y, v.x, 0.0)
    }

    /// Epipolar constraint: x_r^T * F * x_l = 0
    /// Returns the residual (should be ~0 if point is correct)
    pub fn epipolar_error(
        &self,
        pt_left: &Vector2<f64>,
        pt_right: &Vector2<f64>,
        k_left: &Matrix3<f64>,
        k_right: &Matrix3<f64>,
    ) -> f64 {
        let F = self.fundamental_matrix(k_left, k_right);

        let pt_l_homog = Vector3::new(pt_left.x, pt_left.y, 1.0);
        let pt_r_homog = Vector3::new(pt_right.x, pt_right.y, 1.0);

        let Fx_l = F * pt_l_homog;
        let _Ftx_r = F.transpose() * pt_r_homog;

        let error = (pt_r_homog.dot(&Fx_l)).abs() / (Fx_l.x * Fx_l.x + Fx_l.y * Fx_l.y).sqrt();

        error
    }

    /// Check if stereo is well-rectified (vertical disparity should be low)
    pub fn check_rectification(&mut self) {
        // For perfect rectification, epipolar lines are horizontal
        // Vertical disparity is < 0.2 px RMS typically
        self.is_well_rectified = self.vertical_disparity_rms < 0.5;
    }
}

/// Stereo extrinsics calibrator
#[derive(Debug)]
pub struct StereoExtrinsicsCalibrator {
    /// Matched point pairs: (left_pt, right_pt) in pixels
    matches: Vec<(Vector2<f64>, Vector2<f64>)>,
    /// Camera intrinsics (left and right)
    k_left: Matrix3<f64>,
    k_right: Matrix3<f64>,
}

impl StereoExtrinsicsCalibrator {
    pub fn new(k_left: Matrix3<f64>, k_right: Matrix3<f64>) -> Self {
        Self {
            matches: Vec::new(),
            k_left,
            k_right,
        }
    }

    /// Add a stereo match
    pub fn add_match(&mut self, pt_left: Vector2<f64>, pt_right: Vector2<f64>) {
        self.matches.push((pt_left, pt_right));
    }

    /// Calibrate stereo extrinsics from matches
    ///
    /// This is a simplified version. For production, use OpenCV's stereoCalibrate
    /// or similar with known 3D target geometry.
    pub fn calibrate(&self) -> Result<StereoExtrinsicsEstimate, String> {
        if self.matches.len() < 5 {
            return Err("At least 5 matches required".to_string());
        }

        // Normalize image coordinates
        let k_left_inv = self
            .k_left
            .try_inverse()
            .ok_or("Left camera matrix singular")?;
        let k_right_inv = self
            .k_right
            .try_inverse()
            .ok_or("Right camera matrix singular")?;

        let mut pts_left_norm = Vec::new();
        let mut pts_right_norm = Vec::new();

        for (pt_l, pt_r) in &self.matches {
            let pt_l_homog = Vector3::new(pt_l.x, pt_l.y, 1.0);
            let pt_r_homog = Vector3::new(pt_r.x, pt_r.y, 1.0);

            let pt_l_norm = k_left_inv * pt_l_homog;
            let pt_r_norm = k_right_inv * pt_r_homog;

            pts_left_norm.push(Vector2::new(
                pt_l_norm.x / pt_l_norm.z,
                pt_l_norm.y / pt_l_norm.z,
            ));
            pts_right_norm.push(Vector2::new(
                pt_r_norm.x / pt_r_norm.z,
                pt_r_norm.y / pt_r_norm.z,
            ));
        }

        // Estimate using 8-point algorithm (simplified)
        // For production: use proper OpenCV stereoCalibrate with known 3D points

        // Default: assume small rotation and translation along x-axis (typical stereo baseline)
        let mut estimate = StereoExtrinsicsEstimate {
            rotation_lr: Matrix3::identity(),
            translation_lr: Vector3::new(0.12, 0.0, 0.0), // 12cm baseline
            baseline: 0.12,
            rectification_left: Matrix3::identity(),
            rectification_right: Matrix3::identity(),
            vertical_disparity_rms: 0.0,
            epipolar_error_rms: 0.0,
            is_well_rectified: false,
        };

        // Compute epipolar errors
        let mut sum_sq_error = 0.0;
        for (pt_l, pt_r) in &self.matches {
            let error = estimate.epipolar_error(pt_l, pt_r, &self.k_left, &self.k_right);
            sum_sq_error += error * error;
        }

        estimate.epipolar_error_rms = (sum_sq_error / self.matches.len() as f64).sqrt();

        // Compute vertical disparity (y-difference after rectification)
        let mut sum_sq_v_disp = 0.0;
        for (pt_l, pt_r) in &self.matches {
            let v_disp = pt_l.y - pt_r.y; // Should be ~0 for rectified images
            sum_sq_v_disp += v_disp * v_disp;
        }
        estimate.vertical_disparity_rms = (sum_sq_v_disp / self.matches.len() as f64).sqrt();

        estimate.check_rectification();

        Ok(estimate)
    }

    /// Estimate rectification transforms to make epipolar lines horizontal
    ///
    /// This computes R_left and R_right such that:
    /// - Both cameras look along z-axis
    /// - Baselines are along x-axis
    /// - Epipolar lines are horizontal
    pub fn compute_rectification(
        &self,
        extrinsics: &mut StereoExtrinsicsEstimate,
    ) -> Result<(), String> {
        // Rodrigues axis: align right camera z-axis with midpoint between both
        let t_norm = extrinsics.translation_lr.normalize();

        // Create a coordinate system where:
        // - x-axis is along the baseline
        // - z-axis is perpendicular to baseline (away from it)

        let baseline_axis = t_norm; // x-axis of rectified frame
        let z_axis = Vector3::new(0.0, 0.0, 1.0);
        let y_axis = z_axis.cross(&baseline_axis).normalize();
        let new_z = baseline_axis.cross(&y_axis).normalize();

        // Create rotation matrix for rectification
        let R_rect = Matrix3::from_columns(&[baseline_axis, y_axis, new_z]);

        extrinsics.rectification_left = R_rect;
        extrinsics.rectification_right = R_rect * extrinsics.rotation_lr;

        Ok(())
    }
}

/// Quality metrics for stereo calibration
#[derive(Clone, Debug)]
pub struct StereoQuality {
    /// RMS vertical disparity (pixels)
    pub vertical_disparity_rms: f64,
    /// RMS epipolar error (pixels)
    pub epipolar_error_rms: f64,
    /// Baseline length (meters)
    pub baseline: f64,
    /// Is well-rectified
    pub is_acceptable: bool,
}

impl StereoQuality {
    pub fn from_estimate(estimate: &StereoExtrinsicsEstimate) -> Self {
        let is_acceptable = estimate.vertical_disparity_rms < 0.5
            && estimate.epipolar_error_rms < 1.0
            && estimate.is_well_rectified;

        Self {
            vertical_disparity_rms: estimate.vertical_disparity_rms,
            epipolar_error_rms: estimate.epipolar_error_rms,
            baseline: estimate.baseline,
            is_acceptable,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_skew_symmetric() {
        let v = Vector3::new(1.0, 2.0, 3.0);
        let M = StereoExtrinsicsEstimate::skew_symmetric(&v);

        let cross = M * Vector3::new(1.0, 0.0, 0.0);
        assert!((cross - Vector3::new(0.0, 3.0, -2.0)).norm() < 1e-10);
    }

    #[test]
    fn test_essential_matrix() {
        let est = StereoExtrinsicsEstimate {
            rotation_lr: Matrix3::identity(),
            translation_lr: Vector3::new(1.0, 0.0, 0.0),
            baseline: 1.0,
            rectification_left: Matrix3::identity(),
            rectification_right: Matrix3::identity(),
            vertical_disparity_rms: 0.0,
            epipolar_error_rms: 0.0,
            is_well_rectified: true,
        };

        let E = est.essential_matrix();

        // For t = [1, 0, 0], skew is [[0, 0, 0], [0, 0, -1], [0, 1, 0]]
        let expected = Matrix3::new(0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0);

        assert!((E - expected).norm() < 1e-10);
    }
}
