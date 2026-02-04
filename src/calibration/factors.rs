//! Optimization factors for stereo camera calibration

use apex_solver::factors::Factor;
use nalgebra as na;

/// Reprojection factor for stereo calibration
///
/// This factor enforces that a 3D point projects correctly into both left and right cameras.
/// It optimizes camera intrinsics, extrinsics, and 3D point positions.
///
/// Parameters:
/// - params[0]: Left camera intrinsics (9D: fx, fy, cx, cy, k1, k2, p1, p2, k3)
/// - params[1]: Right camera intrinsics (9D: fx, fy, cx, cy, k1, k2, p1, p2, k3)
/// - params[2]: Stereo extrinsics (6D: rx, ry, rz, tx, ty, tz) - relative pose
/// - params[3]: 3D point position (3D: X, Y, Z)
///
/// Residual: 4D (left_u, left_v, right_u, right_v reprojection errors)
pub struct StereoReprojectionFactor {
    /// Observed left camera feature point
    pub left_observation: na::Vector2<f64>,
    /// Observed right camera feature point
    pub right_observation: na::Vector2<f64>,
}

impl StereoReprojectionFactor {
    pub fn new(left_obs: na::Vector2<f64>, right_obs: na::Vector2<f64>) -> Self {
        Self {
            left_observation: left_obs,
            right_observation: right_obs,
        }
    }

    /// Extract camera intrinsics from parameter vector
    fn extract_intrinsics(params: &[f64]) -> Vec<f64> {
        params.iter().take(9).cloned().collect()
    }

    /// Extract relative pose from parameter vector (6D -> SE(3))
    fn extract_relative_pose(params: &[f64]) -> na::Isometry3<f64> {
        let rx = params[0];
        let ry = params[1];
        let rz = params[2];
        let tx = params[3];
        let ty = params[4];
        let tz = params[5];

        let rotation = na::UnitQuaternion::from_euler_angles(rx, ry, rz);
        let translation = na::Vector3::new(tx, ty, tz);

        na::Isometry3::from_parts(translation.into(), rotation)
    }

    /// Extract 3D point from parameter vector
    fn extract_point(params: &[f64]) -> na::Vector3<f64> {
        na::Vector3::new(params[0], params[1], params[2])
    }
}

impl Factor for StereoReprojectionFactor {
    fn linearize(
        &self,
        params: &[na::DVector<f64>],
        compute_jacobian: bool,
    ) -> (na::DVector<f64>, Option<na::DMatrix<f64>>) {
        // params[0] = left intrinsics (9D)
        // params[1] = right intrinsics (9D)
        // params[2] = relative extrinsics (6D: rx, ry, rz, tx, ty, tz)
        // params[3] = 3D point (3D: x, y, z)

        assert_eq!(params.len(), 4, "StereoReprojectionFactor requires 4 parameter vectors");
        assert_eq!(params[0].len(), 9, "Left intrinsics must have 9 parameters");
        assert_eq!(params[1].len(), 9, "Right intrinsics must have 9 parameters");
        assert_eq!(params[2].len(), 6, "Extrinsics must have 6 parameters");
        assert_eq!(params[3].len(), 3, "3D point must have 3 parameters");

        let left_intrinsics = params[0].as_slice();
        let right_intrinsics = params[1].as_slice();
        let extrinsics = params[2].as_slice();
        let point_3d = na::Vector3::new(params[3][0], params[3][1], params[3][2]);

        // Extract camera parameters (simplified pinhole model)
        let fx_l = left_intrinsics[0];
        let fy_l = left_intrinsics[1];
        let cx_l = left_intrinsics[2];
        let cy_l = left_intrinsics[3];

        let fx_r = right_intrinsics[0];
        let fy_r = right_intrinsics[1];
        let cx_r = right_intrinsics[2];
        let cy_r = right_intrinsics[3];

        // Extract relative pose
        let rx = extrinsics[0];
        let ry = extrinsics[1];
        let rz = extrinsics[2];
        let tx = extrinsics[3];
        let ty = extrinsics[4];
        let tz = extrinsics[5];

        let rotation = na::UnitQuaternion::from_euler_angles(rx, ry, rz);
        let translation = na::Vector3::new(tx, ty, tz);
        let relative_pose = na::Isometry3::from_parts(translation.into(), rotation);

        // Project point into left camera
        let left_proj_x = fx_l * point_3d.x / point_3d.z + cx_l;
        let left_proj_y = fy_l * point_3d.y / point_3d.z + cy_l;

        // Transform point to right camera frame and project
        let point_in_right = relative_pose.inverse() * point_3d;
        let right_proj_x = fx_r * point_in_right.x / point_in_right.z + cx_r;
        let right_proj_y = fy_r * point_in_right.y / point_in_right.z + cy_r;

        // Compute residuals (4D)
        let mut residuals = na::DVector::zeros(4);
        residuals[0] = left_proj_x - self.left_observation.x;
        residuals[1] = left_proj_y - self.left_observation.y;
        residuals[2] = right_proj_x - self.right_observation.x;
        residuals[3] = right_proj_y - self.right_observation.y;

        // For now, skip analytical Jacobians (numerical differentiation will be used)
        let jacobian = if compute_jacobian {
            // TODO: Implement analytical Jacobians
            // This would be a 4x(9+9+6+3) = 4x27 matrix
            Some(na::DMatrix::zeros(4, 27))
        } else {
            None
        };

        (residuals, jacobian)
    }

    fn get_dimension(&self) -> usize {
        4 // left_u, left_v, right_u, right_v
    }
}

/// Epipolar constraint factor for stereo calibration
///
/// This factor enforces the epipolar constraint between stereo views.
/// It provides a geometric constraint that doesn't require 3D triangulation.
///
/// Parameters:
/// - params[0]: Left camera intrinsics (9D)
/// - params[1]: Right camera intrinsics (9D)
/// - params[2]: Stereo extrinsics (6D)
///
/// Residual: 1D (epipolar error)
pub struct EpipolarFactor {
    /// Observed left camera feature point
    pub left_point: na::Vector2<f64>,
    /// Observed right camera feature point
    pub right_point: na::Vector2<f64>,
}

impl EpipolarFactor {
    pub fn new(left_pt: na::Vector2<f64>, right_pt: na::Vector2<f64>) -> Self {
        Self {
            left_point: left_pt,
            right_point: right_pt,
        }
    }
}

impl Factor for EpipolarFactor {
    fn linearize(
        &self,
        params: &[na::DVector<f64>],
        compute_jacobian: bool,
    ) -> (na::DVector<f64>, Option<na::DMatrix<f64>>) {
        // params[0] = left intrinsics (9D)
        // params[1] = right intrinsics (9D)
        // params[2] = relative extrinsics (6D)

        assert_eq!(params.len(), 3, "EpipolarFactor requires 3 parameter vectors");
        assert_eq!(params[0].len(), 9, "Left intrinsics must have 9 parameters");
        assert_eq!(params[1].len(), 9, "Right intrinsics must have 9 parameters");
        assert_eq!(params[2].len(), 6, "Extrinsics must have 6 parameters");

        let left_intrinsics = params[0].as_slice();
        let right_intrinsics = params[1].as_slice();
        let extrinsics = params[2].as_slice();

        // Extract camera matrices
        let k_left = na::Matrix3::new(
            left_intrinsics[0], 0.0, left_intrinsics[2],
            0.0, left_intrinsics[1], left_intrinsics[3],
            0.0, 0.0, 1.0
        );

        let k_right = na::Matrix3::new(
            right_intrinsics[0], 0.0, right_intrinsics[2],
            0.0, right_intrinsics[1], right_intrinsics[3],
            0.0, 0.0, 1.0
        );

        // Extract relative pose
        let rx = extrinsics[0];
        let ry = extrinsics[1];
        let rz = extrinsics[2];
        let tx = extrinsics[3];
        let ty = extrinsics[4];
        let tz = extrinsics[5];

        let rotation = na::UnitQuaternion::from_euler_angles(rx, ry, rz);
        let translation = na::Vector3::new(tx, ty, tz);
        let relative_pose = na::Isometry3::from_parts(translation.into(), rotation);

        let rotation_matrix = relative_pose.rotation.to_rotation_matrix();
        let r = rotation_matrix.matrix();
        let t = relative_pose.translation.vector;

        // Essential matrix: E = [t]× * R
        let t_skew = na::Matrix3::new(
            0.0, -t.z, t.y,
            t.z, 0.0, -t.x,
            -t.y, t.x, 0.0
        );
        let e = t_skew * r;

        // Fundamental matrix: F = K_right^(-T) * E * K_left^(-1)
        let k_right_inv_t = k_right.transpose().try_inverse().unwrap_or(na::Matrix3::identity());
        let k_left_inv = k_left.try_inverse().unwrap_or(na::Matrix3::identity());
        let f = k_right_inv_t * e * k_left_inv;

        // Compute epipolar line in right image: l = F * p_left
        let p_left_homogeneous = na::Vector3::new(self.left_point.x, self.left_point.y, 1.0);
        let epipolar_line = f * p_left_homogeneous;

        // Compute distance from right point to epipolar line
        let p_right_homogeneous = na::Vector3::new(self.right_point.x, self.right_point.y, 1.0);
        let numerator = epipolar_line.dot(&p_right_homogeneous).abs();
        let denominator = (epipolar_line.x.powi(2) + epipolar_line.y.powi(2)).sqrt();
        let distance = if denominator > 1e-12 { numerator / denominator } else { 0.0 };

        let mut residuals = na::DVector::zeros(1);
        residuals[0] = distance;

        // For now, skip analytical Jacobians
        let jacobian = if compute_jacobian {
            // TODO: Implement analytical Jacobians (1x24 matrix)
            Some(na::DMatrix::zeros(1, 24))
        } else {
            None
        };

        (residuals, jacobian)
    }

    fn get_dimension(&self) -> usize {
        1 // epipolar error
    }
}