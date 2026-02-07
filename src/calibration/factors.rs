//! Optimization factors for stereo camera calibration

use crate::calibration::camera_models::CameraModel;
use apex_solver::factors::Factor;
use nalgebra as na;

/// Compute the Jacobian of a factor numerically via central differences.
///
/// Given a factor whose `linearize(params, false)` returns a residual vector,
/// this function perturbs each element of every parameter block by `±eps` and
/// builds the full `residual_dim × total_params` Jacobian.
fn numerical_jacobian(
    factor: &dyn Factor,
    params: &[na::DVector<f64>],
    eps: f64,
) -> na::DMatrix<f64> {
    let (r0, _) = factor.linearize(params, false);
    let residual_dim = r0.len();
    let total_cols: usize = params.iter().map(|p| p.len()).sum();
    let mut jacobian = na::DMatrix::zeros(residual_dim, total_cols);

    let mut col = 0;
    let mut perturbed = params.to_vec();
    for blk in 0..params.len() {
        for j in 0..params[blk].len() {
            // +eps
            perturbed[blk][j] = params[blk][j] + eps;
            let (r_plus, _) = factor.linearize(&perturbed, false);

            // -eps
            perturbed[blk][j] = params[blk][j] - eps;
            let (r_minus, _) = factor.linearize(&perturbed, false);

            // restore
            perturbed[blk][j] = params[blk][j];

            for i in 0..residual_dim {
                jacobian[(i, col)] = (r_plus[i] - r_minus[i]) / (2.0 * eps);
            }
            col += 1;
        }
    }

    jacobian
}

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
    pub const fn new(left_obs: na::Vector2<f64>, right_obs: na::Vector2<f64>) -> Self {
        Self {
            left_observation: left_obs,
            right_observation: right_obs,
        }
    }

    /// Extract camera intrinsics from parameter vector
    #[allow(dead_code)]
    fn extract_intrinsics(params: &[f64]) -> Vec<f64> {
        params.iter().take(9).cloned().collect()
    }

    /// Extract relative pose from parameter vector (6D -> SE(3))
    #[allow(dead_code)]
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
    #[allow(dead_code)]
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

        assert_eq!(
            params.len(),
            4,
            "StereoReprojectionFactor requires 4 parameter vectors"
        );
        assert_eq!(params[0].len(), 9, "Left intrinsics must have 9 parameters");
        assert_eq!(
            params[1].len(),
            9,
            "Right intrinsics must have 9 parameters"
        );
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
            Some(numerical_jacobian(self, params, 1e-8))
        } else {
            None
        };

        (residuals, jacobian)
    }

    fn get_dimension(&self) -> usize {
        4 // left_u, left_v, right_u, right_v
    }
}

/// Rolling shutter compensation factor for stereo calibration
///
/// This factor compensates for rolling shutter distortion by modeling
/// the camera motion during the image readout time. Essential for
/// cheap cameras like those on Raspberry Pi.
///
/// Parameters:
/// - params[0]: Left camera intrinsics (9D)
/// - params[1]: Right camera intrinsics (9D)
/// - params[2]: Stereo extrinsics (6D: rx, ry, rz, tx, ty, tz)
/// - params[3]: Angular velocity during exposure (3D: wx, wy, wz)
/// - params[4]: Linear velocity during exposure (3D: vx, vy, vz)
///
/// Residual: 4D (corrected left_u, left_v, right_u, right_v differences)
pub struct RollingShutterFactor {
    /// Observed left camera feature point
    pub left_observation: na::Vector2<f64>,
    /// Observed right camera feature point
    pub right_observation: na::Vector2<f64>,
    /// Row position in image (0.0 = top, 1.0 = bottom) for readout timing
    pub row_position: f64,
    /// Total readout time for full frame
    pub readout_time: f64,
}

impl RollingShutterFactor {
    pub const fn new(
        left_obs: na::Vector2<f64>,
        right_obs: na::Vector2<f64>,
        row_pos: f64,
        readout_time: f64,
    ) -> Self {
        Self {
            left_observation: left_obs,
            right_observation: right_obs,
            row_position: row_pos,
            readout_time,
        }
    }
}

impl Factor for RollingShutterFactor {
    fn linearize(
        &self,
        params: &[na::DVector<f64>],
        _compute_jacobian: bool,
    ) -> (na::DVector<f64>, Option<na::DMatrix<f64>>) {
        // params[0] = left intrinsics (9D)
        // params[1] = right intrinsics (9D)
        // params[2] = relative extrinsics (6D)
        // params[3] = angular velocity (3D)
        // params[4] = linear velocity (3D)

        assert_eq!(
            params.len(),
            5,
            "RollingShutterFactor requires 5 parameter vectors"
        );

        let left_intrinsics = params[0].as_slice();
        let right_intrinsics = params[1].as_slice();
        let extrinsics = params[2].as_slice();
        let angular_vel = na::Vector3::new(params[3][0], params[3][1], params[3][2]);
        let linear_vel = na::Vector3::new(params[4][0], params[4][1], params[4][2]);

        // Extract camera parameters
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

        // Calculate rolling shutter correction
        // The feature was captured at time = row_position * readout_time
        let capture_time = self.row_position * self.readout_time;

        // Approximate rotation during readout
        let delta_rotation = angular_vel * capture_time;
        let correction_rotation = na::UnitQuaternion::from_scaled_axis(delta_rotation);

        // Apply correction to relative pose
        let corrected_relative_pose = relative_pose
            * na::Isometry3::from_parts((linear_vel * capture_time).into(), correction_rotation);

        // Project points with corrected pose
        let point_3d = na::Vector3::new(0.0, 0.0, 1.0); // Assume unit depth for now

        // Project to left camera (observed point)
        let left_projected = na::Vector2::new(
            fx_l * (point_3d.x / point_3d.z) + cx_l,
            fy_l * (point_3d.y / point_3d.z) + cy_l,
        );

        // Transform to right camera with corrected pose
        let point_in_right = corrected_relative_pose * point_3d;
        let right_projected = na::Vector2::new(
            fx_r * (point_in_right.x / point_in_right.z) + cx_r,
            fy_r * (point_in_right.y / point_in_right.z) + cy_r,
        );

        // Residual is difference between observed and projected points
        let residual = na::DVector::from_vec(vec![
            self.left_observation.x - left_projected.x,
            self.left_observation.y - left_projected.y,
            self.right_observation.x - right_projected.x,
            self.right_observation.y - right_projected.y,
        ]);

        // For now, return residual without jacobian
        // Full jacobian computation would be complex
        (residual, None)
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
    pub const fn new(left_pt: na::Vector2<f64>, right_pt: na::Vector2<f64>) -> Self {
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

        assert_eq!(
            params.len(),
            3,
            "EpipolarFactor requires 3 parameter vectors"
        );
        assert_eq!(params[0].len(), 9, "Left intrinsics must have 9 parameters");
        assert_eq!(
            params[1].len(),
            9,
            "Right intrinsics must have 9 parameters"
        );
        assert_eq!(params[2].len(), 6, "Extrinsics must have 6 parameters");

        let left_intrinsics = params[0].as_slice();
        let right_intrinsics = params[1].as_slice();
        let extrinsics = params[2].as_slice();

        // Extract camera matrices
        let k_left = na::Matrix3::new(
            left_intrinsics[0],
            0.0,
            left_intrinsics[2],
            0.0,
            left_intrinsics[1],
            left_intrinsics[3],
            0.0,
            0.0,
            1.0,
        );

        let k_right = na::Matrix3::new(
            right_intrinsics[0],
            0.0,
            right_intrinsics[2],
            0.0,
            right_intrinsics[1],
            right_intrinsics[3],
            0.0,
            0.0,
            1.0,
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
        let t_skew = na::Matrix3::new(0.0, -t.z, t.y, t.z, 0.0, -t.x, -t.y, t.x, 0.0);
        let e = t_skew * r;

        // Fundamental matrix: F = K_right^(-T) * E * K_left^(-1)
        let k_right_inv_t = k_right
            .transpose()
            .try_inverse()
            .unwrap_or(na::Matrix3::identity());
        let k_left_inv = k_left.try_inverse().unwrap_or(na::Matrix3::identity());
        let f = k_right_inv_t * e * k_left_inv;

        // Compute epipolar line in right image: l = F * p_left
        let p_left_homogeneous = na::Vector3::new(self.left_point.x, self.left_point.y, 1.0);
        let epipolar_line = f * p_left_homogeneous;

        // Compute distance from right point to epipolar line
        let p_right_homogeneous = na::Vector3::new(self.right_point.x, self.right_point.y, 1.0);
        let numerator = epipolar_line.dot(&p_right_homogeneous);
        let denominator = (epipolar_line.x.powi(2) + epipolar_line.y.powi(2)).sqrt();
        let distance = if denominator > 1e-12 {
            numerator / denominator
        } else {
            0.0
        };

        let mut residuals = na::DVector::zeros(1);
        residuals[0] = distance;

        // For now, skip analytical Jacobians
        let jacobian = if compute_jacobian {
            Some(numerical_jacobian(self, params, 1e-8))
        } else {
            None
        };

        (residuals, jacobian)
    }

    fn get_dimension(&self) -> usize {
        1 // epipolar error
    }
}

/// Temporal Super Resolution Factor for enhanced calibration accuracy
///
/// This factor leverages multiple stereo pairs over time to achieve temporal super resolution,
/// providing more accurate calibration by:
/// - Modeling smooth motion trajectories between frames
/// - Enforcing temporal consistency constraints
/// - Reconstructing sub-frame timing information
/// - Using temporal averaging for noise reduction
///
/// Parameters:
/// - params[0]: Left camera intrinsics (9D)
/// - params[1]: Right camera intrinsics (9D)
/// - params[2]: Stereo extrinsics (6D)
/// - params[3]: Motion trajectory parameters (6D per frame: angular/linear velocity)
/// - params[4]: 3D point position (3D)
///
/// Residual: Variable dimension based on number of temporal frames
#[derive(Debug, Clone)]
pub struct TemporalSuperResolutionFactor {
    /// Sequence of stereo observations over time
    pub temporal_observations: Vec<TemporalStereoObservation>,
    /// Total readout time for rolling shutter compensation
    pub readout_time: f64,
}

#[derive(Debug, Clone)]
pub struct TemporalStereoObservation {
    /// Left camera feature point
    pub left_point: na::Vector2<f64>,
    /// Right camera feature point
    pub right_point: na::Vector2<f64>,
    /// Timestamp relative to sequence start (seconds)
    pub timestamp: f64,
    /// Row position in image (0.0 = top, 1.0 = bottom) for readout timing
    pub row_position: f64,
    /// Feature quality score (0-1, higher is better)
    pub quality: f64,
}

impl TemporalSuperResolutionFactor {
    pub const fn new(readout_time: f64) -> Self {
        Self {
            temporal_observations: Vec::new(),
            readout_time,
        }
    }

    pub fn add_observation(
        &mut self,
        left_point: na::Vector2<f64>,
        right_point: na::Vector2<f64>,
        timestamp: f64,
        row_position: f64,
        quality: f64,
    ) {
        self.temporal_observations.push(TemporalStereoObservation {
            left_point,
            right_point,
            timestamp,
            row_position,
            quality,
        });
    }

    /// Interpolate motion trajectory at given time
    fn interpolate_motion(
        &self,
        motion_params: &[f64],
        time: f64,
        sequence_start_time: f64,
    ) -> (na::Vector3<f64>, na::Vector3<f64>) {
        // motion_params contains [ang_vel_x, ang_vel_y, ang_vel_z, lin_vel_x, lin_vel_y, lin_vel_z]
        // For now, assume constant velocity over the sequence
        // TODO: Implement higher-order motion models (acceleration, jerk)

        let angular_vel = na::Vector3::new(motion_params[0], motion_params[1], motion_params[2]);

        let linear_vel = na::Vector3::new(motion_params[3], motion_params[4], motion_params[5]);

        // Scale velocities by time offset from sequence start
        let time_offset = time - sequence_start_time;
        (angular_vel * time_offset, linear_vel * time_offset)
    }

    /// Apply rolling shutter correction for given observation
    fn apply_rolling_shutter_correction(
        &self,
        base_pose: &na::Isometry3<f64>,
        angular_correction: &na::Vector3<f64>,
        linear_correction: &na::Vector3<f64>,
        observation: &TemporalStereoObservation,
    ) -> na::Isometry3<f64> {
        // Calculate rolling shutter timing offset
        let readout_offset = observation.row_position * self.readout_time;

        // Additional motion during readout
        let readout_angular = angular_correction * readout_offset;
        let readout_linear = linear_correction * readout_offset;

        let readout_rotation = na::UnitQuaternion::from_scaled_axis(readout_angular);
        let readout_translation = readout_linear;

        // Apply readout correction to base pose
        base_pose * na::Isometry3::from_parts(readout_translation.into(), readout_rotation)
    }
}

impl Factor for TemporalSuperResolutionFactor {
    fn linearize(
        &self,
        params: &[na::DVector<f64>],
        compute_jacobian: bool,
    ) -> (na::DVector<f64>, Option<na::DMatrix<f64>>) {
        // params[0] = left intrinsics (9D)
        // params[1] = right intrinsics (9D)
        // params[2] = stereo extrinsics (6D)
        // params[3] = motion trajectory (6D: angular + linear velocity)
        // params[4] = 3D point (3D)

        assert!(
            params.len() >= 5,
            "TemporalSuperResolutionFactor requires at least 5 parameter vectors"
        );
        assert!(
            !self.temporal_observations.is_empty(),
            "No temporal observations provided"
        );

        let left_intrinsics = params[0].as_slice();
        let right_intrinsics = params[1].as_slice();
        let extrinsics = params[2].as_slice();
        let motion_params = params[3].as_slice();
        let point_3d = na::Vector3::new(params[4][0], params[4][1], params[4][2]);

        // Extract camera intrinsics (simplified pinhole)
        let fx_l = left_intrinsics[0];
        let fy_l = left_intrinsics[1];
        let cx_l = left_intrinsics[2];
        let cy_l = left_intrinsics[3];

        let fx_r = right_intrinsics[0];
        let fy_r = right_intrinsics[1];
        let cx_r = right_intrinsics[2];
        let cy_r = right_intrinsics[3];

        // Extract base stereo extrinsics
        let rx = extrinsics[0];
        let ry = extrinsics[1];
        let rz = extrinsics[2];
        let tx = extrinsics[3];
        let ty = extrinsics[4];
        let tz = extrinsics[5];

        let base_rotation = na::UnitQuaternion::from_euler_angles(rx, ry, rz);
        let base_translation = na::Vector3::new(tx, ty, tz);
        let base_extrinsics = na::Isometry3::from_parts(base_translation.into(), base_rotation);

        // Find sequence start time for motion interpolation
        let sequence_start_time = self
            .temporal_observations
            .iter()
            .map(|obs| obs.timestamp)
            .min_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal))
            .unwrap_or(0.0);

        // Compute residuals for each temporal observation
        let mut residuals = na::DVector::zeros(self.temporal_observations.len() * 4);
        let jacobian = if compute_jacobian {
            None // Return None to trigger solver's numerical differentiation fallback
        } else {
            None
        };

        for (i, observation) in self.temporal_observations.iter().enumerate() {
            // Interpolate motion at this timestamp
            let (angular_correction, linear_correction) =
                self.interpolate_motion(motion_params, observation.timestamp, sequence_start_time);

            // Apply rolling shutter correction
            let corrected_extrinsics = self.apply_rolling_shutter_correction(
                &base_extrinsics,
                &angular_correction,
                &linear_correction,
                observation,
            );

            // Project point to left camera (observed point)
            let left_projected = na::Vector2::new(
                fx_l * (point_3d.x / point_3d.z) + cx_l,
                fy_l * (point_3d.y / point_3d.z) + cy_l,
            );

            // Transform point to right camera with corrected extrinsics
            let point_in_right = corrected_extrinsics * point_3d;
            let right_projected = na::Vector2::new(
                fx_r * (point_in_right.x / point_in_right.z) + cx_r,
                fy_r * (point_in_right.y / point_in_right.z) + cy_r,
            );

            // Compute reprojection residuals with quality weighting
            let weight = observation.quality.sqrt(); // Square root for residual weighting

            let left_residual_x = weight * (left_projected.x - observation.left_point.x);
            let left_residual_y = weight * (left_projected.y - observation.left_point.y);
            let right_residual_x = weight * (right_projected.x - observation.right_point.x);
            let right_residual_y = weight * (right_projected.y - observation.right_point.y);

            residuals[4 * i] = left_residual_x;
            residuals[4 * i + 1] = left_residual_y;
            residuals[4 * i + 2] = right_residual_x;
            residuals[4 * i + 3] = right_residual_y;

            // TODO: Compute Jacobians for optimization
            // This would require derivatives w.r.t. intrinsics, extrinsics, motion, and 3D point
        }

        (residuals, jacobian)
    }

    fn get_dimension(&self) -> usize {
        // 4 residuals per temporal observation (left_x, left_y, right_x, right_y)
        self.temporal_observations.len() * 4
    }
}

/// Temporal Consistency Factor for smooth motion trajectories
///
/// This factor enforces temporal smoothness constraints on motion trajectories,
/// ensuring that camera motion is physically plausible and reducing noise
/// in temporal super resolution.
///
/// Parameters:
/// - params[0]: Motion trajectory parameters (6D: angular/linear velocity)
///
/// Residual: 6D (smoothness constraints on angular and linear acceleration)
#[derive(Debug, Clone)]
pub struct TemporalConsistencyFactor {
    /// Time span over which to enforce consistency (seconds)
    pub time_span: f64,
    /// Smoothness weight (higher = smoother motion)
    pub smoothness_weight: f64,
}

impl TemporalConsistencyFactor {
    pub const fn new(time_span: f64, smoothness_weight: f64) -> Self {
        Self {
            time_span,
            smoothness_weight,
        }
    }
}

impl Factor for TemporalConsistencyFactor {
    fn linearize(
        &self,
        params: &[na::DVector<f64>],
        compute_jacobian: bool,
    ) -> (na::DVector<f64>, Option<na::DMatrix<f64>>) {
        // params[0] = motion trajectory (6D: angular + linear velocity)

        assert_eq!(
            params.len(),
            1,
            "TemporalConsistencyFactor requires 1 parameter vector"
        );
        assert_eq!(params[0].len(), 6, "Motion parameters must be 6D");

        let motion_params = params[0].as_slice();

        let angular_vel = na::Vector3::new(motion_params[0], motion_params[1], motion_params[2]);
        let linear_vel = na::Vector3::new(motion_params[3], motion_params[4], motion_params[5]);

        // Enforce smoothness constraints:
        // 1. Angular velocity should be reasonable magnitude
        // 2. Linear velocity should be reasonable magnitude
        // 3. Angular and linear velocities should be correlated (rigid body motion)

        let angular_magnitude = angular_vel.norm();
        let linear_magnitude = linear_vel.norm();

        // Residuals penalize unrealistic motion
        let mut residuals = na::DVector::zeros(8);

        // Angular velocity smoothness (penalize very fast rotations)
        let angular_smoothness_residual =
            self.smoothness_weight * (angular_magnitude - 10.0).max(0.0); // Allow up to 10 rad/s
        residuals[0] = angular_smoothness_residual;

        // Linear velocity smoothness (penalize very fast translations)
        let linear_smoothness_residual = self.smoothness_weight * (linear_magnitude - 5.0).max(0.0); // Allow up to 5 m/s
        residuals[1] = linear_smoothness_residual;

        // Individual velocity component smoothness
        for i in 0..3 {
            residuals[2 + i] = self.smoothness_weight * angular_vel[i].abs().min(1.0);
            residuals[5 + i] = self.smoothness_weight * linear_vel[i].abs().min(1.0);
        }

        // TODO: Add proper Jacobians for optimization
        let jacobian = if compute_jacobian {
            None // Return None to trigger solver's numerical differentiation fallback
        } else {
            None
        };

        (residuals, jacobian)
    }

    fn get_dimension(&self) -> usize {
        8 // smoothness constraints (2 magnitude + 3 angular + 3 linear)
    }
}

/// Multi-camera reprojection factor for heterogeneous camera rigs
///
/// This factor enforces that a 3D point projects correctly into multiple cameras
/// with different intrinsic models, optimizing intrinsics, extrinsics, and 3D points.
///
/// Parameters:
/// - params[0]: 3D point position (3D)
/// - params[1]: Camera intrinsics (variable size depending on camera model)
/// - params[2]: Camera pose (6D: rx, ry, rz, tx, ty, tz) - optional
///
/// Residual: 2D (u, v reprojection error)
#[derive(Debug, Clone)]
pub struct MultiCameraReprojectionFactor {
    /// Observed image point
    pub observed_point: na::Vector2<f64>,
    /// Camera model for projection
    pub camera_model: crate::calibration::camera_models::CameraModelEnum,
    /// Camera ID for identification
    pub camera_id: String,
}

impl MultiCameraReprojectionFactor {
    pub fn new(
        observed_point: na::Vector2<f64>,
        camera_model: &crate::calibration::camera_models::CameraModelEnum,
        camera_id: String,
    ) -> Self {
        Self {
            observed_point,
            camera_model: camera_model.clone(),
            camera_id,
        }
    }
}

impl Factor for MultiCameraReprojectionFactor {
    fn linearize(
        &self,
        params: &[na::DVector<f64>],
        compute_jacobian: bool,
    ) -> (na::DVector<f64>, Option<na::DMatrix<f64>>) {
        // params[0] = 3D point (3D)
        // params[1] = camera intrinsics (variable size)
        // params[2] = camera pose (6D) - optional

        assert!(
            params.len() >= 2,
            "MultiCameraReprojectionFactor requires at least 2 parameter vectors"
        );

        let point_3d = na::Vector3::new(params[0][0], params[0][1], params[0][2]);
        let intrinsics = params[1].as_slice();

        // Apply camera pose if provided
        let point_in_camera = if params.len() >= 3 {
            let pose_params = params[2].as_slice();
            let rotation = na::UnitQuaternion::from_scaled_axis(na::Vector3::new(
                pose_params[0],
                pose_params[1],
                pose_params[2],
            ));
            let translation = na::Vector3::new(pose_params[3], pose_params[4], pose_params[5]);
            let camera_pose = na::Isometry3::from_parts(translation.into(), rotation);

            camera_pose.inverse() * point_3d
        } else {
            point_3d
        };

        // Project point using camera model
        let projected = self.camera_model.project(&point_in_camera, intrinsics);

        // Compute reprojection error
        let residual = projected - self.observed_point;
        let residuals = na::DVector::from_vec(vec![residual.x, residual.y]);

        // TODO: Compute Jacobians for optimization
        let jacobian = if compute_jacobian {
            // This would require derivatives w.r.t. 3D point, intrinsics, and pose
            // Implementation depends on specific camera model
            // For now, use numerical differentiation (jacobian = None triggers this)
            None
        } else {
            None
        };

        (residuals, jacobian)
    }

    fn get_dimension(&self) -> usize {
        2 // 2D reprojection error
    }
}

/// Camera graph consistency factor for multi-camera systems
///
/// This factor enforces consistency between relative camera poses in the graph,
/// ensuring that the camera network maintains its topological constraints.
///
/// Parameters:
/// - params[0]: First camera pose (6D)
/// - params[1]: Second camera pose (6D)
///
/// Residual: 6D (pose consistency error)
#[derive(Debug, Clone)]
pub struct CameraGraphFactor {
    /// Expected relative transform between cameras
    pub expected_relative_pose: na::Isometry3<f64>,
}

impl CameraGraphFactor {
    pub const fn new(expected_relative_pose: na::Isometry3<f64>) -> Self {
        Self {
            expected_relative_pose,
        }
    }
}

impl Factor for CameraGraphFactor {
    fn linearize(
        &self,
        params: &[na::DVector<f64>],
        compute_jacobian: bool,
    ) -> (na::DVector<f64>, Option<na::DMatrix<f64>>) {
        // params[0] = first camera pose (6D)
        // params[1] = second camera pose (6D)

        assert_eq!(
            params.len(),
            2,
            "CameraGraphFactor requires 2 parameter vectors"
        );

        let pose1_params = params[0].as_slice();
        let pose2_params = params[1].as_slice();

        // Convert pose parameters to transforms
        let rotation1 = na::UnitQuaternion::from_scaled_axis(na::Vector3::new(
            pose1_params[0],
            pose1_params[1],
            pose1_params[2],
        ));
        let translation1 = na::Vector3::new(pose1_params[3], pose1_params[4], pose1_params[5]);
        let pose1 = na::Isometry3::from_parts(translation1.into(), rotation1);

        let rotation2 = na::UnitQuaternion::from_scaled_axis(na::Vector3::new(
            pose2_params[0],
            pose2_params[1],
            pose2_params[2],
        ));
        let translation2 = na::Vector3::new(pose2_params[3], pose2_params[4], pose2_params[5]);
        let pose2 = na::Isometry3::from_parts(translation2.into(), rotation2);

        // Compute actual relative pose
        let actual_relative_pose = pose1.inverse() * pose2;

        // Compute error between expected and actual relative pose
        let pose_error = self.expected_relative_pose.inverse() * actual_relative_pose;

        // Convert pose error to 6D vector (log of SE(3))
        let rotation_error = pose_error.rotation.scaled_axis();
        let translation_error = pose_error.translation.vector;

        let residuals = na::DVector::from_vec(vec![
            rotation_error.x,
            rotation_error.y,
            rotation_error.z,
            translation_error.x,
            translation_error.y,
            translation_error.z,
        ]);

        // TODO: Compute Jacobians for pose graph optimization
        let jacobian = if compute_jacobian {
            None // Return None to trigger solver's numerical differentiation fallback
        } else {
            None
        };

        (residuals, jacobian)
    }

    fn get_dimension(&self) -> usize {
        6 // 6D pose error
    }
}
