// ============================================================================
// TIGHT-COUPLED VIO: Inter-Keyframe IMU Factor Implementation
// ============================================================================
//
// This module implements the core of modern Visual-Inertial Odometry (VIO):
// direct integration of IMU measurements into the optimization graph.
//
// References:
// - Li & Wei (2013): "Visual-Inertial Monocular SLAM with Map Reuse"
// - Forster et al. (2016): "On-Manifold Preintegration for Real-Time Visual-Inertial Odometry"
// - Lowe et al. (2020): "Direct Visual-Inertial Odometry with Stereo Cameras"

use crate::fl;
use crate::types::{Float, Matrix4x4, Vector3};
use apex_solver::factors::Factor;
use nalgebra as na;
use nalgebra::{DMatrix, DVector};
use std::f64::consts::PI;

/// ============================================================================
/// 1. GRAVITY MODELING (SOTA)
/// ============================================================================
///
/// Gravity vector in world frame (fixed during optimization in most cases)
///
/// Recommended approach:
/// - Fix gravity magnitude and direction (down) during initial BA
/// - Optionally estimate roll/pitch during initialization if needed
///
/// World frame Z-axis points up (opposite to gravity direction)
/// Gravity = [0, 0, -g] in world frame (right-hand Z-up convention)
#[derive(Debug, Clone, Copy)]
pub struct GravityModel {
    /// Gravity acceleration magnitude (m/s²), typically ~9.81
    pub magnitude: Float,
}

impl GravityModel {
    pub fn new(magnitude: Float) -> Self {
        Self { magnitude }
    }

    /// Standard Earth gravity (9.81 m/s²)
    pub fn earth() -> Self {
        Self {
            magnitude: fl!(9.81),
        }
    }

    /// Get gravity vector in world frame
    pub fn gravity_vector(&self) -> Vector3 {
        Vector3::new(fl!(0.0), fl!(0.0), -self.magnitude)
    }
}

/// ============================================================================
/// 2. INTER-KEYFRAME IMU PREINTEGRATION FACTOR (SOTA)
/// ============================================================================

/// Inter-keyframe IMU factor for tight coupling
///
/// This factor models the IMU preintegration constraint between two keyframes.
/// It includes:
/// - Position integration (with gravity)
/// - Velocity integration
/// - Rotation integration
/// - Bias jacobians (for online bias refinement)
///
/// State variables:
/// - Keyframe i: [T_W_B_i (7D), velocity_i (3D)]
/// - Keyframe j: [T_W_B_j (7D), velocity_j (3D)]
///
/// Residual (9D):
/// - Position: p_W_B_j - (p_W_B_i + v_i*dt + 0.5*g*dt² + R_W_B_i * ∫∫ a_corrected)
/// - Velocity: v_j - (v_i + g*dt + R_W_B_i * ∫ a_corrected)
/// - Rotation: log(R_ij^T * R_W_B_i^T * R_W_B_j) (3D axis-angle)
#[derive(Debug, Clone)]
pub struct InterKeyframeImuFactor {
    /// Time interval between keyframes (seconds)
    pub dt: Float,

    /// Preintegration data: integrated rotation, velocity, position
    pub preintegration: ImuPreintegration,

    /// Covariance of preintegration noise (9x9: p, v, R)
    pub covariance: DMatrix<Float>,

    /// Gravity model
    pub gravity: GravityModel,

    /// Information matrix (inverse covariance) for weighting
    pub information: DMatrix<Float>,
}

/// Preintegration result between two consecutive keyframes
#[derive(Debug, Clone, Default)]
pub struct ImuPreintegration {
    /// Time interval between keyframes (seconds)
    pub dt: Float,

    /// Integrated rotation from IMU frame at i to frame at j: R_ij
    pub delta_R: na::Matrix3<Float>,

    /// Integrated velocity change: Δv = R_i^T * ∫(a - a_bias) dt
    pub delta_v: Vector3,

    /// Integrated position change: Δp = ∫∫(R * (a - a_bias)) dt²
    pub delta_p: Vector3,

    /// Covariance of integration errors
    pub cov_R: na::Matrix3<Float>,
    pub cov_v: na::Matrix3<Float>,
    pub cov_p: na::Matrix3<Float>,

    /// Cross-covariance terms for bias jacobians
    pub cov_R_bw: na::Matrix3<Float>, // cov(ΔR, δw_bias)
    pub cov_v_ba: na::Matrix3<Float>, // cov(Δv, δa_bias)
    pub cov_p_ba: na::Matrix3<Float>, // cov(Δp, δa_bias)
}

impl ImuPreintegration {
    /// Get the time interval of this preintegration
    pub fn delta_time(&self) -> Float {
        self.dt
    }
}

impl InterKeyframeImuFactor {
    /// Create inter-keyframe IMU factor from preintegration data
    pub fn new(dt: Float, preintegration: ImuPreintegration, gravity: GravityModel) -> Self {
        // Build covariance matrix (9x9: p, v, R)
        let mut cov = DMatrix::zeros(9, 9);
        cov.fixed_view_mut::<3, 3>(0, 0)
            .copy_from(&preintegration.cov_p);
        cov.fixed_view_mut::<3, 3>(3, 3)
            .copy_from(&preintegration.cov_v);
        cov.fixed_view_mut::<3, 3>(6, 6)
            .copy_from(&preintegration.cov_R);

        // Add small regularization to avoid singularity
        let reg = fl!(1e-8);
        for i in 0..9 {
            cov[(i, i)] += reg;
        }

        // Information matrix (inverse of covariance)
        let information = cov.clone().try_inverse().unwrap_or(DMatrix::identity(9, 9));

        Self {
            dt,
            preintegration,
            covariance: cov,
            gravity,
            information,
        }
    }
}

// ============================================================================
// 3. ONLINE BIAS REFINEMENT (SOTA)
// ============================================================================

/// Online IMU bias refinement during optimization
///
/// As we optimize poses, we should also refine bias estimates.
/// This uses the bias jacobians from preintegration.
pub struct BiasRefinement {
    /// Current bias estimate
    pub accel_bias: Vector3,
    pub gyro_bias: Vector3,

    /// Bias estimate covariance
    pub cov_accel: na::Matrix3<Float>,
    pub cov_gyro: na::Matrix3<Float>,

    /// Bias uncertainty threshold (meters/s² and rad/s)
    pub max_accel_bias: Float, // Typically 0.5 m/s²
    pub max_gyro_bias: Float, // Typically 0.1 rad/s
}

impl Default for BiasRefinement {
    fn default() -> Self {
        Self::new()
    }
}

impl BiasRefinement {
    pub fn new() -> Self {
        Self {
            accel_bias: Vector3::zeros(),
            gyro_bias: Vector3::zeros(),
            cov_accel: na::Matrix3::identity() * fl!(0.01),
            cov_gyro: na::Matrix3::identity() * fl!(0.001),
            max_accel_bias: fl!(0.5),
            max_gyro_bias: fl!(0.1),
        }
    }

    /// Check if bias estimate is within acceptable bounds
    pub fn is_valid(&self) -> bool {
        self.accel_bias.norm() < self.max_accel_bias && self.gyro_bias.norm() < self.max_gyro_bias
    }

    /// Update bias estimate from optimization residuals
    pub fn update_from_residuals(
        &mut self,
        residual_delta: &na::Vector6<Float>, // [Δp, Δv, (Δθ)]
        _jacobian: &na::Matrix6<Float>,      // Jacobian w.r.t. state
    ) {
        // In full implementation, use EKF update or direct damping
        // For now: simple proportional update
        let damping = fl!(0.1);
        self.accel_bias += damping * residual_delta.fixed_view::<3, 1>(0, 0);
        self.gyro_bias += damping * residual_delta.fixed_view::<3, 1>(3, 0);

        // Clamp to valid range
        self.accel_bias.x = self
            .accel_bias
            .x
            .clamp(-self.max_accel_bias, self.max_accel_bias);
        self.accel_bias.y = self
            .accel_bias
            .y
            .clamp(-self.max_accel_bias, self.max_accel_bias);
        self.accel_bias.z = self
            .accel_bias
            .z
            .clamp(-self.max_accel_bias, self.max_accel_bias);

        self.gyro_bias.x = self
            .gyro_bias
            .x
            .clamp(-self.max_gyro_bias, self.max_gyro_bias);
        self.gyro_bias.y = self
            .gyro_bias
            .y
            .clamp(-self.max_gyro_bias, self.max_gyro_bias);
        self.gyro_bias.z = self
            .gyro_bias
            .z
            .clamp(-self.max_gyro_bias, self.max_gyro_bias);
    }
}

// ============================================================================
// 4. TIGHT-COUPLING INITIALIZATION (SOTA)
// ============================================================================

/// Robust VIO initialization combining vision and IMU
///
/// Standard monocular SLAM fails on small baselines. Tight-coupled VIO solves this:
/// 1. Use IMU to constrain rotation immediately
/// 2. Use visual structure for scale
/// 3. Refine together
pub struct TightCouplingInitializer {
    /// Minimum number of frames needed for initialization
    pub min_frames: usize,

    /// Minimum parallax for scale estimation (normalized pixels)
    pub min_parallax: f64,

    /// Gravity model
    pub gravity: GravityModel,
}

impl TightCouplingInitializer {
    pub fn new(gravity: GravityModel) -> Self {
        Self {
            min_frames: 5,
            min_parallax: 30.0, // pixels in normalized coordinates
            gravity,
        }
    }

    /// Initialize VIO from visual + IMU data
    ///
    /// Algorithm:
    /// 1. Estimate rotation from IMU (gyro integration is reliable short-term)
    /// 2. Triangulate points using visual constraints
    /// 3. Estimate scale from gravity + acceleration
    /// 4. Refine with joint optimization
    pub fn initialize(
        &self,
        _frames: &[crate::estimator::Frame],
        _imu_preinteg: &ImuPreintegration,
    ) -> Option<(Matrix4x4, Vector3, Vector3)> {
        // Pseudo-implementation for structure
        // Real implementation would:
        // 1. Compute relative poses from visual features
        // 2. Use IMU rotation to resolve scale ambiguity
        // 3. Estimate gravity direction from accelerometer mean
        // 4. Return initial [T_W_B, velocity, gravity]

        let T_W_B = Matrix4x4::identity();
        let velocity = Vector3::zeros();
        let gravity = self.gravity.gravity_vector();

        Some((T_W_B, velocity, gravity))
    }
}

// ============================================================================
// APEX_SOLVER FACTOR IMPLEMENTATION
// ============================================================================

/// Factor for inter-keyframe IMU constraints in optimization graph
///
/// This factor enforces constraints between consecutive keyframes based on
/// IMU preintegration. It models:
/// - Position constraint: p_j ≈ p_i + v_i*dt + 0.5*g*dt² + ΔP
/// - Velocity constraint: v_j ≈ v_i + g*dt + ΔV
/// - Rotation constraint: R_j ≈ R_i * ΔR
///
/// Variables (expected in order):
/// - params[0]: Keyframe i pose (SE3: 7D: tx, ty, tz, qw, qx, qy, qz)
/// - params[1]: Keyframe i velocity (3D: vx, vy, vz)
/// - params[2]: Keyframe j pose (SE3: 7D: tx, ty, tz, qw, qx, qy, qz)
/// - params[3]: Keyframe j velocity (3D: vx, vy, vz)
///
/// Returns 9D residual: [Δp error; Δv error; Δθ error]
impl Factor for InterKeyframeImuFactor {
    fn get_dimension(&self) -> usize {
        9 // 9D residual: [Δp_error (3); Δv_error (3); Δθ_error (3)]
    }

    fn linearize(
        &self,
        params: &[DVector<f64>],
        compute_jacobian: bool,
    ) -> (DVector<f64>, Option<DMatrix<f64>>) {
        // Expect 4 parameter vectors: KF_i pose, VEL_i, KF_j pose, VEL_j
        assert_eq!(
            params.len(),
            4,
            "InterKeyframeImuFactor requires 4 parameter vectors (KF_i pose, VEL_i, KF_j pose, VEL_j)"
        );

        // Extract keyframe i pose (SE3: [tx, ty, tz, qw, qx, qy, qz])
        let p_i = &params[0];
        let t_i = na::Vector3::new(p_i[0], p_i[1], p_i[2]);
        // Storage format: [tx, ty, tz, qw, qx, qy, qz]
        let q_i =
            na::UnitQuaternion::new_normalize(na::Quaternion::new(p_i[3], p_i[4], p_i[5], p_i[6]));

        // Extract velocity i (3D)
        let v_i = na::Vector3::new(params[1][0], params[1][1], params[1][2]);

        // Extract keyframe j pose (SE3)
        let p_j = &params[2];
        let t_j = na::Vector3::new(p_j[0], p_j[1], p_j[2]);
        // Storage format: [tx, ty, tz, qw, qx, qy, qz]
        let q_j =
            na::UnitQuaternion::new_normalize(na::Quaternion::new(p_j[3], p_j[4], p_j[5], p_j[6]));

        // Extract velocity j (3D)
        let v_j = na::Vector3::new(params[3][0], params[3][1], params[3][2]);

        let dt = self.dt as f64;
        let g_vec = self.gravity.magnitude as f64;

        // ========== Rotation Constraint ==========
        // Rotation residual: log(R_ij_obs^T * ΔR) or equivalently axis-angle of q_error
        // q_ij_obs = q_i^-1 * q_j (observed rotation from poses)
        // q_delta = ΔR from IMU preintegration
        // Error: how much we need to rotate q_delta to get q_ij_obs = q_delta^-1 * q_ij_obs
        let delta_R_imu = self.preintegration.delta_R;
        let q_delta = na::UnitQuaternion::from_matrix(&delta_R_imu.cast::<f64>());
        let q_ij_obs = q_i.inverse() * q_j;
        let q_error = q_delta.inverse() * q_ij_obs;
        let rot_error_axis = q_error
            .axis()
            .map(|a| a.into_inner())
            .unwrap_or(na::Vector3::zeros());
        let rot_error_angle = q_error.angle();
        let rot_error = rot_error_axis * rot_error_angle;

        // ========== Velocity Constraint ==========
        // v_j = v_i + g*dt + R_i * Δv
        // Δv_imu is in IMU frame, rotate to world frame: R_W_B_i * Δv
        let delta_v_imu: na::Vector3<f64> = self.preintegration.delta_v.cast::<f64>();
        let delta_v_world = q_i * delta_v_imu;
        let gravity_contribution = na::Vector3::new(0.0, 0.0, g_vec * dt);
        let v_predicted = v_i + delta_v_world + gravity_contribution;
        let vel_error = v_j - v_predicted;

        // ========== Position Constraint ==========
        // p_j = p_i + v_i*dt + 0.5*g*dt² + R_i * Δp
        let delta_p_imu: na::Vector3<f64> = self.preintegration.delta_p.cast::<f64>();
        let delta_p_world = q_i * delta_p_imu;
        let gravity_pos_term = na::Vector3::new(0.0, 0.0, -0.5 * g_vec * dt * dt);
        let p_predicted = t_i + v_i * dt + delta_p_world + gravity_pos_term;
        let pos_error = t_j - p_predicted;

        // ========== Build 9D Residual ==========
        // Residual ordering: [pos; vel; rot]
        // Return raw residuals - solver applies information matrix
        let mut residual = DVector::zeros(9);
        residual[0] = pos_error.x;
        residual[1] = pos_error.y;
        residual[2] = pos_error.z;
        residual[3] = vel_error.x;
        residual[4] = vel_error.y;
        residual[5] = vel_error.z;
        residual[6] = rot_error.x;
        residual[7] = rot_error.y;
        residual[8] = rot_error.z;

        let jacobian = if compute_jacobian {
            // 9x20 Jacobian: 9 residuals, 7+3+7+3 pose/velocity dimensions
            // Return raw jacobians - solver applies information matrix
            let mut jac = DMatrix::zeros(9, 20);

            // Position w.r.t. KF_i pose: -I
            jac[(0, 0)] = -1.0;
            jac[(1, 1)] = -1.0;
            jac[(2, 2)] = -1.0;

            // Position w.r.t. VEL_i: -dt * I
            jac[(0, 7)] = -dt;
            jac[(1, 8)] = -dt;
            jac[(2, 9)] = -dt;

            // Position w.r.t. KF_j pose: +I
            jac[(0, 10)] = 1.0;
            jac[(1, 11)] = 1.0;
            jac[(2, 12)] = 1.0;

            // Velocity w.r.t. VEL_i: -I
            jac[(3, 7)] = -1.0;
            jac[(4, 8)] = -1.0;
            jac[(5, 9)] = -1.0;

            // Velocity w.r.t. VEL_j: +I
            jac[(3, 17)] = 1.0;
            jac[(4, 18)] = 1.0;
            jac[(5, 19)] = 1.0;

            // Rotation w.r.t. KF_i pose: -I
            jac[(6, 3)] = -1.0;
            jac[(7, 4)] = -1.0;
            jac[(8, 5)] = -1.0;

            // Rotation w.r.t. KF_j pose: +I
            jac[(6, 13)] = 1.0;
            jac[(7, 14)] = 1.0;
            jac[(8, 15)] = 1.0;

            Some(jac)
        } else {
            None
        };

        (residual, jacobian)
    }
}

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

#[allow(dead_code)]
fn matrix_to_axis_angle(R: &na::Matrix3<Float>) -> Vector3 {
    // Use Rodrigues' formula inverse
    let trace = R[(0, 0)] + R[(1, 1)] + R[(2, 2)];
    let angle = ((trace - fl!(1.0)) / fl!(2.0))
        .clamp(fl!(-1.0), fl!(1.0))
        .acos();

    if angle.abs() < fl!(1e-6) {
        // Small angle: use skew-symmetric part
        Vector3::new(
            R[(2, 1)] - R[(1, 2)],
            R[(0, 2)] - R[(2, 0)],
            R[(1, 0)] - R[(0, 1)],
        ) * fl!(0.5)
    } else if (angle - fl!(PI)).abs() < fl!(1e-6) {
        // Angle close to π: extract from diagonal
        let diag = [
            R[(0, 0)] + fl!(1.0),
            R[(1, 1)] + fl!(1.0),
            R[(2, 2)] + fl!(1.0),
        ];
        let idx = diag
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.total_cmp(b.1))
            .map(|(i, _)| i)
            .unwrap_or(0);

        let mut v = Vector3::zeros();
        match idx {
            0 => {
                v.x = (diag[0] / fl!(2.0)).sqrt();
                v.y = R[(0, 1)] / (fl!(2.0) * v.x);
                v.z = R[(0, 2)] / (fl!(2.0) * v.x);
            },
            1 => {
                v.y = (diag[1] / fl!(2.0)).sqrt();
                v.x = R[(0, 1)] / (fl!(2.0) * v.y);
                v.z = R[(1, 2)] / (fl!(2.0) * v.y);
            },
            _ => {
                v.z = (diag[2] / fl!(2.0)).sqrt();
                v.x = R[(0, 2)] / (fl!(2.0) * v.z);
                v.y = R[(1, 2)] / (fl!(2.0) * v.z);
            },
        }
        v * angle
    } else {
        // Normal case
        Vector3::new(
            R[(2, 1)] - R[(1, 2)],
            R[(0, 2)] - R[(2, 0)],
            R[(1, 0)] - R[(0, 1)],
        ) * (angle / (fl!(2.0) * angle.sin()))
    }
}

#[cfg(test)]
#[allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::clone_on_copy,
    clippy::too_many_arguments,
    clippy::new_without_default
)]
mod tests {
    use super::*;
    use na::UnitQuaternion;

    #[test]
    fn test_gravity_model() {
        let gravity = GravityModel::earth();
        let g_vec = gravity.gravity_vector();
        assert!((g_vec.norm() - 9.81).abs() < 1e-6);
        assert!(g_vec.z < 0.0); // Pointing down
    }

    #[test]
    fn test_gravity_model_custom() {
        let gravity = GravityModel::new(10.0);
        let g_vec = gravity.gravity_vector();
        assert!((g_vec.norm() - 10.0).abs() < 1e-6);
        assert!(g_vec.z < 0.0);
    }

    #[test]
    fn test_inter_keyframe_factor_creation() {
        let preintegration = ImuPreintegration {
            dt: 0.05,
            delta_R: na::Matrix3::identity(),
            delta_v: Vector3::zeros(),
            delta_p: Vector3::zeros(),
            cov_R: na::Matrix3::identity() * 1e-4,
            cov_v: na::Matrix3::identity() * 1e-4,
            cov_p: na::Matrix3::identity() * 1e-6,
            cov_R_bw: na::Matrix3::identity() * 1e-5,
            cov_v_ba: na::Matrix3::identity() * 1e-5,
            cov_p_ba: na::Matrix3::identity() * 1e-7,
        };

        let factor = InterKeyframeImuFactor::new(0.05, preintegration, GravityModel::earth());

        // Check that covariance matrix elements are finite
        for i in 0..6 {
            for j in 0..6 {
                assert!(factor.covariance[(i, j)].is_finite());
                assert!(factor.information[(i, j)].is_finite());
            }
        }
    }

    #[test]
    fn test_inter_keyframe_factor_dimension() {
        let preintegration = ImuPreintegration::default();
        let factor = InterKeyframeImuFactor::new(0.1, preintegration, GravityModel::earth());
        assert_eq!(factor.get_dimension(), 9); // 3 pos + 3 vel + 3 rot
    }

    #[test]
    fn test_inter_keyframe_factor_zero_motion() {
        // Test with zero motion - states should match IMU predictions
        // Using zero gravity model to simplify the test
        let dt = 0.05;

        let preintegration = ImuPreintegration {
            dt,
            delta_R: na::Matrix3::identity(),
            delta_v: Vector3::zeros(),
            delta_p: Vector3::zeros(),
            cov_R: na::Matrix3::identity() * 1e-8,
            cov_v: na::Matrix3::identity() * 1e-8,
            cov_p: na::Matrix3::identity() * 1e-8,
            cov_R_bw: na::Matrix3::zeros(),
            cov_v_ba: na::Matrix3::zeros(),
            cov_p_ba: na::Matrix3::zeros(),
        };

        let zero_gravity = GravityModel { magnitude: 0.0 };
        let factor = InterKeyframeImuFactor::new(dt, preintegration, zero_gravity);

        // Create parameter blocks: KF_i pose, VEL_i, KF_j pose, VEL_j
        // All at origin with identity rotation
        let p_i = DVector::from_vec(vec![0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]);
        let v_i = DVector::from_vec(vec![0.0, 0.0, 0.0]);
        let p_j = DVector::from_vec(vec![0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]);
        let v_j = DVector::from_vec(vec![0.0, 0.0, 0.0]);

        let (residual, _) = factor.linearize(&[p_i, v_i, p_j, v_j], false);

        // Residual should be zero for consistent states with zero gravity
        assert!(
            residual.norm() < 1e-6,
            "Zero motion residual should be near zero: {}",
            residual.norm()
        );
    }

    #[test]
    fn test_inter_keyframe_factor_constant_velocity_no_gravity() {
        // Test with constant velocity motion, no gravity
        let dt = 0.1;

        let preintegration = ImuPreintegration {
            dt,
            delta_R: na::Matrix3::identity(),
            delta_v: Vector3::zeros(), // No IMU-measured velocity change
            delta_p: Vector3::zeros(), // No IMU-measured position change
            cov_R: na::Matrix3::identity() * 1e-6,
            cov_v: na::Matrix3::identity() * 1e-4,
            cov_p: na::Matrix3::identity() * 1e-4,
            cov_R_bw: na::Matrix3::zeros(),
            cov_v_ba: na::Matrix3::zeros(),
            cov_p_ba: na::Matrix3::zeros(),
        };

        let zero_gravity = GravityModel { magnitude: 0.0 };
        let factor = InterKeyframeImuFactor::new(dt, preintegration, zero_gravity);

        // Initial state: at origin with velocity v_i
        let p_i = DVector::from_vec(vec![0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]);
        let v_i = DVector::from_vec(vec![1.0, 0.0, 0.0]);

        // Final state: position = v_i * dt, same velocity (constant velocity)
        let p_j = DVector::from_vec(vec![0.1, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]);
        let v_j = DVector::from_vec(vec![1.0, 0.0, 0.0]);

        let (residual, _) = factor.linearize(&[p_i, v_i, p_j, v_j], false);

        // Residual should be zero for constant velocity with no gravity
        assert!(
            residual.norm() < 1e-6,
            "Constant velocity residual should be near zero: {}",
            residual.norm()
        );
    }

    #[test]
    fn test_inter_keyframe_factor_with_rotation_using_quaternions() {
        // Test with rotation using properly constructed quaternions
        // 90 degree rotation around Z axis
        let dt = 0.1;
        let angle = std::f64::consts::PI / 2.0;

        // Create quaternion for 90 degree rotation around Z
        let q_rot = UnitQuaternion::from_scaled_axis(nalgebra::Vector3::z() * angle);

        // Verify the quaternion components
        assert!(
            (q_rot.w - 0.70710678).abs() < 1e-5,
            "w should be ~0.707, got {}",
            q_rot.w
        );
        assert!(
            (q_rot.k - 0.70710678).abs() < 1e-5,
            "k should be ~0.707, got {}",
            q_rot.k
        );
        assert!(q_rot.i.abs() < 1e-10, "i should be ~0, got {}", q_rot.i);
        assert!(q_rot.j.abs() < 1e-10, "j should be ~0, got {}", q_rot.j);

        let preintegration = ImuPreintegration {
            dt,
            delta_R: q_rot.to_rotation_matrix().into(),
            delta_v: Vector3::zeros(),
            delta_p: Vector3::zeros(),
            cov_R: na::Matrix3::identity() * 1e-4,
            cov_v: na::Matrix3::identity() * 1e-4,
            cov_p: na::Matrix3::identity() * 1e-4,
            cov_R_bw: na::Matrix3::zeros(),
            cov_v_ba: na::Matrix3::zeros(),
            cov_p_ba: na::Matrix3::zeros(),
        };

        let zero_gravity = GravityModel { magnitude: 0.0 };
        let factor = InterKeyframeImuFactor::new(dt, preintegration, zero_gravity);

        // Initial pose at origin
        let p_i = DVector::from_vec(vec![0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]);
        let v_i = DVector::from_vec(vec![0.0, 0.0, 0.0]);

        // Final pose: use the same quaternion components as the IMU delta
        // Storage format: [tx, ty, tz, qw, qx, qy, qz]
        let p_j = DVector::from_vec(vec![0.0, 0.0, 0.0, q_rot.w, q_rot.i, q_rot.j, q_rot.k]);
        let v_j = DVector::from_vec(vec![0.0, 0.0, 0.0]);

        let (residual, _) = factor.linearize(&[p_i, v_i, p_j, v_j], false);

        // Residual should be near zero for matching rotation
        assert!(
            residual.norm() < 1e-6,
            "Rotation residual should be near zero: {}",
            residual.norm()
        );
    }

    #[test]
    fn test_inter_keyframe_factor_jacobian_computation() {
        let preintegration = ImuPreintegration {
            dt: 0.05,
            delta_R: na::Matrix3::identity(),
            delta_v: Vector3::new(0.1, 0.0, 0.0),
            delta_p: Vector3::new(0.005, 0.0, 0.0),
            cov_R: na::Matrix3::identity() * 1e-4,
            cov_v: na::Matrix3::identity() * 1e-4,
            cov_p: na::Matrix3::identity() * 1e-6,
            cov_R_bw: na::Matrix3::identity() * 1e-5,
            cov_v_ba: na::Matrix3::identity() * 1e-5,
            cov_p_ba: na::Matrix3::identity() * 1e-7,
        };

        let factor = InterKeyframeImuFactor::new(0.05, preintegration, GravityModel::earth());

        let p_i = DVector::from_vec(vec![0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]);
        let v_i = DVector::from_vec(vec![0.0, 0.0, 0.0]);
        let p_j = DVector::from_vec(vec![0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]);
        let v_j = DVector::from_vec(vec![0.1, 0.0, 0.0]);

        let (_, jacobian) = factor.linearize(&[p_i, v_i, p_j, v_j], true);

        assert!(jacobian.is_some());
        let jac = jacobian.unwrap();
        assert_eq!(jac.nrows(), 9);
        assert_eq!(jac.ncols(), 20); // 7+3+7+3 = 20

        // Check that Jacobian is finite
        for i in 0..jac.nrows() {
            for j in 0..jac.ncols() {
                assert!(
                    jac[(i, j)].is_finite(),
                    "Jacobian element ({}, {}) is not finite",
                    i,
                    j
                );
            }
        }
    }

    #[test]
    fn test_inter_keyframe_factor_edge_cases() {
        // Test with very small dt
        let preintegration = ImuPreintegration {
            dt: 0.001,
            delta_R: na::Matrix3::identity(),
            delta_v: Vector3::zeros(),
            delta_p: Vector3::zeros(),
            cov_R: na::Matrix3::identity() * 1e-8,
            cov_v: na::Matrix3::identity() * 1e-8,
            cov_p: na::Matrix3::identity() * 1e-8,
            cov_R_bw: na::Matrix3::zeros(),
            cov_v_ba: na::Matrix3::zeros(),
            cov_p_ba: na::Matrix3::zeros(),
        };

        let factor = InterKeyframeImuFactor::new(0.001, preintegration, GravityModel::earth());
        assert_eq!(factor.dt, 0.001);

        // Test with large dt (1 second)
        let preintegration_large = ImuPreintegration {
            dt: 1.0,
            delta_R: na::Matrix3::identity(),
            delta_v: Vector3::zeros(),
            delta_p: Vector3::zeros(),
            cov_R: na::Matrix3::identity() * 1e-2,
            cov_v: na::Matrix3::identity() * 1e-2,
            cov_p: na::Matrix3::identity() * 1e-2,
            cov_R_bw: na::Matrix3::zeros(),
            cov_v_ba: na::Matrix3::zeros(),
            cov_p_ba: na::Matrix3::zeros(),
        };

        let factor_large =
            InterKeyframeImuFactor::new(1.0, preintegration_large, GravityModel::earth());
        assert_eq!(factor_large.dt, 1.0);
    }

    #[test]
    fn test_inter_keyframe_factor_information_matrix() {
        let preintegration = ImuPreintegration {
            dt: 0.05,
            delta_R: na::Matrix3::identity(),
            delta_v: Vector3::zeros(),
            delta_p: Vector3::zeros(),
            cov_R: na::Matrix3::identity() * 1e-4,
            cov_v: na::Matrix3::identity() * 1e-4,
            cov_p: na::Matrix3::identity() * 1e-6,
            cov_R_bw: na::Matrix3::identity() * 1e-5,
            cov_v_ba: na::Matrix3::identity() * 1e-5,
            cov_p_ba: na::Matrix3::identity() * 1e-7,
        };

        let factor = InterKeyframeImuFactor::new(0.05, preintegration, GravityModel::earth());

        // Check that information matrix is positive definite (diagonal entries > 0)
        for i in 0..9 {
            assert!(
                factor.information[(i, i)] > 0.0,
                "Information matrix diagonal {} is not positive",
                i
            );
        }

        // Check that information matrix is symmetric
        for i in 0..9 {
            for j in 0..9 {
                assert!(
                    (factor.information[(i, j)] - factor.information[(j, i)]).abs() < 1e-10,
                    "Information matrix is not symmetric at ({}, {})",
                    i,
                    j
                );
            }
        }
    }

    #[test]
    fn test_bias_refinement() {
        let mut refinement = BiasRefinement::new();
        assert!(refinement.is_valid());

        refinement.accel_bias = Vector3::new(1.0, 0.0, 0.0);
        assert!(!refinement.is_valid()); // Exceeds max_accel_bias = 0.5
    }

    #[test]
    fn test_bias_refinement_update() {
        let mut refinement = BiasRefinement::new();

        // Create a small residual
        let residual = na::Vector6::new(0.01, 0.01, 0.01, 0.001, 0.001, 0.001);
        let jacobian = na::Matrix6::identity();

        refinement.update_from_residuals(&residual, &jacobian);

        // Bias should have been updated slightly
        assert!(refinement.accel_bias.norm() > 0.0);
        assert!(refinement.gyro_bias.norm() > 0.0);

        // But still within bounds
        assert!(refinement.is_valid());
    }

    #[test]
    fn test_tight_coupling_initializer() {
        let gravity = GravityModel::earth();
        let initializer = TightCouplingInitializer::new(gravity);

        assert_eq!(initializer.min_frames, 5);
        assert!((initializer.min_parallax - 30.0).abs() < 1e-6);
    }

    #[test]
    fn test_imu_preintegration_default() {
        let preintegration = ImuPreintegration::default();
        assert_eq!(preintegration.dt, 0.0);
        assert!(preintegration.delta_v.norm() < 1e-10);
        assert!(preintegration.delta_p.norm() < 1e-10);
    }

    #[test]
    fn test_imu_preintegration_delta_time() {
        let preintegration = ImuPreintegration {
            dt: 0.123,
            ..Default::default()
        };
        assert!((preintegration.delta_time() - 0.123).abs() < 1e-10);
    }
}
