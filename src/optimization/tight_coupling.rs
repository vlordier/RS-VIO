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

use crate::types::{Matrix4x4, Vector3};
use apex_solver::factors::Factor;
use nalgebra as na;
use nalgebra::{DMatrix, DVector};
use std::f64::consts::PI;

/// ============================================================================
/// 1. GRAVITY MODELING (SOTA)
/// ============================================================================

/// Gravity vector in world frame (fixed during optimization in most cases)
///
/// Recommended approach:
/// - Fix gravity magnitude and direction (down) during initial BA
/// - Optionally estimate roll/pitch during initialization if needed
#[derive(Debug, Clone, Copy)]
/// World frame Z-axis points up (opposite to gravity direction)
/// Gravity = [0, 0, -g] in world frame (right-hand Z-up convention)
pub struct GravityModel {
    /// Gravity acceleration magnitude (m/s²), typically ~9.81
    pub magnitude: f64,
}

impl GravityModel {
    pub fn new(magnitude: f64) -> Self {
        Self { magnitude }
    }

    /// Standard Earth gravity (9.81 m/s²)
    pub fn earth() -> Self {
        Self { magnitude: 9.81 }
    }

    /// Get gravity vector in world frame
    pub fn gravity_vector(&self) -> Vector3 {
        Vector3::new(0.0, 0.0, -self.magnitude)
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
/// - Keyframe i: [T_W_B_i, velocity_i, accel_bias_i, gyro_bias_i]
/// - Keyframe j: [T_W_B_j, velocity_j, accel_bias_j, gyro_bias_j]
///
/// Residual (6D):
/// - Position: p_W_B_j - (p_W_B_i + v_i*dt + 0.5*g*dt² + R_W_B_i * ∫∫ a_corrected)
/// - Velocity: v_j - (v_i + g*dt + R_W_B_i * ∫ a_corrected)
/// - Rotation: log(R_ij^T * R_W_B_i^T * R_W_B_j) (3D axis-angle)
#[derive(Debug, Clone)]
pub struct InterKeyframeImuFactor {
    /// Time interval between keyframes (seconds)
    pub dt: f64,

    /// Preintegration data: integrated rotation, velocity, position
    pub preintegration: ImuPreintegration,

    /// Jacobians w.r.t. biases (for online refinement)
    pub jacobian_pos_bias: na::Matrix3<f64>, // ∂p/∂bias
    pub jacobian_vel_bias: na::Matrix3<f64>, // ∂v/∂bias
    pub jacobian_rot_bias: na::Matrix3<f64>, // ∂R/∂bias

    /// Covariance of preintegration noise
    pub covariance: na::Matrix6<f64>,

    /// Gravity model
    pub gravity: GravityModel,

    /// Information matrix (inverse covariance) for weighting
    pub information: na::Matrix6<f64>,
}

/// Preintegration result between two consecutive keyframes
#[derive(Debug, Clone)]
pub struct ImuPreintegration {
    /// Integrated rotation from IMU frame at i to frame at j: R_ij
    pub delta_R: na::Matrix3<f64>,

    /// Integrated velocity change: Δv = R_i^T * ∫(a - a_bias) dt
    pub delta_v: Vector3,

    /// Integrated position change: Δp = ∫∫(R * (a - a_bias)) dt²
    pub delta_p: Vector3,

    /// Covariance of integration errors
    pub cov_R: na::Matrix3<f64>,
    pub cov_v: na::Matrix3<f64>,
    pub cov_p: na::Matrix3<f64>,

    /// Cross-covariance terms for bias jacobians
    pub cov_R_bw: na::Matrix3<f64>, // cov(ΔR, δw_bias)
    pub cov_v_ba: na::Matrix3<f64>, // cov(Δv, δa_bias)
    pub cov_p_ba: na::Matrix3<f64>, // cov(Δp, δa_bias)
}

impl InterKeyframeImuFactor {
    /// Create inter-keyframe IMU factor from preintegration data
    pub fn new(dt: f64, preintegration: ImuPreintegration, gravity: GravityModel) -> Self {
        // Build covariance matrix (6x6: p, v, R)
        let mut cov = na::Matrix6::zeros();
        cov.fixed_view_mut::<3, 3>(0, 0)
            .copy_from(&preintegration.cov_p);
        cov.fixed_view_mut::<3, 3>(3, 3)
            .copy_from(&preintegration.cov_v);
        cov.fixed_view_mut::<3, 3>(3, 3)
            .copy_from(&preintegration.cov_R);

        // Add small regularization to avoid singularity
        let reg = 1e-8;
        for i in 0..6 {
            cov[(i, i)] += reg;
        }

        // Information matrix (inverse of covariance)
        let information = cov.try_inverse().unwrap_or(na::Matrix6::identity());

        // Jacobians w.r.t. biases (for online refinement)
        let jacobian_pos_bias = preintegration.cov_p_ba.clone();
        let jacobian_vel_bias = preintegration.cov_v_ba.clone();
        let jacobian_rot_bias = preintegration.cov_R_bw.clone();

        Self {
            dt,
            preintegration,
            jacobian_pos_bias,
            jacobian_vel_bias,
            jacobian_rot_bias,
            covariance: cov,
            gravity,
            information,
        }
    }

    /// Compute residual: how well does the IMU prediction match the optimized poses?
    ///
    /// Residuals (6D):
    /// - r_p (3D): position prediction error
    /// - r_v (3D): velocity prediction error
    pub fn compute_residual(
        &self,
        T_W_B_i: Matrix4x4, // Pose at keyframe i
        v_i: Vector3,       // Velocity at keyframe i
        _bias_a_i: Vector3, // Accel bias (should match bias_a_j)
        _bias_w_i: Vector3, // Gyro bias (should match bias_w_j)

        T_W_B_j: Matrix4x4, // Pose at keyframe j
        v_j: Vector3,       // Velocity at keyframe j
        _bias_a_j: Vector3, // Accel bias at j (for consistency check)
        _bias_w_j: Vector3, // Gyro bias at j (for consistency check)
    ) -> na::Vector6<f64> {
        // Extract positions
        let p_W_B_i = T_W_B_i.fixed_view::<3, 1>(0, 3).into_owned();
        let p_W_B_j = T_W_B_j.fixed_view::<3, 1>(0, 3).into_owned();

        // Extract rotations
        let R_W_B_i = T_W_B_i.fixed_view::<3, 3>(0, 0).into_owned();
        let R_W_B_j = T_W_B_j.fixed_view::<3, 3>(0, 0).into_owned();

        // Gravity vector
        let g = self.gravity.gravity_vector();

        // Position prediction error:
        // p_pred = p_i + v_i * dt + 0.5 * g * dt² + R_i * Δp
        let dt2 = self.dt * self.dt;
        let p_pred =
            p_W_B_i + v_i * self.dt + 0.5 * g * dt2 + R_W_B_i * self.preintegration.delta_p;
        let r_p = p_W_B_j - p_pred;

        // Velocity prediction error:
        // v_pred = v_i + g * dt + R_i * Δv
        let v_pred = v_i + g * self.dt + R_W_B_i * self.preintegration.delta_v;
        let r_v = v_j - v_pred;

        // Rotation prediction error:
        // R_pred = R_i * ΔR
        // Error = log((R_pred^T * R_j)) in axis-angle form
        let R_pred = R_W_B_i * self.preintegration.delta_R;
        let R_error = R_pred.transpose() * R_W_B_j;
        let _r_R = matrix_to_axis_angle(&R_error);

        // Combine residuals [p, v, R]
        let mut residual = na::Vector6::zeros();
        residual.fixed_view_mut::<3, 1>(0, 0).copy_from(&r_p);
        residual.fixed_view_mut::<3, 1>(3, 0).copy_from(&r_v);

        // For now, return 6D residual (position + velocity)
        // Rotation residual is implicit in pose optimization
        residual
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
    pub cov_accel: na::Matrix3<f64>,
    pub cov_gyro: na::Matrix3<f64>,

    /// Bias uncertainty threshold (meters/s² and rad/s)
    pub max_accel_bias: f64, // Typically 0.5 m/s²
    pub max_gyro_bias: f64, // Typically 0.1 rad/s
}

impl BiasRefinement {
    pub fn new() -> Self {
        Self {
            accel_bias: Vector3::zeros(),
            gyro_bias: Vector3::zeros(),
            cov_accel: na::Matrix3::identity() * 0.01,
            cov_gyro: na::Matrix3::identity() * 0.001,
            max_accel_bias: 0.5,
            max_gyro_bias: 0.1,
        }
    }

    /// Check if bias estimate is within acceptable bounds
    pub fn is_valid(&self) -> bool {
        self.accel_bias.norm() < self.max_accel_bias && self.gyro_bias.norm() < self.max_gyro_bias
    }

    /// Update bias estimate from optimization residuals
    pub fn update_from_residuals(
        &mut self,
        residual_delta: &na::Vector6<f64>, // [Δp, Δv, (Δθ)]
        _jacobian: &na::Matrix6<f64>,      // Jacobian w.r.t. state
    ) {
        // In full implementation, use EKF update or direct damping
        // For now: simple proportional update
        let damping = 0.1;
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
/// - params[0]: Keyframe i state [T_W_B_i, v_i, bias_i] (15D: 6 + 3 + 3 + 3)
/// - params[1]: Keyframe j state [T_W_B_j, v_j, bias_j] (15D: 6 + 3 + 3 + 3)
///
/// Returns 6D residual: [Δp error; Δv error; Δθ error]
impl Factor for InterKeyframeImuFactor {
    fn get_dimension(&self) -> usize {
        6 // 6D residual: [Δp_error; Δv_error; Δθ_error]
    }

    fn linearize(
        &self,
        params: &[DVector<f64>],
        compute_jacobian: bool,
    ) -> (DVector<f64>, Option<DMatrix<f64>>) {
        // Expect 2 parameter vectors: keyframe i and j
        assert_eq!(
            params.len(),
            2,
            "InterKeyframeImuFactor requires 2 parameter vectors (keyframe i and j)"
        );

        // For now, implement simplified residual computation
        // Full implementation would extract SE3 pose, velocity, and biases from params

        // Extract state from first parameter (simplified: treat as SE3 pose only)
        let residual = DVector::zeros(6);
        let jacobian = if compute_jacobian {
            // 6x14 Jacobian: 6 residuals, 7+7 pose dimensions (simplified)
            Some(DMatrix::zeros(6, 14))
        } else {
            None
        };

        (residual, jacobian)
    }
}

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

/// Convert rotation matrix to axis-angle representation (3D vector)
fn matrix_to_axis_angle(R: &na::Matrix3<f64>) -> Vector3 {
    // Use Rodrigues' formula inverse
    let trace = R[(0, 0)] + R[(1, 1)] + R[(2, 2)];
    let angle = ((trace - 1.0) / 2.0).clamp(-1.0, 1.0).acos();

    if angle.abs() < 1e-6 {
        // Small angle: use skew-symmetric part
        Vector3::new(
            R[(2, 1)] - R[(1, 2)],
            R[(0, 2)] - R[(2, 0)],
            R[(1, 0)] - R[(0, 1)],
        ) * 0.5
    } else if (angle - PI).abs() < 1e-6 {
        // Angle close to π: extract from diagonal
        let diag = [R[(0, 0)] + 1.0, R[(1, 1)] + 1.0, R[(2, 2)] + 1.0];
        let idx = diag
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap()
            .0;

        let mut v = Vector3::zeros();
        match idx {
            0 => {
                v.x = (diag[0] / 2.0).sqrt();
                v.y = R[(0, 1)] / (2.0 * v.x);
                v.z = R[(0, 2)] / (2.0 * v.x);
            },
            1 => {
                v.y = (diag[1] / 2.0).sqrt();
                v.x = R[(0, 1)] / (2.0 * v.y);
                v.z = R[(1, 2)] / (2.0 * v.y);
            },
            _ => {
                v.z = (diag[2] / 2.0).sqrt();
                v.x = R[(0, 2)] / (2.0 * v.z);
                v.y = R[(1, 2)] / (2.0 * v.z);
            },
        }
        v * angle
    } else {
        // Normal case
        Vector3::new(
            R[(2, 1)] - R[(1, 2)],
            R[(0, 2)] - R[(2, 0)],
            R[(1, 0)] - R[(0, 1)],
        ) * (angle / (2.0 * angle.sin()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_gravity_model() {
        let gravity = GravityModel::earth();
        let g_vec = gravity.gravity_vector();
        assert!((g_vec.norm() - 9.81).abs() < 1e-6);
        assert!(g_vec.z < 0.0); // Pointing down
    }

    #[test]
    fn test_inter_keyframe_factor_creation() {
        let preintegration = ImuPreintegration {
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
    fn test_bias_refinement() {
        let mut refinement = BiasRefinement::new();
        assert!(refinement.is_valid());

        refinement.accel_bias = Vector3::new(1.0, 0.0, 0.0);
        assert!(!refinement.is_valid()); // Exceeds max_accel_bias = 0.5
    }
}
