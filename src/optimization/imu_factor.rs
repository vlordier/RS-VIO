//! IMU preintegration factor for visual-inertial bundle adjustment.
//!
//! Implements a 15-DOF IMU factor connecting two consecutive keyframes.
//! The factor uses preintegrated IMU measurements to constrain:
//! - Relative rotation (3 DOF)
//! - Relative velocity (3 DOF)
//! - Relative position (3 DOF)
//! - Gyro bias prior (3 DOF)
//! - Accel bias prior (3 DOF)
//!
//! ## Variables (in order)
//! 1. `pose_i` — SE(3) pose of keyframe i: `[tx, ty, tz, qw, qx, qy, qz]`
//! 2. `pose_j` — SE(3) pose of keyframe j: `[tx, ty, tz, qw, qx, qy, qz]`
//! 3. `vel_i` — Velocity of keyframe i (world frame): `[vx, vy, vz]`
//! 4. `vel_j` — Velocity of keyframe j (world frame): `[vx, vy, vz]`
//! 5. `bg` — Gyro bias: `[bgx, bgy, bgz]`
//! 6. `ba` — Accel bias: `[bax, bay, baz]`
//!
//! The residual dimension is 15.

use crate::optimization::imu_preintegration::{PreintegratedImuMeasurements, Matrix15, Matrix15x3, Matrix15x6};
use apex_solver::factors::Factor;
use nalgebra::{DMatrix, DVector, Matrix3, Quaternion, UnitQuaternion, Vector3};
use std::sync::Arc;

/// IMU preintegration factor connecting two consecutive keyframes.
///
/// This factor implements the on-manifold preintegration theory from
/// Forster et al. (T-RO 2017). The 15-DOF residual is:
/// - 3 DOF rotation: log(ΔR_corrected^T · R_i^T · R_j)
/// - 3 DOF velocity: Δv_corrected - R_i^T · (v_j - v_i + g·Δt)
/// - 3 DOF position: Δp_corrected - R_i^T · (p_j - p_i - v_i·Δt - ½g·Δt²)
/// - 3 DOF gyro bias: bg - bg_lin (random walk prior)
/// - 3 DOF accel bias: ba - ba_lin (random walk prior)
///
/// Uses a diagonal information matrix (whitened residuals) for numerical stability.
#[derive(Debug, Clone)]
pub struct ImuPreintegrationFactor {
    /// Preintegrated IMU measurements (cached from linearization point).
    preintegrated: Arc<PreintegratedImuMeasurements>,
    /// Gravity vector in world frame.
    gravity: Vector3<f64>,
    /// Square root of the information matrix (for whitened residuals).
    /// Stored as lower-triangular from Cholesky decomposition.
    sqrt_info: Matrix15<f64>,
}

impl ImuPreintegrationFactor {
    /// Create a new IMU preintegration factor.
    ///
    /// # Arguments
    /// * `preintegrated` — preintegrated IMU measurements between two keyframes
    /// * `gravity` — gravity vector in world frame (typically [0, 0, 9.81])
    /// * `gyro_random_walk_std` — gyro bias random walk standard deviation
    /// * `accel_random_walk_std` — accel bias random walk standard deviation
    pub fn new(
        preintegrated: PreintegratedImuMeasurements,
        gravity: Vector3<f64>,
        gyro_random_walk_std: f64,
        accel_random_walk_std: f64,
    ) -> Self {
        let info = preintegrated.information_matrix(gyro_random_walk_std, accel_random_walk_std);

        // Compute Cholesky square root of information matrix for whitening.
        // If Cholesky fails, fall back to identity (unweighted residuals).
        let sqrt_info = if let Some(cholesky) = info.cholesky() {
            // L from Cholesky: Σ = L·L^T → Σ⁻¹ = L⁻ᵀ·L⁻¹
            // sqrt_info = L⁻¹ so that sqrt_info^T · sqrt_info = Σ⁻¹
            cholesky.l().try_inverse()
                .unwrap_or(Matrix15::identity())
        } else {
            Matrix15::identity()
        };

        Self {
            preintegrated: Arc::new(preintegrated),
            gravity,
            sqrt_info,
        }
    }

    /// Extract pose (R, t) from SE(3) parameters: [tx, ty, tz, qw, qx, qy, qz].
    #[inline]
    fn pose_from_params(p: &DVector<f64>) -> (Matrix3<f64>, Vector3<f64>) {
        let t = Vector3::new(p[0], p[1], p[2]);
        let q = UnitQuaternion::from_quaternion(
            Quaternion::new(p[3], p[4], p[5], p[6]).normalize(),
        );
        (q.to_rotation_matrix().into_inner(), t)
    }
}

/// IMU factor state for Jacobian computation.
struct ImuFactorState<'a> {
    r_i: &'a Matrix3<f64>,
    t_i: &'a Vector3<f64>,
    r_j: &'a Matrix3<f64>,
    t_j: &'a Vector3<f64>,
    v_i: &'a Vector3<f64>,
    v_j: &'a Vector3<f64>,
    bg: &'a Vector3<f64>,
}

impl ImuPreintegrationFactor {
    ///
    /// The residual is whitened: r_whitened = sqrt_info · r_raw
    /// So the Jacobian is: J_whitened = sqrt_info · J_raw
    ///
    /// For pose_i (variables: t_i, ω_i):
    ///   ∂r_R/∂t_i = 0
    ///   ∂r_R/∂ω_i = -J_r⁻¹(ΔR_corrected^T · R_i^T · R_j) · ΔR_corrected^T  (adjoint-ish)
    ///     ≈ -I (for small residuals, J_r⁻¹ ≈ I)
    ///   ∂r_v/∂t_i = 0
    ///   ∂r_v/∂ω_i = skew(R_i^T · (v_j - v_i + g·Δt))  (from R_i^T rotation derivative)
    ///   ∂r_p/∂t_i = -R_i^T
    ///   ∂r_p/∂ω_i = skew(R_i^T · (p_j - p_i - v_i·Δt - ½g·Δt²))
    ///
    /// For pose_j (variables: t_j, ω_j):
    ///   ∂r_R/∂t_j = 0
    ///   ∂r_R/∂ω_j = J_r⁻¹(ΔR_corrected^T · R_i^T · R_j) ≈ I
    ///   ∂r_v/∂t_j = 0
    ///   ∂r_v/∂ω_j = 0
    ///   ∂r_p/∂t_j = R_i^T
    ///   ∂r_p/∂ω_j = 0
    fn jacobian_wrt_pose_i(&self, state: ImuFactorState) -> Matrix15x6<f64> {
        let ImuFactorState { r_i, t_i, r_j, t_j, v_i, v_j, bg } = state;
        let db_g = bg - self.preintegrated.bg_lin;
        let delta_r_corrected = self.preintegrated.delta_r
            * crate::optimization::imu_preintegration::exp_so3(
                &(self.preintegrated.jacobian_delta_r_wrt_bg * db_g),
            );

        // Rotation error: R_err = ΔR_corrected^T · R_i^T · R_j
        let r_err = delta_r_corrected.transpose() * r_i.transpose() * r_j;
        let phi = crate::optimization::imu_preintegration::log_so3(&r_err);

        // Inverse right Jacobian for SO(3) log map derivative
        // For small residuals, Jr⁻¹ ≈ I + ½[φ]×
        let skew_phi = Matrix3::new(
            0.0, -phi.z, phi.y,
            phi.z, 0.0, -phi.x,
            -phi.y, phi.x, 0.0,
        );
        let jr_inv_approx = Matrix3::identity() + 0.5 * skew_phi;

        // ∂r_R/∂ω_i: derivative of log(ΔR^T · R_i^T · R_j) w.r.t. R_i
        // Using the chain rule: ∂log(R_err)/∂R_i ≈ -Jr⁻¹ · ΔR_corrected^T
        // (This is a simplification; the full derivation uses the adjoint)
        let dr_r_dwi = -jr_inv_approx * delta_r_corrected.transpose();

        // ∂r_v/∂ω_i = -skew(R_i^T · (v_j - v_i + g·Δt))
        // Derivation: under right perturbation R_new = R*Exp(δ),
        //   ∂(-R^T·f)/∂δ = -∂(δ×f)/∂δ evaluated using ∂(δ×v)/∂δ = -skew(v),
        //   giving ∂r_v/∂δ = -skew(dv_term).
        let dv_term = r_i.transpose() * (v_j - v_i + self.gravity * self.preintegrated.delta_t);
        let skew_dv = Matrix3::new(
            0.0, dv_term.z, -dv_term.y,
            -dv_term.z, 0.0, dv_term.x,
            dv_term.y, -dv_term.x, 0.0,
        );

        // ∂r_p/∂t_i = -R_i^T
        let dr_p_dti = -r_i.transpose();

        // ∂r_p/∂ω_i = -skew(R_i^T · (p_j - p_i - v_i·Δt + ½g·Δt²))
        // Same derivation as velocity: under right perturbation, ∂r_p/∂δ = -skew(dp_term).
        let dt = self.preintegrated.delta_t;
        let dp_term = r_i.transpose() * (t_j - t_i - v_i * dt + self.gravity * 0.5 * dt * dt);
        let skew_dp = Matrix3::new(
            0.0, dp_term.z, -dp_term.y,
            -dp_term.z, 0.0, dp_term.x,
            dp_term.y, -dp_term.x, 0.0,
        );

        // Build 15×6 Jacobian: [∂r/∂t_i | ∂r/∂ω_i]
        let mut j = Matrix15x6::<f64>::zeros();

        // Rotation rows (0-2): only ∂r_R/∂ω_i
        j.fixed_view_mut::<3, 3>(0, 3).copy_from(&dr_r_dwi);

        // Velocity rows (3-5): only ∂r_v/∂ω_i
        j.fixed_view_mut::<3, 3>(3, 3).copy_from(&skew_dv);

        // Position rows (6-8): ∂r_p/∂t_i and ∂r_p/∂ω_i
        j.fixed_view_mut::<3, 3>(6, 0).copy_from(&dr_p_dti);
        j.fixed_view_mut::<3, 3>(6, 3).copy_from(&skew_dp);

        // Bias rows (9-14): zero w.r.t. pose

        // Whiten: J_whitened = sqrt_info · J
        self.sqrt_info * j
    }

    fn jacobian_wrt_pose_j(
        &self,
        r_i: &Matrix3<f64>,
        r_j: &Matrix3<f64>,
    ) -> Matrix15x6<f64> {
        let db_g = Vector3::<f64>::zeros(); // bias is separate variable
        let delta_r_corrected = self.preintegrated.delta_r
            * crate::optimization::imu_preintegration::exp_so3(
                &(self.preintegrated.jacobian_delta_r_wrt_bg * db_g),
            );

        let r_err = delta_r_corrected.transpose() * r_i.transpose() * r_j;
        let phi = crate::optimization::imu_preintegration::log_so3(&r_err);
        let skew_phi = Matrix3::new(
            0.0, -phi.z, phi.y,
            phi.z, 0.0, -phi.x,
            -phi.y, phi.x, 0.0,
        );
        let jr_inv_approx = Matrix3::identity() + 0.5 * skew_phi;

        // ∂r_R/∂ω_j ≈ Jr⁻¹
        let dr_r_dwj = jr_inv_approx;

        // ∂r_p/∂t_j = R_i^T
        let dr_p_dtj = r_i.transpose();

        let mut j = Matrix15x6::<f64>::zeros();

        // Rotation rows: only ∂r_R/∂ω_j
        j.fixed_view_mut::<3, 3>(0, 3).copy_from(&dr_r_dwj);

        // Position rows: ∂r_p/∂t_j
        j.fixed_view_mut::<3, 3>(6, 0).copy_from(&dr_p_dtj);

        self.sqrt_info * j
    }

    /// Jacobian w.r.t. velocity variable (3 params).
    /// ∂r_v/∂v_i = R_i^T,  ∂r_v/∂v_j = -R_i^T
    /// ∂r_p/∂v_i = R_i^T · Δt
    fn jacobian_wrt_vel_i(&self, r_i: &Matrix3<f64>) -> Matrix15x3<f64> {
        let mut j = Matrix15x3::<f64>::zeros();
        let r_it = r_i.transpose();
        let dt = self.preintegrated.delta_t;

        // ∂r_v/∂v_i = R_i^T (from -R_i^T · (-v_i))
        j.fixed_view_mut::<3, 3>(3, 0).copy_from(&r_it);

        // ∂r_p/∂v_i = R_i^T · Δt (from -R_i^T · (-v_i·Δt))
        j.fixed_view_mut::<3, 3>(6, 0).copy_from(&(r_it * dt));

        self.sqrt_info * j
    }

    fn jacobian_wrt_vel_j(&self, r_i: &Matrix3<f64>) -> Matrix15x3<f64> {
        let mut j = Matrix15x3::<f64>::zeros();
        // ∂r_v/∂v_j = -R_i^T (from -R_i^T · v_j)
        j.fixed_view_mut::<3, 3>(3, 0).copy_from(&(-r_i.transpose()));

        self.sqrt_info * j
    }

    /// Jacobian w.r.t. gyro bias (3 params).
    fn jacobian_wrt_bg(&self) -> Matrix15x3<f64> {
        let mut j = Matrix15x3::<f64>::zeros();

        // ∂r_R/∂bg = -Jr⁻¹ · ΔR_corrected^T · ∂ΔR_corrected/∂bg
        // ∂ΔR_corrected/∂bg = ΔR_0 · Jr(J_dr_bg·δbg) · J_dr_bg
        // For small δbg: ≈ ΔR_0 · J_dr_bg
        // Simplified: ∂r_R/∂bg ≈ -J_dr_bg (first-order approx)
        j.fixed_view_mut::<3, 3>(0, 0).copy_from(&(-self.preintegrated.jacobian_delta_r_wrt_bg));

        // ∂r_v/∂bg = J_dv_bg
        j.fixed_view_mut::<3, 3>(3, 0).copy_from(&self.preintegrated.jacobian_delta_v_wrt_bg);

        // ∂r_p/∂bg = J_dp_bg
        j.fixed_view_mut::<3, 3>(6, 0).copy_from(&self.preintegrated.jacobian_delta_p_wrt_bg);

        // ∂r_bg/∂bg = I (identity prior)
        j.fixed_view_mut::<3, 3>(9, 0).copy_from(&Matrix3::identity());

        self.sqrt_info * j
    }

    /// Jacobian w.r.t. accel bias (3 params).
    fn jacobian_wrt_ba(&self) -> Matrix15x3<f64> {
        let mut j = Matrix15x3::<f64>::zeros();

        // ∂r_v/∂ba = J_dv_ba
        j.fixed_view_mut::<3, 3>(3, 0).copy_from(&self.preintegrated.jacobian_delta_v_wrt_ba);

        // ∂r_p/∂ba = J_dp_ba
        j.fixed_view_mut::<3, 3>(6, 0).copy_from(&self.preintegrated.jacobian_delta_p_wrt_ba);

        // ∂r_ba/∂ba = I
        j.fixed_view_mut::<3, 3>(12, 0).copy_from(&Matrix3::identity());

        self.sqrt_info * j
    }
}

impl Factor for ImuPreintegrationFactor {
    #![allow(non_snake_case)]
    fn linearize(
        &self,
        params: &[DVector<f64>],
        compute_jacobian: bool,
    ) -> (DVector<f64>, Option<DMatrix<f64>>) {
        // Expected 6 parameter vectors: pose_i, pose_j, vel_i, vel_j, bg, ba
        assert_eq!(params.len(), 6, "ImuPreintegrationFactor requires 6 parameter vectors");
        assert_eq!(params[0].len(), 7, "pose_i must be SE(3) (7 params)");
        assert_eq!(params[1].len(), 7, "pose_j must be SE(3) (7 params)");
        assert_eq!(params[2].len(), 3, "vel_i must be 3 params");
        assert_eq!(params[3].len(), 3, "vel_j must be 3 params");
        assert_eq!(params[4].len(), 3, "bg must be 3 params");
        assert_eq!(params[5].len(), 3, "ba must be 3 params");

        // Extract poses (note: these are T_W_B, so R_W_B and t_W_B)
        let (r_i_w, t_i_w) = Self::pose_from_params(&params[0]);
        let (r_j_w, t_j_w) = Self::pose_from_params(&params[1]);

        // Velocities in world frame
        let v_i = Vector3::new(params[2][0], params[2][1], params[2][2]);
        let v_j = Vector3::new(params[3][0], params[3][1], params[3][2]);

        // Biases
        let bg = Vector3::new(params[4][0], params[4][1], params[4][2]);
        let ba = Vector3::new(params[5][0], params[5][1], params[5][2]);

        // The preintegration is in the body frame of keyframe i.
        // R_i and R_j here are R_W_B (world-to-body rotations from the SE3 params).
        // The IMU factor expects R_B_W (body-to-world), which is R_W_B^T.
        // But actually, the preintegrated ΔR relates R_B_i to R_B_j:
        //   ΔR ≈ R_B_i_W · R_W_B_j = R_W_B_i^T · R_W_B_j
        // So we use r_i = R_W_B_i and r_j = R_W_B_j directly.

        // Compute residual
        let state = crate::optimization::imu_preintegration::ImuResidualState {
            r_i: &r_i_w,
            r_j: &r_j_w,
            v_i: &v_i,
            v_j: &v_j,
            p_i: &t_i_w,
            p_j: &t_j_w,
            bg: &bg,
            ba: &ba,
            gravity: &self.gravity,
        };
        let residual_raw = self.preintegrated.compute_residual(state);

        // Whiten the residual: r_whitened = sqrt_info · r_raw
        let r_vec = DVector::from_column_slice(&residual_raw);
        let residual_fixed = self.sqrt_info * r_vec;
        let residual = DVector::from_column_slice(residual_fixed.as_slice());

        if !compute_jacobian {
            return (residual, None);
        }

        // Compute Jacobians
        // Total Jacobian is 15 × (6+6+3+3+3+3) = 15 × 24
        let mut jacobian = DMatrix::zeros(15, 24);

        // Pose_i Jacobian (cols 0-5)
        let state = ImuFactorState {
            r_i: &r_i_w,
            t_i: &t_i_w,
            r_j: &r_j_w,
            t_j: &t_j_w,
            v_i: &v_i,
            v_j: &v_j,
            bg: &bg,
        };
        let j_pose_i = self.jacobian_wrt_pose_i(state);
        for row in 0..15 {
            for col in 0..6 {
                jacobian[(row, col)] = j_pose_i[(row, col)];
            }
        }

        // Pose_j Jacobian (cols 6-11)
        let j_pose_j = self.jacobian_wrt_pose_j(&r_i_w, &r_j_w);
        for row in 0..15 {
            for col in 0..6 {
                jacobian[(row, col + 6)] = j_pose_j[(row, col)];
            }
        }

        // Vel_i Jacobian (cols 12-14)
        let j_vel_i = self.jacobian_wrt_vel_i(&r_i_w);
        for row in 0..15 {
            for col in 0..3 {
                jacobian[(row, col + 12)] = j_vel_i[(row, col)];
            }
        }

        // Vel_j Jacobian (cols 15-17)
        let j_vel_j = self.jacobian_wrt_vel_j(&r_i_w);
        for row in 0..15 {
            for col in 0..3 {
                jacobian[(row, col + 15)] = j_vel_j[(row, col)];
            }
        }

        // BG Jacobian (cols 18-20)
        let j_bg = self.jacobian_wrt_bg();
        for row in 0..15 {
            for col in 0..3 {
                jacobian[(row, col + 18)] = j_bg[(row, col)];
            }
        }

        // BA Jacobian (cols 21-23)
        let j_ba = self.jacobian_wrt_ba();
        for row in 0..15 {
            for col in 0..3 {
                jacobian[(row, col + 21)] = j_ba[(row, col)];
            }
        }

        (residual, Some(jacobian))
    }

    fn get_dimension(&self) -> usize {
        15
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::datasets::ImuData;
    use crate::optimization::imu_preintegration::PreintegratedImuMeasurements;

    fn make_imu_sample(timestamp_ns: i64, gyro: [f64; 3], accel: [f64; 3]) -> ImuData {
        ImuData { timestamp: timestamp_ns, gyro, accel }
    }

    /// Numerical Jacobian verification via finite differences.
    /// Handles SE3 manifold correctly: perturbs translation (3) and rotation axis-angle (3),
    /// not the 7 quaternion params directly.
    fn numerical_jacobian(
        factor: &ImuPreintegrationFactor,
        params: &[DVector<f64>],
        epsilon: f64,
    ) -> DMatrix<f64> {
        let (residual_0, _) = factor.linearize(params, false);
        let dim_res = residual_0.len();
        // SE3: 6 DOF, Rn: N DOF
        let total_dof = params.iter().map(|p| {
            if p.len() == 7 { 6 } else { p.len() }
        }).sum();
        let mut j = DMatrix::zeros(dim_res, total_dof);

        let mut col_offset = 0;
        for (i_param, param) in params.iter().enumerate() {
            if param.len() == 7 {
                // SE3: perturb translation (first 3) and rotation (axis-angle on last 4)
                // Translation perturbation
                for d in 0..3 {
                    let mut p = param.clone();
                    p[d] += epsilon;
                    let mut params_plus = params.to_vec();
                    params_plus[i_param] = p;
                    let (res_plus, _) = factor.linearize(&params_plus, false);
                    for row in 0..dim_res {
                        j[(row, col_offset)] = (res_plus[row] - residual_0[row]) / epsilon;
                    }
                    col_offset += 1;
                }
                // Rotation perturbation: apply small rotation via quaternion multiplication
                for d in 0..3 {
                    let angle = epsilon;
                    let axis_vec = match d {
                        0 => nalgebra::Vector3::x(),
                        1 => nalgebra::Vector3::y(),
                        _ => nalgebra::Vector3::z(),
                    };
                    let delta_q = nalgebra::UnitQuaternion::from_scaled_axis(axis_vec * angle);

                    let q = nalgebra::Quaternion::new(param[3], param[4], param[5], param[6]);
                    let q_perturbed = nalgebra::UnitQuaternion::from_quaternion(q) * delta_q;
                    let q_perturbed = q_perturbed.quaternion();

                    let mut p = param.clone();
                    p[3] = q_perturbed.w;
                    p[4] = q_perturbed.i;
                    p[5] = q_perturbed.j;
                    p[6] = q_perturbed.k;
                    let mut params_plus = params.to_vec();
                    params_plus[i_param] = p;
                    let (res_plus, _) = factor.linearize(&params_plus, false);
                    for row in 0..dim_res {
                        j[(row, col_offset)] = (res_plus[row] - residual_0[row]) / epsilon;
                    }
                    col_offset += 1;
                }
            } else {
                // Rn: direct perturbation
                for j_coord in 0..param.len() {
                    let mut params_plus = params.to_vec();
                    let mut p = params[i_param].clone();
                    p[j_coord] += epsilon;
                    params_plus[i_param] = p;
                    let (res_plus, _) = factor.linearize(&params_plus, false);
                    for row in 0..dim_res {
                        j[(row, col_offset)] = (res_plus[row] - residual_0[row]) / epsilon;
                    }
                    col_offset += 1;
                }
            }
        }
        j
    }

    #[test]
    fn test_imu_factor_numerical_jacobian() {
        // Create synthetic IMU data: pure rotation around Z at 0.5 rad/s for 0.2s
        let gravity = nalgebra::Vector3::new(0.0, 0.0, 9.81);
        let gyro_bias_lin = [0.0; 3];
        let accel_bias_lin = [0.0; 3];
        let omega_z = 0.5;
        let dt_ns = 5_000_000; // 200 Hz
        let num_samples = 40; // 0.2s
        let total_angle = omega_z * (num_samples - 1) as f64 * 0.005;

        let samples: Vec<ImuData> = (0..num_samples)
            .map(|i| make_imu_sample(
                i as i64 * dt_ns,
                [0.0, 0.0, omega_z],
                [0.0, 0.0, 9.81],
            ))
            .collect();

        let noise = crate::optimization::imu_preintegration::ImuNoiseConfig {
            gyro_noise_std: 1e-3,
            gyro_random_walk_std: 1e-4,
            accel_noise_std: 1e-2,
            accel_random_walk_std: 1e-3,
        };
        let preint = PreintegratedImuMeasurements::preintegrate(
            &samples,
            gyro_bias_lin,
            accel_bias_lin,
            noise,
            &gravity,
        );

        let factor = ImuPreintegrationFactor::new(
            preint.clone(),
            gravity,
            1e-4,
            1e-3,
        );

        // Build consistent state parameters
        // Pose i: identity
        let pose_i = DVector::from_row_slice(&[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]);
        // Pose j: rotated by total_angle around Z
        let q_j = nalgebra::UnitQuaternion::from_euler_angles(0.0, 0.0, total_angle);
        let pose_j = DVector::from_row_slice(&[
            0.0, 0.0, 0.0,
            q_j.w, q_j.i, q_j.j, q_j.k,
        ]);
        // Velocities: consistent with Forster kinematic model.
        // For a pure rotation with a_body=[0,0,g], Δv=g·Δt in initial body frame.
        // r_v = Δv - R_i^T·(v_j-v_i+g·Δt) = g·Δt - (v_j+g·Δt) = 0 requires v_j=0.
        let v_i = DVector::from_row_slice(&[0.0, 0.0, 0.0]);
        let v_j = DVector::zeros(3);
        // Biases (at linearization point)
        let bg = DVector::from_row_slice(&[0.0, 0.0, 0.0]);
        let ba = DVector::from_row_slice(&[0.0, 0.0, 0.0]);

        let params = vec![pose_i, pose_j, v_i.clone(), v_j.clone(), bg.clone(), ba.clone()];

        // Compute analytical Jacobian
        let (_, jacobian_analytical) = factor.linearize(&params, true);
        let j_analytical = jacobian_analytical.expect("Jacobian should be computed");

        // Compute numerical Jacobian
        let j_numerical = numerical_jacobian(&factor, &params, 1e-8);

        // Compare
        assert_eq!(j_analytical.nrows(), j_numerical.nrows());
        assert_eq!(j_analytical.ncols(), j_numerical.ncols());

        let mut max_abs_err = 0.0;
        let mut max_rel_err = 0.0;
        let mut max_err_pos = (0, 0);

        for row in 0..j_analytical.nrows() {
            for col in 0..j_analytical.ncols() {
                let a = j_analytical[(row, col)];
                let n = j_numerical[(row, col)];
                let abs_err = (a - n).abs();
                let denom = a.abs().max(n.abs()).max(1.0);
                let rel_err = abs_err / denom;
                if rel_err > max_rel_err {
                    max_rel_err = rel_err;
                    max_abs_err = abs_err;
                    max_err_pos = (row, col);
                }
            }
        }

        // Allow 1% relative error (numerical differentiation is approximate)
        assert!(
            max_rel_err < 0.01,
            "Jacobian mismatch: max_rel_err = {:.2e} at ({}, {}), |analytical| = {:.6}, |numerical| = {:.6}",
            max_rel_err,
            max_err_pos.0,
            max_err_pos.1,
            j_analytical[(max_err_pos.0, max_err_pos.1)],
            j_numerical[(max_err_pos.0, max_err_pos.1)],
        );
    }

    #[test]
    fn test_imu_factor_residual_small_at_linearization() {
        // When the state is kinematically consistent with the preintegration, the IMU residual should be small
        let gravity = nalgebra::Vector3::new(0.0, 0.0, 9.81);
        let samples: Vec<ImuData> = (0..20)
            .map(|i| make_imu_sample(
                i as i64 * 5_000_000,
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 9.81],
            ))
            .collect();

        let noise = crate::optimization::imu_preintegration::ImuNoiseConfig {
            gyro_noise_std: 1e-3,
            gyro_random_walk_std: 1e-4,
            accel_noise_std: 1e-2,
            accel_random_walk_std: 1e-3,
        };
        let preint = PreintegratedImuMeasurements::preintegrate(
            &samples,
            [0.0; 3],
            [0.0; 3],
            noise,
            &gravity,
        );

        let factor = ImuPreintegrationFactor::new(preint.clone(), gravity, 1e-4, 1e-3);

        // 6 params: pose_i (SE3), pose_j (SE3), vel_i (R3), vel_j (R3), bg (R3), ba (R3)
        let pose_i = DVector::from_row_slice(&[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]);
        let pose_j = DVector::from_row_slice(&[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]);
        // Forster convention: v_j=0 for stationary (gravity in residual, not preintegration).
        let v_i = DVector::from_row_slice(&[0.0, 0.0, 0.0]);
        let v_j = DVector::zeros(3);
        let bg = DVector::from_row_slice(&[0.0; 3]);
        let ba = DVector::from_row_slice(&[0.0; 3]);

        let params = vec![pose_i, pose_j, v_i, v_j, bg, ba];

        let (residual, _) = factor.linearize(&params, false);
        let res_norm = residual.norm();

        // Residual should be small (whitened, so roughly unit-scale)
        assert!(res_norm < 15.0, "Residual norm too large: {:.3} (expected ≈ 0)", res_norm);
    }
}
