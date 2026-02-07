//! IMU preintegration factor for inter-keyframe constraints.
//!
//! Encodes integrated gyroscope and accelerometer measurements as a
//! nonlinear factor with analytic Jacobians for bundle adjustment.

use apex_solver::factors::Factor;
use na::{DMatrix, DVector, UnitQuaternion, Vector3};
use nalgebra as na;

use crate::imu::preintegration::{exp_map_so3, PreintegratedImu};

/// IMU preintegration factor for bundle adjustment
///
/// This factor represents the constraint imposed by integrated IMU measurements
/// between two keyframes. It provides:
/// - Measurement residuals (9D: rotation, velocity, position errors)
/// - Jacobians w.r.t. poses, velocities, and biases
/// - Integration with optimization frameworks
///
/// # Mathematical Formulation
///
/// The residual compares predicted state transitions (from IMU preintegration)
/// with the actual state transitions (from optimization variables):
///
/// ```text
/// r_R = Log(ΔR_measured^T * R_i^T * R_j)
/// r_v = R_i^T * (v_j - v_i - g*Δt) - Δv_measured
/// r_p = R_i^T * (p_j - p_i - v_i*Δt - 0.5*g*Δt²) - Δp_measured
/// ```
///
/// All measurements are bias-corrected using first-order approximation.
///
/// # Parameters
///
/// The factor takes 5 parameter blocks:
/// - params[0]: R_i (rotation at frame i, 4D quaternion)
/// - params[1]: v_i (velocity at frame i, 3D vector)
/// - params[2]: p_i (position at frame i, 3D vector)
/// - params[3]: R_j (rotation at frame j, 4D quaternion)
/// - params[4]: v_j (velocity at frame j, 3D vector)
/// - params[5]: p_j (position at frame j, 3D vector)
/// - params[6]: b_g (gyro bias, 3D vector)
/// - params[7]: b_a (accel bias, 3D vector)
#[derive(Debug, Clone)]
pub struct ImuFactor {
    /// Preintegrated IMU measurements and Jacobians
    pub preintegration: PreintegratedImu,

    /// Gravity vector in world frame [m/s²]
    pub gravity: Vector3<f64>,

    /// Square root information matrix (9x9)
    /// This is the Cholesky decomposition of the information matrix
    /// for numerical stability in optimization
    pub sqrt_information: na::SMatrix<f64, 9, 9>,
}

impl ImuFactor {
    /// Create new IMU factor
    ///
    /// # Arguments
    ///
    /// * `preintegration` - Preintegrated IMU measurements with Jacobians
    /// * `gravity` - Gravity vector in world frame (typically [0, 0, -9.81])
    pub fn new(preintegration: PreintegratedImu, gravity: Vector3<f64>) -> Self {
        // Compute square root information matrix from covariance
        // Information = Covariance^{-1}
        // sqrt_information such that sqrt_information^T * sqrt_information = Information
        let cov = preintegration.covariance;

        // Compute square root of information matrix via Cholesky of covariance.
        // cov = L * L^T, so cov^{-1} = L^{-T} * L^{-1}.
        // Setting M = L^{-1} gives M^T * M = cov^{-1} = Information.
        // This avoids the numerically unstable explicit dense inverse.
        let sqrt_information = if let Some(chol) = cov.cholesky() {
            chol.l().try_inverse().unwrap_or_else(na::SMatrix::identity)
        } else {
            // Fallback: use identity if covariance is not positive definite
            na::SMatrix::identity()
        };

        Self {
            preintegration,
            gravity,
            sqrt_information,
        }
    }

    /// Compute bias-corrected preintegration predictions
    ///
    /// Uses first-order approximation from Forster et al. 2017:
    /// ```text
    /// ΔR' = ΔR * Exp(J_R_bg * δb_g)
    /// Δv' = Δv - J_v_bg * δb_g - J_v_ba * δb_a
    /// Δp' = Δp - J_p_bg * δb_g - J_p_ba * δb_a
    /// ```
    fn correct_for_bias(
        &self,
        bias_g: &Vector3<f64>,
        bias_a: &Vector3<f64>,
    ) -> (UnitQuaternion<f64>, Vector3<f64>, Vector3<f64>) {
        // Compute bias delta from linearization point
        let d_bias_g = bias_g - self.preintegration.linearization_point_bg;
        let d_bias_a = bias_a - self.preintegration.linearization_point_ba;

        // Correct rotation: ΔR' = ΔR * Exp(J_R_bg * δb_g)
        // J_R_bg already encodes the negative sign from ∂Exp((ω-bg)dt)/∂bg
        let delta_R_correction = exp_map_so3(self.preintegration.J_R_bg * d_bias_g);
        let corrected_delta_R = self.preintegration.delta_R * delta_R_correction;

        // Correct velocity: Δv' = Δv - J_v_bg * δb_g - J_v_ba * δb_a
        let corrected_delta_v = self.preintegration.delta_v
            - self.preintegration.J_v_bg * d_bias_g
            - self.preintegration.J_v_ba * d_bias_a;

        // Correct position: Δp' = Δp - J_p_bg * δb_g - J_p_ba * δb_a
        let corrected_delta_p = self.preintegration.delta_p
            - self.preintegration.J_p_bg * d_bias_g
            - self.preintegration.J_p_ba * d_bias_a;

        (corrected_delta_R, corrected_delta_v, corrected_delta_p)
    }

    /// Compute weighted residual for given states and biases (stack-allocated)
    #[allow(clippy::too_many_arguments)]
    fn compute_weighted_residual_svec(
        &self,
        R_i: UnitQuaternion<f64>,
        v_i: Vector3<f64>,
        p_i: Vector3<f64>,
        R_j: UnitQuaternion<f64>,
        v_j: Vector3<f64>,
        p_j: Vector3<f64>,
        bias_g: Vector3<f64>,
        bias_a: Vector3<f64>,
    ) -> na::SVector<f64, 9> {
        let (corrected_delta_R, corrected_delta_v, corrected_delta_p) =
            self.correct_for_bias(&bias_g, &bias_a);

        let dt = self.preintegration.delta_t;
        let dt2 = dt * dt;

        // Cache R_i inverse — used by rotation, velocity, and position residuals
        let R_i_inv = R_i.inverse();

        // ==================== Rotation Residual ====================
        let predicted_R_ij = R_i_inv * R_j;
        let rotation_error_quat = corrected_delta_R.inverse() * predicted_R_ij;
        let r_R = Self::quat_to_rotation_vector(&rotation_error_quat);

        // ==================== Velocity Residual ====================
        let predicted_dv_world = v_j - v_i - self.gravity * dt;
        let predicted_dv_i = R_i_inv * predicted_dv_world;
        let r_v = predicted_dv_i - corrected_delta_v;

        // ==================== Position Residual ====================
        let predicted_dp_world = p_j - p_i - v_i * dt - 0.5 * self.gravity * dt2;
        let predicted_dp_i = R_i_inv * predicted_dp_world;
        let r_p = predicted_dp_i - corrected_delta_p;

        let mut residual = na::SVector::<f64, 9>::zeros();
        residual.rows_mut(0, 3).copy_from(&r_R);
        residual.rows_mut(3, 3).copy_from(&r_v);
        residual.rows_mut(6, 3).copy_from(&r_p);

        self.sqrt_information * residual
    }

    /// Convert unit quaternion to rotation vector (Log map)
    fn quat_to_rotation_vector(q: &UnitQuaternion<f64>) -> Vector3<f64> {
        q.scaled_axis()
    }
}

impl Factor for ImuFactor {
    fn get_dimension(&self) -> usize {
        9 // 9D residual (rotation 3D, velocity 3D, position 3D)
    }

    /// Linearize the IMU factor
    ///
    /// # Parameters
    ///
    /// - params[0]: R_i as quaternion [w, x, y, z] (4D)
    /// - params[1]: v_i (3D)
    /// - params[2]: p_i (3D)
    /// - params[3]: R_j as quaternion [w, x, y, z] (4D)
    /// - params[4]: v_j (3D)
    /// - params[5]: p_j (3D)
    /// - params[6]: b_g (3D)
    /// - params[7]: b_a (3D)
    ///
    /// # Returns
    ///
    /// - Residual: 9D vector [r_R (3D), r_v (3D), r_p (3D)]
    /// - Jacobian: 9 x (4+3+3+4+3+3+3+3) = 9x26 matrix (if requested)
    fn linearize(
        &self,
        params: &[DVector<f64>],
        compute_jacobian: bool,
    ) -> (DVector<f64>, Option<DMatrix<f64>>) {
        assert_eq!(params.len(), 8, "ImuFactor requires 8 parameter blocks");
        assert_eq!(params[0].len(), 4, "R_i must be 4D quaternion");
        assert_eq!(params[1].len(), 3, "v_i must be 3D");
        assert_eq!(params[2].len(), 3, "p_i must be 3D");
        assert_eq!(params[3].len(), 4, "R_j must be 4D quaternion");
        assert_eq!(params[4].len(), 3, "v_j must be 3D");
        assert_eq!(params[5].len(), 3, "p_j must be 3D");
        assert_eq!(params[6].len(), 3, "b_g must be 3D");
        assert_eq!(params[7].len(), 3, "b_a must be 3D");

        // Extract parameters
        let R_i = UnitQuaternion::new_normalize(na::Quaternion::new(
            params[0][0],
            params[0][1],
            params[0][2],
            params[0][3],
        ));
        let v_i = Vector3::new(params[1][0], params[1][1], params[1][2]);
        let p_i = Vector3::new(params[2][0], params[2][1], params[2][2]);

        let R_j = UnitQuaternion::new_normalize(na::Quaternion::new(
            params[3][0],
            params[3][1],
            params[3][2],
            params[3][3],
        ));
        let v_j = Vector3::new(params[4][0], params[4][1], params[4][2]);
        let p_j = Vector3::new(params[5][0], params[5][1], params[5][2]);

        let bias_g = Vector3::new(params[6][0], params[6][1], params[6][2]);
        let bias_a = Vector3::new(params[7][0], params[7][1], params[7][2]);

        let weighted_residual_svec =
            self.compute_weighted_residual_svec(R_i, v_i, p_i, R_j, v_j, p_j, bias_g, bias_a);
        let weighted_residual = DVector::from_column_slice(weighted_residual_svec.as_slice());

        // Compute Jacobians if requested
        let jacobian = if compute_jacobian {
            // Use numerical differentiation for now
            // TODO: Implement analytical Jacobians for performance
            let eps = 1e-7;
            let inv_eps = 1.0 / eps;
            let mut jac = DMatrix::zeros(9, 26); // 9 residuals x 26 parameters

            // Clone once and perturb/restore in-place to avoid 26 full clones
            let mut params_pert = params.to_vec();

            // Helper closure: extract params → call compute_weighted_residual_svec directly,
            // avoiding re-parsing asserts and DVector allocs per column
            let extract_and_eval = |p: &[DVector<f64>]| -> na::SVector<f64, 9> {
                let r_i = UnitQuaternion::new_normalize(na::Quaternion::new(
                    p[0][0], p[0][1], p[0][2], p[0][3],
                ));
                let vi = Vector3::new(p[1][0], p[1][1], p[1][2]);
                let pi = Vector3::new(p[2][0], p[2][1], p[2][2]);
                let r_j = UnitQuaternion::new_normalize(na::Quaternion::new(
                    p[3][0], p[3][1], p[3][2], p[3][3],
                ));
                let vj = Vector3::new(p[4][0], p[4][1], p[4][2]);
                let pj = Vector3::new(p[5][0], p[5][1], p[5][2]);
                let bg = Vector3::new(p[6][0], p[6][1], p[6][2]);
                let ba = Vector3::new(p[7][0], p[7][1], p[7][2]);
                self.compute_weighted_residual_svec(r_i, vi, pi, r_j, vj, pj, bg, ba)
            };

            let block_sizes: [(usize, usize); 8] = [
                (0, 4), // R_i → col offset 0
                (1, 3), // v_i → col offset 4
                (2, 3), // p_i → col offset 7
                (3, 4), // R_j → col offset 10
                (4, 3), // v_j → col offset 14
                (5, 3), // p_j → col offset 17
                (6, 3), // b_g → col offset 20
                (7, 3), // b_a → col offset 23
            ];
            let mut col_offset = 0usize;
            for &(block_idx, block_size) in &block_sizes {
                for i in 0..block_size {
                    params_pert[block_idx][i] += eps;
                    // SVector path: no heap allocs for residual or diff
                    let r_pert = extract_and_eval(&params_pert);
                    let col = (r_pert - weighted_residual_svec) * inv_eps;
                    jac.column_mut(col_offset + i).copy_from(&col);
                    params_pert[block_idx][i] -= eps; // restore
                }
                col_offset += block_size;
            }

            Some(jac)
        } else {
            None
        };

        (weighted_residual, jacobian)
    }
}

/// IMU preintegration factor using SE3 pose variables
///
/// Parameter blocks:
/// - params[0]: T_B_W_i (SE3, 7D: t, q)
/// - params[1]: v_i (3D)
/// - params[2]: T_B_W_j (SE3, 7D: t, q)
/// - params[3]: v_j (3D)
/// - params[4]: b_g (3D)
/// - params[5]: b_a (3D)
#[derive(Debug, Clone)]
pub struct ImuFactorSe3 {
    inner: ImuFactor,
}

impl ImuFactorSe3 {
    /// Create new IMU factor (SE3 version)
    pub fn new(preintegration: PreintegratedImu, gravity: Vector3<f64>) -> Self {
        Self {
            inner: ImuFactor::new(preintegration, gravity),
        }
    }

    fn pose_from_se3(param: &DVector<f64>) -> (UnitQuaternion<f64>, Vector3<f64>) {
        let t_B_W = Vector3::new(param[0], param[1], param[2]);
        let q_B_W = UnitQuaternion::new_normalize(na::Quaternion::new(
            param[3], param[4], param[5], param[6],
        ));
        let q_W_B = q_B_W.inverse();
        let p_W_B = -(q_W_B.transform_vector(&t_B_W));
        (q_W_B, p_W_B)
    }
}

impl Factor for ImuFactorSe3 {
    fn get_dimension(&self) -> usize {
        9
    }

    fn linearize(
        &self,
        params: &[DVector<f64>],
        compute_jacobian: bool,
    ) -> (DVector<f64>, Option<DMatrix<f64>>) {
        assert_eq!(params.len(), 6, "ImuFactorSe3 requires 6 parameter blocks");
        assert_eq!(params[0].len(), 7, "T_B_W_i must be 7D (t, q)");
        assert_eq!(params[1].len(), 3, "v_i must be 3D");
        assert_eq!(params[2].len(), 7, "T_B_W_j must be 7D (t, q)");
        assert_eq!(params[3].len(), 3, "v_j must be 3D");
        assert_eq!(params[4].len(), 3, "b_g must be 3D");
        assert_eq!(params[5].len(), 3, "b_a must be 3D");

        let (R_i, p_i) = Self::pose_from_se3(&params[0]);
        let v_i = Vector3::new(params[1][0], params[1][1], params[1][2]);
        let (R_j, p_j) = Self::pose_from_se3(&params[2]);
        let v_j = Vector3::new(params[3][0], params[3][1], params[3][2]);
        let bias_g = Vector3::new(params[4][0], params[4][1], params[4][2]);
        let bias_a = Vector3::new(params[5][0], params[5][1], params[5][2]);

        // Use stack-allocated SVector for base residual, convert to DVector only at return
        let weighted_svec = self
            .inner
            .compute_weighted_residual_svec(R_i, v_i, p_i, R_j, v_j, p_j, bias_g, bias_a);
        let weighted_residual = DVector::from_column_slice(weighted_svec.as_slice());

        let jacobian = if compute_jacobian {
            let eps = 1e-7;
            let inv_eps = 1.0 / eps;
            let mut jac = DMatrix::zeros(9, 26);

            // Clone once and perturb/restore in-place
            let mut params_pert = params.to_vec();

            // Inline extraction+eval closure: avoids recursive self.linearize() call
            // which would heap-allocate 3 DVectors per column (78 total for 26 columns)
            let eval_svec = |p: &[DVector<f64>]| -> na::SVector<f64, 9> {
                let (r_i, pos_i) = Self::pose_from_se3(&p[0]);
                let vi = Vector3::new(p[1][0], p[1][1], p[1][2]);
                let (r_j, pos_j) = Self::pose_from_se3(&p[2]);
                let vj = Vector3::new(p[3][0], p[3][1], p[3][2]);
                let bg = Vector3::new(p[4][0], p[4][1], p[4][2]);
                let ba = Vector3::new(p[5][0], p[5][1], p[5][2]);
                self.inner
                    .compute_weighted_residual_svec(r_i, vi, pos_i, r_j, vj, pos_j, bg, ba)
            };

            let block_sizes: [(usize, usize); 6] = [
                (0, 7), // T_B_W_i → col offset 0
                (1, 3), // v_i     → col offset 7
                (2, 7), // T_B_W_j → col offset 10
                (3, 3), // v_j     → col offset 17
                (4, 3), // b_g     → col offset 20
                (5, 3), // b_a     → col offset 23
            ];
            let mut col_offset = 0usize;
            for &(block_idx, block_size) in &block_sizes {
                for i in 0..block_size {
                    params_pert[block_idx][i] += eps;
                    // SVector path: zero heap allocs for residual computation and diff
                    let r_pert = eval_svec(&params_pert);
                    let col = (r_pert - weighted_svec) * inv_eps;
                    jac.column_mut(col_offset + i).copy_from(&col);
                    params_pert[block_idx][i] -= eps; // restore
                }
                col_offset += block_size;
            }

            Some(jac)
        } else {
            None
        };

        (weighted_residual, jacobian)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::imu::ImuNoise;

    #[test]
    fn test_imu_factor_zero_motion() {
        // Create preintegration with zero motion (identity)
        let noise = ImuNoise::default();
        let preint = PreintegratedImu::new(noise);

        let gravity = Vector3::new(0.0, 0.0, -9.81);
        let factor = ImuFactor::new(preint, gravity);

        // Set up states: no motion between frames (except gravity effect on velocity)
        let R_i = UnitQuaternion::identity();
        let v_i = Vector3::zeros();
        let p_i = Vector3::zeros();

        let R_j = UnitQuaternion::identity();
        let v_j = Vector3::zeros(); // Zero motion in this test
        let p_j = Vector3::zeros();

        let bias_g = Vector3::zeros();
        let bias_a = Vector3::zeros();

        // Create parameter blocks
        let params = vec![
            DVector::from_vec(vec![R_i.w, R_i.i, R_i.j, R_i.k]),
            DVector::from_vec(vec![v_i[0], v_i[1], v_i[2]]),
            DVector::from_vec(vec![p_i[0], p_i[1], p_i[2]]),
            DVector::from_vec(vec![R_j.w, R_j.i, R_j.j, R_j.k]),
            DVector::from_vec(vec![v_j[0], v_j[1], v_j[2]]),
            DVector::from_vec(vec![p_j[0], p_j[1], p_j[2]]),
            DVector::from_vec(vec![bias_g[0], bias_g[1], bias_g[2]]),
            DVector::from_vec(vec![bias_a[0], bias_a[1], bias_a[2]]),
        ];

        let (residual, _) = factor.linearize(&params, false);

        // With zero preintegration, residual should be small
        // (but not exactly zero due to gravity term in velocity)
        assert_eq!(residual.len(), 9, "Residual should be 9D");
        println!("Zero motion residual norm: {:.6}", residual.norm());

        // Rotation and position residuals should be near zero
        let r_R_norm = residual.rows(0, 3).norm();
        let r_p_norm = residual.rows(6, 3).norm();
        assert!(r_R_norm < 1e-6, "Rotation residual should be near zero");
        assert!(r_p_norm < 1e-6, "Position residual should be near zero");
    }

    #[test]
    fn test_imu_factor_jacobians_finite() {
        // Test that Jacobians are finite and non-degenerate
        let noise = ImuNoise::default();
        let preint = PreintegratedImu::new(noise);

        let gravity = Vector3::new(0.0, 0.0, -9.81);
        let factor = ImuFactor::new(preint, gravity);

        // Random state
        let R_i = UnitQuaternion::identity();
        let v_i = Vector3::new(0.1, 0.2, 0.3);
        let p_i = Vector3::new(1.0, 2.0, 3.0);

        let R_j = UnitQuaternion::identity();
        let v_j = Vector3::new(0.15, 0.25, 0.25);
        let p_j = Vector3::new(1.1, 2.1, 3.0);

        let bias_g = Vector3::new(0.01, -0.01, 0.005);
        let bias_a = Vector3::new(0.05, 0.02, -0.03);

        let params = vec![
            DVector::from_vec(vec![R_i.w, R_i.i, R_i.j, R_i.k]),
            DVector::from_vec(vec![v_i[0], v_i[1], v_i[2]]),
            DVector::from_vec(vec![p_i[0], p_i[1], p_i[2]]),
            DVector::from_vec(vec![R_j.w, R_j.i, R_j.j, R_j.k]),
            DVector::from_vec(vec![v_j[0], v_j[1], v_j[2]]),
            DVector::from_vec(vec![p_j[0], p_j[1], p_j[2]]),
            DVector::from_vec(vec![bias_g[0], bias_g[1], bias_g[2]]),
            DVector::from_vec(vec![bias_a[0], bias_a[1], bias_a[2]]),
        ];

        let (residual, jacobian) = factor.linearize(&params, true);

        assert!(
            residual.iter().all(|x| x.is_finite()),
            "Residual should be finite"
        );

        let jac = jacobian.expect("Jacobian should be computed");
        assert_eq!(jac.nrows(), 9, "Jacobian should have 9 rows");
        assert_eq!(jac.ncols(), 26, "Jacobian should have 26 columns");
        assert!(
            jac.iter().all(|x| x.is_finite()),
            "Jacobian should be finite"
        );

        // Check that Jacobian is not all zeros
        let jac_norm = jac.norm();
        assert!(jac_norm > 1e-10, "Jacobian should not be degenerate");
    }

    #[test]
    fn test_imu_factor_se3_residual_finite() {
        let noise = ImuNoise::default();
        let preint = PreintegratedImu::new(noise);

        let gravity = Vector3::new(0.0, 0.0, -9.81);
        let factor = ImuFactorSe3::new(preint, gravity);

        let t_B_W_i = Vector3::zeros();
        let q_B_W_i = UnitQuaternion::identity();
        let t_B_W_j = Vector3::zeros();
        let q_B_W_j = UnitQuaternion::identity();

        let v_i = Vector3::zeros();
        let v_j = Vector3::zeros();
        let bias_g = Vector3::zeros();
        let bias_a = Vector3::zeros();

        let params = vec![
            DVector::from_vec(vec![
                t_B_W_i.x, t_B_W_i.y, t_B_W_i.z, q_B_W_i.w, q_B_W_i.i, q_B_W_i.j, q_B_W_i.k,
            ]),
            DVector::from_vec(vec![v_i.x, v_i.y, v_i.z]),
            DVector::from_vec(vec![
                t_B_W_j.x, t_B_W_j.y, t_B_W_j.z, q_B_W_j.w, q_B_W_j.i, q_B_W_j.j, q_B_W_j.k,
            ]),
            DVector::from_vec(vec![v_j.x, v_j.y, v_j.z]),
            DVector::from_vec(vec![bias_g.x, bias_g.y, bias_g.z]),
            DVector::from_vec(vec![bias_a.x, bias_a.y, bias_a.z]),
        ];

        let (residual, jacobian) = factor.linearize(&params, true);
        assert_eq!(residual.len(), 9);
        assert!(residual.iter().all(|x| x.is_finite()));

        let jac = jacobian.expect("Jacobian should be computed");
        assert_eq!(jac.nrows(), 9);
        assert_eq!(jac.ncols(), 26);
        assert!(jac.iter().all(|x| x.is_finite()));
    }
}
