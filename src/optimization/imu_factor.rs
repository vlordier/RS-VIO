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

        // Try to compute Cholesky of inverse
        // If covariance is singular, use identity (unit information)
        let information = cov.try_inverse().unwrap_or_else(na::SMatrix::identity);

        // Compute Cholesky decomposition: Information = L * L^T
        // We need S such that S^T * S = Information, so S = L^T
        let sqrt_information = if let Some(chol) = information.cholesky() {
            chol.l().transpose()
        } else {
            // Fallback: use identity if information is not positive definite
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
    /// ΔR' = ΔR * Exp(-J_R_bg * δb_g)
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

        // Correct rotation: ΔR' = ΔR * Exp(-J_R_bg * δb_g)
        let delta_R_correction = exp_map_so3(-self.preintegration.J_R_bg * d_bias_g);
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

    /// Compute weighted residual for given states and biases
    #[allow(clippy::too_many_arguments)]
    fn compute_weighted_residual(
        &self,
        R_i: UnitQuaternion<f64>,
        v_i: Vector3<f64>,
        p_i: Vector3<f64>,
        R_j: UnitQuaternion<f64>,
        v_j: Vector3<f64>,
        p_j: Vector3<f64>,
        bias_g: Vector3<f64>,
        bias_a: Vector3<f64>,
    ) -> DVector<f64> {
        let (corrected_delta_R, corrected_delta_v, corrected_delta_p) =
            self.correct_for_bias(&bias_g, &bias_a);

        let dt = self.preintegration.delta_t;

        // ==================== Rotation Residual ====================
        let predicted_R_ij = R_i.inverse() * R_j;
        let rotation_error_quat = corrected_delta_R.inverse() * predicted_R_ij;
        let r_R = Self::quat_to_rotation_vector(&rotation_error_quat);

        // ==================== Velocity Residual ====================
        let predicted_dv_world = v_j - v_i - self.gravity * dt;
        let predicted_dv_i = R_i.inverse() * predicted_dv_world;
        let r_v = predicted_dv_i - corrected_delta_v;

        // ==================== Position Residual ====================
        let predicted_dp_world = p_j - p_i - v_i * dt - 0.5 * self.gravity * dt.powi(2);
        let predicted_dp_i = R_i.inverse() * predicted_dp_world;
        let r_p = predicted_dp_i - corrected_delta_p;

        let mut residual = na::SVector::<f64, 9>::zeros();
        residual.rows_mut(0, 3).copy_from(&r_R);
        residual.rows_mut(3, 3).copy_from(&r_v);
        residual.rows_mut(6, 3).copy_from(&r_p);

        let weighted = self.sqrt_information * residual;
        DVector::from_vec(weighted.as_slice().to_vec())
    }

    /// Convert unit quaternion to rotation vector (Log map)
    ///
    /// This is the inverse of exp_map (Rodrigues formula)
    fn quat_to_rotation_vector(q: &UnitQuaternion<f64>) -> Vector3<f64> {
        // For unit quaternion q = [w, x, y, z]
        // The rotation angle θ = 2 * acos(w)
        // The rotation axis n = [x, y, z] / sin(θ/2)
        // Rotation vector = θ * n

        let w = q.w;
        let vec = Vector3::new(q.i, q.j, q.k);

        // Small angle approximation for numerical stability
        if vec.norm() < 1e-8 {
            // θ ≈ 0, so rotation vector ≈ 2 * [x, y, z]
            return 2.0 * vec;
        }

        let theta = 2.0 * w.acos();
        let axis = vec / vec.norm();

        theta * axis
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
        let R_i = UnitQuaternion::from_quaternion(na::Quaternion::new(
            params[0][0],
            params[0][1],
            params[0][2],
            params[0][3],
        ));
        let v_i = Vector3::new(params[1][0], params[1][1], params[1][2]);
        let p_i = Vector3::new(params[2][0], params[2][1], params[2][2]);

        let R_j = UnitQuaternion::from_quaternion(na::Quaternion::new(
            params[3][0],
            params[3][1],
            params[3][2],
            params[3][3],
        ));
        let v_j = Vector3::new(params[4][0], params[4][1], params[4][2]);
        let p_j = Vector3::new(params[5][0], params[5][1], params[5][2]);

        let bias_g = Vector3::new(params[6][0], params[6][1], params[6][2]);
        let bias_a = Vector3::new(params[7][0], params[7][1], params[7][2]);

        let weighted_residual =
            self.compute_weighted_residual(R_i, v_i, p_i, R_j, v_j, p_j, bias_g, bias_a);

        // Compute Jacobians if requested
        let jacobian = if compute_jacobian {
            // Use numerical differentiation for now
            // TODO: Implement analytical Jacobians for performance
            let eps = 1e-7;
            let mut jac = DMatrix::zeros(9, 26); // 9 residuals x 26 parameters

            // Jacobian w.r.t. R_i (4 params)
            for i in 0..4 {
                let mut params_perturbed = params.to_vec();
                params_perturbed[0][i] += eps;
                let (r_pert, _) = self.linearize(&params_perturbed, false);
                let col = (r_pert - &weighted_residual) / eps;
                jac.column_mut(i).copy_from(&col);
            }

            // Jacobian w.r.t. v_i (3 params, offset 4)
            for i in 0..3 {
                let mut params_perturbed = params.to_vec();
                params_perturbed[1][i] += eps;
                let (r_pert, _) = self.linearize(&params_perturbed, false);
                let col = (r_pert - &weighted_residual) / eps;
                jac.column_mut(4 + i).copy_from(&col);
            }

            // Jacobian w.r.t. p_i (3 params, offset 7)
            for i in 0..3 {
                let mut params_perturbed = params.to_vec();
                params_perturbed[2][i] += eps;
                let (r_pert, _) = self.linearize(&params_perturbed, false);
                let col = (r_pert - &weighted_residual) / eps;
                jac.column_mut(7 + i).copy_from(&col);
            }

            // Jacobian w.r.t. R_j (4 params, offset 10)
            for i in 0..4 {
                let mut params_perturbed = params.to_vec();
                params_perturbed[3][i] += eps;
                let (r_pert, _) = self.linearize(&params_perturbed, false);
                let col = (r_pert - &weighted_residual) / eps;
                jac.column_mut(10 + i).copy_from(&col);
            }

            // Jacobian w.r.t. v_j (3 params, offset 14)
            for i in 0..3 {
                let mut params_perturbed = params.to_vec();
                params_perturbed[4][i] += eps;
                let (r_pert, _) = self.linearize(&params_perturbed, false);
                let col = (r_pert - &weighted_residual) / eps;
                jac.column_mut(14 + i).copy_from(&col);
            }

            // Jacobian w.r.t. p_j (3 params, offset 17)
            for i in 0..3 {
                let mut params_perturbed = params.to_vec();
                params_perturbed[5][i] += eps;
                let (r_pert, _) = self.linearize(&params_perturbed, false);
                let col = (r_pert - &weighted_residual) / eps;
                jac.column_mut(17 + i).copy_from(&col);
            }

            // Jacobian w.r.t. b_g (3 params, offset 20)
            for i in 0..3 {
                let mut params_perturbed = params.to_vec();
                params_perturbed[6][i] += eps;
                let (r_pert, _) = self.linearize(&params_perturbed, false);
                let col = (r_pert - &weighted_residual) / eps;
                jac.column_mut(20 + i).copy_from(&col);
            }

            // Jacobian w.r.t. b_a (3 params, offset 23)
            for i in 0..3 {
                let mut params_perturbed = params.to_vec();
                params_perturbed[7][i] += eps;
                let (r_pert, _) = self.linearize(&params_perturbed, false);
                let col = (r_pert - &weighted_residual) / eps;
                jac.column_mut(23 + i).copy_from(&col);
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
        let q_B_W = UnitQuaternion::from_quaternion(na::Quaternion::new(
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

        let weighted_residual = self
            .inner
            .compute_weighted_residual(R_i, v_i, p_i, R_j, v_j, p_j, bias_g, bias_a);

        let jacobian = if compute_jacobian {
            let eps = 1e-7;
            let mut jac = DMatrix::zeros(9, 26);

            for i in 0..7 {
                let mut params_perturbed = params.to_vec();
                params_perturbed[0][i] += eps;
                let (r_pert, _) = self.linearize(&params_perturbed, false);
                let col = (r_pert - &weighted_residual) / eps;
                jac.column_mut(i).copy_from(&col);
            }

            for i in 0..3 {
                let mut params_perturbed = params.to_vec();
                params_perturbed[1][i] += eps;
                let (r_pert, _) = self.linearize(&params_perturbed, false);
                let col = (r_pert - &weighted_residual) / eps;
                jac.column_mut(7 + i).copy_from(&col);
            }

            for i in 0..7 {
                let mut params_perturbed = params.to_vec();
                params_perturbed[2][i] += eps;
                let (r_pert, _) = self.linearize(&params_perturbed, false);
                let col = (r_pert - &weighted_residual) / eps;
                jac.column_mut(10 + i).copy_from(&col);
            }

            for i in 0..3 {
                let mut params_perturbed = params.to_vec();
                params_perturbed[3][i] += eps;
                let (r_pert, _) = self.linearize(&params_perturbed, false);
                let col = (r_pert - &weighted_residual) / eps;
                jac.column_mut(17 + i).copy_from(&col);
            }

            for i in 0..3 {
                let mut params_perturbed = params.to_vec();
                params_perturbed[4][i] += eps;
                let (r_pert, _) = self.linearize(&params_perturbed, false);
                let col = (r_pert - &weighted_residual) / eps;
                jac.column_mut(20 + i).copy_from(&col);
            }

            for i in 0..3 {
                let mut params_perturbed = params.to_vec();
                params_perturbed[5][i] += eps;
                let (r_pert, _) = self.linearize(&params_perturbed, false);
                let col = (r_pert - &weighted_residual) / eps;
                jac.column_mut(23 + i).copy_from(&col);
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
