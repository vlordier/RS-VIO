use apex_solver::factors::Factor;
use apex_solver::manifold::se3;
use na::{DMatrix, DVector, Matrix4};
use nalgebra as na;

/// IMU prior factor
/// Data: Predicted body-from-world pose `T_B_W_pred` from IMU preintegration
/// Variables: System pose `T_B_W` (SE3)
/// Residual: 6D vector combining position error and rotation error (axis-angle)
#[derive(Debug, Clone)]
pub struct ImuPriorFactor {
    /// Predicted body-from-world pose from IMU preintegration
    pub T_B_W_pred: Matrix4<f64>,
    /// Weight for position residual components
    pub weight_pos: f64,
    /// Weight for rotation residual components
    pub weight_rot: f64,
}

impl ImuPriorFactor {
    pub fn new(T_B_W_pred: Matrix4<f64>, weight_pos: f64, weight_rot: f64) -> Self {
        Self {
            T_B_W_pred,
            weight_pos,
            weight_rot,
        }
    }
}

impl Factor for ImuPriorFactor {
    fn linearize(
        &self,
        params: &[DVector<f64>],
        compute_jacobian: bool,
    ) -> (DVector<f64>, Option<DMatrix<f64>>) {
        assert_eq!(
            params.len(),
            1,
            "ImuPriorFactor requires 1 parameter vector"
        );
        assert_eq!(
            params[0].len(),
            7,
            "System pose must have 7 parameters (tx, ty, tz, qw, qx, qy, qz)",
        );

        // Variable pose (body-from-world)
        let T_B_W_var = se3::SE3::from(params[0].clone());
        let R_B_W_var: na::Matrix3<f64> = T_B_W_var.rotation_so3().rotation_matrix();
        let t_B_W_var: na::Vector3<f64> = T_B_W_var.translation();

        // Predicted pose components
        let R_B_W_pred = self.T_B_W_pred.fixed_view::<3, 3>(0, 0).into_owned();
        let t_B_W_pred = self.T_B_W_pred.fixed_view::<3, 1>(0, 3).into_owned();

        // Position residual
        let t_err = t_B_W_var - t_B_W_pred;

        // Rotation residual: axis-angle from R_pred^T * R_var
        let R_err = R_B_W_pred.transpose() * R_B_W_var;
        let q_err =
            na::UnitQuaternion::from_rotation_matrix(&na::Rotation3::from_matrix_unchecked(R_err));
        let angle = q_err.angle();
        // For small angles, axis might be ill-defined; handle gracefully
        let axis = if angle > 1e-12 {
            q_err
                .axis()
                .map(|u| u.into_inner())
                .unwrap_or(na::Vector3::zeros())
        } else {
            na::Vector3::zeros()
        };
        let rot_vec = axis * angle;

        // Build 6D residual [pos; rot] with weights
        let mut residuals = DVector::zeros(6);
        residuals[0] = self.weight_pos * t_err.x;
        residuals[1] = self.weight_pos * t_err.y;
        residuals[2] = self.weight_pos * t_err.z;
        residuals[3] = self.weight_rot * rot_vec.x;
        residuals[4] = self.weight_rot * rot_vec.y;
        residuals[5] = self.weight_rot * rot_vec.z;

        let jacobian_matrix = if compute_jacobian {
            // Approximate Jacobian w.r.t. SE3 tangent as identity scaled by weights
            let mut jac = DMatrix::zeros(6, 6);
            // Position components map primarily to translation tangent
            jac[(0, 0)] = self.weight_pos;
            jac[(1, 1)] = self.weight_pos;
            jac[(2, 2)] = self.weight_pos;
            // Rotation components map to rotation tangent
            jac[(3, 3)] = self.weight_rot;
            jac[(4, 4)] = self.weight_rot;
            jac[(5, 5)] = self.weight_rot;
            Some(jac)
        } else {
            None
        };

        (residuals, jacobian_matrix)
    }

    fn get_dimension(&self) -> usize {
        6 // 3 position + 3 rotation
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn imu_prior_factor_zero_residual_and_weighted_jacobian() {
        let pred = Matrix4::identity();
        let factor = ImuPriorFactor::new(pred, 2.0, 3.0);

        // Parameters: [tx, ty, tz, qw, qx, qy, qz]
        let params = vec![DVector::from_vec(vec![0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])];
        let (residuals, jac) = factor.linearize(&params, true);

        // Zero residual at the prior mean
        assert!(residuals.iter().all(|v| v.abs() < 1e-12));

        // Jacobian is diagonal with the provided weights
        let jac = jac.expect("jacobian should be present when requested");
        assert_eq!(jac[(0, 0)], 2.0);
        assert_eq!(jac[(1, 1)], 2.0);
        assert_eq!(jac[(2, 2)], 2.0);
        assert_eq!(jac[(3, 3)], 3.0);
        assert_eq!(jac[(4, 4)], 3.0);
        assert_eq!(jac[(5, 5)], 3.0);
    }

    #[test]
    fn imu_prior_factor_translational_error_scales_weights() {
        let mut pred = Matrix4::identity();
        pred[(0, 3)] = 1.0; // expected x = 1

        let factor = ImuPriorFactor::new(pred, 2.0, 3.0);
        let params = vec![DVector::from_vec(vec![2.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])];
        let (residuals, _) = factor.linearize(&params, false);

        // Position error = +1 in x, weighted by 2.0
        assert!((residuals[0] - 2.0).abs() < 1e-12);
        assert!(residuals[1].abs() < 1e-12);
        assert!(residuals[2].abs() < 1e-12);
    }
}
