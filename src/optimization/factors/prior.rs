use apex_solver::factors::Factor;
use na::{DMatrix, DVector, Matrix4};
use nalgebra as na;

/// Prior factor for marginalization.
///
/// This factor encodes the information from marginalized states as a prior
/// on the remaining parameters. It implements a quadratic prior:
///
/// ```text
/// r = prior_weight * (x - x0)
/// J = prior_weight
/// H = J^T * Information * J = prior_weight^2 * Information
/// ```
///
/// - Variables: Parameters with prior (variable dimension)
/// - Data: Linearization point `x0`, information matrix `Omega`
/// - Residual: `sqrt(Omega) * (x - x0)` scaled by prior_weight
///
/// # Mathematical Formulation
///
/// Given parameter vector `x` and prior information `x0`, `Omega`:
///
/// ```text
/// r = prior_weight * (x - x0)
/// Cost = 0.5 * r^T * Omega * r
/// ```
///
/// For Gaussian priors, `Omega` is the information matrix (inverse covariance).
#[derive(Debug, Clone)]
pub struct PriorFactor {
    /// Linearization point (prior mean)
    pub linearization_point: DVector<f64>,
    /// Information matrix (inverse covariance)
    pub information: DMatrix<f64>,
    /// Prior weight (scales the prior strength)
    pub prior_weight: f64,
}

impl PriorFactor {
    /// Create a new prior factor.
    ///
    /// # Arguments
    /// * `linearization_point` - Prior mean vector
    /// * `information` - Information matrix (must be square, matching parameter dimension)
    /// * `prior_weight` - Scaling factor for prior strength
    pub fn new(
        linearization_point: DVector<f64>,
        information: DMatrix<f64>,
        prior_weight: f64,
    ) -> Self {
        assert_eq!(
            linearization_point.len(),
            information.nrows(),
            "Linearization point dimension must match information matrix"
        );
        assert_eq!(
            information.nrows(),
            information.ncols(),
            "Information matrix must be square"
        );

        Self {
            linearization_point,
            information,
            prior_weight,
        }
    }

    /// Create a prior factor for SE3 pose (7 parameters).
    pub fn for_se3_pose(T_B_W: Matrix4<f64>, information: DMatrix<f64>, prior_weight: f64) -> Self {
        // Extract SE3 parameters: [tx, ty, tz, qw, qx, qy, qz]
        let t_B_W = T_B_W.fixed_view::<3, 1>(0, 3).into_owned();
        let R_B_W = T_B_W.fixed_view::<3, 3>(0, 0).into_owned();
        let q = na::UnitQuaternion::from_matrix(&R_B_W);

        let linearization_point =
            DVector::from_vec(vec![t_B_W.x, t_B_W.y, t_B_W.z, q.w, q.i, q.j, q.k]);

        Self::new(linearization_point, information, prior_weight)
    }
}

impl Factor for PriorFactor {
    fn linearize(
        &self,
        params: &[DVector<f64>],
        compute_jacobian: bool,
    ) -> (DVector<f64>, Option<DMatrix<f64>>) {
        assert_eq!(params.len(), 1, "PriorFactor requires 1 parameter vector");

        let param = &params[0];
        assert_eq!(
            param.len(),
            self.linearization_point.len(),
            "Parameter dimension must match prior"
        );

        // Residual: r = prior_weight * (x - x0)
        let delta = param - self.linearization_point.clone();
        let residuals = &delta * self.prior_weight;

        // Jacobian: J = prior_weight * I
        let jacobian_matrix = if compute_jacobian {
            let dim = param.len();
            let mut jac = DMatrix::zeros(dim, dim);
            for i in 0..dim {
                jac[(i, i)] = self.prior_weight;
            }
            Some(jac)
        } else {
            None
        };

        (residuals, jacobian_matrix)
    }

    fn get_dimension(&self) -> usize {
        self.linearization_point.len()
    }
}

/// Joint prior factor for marginalization that handles multiple coupled parameters.
///
/// This factor encodes the marginalization prior from Schur complement reduction,
/// which produces a joint information matrix coupling all kept parameters together.
///
/// Unlike `PriorFactor` which acts on a single parameter, this factor:
/// - Takes multiple parameter blocks as inputs
/// - Concatenates them in a specified order
/// - Applies the full coupled information matrix
///
/// # Mathematical Formulation
///
/// Given parameter blocks `x_1, ..., x_n` and joint linearization point `x0`, information `Omega`:
///
/// ```text
/// x_concat = [x_1; x_2; ...; x_n]  (concatenation)
/// r = sqrt(Omega) * (x_concat - x0)
/// Cost = 0.5 * r^T * Omega * r
/// ```
#[derive(Debug, Clone)]
pub struct JointPriorFactor {
    /// Linearization point for concatenated parameter vector
    pub linearization_point: DVector<f64>,
    /// Information matrix (inverse covariance) for all parameters jointly
    pub information: DMatrix<f64>,
    /// Prior weight (scales the prior strength)
    pub prior_weight: f64,
    /// Dimension of each parameter block (in order)
    pub param_dims: Vec<usize>,
}

impl JointPriorFactor {
    /// Create a new joint prior factor.
    ///
    /// # Arguments
    /// * `linearization_point` - Joint linearization point (concatenated)
    /// * `information` - Joint information matrix (must be square, matching concatenated dimension)
    /// * `prior_weight` - Scaling factor for prior strength
    /// * `param_dims` - Dimensions of each parameter block in order
    pub fn new(
        linearization_point: DVector<f64>,
        information: DMatrix<f64>,
        prior_weight: f64,
        param_dims: Vec<usize>,
    ) -> Self {
        let total_dim: usize = param_dims.iter().sum();
        assert_eq!(
            linearization_point.len(),
            total_dim,
            "Linearization point dimension must match sum of parameter dimensions"
        );
        assert_eq!(
            linearization_point.len(),
            information.nrows(),
            "Linearization point dimension must match information matrix"
        );
        assert_eq!(
            information.nrows(),
            information.ncols(),
            "Information matrix must be square"
        );

        Self {
            linearization_point,
            information,
            prior_weight,
            param_dims,
        }
    }
}

impl Factor for JointPriorFactor {
    fn linearize(
        &self,
        params: &[DVector<f64>],
        compute_jacobian: bool,
    ) -> (DVector<f64>, Option<DMatrix<f64>>) {
        assert_eq!(
            params.len(),
            self.param_dims.len(),
            "Number of parameters must match param_dims"
        );

        // Verify each parameter has the expected dimension
        for (i, (param, &expected_dim)) in params.iter().zip(self.param_dims.iter()).enumerate() {
            assert_eq!(
                param.len(),
                expected_dim,
                "Parameter {} dimension mismatch: expected {}, got {}",
                i,
                expected_dim,
                param.len()
            );
        }

        // Concatenate all parameter vectors
        let total_dim: usize = self.param_dims.iter().sum();
        let mut x_concat = DVector::zeros(total_dim);
        let mut offset = 0;
        for param in params {
            let dim = param.len();
            x_concat.rows_mut(offset, dim).copy_from(param);
            offset += dim;
        }

        // Compute residual: r = sqrt(Omega) * (x_concat - x0) * prior_weight
        // For numerical stability, we use Cholesky decomposition if available
        let delta = &x_concat - &self.linearization_point;

        // Try Cholesky decomposition for sqrt(Omega)
        let residuals = if let Some(chol) = self.information.clone().cholesky() {
            chol.l() * delta * self.prior_weight
        } else {
            // Fallback: use information matrix directly (less accurate but stable)
            &self.information * delta * self.prior_weight
        };

        // Jacobian: Block diagonal structure, each block is sqrt(Omega_ii) * prior_weight
        let jacobian_matrix = if compute_jacobian {
            if let Some(chol) = self.information.clone().cholesky() {
                let sqrt_omega = chol.l();
                Some(&sqrt_omega * self.prior_weight)
            } else {
                // Fallback: use information matrix directly
                Some(&self.information * self.prior_weight)
            }
        } else {
            None
        };

        (residuals, jacobian_matrix)
    }

    fn get_dimension(&self) -> usize {
        self.linearization_point.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn prior_factor_zero_residual_at_linearization_point() {
        let lin_point = DVector::from_vec(vec![1.0, 2.0]);
        let info = DMatrix::identity(2, 2);
        let factor = PriorFactor::new(lin_point.clone(), info, 2.5);

        let params = vec![lin_point];
        let (residuals, jac) = factor.linearize(&params, true);

        assert!(residuals.iter().all(|v| v.abs() < 1e-12));
        let jac = jac.expect("jacobian should be present");
        assert_eq!(jac[(0, 0)], 2.5);
        assert_eq!(jac[(1, 1)], 2.5);
    }

    #[test]
    fn prior_factor_residual_scales_delta_with_weight() {
        let lin_point = DVector::from_vec(vec![0.0, 0.0, 0.0]);
        let info = DMatrix::identity(3, 3);
        let factor = PriorFactor::new(lin_point, info, 1.5);

        let params = vec![DVector::from_vec(vec![1.0, -2.0, 0.5])];
        let (residuals, _) = factor.linearize(&params, false);

        assert!((residuals[0] - 1.5).abs() < 1e-12);
        assert!((residuals[1] + 3.0).abs() < 1e-12);
        assert!((residuals[2] - 0.75).abs() < 1e-12);
    }

    #[test]
    fn joint_prior_factor_zero_residual_at_linearization_point() {
        // Two parameters: 2D and 3D
        let lin_point = DVector::from_vec(vec![1.0, 2.0, 3.0, 4.0, 5.0]);
        let info = DMatrix::identity(5, 5);
        let param_dims = vec![2, 3];
        let factor = JointPriorFactor::new(lin_point.clone(), info, 1.0, param_dims);

        let params = vec![
            DVector::from_vec(vec![1.0, 2.0]),
            DVector::from_vec(vec![3.0, 4.0, 5.0]),
        ];
        let (residuals, jac) = factor.linearize(&params, true);

        assert!(residuals.iter().all(|v| v.abs() < 1e-12));
        assert!(jac.is_some());
    }

    #[test]
    fn joint_prior_factor_concatenates_parameters_correctly() {
        // Two parameters with different values
        let lin_point = DVector::from_vec(vec![0.0, 0.0, 0.0, 0.0]);
        let info = DMatrix::identity(4, 4) * 2.0; // Scale for easier verification
        let param_dims = vec![2, 2];
        let factor = JointPriorFactor::new(lin_point, info, 1.0, param_dims);

        let params = vec![
            DVector::from_vec(vec![1.0, 2.0]),
            DVector::from_vec(vec![3.0, 4.0]),
        ];
        let (residuals, _) = factor.linearize(&params, false);

        // Residual should be sqrt(info) * [1,2,3,4] * weight
        // With info = 2*I, sqrt(info) ≈ sqrt(2)*I
        let expected_scale = 2.0_f64.sqrt();
        assert!((residuals[0] - expected_scale * 1.0).abs() < 1e-10);
        assert!((residuals[1] - expected_scale * 2.0).abs() < 1e-10);
        assert!((residuals[2] - expected_scale * 3.0).abs() < 1e-10);
        assert!((residuals[3] - expected_scale * 4.0).abs() < 1e-10);
    }
}
