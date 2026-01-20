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
}
