//! Trait definitions for marginalization strategies

use na::{DMatrix, DVector};
use nalgebra as na;
use std::collections::HashMap;
use std::fmt::Debug;

use super::config::{MarginalizationConfig, ParamId};
use crate::MarginalizationPrior;

// ============================================================================
// Hessian Approximator Trait
// ============================================================================

/// Trait for computing Hessian approximations.
///
/// This trait abstracts the computation of approximate Hessian matrices,
/// allowing different approximation strategies to be used interchangeably.
///
/// # Example
///
/// ```rust
/// use rs_vio::optimization::marginalization::{HessianApproximator, GaussNewtonApproximator};
/// use nalgebra as na;
///
/// let approximator = GaussNewtonApproximator::default();
/// let residuals = na::DVector::zeros(5);
/// let hessian = approximator.compute_hessian(&residuals, 10, None);
/// ```
pub trait HessianApproximator: Debug + 'static {
    /// Compute approximate Hessian matrix
    ///
    /// # Arguments
    /// * `residuals` - Vector of residual values at current linearization point
    /// * `param_dim` - Total dimension of all parameters
    /// * `jacobians` - Optional pre-computed Jacobians
    ///
    /// # Returns
    /// Approximate Hessian matrix
    fn compute_hessian(
        &self,
        residuals: &DVector<f64>,
        param_dim: usize,
        jacobians: Option<&Vec<DMatrix<f64>>>,
    ) -> DMatrix<f64>;

    /// Get the name of this approximator for logging
    fn name(&self) -> &str;

    /// Clone as boxed trait object
    fn clone_box(&self) -> Box<dyn HessianApproximator>;
}

// ============================================================================
// Gradient Computer Trait
// ============================================================================

/// Trait for computing gradients.
///
/// This trait abstracts gradient computation, supporting different
/// approximation strategies when exact Jacobians are not available.
pub trait GradientComputer: Debug + 'static {
    /// Compute gradient vector
    ///
    /// # Arguments
    /// * `residuals` - Residual vector
    /// * `param_dim` - Total parameter dimension
    /// * `jacobians` - Optional pre-computed Jacobians
    ///
    /// # Returns
    /// Gradient vector
    fn compute_gradient(
        &self,
        residuals: &DVector<f64>,
        param_dim: usize,
        jacobians: Option<&Vec<DMatrix<f64>>>,
    ) -> DVector<f64>;

    /// Get the name of this computer for logging
    fn name(&self) -> &str;

    /// Clone as boxed trait object
    fn clone_box(&self) -> Box<dyn GradientComputer>;
}

// ============================================================================
// Prior Constructor Trait
// ============================================================================

/// Trait for constructing marginalization priors.
///
/// This trait abstracts the construction of prior factors from the
/// Schur complement computation.
pub trait PriorConstructor: Debug + 'static {
    /// Construct prior from Schur complement
    ///
    /// # Arguments
    /// * `schur_complement` - Schur complement matrix S
    /// * `reduced_gradient` - Reduced gradient vector b_eff
    /// * `param_ids` - Parameter IDs for the prior
    /// * `residual_dim` - Dimension of the prior residual
    /// * `config` - Configuration for prior construction
    ///
    /// # Returns
    /// MarginalizationPrior or None if construction fails
    fn construct_prior(
        &self,
        schur_complement: &DMatrix<f64>,
        reduced_gradient: &DVector<f64>,
        param_ids: &[ParamId],
        residual_dim: usize,
        config: &MarginalizationConfig,
        linearization_points: &HashMap<ParamId, DVector<f64>>,
    ) -> Option<MarginalizationPrior>;

    /// Get the name of this constructor for logging
    fn name(&self) -> &str;

    /// Clone as boxed trait object
    fn clone_box(&self) -> Box<dyn PriorConstructor>;
}
