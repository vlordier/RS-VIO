//! Concrete implementations of Hessian, Gradient, and Prior constructors

use nalgebra as na;
use na::{DMatrix, DVector};
use std::collections::HashMap;

use super::config::{MarginalizationConfig, ParamId};
use super::traits::{GradientComputer, HessianApproximator, PriorConstructor};
use crate::MarginalizationPrior;

// ============================================================================
// Concrete Implementations: Hessian Approximators
// ============================================================================

/// Gauss-Newton Hessian approximation: H = J^T * J
///
/// This is the standard least-squares approximation when Jacobians are available.
/// When Jacobians are not available, uses a scaled identity matrix.
#[derive(Debug, Clone)]
pub struct GaussNewtonApproximator {
    fallback_scale: f64,
}

impl GaussNewtonApproximator {
    pub fn new(fallback_scale: f64) -> Self {
        Self { fallback_scale }
    }
}

impl Default for GaussNewtonApproximator {
    fn default() -> Self {
        Self::new(1e-6)
    }
}

impl HessianApproximator for GaussNewtonApproximator {
    fn compute_hessian(
        &self,
        residuals: &DVector<f64>,
        param_dim: usize,
        jacobians: Option<&Vec<DMatrix<f64>>>,
    ) -> DMatrix<f64> {
        if let Some(jacs) = jacobians {
            if jacs.is_empty() {
                return DMatrix::zeros(param_dim, param_dim);
            }
            let mut H = DMatrix::zeros(jacs[0].ncols(), jacs[0].ncols());
            for J in jacs {
                let Jt = J.transpose();
                H = H + &Jt * J;
            }
            H
        } else {
            let r_norm_sq = residuals.dot(residuals);
            let r_mean = (r_norm_sq / residuals.len().max(1) as f64).sqrt();
            DMatrix::identity(param_dim, param_dim) * r_mean.max(self.fallback_scale)
        }
    }

    fn name(&self) -> &str {
        "GaussNewton"
    }

    fn clone_box(&self) -> Box<dyn HessianApproximator> {
        Box::new(self.clone())
    }
}

/// Diagonal Hessian approximation (only diagonal elements)
///
/// This is a fast approximation that only uses diagonal elements of J^T * J.
/// Useful for large-scale problems where memory is constrained.
#[derive(Debug, Clone)]
pub struct DiagonalApproximator {
    min_diagonal: f64,
}

impl DiagonalApproximator {
    pub fn new(min_diagonal: f64) -> Self {
        Self { min_diagonal }
    }
}

impl Default for DiagonalApproximator {
    fn default() -> Self {
        Self::new(1e-8)
    }
}

impl HessianApproximator for DiagonalApproximator {
    fn compute_hessian(
        &self,
        residuals: &DVector<f64>,
        param_dim: usize,
        jacobians: Option<&Vec<DMatrix<f64>>>,
    ) -> DMatrix<f64> {
        if let Some(jacs) = jacobians {
            if jacs.is_empty() {
                return DMatrix::identity(param_dim, param_dim) * self.min_diagonal;
            }
            let mut diag: DVector<f64> = DVector::zeros(jacs[0].ncols());
            for J in jacs {
                for i in 0..J.nrows() {
                    for j in 0..J.ncols() {
                        diag[j] += J[(i, j)] * J[(i, j)];
                    }
                }
            }
            // Apply min_diagonal threshold without closure monomorphization
            for i in 0..diag.len() {
                diag[i] = diag[i].max(self.min_diagonal);
            }
            DMatrix::from_diagonal(&diag)
        } else {
            let r_mean = residuals.norm() / residuals.len() as f64;
            DMatrix::identity(param_dim, param_dim) * r_mean.max(self.min_diagonal)
        }
    }

    fn name(&self) -> &str {
        "Diagonal"
    }

    fn clone_box(&self) -> Box<dyn HessianApproximator> {
        Box::new(self.clone())
    }
}

/// Levenberg-Marquardt Hessian approximation: H + λI
///
/// This adds adaptive damping to improve numerical stability when the
/// Hessian is poorly conditioned.
#[derive(Debug)]
pub struct LevenbergMarquardtApproximator {
    base_approximator: Box<dyn HessianApproximator>,
    damping_adaptation: f64,
}

impl LevenbergMarquardtApproximator {
    pub fn new(base_approximator: Box<dyn HessianApproximator>, damping_adaptation: f64) -> Self {
        Self {
            base_approximator,
            damping_adaptation,
        }
    }
}

impl Clone for LevenbergMarquardtApproximator {
    fn clone(&self) -> Self {
        Self {
            base_approximator: self.base_approximator.clone_box(),
            damping_adaptation: self.damping_adaptation,
        }
    }
}

impl Default for LevenbergMarquardtApproximator {
    fn default() -> Self {
        Self::new(Box::new(GaussNewtonApproximator::default()), 1.0)
    }
}

impl HessianApproximator for LevenbergMarquardtApproximator {
    fn compute_hessian(
        &self,
        residuals: &DVector<f64>,
        param_dim: usize,
        jacobians: Option<&Vec<DMatrix<f64>>>,
    ) -> DMatrix<f64> {
        let base_hessian = self
            .base_approximator
            .compute_hessian(residuals, param_dim, jacobians);
        let r_mean = residuals.norm() / residuals.len() as f64;
        let damping = (r_mean * r_mean).max(1e-6) * self.damping_adaptation;

        let mut H_lm = base_hessian.clone();
        for i in 0..H_lm.nrows() {
            H_lm[(i, i)] += damping;
        }
        H_lm
    }

    fn name(&self) -> &str {
        "LevenbergMarquardt"
    }

    fn clone_box(&self) -> Box<dyn HessianApproximator> {
        Box::new(self.clone())
    }
}

/// Identity Hessian approximation (scaled identity only)
///
/// This is the fastest approximation but least accurate.
/// Primarily used for debugging or as a fallback.
#[derive(Debug, Clone, Default)]
pub struct IdentityApproximator;

impl HessianApproximator for IdentityApproximator {
    fn compute_hessian(
        &self,
        residuals: &DVector<f64>,
        param_dim: usize,
        _jacobians: Option<&Vec<DMatrix<f64>>>,
    ) -> DMatrix<f64> {
        let r_mean = residuals.norm() / residuals.len() as f64;
        DMatrix::identity(param_dim, param_dim) * r_mean.max(1e-6)
    }

    fn name(&self) -> &str {
        "Identity"
    }

    fn clone_box(&self) -> Box<dyn HessianApproximator> {
        Box::new(self.clone())
    }
}

/// Exact Hessian from optimizer (when available)
#[derive(Debug, Clone, Default)]
pub struct ExactHessianApproximator;

impl HessianApproximator for ExactHessianApproximator {
    fn compute_hessian(
        &self,
        residuals: &DVector<f64>,
        param_dim: usize,
        jacobians: Option<&Vec<DMatrix<f64>>>,
    ) -> DMatrix<f64> {
        if let Some(jacs) = jacobians {
            if jacs.is_empty() {
                return DMatrix::zeros(param_dim, param_dim);
            }
            let mut H = DMatrix::zeros(jacs[0].ncols(), jacs[0].ncols());
            for J in jacs {
                let Jt = J.transpose();
                H = H + &Jt * J;
            }
            H
        } else {
            IdentityApproximator.compute_hessian(residuals, param_dim, None)
        }
    }

    fn name(&self) -> &str {
        "Exact"
    }

    fn clone_box(&self) -> Box<dyn HessianApproximator> {
        Box::new(self.clone())
    }
}

// ============================================================================
// Concrete Implementations: Gradient Computers
// ============================================================================

/// Standard gradient computer: g = J^T * r
#[derive(Debug, Clone, Default)]
pub struct StandardGradientComputer;

impl GradientComputer for StandardGradientComputer {
    fn compute_gradient(
        &self,
        residuals: &DVector<f64>,
        param_dim: usize,
        jacobians: Option<&Vec<DMatrix<f64>>>,
    ) -> DVector<f64> {
        if let Some(jacs) = jacobians {
            if jacs.is_empty() {
                return DVector::zeros(param_dim);
            }
            let mut g = DVector::zeros(jacs[0].ncols());
            for J in jacs {
                g = g + J.transpose() * residuals;
            }
            g
        } else {
            let r_mean = residuals.sum() / residuals.len() as f64;
            DVector::from_element(param_dim, r_mean)
        }
    }

    fn name(&self) -> &str {
        "Standard"
    }

    fn clone_box(&self) -> Box<dyn GradientComputer> {
        Box::new(self.clone())
    }
}

/// Zero gradient (for debugging)
#[derive(Debug, Clone, Default)]
pub struct ZeroGradientComputer;

impl GradientComputer for ZeroGradientComputer {
    fn compute_gradient(
        &self,
        _residuals: &DVector<f64>,
        param_dim: usize,
        _jacobians: Option<&Vec<DMatrix<f64>>>,
    ) -> DVector<f64> {
        DVector::zeros(param_dim)
    }

    fn name(&self) -> &str {
        "Zero"
    }

    fn clone_box(&self) -> Box<dyn GradientComputer> {
        Box::new(self.clone())
    }
}

// ============================================================================
// Prior Constructors
// ============================================================================

/// Standard prior constructor with Schur complement
#[derive(Debug, Clone, Default)]
pub struct StandardPriorConstructor;

impl PriorConstructor for StandardPriorConstructor {
    fn construct_prior(
        &self,
        schur_complement: &DMatrix<f64>,
        reduced_gradient: &DVector<f64>,
        param_ids: &[ParamId],
        residual_dim: usize,
        config: &MarginalizationConfig,
        linearization_points: &HashMap<ParamId, DVector<f64>>,
    ) -> Option<MarginalizationPrior> {
        // Scale information matrix
        let info = schur_complement.clone() * config.prior_info_scale;

        // Filter linearization points to only include kept parameters
        let filtered_lin_points: HashMap<ParamId, DVector<f64>> = param_ids
            .iter()
            .filter_map(|id| {
                linearization_points.get(id).map(|lp| (id.clone(), lp.clone()))
            })
            .collect();

        Some(MarginalizationPrior {
            param_ids: param_ids.to_vec(),
            residual_dim,
            residual: reduced_gradient.clone(),
            information: info,
            damping: config.damping,
            linearization_points: filtered_lin_points,
        })
    }

    fn name(&self) -> &str {
        "Standard"
    }

    fn clone_box(&self) -> Box<dyn PriorConstructor> {
        Box::new(self.clone())
    }
}

/// Prior constructor with eigenvalue regularization
#[derive(Debug, Clone)]
pub struct RegularizedPriorConstructor {
    min_eigenvalue: f64,
}

impl RegularizedPriorConstructor {
    pub fn new(min_eigenvalue: f64) -> Self {
        Self { min_eigenvalue }
    }
}

impl Default for RegularizedPriorConstructor {
    fn default() -> Self {
        Self::new(1e-6)
    }
}

impl PriorConstructor for RegularizedPriorConstructor {
    fn construct_prior(
        &self,
        schur_complement: &DMatrix<f64>,
        reduced_gradient: &DVector<f64>,
        param_ids: &[ParamId],
        residual_dim: usize,
        config: &MarginalizationConfig,
        linearization_points: &HashMap<ParamId, DVector<f64>>,
    ) -> Option<MarginalizationPrior> {
        // Add regularization to ensure positive definiteness
        let n = schur_complement.nrows();
        let mut info = schur_complement.clone() * config.prior_info_scale;

        for i in 0..n {
            info[(i, i)] = info[(i, i)].max(self.min_eigenvalue);
        }

        Some(MarginalizationPrior {
            param_ids: param_ids.to_vec(),
            residual_dim,
            residual: reduced_gradient.clone(),
            information: info,
            damping: config.damping,
            linearization_points: linearization_points.clone(),
        })
    }

    fn name(&self) -> &str {
        "Regularized"
    }

    fn clone_box(&self) -> Box<dyn PriorConstructor> {
        Box::new(self.clone())
    }
}
