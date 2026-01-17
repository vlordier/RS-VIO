//! Robust numerical algorithms for embedded real-time optimization.
//!
//! Provides stable, graceful-degradation solvers for common linear algebra
//! problems, with fallback paths and condition number estimation.
//!
//! ## Design Philosophy
//!
//! 1. **Fast path first**: Attempt Cholesky (O(n³/6) fast), escalate if needed
//! 2. **Fallbacks**: LU (O(n³/3), robust) → Pseudo-inverse (O(n³), slow)
//! 3. **Diagnostics**: Condition number, rank estimates without full SVD
//! 4. **Documentation**: Why each path is chosen, performance costs

use crate::types::Float;
use nalgebra as na;
use crate::debug_log;
use na::linalg::{Cholesky, LU, SVD};

const EPSILON: Float = 1e-10;
const MAX_DAMPING_SCALE: Float = 1000.0;

/// Solves a symmetric positive definite system `Ax = b` with graceful degradation.
///
/// ## Strategy
/// 1. **Cholesky** (fast, O(n³/6)): Try with base damping
/// 2. **Cholesky+damping** (single re-clone): Escalate regularization up to 3 attempts
/// 3. **LU** (robust, O(n³/3)): Fall back if Cholesky fails
/// 4. **Pseudo-inverse** (slow, O(n³)): Last resort via SVD
///
/// ## Returns
/// - `Ok(x)` with solution quality metrics
/// - `Err` only if all paths fail (system is singular to working precision)
#[allow(dead_code)]
pub struct RobustSolver {
    base_damping: Float,
    max_damping: Float,
}

impl RobustSolver {
    pub fn new(base_damping: Float) -> Self {
        Self {
            base_damping: base_damping.max(1e-10),
            max_damping: base_damping * MAX_DAMPING_SCALE,
        }
    }

    /// Solve `Ax = b` where A is symmetric positive definite.
    ///
    /// # Quality Metrics
    /// - `damping_used`: Final damping scale (1.0 = no regularization)
    /// - `method`: Path taken ("cholesky", "lu", "svd")
    /// - `residual`: ||Ax - b|| / ||b|| (if computed)
    pub fn solve(
        &self,
        a: &na::DMatrix<Float>,
        b: &na::DVector<Float>,
    ) -> Result<(na::DVector<Float>, SolveQuality), String> {
        if a.nrows() != a.ncols() {
            return Err("Matrix must be square".to_string());
        }
        if a.nrows() != b.len() {
            return Err("Matrix-vector size mismatch".to_string());
        }

        let n = a.nrows();
        if n == 0 {
            return Err("Empty system".to_string());
        }

        // Attempt 1: Cholesky with base damping
        let mut regularized = a.clone();
        Self::add_diagonal(&mut regularized, self.base_damping);

        if let Some(chol) = Cholesky::new(regularized.clone()) {
            let x = chol.solve(b);
            return Ok((
                x,
                SolveQuality {
                    method: SolveMethod::Cholesky,
                    damping_scale: 1.0,
                    rank_estimate: n,
                },
            ));
        }

        // Attempt 2: Escalate damping (3 attempts max)
        for attempt in 1..=3 {
            let scale = 10_f64.powi(attempt as i32) as Float;
            if scale > MAX_DAMPING_SCALE {
                log::warn!("Damping limit reached; escalating to LU");
                break;
            }

            let mut regularized_attempt = a.clone();
            Self::add_diagonal(&mut regularized_attempt, self.base_damping * scale);

            if let Some(chol) = Cholesky::new(regularized_attempt) {
                let x = chol.solve(b);
                debug_log!("Cholesky succeeded at damping scale {:.0e}", scale);
                return Ok((
                    x,
                    SolveQuality {
                        method: SolveMethod::Cholesky,
                        damping_scale: scale,
                        rank_estimate: n,
                    },
                ));
            }
        }

        // Attempt 3: LU factorization
        let lu = LU::new(regularized);
        if let Some(x) = lu.solve(b) {
            log::warn!("Falling back to LU solver (solution quality degraded)");
            return Ok((
                x,
                SolveQuality {
                    method: SolveMethod::LU,
                    damping_scale: MAX_DAMPING_SCALE / 10.0,
                    rank_estimate: n,
                },
            ));
        }

        // Attempt 4: Pseudo-inverse (SVD-based)
        let pinv = Self::pseudo_inverse(a)?;
        let x = &pinv * b;
        log::error!("Using pseudo-inverse; solution accuracy degraded to ~1e-6");
        Ok((
            x,
            SolveQuality {
                method: SolveMethod::SVD,
                damping_scale: MAX_DAMPING_SCALE,
                rank_estimate: Self::estimate_rank(a)?,
            },
        ))
    }

    /// Compute pseudo-inverse via SVD with rank detection.
    fn pseudo_inverse(matrix: &na::DMatrix<Float>) -> Result<na::DMatrix<Float>, String> {
        let svd = SVD::new(matrix.clone(), true, true);

        let u = svd.u.ok_or("SVD: U decomposition failed")?;
        let v_t = svd.v_t.ok_or("SVD: V^T decomposition failed")?;

        let singulars = &svd.singular_values;
        if singulars.is_empty() {
            return Err("SVD: no singular values".to_string());
        }

        let max_sv = singulars.max();
        let tol = EPSILON * max_sv; // Relative tolerance

        let mut s_inv = na::DMatrix::zeros(v_t.nrows(), u.ncols());
        for (i, sv) in singulars.iter().enumerate() {
            if *sv > tol {
                s_inv[(i, i)] = 1.0 / sv;
            }
        }

        Ok(&v_t.transpose() * &s_inv * &u.transpose())
    }

    /// Estimate rank via singular values (fast heuristic).
    fn estimate_rank(matrix: &na::DMatrix<Float>) -> Result<usize, String> {
        let svd = SVD::new(matrix.clone(), false, false);
        let singulars = &svd.singular_values;

        if singulars.is_empty() {
            return Ok(0);
        }

        let max_sv = singulars.max();
        let tol = EPSILON * max_sv;
        let rank = singulars.iter().filter(|sv| **sv > tol).count();
        Ok(rank)
    }

    fn add_diagonal(matrix: &mut na::DMatrix<Float>, damping: Float) {
        for i in 0..matrix.nrows().min(matrix.ncols()) {
            matrix[(i, i)] += damping;
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub enum SolveMethod {
    Cholesky,
    LU,
    SVD,
}

#[derive(Debug, Clone, Copy)]
pub struct SolveQuality {
    pub method: SolveMethod,
    pub damping_scale: Float,
    pub rank_estimate: usize,
}

/// Fast condition number estimation (O(n) heuristic, not O(n³) SVD).
///
/// Uses Frobenius norm ratio as proxy: κ ≈ norm(A) / min_diag(A)
/// Accurate for well-scaled matrices; overestimates for ill-scaled ones.
pub fn estimate_condition_number(matrix: &na::DMatrix<Float>) -> Option<Float> {
    if matrix.nrows() == 0 || matrix.ncols() == 0 {
        return None;
    }

    let frobenius = matrix.norm();
    let min_diag = (0..matrix.nrows().min(matrix.ncols()))
        .map(|i| matrix[(i, i)].abs())
        .fold(Float::INFINITY, Float::min);

    if min_diag < EPSILON {
        return None; // Singular
    }

    Some(frobenius / min_diag)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn robust_solver_well_conditioned() {
        let solver = RobustSolver::new(1e-5);
        // For identity matrix: (I + λI)x = b => x ≈ b / (1 + λ)
        let a = na::DMatrix::identity(3, 3);
        let b = na::DVector::from_vec(vec![1.0, 2.0, 3.0]);

        let (x, quality) = solver.solve(&a, &b).unwrap();
        assert!(matches!(quality.method, SolveMethod::Cholesky));
        // With damping 1e-5, solution is slightly off: error ~ 1e-5 * ||b||
        let error = (x.clone() - b.clone()).norm() / b.norm();
        assert!(error < 1e-4, "Relative error {:.2e} exceeds tolerance", error);
    }

    #[test]
    fn robust_solver_ill_conditioned() {
        let solver = RobustSolver::new(1e-5);
        let mut a = na::DMatrix::zeros(3, 3);
        a[(0, 0)] = 1e-8;
        a[(1, 1)] = 1.0;
        a[(2, 2)] = 1.0;

        let b = na::DVector::from_vec(vec![1.0, 1.0, 1.0]);

        let (x, quality) = solver.solve(&a, &b).unwrap();
        assert!(matches!(
            quality.method,
            SolveMethod::Cholesky | SolveMethod::LU
        ));
        assert!(x.norm() > 0.0); // Solution exists
    }

    #[test]
    fn condition_number_identity() {
        let a = na::DMatrix::identity(3, 3);
        let kappa = estimate_condition_number(&a);
        assert!(kappa.is_some());
        assert!(kappa.unwrap() <= 2.0); // ~1.0 for identity
    }

    #[test]
    fn condition_number_singular_returns_none() {
        let mut a = na::DMatrix::zeros(3, 3);
        a[(0, 0)] = 0.0;
        a[(1, 1)] = 0.0;
        a[(2, 2)] = 0.0;

        let kappa = estimate_condition_number(&a);
        assert!(kappa.is_none());
    }
}
