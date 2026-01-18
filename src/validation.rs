//! Lightweight validation helpers for numerical robustness
//!
//! These helpers are intentionally small and allocation-free to keep
//! runtime overhead low while catching obvious numerical issues early.

use crate::types::{Float, Matrix4x4, Vector3};

/// Divide two numbers with explicit checks for non-finite inputs and zero divisors.
pub fn safe_divide(num: Float, denom: Float) -> Result<Float, String> {
    if !num.is_finite() || !denom.is_finite() {
        return Err("inputs must be finite".into());
    }
    if denom.abs() < 1e-12 {
        return Err("division by zero".into());
    }
    Ok(num / denom)
}

/// Approximate equality with a tolerance that scales with magnitude.
pub fn approx_equal(a: Float, b: Float) -> bool {
    let tol = 1e-9 * (1.0 + a.abs() + b.abs());
    (a - b).abs() <= tol
}

/// Validate that all vector components are finite.
pub fn validate_finite_vector(v: &Vector3) -> Result<(), String> {
    if v.iter().all(|c| c.is_finite()) {
        Ok(())
    } else {
        Err("vector contains non-finite components".into())
    }
}

/// Validate that all matrix elements are finite.
pub fn validate_finite_matrix(m: &Matrix4x4) -> Result<(), String> {
    if m.iter().all(|c| c.is_finite()) {
        Ok(())
    } else {
        Err("matrix contains non-finite elements".into())
    }
}
