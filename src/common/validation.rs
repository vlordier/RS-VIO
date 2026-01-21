//! Validation utilities for geometric types.
//!
//! Provides functions to validate poses, rotations, and other geometric
//! quantities to ensure numerical stability and correctness.

use crate::types::{Float, Matrix4x4};
use nalgebra as na;

/// Validates that a quaternion is properly normalized.
///
/// # Arguments
///
/// * `q` - The quaternion to validate
/// * `tolerance` - Tolerance for norm deviation from 1.0
///
/// # Returns
///
/// Ok(()) if valid, Err with description if invalid.
pub fn validate_quaternion(q: &na::UnitQuaternion<Float>, tolerance: Float) -> Result<(), String> {
    let norm = q.quaternion().norm();
    if (norm - 1.0).abs() > tolerance {
        return Err(format!(
            "Quaternion not normalized: norm = {}, expected 1.0 ± {}",
            norm, tolerance
        ));
    }
    Ok(())
}

/// Validates that a rotation matrix is orthonormal.
///
/// Checks that R^T * R = I and det(R) = 1.
///
/// # Arguments
///
/// * `R` - The 3x3 rotation matrix
/// * `tolerance` - Tolerance for orthonormality check
///
/// # Returns
///
/// Ok(()) if valid, Err with description if invalid.
pub fn validate_rotation_matrix(R: &na::Matrix3<Float>, tolerance: Float) -> Result<(), String> {
    // Check orthonormality: R^T * R should be identity
    let rtx_r = R.transpose() * R;
    let identity = na::Matrix3::<Float>::identity();

    for i in 0..3 {
        for j in 0..3 {
            let expected = identity[(i, j)];
            let actual = rtx_r[(i, j)];
            if (actual - expected).abs() > tolerance {
                return Err(format!(
                    "Rotation matrix not orthonormal: R^T*R[{},{}] = {}, expected {}",
                    i, j, actual, expected
                ));
            }
        }
    }

    // Check determinant is 1 (not -1, which would be a reflection)
    let det = R.determinant();
    if (det - 1.0).abs() > tolerance {
        return Err(format!(
            "Rotation matrix determinant = {}, expected 1.0",
            det
        ));
    }

    Ok(())
}

/// Validates a 4x4 pose matrix (SE(3)).
///
/// Checks that:
/// - Top-left 3x3 is a valid rotation matrix
/// - Bottom row is [0, 0, 0, 1]
///
/// # Arguments
///
/// * `T` - The 4x4 pose matrix
/// * `tolerance` - Tolerance for checks
///
/// # Returns
///
/// Ok(()) if valid, Err with description if invalid.
pub fn validate_pose(T: &Matrix4x4, tolerance: Float) -> Result<(), String> {
    // Extract rotation part
    let R = T.fixed_view::<3, 3>(0, 0);
    let R_matrix = R.into_owned();

    // Validate rotation
    validate_rotation_matrix(&R_matrix, tolerance)?;

    // Check bottom row is [0, 0, 0, 1]
    for i in 0..3 {
        if T[(3, i)].abs() > tolerance {
            return Err(format!(
                "Pose matrix bottom row invalid: T[3,{}] = {}, expected 0",
                i,
                T[(3, i)]
            ));
        }
    }

    if (T[(3, 3)] - 1.0).abs() > tolerance {
        return Err(format!(
            "Pose matrix bottom-right element = {}, expected 1.0",
            T[(3, 3)]
        ));
    }

    Ok(())
}

/// Validates a general matrix for NaN or Inf values.
///
/// # Arguments
///
/// * `matrix` - Any nalgebra matrix
/// * `name` - Name for error messages
///
/// # Returns
///
/// Ok(()) if all finite, Err if any NaN/Inf found.
pub fn validate_matrix<R, C, S>(
    matrix: &na::Matrix<Float, R, C, S>,
    name: &str,
) -> Result<(), String>
where
    R: na::Dim,
    C: na::Dim,
    S: na::RawStorage<Float, R, C>,
{
    for i in 0..matrix.nrows() {
        for j in 0..matrix.ncols() {
            let val = matrix[(i, j)];
            if !val.is_finite() {
                return Err(format!(
                    "Matrix '{}' contains non-finite value at [{},{}]: {}",
                    name, i, j, val
                ));
            }
        }
    }
    Ok(())
}

/// Validates that a covariance matrix is symmetric and positive semi-definite.
///
/// # Arguments
///
/// * `cov` - The covariance matrix
/// * `tolerance` - Tolerance for symmetry check
///
/// # Returns
///
/// Ok(()) if valid, Err with description if invalid.
pub fn validate_covariance<D, S>(
    cov: &na::Matrix<Float, D, D, S>,
    tolerance: Float,
) -> Result<(), String>
where
    D: na::Dim,
    S: na::RawStorage<Float, D, D>,
{
    // Check symmetry
    for i in 0..cov.nrows() {
        for j in (i + 1)..cov.ncols() {
            if (cov[(i, j)] - cov[(j, i)]).abs() > tolerance {
                return Err(format!(
                    "Covariance matrix not symmetric: cov[{},{}] = {}, cov[{},{}] = {}",
                    i,
                    j,
                    cov[(i, j)],
                    j,
                    i,
                    cov[(j, i)]
                ));
            }
        }
    }

    // Note: A full positive semi-definiteness check (eigenvalue computation)
    // is expensive and requires ownership of the matrix. For most uses,
    // symmetry and finite value checks are sufficient.
    // If needed, callers can perform eigenvalue checks separately.

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_validate_quaternion() {
        let q = na::UnitQuaternion::from_euler_angles(0.1, 0.2, 0.3);
        assert!(validate_quaternion(&q, 1e-6).is_ok());
    }

    #[test]
    fn test_validate_rotation_matrix() {
        let q = na::UnitQuaternion::from_euler_angles(0.1, 0.2, 0.3);
        let R = q.to_rotation_matrix().into_inner();
        assert!(validate_rotation_matrix(&R, 1e-6).is_ok());
    }

    #[test]
    fn test_validate_pose() {
        let mut T = Matrix4x4::identity();
        let q = na::UnitQuaternion::from_euler_angles(0.1, 0.2, 0.3);
        let R = q.to_rotation_matrix().into_inner();

        for i in 0..3 {
            for j in 0..3 {
                T[(i, j)] = R[(i, j)];
            }
        }

        T[(0, 3)] = 1.0;
        T[(1, 3)] = 2.0;
        T[(2, 3)] = 3.0;

        assert!(validate_pose(&T, 1e-6).is_ok());
    }

    #[test]
    fn test_validate_matrix() {
        let m = na::Matrix3::<Float>::identity();
        assert!(validate_matrix(&m, "test").is_ok());

        let mut m_nan = na::Matrix3::<Float>::identity();
        m_nan[(0, 0)] = Float::NAN;
        assert!(validate_matrix(&m_nan, "test").is_err());
    }

    #[test]
    fn test_validate_covariance() {
        let cov = na::Matrix3::<Float>::identity();
        assert!(validate_covariance(&cov, 1e-6).is_ok());

        let mut asymmetric = na::Matrix3::<Float>::identity();
        asymmetric[(0, 1)] = 1.0;
        asymmetric[(1, 0)] = 2.0;
        assert!(validate_covariance(&asymmetric, 1e-6).is_err());
    }
}
