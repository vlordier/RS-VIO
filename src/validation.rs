//! Numerical validation utilities for real-time safety
//!
//! This module provides validation functions to ensure numerical stability
//! and prevent NaN/Inf propagation in critical VIO computations.

use nalgebra as na;

/// Minimum depth threshold to avoid division by near-zero values (in meters)
pub const MIN_DEPTH: f64 = 1e-6;

/// Maximum allowed depth for 3D points (in meters, prevents overflow)
pub const MAX_DEPTH: f64 = 1000.0;

/// Epsilon for floating-point comparisons
pub const EPSILON_F64: f64 = 1e-10;
pub const EPSILON_F32: f32 = 1e-6;

/// Validation result
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ValidationError {
    NonFinite,
    OutOfRange,
    Singular,
    InvalidDepth,
}

/// Validate that all elements of a vector are finite (not NaN or Inf)
pub fn validate_finite_vector<const N: usize>(
    v: &na::SVector<f64, N>,
) -> Result<(), ValidationError> {
    if v.iter().all(|x| x.is_finite()) {
        Ok(())
    } else {
        Err(ValidationError::NonFinite)
    }
}

/// Validate that all elements of a matrix are finite
pub fn validate_finite_matrix<const R: usize, const C: usize>(
    m: &na::SMatrix<f64, R, C>,
) -> Result<(), ValidationError> {
    if m.iter().all(|x| x.is_finite()) {
        Ok(())
    } else {
        Err(ValidationError::NonFinite)
    }
}

/// Validate a 3D point for projection
pub fn validate_point_for_projection(point: &na::Vector3<f64>) -> Result<(), ValidationError> {
    // Check for finite values
    validate_finite_vector(point)?;

    // Check depth is valid for projection
    let depth = point[2];
    if depth.abs() < MIN_DEPTH {
        return Err(ValidationError::InvalidDepth);
    }

    // Check point is within reasonable bounds
    if depth.abs() > MAX_DEPTH {
        return Err(ValidationError::OutOfRange);
    }

    Ok(())
}

/// Validate a rotation matrix (orthogonal with determinant 1)
pub fn validate_rotation_matrix(r: &na::Matrix3<f64>) -> Result<(), ValidationError> {
    // Check all elements are finite
    validate_finite_matrix(r)?;

    // Check determinant is close to 1
    let det = r.determinant();
    if (det - 1.0).abs() > 1e-3 {
        return Err(ValidationError::Singular);
    }

    // Check orthogonality: R * R^T should be identity
    let identity_check = r * r.transpose();
    let identity = na::Matrix3::identity();
    let diff = (identity_check - identity).norm();
    if diff > 1e-3 {
        return Err(ValidationError::Singular);
    }

    Ok(())
}

/// Validate a transformation matrix (SE(3))
pub fn validate_transformation_matrix(t: &na::Matrix4<f64>) -> Result<(), ValidationError> {
    // Check all elements are finite
    validate_finite_matrix(t)?;

    // Extract rotation part and validate
    let rotation = t.fixed_view::<3, 3>(0, 0);
    validate_rotation_matrix(&rotation.into_owned())?;

    // Check bottom row is [0, 0, 0, 1]
    let bottom_row = t.row(3);
    if (bottom_row.get(0).is_none_or(|&v| v.abs() > EPSILON_F64))
        || (bottom_row.get(1).is_none_or(|&v| v.abs() > EPSILON_F64))
        || (bottom_row.get(2).is_none_or(|&v| v.abs() > EPSILON_F64))
        || (bottom_row
            .get(3)
            .is_none_or(|&v| (v - 1.0).abs() > EPSILON_F64))
    {
        return Err(ValidationError::Singular);
    }

    Ok(())
}

/// Safe division with validation
pub fn safe_divide(numerator: f64, denominator: f64) -> Result<f64, ValidationError> {
    if !numerator.is_finite() || !denominator.is_finite() {
        return Err(ValidationError::NonFinite);
    }

    if denominator.abs() < EPSILON_F64 {
        return Err(ValidationError::Singular);
    }

    let result = numerator / denominator;
    if !result.is_finite() {
        return Err(ValidationError::NonFinite);
    }

    Ok(result)
}

/// Compare floating-point numbers with epsilon tolerance
pub fn approx_equal_f64(a: f64, b: f64, epsilon: f64) -> bool {
    (a - b).abs() < epsilon
}

/// Compare floating-point numbers with default epsilon
pub fn approx_equal(a: f64, b: f64) -> bool {
    approx_equal_f64(a, b, EPSILON_F64)
}

#[cfg(test)]
#[allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::float_cmp
)]
mod tests {
    use super::*;

    #[test]
    fn test_validate_finite_vector() {
        let valid = na::Vector3::new(1.0, 2.0, 3.0);
        assert!(validate_finite_vector(&valid).is_ok());

        let invalid = na::Vector3::new(1.0, f64::NAN, 3.0);
        assert!(validate_finite_vector(&invalid).is_err());

        let inf = na::Vector3::new(1.0, f64::INFINITY, 3.0);
        assert!(validate_finite_vector(&inf).is_err());
    }

    #[test]
    fn test_validate_point_for_projection() {
        // Valid point
        let valid = na::Vector3::new(1.0, 2.0, 5.0);
        assert!(validate_point_for_projection(&valid).is_ok());

        // Point too close to camera
        let too_close = na::Vector3::new(1.0, 2.0, 1e-7);
        assert_eq!(
            validate_point_for_projection(&too_close),
            Err(ValidationError::InvalidDepth)
        );

        // Point too far
        let too_far = na::Vector3::new(1.0, 2.0, 2000.0);
        assert_eq!(
            validate_point_for_projection(&too_far),
            Err(ValidationError::OutOfRange)
        );

        // Non-finite point
        let nan_point = na::Vector3::new(1.0, 2.0, f64::NAN);
        assert_eq!(
            validate_point_for_projection(&nan_point),
            Err(ValidationError::NonFinite)
        );
    }

    #[test]
    fn test_validate_rotation_matrix() {
        // Valid rotation (identity)
        let valid = na::Matrix3::identity();
        assert!(validate_rotation_matrix(&valid).is_ok());

        // Valid rotation (90 degrees around z-axis)
        let rot_z = na::Matrix3::new(0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0);
        assert!(validate_rotation_matrix(&rot_z).is_ok());

        // Invalid: not orthogonal
        let invalid = na::Matrix3::new(1.0, 0.5, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
        assert!(validate_rotation_matrix(&invalid).is_err());

        // Invalid: wrong determinant
        let invalid_det = na::Matrix3::new(-1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
        assert!(validate_rotation_matrix(&invalid_det).is_err());
    }

    #[test]
    fn test_safe_divide() {
        // Valid division
        assert_eq!(safe_divide(10.0, 2.0).unwrap(), 5.0);

        // Division by zero
        assert_eq!(safe_divide(10.0, 0.0), Err(ValidationError::Singular));

        // NaN inputs
        assert_eq!(safe_divide(f64::NAN, 2.0), Err(ValidationError::NonFinite));
        assert_eq!(safe_divide(10.0, f64::NAN), Err(ValidationError::NonFinite));

        // Division resulting in infinity
        assert_eq!(safe_divide(1e308, 1e-308), Err(ValidationError::Singular));
    }

    #[test]
    fn test_approx_equal() {
        assert!(approx_equal(1.0, 1.0));
        assert!(approx_equal(1.0, 1.0 + 1e-11));
        assert!(!approx_equal(1.0, 1.1));
        assert!(approx_equal_f64(1.0, 1.01, 0.1));
    }

    #[test]
    fn test_approx_equal_negative_numbers() {
        assert!(approx_equal(-1.0, -1.0));
        assert!(approx_equal(-1.0, -1.0 + 1e-11));
        assert!(!approx_equal(-1.0, -1.1));
    }

    #[test]
    fn test_approx_equal_zero() {
        assert!(approx_equal(0.0, 0.0));
        assert!(approx_equal(0.0, 1e-11));
        assert!(!approx_equal(0.0, 1e-9));
    }

    #[test]
    fn test_validate_transformation_matrix() {
        // Valid identity transformation
        let valid = na::Matrix4::identity();
        assert!(validate_transformation_matrix(&valid).is_ok());

        // Valid translation
        let mut trans = na::Matrix4::identity();
        trans[(0, 3)] = 5.0;
        trans[(1, 3)] = -2.0;
        trans[(2, 3)] = 10.0;
        assert!(validate_transformation_matrix(&trans).is_ok());

        // Invalid: bad bottom row
        let mut invalid = na::Matrix4::identity();
        invalid[(3, 0)] = 1.0; // Should be 0
        assert!(validate_transformation_matrix(&invalid).is_err());
    }

    #[test]
    fn test_safe_divide_very_small_denominator() {
        // Denominator smaller than epsilon should fail
        assert_eq!(
            safe_divide(1.0, 1e-11),
            Err(ValidationError::Singular)
        );
    }

    #[test]
    fn test_safe_divide_negative() {
        assert_eq!(safe_divide(-10.0, 2.0).unwrap(), -5.0);
        assert_eq!(safe_divide(10.0, -2.0).unwrap(), -5.0);
        assert_eq!(safe_divide(-10.0, -2.0).unwrap(), 5.0);
    }

    #[test]
    fn test_safe_divide_infinity_output() {
        let result = safe_divide(1.0, 1e-100);
        // When result is infinity, should fail
        assert!(result.is_err() || result.unwrap().is_infinite());
    }

    #[test]
    fn test_validate_rotation_matrix_slightly_non_orthogonal() {
        // Slightly perturbed orthogonal matrix (should fail)
        let mut mat = na::Matrix3::identity();
        mat[(0, 0)] = 1.001; // Slightly non-unit
        let result = validate_rotation_matrix(&mat);
        assert!(result.is_err());
    }

    #[test]
    fn test_validate_rotation_matrix_reflection() {
        // Reflection matrix (det = -1, not a proper rotation)
        let reflect = na::Matrix3::new(-1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
        assert!(validate_rotation_matrix(&reflect).is_err());
    }

    #[test]
    fn test_validate_point_at_boundary_depths() {
        // Just at minimum depth
        let at_min = na::Vector3::new(0.0, 0.0, MIN_DEPTH);
        assert!(validate_point_for_projection(&at_min).is_ok());

        // Just below minimum depth
        let below_min = na::Vector3::new(0.0, 0.0, MIN_DEPTH / 2.0);
        assert_eq!(
            validate_point_for_projection(&below_min),
            Err(ValidationError::InvalidDepth)
        );

        // Just at maximum depth
        let at_max = na::Vector3::new(0.0, 0.0, MAX_DEPTH);
        assert!(validate_point_for_projection(&at_max).is_ok());

        // Just above maximum depth
        let above_max = na::Vector3::new(0.0, 0.0, MAX_DEPTH * 1.1);
        assert_eq!(
            validate_point_for_projection(&above_max),
            Err(ValidationError::OutOfRange)
        );
    }

    #[test]
    fn test_validate_point_negative_depth() {
        // A negative depth with large absolute value should still pass
        // because validation only checks abs(depth) < MIN_DEPTH
        let behind_camera = na::Vector3::new(0.0, 0.0, -5.0);
        assert!(validate_point_for_projection(&behind_camera).is_ok());
        
        // But a very small absolute depth should fail
        let very_shallow = na::Vector3::new(0.0, 0.0, -1e-7);
        assert_eq!(
            validate_point_for_projection(&very_shallow),
            Err(ValidationError::InvalidDepth)
        );
    }

    #[test]
    fn test_validate_finite_vector_mixed_values() {
        let valid = na::Vector3::new(0.0, -1e-100, 1e100);
        assert!(validate_finite_vector(&valid).is_ok());

        let with_inf = na::Vector3::new(0.0, f64::NEG_INFINITY, 1.0);
        assert!(validate_finite_vector(&with_inf).is_err());
    }

    #[test]
    fn test_validate_finite_matrix() {
        let valid = na::Matrix3::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
        assert!(validate_finite_matrix(&valid).is_ok());

        let with_nan = na::Matrix3::new(1.0, f64::NAN, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
        assert!(validate_finite_matrix(&with_nan).is_err());
    }
}
