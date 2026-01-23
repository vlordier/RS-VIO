//! Common mathematical utilities and helpers.
//!
//! Provides reusable mathematical operations that are used throughout
//! the codebase, reducing duplication and improving consistency.

use crate::types::Float;

/// Clamps a value between min and max.
///
/// This is a generic implementation that works with any comparable type.
///
/// # Example
///
/// ```
/// use rs_vio::common::math::clamp;
///
/// assert_eq!(clamp(5.0, 0.0, 10.0), 5.0);
/// assert_eq!(clamp(-1.0, 0.0, 10.0), 0.0);
/// assert_eq!(clamp(15.0, 0.0, 10.0), 10.0);
/// ```
#[inline]
pub fn clamp<T: PartialOrd>(value: T, min: T, max: T) -> T {
    if value < min {
        min
    } else if value > max {
        max
    } else {
        value
    }
}

/// Normalizes an angle to the range [-π, π].
///
/// This is commonly used in rotation and orientation calculations.
///
/// # Example
///
/// ```
/// use rs_vio::common::math::normalize_angle;
/// use std::f64::consts::PI;
///
/// assert!((normalize_angle(0.0) - 0.0).abs() < 1e-9);
/// assert!((normalize_angle(3.0 * PI) - PI).abs() < 1e-9);
/// ```
#[inline]
pub fn normalize_angle(angle: Float) -> Float {
    let two_pi = 2.0 * std::f64::consts::PI;
    let mut normalized = angle % two_pi;

    if normalized > std::f64::consts::PI {
        normalized -= two_pi;
    } else if normalized < -std::f64::consts::PI {
        normalized += two_pi;
    }

    normalized
}

/// Computes square root with safety check for negative values.
///
/// Returns 0.0 for negative inputs instead of NaN, which is often
/// more useful in numerical optimization contexts.
///
/// # Example
///
/// ```
/// use rs_vio::common::math::safe_sqrt;
///
/// assert_eq!(safe_sqrt(4.0), 2.0);
/// assert_eq!(safe_sqrt(-1.0), 0.0);
/// ```
#[inline]
pub fn safe_sqrt(value: Float) -> Float {
    if value <= 0.0 {
        0.0
    } else {
        value.sqrt()
    }
}

/// Linear interpolation between two values.
///
/// # Arguments
///
/// * `a` - Start value
/// * `b` - End value
/// * `t` - Interpolation factor in [0, 1]
///
/// # Example
///
/// ```
/// use rs_vio::common::math::lerp;
///
/// assert_eq!(lerp(0.0, 10.0, 0.5), 5.0);
/// assert_eq!(lerp(0.0, 10.0, 0.0), 0.0);
/// assert_eq!(lerp(0.0, 10.0, 1.0), 10.0);
/// ```
#[inline]
pub fn lerp(a: Float, b: Float, t: Float) -> Float {
    a + (b - a) * t
}

/// Computes the median of a slice.
///
/// Returns None if the slice is empty.
/// Note: This function clones and sorts the input.
///
/// # Example
///
/// ```
/// use rs_vio::common::math::median;
///
/// assert_eq!(median(&[1.0, 2.0, 3.0, 4.0, 5.0]), Some(3.0));
/// assert_eq!(median(&[1.0, 2.0, 3.0, 4.0]), Some(2.5));
/// assert_eq!(median(&[]), None);
/// ```
pub fn median(values: &[Float]) -> Option<Float> {
    if values.is_empty() {
        return None;
    }

    let mut sorted = values.to_vec();
    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));

    let mid = sorted.len() / 2;
    if sorted.len() % 2 == 0 {
        Some((sorted[mid - 1] + sorted[mid]) / 2.0)
    } else {
        Some(sorted[mid])
    }
}

/// Computes the Median Absolute Deviation (MAD), a robust measure of variability.
///
/// # Arguments
///
/// * `values` - Input values
/// * `median_val` - Optional pre-computed median (will be computed if None)
///
/// # Returns
///
/// The MAD value, or None if the input is empty.
pub fn mad(values: &[Float], median_val: Option<Float>) -> Option<Float> {
    if values.is_empty() {
        return None;
    }

    let med = median_val.or_else(|| median(values))?;
    let deviations: Vec<Float> = values.iter().map(|&v| (v - med).abs()).collect();
    median(&deviations)
}

/// Robust outlier detection using MAD-based Z-score.
///
/// Returns true if the value is an outlier (Z-score > threshold).
///
/// # Arguments
///
/// * `value` - Value to test
/// * `values` - All values in the distribution
/// * `threshold` - Z-score threshold (typical: 2.5 to 3.5)
pub fn is_outlier_mad(value: Float, values: &[Float], threshold: Float) -> bool {
    if let (Some(med), Some(mad_val)) = (median(values), mad(values, None)) {
        if mad_val > 0.0 {
            let z_score = ((value - med).abs()) / (1.4826 * mad_val); // 1.4826 normalizes to std dev
            return z_score > threshold;
        }
    }
    false
}

/// Safe division with epsilon protection
///
/// Returns `numerator / max(denominator, epsilon)` to avoid division by zero.
/// If denominator is very small, returns a default value instead.
///
/// # Arguments
///
/// * `numerator` - The numerator
/// * `denominator` - The denominator
/// * `epsilon` - Minimum value for denominator (default: 1e-10)
/// * `default` - Value to return if denominator is too small
///
/// # Example
///
/// ```
/// use rs_vio::common::math::safe_div;
///
/// assert_eq!(safe_div(10.0, 2.0, 1e-10, 0.0), 5.0);
/// assert_eq!(safe_div(10.0, 0.0, 1e-10, 0.0), 0.0);
/// ```
#[inline]
pub fn safe_div(numerator: Float, denominator: Float, epsilon: Float, default: Float) -> Float {
    if denominator.abs() < epsilon {
        default
    } else {
        numerator / denominator
    }
}

/// Normalized vector with safety check for zero-length vectors
///
/// Returns the normalized vector, or a default vector if the input is too small.
///
/// # Arguments
///
/// * `v` - Input vector as [x, y, z]
/// * `epsilon` - Minimum magnitude threshold
/// * `default` - Default vector to return if magnitude < epsilon
pub fn safe_normalize(v: [Float; 3], epsilon: Float, default: [Float; 3]) -> [Float; 3] {
    let mag_sq = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];

    if mag_sq < epsilon * epsilon {
        default
    } else {
        let mag = mag_sq.sqrt();
        [v[0] / mag, v[1] / mag, v[2] / mag]
    }
}

/// Safe inverse with stability check
///
/// Returns `1.0 / value` if value is above epsilon, otherwise returns default.
///
/// # Example
///
/// ```
/// use rs_vio::common::math::safe_inverse;
///
/// assert_eq!(safe_inverse(2.0, 1e-10, 0.0), 0.5);
/// assert_eq!(safe_inverse(0.0, 1e-10, 0.0), 0.0);
/// ```
#[inline]
pub fn safe_inverse(value: Float, epsilon: Float, default: Float) -> Float {
    if value.abs() < epsilon {
        default
    } else {
        1.0 / value
    }
}

/// Clamp a value to ensure it stays in a numerically stable range
///
/// Useful for preventing floating point operations from producing extreme values.
#[inline]
pub fn clamp_stable(value: Float, min: Float, max: Float) -> Float {
    if value.is_nan() {
        (min + max) / 2.0
    } else if value.is_infinite() {
        if value > 0.0 {
            max
        } else {
            min
        }
    } else {
        clamp(value, min, max)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_clamp() {
        assert_eq!(clamp(5.0, 0.0, 10.0), 5.0);
        assert_eq!(clamp(-1.0, 0.0, 10.0), 0.0);
        assert_eq!(clamp(15.0, 0.0, 10.0), 10.0);
    }

    #[test]
    fn test_normalize_angle() {
        use std::f64::consts::PI;

        assert!((normalize_angle(0.0) - 0.0).abs() < 1e-6);
        assert!((normalize_angle(PI as Float) - PI as Float).abs() < 1e-6);
        assert!((normalize_angle(-PI as Float) - (-PI as Float)).abs() < 1e-6);
    }

    #[test]
    fn test_safe_sqrt() {
        assert_eq!(safe_sqrt(4.0), 2.0);
        assert_eq!(safe_sqrt(0.0), 0.0);
        assert_eq!(safe_sqrt(-1.0), 0.0);
    }

    #[test]
    fn test_lerp() {
        assert_eq!(lerp(0.0, 10.0, 0.0), 0.0);
        assert_eq!(lerp(0.0, 10.0, 0.5), 5.0);
        assert_eq!(lerp(0.0, 10.0, 1.0), 10.0);
    }

    #[test]
    fn test_median() {
        assert_eq!(median(&[1.0, 2.0, 3.0, 4.0, 5.0]), Some(3.0));
        assert_eq!(median(&[1.0, 2.0, 3.0, 4.0]), Some(2.5));
        assert_eq!(median(&[5.0]), Some(5.0));
        assert_eq!(median(&[]), None);
    }

    #[test]
    fn test_mad() {
        let values = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let mad_val = mad(&values, None);
        assert!(mad_val.is_some());
        assert!(mad_val.unwrap() > 0.0);
    }

    #[test]
    fn test_is_outlier_mad() {
        let values = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        assert!(!is_outlier_mad(3.0, &values, 3.0));
        assert!(is_outlier_mad(100.0, &values, 3.0));
    }

    #[test]
    fn test_safe_div() {
        assert_eq!(safe_div(10.0, 2.0, 1e-10, 0.0), 5.0);
        assert_eq!(safe_div(10.0, 0.0, 1e-10, 0.0), 0.0);
        assert_eq!(safe_div(10.0, 1e-11, 1e-10, 0.0), 0.0);
        assert_eq!(safe_div(10.0, 1e-11, 1e-10, 99.0), 99.0);
    }

    #[test]
    fn test_safe_normalize() {
        let v = [3.0, 4.0, 0.0];
        let norm = safe_normalize(v, 1e-10, [1.0, 0.0, 0.0]);
        assert!((norm[0] - 0.6).abs() < 1e-6);
        assert!((norm[1] - 0.8).abs() < 1e-6);

        let zero = [0.0, 0.0, 0.0];
        let default = [1.0, 0.0, 0.0];
        let result = safe_normalize(zero, 1e-10, default);
        assert_eq!(result, default);
    }

    #[test]
    fn test_safe_inverse() {
        assert_eq!(safe_inverse(2.0, 1e-10, 0.0), 0.5);
        assert_eq!(safe_inverse(0.0, 1e-10, 0.0), 0.0);
        assert_eq!(safe_inverse(1e-11, 1e-10, 99.0), 99.0);
    }

    #[test]
    fn test_clamp_stable() {
        assert_eq!(clamp_stable(5.0, 0.0, 10.0), 5.0);
        assert_eq!(clamp_stable(f64::NAN, 0.0, 10.0), 5.0);
        assert_eq!(clamp_stable(f64::INFINITY, 0.0, 10.0), 10.0);
        assert_eq!(clamp_stable(f64::NEG_INFINITY, 0.0, 10.0), 0.0);
    }
}
