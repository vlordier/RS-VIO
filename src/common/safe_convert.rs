//! Safe type conversions with proper error handling
//!
//! Provides checked conversions to avoid panics from lossy casts and overflows.

use std::fmt;

/// Error type for conversion failures
#[derive(Debug, Clone)]
pub enum ConversionError {
    Overflow { from: String, to: String },
    Underflow { from: String, to: String },
    Precision { from: String, to: String },
    OutOfRange { value: String, target_type: String },
}

impl fmt::Display for ConversionError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ConversionError::Overflow { from, to } => {
                write!(f, "Overflow converting {} to {}", from, to)
            },
            ConversionError::Underflow { from, to } => {
                write!(f, "Underflow converting {} to {}", from, to)
            },
            ConversionError::Precision { from, to } => {
                write!(f, "Precision loss converting {} to {}", from, to)
            },
            ConversionError::OutOfRange { value, target_type } => {
                write!(f, "Value {} out of range for {}", value, target_type)
            },
        }
    }
}

impl std::error::Error for ConversionError {}

/// Safe conversion from f64 to usize with bounds checking
pub fn f64_to_usize(value: f64) -> Result<usize, ConversionError> {
    if value < 0.0 {
        return Err(ConversionError::Underflow {
            from: format!("f64({})", value),
            to: "usize".to_string(),
        });
    }

    if value > usize::MAX as f64 {
        return Err(ConversionError::Overflow {
            from: format!("f64({})", value),
            to: "usize".to_string(),
        });
    }

    if value.is_nan() || value.is_infinite() {
        return Err(ConversionError::OutOfRange {
            value: format!("f64({})", value),
            target_type: "usize".to_string(),
        });
    }

    Ok(value as usize)
}

/// Safe conversion from f32 to usize with bounds checking
pub fn f32_to_usize(value: f32) -> Result<usize, ConversionError> {
    f64_to_usize(value as f64)
}

/// Safe conversion from i64 to usize with bounds checking
pub fn i64_to_usize(value: i64) -> Result<usize, ConversionError> {
    if value < 0 {
        return Err(ConversionError::Underflow {
            from: format!("i64({})", value),
            to: "usize".to_string(),
        });
    }

    usize::try_from(value).map_err(|_| ConversionError::Overflow {
        from: format!("i64({})", value),
        to: "usize".to_string(),
    })
}

/// Safe conversion from usize to i32 with bounds checking
pub fn usize_to_i32(value: usize) -> Result<i32, ConversionError> {
    i32::try_from(value).map_err(|_| ConversionError::Overflow {
        from: format!("usize({})", value),
        to: "i32".to_string(),
    })
}

/// Safe conversion from usize to u32 with bounds checking
pub fn usize_to_u32(value: usize) -> Result<u32, ConversionError> {
    u32::try_from(value).map_err(|_| ConversionError::Overflow {
        from: format!("usize({})", value),
        to: "u32".to_string(),
    })
}

/// Safe percentile index calculation
///
/// Calculates the index for a given percentile in a sorted array.
/// Uses linear interpolation between closest ranks.
pub fn percentile_index(count: usize, percentile: f64) -> Result<usize, ConversionError> {
    if count == 0 {
        return Err(ConversionError::OutOfRange {
            value: "0".to_string(),
            target_type: "non-zero count".to_string(),
        });
    }

    if !(0.0..=1.0).contains(&percentile) {
        return Err(ConversionError::OutOfRange {
            value: format!("{}", percentile),
            target_type: "percentile [0.0, 1.0]".to_string(),
        });
    }

    // Linear interpolation: index = percentile * (count - 1)
    // For count=100: p95 = 0.95 * 99 = 94.05 → floor to 94
    // For count=10: p50 = 0.5 * 9 = 4.5 → round to 5
    let index = (percentile * (count - 1) as f64).round() as usize;
    Ok(index.min(count - 1))
}

/// Safe array index with bounds checking
pub fn checked_index<T>(slice: &[T], index: usize) -> Option<&T> {
    slice.get(index)
}

/// Safe mutable array index with bounds checking
pub fn checked_index_mut<T>(slice: &mut [T], index: usize) -> Option<&mut T> {
    slice.get_mut(index)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_f64_to_usize_valid() {
        assert_eq!(f64_to_usize(42.7).unwrap(), 42);
        assert_eq!(f64_to_usize(0.0).unwrap(), 0);
    }

    #[test]
    fn test_f64_to_usize_negative() {
        assert!(f64_to_usize(-1.0).is_err());
    }

    #[test]
    fn test_f64_to_usize_nan() {
        assert!(f64_to_usize(f64::NAN).is_err());
    }

    #[test]
    fn test_f64_to_usize_infinity() {
        assert!(f64_to_usize(f64::INFINITY).is_err());
    }

    #[test]
    fn test_i64_to_usize_valid() {
        assert_eq!(i64_to_usize(42).unwrap(), 42);
        assert_eq!(i64_to_usize(0).unwrap(), 0);
    }

    #[test]
    fn test_i64_to_usize_negative() {
        assert!(i64_to_usize(-1).is_err());
    }

    #[test]
    fn test_percentile_index_valid() {
        assert_eq!(percentile_index(100, 0.95).unwrap(), 94);
        assert_eq!(percentile_index(100, 0.99).unwrap(), 98);
        assert_eq!(percentile_index(10, 0.5).unwrap(), 5);
    }

    #[test]
    fn test_percentile_index_empty() {
        assert!(percentile_index(0, 0.5).is_err());
    }

    #[test]
    fn test_percentile_index_out_of_range() {
        assert!(percentile_index(100, 1.5).is_err());
        assert!(percentile_index(100, -0.1).is_err());
    }

    #[test]
    fn test_checked_index() {
        let arr = vec![1, 2, 3, 4, 5];
        assert_eq!(checked_index(&arr, 2), Some(&3));
        assert_eq!(checked_index(&arr, 10), None);
    }
}
