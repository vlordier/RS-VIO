//! Type Portability and Numeric Safety Macros
//!
//! This module provides comprehensive macros for:
//! - F32/F64 abstraction and runtime selection
//! - Type-safe numeric conversions
//! - Dimensioned types (Pixels, Meters, Radians)
//! - SIMD-friendly vector operations
//! - Camera calibration helpers
//! - Floating-point comparisons with epsilon tolerance
//! - Batch numeric operations

/// Define a numeric type that can switch between f32 and f64 at runtime
/// Helps maintain single codebase for different precision requirements
#[macro_export]
macro_rules! numeric_type {
    ($float_type:ty, $dims:expr) => {
        type Scalar = $float_type;
        const FLOAT_PRECISION_DIMS: usize = $dims;
    };
}

/// Create a numeric literal with consistent type across f32/f64
#[macro_export]
macro_rules! numeric_literal {
    ($value:expr, f32) => {
        $value as f32
    };
    ($value:expr, f64) => {
        $value as f64
    };
    ($value:expr) => {
        $value
    };
}

/// Safe numeric conversion with overflow/underflow checking
#[macro_export]
macro_rules! safe_convert {
    // f64 to f32 with underflow protection
    ($val:expr => f32) => {{
        let f64_val = $val as f64;
        if f64_val.is_nan() || f64_val.is_infinite() {
            log::warn!("Invalid float conversion: {:?}", f64_val);
            0.0_f32
        } else if f64_val > f32::MAX as f64 {
            log::warn!(
                "Overflow in f64->f32 conversion: {} > {}",
                f64_val,
                f32::MAX
            );
            f32::MAX
        } else if f64_val < f32::MIN as f64 {
            log::warn!(
                "Underflow in f64->f32 conversion: {} < {}",
                f64_val,
                f32::MIN
            );
            f32::MIN
        } else {
            f64_val as f32
        }
    }};
    // Integer to f32/f64
    ($val:expr => f32, checked) => {{
        TryFrom::<i64>::try_from($val as i64)
            .ok()
            .and_then(|i| Some(i as f32))
            .unwrap_or(0.0_f32)
    }};
    ($val:expr => f64, checked) => {{
        TryFrom::<i64>::try_from($val as i64)
            .ok()
            .and_then(|i| Some(i as f64))
            .unwrap_or(0.0_f64)
    }};
}

/// Create sized vectors with type safety
#[macro_export]
macro_rules! vec_sized {
    // Named vector for N elements of type T
    ($name:ident : Vec<$t:ty> = [$($elem:expr),+ $(,)?]) => {
        let $name: Vec<$t> = vec![$($elem),+];
    };
    // Create vector of N zeros
    (zeros: $n:expr => $t:ty) => {{
        vec![<$t>::default(); $n]
    }};
    // Create vector of N ones (numeric types only)
    (ones: $n:expr => f32) => {{
        vec![1.0_f32; $n]
    }};
    (ones: $n:expr => f64) => {{
        vec![1.0_f64; $n]
    }};
}

/// Define a dimensioned type for type-safe dimensional values
/// Prevents mixing pixels with meters, or radians with degrees
#[macro_export]
macro_rules! dimensioned {
    // Define a new dimensional type
    ($vis:vis struct $name:ident($inner:ty);) => {
        #[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
        $vis struct $name(pub $inner);

        impl $name {
            pub const fn new(value: $inner) -> Self {
                Self(value)
            }

            pub const fn get(&self) -> $inner {
                self.0
            }

            pub fn into_inner(self) -> $inner {
                self.0
            }
        }

        impl From<$inner> for $name {
            fn from(value: $inner) -> Self {
                Self(value)
            }
        }

        impl std::fmt::Display for $name {
            fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
                write!(f, "{}", self.0)
            }
        }
    };
}

/// Create SIMD-friendly fixed-size aligned arrays
#[macro_export]
macro_rules! simd_vec {
    // Create aligned array: simd_vec!(f32; 16) -> [f32; 16] with alignment
    ($t:ty; $n:expr) => {{
        #[repr(align(32))]
        struct Aligned<T>([T; $n]);
        Aligned::<$t>([<$t>::default(); $n]).0
    }};
}

/// Epsilon-based floating-point equality check
#[macro_export]
macro_rules! eps_eq {
    // Compare with default epsilon
    ($a:expr, $b:expr) => {{
        let a = $a as f64;
        let b = $b as f64;
        (a - b).abs() < 1e-10
    }};
    // Compare with custom epsilon
    ($a:expr, $b:expr, $eps:expr) => {{
        let a = $a as f64;
        let b = $b as f64;
        let eps = $eps as f64;
        (a - b).abs() < eps
    }};
}

/// Batch numeric operations with single macro
#[macro_export]
macro_rules! batch_op {
    // Element-wise operation on vectors
    ($op:tt, $a:expr, $b:expr) => {{
        $a.iter()
            .zip($b.iter())
            .map(|(x, y)| x $op y)
            .collect::<Vec<_>>()
    }};
    // Map operation across vector
    (map $op:expr, $vec:expr) => {{
        $vec.iter().map($op).collect::<Vec<_>>()
    }};
}

/// Define camera focal length with dimension safety
#[macro_export]
macro_rules! focal_length {
    ($fx:expr, $fy:expr) => {
        (
            $crate::dimensioned!(
                struct FocalLengthX(f64);
            ),
            $crate::dimensioned!(
                struct FocalLengthY(f64);
            ),
        )
    };
    ($f:expr) => {{
        let f = $f as f64;
        (f, f)
    }};
}

/// Define camera principal point (optical center)
#[macro_export]
macro_rules! principal_point {
    ($cx:expr, $cy:expr) => {{
        (
            $crate::dimensioned!(
                struct PrincipalPointX(f64);
            ),
            $crate::dimensioned!(
                struct PrincipalPointY(f64);
            ),
        )
    }};
}

/// Define distortion coefficients for lens distortion
#[macro_export]
macro_rules! distortion_coeffs {
    // Brown-Conrady model: k1, k2, p1, p2, k3
    (brown_conrady: $k1:expr, $k2:expr, $p1:expr, $p2:expr, $k3:expr) => {
        [$k1 as f64, $k2 as f64, $p1 as f64, $p2 as f64, $k3 as f64]
    };
    // Fisheye model: k1, k2, k3, k4
    (fisheye: $k1:expr, $k2:expr, $k3:expr, $k4:expr) => {
        [$k1 as f64, $k2 as f64, $k3 as f64, $k4 as f64]
    };
}

/// Convert degrees to radians
#[macro_export]
macro_rules! deg_to_rad {
    ($deg:expr) => {{
        ($deg as f64) * std::f64::consts::PI / 180.0
    }};
}

/// Convert radians to degrees
#[macro_export]
macro_rules! rad_to_deg {
    ($rad:expr) => {{
        ($rad as f64) * 180.0 / std::f64::consts::PI
    }};
}

/// Matrix-style operation macro for 2D arrays
#[macro_export]
macro_rules! matrix_op {
    // Transpose 2D array
    (transpose, $matrix:expr) => {{
        let rows = $matrix.len();
        if rows == 0 {
            vec![]
        } else {
            let cols = $matrix[0].len();
            (0..cols)
                .map(|c| (0..rows).map(|r| $matrix[r][c]).collect::<Vec<_>>())
                .collect::<Vec<_>>()
        }
    }};
}

/// Define a numeric bounds constraint
#[macro_export]
macro_rules! numeric_bounds {
    ($val:expr, $min:expr, $max:expr) => {{
        if $val < $min {
            log::warn!("Value {} below minimum {}", $val, $min);
            $min
        } else if $val > $max {
            log::warn!("Value {} above maximum {}", $val, $max);
            $max
        } else {
            $val
        }
    }};
}

/// Macro for common derive combinations
///
/// # Example
/// ```ignore
/// common_derives!(MyStruct); // Expands to #[derive(Debug, Clone)]
/// common_derives!(MyEnum, with_copy); // Expands to #[derive(Debug, Clone, Copy)]
/// ```
#[macro_export]
macro_rules! common_derives {
    // Base: Debug + Clone
    () => {
        #[derive(Debug, Clone)]
    };
    (with_copy) => {
        #[derive(Debug, Clone, Copy)]
    };
    (with_eq) => {
        #[derive(Debug, Clone, PartialEq, Eq)]
    };
    (with_ord) => {
        #[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
    };
}

/// Macro for collecting iterator results with type inference
///
/// # Example
/// ```ignore
/// let values: Vec<_> = collect_vec!(iter.map(|x| x * 2));
/// let points = collect_vec!([f32; 3], positions.iter().map(to_3d));
/// ```
#[macro_export]
macro_rules! collect_vec {
    ($iter:expr) => {
        $iter.collect::<Vec<_>>()
    };
    ([$ty:ty; $n:expr], $iter:expr) => {
        $iter.collect::<Vec<[$ty; $n]>>()
    };
    ($ty:ty, $iter:expr) => {
        $iter.collect::<Vec<$ty>>()
    };
}

/// Macro for initializing collections with capacity
///
/// # Example
/// ```ignore
/// let vec = init_vec!(100);  // Vec::with_capacity(100)
/// let map = init_map!();     // HashMap::new()
/// let map = init_map!(String, i32, 50);  // HashMap::with_capacity(50)
/// ```
#[macro_export]
macro_rules! init_vec {
    ($capacity:expr) => {
        Vec::with_capacity($capacity)
    };
    ($ty:ty, $capacity:expr) => {
        Vec::<$ty>::with_capacity($capacity)
    };
}

/// Macro for HashMap/BTreeMap initialization
#[macro_export]
macro_rules! init_map {
    () => {
        std::collections::HashMap::new()
    };
    (btree) => {
        std::collections::BTreeMap::new()
    };
    ($key:ty, $val:ty) => {
        std::collections::HashMap::<$key, $val>::new()
    };
    ($key:ty, $val:ty, $capacity:expr) => {
        std::collections::HashMap::<$key, $val>::with_capacity($capacity)
    };
}

#[cfg(test)]
mod tests {
    #[test]
    fn test_safe_convert_f64_to_f32() {
        let val = 1.5_f64;
        let result = safe_convert!(val => f32);
        assert!((result - 1.5_f32).abs() < 1e-5);
    }

    #[test]
    fn test_safe_convert_overflow() {
        let val = (f32::MAX as f64) + 1e30;
        let result = safe_convert!(val => f32);
        assert_eq!(result, f32::MAX);
    }

    #[test]
    fn test_safe_convert_underflow() {
        let val = (f32::MIN as f64) - 1e30;
        let result = safe_convert!(val => f32);
        assert_eq!(result, f32::MIN);
    }

    #[test]
    fn test_safe_convert_nan() {
        let val = f64::NAN;
        let result = safe_convert!(val => f32);
        assert_eq!(result, 0.0_f32);
    }

    #[test]
    fn test_eps_eq_default() {
        assert!(eps_eq!(1.0, 1.0));
        assert!(eps_eq!(1.0, 1.0 + 1e-11));
        assert!(!eps_eq!(1.0, 1.1));
    }

    #[test]
    fn test_eps_eq_custom() {
        assert!(eps_eq!(1.0, 1.05, 0.1));
        assert!(!eps_eq!(1.0, 1.2, 0.1));
    }

    #[test]
    fn test_deg_to_rad() {
        let rad = deg_to_rad!(180.0);
        assert!(eps_eq!(rad, std::f64::consts::PI));
    }

    #[test]
    fn test_rad_to_deg() {
        let deg = rad_to_deg!(std::f64::consts::PI);
        assert!(eps_eq!(deg, 180.0));
    }

    #[test]
    fn test_vec_sized_zeros() {
        let v: Vec<f32> = vec_sized!(zeros: 5 => f32);
        assert_eq!(v.len(), 5);
        assert!(v.iter().all(|x| *x == 0.0));
    }

    #[test]
    fn test_vec_sized_ones() {
        let v: Vec<f32> = vec_sized!(ones: 3 => f32);
        assert_eq!(v.len(), 3);
        assert!(v.iter().all(|x| *x == 1.0));
    }

    #[test]
    fn test_batch_op_add() {
        let a = vec![1.0, 2.0, 3.0];
        let b = vec![1.0, 1.0, 1.0];
        let result = batch_op!(+, a, b);
        assert_eq!(result, vec![2.0, 3.0, 4.0]);
    }

    #[test]
    fn test_batch_op_map() {
        let v = vec![1.0, 2.0, 3.0];
        let result = batch_op!(map |x: &f64| x * 2.0, v);
        assert_eq!(result, vec![2.0, 4.0, 6.0]);
    }

    #[test]
    fn test_numeric_bounds_clamp() {
        assert_eq!(numeric_bounds!(5, 0, 10), 5);
        assert_eq!(numeric_bounds!(-5, 0, 10), 0);
        assert_eq!(numeric_bounds!(15, 0, 10), 10);
    }

    #[test]
    fn test_distortion_coeffs_brown_conrady() {
        let coeffs = distortion_coeffs!(brown_conrady: 0.1, 0.2, 0.01, 0.02, 0.03);
        assert_eq!(coeffs.len(), 5);
        assert!(eps_eq!(coeffs[0], 0.1));
        assert!(eps_eq!(coeffs[4], 0.03));
    }

    #[test]
    fn test_distortion_coeffs_fisheye() {
        let coeffs = distortion_coeffs!(fisheye: 0.1, 0.2, 0.3, 0.4);
        assert_eq!(coeffs.len(), 4);
        assert!(eps_eq!(coeffs[0], 0.1));
        assert!(eps_eq!(coeffs[3], 0.4));
    }

    #[test]
    fn test_collect_vec() {
        let values = collect_vec!((0..5).map(|x| x * 2));
        assert_eq!(values, vec![0, 2, 4, 6, 8]);

        let points = collect_vec!([f32; 2], vec![[1.0f32, 2.0], [3.0, 4.0]].into_iter());
        assert_eq!(points.len(), 2);
    }

    #[test]
    fn test_init_vec() {
        let vec: Vec<i32> = init_vec!(100);
        assert_eq!(vec.capacity(), 100);
        assert_eq!(vec.len(), 0);

        let vec = init_vec!(f64, 50);
        assert_eq!(vec.capacity(), 50);
    }

    #[test]
    fn test_init_map() {
        let map: std::collections::HashMap<String, i32> = init_map!();
        assert!(map.is_empty());

        let map = init_map!(String, i32, 100);
        assert!(map.capacity() >= 100);

        let btree: std::collections::BTreeMap<i32, String> = init_map!(btree);
        assert!(btree.is_empty());
    }
}
