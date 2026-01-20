//! Common test utilities and helpers.
//!
//! This module provides reusable test fixtures, assertions, and helpers
//! to reduce duplication across test modules and improve test maintainability.

#![cfg(test)]

use crate::types::{Matrix4x4, Float};
use nalgebra as na;

/// Creates a test pose matrix with a simple rotation and translation.
///
/// Useful for generating test data without needing to construct
/// complex transformations manually.
///
/// # Arguments
///
/// * `tx`, `ty`, `tz` - Translation components
/// * `roll`, `pitch`, `yaw` - Rotation angles in radians
///
/// # Returns
///
/// A 4x4 pose matrix
pub fn create_test_pose(
    tx: Float,
    ty: Float,
    tz: Float,
    roll: Float,
    pitch: Float,
    yaw: Float,
) -> Matrix4x4 {
    let mut T = Matrix4x4::identity();
    
    let q = na::UnitQuaternion::from_euler_angles(roll, pitch, yaw);
    let R = q.to_rotation_matrix().into_inner();
    
    for i in 0..3 {
        for j in 0..3 {
            T[(i, j)] = R[(i, j)];
        }
    }
    
    T[(0, 3)] = tx;
    T[(1, 3)] = ty;
    T[(2, 3)] = tz;
    
    T
}

/// Creates an identity pose matrix.
pub fn identity_pose() -> Matrix4x4 {
    Matrix4x4::identity()
}

/// Creates a random pose matrix for testing.
///
/// Uses a fixed seed for reproducibility.
pub fn random_pose(seed: u64) -> Matrix4x4 {
    use rand::{rngs::StdRng, Rng, SeedableRng};
    
    let mut rng = StdRng::seed_from_u64(seed);
    
    create_test_pose(
        rng.gen_range(-10.0..10.0),
        rng.gen_range(-10.0..10.0),
        rng.gen_range(-10.0..10.0),
        rng.gen_range(-3.14..3.14),
        rng.gen_range(-3.14..3.14),
        rng.gen_range(-3.14..3.14),
    )
}

/// Asserts that two floating point values are approximately equal.
///
/// # Arguments
///
/// * `a`, `b` - Values to compare
/// * `tolerance` - Maximum allowed absolute difference
/// * `message` - Optional message for assertion failure
#[macro_export]
macro_rules! assert_float_eq {
    ($a:expr, $b:expr, $tol:expr) => {
        assert!(
            ($a - $b).abs() < $tol,
            "assertion failed: {} ≈ {} (tolerance: {})\n  left: {}\n right: {}",
            stringify!($a),
            stringify!($b),
            $tol,
            $a,
            $b
        );
    };
    ($a:expr, $b:expr, $tol:expr, $msg:expr) => {
        assert!(
            ($a - $b).abs() < $tol,
            "{}\nassertion failed: {} ≈ {} (tolerance: {})\n  left: {}\n right: {}",
            $msg,
            stringify!($a),
            stringify!($b),
            $tol,
            $a,
            $b
        );
    };
}

/// Asserts that two matrices are approximately equal element-wise.
///
/// # Arguments
///
/// * `a`, `b` - Matrices to compare
/// * `tolerance` - Maximum allowed absolute difference per element
#[macro_export]
macro_rules! assert_matrix_eq {
    ($a:expr, $b:expr, $tol:expr) => {{
        let a_mat = &$a;
        let b_mat = &$b;
        assert_eq!(a_mat.nrows(), b_mat.nrows(), "Matrix row count mismatch");
        assert_eq!(a_mat.ncols(), b_mat.ncols(), "Matrix column count mismatch");
        
        for i in 0..a_mat.nrows() {
            for j in 0..a_mat.ncols() {
                assert!(
                    (a_mat[(i, j)] - b_mat[(i, j)]).abs() < $tol,
                    "Matrix element mismatch at ({}, {}): {} vs {} (tolerance: {})",
                    i,
                    j,
                    a_mat[(i, j)],
                    b_mat[(i, j)],
                    $tol
                );
            }
        }
    }};
}

/// Creates a test camera intrinsics matrix.
///
/// # Arguments
///
/// * `fx`, `fy` - Focal lengths
/// * `cx`, `cy` - Principal point
///
/// # Returns
///
/// A 3x3 camera matrix
pub fn create_test_camera_matrix(
    fx: Float,
    fy: Float,
    cx: Float,
    cy: Float,
) -> na::Matrix3<Float> {
    na::Matrix3::new(
        fx, 0.0, cx,
        0.0, fy, cy,
        0.0, 0.0, 1.0,
    )
}

/// Generates synthetic feature matches for testing.
///
/// Creates pairs of 2D points with known correspondence and optional noise.
///
/// # Arguments
///
/// * `count` - Number of correspondences to generate
/// * `noise_stddev` - Standard deviation of Gaussian noise to add
/// * `seed` - Random seed for reproducibility
///
/// # Returns
///
/// Two vectors of 2D points (left and right observations)
pub fn generate_test_correspondences(
    count: usize,
    noise_stddev: Float,
    seed: u64,
) -> (Vec<na::Vector2<Float>>, Vec<na::Vector2<Float>>) {
    use rand::{rngs::StdRng, Rng, SeedableRng};
    use rand_distr::{Distribution, Normal};
    
    let mut rng = StdRng::seed_from_u64(seed);
    let normal = Normal::new(0.0, noise_stddev as f64).unwrap();
    
    let mut left = Vec::with_capacity(count);
    let mut right = Vec::with_capacity(count);
    
    for _ in 0..count {
        let x = rng.gen_range(0.0..640.0);
        let y = rng.gen_range(0.0..480.0);
        
        let noise_x = normal.sample(&mut rng) as Float;
        let noise_y = normal.sample(&mut rng) as Float;
        
        left.push(na::Vector2::new(x, y));
        right.push(na::Vector2::new(x + noise_x, y + noise_y));
    }
    
    (left, right)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_create_test_pose() {
        let T = create_test_pose(1.0, 2.0, 3.0, 0.1, 0.2, 0.3);
        
        // Check translation
        assert_float_eq!(T[(0, 3)], 1.0, 1e-6);
        assert_float_eq!(T[(1, 3)], 2.0, 1e-6);
        assert_float_eq!(T[(2, 3)], 3.0, 1e-6);
        
        // Check bottom row
        assert_float_eq!(T[(3, 0)], 0.0, 1e-6);
        assert_float_eq!(T[(3, 1)], 0.0, 1e-6);
        assert_float_eq!(T[(3, 2)], 0.0, 1e-6);
        assert_float_eq!(T[(3, 3)], 1.0, 1e-6);
    }

    #[test]
    fn test_identity_pose() {
        let T = identity_pose();
        let I = Matrix4x4::identity();
        assert_matrix_eq!(T, I, 1e-10);
    }

    #[test]
    fn test_random_pose_deterministic() {
        let T1 = random_pose(42);
        let T2 = random_pose(42);
        assert_matrix_eq!(T1, T2, 1e-10);
    }

    #[test]
    fn test_create_camera_matrix() {
        let K = create_test_camera_matrix(500.0, 500.0, 320.0, 240.0);
        assert_float_eq!(K[(0, 0)], 500.0, 1e-6);
        assert_float_eq!(K[(1, 1)], 500.0, 1e-6);
        assert_float_eq!(K[(0, 2)], 320.0, 1e-6);
        assert_float_eq!(K[(1, 2)], 240.0, 1e-6);
    }

    #[test]
    fn test_generate_correspondences() {
        let (left, right) = generate_test_correspondences(10, 0.5, 42);
        assert_eq!(left.len(), 10);
        assert_eq!(right.len(), 10);
        
        // Deterministic with same seed
        let (left2, _right2) = generate_test_correspondences(10, 0.5, 42);
        for i in 0..10 {
            assert_float_eq!(left[i].x, left2[i].x, 1e-6);
            assert_float_eq!(left[i].y, left2[i].y, 1e-6);
        }
    }
}
