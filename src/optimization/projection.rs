//! Common projection and Jacobian utilities for optimization factors
//!
//! This module provides shared projection functions used by multiple factors
//! to eliminate code duplication and ensure consistency across the codebase.

use na::{Matrix2x3, Vector2, Vector3};
use nalgebra as na;

/// Project a 3D point in camera frame to normalized coordinates
///
/// Performs simple pinhole projection: (x/z, y/z)
///
/// # Arguments
/// * `point_3d_cam` - 3D point in camera coordinates
///
/// # Returns
/// 2D normalized coordinates
#[inline]
pub fn project_normalized(point_3d_cam: Vector3<f64>) -> Vector2<f64> {
    let x = point_3d_cam[0] / point_3d_cam[2];
    let y = point_3d_cam[1] / point_3d_cam[2];
    Vector2::new(x, y)
}

/// Compute Jacobian of normalized projection w.r.t. 3D point in camera frame
///
/// For pinhole projection [x/z, y/z], this computes the partial derivatives:
///
/// ```text
/// ∂[x/z, y/z]/∂[x, y, z]
/// ```
///
/// # Mathematical Details
///
/// The Jacobian is a 2x3 matrix where:
///
/// ```text
/// [ ∂(x/z)/∂x   ∂(x/z)/∂y   ∂(x/z)/∂z ]   [ 1/z    0      -x/z² ]
/// [ ∂(y/z)/∂x   ∂(y/z)/∂y   ∂(y/z)/∂z ] = [  0    1/z    -y/z² ]
/// ```
///
/// # Arguments
/// * `point_3d_cam` - 3D point in camera coordinates
///
/// # Returns
/// 2x3 Jacobian matrix
#[inline]
pub fn jacobian_proj_wrt_point(point_3d_cam: Vector3<f64>) -> Matrix2x3<f64> {
    let x = point_3d_cam[0];
    let y = point_3d_cam[1];
    let z = point_3d_cam[2];

    let inv_z = 1.0 / z;
    let inv_z_sq = inv_z * inv_z;

    let mut jac = Matrix2x3::zeros();
    jac[(0, 0)] = inv_z; // ∂(x/z)/∂x
    jac[(0, 1)] = 0.0; // ∂(x/z)/∂y
    jac[(0, 2)] = -x * inv_z_sq; // ∂(x/z)/∂z
    jac[(1, 0)] = 0.0; // ∂(y/z)/∂x
    jac[(1, 1)] = inv_z; // ∂(y/z)/∂y
    jac[(1, 2)] = -y * inv_z_sq; // ∂(y/z)/∂z

    jac
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_project_normalized() {
        let point = Vector3::new(2.0, 4.0, 2.0);
        let proj = project_normalized(point);
        assert_eq!(proj[0], 1.0); // 2.0 / 2.0
        assert_eq!(proj[1], 2.0); // 4.0 / 2.0
    }

    #[test]
    fn test_jacobian_proj_wrt_point() {
        let point = Vector3::new(2.0, 4.0, 2.0);
        let jac = jacobian_proj_wrt_point(point);

        // For z=2: inv_z = 0.5, inv_z_sq = 0.25
        // ∂(x/z)/∂x = 0.5, ∂(x/z)/∂z = -2.0 * 0.25 = -0.5
        // ∂(y/z)/∂y = 0.5, ∂(y/z)/∂z = -4.0 * 0.25 = -1.0

        assert_eq!(jac[(0, 0)], 0.5);
        assert_eq!(jac[(0, 1)], 0.0);
        assert_eq!(jac[(0, 2)], -0.5);
        assert_eq!(jac[(1, 0)], 0.0);
        assert_eq!(jac[(1, 1)], 0.5);
        assert_eq!(jac[(1, 2)], -1.0);
    }
}
