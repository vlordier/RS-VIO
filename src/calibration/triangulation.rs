//! Triangulation and geometry helpers for stereo calibration.
//!
//! Provides free functions for triangulating 3D points from stereo observations,
//! projecting points back into camera frames, and computing reprojection errors.

use crate::calibration::config::CalibrationConfig;
use crate::calibration::stereo_calibrator::StereoPair;
use nalgebra as na;

/// Helper struct for initial parameter estimation.
pub(crate) struct InitialParameters {
    pub left_intrinsics: Vec<f64>,
    pub right_intrinsics: Vec<f64>,
    pub extrinsics: Vec<f64>,
}

/// Initialize optimization parameters from a [`CalibrationConfig`].
pub(crate) fn initialize_parameters(config: &CalibrationConfig) -> InitialParameters {
    // Use config defaults or estimate from data
    let left_intrinsics = vec![
        config.initial_focal_length,      // fx
        config.initial_focal_length,      // fy
        config.initial_principal_point.0, // cx
        config.initial_principal_point.1, // cy
        0.0,
        0.0,
        0.0,
        0.0,
        0.0, // distortion (k1, k2, p1, p2, k3)
    ];

    let right_intrinsics = left_intrinsics.clone();

    // Initialize extrinsics with small baseline
    let extrinsics = vec![0.0, 0.0, 0.0, 0.1, 0.0, 0.0]; // rx, ry, rz, tx, ty, tz

    InitialParameters {
        left_intrinsics,
        right_intrinsics,
        extrinsics,
    }
}

/// Triangulate an initial 3D point for optimization (rough initialization).
pub(crate) fn triangulate_initial_point(
    left_point: &na::Vector2<f64>,
    right_point: &na::Vector2<f64>,
    params: &InitialParameters,
) -> Vec<f64> {
    // Simple triangulation assuming known intrinsics and small baseline
    // This is a rough initialization - optimization will refine it

    let fx = params.left_intrinsics[0];
    let fy = params.left_intrinsics[1];
    let cx = params.left_intrinsics[2];
    let cy = params.left_intrinsics[3];

    let baseline = params.extrinsics[3]; // tx

    // Convert to normalized coordinates
    let xl = (left_point.x - cx) / fx;
    let yl = (left_point.y - cy) / fy;
    let xr = (right_point.x - cx) / fx;
    let _yr = (right_point.y - cy) / fy;

    // Disparity
    let disparity = xl - xr;
    if disparity.abs() < 1e-6 {
        // Points too close, use default depth
        return vec![0.0, 0.0, 1.0];
    }

    // Triangulate
    let z = baseline / disparity;
    let x = xl * z;
    let y = yl * z;

    vec![x, y, z]
}

/// Convert a parameter vector `[rx, ry, rz, tx, ty, tz]` to an SE(3) isometry.
pub(crate) fn vector_to_isometry(params: &[f64]) -> na::Isometry3<f64> {
    let rx = params[0];
    let ry = params[1];
    let rz = params[2];
    let tx = params[3];
    let ty = params[4];
    let tz = params[5];

    let rotation = na::UnitQuaternion::from_euler_angles(rx, ry, rz);
    let translation = na::Vector3::new(tx, ty, tz);

    na::Isometry3::from_parts(translation.into(), rotation)
}

/// Triangulate a 3D point from stereo observations with known intrinsics.
pub(crate) fn triangulate_point(
    left_point: &na::Vector2<f64>,
    right_point: &na::Vector2<f64>,
    left_intrinsics: &[f64],
    extrinsics: &na::Isometry3<f64>,
) -> na::Vector3<f64> {
    // Simplified triangulation - in practice, you'd use proper stereo triangulation
    let fx = left_intrinsics[0];
    let fy = left_intrinsics[1];
    let cx = left_intrinsics[2];
    let cy = left_intrinsics[3];

    let baseline = extrinsics.translation.x; // Assume horizontal baseline

    let xl = (left_point.x - cx) / fx;
    let yl = (left_point.y - cy) / fy;
    let xr = (right_point.x - cx) / fx;
    let _yr = (right_point.y - cy) / fy;

    let disparity = xl - xr;
    if disparity.abs() < 1e-6 {
        return na::Vector3::new(0.0, 0.0, 1.0);
    }

    let z = baseline / disparity;
    let x = xl * z;
    let y = yl * z;

    na::Vector3::new(x, y, z)
}

/// Project a 3D point into a camera using the given intrinsics.
pub(crate) fn project_point(point: &na::Vector3<f64>, intrinsics: &[f64]) -> na::Vector2<f64> {
    let fx = intrinsics[0];
    let fy = intrinsics[1];
    let cx = intrinsics[2];
    let cy = intrinsics[3];

    let u = fx * point.x / point.z + cx;
    let v = fy * point.y / point.z + cy;

    na::Vector2::new(u, v)
}

/// Compute the mean reprojection error across all stereo pairs.
pub(crate) fn compute_reprojection_error(
    stereo_pairs: &[StereoPair],
    left_intrinsics: &[f64],
    right_intrinsics: &[f64],
    extrinsics: &na::Isometry3<f64>,
) -> f64 {
    let mut total_error = 0.0;
    let mut total_points = 0;

    for stereo_pair in stereo_pairs {
        for &(left_idx, right_idx) in &stereo_pair.correspondences {
            let left_obs = stereo_pair.left_features[left_idx];
            let right_obs = stereo_pair.right_features[right_idx];

            // Triangulate point
            let point_3d = triangulate_point(&left_obs, &right_obs, left_intrinsics, extrinsics);

            // Project back to cameras
            let left_proj = project_point(&point_3d, left_intrinsics);
            let right_proj = project_point(&(extrinsics.inverse() * point_3d), right_intrinsics);

            // Compute errors
            let left_error = (left_proj - left_obs).norm();
            let right_error = (right_proj - right_obs).norm();

            total_error += left_error + right_error;
            total_points += 2;
        }
    }

    if total_points > 0 {
        total_error / total_points as f64
    } else {
        0.0
    }
}

/// Compute per-point reprojection errors for quality assessment.
pub(crate) fn compute_per_point_errors(
    stereo_pairs: &[StereoPair],
    left_intrinsics: &[f64],
    right_intrinsics: &[f64],
    extrinsics: &na::Isometry3<f64>,
) -> Vec<f64> {
    let mut errors = Vec::new();

    for stereo_pair in stereo_pairs {
        for &(left_idx, right_idx) in &stereo_pair.correspondences {
            let observed_left = stereo_pair.left_features[left_idx];
            let observed_right = stereo_pair.right_features[right_idx];

            // Triangulate 3D point
            let point_3d = triangulate_point(
                &observed_left,
                &observed_right,
                left_intrinsics,
                extrinsics,
            );

            // Project back to cameras
            let projected_left = project_point(&point_3d, left_intrinsics);
            let projected_right =
                project_point(&(extrinsics.inverse() * point_3d), right_intrinsics);

            // Compute reprojection errors
            let left_error = (observed_left - projected_left).norm();
            let right_error = (observed_right - projected_right).norm();

            // Use maximum of left/right errors for this point
            errors.push(left_error.max(right_error));
        }
    }

    errors
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use super::*;

    fn default_params() -> InitialParameters {
        InitialParameters {
            left_intrinsics: vec![500.0, 500.0, 320.0, 240.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            right_intrinsics: vec![500.0, 500.0, 320.0, 240.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            extrinsics: vec![0.0, 0.0, 0.0, 0.1, 0.0, 0.0],
        }
    }

    #[test]
    fn test_triangulate_initial_point_known_depth() {
        let params = default_params();
        // Left point at principal point, right point shifted by disparity
        // disparity = baseline * fx / Z  =>  for Z=1.0, disparity = 0.1 * 500 / 1.0 = 50 pixels
        // In normalised coords: xl=0, xr = -baseline/Z = -0.1
        let left = na::Vector2::new(320.0, 240.0);
        let right = na::Vector2::new(320.0 - 50.0, 240.0); // 50 px disparity
        let pt = triangulate_initial_point(&left, &right, &params);
        // Expected: x=0, y=0, z = baseline / (xl - xr) in normalised coords
        // xl = 0.0, xr = (270-320)/500 = -0.1, disparity = 0.1, z = 0.1/0.1 = 1.0
        assert!((pt[2] - 1.0).abs() < 1e-8, "depth should be ~1.0, got {}", pt[2]);
        assert!(pt[0].abs() < 1e-8, "x should be ~0, got {}", pt[0]);
        assert!(pt[1].abs() < 1e-8, "y should be ~0, got {}", pt[1]);
    }

    #[test]
    fn test_triangulate_initial_point_zero_disparity() {
        let params = default_params();
        let left = na::Vector2::new(320.0, 240.0);
        let right = na::Vector2::new(320.0, 240.0); // zero disparity
        let pt = triangulate_initial_point(&left, &right, &params);
        // Should return default depth
        assert_eq!(pt, vec![0.0, 0.0, 1.0]);
    }

    #[test]
    fn test_vector_to_isometry_identity() {
        let params = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let iso = vector_to_isometry(&params);
        let identity = na::Isometry3::identity();
        assert!((iso.translation.vector - identity.translation.vector).norm() < 1e-12);
        assert!((iso.rotation.angle()) < 1e-12);
    }

    #[test]
    fn test_vector_to_isometry_translation() {
        let params = vec![0.0, 0.0, 0.0, 1.0, 2.0, 3.0];
        let iso = vector_to_isometry(&params);
        assert!((iso.translation.x - 1.0).abs() < 1e-12);
        assert!((iso.translation.y - 2.0).abs() < 1e-12);
        assert!((iso.translation.z - 3.0).abs() < 1e-12);
    }

    #[test]
    fn test_project_point_on_axis() {
        let intrinsics = [500.0, 500.0, 320.0, 240.0];
        let pt = na::Vector3::new(0.0, 0.0, 1.0);
        let px = project_point(&pt, &intrinsics);
        assert!((px.x - 320.0).abs() < 1e-10);
        assert!((px.y - 240.0).abs() < 1e-10);
    }

    #[test]
    fn test_project_point_off_axis() {
        let intrinsics = [500.0, 500.0, 320.0, 240.0];
        let pt = na::Vector3::new(0.1, -0.2, 1.0);
        let px = project_point(&pt, &intrinsics);
        assert!((px.x - 370.0).abs() < 1e-10);
        assert!((px.y - 140.0).abs() < 1e-10);
    }

    #[test]
    fn test_triangulate_and_project_roundtrip() {
        let intrinsics = vec![500.0, 500.0, 320.0, 240.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let extrinsics = vector_to_isometry(&[0.0, 0.0, 0.0, 0.1, 0.0, 0.0]);

        let left = na::Vector2::new(370.0, 190.0);
        // Compute expected right pixel for consistency
        let pt3d = triangulate_point(&left, &na::Vector2::new(345.0, 190.0), &intrinsics, &extrinsics);
        let reprojected = project_point(&pt3d, &intrinsics);
        assert!((reprojected - left).norm() < 1e-6, "left reprojection error too large");
    }
}
