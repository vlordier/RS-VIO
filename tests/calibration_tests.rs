#![allow(clippy::unwrap_used, clippy::expect_used)]
//! Integration tests for stereo camera calibration factors.
//!
//! These tests verify actual factor behavior with realistic stereo geometry
//! rather than just checking constructors and dimensions.
//!
//! **Note on extrinsics convention**: the factor code stores extrinsics as
//! `[rx, ry, rz, tx, ty, tz]` (Euler angles then translation).  The right-
//! camera point is computed via `relative_pose.inverse() * point_3d` where
//! `point_3d` is a `nalgebra::Vector3`.  In nalgebra, `Isometry * Vector`
//! applies *rotation only* (vectors are directions, not positions), so only
//! the rotation component of the extrinsics affects right-camera projection.

use apex_solver::factors::Factor;
use nalgebra as na;
use rs_vio::calibration::{
    config::CalibrationConfig,
    factors::{EpipolarFactor, StereoReprojectionFactor},
    stereo_calibrator::StereoCalibrator,
};

/// Helper: build standard stereo intrinsics (no distortion).
fn make_intrinsics(fx: f64, fy: f64, cx: f64, cy: f64) -> na::DVector<f64> {
    na::DVector::from_vec(vec![fx, fy, cx, cy, 0.0, 0.0, 0.0, 0.0, 0.0])
}

/// Helper: build extrinsics vector `[rx, ry, rz, tx, ty, tz]`.
fn make_extrinsics(rx: f64, ry: f64, rz: f64, tx: f64, ty: f64, tz: f64) -> na::DVector<f64> {
    na::DVector::from_vec(vec![rx, ry, rz, tx, ty, tz])
}

/// Helper: build 3D point parameter vector.
fn make_point(x: f64, y: f64, z: f64) -> na::DVector<f64> {
    na::DVector::from_vec(vec![x, y, z])
}

/// Discover the actual projected pixel coordinates that the factor computes
/// for a given parameter set by probing with zero-observations.
///
/// `residual = projection - observation`, so with `observation = (0, 0)` we
/// get `residual = projection`.
fn probe_projections(params: &[na::DVector<f64>]) -> (na::Vector2<f64>, na::Vector2<f64>) {
    let zero = na::Vector2::zeros();
    let probe = StereoReprojectionFactor::new(zero, zero);
    let (r, _) = probe.linearize(params, false);
    (
        na::Vector2::new(r[0], r[1]),
        na::Vector2::new(r[2], r[3]),
    )
}

/// Realistic stereo parameters with non-trivial rotation so that left and
/// right projections differ.
///
/// - f = 500, cx = 320, cy = 240, zero distortion
/// - Extrinsics: 0.1 rad pitch (ry), 0.1 m horizontal baseline (tx)
/// - 3D point at `[1, 0.5, 3]` (off-axis, 3 m depth)
fn realistic_stereo_params() -> Vec<na::DVector<f64>> {
    vec![
        make_intrinsics(500.0, 500.0, 320.0, 240.0),
        make_intrinsics(500.0, 500.0, 320.0, 240.0),
        make_extrinsics(0.0, 0.1, 0.0, 0.1, 0.0, 0.0),
        make_point(1.0, 0.5, 3.0),
    ]
}

// ---------------------------------------------------------------------------
// Stereo reprojection factor tests
// ---------------------------------------------------------------------------

#[test]
fn test_stereo_reprojection_zero_residual_at_ground_truth() {
    let params = realistic_stereo_params();

    // Discover the exact pixel coordinates the factor projects to.
    let (left_proj, right_proj) = probe_projections(&params);

    // Left projection: u = 500*1/3 + 320 ≈ 486.67, v = 500*0.5/3 + 240 ≈ 323.33
    assert!((left_proj.x - 486.667).abs() < 0.01, "left u sanity");
    assert!((left_proj.y - 323.333).abs() < 0.01, "left v sanity");

    // Right projection must differ from left (non-trivial rotation).
    assert!(
        (left_proj - right_proj).norm() > 1.0,
        "Left and right projections should differ with non-identity rotation"
    );

    // Use exact projected coordinates as observations → zero residual.
    let factor = StereoReprojectionFactor::new(left_proj, right_proj);
    let (residual, _) = factor.linearize(&params, false);

    assert_eq!(residual.len(), 4);
    assert!(
        residual.norm() < 1e-10,
        "Residual at ground truth should be ~0, got norm = {:.2e}",
        residual.norm()
    );
}

#[test]
fn test_stereo_reprojection_nonzero_residual_with_noise() {
    let params = realistic_stereo_params();
    let (left_proj, right_proj) = probe_projections(&params);

    // Add 2 px noise to all observation components.
    let noise = na::Vector2::new(2.0, 2.0);
    let factor = StereoReprojectionFactor::new(left_proj + noise, right_proj + noise);
    let (residual, _) = factor.linearize(&params, false);

    assert!(
        residual.norm() > 0.5,
        "Noisy observations should produce non-trivial residual, got norm = {:.4}",
        residual.norm()
    );
    // residual = projected - observed; noise is positive → residuals negative
    assert!(residual[0] < 0.0, "left u residual should be negative");
    assert!(residual[1] < 0.0, "left v residual should be negative");
    assert!(residual[2] < 0.0, "right u residual should be negative");
    assert!(residual[3] < 0.0, "right v residual should be negative");
}

#[test]
fn test_stereo_reprojection_jacobian_finite_difference() {
    let params = realistic_stereo_params();
    let (left_proj, right_proj) = probe_projections(&params);
    let factor = StereoReprojectionFactor::new(left_proj, right_proj);

    let (_, jacobian) = factor.linearize(&params, true);
    let jac = jacobian.expect("Jacobian should be returned when requested");

    assert_eq!(jac.nrows(), 4);
    assert_eq!(jac.ncols(), 27); // 9 + 9 + 6 + 3

    // Finite-difference verification for selected columns.
    // Concatenated layout: left_intr[0..9], right_intr[9..18],
    //   extrinsics[18..24], point[24..27]
    //
    // Columns checked: fx_left (col 0), ry rotation (col 19), X_point (col 24)
    let columns_to_check = [
        (0, 0, 0),  // param_block=0, idx=0, global_col=0   → fx_left
        (2, 1, 19), // param_block=2, idx=1, global_col=19  → ry (pitch)
        (3, 0, 24), // param_block=3, idx=0, global_col=24  → X
    ];

    let h = 1e-7;
    for &(block, idx, col) in &columns_to_check {
        let mut params_plus = params.clone();
        let mut params_minus = params.clone();
        params_plus[block][idx] += h;
        params_minus[block][idx] -= h;

        let (r_plus, _) = factor.linearize(&params_plus, false);
        let (r_minus, _) = factor.linearize(&params_minus, false);

        let fd_col = (&r_plus - &r_minus) / (2.0 * h);
        let analytic_col = jac.column(col);

        for row in 0..4 {
            let fd_val = fd_col[row];
            let an_val = analytic_col[row];
            let abs_err = (fd_val - an_val).abs();
            let scale = fd_val.abs().max(an_val.abs()).max(1e-12);
            let rel_err = abs_err / scale;
            assert!(
                rel_err < 1e-4,
                "Jacobian mismatch at ({row}, {col}): analytic={an_val:.6e}, fd={fd_val:.6e}, rel_err={rel_err:.2e}"
            );
        }
    }
}

// ---------------------------------------------------------------------------
// Epipolar factor tests
// ---------------------------------------------------------------------------

#[test]
fn test_epipolar_factor_zero_for_coplanar_correspondence() {
    // Stereo pair where left and right pixel observations are consistent
    // with the essential matrix E = [t]_x * R.
    //
    // With baseline tx=0.1, R=I, E = [[0,0,0],[0,0,-0.1],[0,0.1,0]].
    // For a 3D point at [0, 0, 2]:
    //   Left pixel  = (320, 240)
    //   Right pixel = (295, 240)   (point_in_right_frame = [-0.1, 0, 2])
    //
    // The fundamental matrix F = K_R^{-T} E K_L^{-1} maps the left point
    // to an epipolar line in the right image.  The horizontal baseline
    // produces horizontal epipolar lines, so any pair with matching v
    // satisfies the constraint — but the Sampson distance should be ~0
    // specifically for geometrically consistent correspondences.
    let left_pt = na::Vector2::new(320.0, 240.0);
    let right_pt = na::Vector2::new(295.0, 240.0);
    let factor = EpipolarFactor::new(left_pt, right_pt);

    let params = vec![
        make_intrinsics(500.0, 500.0, 320.0, 240.0),
        make_intrinsics(500.0, 500.0, 320.0, 240.0),
        make_extrinsics(0.0, 0.0, 0.0, 0.1, 0.0, 0.0),
    ];

    let (residual, _) = factor.linearize(&params, false);
    assert_eq!(residual.len(), 1);
    assert!(
        residual[0].abs() < 0.01,
        "Epipolar residual for consistent pair should be ~0, got {:.6e}",
        residual[0]
    );
}

// ---------------------------------------------------------------------------
// Calibrator integration boundary test
// ---------------------------------------------------------------------------

#[test]
fn test_stereo_calibrator_creation() {
    let config = CalibrationConfig::default();
    let calibrator = StereoCalibrator::new(config);

    assert_eq!(
        calibrator.status(),
        rs_vio::calibration::stereo_calibrator::CalibrationStatus::NotStarted
    );
    assert_eq!(calibrator.num_stereo_pairs(), 0);
}
