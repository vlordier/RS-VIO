#![allow(clippy::unwrap_used)]

use apex_solver::factors::Factor;
use na::{DVector, Matrix4, Vector2, Vector3};
use nalgebra as na;
use rs_vio::optimization::factors::BundleAdjustmentFactor;
use std::time::Instant;

#[test]
fn bench_factor_linearize() {
    let obs = Vector2::new(0.5, 0.5);
    let T_C_B = std::sync::Arc::new(Matrix4::identity());
    let factor = BundleAdjustmentFactor::new(obs, T_C_B);

    // Initial guess
    let p_W = Vector3::new(10.0, 5.0, 20.0);
    // T_B_W (Identity)
    let T_B_W = DVector::from_vec(vec![0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]); // tx,ty,tz, qw,qx,qy,qz

    let params = vec![DVector::from_vec(vec![p_W.x, p_W.y, p_W.z]), T_B_W];

    // Warmup
    factor.linearize(&params, true);

    let start = Instant::now();
    let iterations = 100_000;
    for _ in 0..iterations {
        // Just execute, don't try to convert the return value (it's a DMatrix)
        let _ = factor.linearize(&params, true);
    }
    let duration = start.elapsed();

    println!(
        "BENCHMARK: BundleAdjustmentFactor::linearize took: {:?} for {} iterations",
        duration, iterations
    );
    println!("  Avg: {:?}", duration / iterations as u32);

    // Correctness: residual should be finite, Jacobian should have correct dimensions
    let (residual, jacobian) = factor.linearize(&params, true);
    assert!(residual.iter().all(|v| v.is_finite()), "Residual must be finite");
    assert_eq!(residual.len(), 2, "Reprojection residual must be 2D");
    let jac = jacobian.unwrap();
    assert_eq!(jac.nrows(), 2, "Jacobian rows = residual dim");
    assert_eq!(jac.ncols(), 9, "Jacobian cols = 3 (point) + 6 (pose)");
    assert!(jac.iter().all(|v| v.is_finite()), "Jacobian must be finite");
}

#[test]
fn bench_pnp_factor_linearize() {
    use rs_vio::optimization::factors::PnPFactor;

    let obs = Vector2::new(0.5, 0.5);
    let T_C_B = Matrix4::identity();
    let p_W = Vector3::new(10.0, 5.0, 20.0);

    let factor = PnPFactor::new(obs, T_C_B, p_W);

    // Initial guess for T_B_W (Identity)
    // 7 params: tx,ty,tz, qw,qx,qy,qz
    let T_B_W = DVector::from_vec(vec![0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]);

    let params = vec![T_B_W];

    // Warmup
    factor.linearize(&params, true);

    let start = Instant::now();
    let iterations = 100_000;
    for _ in 0..iterations {
        let _ = factor.linearize(&params, true);
    }
    let duration = start.elapsed();

    println!(
        "BENCHMARK: PnPFactor::linearize took: {:?} for {} iterations",
        duration, iterations
    );
    println!("  Avg: {:?}", duration / iterations as u32);

    // Correctness: residual should be finite, Jacobian should have correct dimensions
    let (residual, jacobian) = factor.linearize(&params, true);
    assert!(residual.iter().all(|v| v.is_finite()), "Residual must be finite");
    assert_eq!(residual.len(), 2, "Reprojection residual must be 2D");
    let jac = jacobian.unwrap();
    assert_eq!(jac.nrows(), 2, "Jacobian rows = residual dim");
    assert_eq!(jac.ncols(), 6, "Jacobian cols = 6 (pose)");
    assert!(jac.iter().all(|v| v.is_finite()), "Jacobian must be finite");
}
