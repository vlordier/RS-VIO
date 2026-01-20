use super::*;
use apex_solver::factors::Factor;
use na::DVector;
use nalgebra as na;

fn se3_identity_vec() -> DVector<f64> {
    DVector::from_vec(vec![0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
}

#[test]
fn loop_closure_zero_residual_for_perfect_measurement() {
    let factor = LoopClosurePoseFactor::new(na::Matrix4::identity(), na::Matrix6::identity());
    let (res, jac) = factor.linearize(&[se3_identity_vec(), se3_identity_vec()], true);
    assert!(
        res.amax() < 1e-9,
        "residual should be zero for identity measurement"
    );
    let jac = jac.expect("jacobian should be present");
    assert_eq!(jac.nrows(), 6);
    assert_eq!(jac.ncols(), 12);
}

#[test]
fn loop_closure_translation_error_propagates() {
    let T_1_2 = na::Isometry3::translation(0.0, 0.0, 0.0).to_homogeneous();
    let info = na::Matrix6::identity();
    let factor = LoopClosurePoseFactor::new(T_1_2, info);

    let pose1 = DVector::from_vec(vec![1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]);
    let pose2 = se3_identity_vec();
    let (res, _) = factor.linearize(&[pose1, pose2], false);
    assert!((res[0] - 1.0).abs() < 1e-6);
}
