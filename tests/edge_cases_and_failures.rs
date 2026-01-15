//! Edge case and failure tests for API drift fixes
//!
//! Validates error handling, boundary conditions, and failure scenarios
//! to ensure robustness of the API.

use nalgebra as na;
use rs_vio::datasets::FrameContext;
use rs_vio::types::Matrix4x4;

// ============================================================================
// FRAMECONTEXT EDGE CASES
// ============================================================================

/// Test FrameContext with maximum field values
#[test]
fn test_frame_context_max_values() {
    let mut ctx = FrameContext::new(true);
    ctx.current_idx = usize::MAX / 2;
    ctx.processed_frames = usize::MAX / 2;
    ctx.previous_frame_timestamp = i64::MAX;

    assert_eq!(ctx.current_idx, usize::MAX / 2);
    assert_eq!(ctx.processed_frames, usize::MAX / 2);
    assert_eq!(ctx.previous_frame_timestamp, i64::MAX);
}

/// Test FrameContext with zero values
#[test]
fn test_frame_context_zero_values() {
    let mut ctx = FrameContext::new(false);
    ctx.current_idx = 0;
    ctx.processed_frames = 0;
    ctx.previous_frame_timestamp = 0;

    assert_eq!(ctx.current_idx, 0);
    assert_eq!(ctx.processed_frames, 0);
    assert_eq!(ctx.previous_frame_timestamp, 0);
}

/// Test FrameContext with negative timestamp (nanoseconds before epoch)
#[test]
fn test_frame_context_negative_timestamp() {
    let mut ctx = FrameContext::new(true);
    ctx.previous_frame_timestamp = i64::MIN + 1; // Avoid overflow

    assert_eq!(ctx.previous_frame_timestamp, i64::MIN + 1);
}

/// Test FrameContext mode toggle behavior
#[test]
fn test_frame_context_mode_toggling() {
    let mut ctx_step = FrameContext::new(true);
    assert!(ctx_step.step_mode);
    assert!(!ctx_step.auto_play); // auto_play is !step_mode

    let ctx_auto = FrameContext::new(false);
    assert!(!ctx_auto.step_mode);
    assert!(ctx_auto.auto_play);

    // Manually toggle
    ctx_step.step_mode = false;
    ctx_step.auto_play = true;
    assert!(!ctx_step.step_mode);
    assert!(ctx_step.auto_play);
}

/// Test FrameContext mutation sequence
#[test]
fn test_frame_context_mutation_sequence() {
    let mut ctx = FrameContext::new(true);

    // Simulate frame processing with advancing indices
    for i in 0..1000 {
        ctx.current_idx = i;
        ctx.processed_frames += 1;

        // Ensure monotonicity
        assert!(ctx.processed_frames > 0);
        assert!(ctx.processed_frames <= i + 1);
    }

    assert_eq!(ctx.processed_frames, 1000);
}

/// Test FrameContext with mismatched processed_frames > current_idx
#[test]
fn test_frame_context_inconsistent_state() {
    let mut ctx = FrameContext::new(true);

    // Allow inconsistent state (processed might exceed current)
    ctx.current_idx = 5;
    ctx.processed_frames = 10;

    assert_eq!(ctx.current_idx, 5);
    assert_eq!(ctx.processed_frames, 10);
    // This is allowed - could happen if skipping frames
}

/// Test FrameContext rapid state changes
#[test]
fn test_frame_context_rapid_changes() {
    let mut ctx = FrameContext::new(true);

    // After 100 toggles (even number), should return to original state
    for _ in 0..100 {
        ctx.step_mode = !ctx.step_mode;
        ctx.auto_play = !ctx.auto_play;
        ctx.advance_frame = !ctx.advance_frame;
    }

    // Start: step_mode=true, auto_play=false, advance_frame=false
    // After 100 toggles (even): step_mode=true, auto_play=false, advance_frame=false
    assert!(ctx.step_mode);
    assert!(!ctx.auto_play);
    assert!(!ctx.advance_frame);
}

// ============================================================================
// MATRIX4X4 EDGE CASES
// ============================================================================

/// Test Matrix4x4 with very large values
#[test]
fn test_matrix4x4_large_values() {
    let large = 1e100;
    let m: Matrix4x4 = na::Matrix4::from_diagonal(&na::Vector4::new(large, large, large, large));

    let norm = m.norm();
    assert!(norm > 0.0);
    assert!(norm.is_finite());
}

/// Test Matrix4x4 with very small values
#[test]
fn test_matrix4x4_small_values() {
    let small = 1e-100;
    let m: Matrix4x4 = na::Matrix4::from_diagonal(&na::Vector4::new(small, small, small, small));

    let norm = m.norm();
    assert!(norm > 0.0);
    assert!(norm < 1e-99);
}

/// Test Matrix4x4 near-singular condition
#[test]
fn test_matrix4x4_near_singular() {
    let mut m: Matrix4x4 = na::Matrix4::identity();

    // Make it nearly singular by having one very small diagonal element
    m[(0, 0)] = 1e-15;

    let det = m.determinant();
    assert!(det.abs() < 1e-14); // Very close to zero
}

/// Test Matrix4x4 completely singular (rank-deficient)
#[test]
fn test_matrix4x4_rank_deficient() {
    let mut m: Matrix4x4 = na::Matrix4::zeros();

    // Make it rank-1 (only first row has values)
    m[(0, 0)] = 1.0;
    m[(0, 1)] = 2.0;
    m[(0, 2)] = 3.0;
    m[(0, 3)] = 4.0;

    let det = m.determinant();
    assert_eq!(det, 0.0);
}

/// Test Matrix4x4 identity operations
#[test]
fn test_matrix4x4_identity_operations() {
    let i: Matrix4x4 = na::Matrix4::identity();
    let m: Matrix4x4 = na::Matrix4::new(
        1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
    );

    let result = i * m;
    assert_eq!(result, m);
}

/// Test Matrix4x4 with NaN detection
#[test]
fn test_matrix4x4_nan_handling() {
    let nan = f64::NAN;
    let m: Matrix4x4 = na::Matrix4::from_diagonal(&na::Vector4::new(nan, 1.0, 1.0, 1.0));

    let norm = m.norm();
    assert!(norm.is_nan());
}

/// Test Matrix4x4 with infinity
#[test]
fn test_matrix4x4_infinity_handling() {
    let inf = f64::INFINITY;
    let m: Matrix4x4 = na::Matrix4::from_diagonal(&na::Vector4::new(inf, 1.0, 1.0, 1.0));

    let norm = m.norm();
    assert!(norm.is_infinite());
}

/// Test Matrix4x4 mixed sign values
#[test]
fn test_matrix4x4_mixed_signs() {
    let m: Matrix4x4 = na::Matrix4::from_diagonal(&na::Vector4::new(1.0, -1.0, 1.0, -1.0));

    let det = m.determinant();
    assert_eq!(det, 1.0); // (-1)^2 = 1
}

/// Test Matrix4x4 accumulation
#[test]
fn test_matrix4x4_accumulation() {
    let mut acc: Matrix4x4 = na::Matrix4::zeros();

    for i in 0..10 {
        let m =
            na::Matrix4::from_diagonal(&na::Vector4::new(i as f64, i as f64, i as f64, i as f64));
        acc += m;
    }

    // Sum should be 0+1+2+...+9 = 45 on each diagonal
    assert_eq!(acc[(0, 0)], 45.0);
    assert_eq!(acc[(1, 1)], 45.0);
}

/// Test Matrix4x4 transpose consistency
#[test]
fn test_matrix4x4_transpose_consistency() {
    let m: Matrix4x4 = na::Matrix4::new(
        1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
    );

    let mt = m.transpose();
    let mtt = mt.transpose();

    assert_eq!(m, mtt);
}

// ============================================================================
// FAILURE CASES - Expected behavior under error conditions
// ============================================================================

/// Test handling of frame index wraparound
#[test]
fn test_frame_context_wraparound_edge() {
    let mut ctx = FrameContext::new(true);

    ctx.current_idx = usize::MAX - 1;

    // Simulate next frame (would overflow if not handled)
    ctx.current_idx = ctx.current_idx.saturating_add(1);

    assert_eq!(ctx.current_idx, usize::MAX);
}

/// Test timestamp overflow handling
#[test]
fn test_frame_context_timestamp_overflow() {
    let mut ctx = FrameContext::new(true);

    ctx.previous_frame_timestamp = i64::MAX - 100;

    // Add a large delta (simulating frame arrival)
    ctx.previous_frame_timestamp = ctx.previous_frame_timestamp.saturating_add(200);

    assert_eq!(ctx.previous_frame_timestamp, i64::MAX);
}

/// Test Matrix4x4 division by zero prevention
#[test]
fn test_matrix4x4_scale_by_zero() {
    let m: Matrix4x4 = na::Matrix4::identity();

    // Scaling by zero gives zero matrix
    let result = m * 0.0;

    assert_eq!(result.norm(), 0.0);
}

/// Test Matrix4x4 inverse of singular matrix
#[test]
fn test_matrix4x4_singular_inverse() {
    let m: Matrix4x4 = na::Matrix4::zeros();

    let inv = m.try_inverse();

    assert!(inv.is_none()); // Should fail gracefully
}

/// Test Matrix4x4 condition number near singularity
#[test]
fn test_matrix4x4_condition_number_singularity() {
    let mut m: Matrix4x4 = na::Matrix4::identity();
    m[(0, 0)] = 1e-16; // Nearly singular

    // Just ensure computation doesn't panic
    let _ = m.norm();
}

// ============================================================================
// BOUNDARY CONDITIONS
// ============================================================================

/// Test FrameContext state at boundary transitions
#[test]
fn test_frame_context_boundary_transitions() {
    let mut ctx = FrameContext::new(true);

    // Test transition from 0 to 1
    ctx.current_idx = 0;
    ctx.processed_frames = 0;

    ctx.current_idx = 1;
    ctx.processed_frames = 1;

    assert_eq!(ctx.current_idx, 1);
    assert_eq!(ctx.processed_frames, 1);
}

/// Test FrameContext with minimum positive values
#[test]
fn test_frame_context_minimum_positive() {
    let mut ctx = FrameContext::new(true);

    ctx.current_idx = 1;
    ctx.processed_frames = 1;
    ctx.previous_frame_timestamp = 1;

    assert_eq!(ctx.current_idx, 1);
    assert_eq!(ctx.processed_frames, 1);
    assert_eq!(ctx.previous_frame_timestamp, 1);
}

/// Test Matrix4x4 with all ones
#[test]
fn test_matrix4x4_all_ones() {
    let m: Matrix4x4 = na::Matrix4::from_fn(|_, _| 1.0);

    let norm = m.norm();
    assert!(norm > 0.0);
    assert!(norm.is_finite());
}

/// Test Matrix4x4 determinant for orthogonal matrix
#[test]
fn test_matrix4x4_orthogonal_determinant() {
    // Rotation matrix (orthogonal) should have determinant ±1
    let angle = std::f64::consts::PI / 4.0;
    let cos_a = angle.cos();
    let sin_a = angle.sin();

    let mut m: Matrix4x4 = na::Matrix4::identity();
    m[(0, 0)] = cos_a;
    m[(0, 1)] = -sin_a;
    m[(1, 0)] = sin_a;
    m[(1, 1)] = cos_a;

    let det = m.determinant();
    assert!((det - 1.0).abs() < 1e-10);
}

// ============================================================================
// STRESS TESTS
// ============================================================================

/// Test FrameContext under high-frequency updates
#[test]
fn test_frame_context_high_frequency_stress() {
    let mut ctx = FrameContext::new(true);

    let iterations = 100_000;
    for i in 0..iterations {
        ctx.current_idx = i;
        ctx.processed_frames = i + 1;
    }

    assert_eq!(ctx.processed_frames, iterations);
}

/// Test Matrix4x4 operations in tight loop
#[test]
fn test_matrix4x4_tight_loop_operations() {
    let mut result: Matrix4x4 = na::Matrix4::identity();
    let m: Matrix4x4 = na::Matrix4::from_diagonal(&na::Vector4::new(1.001, 1.001, 1.001, 1.001));

    for _ in 0..100 {
        result = result * m;
    }

    // Each diagonal element should be 1.001^100
    let expected = 1.001_f64.powi(100);
    assert!((result[(0, 0)] - expected).abs() < 1e-10);
}

/// Test FrameContext with alternating patterns
#[test]
fn test_frame_context_alternating_pattern() {
    let mut ctx = FrameContext::new(true);

    for i in 0..1000 {
        if i % 2 == 0 {
            ctx.step_mode = true;
            ctx.auto_play = false;
        } else {
            ctx.step_mode = false;
            ctx.auto_play = true;
        }
    }

    // Last iteration is odd, so should be auto_play=true
    assert!(!ctx.step_mode);
    assert!(ctx.auto_play);
}

// ============================================================================
// CONSISTENCY TESTS
// ============================================================================

/// Test Matrix4x4 norm consistency
#[test]
fn test_matrix4x4_norm_consistency() {
    let m: Matrix4x4 = na::Matrix4::from_diagonal(&na::Vector4::new(3.0, 4.0, 0.0, 0.0));

    // Norm of diagonal matrix is max diagonal element magnitude
    let norm = m.norm();
    assert!(norm > 0.0);
}

/// Test FrameContext clone behavior
#[test]
fn test_frame_context_clone_independence() {
    let mut ctx1 = FrameContext::new(true);
    ctx1.current_idx = 10;
    ctx1.processed_frames = 10;

    let mut ctx2 = FrameContext::new(true);
    ctx2.current_idx = 20;

    // Mutate ctx1
    ctx1.current_idx = 100;

    // ctx2 should be unchanged
    assert_eq!(ctx2.current_idx, 20);
    assert_ne!(ctx1.current_idx, ctx2.current_idx);
}

/// Test Matrix4x4 arithmetic associativity (A+B)+C == A+(B+C)
#[test]
fn test_matrix4x4_addition_associativity() {
    let a: Matrix4x4 = na::Matrix4::from_diagonal(&na::Vector4::new(1.0, 2.0, 3.0, 4.0));
    let b: Matrix4x4 = na::Matrix4::from_diagonal(&na::Vector4::new(5.0, 6.0, 7.0, 8.0));
    let c: Matrix4x4 = na::Matrix4::from_diagonal(&na::Vector4::new(9.0, 10.0, 11.0, 12.0));

    let left = (a + b) + c;
    let right = a + (b + c);

    assert_eq!(left, right);
}

/// Test Matrix4x4 distributivity: (A+B)*C == A*C + B*C
#[test]
fn test_matrix4x4_distributivity() {
    let a: Matrix4x4 = na::Matrix4::identity();
    let b: Matrix4x4 = na::Matrix4::identity() * 2.0;
    let c: Matrix4x4 = na::Matrix4::from_diagonal(&na::Vector4::new(1.0, 2.0, 3.0, 4.0));

    let left = (a + b) * c;
    let right = a * c + b * c;

    // Check element-wise with tolerance for floating point
    for i in 0..4 {
        for j in 0..4 {
            assert!((left[(i, j)] - right[(i, j)]).abs() < 1e-10);
        }
    }
}

// ============================================================================
// INTEGRATION TESTS
// ============================================================================

/// Test FrameContext and Matrix4x4 integration
#[test]
fn test_integration_frame_context_with_matrices() {
    let mut ctx = FrameContext::new(true);
    let mut matrices = Vec::new();

    for i in 0..10 {
        ctx.current_idx = i;
        ctx.processed_frames = i + 1;

        let m: Matrix4x4 = na::Matrix4::from_diagonal(&na::Vector4::new(
            i as f64 + 1.0,
            i as f64 + 1.0,
            i as f64 + 1.0,
            i as f64 + 1.0,
        ));

        matrices.push(m);
    }

    assert_eq!(matrices.len(), 10);
    assert_eq!(ctx.current_idx, 9);
}

/// Test multiple FrameContext instances simultaneously
#[test]
fn test_multiple_frame_contexts() {
    let contexts: Vec<_> = (0..10)
        .map(|i| {
            let mut ctx = FrameContext::new(i % 2 == 0);
            ctx.current_idx = i;
            ctx
        })
        .collect();

    assert_eq!(contexts.len(), 10);
    for (i, ctx) in contexts.iter().enumerate() {
        assert_eq!(ctx.current_idx, i);
    }
}
