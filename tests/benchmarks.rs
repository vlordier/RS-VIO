#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    clippy::cast_precision_loss,
    clippy::needless_range_loop,
    clippy::cast_lossless,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::unnecessary_cast,
    clippy::assign_op_pattern,
    clippy::field_reassign_with_default,
    clippy::redundant_pattern_matching
)]

//! Benchmark tests for performance validation
//!
//! These tests measure performance characteristics of critical paths
//! in the RS-VIO system. Not all are marked as benchmarks; some measure
//! performance characteristics within unit test bounds.

use nalgebra as na;
use rs_vio::datasets::FrameContext;
use rs_vio::types::Matrix4x4;
use std::time::Instant;

// ============================================================================
// FRAMECONTEXT PERFORMANCE
// ============================================================================

/// Measure FrameContext creation time
#[test]
fn bench_frame_context_creation() {
    let start = Instant::now();

    for _ in 0..10_000 {
        let _ = FrameContext::new(true);
    }

    let duration = start.elapsed();
    println!("FrameContext creation (10k): {:?}", duration);

    // Should be very fast (< 1ms for 10k)
    assert!(duration.as_millis() < 10, "FrameContext creation too slow");
}

/// Measure FrameContext field updates
#[test]
fn bench_frame_context_updates() {
    let mut ctx = FrameContext::new(true);
    let start = Instant::now();

    for i in 0..100_000 {
        ctx.current_idx = i;
        ctx.processed_frames = i;
        ctx.step_mode = i % 2 == 0;
    }

    let duration = start.elapsed();
    println!("FrameContext updates (100k): {:?}", duration);

    assert!(duration.as_millis() < 10, "FrameContext updates too slow");
    assert_eq!(ctx.current_idx, 99_999);
}

/// Measure FrameContext batch construction
#[test]
fn bench_frame_context_batch_construction() {
    let start = Instant::now();

    let _contexts: Vec<_> = (0..10_000)
        .map(|i| {
            let mut ctx = FrameContext::new(i % 2 == 0);
            ctx.current_idx = i;
            ctx
        })
        .collect();

    let duration = start.elapsed();
    println!("FrameContext batch construction (10k): {:?}", duration);

    assert!(duration.as_millis() < 20, "Batch construction too slow");
}

// ============================================================================
// MATRIX4X4 PERFORMANCE
// ============================================================================

/// Measure Matrix4x4 creation time
#[test]
fn bench_matrix4x4_creation() {
    let start = Instant::now();

    for _ in 0..10_000 {
        let _: Matrix4x4 = na::Matrix4::identity();
    }

    let duration = start.elapsed();
    println!("Matrix4x4 creation (10k): {:?}", duration);

    assert!(duration.as_millis() < 5, "Matrix creation too slow");
}

/// Measure Matrix4x4 arithmetic operations
#[test]
fn bench_matrix4x4_addition() {
    let m1: Matrix4x4 = na::Matrix4::identity();
    let m2: Matrix4x4 = na::Matrix4::identity() * 2.0;

    let start = Instant::now();

    let mut result = m1;
    for _ in 0..100_000 {
        result = result + m2;
    }

    let duration = start.elapsed();
    println!("Matrix4x4 addition (100k): {:?}", duration);

    assert!(duration.as_millis() < 100, "Matrix addition too slow");

    // Use result to prevent optimization
    assert!(result.norm() > 0.0);
}

/// Measure Matrix4x4 multiplication
#[test]
fn bench_matrix4x4_multiplication() {
    let m: Matrix4x4 = na::Matrix4::<f64>::from_diagonal(&na::Vector4::new(1.01, 1.01, 1.01, 1.01));

    let start = Instant::now();

    let mut result = na::Matrix4::<f64>::identity();
    for _ in 0..10_000 {
        result = result * m;
    }

    let duration = start.elapsed();
    println!("Matrix4x4 multiplication (10k): {:?}", duration);

    assert!(duration.as_millis() < 100, "Matrix multiplication too slow");

    // Result will have finite norm (diagonal matrices)
    assert!(result.norm().is_finite());
}

/// Measure Matrix4x4 determinant computation
#[test]
fn bench_matrix4x4_determinant() {
    let m: Matrix4x4 = na::Matrix4::from_diagonal(&na::Vector4::new(1.0, 2.0, 3.0, 4.0));

    let start = Instant::now();

    let mut sum = 0.0;
    for _ in 0..10_000 {
        sum += m.determinant();
    }

    let duration = start.elapsed();
    println!("Matrix4x4 determinant (10k): {:?}", duration);

    assert!(duration.as_millis() < 50, "Determinant too slow");

    assert!(sum > 0.0);
}

/// Measure Matrix4x4 inverse computation
#[test]
fn bench_matrix4x4_inverse() {
    let m: Matrix4x4 = na::Matrix4::from_diagonal(&na::Vector4::new(1.0, 2.0, 3.0, 4.0));

    let start = Instant::now();

    let mut count = 0;
    for _ in 0..10_000 {
        if let Some(_) = m.try_inverse() {
            count += 1;
        }
    }

    let duration = start.elapsed();
    println!("Matrix4x4 inverse (10k): {:?}", duration);

    assert!(duration.as_millis() < 100, "Inverse too slow");
    assert_eq!(count, 10_000);
}

/// Measure Matrix4x4 norm computation
#[test]
fn bench_matrix4x4_norm() {
    let m: Matrix4x4 = na::Matrix4::from_diagonal(&na::Vector4::new(1.0, 2.0, 3.0, 4.0));

    let start = Instant::now();

    let mut sum = 0.0;
    for _ in 0..10_000 {
        sum += m.norm();
    }

    let duration = start.elapsed();
    println!("Matrix4x4 norm (10k): {:?}", duration);

    assert!(duration.as_millis() < 20, "Norm computation too slow");

    assert!(sum > 0.0);
}

/// Measure Matrix4x4 transpose
#[test]
fn bench_matrix4x4_transpose() {
    let m: Matrix4x4 = na::Matrix4::new(
        1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
    );

    let start = Instant::now();

    let mut result = m;
    for _ in 0..10_000 {
        result = result.transpose();
    }

    let duration = start.elapsed();
    println!("Matrix4x4 transpose (10k): {:?}", duration);

    assert!(duration.as_millis() < 20, "Transpose too slow");

    assert!(result.norm() > 0.0);
}

// ============================================================================
// COMBINED OPERATION BENCHMARKS
// ============================================================================

/// Measure combined FrameContext and Matrix operations
#[test]
fn bench_integrated_frame_and_matrix_operations() {
    let start = Instant::now();

    let mut ctx = FrameContext::new(true);
    let mut matrices = Vec::with_capacity(1000);

    for i in 0..1000 {
        ctx.current_idx = i;
        ctx.processed_frames = i + 1;

        let m: Matrix4x4 = na::Matrix4::from_diagonal(&na::Vector4::new(
            1.0 + i as f64 * 0.001,
            1.0 + i as f64 * 0.001,
            1.0 + i as f64 * 0.001,
            1.0 + i as f64 * 0.001,
        ));

        matrices.push(m);
    }

    let duration = start.elapsed();
    println!("Integrated operations (1k iterations): {:?}", duration);

    assert!(duration.as_millis() < 50, "Integrated operations too slow");
    assert_eq!(matrices.len(), 1000);
    assert_eq!(ctx.current_idx, 999);
}

/// Measure cache efficiency with repeated matrix operations
#[test]
fn bench_matrix_cache_efficiency() {
    let matrices: Vec<Matrix4x4> = (0..100)
        .map(|i| {
            let scale = 0.99 + i as f64 * 0.0001;
            na::Matrix4::from_diagonal(&na::Vector4::new(scale, scale, scale, scale))
        })
        .collect();

    let start = Instant::now();

    let mut result: Matrix4x4 = na::Matrix4::<f64>::identity();
    for _ in 0..100 {
        for m in &matrices {
            result = result * m;
        }
    }

    let duration = start.elapsed();
    println!(
        "Matrix cache efficiency (10k multiplications): {:?}",
        duration
    );

    assert!(duration.as_millis() < 500, "Cache efficiency poor");

    assert!(result.norm().is_finite());
}

// ============================================================================
// WORST-CASE PERFORMANCE
// ============================================================================

/// Measure performance with singular matrix operations
#[test]
fn bench_singular_matrix_operations() {
    let m: Matrix4x4 = na::Matrix4::zeros();

    let start = Instant::now();

    let mut count = 0;
    for _ in 0..1_000 {
        if let Some(_) = m.try_inverse() {
            count += 1;
        }
    }

    let duration = start.elapsed();
    println!("Singular matrix inverse attempts (1k): {:?}", duration);

    assert!(
        duration.as_millis() < 50,
        "Singular matrix handling too slow"
    );
    assert_eq!(count, 0); // All should fail
}

/// Measure performance with large-value matrices
#[test]
fn bench_large_value_matrices() {
    let m: Matrix4x4 = na::Matrix4::from_diagonal(&na::Vector4::new(1e100, 1e100, 1e100, 1e100));

    let start = Instant::now();

    let mut sum = 0.0;
    for _ in 0..10_000 {
        sum += m.norm();
    }

    let duration = start.elapsed();
    println!("Large-value matrix operations (10k): {:?}", duration);

    assert!(duration.as_millis() < 20, "Large-value operations too slow");
    assert!(sum.is_finite());
}

// ============================================================================
// MEMORY AND ALLOCATION BENCHMARKS
// ============================================================================

/// Measure memory efficiency of FrameContext collections
#[test]
fn bench_frame_context_memory_efficiency() {
    let start = Instant::now();

    let contexts: Vec<_> = (0..100_000)
        .map(|i| {
            let mut ctx = FrameContext::new(i % 2 == 0);
            ctx.current_idx = i;
            ctx
        })
        .collect();

    let duration = start.elapsed();
    println!("FrameContext 100k allocation: {:?}", duration);

    assert!(duration.as_millis() < 200, "Memory allocation too slow");
    assert_eq!(contexts.len(), 100_000);
}

/// Measure memory efficiency of Matrix collections
#[test]
fn bench_matrix_memory_efficiency() {
    let start = Instant::now();

    let matrices: Vec<Matrix4x4> = (0..10_000)
        .map(|i| {
            na::Matrix4::from_diagonal(&na::Vector4::new(i as f64, i as f64, i as f64, i as f64))
        })
        .collect();

    let duration = start.elapsed();
    println!("Matrix4x4 10k allocation: {:?}", duration);

    assert!(duration.as_millis() < 100, "Matrix allocation too slow");
    assert_eq!(matrices.len(), 10_000);
}

// ============================================================================
// REAL-TIME PERFORMANCE (EMBEDDED CONSTRAINTS)
// ============================================================================

/// Simulate frame processing loop (30 Hz constraint)
/// Should complete in ~33ms per frame on real hardware
#[test]
fn bench_frame_processing_loop_simulation() {
    const NUM_FRAMES: usize = 100;

    let start = Instant::now();

    let mut ctx = FrameContext::new(true);

    for frame_num in 0..NUM_FRAMES {
        // Simulate frame arrival and processing
        ctx.current_idx = frame_num;
        ctx.processed_frames += 1;
        ctx.advance_frame = true;

        // Create and process matrices
        let _transform: Matrix4x4 = na::Matrix4::identity();
        let _pose: Matrix4x4 = na::Matrix4::from_diagonal(&na::Vector4::new(1.0, 1.0, 1.0, 1.0));

        // Simulate some computation
        let _ = _transform * _pose;
    }

    let duration = start.elapsed();
    let frame_time = duration.as_millis() / NUM_FRAMES as u128;

    println!(
        "Frame processing loop (100 frames): {:?} (avg {}ms/frame)",
        duration, frame_time
    );

    // Overall should be reasonable (not strict per-frame timing in unit tests)
    assert!(duration.as_millis() < 1000, "Frame loop too slow");
    assert_eq!(ctx.current_idx, NUM_FRAMES - 1);
}

/// Measure triangulation-like matrix operations
#[test]
fn bench_triangulation_matrix_operations() {
    const NUM_CORRESPONDENCES: usize = 1000;

    let start = Instant::now();

    let mut sum_norms = 0.0;

    for _ in 0..NUM_CORRESPONDENCES {
        let p1: Matrix4x4 = na::Matrix4::identity();
        let p2: Matrix4x4 = na::Matrix4::identity() * 1.5;

        let result = p1 * p2;
        sum_norms += result.norm();
    }

    let duration = start.elapsed();
    println!(
        "Triangulation matrix ops (1k correspondences): {:?}",
        duration
    );

    assert!(
        duration.as_millis() < 200,
        "Triangulation operations too slow"
    );
    assert!(sum_norms > 0.0);
}

// ============================================================================
// SCALABILITY TESTS
// ============================================================================

/// Test performance scaling with growing FrameContext operations
#[test]
fn bench_scaling_frame_context_operations() {
    for scale in [100, 1_000, 10_000].iter() {
        let start = Instant::now();

        let mut ctx = FrameContext::new(true);

        for i in 0..*scale {
            ctx.current_idx = i;
            ctx.processed_frames = i + 1;
        }

        let duration = start.elapsed();
        println!("FrameContext scaling {} operations: {:?}", scale, duration);

        assert_eq!(ctx.current_idx, scale - 1);
    }
}

/// Test performance scaling with matrix operations
#[test]
fn bench_scaling_matrix_operations() {
    for scale in [100, 1_000, 10_000].iter() {
        let m: Matrix4x4 = na::Matrix4::identity();

        let start = Instant::now();

        let mut sum = 0.0;
        for _ in 0..*scale {
            sum += m.norm();
        }

        let duration = start.elapsed();
        println!("Matrix scaling {} operations: {:?}", scale, duration);

        assert!(sum > 0.0);
    }
}
