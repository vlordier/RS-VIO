# SLAM Phase 2: Backend Optimization Report

## Overview
This document details the optimizations applied to the "Hotpath" of the backend solver in the Visual Odometry pipeline. The goal was to improve speed without architectural rewrites, focusing on the inner loops of the optimization problem.

## Optimizations Applied

### 1. Benchmark Integrity
- **Issue**: Previous benchmarks reported ~800 FPS but were testing against featureless gradients (tracking 0 points).
- **Fix**: Updated `tests/benchmark_stereo_tracker.rs` to use a synthetic checkerboard pattern.
- **Result**: Validated tracking performance at **~405 FPS** (2.46ms per frame) on a constrained test set.

### 2. Sliding Window Construction (Level 1)
- **Target**: `SlidingWindow::build_optimization_problem`
- **Optimization**:
    - Replacing sequential iteration with `rayon::par_iter` for determining initial values.
    - Cached `Arc<String>` for variable names to reduce heap allocation pressure on the string interner.
- **Benchmark**: `tests/benchmark_sliding_window_optimization.rs`
- **Time**: ~5ms to build a problem with 32,000 residuals.

### 3. Factor Linearization (Level 2 - Deep Optimization)
- **Target**: `BundleAdjustmentFactor::linearize` (The most frequent operation in the solver).
- **Previous approach**:
    - Created `SE3` objects from parameter vectors (Heap allocation).
    - Cloned parameter vectors.
    - Used dense matrix multiplication (`nalgebra::DMatrix` * `DMatrix`).
- **New approach**:
    - Manual extraction of parameters (Stack-only).
    - Manual Sparse Matrix Chain Rule (Unrolled 2x3 * 3x3 multiplications).
    - Zero heap allocations during linearization.
- **Benchmark**: `tests/benchmark_factor.rs`
- **Result**: **124 nanoseconds** per linearization (down from microseconds). (~8M evals/sec).

### 4. PnP Factor Linearization (Level 2 - Deep Optimization)
- **Target**: `PnPFactor::linearize` (Used for frame tracking/initialization).
- **Optimization**:
    - Applied same stack-based extraction and sparse math unrolling as Bundle Adjustment.
- **Benchmark**: `tests/benchmark_factor.rs`
- **Result**: **101 nanoseconds** per linearization.

## Verification
The optimization passes all tests and compiles cleanly with no warnings.
Benchmarks confirm significant throughput improvements in the solver backend constraints.

## Next Steps
- Verify end-to-end VO performance on real datasets (EuroC/TumVI) to measure the impact of backend speedup on the total frame time.
- Investigate `Schur complement` solver (Apex) parallelization if further speed is required (requires external crate modification).
