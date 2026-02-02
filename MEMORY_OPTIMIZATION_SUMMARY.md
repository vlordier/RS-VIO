# Memory Optimization Report: Factor Graph

**Date**: February 2, 2026
**Status**: ✅ COMPLETE
**Module**: `src/optimization/factors.rs`, `src/estimator/sliding_window.rs`

## Problem
The `BundleAdjustmentFactor` struct was storing 4x4 transformation matrices (`Matrix4<f64>`, 128 bytes) by value. 
- **T_C_B** (Camera to Body): Constant for all factors of a specific camera.
- **fixed_pose**: Optional constant pose for fixed frames.

With a sliding window of 10-20 frames and thousands of landmarks, the optimization problem creates 20,000 - 50,000 factors.
- **Previous Size**: ~280 bytes per factor.
- **Total Memory**: ~14 MB for 50k factors (redundant data).
- **Overhead**: Frequent memory copies when resizing vectors or moving factors between threads.

## Solution implemented
Refactored `BundleAdjustmentFactor` to use `std::sync::Arc` for shared matrices.

### 1. Structure Change
```rust
// Before
pub struct BundleAdjustmentFactor {
    pub observation: Vector2<f64>,
    pub T_C_B: Matrix4<f64>,              // 128 bytes
    pub fixed_pose: Option<Matrix4<f64>>, // ~136 bytes
}

// After
pub struct BundleAdjustmentFactor {
    pub observation: Vector2<f64>,
    pub T_C_B: Arc<Matrix4<f64>>,              // 8 bytes (pointer)
    pub fixed_pose: Option<Arc<Matrix4<f64>>>, // 8 bytes (pointer)
}
```

### 2. Allocation Strategy
- **Per Frame**: `T_C_B` (left/right) is wrapped in `Arc` once.
- **Per Factor**: The `Arc` is cloned (cheap atomic increment), not the data.
- **Residual Storage**: Removed `Box<BundleAdjustmentFactor>` wrapper in intermediate vectors. Factors are now small enough to store contiguously in `Vec<BundleAdjustmentFactor>`, improving cache locality.

## Results
- **Struct Size**: Reduced by **~85%** (from ~280 bytes to ~32-40 bytes).
- **Memory Savings**: ~10-12 MB pure data reduction for a typical window.
- **Performance**: Reduced allocator pressure and `memcpy` operations during graph construction.
- **Correctness**: Build passing (`cargo check --lib`).
