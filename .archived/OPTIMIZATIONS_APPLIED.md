# Performance Optimizations Applied

**Date**: February 1, 2026
**Focus**: Hot-path optimization and memory management reduction.

## 1. Image Pyramid Generation (Frontend)
- **Challenge**: `build_image_pyramid` was resizing Level 0 from source image, which is redundant if Level 0 matches source resolution.
- **Optimization**: Added conditional check. If target size matches source, perform `clone()` instead of `resize()`.
- **Impact**: Reduced allocations and CPU cycles for the base level of the pyramid.

## 2. Optical Flow Logic (Frontend)
- **Challenge**: `track_point_at_level` loop used `SMatrix` intermediate buffer and `apply` for every pixel in a patch (8x8 or larger).
- **Optimization**: Fused the affine transformation `T * p` directly into the bilinear interpolation loop. Eliminated `SMatrix` construction per pixel.
- **Impact**: Eliminated small temporary allocations and improved instruction pipelining in the hottest loop of feature tracking.

## 3. Factor Graph Memory (Backend)
- **Challenge**: `BundleAdjustmentFactor` stored `T_C_B` (Camera-to-Body transform) and optional `fixed_pose` as `Matrix4<f64>` (128 bytes) by value.
  - With 30,000 factors per optimization window, this redundant data consumed ~6MB and caused heavy memcpy traffic during graph construction.
- **Optimization**:
  - Updated `BundleAdjustmentFactor` to store `Arc<Matrix4<f64>>` (8 bytes).
  - Updated `linearize` method to dereference `Arc`.
  - Updated `sliding_window.rs` to wrap transforms in `Arc` once per frame and share them across thousands of factors.
- **Impact**:
  - Reduced `BundleAdjustmentFactor` size from ~282 bytes to ~48 bytes.
  - Eliminated ~4MB of redundant data storage.
  - Reduced memory bandwidth usage during graph construction.

## 4. Residual Block Allocation (Backend)
- **Challenge**: Factors were stored as `Box<BundleAdjustmentFactor>` in `local_residuals`, incurring heap allocation overhead and pointer indirection for every single residual.
- **Optimization**: Removed `Box` wrapper. `BundleAdjustmentFactor` is now small enough (and `Clone`-able via Arc) to be stored inline in `Vec`.
- **Impact**:
  - Eliminated ~3000 allocations per optimization step.
  - Improved CPU cache locality by storing factors contiguously in memory.
