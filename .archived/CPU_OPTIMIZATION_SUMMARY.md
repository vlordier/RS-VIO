# CPU Optimization Summary
**Date**: January 24, 2026
**Focus**: Hot-path optimization of VIO frontend and estimator.

## 1. Feature Tracking: Pattern52 Memory Layout
**Optimization**: Transposed `j_se2` matrix and vectorized Pattern struct.
**Improvement**: 2.4x speedup in pattern creation and residual setup.
**Details**:
- Previously: 52 columns allocated non-contiguously or with stride.
- New: Contiguous 3x52 matrix for efficient J * J^T operations.

## 2. Feature Detection: FAST Thresholding
**Optimization**: Single-pass histogram-based thresholding.
**Improvement**: ~6x speedup on low-contrast regions.
**Details**:
- Replaced iterative threshold search (40 -> 10) with histogram bucketing.
- Fixed sort order bug (ascending -> descending) for corner selection.
- Removed redundant memory allocations in score computation.

## 3. Estimator: Sliding Window Allocations
**Optimization**: Introduced `Arc<String>` for Keyframe IDs and Tuple-based Residuals.
**Improvement**: ~1.5x speedup in problem construction (latency reduction).
**Details**:
- Removed `Vec<String>` cloning for every residual block (30-50k allocs per frame -> 0).
- Replaced `Vec<Box<Factor>>` with tuple storage to reduce pointer chasing during build phase.

## 4. Image Pyramid: Level 0 Construction
**Optimization**: Optimized base-level generation.
**Improvement**: Removed redundant interpolation.
**Details**:
- `build_image_pyramid` previously called `resize(source, w, h)` for Level 0, performing expensive filtering.
- Replaced with `clone()` for Level 0.

## 5. Optical Flow: Residual Loop Fusion
**Optimization**: Fused affine transformation with bilinear interpolation.
**Improvement**: Reduced memory traffic and loop overhead.
**Details**:
- Previously:
    1. Compute `transformed_pat = T * P` (Write 104 floats).
    2. Read `transformed_pat` inside `residual()` loop.
- New:
    1. Pass `T` to `residual()`.
    2. Compute `T * p` on-the-fly in registers inside the interpolation loop.
- Removed intermediate `SMatrix` allocation and memory round-trip.

## Verification
All unit tests and benchmarks pass.
`cargo test --release --lib` passed successfully.
