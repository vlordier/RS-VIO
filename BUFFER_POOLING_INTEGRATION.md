# Buffer Pooling Integration Guide

## Overview

This document describes how to integrate the descriptor and RANSAC buffer pools into the VIO pipeline to eliminate hot-path allocations.

## Architecture

### Descriptor Pooling (Phase 1 - COMPLETED)

**Status**: ✅ Implemented and tested
- `DescriptorPool` trait in `src/optimization/loop_closure/descriptor_pool.rs`
- `OrbBinaryPool`: Pre-allocates [u8; 32] binary descriptor buffers
- `FloatDescriptorPool`: Pre-allocates Vec<Float> for LightGlue descriptors
- `HybridDescriptorPool`: Combines both pools

**Current Integration**:
- `OrbExtractor.extract_with_pool()`: Accepts optional binary pool (future use)
- `OrbMatcher`: Has `binary_pool` field and `new_with_pool()` constructor

**Next Steps for Full Integration**:
```rust
// In LoopClosureDetector
pub struct LoopClosureDetector {
    pool: Arc<HybridDescriptorPool>,
    // ... other fields
}

// In detect_loop_closure() iteration:
for candidate in candidates {
    // Use pool.binary() or pool.float() for reusable buffers
}
```

### RANSAC Buffer Pooling (Phase 2 - INFRASTRUCTURE READY)

**Status**: ✅ Buffers added to FrameWorkspace, RANSAC modules documented

**FrameWorkspace Extensions**:
```rust
pub struct FrameWorkspace {
    ransac_hypothesis_samples: Vec<usize>,   // Sampled point indices
    ransac_inlier_mask: Vec<bool>,          // Inlier flags per point
    ransac_residuals: Vec<Float>,           // Residual scores per point
}

// Accessors:
ws.ransac_hypothesis_samples_mut()
ws.ransac_inlier_mask_mut()
ws.ransac_residuals_mut()
```

**Allocation Savings**:
- Per-iteration allocations eliminated:
  - `sample_indices`: Vec<usize> → replaced by `ransac_hypothesis_samples`
  - `sampled`: Vec<usize> → replaced by buffer slice
  - `inliers`: Vec<usize> → replaced by `ransac_inlier_mask`
  - `residuals`: Vec<Float> → replaced by `ransac_residuals`
- Estimated savings: **10-20 KB per frame** (200 iterations × ~100 bytes/iteration)

## Integration Checklist

### 1. Loop Closure RANSAC Integration

**File**: `src/optimization/loop_closure/pnp_ransac.rs`

**Changes needed**:
```rust
// Add workspace-aware solve method
pub fn solve_with_workspace(
    &self,
    correspondences: Vec<Correspondence>,
    camera_intrinsics: &na::Matrix3<f64>,
    workspace: &mut crate::estimator::frame_workspace::FrameWorkspace,
) -> Result<PnPRansacResult> {
    // Reuse workspace buffers instead of allocating per iteration
    for iteration in 0..self.config.num_iterations {
        // Instead of: let sampled: Vec<usize> = ...
        // Use: workspace.ransac_hypothesis_samples_mut().clear();
        //      workspace.ransac_hypothesis_samples_mut().extend(...);
        
        // Instead of: let mut inliers = Vec::new();
        // Use: workspace.ransac_inlier_mask_mut().iter_mut().for_each(|x| *x = false);
        
        // Instead of: for ... { let error = ...;
        // Use: workspace.ransac_residuals_mut()[i] = error;
    }
}
```

### 2. Feature Tracker RANSAC Integration

**File**: `src/feature_tracker/ransac.rs`

**Changes needed**:
```rust
// Add workspace variants to RansacFundamental, RansacHomography, etc.
impl RansacFundamental {
    pub fn estimate_with_workspace(
        matches: &[FeatureMatch],
        threshold: f32,
        confidence: f32,
        workspace: &mut crate::estimator::frame_workspace::FrameWorkspace,
    ) -> RansacResult<FundamentalMatrix> {
        // Reuse workspace buffers in iteration loop
    }
}
```

### 3. Estimator Integration

**File**: `src/estimator/estimator.rs`

**Changes needed**:
```rust
impl Estimator {
    fn process_frame(&mut self, frame: &StereoFrame) -> Result<EstimatorOutput> {
        // Get workspace from self
        let workspace = &mut self.frame_workspace;
        
        // Pass workspace to RANSAC operations
        if let Ok(result) = PnPRansacSolver::new(config)
            .solve_with_workspace(&correspondences, &intrinsics, workspace) {
            // Use result
        }
        
        // Reset workspace after frame processing
        workspace.reset();
    }
}
```

### 4. Loop Closure Detector Integration

**File**: `src/optimization/loop_closure.rs`

**Changes needed**:
```rust
pub struct LoopClosureDetector {
    config: LoopClosureConfig,
    database: KeyframeDatabase,
    matcher: Box<dyn DescriptorMatcher>,
    verifier: Box<dyn GeometricVerifier>,
    descriptor_pool: Arc<HybridDescriptorPool>,  // Add this
    ransac_config: PnPRansacConfig,
}

impl LoopClosureDetector {
    pub fn with_pools(
        config: LoopClosureConfig,
        orb_capacity: usize,
        float_capacity: (usize, usize),
    ) -> Self {
        let pool = Arc::new(HybridDescriptorPool::new(
            Some(orb_capacity),
            Some(float_capacity),
        ));
        // Initialize with pools
    }
}
```

## Implementation Order

1. **Phase 1 (DONE)**: Descriptor pools infrastructure
   - ✅ Trait definition
   - ✅ OrbBinaryPool and FloatDescriptorPool
   - ✅ ORB extractor/matcher support

2. **Phase 2 (DONE)**: RANSAC buffer infrastructure
   - ✅ FrameWorkspace extensions
   - ✅ Documentation of allocation patterns

3. **Phase 3 (RECOMMENDED)**: PnP-RANSAC integration
   - Priority: HIGH (loop closure is optional module)
   - Risk: LOW (workspace already exists in estimator)
   - Effort: MEDIUM (2-3 hours)
   - Testing: Add workspace-aware tests

4. **Phase 4 (OPTIONAL)**: Feature tracker RANSAC integration
   - Priority: MEDIUM (affects every frame)
   - Risk: MEDIUM (RANSAC is critical path)
   - Effort: MEDIUM (2-3 hours)
   - Testing: Full pipeline test with allocation profiling

5. **Phase 5 (OPTIONAL)**: Loop closure detector pools
   - Priority: LOW (optional module)
   - Risk: LOW
   - Effort: SMALL (1-2 hours)
   - Testing: Loop closure-specific unit tests

## Testing Strategy

### 1. Backward Compatibility
- Keep original `solve()` methods intact
- Add new `solve_with_workspace()` variants
- Run full test suite with both variants

### 2. Allocation Profiling
```bash
# Before pooling
valgrind --tool=massif ./target/release/vio_benchmark
# Record peak memory and allocation counts

# After pooling
valgrind --tool=massif ./target/release/vio_benchmark
# Compare: expect 5-10% reduction in allocations
```

### 3. Latency Impact
```rust
// Add timing in hot path
let start = std::time::Instant::now();
// RANSAC call here
let elapsed = start.elapsed();
// Log timing distribution
```

### 4. Integration Tests
- `test_ransac_with_workspace`: Verify workspace-aware RANSAC produces same results
- `test_workspace_buffer_reuse`: Confirm buffers are actually reused
- `test_end_to_end_with_pooling`: Full VIO pipeline with all pools enabled

## Performance Expectations

### Memory Allocations
- **ORB descriptor extraction**: ~5% reduction (32-byte arrays are small)
- **ORB matching**: ~10-15% reduction (conversion buffers reused)
- **PnP-RANSAC**: ~20-30% reduction (large sample and inlier vector reuse)
- **Feature tracker RANSAC**: ~15-25% reduction (sample vectors reused)
- **Overall**: **10-20% allocation reduction** in hot path

### Latency
- **Per-frame latency**: -0.5-1.5 ms (200 iterations × 2-5 μs per iteration)
- **Peak memory**: -2-5 MB (preallocated vs. dynamic)
- **Jitter**: Reduced due to elimination of runtime allocations

### Cache Efficiency
- **Improved**: Workspace buffers stay in L3 cache
- **Reused**: Hot memory patterns repeated across frames
- **Predictable**: No fragmentation from per-iteration allocations

## Risks and Mitigation

| Risk | Mitigation |
|------|-----------|
| Workspace overflow (too few buffers) | Add assertions in accessors, size buffers with headroom |
| Buffer size mismatches | Document capacity requirements, test with varied inputs |
| Pointer lifetime issues | Use Rust's borrow checker (safe by default) |
| Feature creep in workspace | Keep workspace focused on hot-path buffers only |

## Future Enhancements

1. **Static Buffer Sizing**: Use compile-time constants for buffer capacities
2. **GPU Integration**: Move pooling to GPU device memory
3. **Thread-Local Pools**: Per-thread workspaces for parallel processing
4. **Jemalloc Integration**: Custom allocator with pool awareness
5. **Allocation Tracking**: Runtime monitoring of buffer utilization

## References

- **FrameWorkspace**: `src/estimator/frame_workspace.rs`
- **Descriptor Pools**: `src/optimization/loop_closure/descriptor_pool.rs`
- **PnP RANSAC**: `src/optimization/loop_closure/pnp_ransac.rs`
- **Feature Tracker RANSAC**: `src/feature_tracker/ransac.rs`
- **Estimator**: `src/estimator/estimator.rs`
