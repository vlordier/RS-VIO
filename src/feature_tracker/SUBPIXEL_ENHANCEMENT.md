# Sub-Pixel Disparity Enhancement Implementation Plan

## Status: PHASE 1 READY FOR INTEGRATION

### What's Been Implemented

1. **Sub-pixel disparity refiner** exists in `/src/vision/subpixel_disparity.rs`
   - ✅ Gauss-Newton optimization
   - ✅ Image pyramid (coarse-to-fine)
   - ✅ Huber robust loss
   - ✅ Uncertainty estimation
   - ✅ Photometric error computation

2. **New module created**: `/src/feature_tracker/subpixel_stereo.rs`
   - SubPixelStereoMatcher wrapper
   - Gradient energy computation
   - Photometric consistency validation

### What Needs Integration

The sub-pixel refiner is **NOT being used** in actual stereo matching. Current flow:

```rust
// CURRENT (integer-pixel only):
tmp_tracked_points1 = track_points(left, right, features)  // ← NO REFINEMENT

// NEEDED (sub-pixel):
for each feature in tracked_points:
    result = subpixel_refiner.refine_disparity(left, right, feature)
    if result.converged:
        update_with_subpixel_accuracy(result)
```

### Integration Points

1. **Feature struct enhancement** (line ~17-27 in feature_tracker.rs)
   - Add `disparity: Option<f32>`
   - Add `disparity_uncertainty: Option<f32>`

2. **StereoPatchTracker** (line ~160)
   - Add `subpixel_disparity_refiner: SubPixelDisparityRefiner`
   - Add `feature_quality: HashMap<usize, FeatureQuality>`

3. **Stereo matching loop** (line ~340-370)
   - After `track_points()` returns stereo matches
   - Call sub-pixel refinement for each match
   - Reject low-quality matches (high photometric error, low gradient)

### Performance Impact

**Current**: Integer-pixel disparity (~1-2px accuracy at 2m)
**After Phase 1**: Sub-pixel disparity (~0.1px accuracy)
**Gain**: **5-10× depth precision improvement**

### Next Steps

1. ✅ Create subpixel_stereo.rs module
2. ⬜ Export in mod.rs
3. ⬜ Integrate into feature_tracker process_frame()
4. ⬜ Add weighted BA residuals (Phase 2)
5. ⬜ RANSAC essential matrix (Phase 3)

### Test Validation

Run after integration:
```bash
cargo test --test vio_integration_complete
# Should see disparity refinement in test output
# Reprojection error should drop by ~40-60%
```
