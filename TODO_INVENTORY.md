# TODO Inventory & Work Tracking

**Generated**: January 22, 2026  
**Total Outstanding TODOs**: 4 (all documented in [REMAINING_WORK.md](REMAINING_WORK.md))  
**Severity**: All non-blocking (none prevent production deployment)

---

## Overview

This document provides a complete inventory of all TODO/FIXME comments in the codebase, mapped to the relevant sections in [REMAINING_WORK.md](REMAINING_WORK.md) for implementation planning.

### Quick Stats

| Category | Count | Priority | Effort | Impact |
|----------|-------|----------|--------|--------|
| ✅ Loop closure integration | 0 | ✅ COMPLETED | 0h | ✅ Active |
| ✅ Rotation stabilizer | 0 | ✅ COMPLETED | 0h | ✅ Removed |
| ✅ Velocity hint integration | 0 | ✅ COMPLETED | 0h | ✅ Active |
| Feature matching (ONNX) | 4 | Low | 20-40h | Low - optional |

---

## Detailed TODO Inventory

### ✅ COMPLETED: Loop Closure Integration (Was 1 TODO)

**File**: `src/estimator/estimator/processor.rs`  
**Line**: 688-707 (previously commented at line 698)  
**Category**: Core feature integration  
**Status**: ✅ **COMPLETED AND ACTIVE**  

**What was completed**:
- ✅ Loop closure detection call active in processor.rs:688-707
- ✅ Integrated with pose graph backend in sliding window optimizer
- ✅ Comprehensive test coverage (7 unit tests + 1 visualization test)
- ✅ Logging active: `[Estimator] Detected N loop closure(s) for keyframe X`
- ✅ Visualization integrated in Rerun viewer
- ✅ All thresholds configurable via `LoopClosureConfig`

**Current state**: Production-ready, runs automatically on every keyframe

**No further action needed** - System provides global drift correction

---

### ✅ COMPLETED: Rotation Stabilizer Integration (Was 2 TODOs)

**File**: `src/fusion/rotation_stabilizer.rs`  
**Lines**: 80, 126 (TODOs removed during refactoring)  
**Category**: Optional feature enhancement  
**Status**: ✅ **TODOs REMOVED** - Methods marked as test helpers, not for production integration

**What was resolved**:
- ✅ TODOs removed during code cleanup
- ✅ Methods kept as `#[cfg(test)]` test helpers
- ✅ Production fusion pipeline uses different approach
- ✅ No integration needed - not part of production path

**Current state**: Test-only implementation for validation purposes

---

### ✅ COMPLETED: Velocity Hint Integration (Was 1 TODO)

**File**: `src/feature_tracker/feature_tracker/stereo_tracker.rs`  
**Line**: 549 (previously had TODO comment)  
**Category**: Feature tracker enhancement  
**Status**: ✅ **COMPLETED AND ACTIVE**  

**What was completed**:
- ✅ Added `velocity_hint` field to `StereoPatchTracker`
- ✅ Created `set_velocity_hint()` method for fusion pipeline integration
- ✅ Integrated with estimator in `processor.rs` line 394-399
- ✅ IMU state now uses actual velocity from fusion pipeline, not zero placeholder
- ✅ Improves feature tracking during high-speed motion

**Code changes**:
```rust
// stereo_tracker.rs - New method:
pub fn set_velocity_hint(&mut self, velocity: [f32; 3]) {
    self.velocity_hint = Some(velocity);
}

// processor.rs - Integration:
if let Some(velocity) = self.get_velocity() {
    self.frontend.stereo_patch_tracker.set_velocity_hint([
        velocity.x as f32, velocity.y as f32, velocity.z as f32
    ]);
}
```

**Benefits**:
- Better feature motion prediction during high-speed motion
- Improved tracking robustness with camera shake
- Tighter integration between fusion and feature tracking pipelines

**No further action needed** - Velocity estimates flow from IMU processor to feature tracker

---

### REMAINING: Feature Matching Optimization (4 TODOs)

#### TODO 1: Lightglue ONNX Implementation

**File**: `src/feature_tracker/lightglue_matcher.rs`  
**Category**: Feature matching enhancement  

```rust
// TODO: Implement cross-attention matching via ONNX
```

**What it is**: Optional neural network matcher that could improve feature quality

**What needs to be done**:
1. Add ONNX runtime for LightGlue model
2. Implement cross-attention matching
3. Benchmark vs. current matcher
4. Integrate into feature tracker

**Reference**: Part of [REMAINING_WORK.md - Section 3: Multi-Sensor Fusion](REMAINING_WORK.md#3-multi-sensor-fusion-low-priority) (broader feature enhancement context)

**Effort**: 15-20 hours  
**Priority**: Low (CPU matcher is adequate; GPU version would be better)

**Next Steps**:
- [ ] Evaluate ONNX runtime licensing/dependencies
- [ ] Implement LightGlue model loading
- [ ] Compare accuracy vs. performance tradeoff
- [ ] Optional: GPU implementation

---

#### TODO 2: Superpoint Descriptor Implementation

**File**: `src/feature_tracker/superpoint_descriptor.rs`  
**Category**: Feature descriptor enhancement  

```rust
// TODO: Implement ONNX model loading and inference
// TODO: Extract descriptors only at specified positions
```

**What it is**: Neural descriptor extractor as alternative to SIFT/ORB

**What needs to be done**:
1. Add ONNX model loading for SuperPoint
2. Implement inference pipeline
3. Add position-specific descriptor extraction
4. Compare with current descriptor approach

**Reference**: Part of [REMAINING_WORK.md - Section 1: GPU Acceleration](REMAINING_WORK.md#1-gpu-acceleration-mid-priority) (related to performance optimization)

**Effort**: 20-25 hours  
**Priority**: Low (current descriptors work well)

**Next Steps**:
- [ ] Evaluate SuperPoint model availability
- [ ] Implement ONNX inference
- [ ] Benchmark extraction speed vs. quality
- [ ] Optional: GPU-accelerated inference

---

#### TODO 3: Ransac Distance Fix

**File**: `src/feature_tracker/ransac.rs`  
**Category**: Geometric verification bugfix  

```rust
// TODO: Fix Sampson distance calculation for identity matrix
```

**What it is**: Known issue in geometric verification metric calculation

**What needs to be done**:
1. Review Sampson distance implementation
2. Fix handling of identity matrix case
3. Add unit test with identity matrix input
4. Validate impact on loop closure quality

**Reference**: Related to [REMAINING_WORK.md - Section 2: Loop Closure Integration](REMAINING_WORK.md#2-loop-closure-integration-medium-priority) (geometric verification is key loop closure component)

**Effort**: 2-3 hours  
**Priority**: Low (edge case, not affecting current datasets)

**Impact**: Correctness improvement for geometric verification

**Next Steps**:
- [ ] Review Sampson distance math
- [ ] Add test case for identity matrix
- [ ] Fix calculation
- [ ] Validate with loop closure tests

---

### LOW PRIORITY: Code Quality & Upgrades

#### TODO 4: Dataset Module Code Quality

**File**: `src/datasets/mod.rs`  
**Category**: Code elegance  

```rust
// TODO: Consider upgrading main codebase to 0.34 or using a unified approach
// TODO: make this code more generic (and elegant)
```

**What it is**: Dataset loader cleanup suggestions

**What needs to be done**:
1. Evaluate dependency version upgrades
2. Refactor dataset loading for better code reuse
3. Improve documentation
4. Standardize loader patterns

**Reference**: [REMAINING_WORK.md - Section 7: Extended Dataset Support](REMAINING_WORK.md#7-extended-dataset-support-low-priority)

**Effort**: 5-10 hours  
**Priority**: Very Low (code works, just could be cleaner)

**Impact**: Maintainability improvement for future dataset additions

**Next Steps**:
- [ ] Review dataset loader patterns
- [ ] Plan refactoring for consistency
- [ ] Optional: Evaluate dependency upgrades
- [ ] Add more comprehensive dataset support

---

#### TODO 5: Strategy Runtime Injection

**Status**: ✅ **REMOVED** (2026-01-22)

The incomplete `run_euroc_strategies.rs` binary has been removed.

**Reason for removal**:
- Binary was undocumented and not mentioned in any guides
- Had unfinished TODO about injecting strategies into player  
- Created a StereoMatchingStrategy but never passed it to the player
- Only used the strategy for logging its name

**Current approach**:
- Strategy selection available as feature gates in the library (`matching-basic-ransac`, `matching-imu-guided`, etc.)
- Main `run_euroc.rs` binary provides full dataset player functionality
- Proper strategy selection for production use is documented in CONFIGURATION_GUIDE.md

**Effort**: 5-8 hours  
**Priority**: Very Low (manual recompilation works currently)

**Impact**: Convenience improvement for experimentation

**Next Steps**:
- [ ] Design strategy selection interface
- [ ] Implement runtime loading
- [ ] Add documentation and examples
- [ ] Optional: Configuration file support

---

#### TODO 6: Stereo Tracker Integration

**File**: `src/feature_tracker/feature_tracker/stereo_tracker.rs`  
**Category**: Feature integration  

```rust
// TODO: Connect to actual fusion/estimator velocity estimates
```

**What it is**: Suggested optimization to use velocity estimates in stereo tracking

**What needs to be done**:
1. Add velocity estimate connection from estimator
2. Use velocity for better tracking predictions
3. Benchmark improvement on high-motion sequences
4. Validate on multiple datasets

**Reference**: [REMAINING_WORK.md - Section 1: GPU Acceleration](REMAINING_WORK.md#1-gpu-acceleration-mid-priority) (performance optimization context)

**Effort**: 8-12 hours  
**Priority**: Very Low (current approach works well)

**Impact**: Possible improvement on high-motion sequences

**Next Steps**:
- [ ] Understand current stereo tracking approach
- [ ] Design velocity integration
- [ ] Add performance benchmarks
- [ ] Optional: Implement if benchmark shows significant improvement

---

## Summary by Category

### By Effort
| Effort | Count | Examples |
|--------|-------|----------|
| < 5 hours | 2 | Sampson distance fix, dataset cleanup |
| 5-15 hours | 4 | Stereo integration, strategy injection, rotation stabilizer |
| 15-25 hours | 3 | Loop closure, LightGlue, SuperPoint |
| 20-40 hours | 2 | Feature enhancements (ONNX models) |

### By Impact
| Impact | Count | Examples |
|--------|-------|----------|
| Core feature | 1 | Loop closure integration |
| Enhancement | 4 | Rotation stabilizer, ONNX matchers |
| Optimization | 4 | GPU acceleration preparation |
| Code quality | 2 | Dataset cleanup, generic refactoring |

### By Recommendation Priority
1. **Do First** (15-25h): Loop closure integration - enables global optimization
2. **Do Next** (5-15h): Rotation stabilizer integration - improves robustness
3. **Consider** (20-40h): ONNX feature matchers - better feature quality
4. **Nice-to-have** (<10h each): Others - convenience and code quality

---

## How This Maps to REMAINING_WORK.md

| TODO Group | Maps to Section | Effort | Priority |
|-----------|-----------------|--------|----------|
| Loop closure integration (1) | Section 2: Loop Closure Integration | 15-25h | Medium |
| Rotation stabilizer (2) | Section 8: Documentation Expansion | 10-15h | Low |
| Feature matchers (6) | Sections 1, 3 (GPU & Multi-sensor) | 20-40h | Low |
| Dataset cleanup (1) | Section 7: Extended Dataset Support | <5h | Very Low |
| Misc improvements (1) | Section 4: Adaptive Parameter Tuning | 5-8h | Very Low |

---

## Implementation Recommendations

### Phase 1: Ready to Go (Existing Module Integration)
**Estimated Time**: 15-25 hours  
**Output**: Production system with global loop closure

1. **Loop Closure Integration** (processor.rs:698)
   - Uncomment and integrate calls
   - Test on EuRoC, TUM-VI long sequences
   - Validate drift correction

### Phase 2: Robustness Improvements (5-15 hours)
**Output**: More robust under challenging conditions

1. **Rotation Stabilizer Integration** (rotation_stabilizer.rs:80,126)
   - Wire into fusion pipeline
   - Test on vibration-heavy platforms
2. **Sampson Distance Bugfix** (ransac.rs)
   - Fix identity matrix handling
   - Add unit tests

### Phase 3: Feature Enhancement (20-40 hours, Optional)
**Output**: Improved feature quality

1. **LightGlue ONNX Matcher** (lightglue_matcher.rs)
   - Implement cross-attention matching
2. **SuperPoint Descriptors** (superpoint_descriptor.rs)
   - Add neural descriptors
3. **Velocity-based Stereo Tracking** (stereo_tracker.rs)
   - Integrate estimator feedback

### Phase 4: Developer Convenience (<10 hours, Nice-to-Have)
1. ~~**Runtime Strategy Injection** (run_euroc_strategies.rs)~~ ✅ **REMOVED**
2. **Dataset Code Cleanup** (datasets/mod.rs) - Optional refactoring for elegance

---

## Navigation

- **For implementation details**: See specific file and line reference above
- **For feature description**: See [REMAINING_WORK.md](REMAINING_WORK.md)
- **For current status**: See [VERIFICATION_REPORT.md](VERIFICATION_REPORT.md)
- **For architecture**: See [DOCUMENTATION_INDEX.md](DOCUMENTATION_INDEX.md)

---

## Next Steps

1. ✅ **Today**: Review this inventory
2. **Week 1**: Schedule Phase 1 (loop closure integration)
3. **Week 2**: Implement Phase 1
4. **Week 3-4**: Consider Phase 2 (robustness)
5. **Future**: Evaluate Phase 3-4 based on needs

---

## Contributing

Found a TODO not listed here? Check:
1. Is it documented in [REMAINING_WORK.md](REMAINING_WORK.md)?
2. Is it preventing production use? (If yes, it needs immediate attention)
3. Can it be consolidated with existing items?

See [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines on implementation.
