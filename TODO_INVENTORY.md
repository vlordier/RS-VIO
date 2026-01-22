# TODO Inventory & Work Tracking

**Generated**: January 22, 2026  
**Total Outstanding TODOs**: 11 (all documented in [REMAINING_WORK.md](REMAINING_WORK.md))  
**Severity**: All non-blocking (none prevent production deployment)

---

## Overview

This document provides a complete inventory of all TODO/FIXME comments in the codebase, mapped to the relevant sections in [REMAINING_WORK.md](REMAINING_WORK.md) for implementation planning.

### Quick Stats

| Category | Count | Priority | Effort | Impact |
|----------|-------|----------|--------|--------|
| Loop closure integration | 1 | Medium | 15-25h | High - global drift correction |
| Rotation stabilizer | 2 | Low | Included in loop closure | Medium - frame enhancement |
| Feature matching (ONNX) | 6 | Low | 20-40h | Low - optional optimization |
| Dataset support | 1 | Low | <5h | Low - code quality |
| Misc upgrades | 1 | Low | TBD | Low - future-proofing |

---

## Detailed TODO Inventory

### CRITICAL: Loop Closure Integration (1 TODO)

**File**: `src/estimator/estimator/processor.rs`  
**Line**: 698  
**Category**: Core feature integration  
**Status**: Implementation complete, integration pending  

```rust
// TODO: Loop closure detection not implemented in current refactor
// if let Ok(constraints) =
//     self.loop_closure_detector
//         .detect_loop_closure(kf_id, descriptor, &mut workspace)
// {
//     ...
// }
```

**What it is**: Commented-out code that should be activated to enable loop closure detection

**What needs to be done**:
1. Uncomment the loop closure detection call
2. Integrate with pose graph backend in sliding window optimizer
3. Test threshold tuning for deployment
4. Validate drift correction on long sequences

**Reference**: [REMAINING_WORK.md - Section 2: Loop Closure Integration](REMAINING_WORK.md#2-loop-closure-integration-medium-priority)

**Effort**: 15-25 hours  
**Priority**: Medium (enables global optimization, not required for baseline operation)

**Next Steps**:
- [ ] Review commented code at processor.rs:698
- [ ] Check pose graph backend implementation status
- [ ] Create integration test with long sequences
- [ ] Tune loop closure detection threshold

---

### HIGH: Rotation Stabilizer Integration (2 TODOs)

**File**: `src/fusion/rotation_stabilizer.rs`  
**Lines**: 80, 126  
**Category**: Optional feature enhancement  
**Status**: Implementation complete, production integration pending  

```rust
// Line 80:
/// TODO: Integrate this into fuse() method for production use

// Line 126:
/// TODO: Integrate this into fuse() method for production use
```

**What it is**: Two methods in rotation stabilizer that are fully implemented but not wired into the main fusion pipeline

**What needs to be done**:
1. Integrate rotation stabilizer methods into `fuse()` method
2. Add parameter tuning for frame accumulation
3. Test on high-vibration platforms (drones, mobile robots)
4. Validate performance impact vs. quality gain

**Reference**: Relates to [REMAINING_WORK.md - Section 8: Documentation Expansion](REMAINING_WORK.md#8-documentation-expansion-medium-priority) (better documentation of fusion strategies)

**Effort**: 10-15 hours (mostly integration testing)  
**Priority**: Low (enhancement, not core to baseline)

**Current State**: Rotation stabilizer reduces vibration noise through frame accumulation and gyro-based warping

**Next Steps**:
- [ ] Review rotation stabilizer implementation completeness
- [ ] Design integration into main fuse() pipeline
- [ ] Add unit tests for warping accuracy
- [ ] Test on actual hardware with vibration

---

### MEDIUM: Feature Matching Optimization (6 TODOs)

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
