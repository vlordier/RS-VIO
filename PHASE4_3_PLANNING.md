# Phase 4.3 Integration Plan - Concurrent Algorithm Integration

**Status**: Planning & Scoping  
**Estimated Duration**: 23-25 hours  
**Target**: Move actual feature detection and optimization to async tasks  

---

## Overview

Phase 4.2 delivered the **concurrent infrastructure foundation**:
- ✅ Async channel-based pipeline architecture
- ✅ Reordering buffer for deterministic output
- ✅ Configurable worker pools
- ✅ Simulated work testing framework

Phase 4.3 will **integrate real VIO algorithms** into the pipeline:
- Feature detection as async task
- Bundle adjustment as async task
- Sliding window updates synchronized across stages
- Real-time performance validation on Jetson Nano

---

## Task Breakdown

### Task 4.3.1: Feature Detection Integration (~8-10 hours)

**Objective**: Move `FeatureDetectionCoordinator` to concurrent task

**Subtasks**:
1. **Extract feature detection API** (1-2 hours)
   - Create async function: `async fn detect_features_async(frame: &Frame) -> Result<(usize, Vec<Feature>)>`
   - Wrap synchronous `FeatureDetectionCoordinator::run()` 
   - Handle image buffers without blocking

2. **Integrate into pipeline** (2-3 hours)
   - Update `feature_detection_worker` to call real detector
   - Pass left/right image planes via Arc
   - Return feature counts and coordinates
   - Profile single-threaded latency (target: <20ms for 640×480)

3. **Buffer management** (2-3 hours)
   - Pre-allocate image pools to avoid allocation in hot path
   - Reuse feature vectors across frames
   - Measure allocation rate reduction

4. **Testing & validation** (1-2 hours)
   - Unit tests: feature detection on synthetic images
   - Integration test: pipeline with real features
   - Benchmark: detection throughput per worker thread

**Success Criteria**:
- ✅ Features detected per frame: 100-150 (VGA resolution)
- ✅ Detection latency: <20ms per frame (single worker)
- ✅ Throughput with 2 workers: >100 features/sec per worker

---

### Task 4.3.2: Optimization Integration (~15-20 hours)

**Objective**: Move bundle adjustment to concurrent task

**Subtasks**:
1. **Sliding window async refactor** (4-6 hours)
   - Extract sliding window state to shared Arc<Mutex<>>
   - Make `SlidingWindow::bundle_adjust()` async
   - Handle Hessian/gradient computation on background task
   - Synchronize results back to main thread

2. **Pose estimation sync** (3-4 hours)
   - Receive pose from optimization result
   - Update frame state: timestamp, T_w_b, velocity
   - Ensure causal ordering (pose applies to correct frame)

3. **Map point management** (4-5 hours)
   - Add/remove 3D points from optimization window
   - Update feature-to-point associations
   - Maintain covisibility graph

4. **Real-time constraints** (2-3 hours)
   - Timeout mechanism (33ms default for 30 Hz)
   - Graceful degradation if optimization takes too long
   - Fallback to simpler tracking if BA blocked

5. **Testing & validation** (2-3 hours)
   - Unit tests: pose update propagation
   - Integration test: full pipeline with real bundle adjustment
   - Benchmark: optimization latency vs frame count

**Success Criteria**:
- ✅ Bundle adjustment latency: <33ms for 20-50 frame window
- ✅ Poses refined (improvement in trajectory error): 5-10%
- ✅ No missing frame deadlines: 30 Hz sustained

---

### Task 4.3.3: Full Pipeline Integration (~3-5 hours)

**Objective**: Connect all stages, validate end-to-end behavior

**Subtasks**:
1. **Stage connection** (1-2 hours)
   - Verify frame ordering through full pipeline
   - Validate feature detection → tracking → pose flow
   - Test map point visibility from optimization

2. **Real-time validation** (1-2 hours)
   - Run on representative dataset (EuRoC, KITTI, TUM-VI)
   - Measure frame rate stability
   - Profile CPU and memory usage

3. **Jetson Nano deployment** (1 hour)
   - Cross-compile to aarch64
   - Validate performance on target hardware
   - Adjust worker counts if needed

---

### Task 4.3.4: Benchmarking & Tuning (~3-5 hours)

**Objective**: Measure and validate performance improvements

**Subtasks**:
1. **Performance measurement** (1-2 hours)
   - Throughput: Concurrent vs sequential (target: 2x)
   - Latency P99: <50ms (target: improvement from 95ms)
   - CPU utilization: 85% (target: improvement from 40%)

2. **Worker count tuning** (1 hour)
   - Test 1-4 feature workers
   - Test 1-3 optimization workers
   - Find optimal combination for Jetson Nano

3. **Trajectory quality validation** (1-2 hours)
   - Compare poses from concurrent vs sequential
   - Ensure numerical consistency
   - Validate on standard benchmarks (ATE, RPE)

---

## Implementation Strategy

### Phase 1: Feature Detection (Days 1-2)

```
Day 1:
  - Extract feature detection to async function
  - Create integration test with real features
  - Profile baseline performance

Day 2:
  - Integrate into pipeline workers
  - Add image buffer pooling
  - Run benchmarks
```

### Phase 2: Optimization (Days 3-5)

```
Day 3:
  - Refactor sliding window for async/await
  - Create async bundle adjustment wrapper
  - Unit tests for pose synchronization

Day 4:
  - Integrate with optimization worker
  - Implement timeout/fallback logic
  - Test map point updates

Day 5:
  - Integration testing
  - Performance validation
  - Jetson Nano deployment
```

### Phase 3: Validation (Days 6)

```
Day 6:
  - End-to-end testing on datasets
  - Benchmark vs sequential
  - Tune worker counts
  - Final validation
```

---

## Testing Strategy

### Unit Tests
```rust
#[test]
fn test_async_feature_detection() {
    // Detect features from synthetic image
    // Verify count and bounds
}

#[test]
fn test_async_bundle_adjustment() {
    // Run optimization on synthetic window
    // Verify pose refinement
}

#[test]
fn test_pose_ordering() {
    // Submit frames 0,1,2
    // Verify poses applied in order
    // Validate trajectory consistency
}
```

### Integration Tests
```rust
#[tokio::test]
async fn test_full_pipeline_with_real_features() {
    // Load real image pair
    // Submit through pipeline
    // Verify features + poses + map points
    // Check timing constraints
}
```

### Benchmarks
```rust
criterion_group! {
    "concurrent_algorithm_integration",
    bench_feature_detection,
    bench_bundle_adjustment,
    bench_full_pipeline,
}
```

---

## Risk Mitigation

### Risk 1: Feature Detection Thread Safety
- **Mitigation**: Wrap in Arc<Mutex> if shared state needed
- **Fallback**: Create separate detector instance per worker

### Risk 2: Optimization Window Consistency
- **Mitigation**: Use proper locking for sliding window updates
- **Fallback**: Process optimization sequentially, frame-parallel detection

### Risk 3: Real-time Deadline Violations
- **Mitigation**: Timeout mechanism + graceful degradation
- **Fallback**: Skip optimization for frames, keep tracking online

### Risk 4: Memory Growth with Concurrent Frames
- **Mitigation**: Limit pipeline depth, drop old frames
- **Fallback**: Use bounded queue with backpressure

---

## Success Criteria

### Performance
- ✅ Throughput: 60 Hz (vs 30 Hz sequential)
- ✅ P99 Latency: <50ms (vs 95ms spikes)
- ✅ CPU Utilization: 85% (vs 40%)

### Quality
- ✅ Trajectory ATE: <5% error (same as sequential)
- ✅ Map points: 200-300 per frame (same as sequential)
- ✅ All tests: 100% pass rate

### Deployment
- ✅ Jetson Nano: Runs at 30-60 Hz
- ✅ No memory leaks
- ✅ Graceful shutdown

---

## Dependencies & Assumptions

### Dependencies
- Phase 4.2 foundation (done ✅)
- Feature detection API stable (assumption)
- Bundle adjustment time <33ms (validated via Phase 3)

### Assumptions
- Detector is CPU-bound (not I/O bound)
- Optimization is parallelizable (sliding window)
- Poses update at frame rate (no back-projection)

---

## Rollback Plan

If Phase 4.3 encounters blockers:
1. Keep Phase 4.2 foundation deployed
2. Fall back to sequential processing
3. Debug issues in Phase 4.4+

Phase 4.2 is **fully backward compatible** and can run standalone.

---

## Next Steps

1. **Start Task 4.3.1**: Extract feature detection API
2. **Create async wrapper** around coordinator
3. **Integration test** with real images
4. **Benchmark** baseline feature detection latency
5. **Profile** on Jetson Nano

---

**Estimated**: 23-25 hours total  
**Target Benefit**: 2x throughput (60 Hz on Jetson Nano)  
**Status**: Ready to begin
