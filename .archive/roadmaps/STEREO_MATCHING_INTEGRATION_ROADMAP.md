# Integration Roadmap: Stereo Strategies with StereoTracker

## Current Architecture

```
Feature Detection
       ↓
   Optical Flow Tracking (monocular)
       ↓
   Stereo Matching  ← NEW: Select strategy here
       ↓
   RANSAC Validation (for geometric checks)
       ↓
   Subpixel Refinement
       ↓
   Features → Estimator
```

## Phase 1: Quick Integration (30 minutes)

### Step 1.1: Add Strategy Field to StereoPatchTracker

**File:** `src/feature_tracker/feature_tracker/stereo_tracker.rs`

**Current (line ~22-40):**
```rust
pub struct StereoPatchTracker<const N: u32> {
    last_keypoint_id: usize,
    // ... other fields ...
    essential_ransac: EssentialMatrixRansac,
    imu_rotation_hint: Option<[f32; 3]>,
    // ...
}
```

**Add:**
```rust
pub struct StereoPatchTracker<const N: u32> {
    last_keypoint_id: usize,
    // ... other fields ...
    essential_ransac: EssentialMatrixRansac,
    imu_rotation_hint: Option<[f32; 3]>,

    // NEW: Stereo matching strategy (pluggable)
    matching_strategy: Box<dyn StereoMatchingStrategy>,
    previous_match_results: Vec<StereoMatchResult>,  // For temporal consistency
    // ...
}
```

### Step 1.2: Update Constructor

**Current (line ~65):**
```rust
pub fn new(
    grid_size: u32,
    optical_flow_max_iterations: u32,
    optical_flow_convergence_threshold: f64,
) -> Self {
    // ...
    essential_ransac: EssentialMatrixRansac::new(RansacConfig::default()),
    // ...
}
```

**Update:**
```rust
pub fn new(
    grid_size: u32,
    optical_flow_max_iterations: u32,
    optical_flow_convergence_threshold: f64,
) -> Self {
    // Default strategy: BasicRANSAC
    let config = MatchingStrategyConfig::default();
    let matching_strategy = config.create_strategy()
        .unwrap_or_else(|_| {
            // Fallback to BasicRANSAC if config fails
            Box::new(BasicRANSACStrategy::new(BasicRANSACConfig::default()))
        });

    Self {
        // ... other fields ...
        matching_strategy,
        previous_match_results: Vec::new(),
        // ...
    }
}
```

### Step 1.3: Add Configuration Setter

```rust
impl<const LEVELS: u32> StereoPatchTracker<LEVELS> {
    /// Set the stereo matching strategy at runtime
    pub fn set_matching_strategy(
        &mut self,
        strategy: Box<dyn StereoMatchingStrategy>,
    ) {
        self.matching_strategy = strategy;
    }

    /// Load strategy from configuration file
    pub fn load_strategy_config(&mut self, path: &str) -> Result<(), String> {
        let config = MatchingStrategyConfig::from_file(path)?;
        self.matching_strategy = config.create_strategy()?;
        Ok(())
    }
}
```

**Effort:** 20 lines of code, 5 minutes

---

## Phase 2: Replace RANSAC Calls (45 minutes)

### Step 2.1: Find RANSAC Usage

**Current (line ~380 in process_frame):**
```rust
// Geometric filtering with essential-matrix RANSAC
let camera_matrix = self.make_camera_matrix(w0, h0);
let ransac_inliers = if refined_matches.len() >= 8 {
    let left_points: Vec<[f32; 2]> = refined_matches
        .iter()
        .map(|m| [m.left_pos.matrix().m13, m.left_pos.matrix().m23])
        .collect();
    let right_points: Vec<[f32; 2]> = refined_matches
        .iter()
        .map(|m| [m.right_pos.matrix().m13, m.right_pos.matrix().m23])
        .collect();

    let inliers =
        self.essential_ransac
            .find_inliers(&left_points, &right_points, &camera_matrix);

    if inliers.len() >= 4 {
        Some(inliers)
    } else {
        None
    }
} else {
    None
};
```

### Step 2.2: Replace with Strategy Dispatch

```rust
// Use selected stereo matching strategy
let camera_matrix = self.make_camera_matrix(w0, h0);

// Prepare IMU state for strategy
let imu_state = if let Some(omega) = self.imu_rotation_hint.take() {
    let (fx, fy, cx, cy) = self.imu_intrinsics_hint.take()
        .unwrap_or((camera_matrix.m11, camera_matrix.m22, camera_matrix.m13, camera_matrix.m23));
    Some(IMUState {
        velocity: self.estimate_velocity(),  // From fusion pipeline
        angular_velocity: na::Vector3::new(omega[0], omega[1], omega[2]),
        dt: self.frame_delta_time,
    })
} else {
    None
};

// Build previous depth map for temporal consistency
let previous_depth: Vec<(usize, f32)> = self.previous_match_results
    .iter()
    .map(|m| (m.id, m.disparity))
    .collect();

// Extract features for strategy
let features: Vec<(usize, f32, f32)> = refined_matches
    .iter()
    .map(|m| (m.id, m.left_pos.matrix().m13, m.left_pos.matrix().m23))
    .collect();

// Call strategy
let strategy_result = self.matching_strategy.match_stereo(
    greyscale_image0.as_raw(),
    greyscale_image1.as_raw(),
    w0 as usize,
    h0 as usize,
    &features,
    &camera_matrix,
    imu_state.as_ref(),
    if previous_depth.is_empty() { None } else { Some(&previous_depth) },
);

// Update telemetry
if should_log {
    debug_log!(
        "[FeatureTracker] Strategy '{}': {}/{} inliers in {:.3}ms",
        self.matching_strategy.name(),
        strategy_result.metrics.inliers_final,
        strategy_result.metrics.candidates_initial,
        strategy_result.metrics.time_total_ms
    );
}

let refined_matches = strategy_result.matches;
let ransac_inliers = if refined_matches.len() >= 4 {
    Some((0..refined_matches.len()).collect())
} else {
    None
};

// Store for next frame (temporal consistency)
self.previous_match_results = refined_matches.clone();
```

**Effort:** 50 lines of code, 25 minutes

---

## Phase 3: Add IMU Velocity Estimation (20 minutes)

### Step 3.1: Add Velocity Estimator

**New method in StereoPatchTracker:**
```rust
fn estimate_velocity(&self) -> na::Vector3<f32> {
    // For now, return zero. Should integrate with fusion pipeline
    // TODO: Get velocity from estimator/fusion state
    na::Vector3::zeros()
}

fn estimate_frame_delta_time(&self) -> f32 {
    // Assuming 30 FPS: 0.033 seconds
    // Should get actual frame timestamp from input
    1.0 / 30.0
}
```

### Step 3.2: Update Constructor to Use Velocity

```rust
self.frame_delta_time = self.estimate_frame_delta_time();
```

**Effort:** 10 lines, 5 minutes

---

## Phase 4: Testing Integration (30 minutes)

### Test 1: Load Default Strategy
```rust
#[test]
fn test_tracker_default_strategy() {
    let mut tracker = StereoPatchTracker::<4>::new(15, 30, 0.005);
    assert_eq!(tracker.matching_strategy.name(), "BasicRANSAC");
}
```

### Test 2: Switch Strategy at Runtime
```rust
#[test]
#[cfg(feature = "matching-imu-guided")]
fn test_switch_to_imu_guided() {
    let mut tracker = StereoPatchTracker::<4>::new(15, 30, 0.005);

    let imu_strategy = BasicRANSACConfig::default();
    tracker.set_matching_strategy(
        Box::new(IMUGuidedStrategy::new(8.0, imu_strategy))
    );

    assert_eq!(tracker.matching_strategy.name(), "IMUGuided");
}
```

### Test 3: Load from Configuration
```rust
#[test]
fn test_load_strategy_config() {
    let mut tracker = StereoPatchTracker::<4>::new(15, 30, 0.005);

    // Create test config
    let config_yaml = r#"
strategy: BasicRANSAC
params:
  max_iterations: 500
"#;

    // Would test with actual file
    // tracker.load_strategy_config("test_config.yaml").unwrap();
}
```

**Effort:** 30 lines of test code, 15 minutes

---

## Complete Integration Checklist

- [ ] Add `matching_strategy` field to `StereoPatchTracker` struct
- [ ] Add `previous_match_results` for temporal consistency
- [ ] Update constructor with default strategy
- [ ] Implement `set_matching_strategy()` method
- [ ] Implement `load_strategy_config()` method
- [ ] Add velocity estimation method
- [ ] Replace RANSAC call with strategy dispatch
- [ ] Add IMU state preparation
- [ ] Extract features for strategy input
- [ ] Call strategy.match_stereo() instead of RANSAC
- [ ] Store results for next frame
- [ ] Add debug logging for strategy metrics
- [ ] Update all unit tests
- [ ] Add integration tests
- [ ] Run on actual dataset (EuRoC)
- [ ] Measure fps impact
- [ ] Verify no regression in accuracy

---

## Expected Timeline

| Phase | Task | Time |
|-------|------|------|
| 1 | Add strategy field + constructor | 30 min |
| 2 | Replace RANSAC calls | 45 min |
| 3 | Add IMU integration | 20 min |
| 4 | Testing + debugging | 30 min |
| **Total** | **Complete integration** | **2 hours** |

---

## Risk Mitigation

### Risk 1: Strategy Dispatch Overhead
- **Risk:** Dynamic trait dispatch adds 0.1-0.5 µs per call
- **Mitigation:** Call is made once per frame, not in inner loop → negligible
- **Validation:** Benchmark before/after dispatch

### Risk 2: IMU State Not Available
- **Risk:** IMU velocity is None on first frame
- **Mitigation:** Graceful degradation in IMUGuided (defaults to 30px disparity)
- **Validation:** Test with and without IMU data

### Risk 3: Previous Depth Map Empty
- **Risk:** TemporalConsistency fails on first frame
- **Mitigation:** Builds depth map on-the-fly, passes as Some/None
- **Validation:** Test first frame processing

### Risk 4: Strategy Panics During Matching
- **Risk:** Segfault or panic in match_stereo
- **Mitigation:** All access is bounds-checked; no unwrap/expect
- **Validation:** Fuzzing with random feature positions

---

## Success Criteria

✅ Compilation succeeds with all features
✅ Unit tests pass (with/without each strategy)
✅ Integration tests pass (strategy dispatch works)
✅ No performance regression on BasicRANSAC
✅ IMUGuided strategy 10-20% faster than BasicRANSAC
✅ TemporalConsistency processes correctly (may have higher outlier rate)
✅ HybridOpticalFlow shows 20-40% speedup on active regions
✅ EuRoC dataset produces valid trajectory with all strategies
✅ Fps maintained at 68+ even with slowest strategy

---

## Configuration Example

**stereo_matching.yaml:**
```yaml
# Stereo matching strategy selection
strategy: IMUGuided

# Strategy-specific parameters
params:
  # For IMUGuided
  search_margin_px: 8.0
  max_iterations: 500
  inlier_threshold: 1.0

  # For TemporalConsistency
  depth_change_threshold: 0.2
  temporal_weight: 0.9

  # For HybridOpticalFlow
  flow_magnitude_threshold: 2.0

  # Shared RANSAC config
  min_inliers: 20
  confidence: 0.99
```

---

## Deployment Strategy

### Development/Testing
```bash
cargo build --features matching-basic-ransac,matching-imu-guided,matching-temporal,matching-hybrid-of
```

### Production Drone (with IMU)
```bash
cargo build --release --no-default-features --features matching-imu-guided
```

### Production Embedded (low-power)
```bash
cargo build --release --no-default-features --features matching-hybrid-of
```

### Production General Purpose
```bash
cargo build --release  # Uses BasicRANSAC by default
```

---

## Next Steps

1. **Implement Phase 1** (add strategy field): 30 min ✅
2. **Implement Phase 2** (replace RANSAC): 45 min ⏳
3. **Implement Phase 3** (IMU integration): 20 min ⏳
4. **Testing & Validation**: 2-4 hours ⏳
5. **Real dataset evaluation**: 2-4 hours ⏳

**Total time to full integration:** ~6-8 hours

---

## References

- **Strategy Implementations:** `src/feature_tracker/matching_strategy.rs`
- **Configuration System:** `src/feature_tracker/matching_strategy_config.rs`
- **Integration Target:** `src/feature_tracker/feature_tracker/stereo_tracker.rs`
- **Benchmarks:** `benches/strategy_comparison.rs`
- **Documentation:** `STEREO_MATCHING_STRATEGIES.md`

---

## Questions & Answers

**Q: What if IMU velocity is unavailable?**
A: IMUGuided gracefully degrades—uses default 30px disparity and full RANSAC

**Q: Can I switch strategies at runtime without recompilation?**
A: Yes, via `load_strategy_config()` method with YAML file

**Q: What's the memory overhead of having all 4 strategies compiled?**
A: ~2 MB of binary size; runtime memory is negligible (one strategy active at a time)

**Q: Will TemporalConsistency work on erratic drone motion?**
A: No—it assumes smooth motion. Falls back to BasicRANSAC for jerky movements

**Q: How do I measure which strategy is best for my drone?**
A: Run EuRoC dataset with each strategy, compare fps and trajectory error

---
