# Performance Profiling Guide for RS-VIO

**Target**: Embedded real-time systems (30-60 FPS on ARM/x86)  
**Goal**: Identify and eliminate hotspots in the VIO pipeline

---

## Quick Start

```bash
# 1. Generate flamegraph to find hotspots
make flamegraph

# 2. Run benchmarks and establish baseline
make benchmark-baseline

# 3. After optimizations, compare
make benchmark-compare
```

---

## A. Reduce Logging Overhead

**Problem**: Logging per-feature/per-frame kills performance

**Solution**:
```rust
// ❌ BAD: Per-feature logging in hot path
for feat in features {
    log::debug!("Processing feature {}", feat.id);  // Expensive!
    // ... processing
}

// ✅ GOOD: Batch logging outside hot path
let feature_count = features.len();
// ... process all features
log::debug!("Processed {} features", feature_count);
```

**Check your code**:
```bash
# Find debug/trace logs in hot paths
grep -r "log::debug\|log::trace" src/feature_tracker/
grep -r "log::debug\|log::trace" src/optimization/
```

---

## B. Profile to Find True Hotspots

### Generate Flamegraph

```bash
# Install flamegraph tool
cargo install flamegraph

# Run on sample dataset
make flamegraph

# Open flamegraph.svg in browser to analyze
```

**What to look for**:
1. **Patch tracking cost**: Pyramid + NCC/SSD + interpolation
2. **BA linearization**: apex-solver solve time
3. **PnP + triangulation**: Geometric computation cost

### Using perf (Linux)

```bash
# Record performance data
make perf-record

# View interactive report
perf report

# Focus on specific functions
perf report --stdio | grep -E "feature_tracker|optimization"
```

### Memory Profiling with DHAT

```bash
# Profile allocations
make profile-allocations

# Run your binary to generate dhat-heap.json
./target/release/run_euroc config/euroc_vio.yaml /tmp/dataset/

# View in browser: https://nnethercote.github.io/dh_view/dh_view.html
```

---

## C. Kill Allocations and Copies in Hot Path

### Common Wins

#### 1. Preallocate Buffers

**Before**:
```rust
pub fn process_frame(&mut self, frame: &Frame) {
    let pyramid = build_image_pyramid(frame);  // Allocates Vec<GrayImage>
    // ... use pyramid
}
```

**After**:
```rust
pub struct FeatureTracker {
    pyramid_cache: Vec<GrayImage>,  // Preallocated
}

pub fn process_frame(&mut self, frame: &Frame) {
    reuse_pyramid(&mut self.pyramid_cache, frame);  // Reuse!
    // ... use pyramid
}
```

#### 2. Use Fixed-Capacity Containers

**Before**:
```rust
let features: Vec<Feature> = Vec::new();  // Unbounded growth
```

**After**:
```rust
use arrayvec::ArrayVec;

// Max 400 features, stack-allocated
let features: ArrayVec<Feature, 400> = ArrayVec::new();
```

**For sliding window**:
```rust
use smallvec::SmallVec;

// Up to 10 keyframes inline, heap if needed
type KeyframeBuffer = SmallVec<[Keyframe; 10]>;
```

#### 3. Avoid Matrix Clones

**Before**:
```rust
fn compute_jacobian(&self, pose: Matrix4x4) -> DMatrix<f64> {
    let T = pose.clone();  // Unnecessary clone!
    // ... compute
}
```

**After**:
```rust
fn compute_jacobian(&self, pose: &Matrix4x4) -> DMatrix<f64> {
    // Use reference, or create view
    let T = pose.fixed_view::<3, 3>(0, 0);
    // ... compute
}
```

### Find Allocation Hotspots

```bash
# Build with allocation tracking
RUSTFLAGS="-C force-frame-pointers=yes" cargo build --release

# Run with DHAT profiling
# Check dhat-heap.json for:
# - Total bytes allocated
# - Allocation frequency
# - Peak memory usage
```

---

## D. Make the Solver Cheaper (Biggest Lever)

### Current State (apex-solver with LM)

RS-VIO uses sliding window bundle adjustment with Levenberg-Marquardt.

**Embedded reality**: Bounded runtime is critical.

### Practical Knobs

#### 1. Window Size
```rust
// In estimator config
pub struct EstimatorConfig {
    pub sliding_window_size: usize,  // Start with 5-7, not 10-20
}
```

**Recommendation**: 
- Start: 5-7 keyframes
- Maximum: 10 keyframes
- Trade-off: Smaller window = faster solve, but less context

#### 2. Cap LM Iterations

**File**: `src/optimization/sliding_window.rs`

```rust
// Find optimization settings
let solver_options = SolverOptions {
    max_iterations: 5,  // Was: 20, now: 3-5
    cost_tolerance: 1e-4,  // Early stop
    parameter_tolerance: 1e-6,
    // ... other options
};
```

#### 3. Use Cheap Robust Loss

```rust
// ✅ GOOD: Huber loss (branch-light)
let loss = HuberLoss::new(1.0)?;

// ❌ AVOID: Complex losses in tight loops
// let loss = TukeyLoss::new(4.0)?;  // More branches
```

#### 4. Hard Ceiling on Solve Time

```rust
use std::time::{Duration, Instant};

pub fn optimize(&mut self) -> Result<bool> {
    let start = Instant::now();
    let max_duration = Duration::from_millis(50);  // 50ms budget
    
    // ... optimization loop
    
    if start.elapsed() > max_duration {
        log::warn!("Optimization timeout, using last iterate");
        return Ok(false);  // Accept last solution
    }
    
    // ... continue
}
```

### Measure Solver Performance

```bash
# Check optimization timing in logs
RUST_LOG=debug cargo run --release -- config/euroc_vio.yaml /tmp/dataset/ \
    | grep -i "optimization.*ms"

# Expected output:
# [DEBUG] Optimization successful. Initial cost: 123.4, final cost: 45.6 (12.3ms)
```

**Target**: <20ms per solve on modern hardware

---

## E. Make Tracking Cheaper While Preserving Robustness

### 1. Cap Features with Spatial Bucketing

**File**: `src/feature_tracker/feature_tracker.rs`

```rust
pub struct FeatureTrackerConfig {
    pub max_features: usize,  // 200-400, not 1000+
    pub grid_rows: usize,     // 6-8 for spatial distribution
    pub grid_cols: usize,     // 8-10
}
```

**Implementation**:
```rust
// Enforce per-bucket limits
let features_per_bucket = max_features / (grid_rows * grid_cols);
```

### 2. Reduce Pyramid Levels for High FPS

```rust
// For high FPS (60+) or small inter-frame motion:
pub const PYRAMID_LEVELS: usize = 3;  // Was: 4-5

// Trade-off: Fewer levels = faster, but less robust to large motion
```

### 3. Use IMU to Predict Optical Flow

**When IMU integration is active**:

```rust
// Predict feature location from IMU
let predicted_transform = predict_from_imu(&imu_data, dt);

// Initialize optical flow with prediction
let initial_guess = predicted_transform * prev_feature_pos;

// Fewer iterations needed!
track_point_with_initial_guess(initial_guess, ...);
```

**Benefit**: 2-3x fewer optical flow iterations

---

## Performance Targets

### Frame Processing Budget

| Component | Target (ms) | Max (ms) | Notes |
|-----------|-------------|----------|-------|
| Feature Detection | 5-8 | 10 | FAST corners |
| Optical Flow Tracking | 8-12 | 20 | Depends on feature count |
| Stereo Matching | 5-8 | 15 | Patch-based |
| Motion Tracking (PnP) | 3-5 | 10 | When window full |
| Bundle Adjustment | 10-15 | 25 | Sliding window |
| **Total Pipeline** | **30-50** | **80** | **Target: 60 FPS** |

### Memory Targets

| Component | Typical | Maximum | Notes |
|-----------|---------|---------|-------|
| Image Pyramids | 3-5 MB | 10 MB | Cache reuse |
| Feature Buffers | 50-100 KB | 200 KB | 200-400 features |
| Sliding Window | 100-200 KB | 500 KB | 5-10 keyframes |
| **Total Runtime** | **5-10 MB** | **20 MB** | Excluding images |

---

## Workflow: Find and Fix Bottlenecks

### Step 1: Profile
```bash
# Generate flamegraph
make flamegraph

# Identify top 3 functions by time:
# 1. feature_tracker::track_points (35%)
# 2. apex_solver::solve (28%)
# 3. triangulation::triangulate_stereo (12%)
```

### Step 2: Measure Baseline
```bash
# Save current performance
make benchmark-baseline

# Example output:
# feature_tracker_benchmark: 12.3ms ± 0.5ms
# optimization_benchmark: 18.7ms ± 1.2ms
```

### Step 3: Optimize
- Apply changes from sections C, D, E above
- Focus on the hottest paths from flamegraph
- Make one change at a time

### Step 4: Verify
```bash
# Compare against baseline
make benchmark-compare

# Check for regressions
make quality-quick

# Expected improvement:
# feature_tracker_benchmark: 12.3ms → 8.1ms (34% faster!)
```

### Step 5: Document
```bash
# Update CHANGELOG.md with performance improvements
git add .
git commit -m "perf: Reduce feature tracking time by 34%

- Preallocate pyramid buffers
- Use ArrayVec for feature storage
- Reduce pyramid levels from 5 to 3

Benchmark: 12.3ms → 8.1ms on EuRoC dataset"
```

---

## Common Pitfalls

### ❌ Over-optimization
- Don't optimize without profiling first
- 10% improvement in non-hotspot = <1% total gain

### ❌ Breaking Robustness
- Always run quality checks after optimization
- Verify accuracy on benchmark datasets
- Check edge cases (fast motion, low texture)

### ✅ Best Practices
- Profile → Measure → Optimize → Verify
- Keep baselines for comparison
- Document tradeoffs in code comments

---

## Tools Reference

```bash
# Install all profiling tools
cargo install flamegraph
cargo install cargo-criterion  # Better benchmarking
cargo install cargo-bloat      # Binary size analysis

# Optional (Linux)
sudo apt install linux-perf    # perf profiling
```

---

## See Also

- [BENCHMARKING.md](BENCHMARKING.md) - Benchmark procedures
- [PERFORMANCE.md](PERFORMANCE.md) - Performance analysis
- [QUALITY_IMPLEMENTATION_ROADMAP.md](QUALITY_IMPLEMENTATION_ROADMAP.md) - Quality goals

---

**Last Updated**: January 15, 2026  
**Status**: Ready for embedded optimization workflow
