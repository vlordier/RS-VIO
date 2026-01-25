# RS-VIO SLAM Benchmarking Guide

## Overview

This guide covers how to run and tune the RS-VIO SLAM Phase 2C benchmarking suite on TUM-VI datasets with different speed/accuracy profiles. The benchmark compares VIO (sliding-window visual-inertial odometry) vs SLAM (with global optimization) on real datasets, measuring absolute trajectory error (ATE), relative pose error (RPE), and processing throughput.

## Quick Start

### Prerequisites
- TUM VI dataset (room1 recommended): `/path/to/tum_vi/room1/` (contains `mav0/` folder)
- Rust toolchain (1.70+)
- RS-VIO source in `/Users/vincent/Work/RS-VIO`

### Run Fast Benchmark (300 frames, ~3 min)
```bash
cd /Users/vincent/Work/RS-VIO
export RS_VIO_TUMVI_PATH="/path/to/tum_vi/room1"
export RS_VIO_CONFIG_PATH="config/tum_vi_fast.yaml"
export RS_VIO_MAX_FRAMES=300
export RUSTFLAGS="-C target-cpu=native"
export RUST_LOG=warn
cargo test --release --tests -- --nocapture test_slam_vs_vio_benchmarking
```

### Run Balanced Benchmark (100 frames, ~40 sec)
```bash
export RS_VIO_CONFIG_PATH="config/tum_vi_balanced.yaml"
export RS_VIO_MAX_FRAMES=100
cargo test --release --tests -- --nocapture test_slam_vs_vio_benchmarking
```

### Run Long-Run Benchmark (500 frames, ~4 min, most stable)
```bash
export RS_VIO_CONFIG_PATH="config/tum_vi_safe_longrun.yaml"
export RS_VIO_MAX_FRAMES=500
cargo test --release --tests -- --nocapture test_slam_vs_vio_benchmarking
```

---

## Configuration Profiles

### Profile 1: `tum_vi_fast.yaml` (Maximum Speed)
**Best for:** Quick throughput testing, CI/CD, speed profiling.

**Characteristics:**
- 10-frame sliding window (minimal state size)
- 5 PnP iterations, 5 BA iterations (minimal optimization)
- 16×4 feature grid (96 features max)
- Diagonal Hessian marginalization (fastest)
- No fallback depth seeding (skip expensive path)
- Processing: ~6 fps (release mode, native CPU)

**Trade-offs:**
- ATE: ~1.54 m (may drift faster due to sparse features)
- RPE translation: ~0.015 m (low error due to small, densely-sampled window)
- Best for: Throughput benchmarks, rapid iteration

**When to use:**
- Testing code changes that shouldn't affect speed dramatically
- Running hundreds of frames to isolate long-term drift
- CI/CD pipelines with time constraints

---

### Profile 2: `tum_vi_balanced.yaml` (Recommended Default)
**Best for:** General benchmarking, accuracy/speed balance, research.

**Characteristics:**
- 15-frame sliding window (moderate state)
- 8 PnP, 10 BA iterations (reasonable optimization)
- 20×6 feature grid (120 features max)
- Diagonal Hessian marginalization
- IMU prior enabled (soft constraint on latest pose)
- Processing: ~5.2 fps (release, native)

**Trade-offs:**
- ATE: ~1.55 m (small error, good pose spacing)
- RPE translation: ~0.017 m (slightly higher but more representative of real motion)
- Faster than accuracy profile; more accurate than fast
- Suitable for papers, comparisons, and public results

**When to use:**
- Default for new datasets or validation runs
- Generating publishable metrics
- Balancing computational cost with accuracy

---

### Profile 3: `tum_vi_accuracy.yaml` (Maximum Accuracy)
**Best for:** Ground truth validation, high-fidelity trajectory, loop closure study.

**Characteristics:**
- 22-frame sliding window (larger state)
- 10 PnP, 20 BA iterations (full optimization)
- 24×8 feature grid (192 features max)
- Gauss-Newton Hessian marginalization (most accurate but slower)
- Fallback depth enabled (fill gaps when triangulation fails)
- Strong IMU prior (0.8 pos, 1.5 rot)
- Processing: ~5.3 fps (release, native)

**Trade-offs:**
- ATE: ~1.57 m (slightly higher in early window, but denser features improve later)
- RPE translation: ~0.011 m (best RPE, more stable increments)
- Larger memory footprint, slower marginalization
- Rarely improves throughput (per-frame time ~180 ms vs ~170 ms fast)

**When to use:**
- Validating against GT with high fidelity
- Computing loop closure detection thresholds
- Academic papers requiring "best effort" accuracy

---

### Profile 4: `tum_vi_safe_longrun.yaml` (Ultra-Stable, Long Runs)
**Best for:** 500+ frame sequences, production robustness, numerical stability.

**Characteristics:**
- 8-frame sliding window (tiny, minimal marginalization cost)
- 3 PnP, 3 BA iterations (ultra-lightweight)
- 12×3 feature grid (36 features max)
- Diagonal Hessian, high damping (1e-5), FEJ disabled
- IMU prior disabled (reduce coupling, avoid ill-conditioning)
- Zero gradient computer (skip costly derivatives)
- Processing: **4.0 fps (very consistent, no slowdowns)**

**Trade-offs:**
- ATE: ~1.50 m (acceptable for long sequences)
- RPE translation: ~0.67 m (high RPE, reflects sparse features and minimal BA)
- Designed to prevent faer sparse solver crashes; proven stable to 500+ frames
- Smallest possible window to guarantee marginalization stability

**When to use:**
- Benchmarking full room traversals or long outdoor sequences
- Production deployments needing guaranteed stability
- Cases where 500+ frames must complete without crashes
- Testing loop closure on revisit without solver failures

---

## Key Parameters and Tuning

### Keyframe Management
```yaml
keyframe_management:
  keyframe_window_size: N           # Sliding window size (larger = more accuracy, more cost)
  translation_threshold: T_m        # Min translation since last KF to create new KF (larger = fewer KFs)
  rotation_threshold: R_rad         # Min rotation since last KF (radians)
  processing_timeout_ms: T_ms       # Max time per frame before warning
```

**Tuning:**
- **Increase `keyframe_window_size`** (15→25) → more accuracy, higher marginalization cost
- **Increase thresholds** (0.03→0.10) → fewer keyframes, lower cost, faster drift
- **Decrease thresholds** (0.03→0.01) → more KFs, denser sampling, higher cost

### Feature Detection
```yaml
feature_detection:
  grid_size: G                  # Grid columns (larger = more features across image)
  max_features_per_grid: F      # Features per cell
  optical_flow_max_iterations: I_of
  optical_flow_convergence_threshold: eps_of
```

**Tuning:**
- **Reduce `grid_size` + `max_features_per_grid`** → fewer features, faster feature tracking/matching
- **Increase `optical_flow_max_iterations`** → better feature matching, slower
- **Relax `optical_flow_convergence_threshold`** → faster convergence, possible miss

### Optimization
```yaml
optimization:
  pnp_max_iterations: I_pnp     # Iterations for pose solve
  bundle_adjustment_max_iterations: I_ba  # BA iterations per keyframe
  imu_prior_enable: bool        # Constrain latest pose to IMU prediction
  imu_prior_weight_pos: w_pos   # Position prior strength
```

**Tuning:**
- **Reduce BA/PnP iterations** → ~30% speedup per frame (at cost of optimization quality)
- **Disable `imu_prior_enable`** → marginally faster, avoid ill-conditioning in long runs
- **Increase `imu_prior_weight_*`** → trust IMU more, rely on vision less (risk of divergence)

### Marginalization
```yaml
marginalization:
  enabled: bool                       # Enable/disable sliding window marginalization
  max_keyframes: K                    # Max KFs before marginalization (should ≈ keyframe_window_size)
  hessian_approximator: "Strategy"    # "Diagonal", "GaussNewton", "LevenbergMarquardt", "Exact"
```

**Hessian Approximators (speed vs accuracy):**
| Strategy | Speed | Stability | Use Case |
|----------|-------|-----------|----------|
| Diagonal | Fastest | Excellent | Long runs, production |
| GaussNewton | Slower | Good | Balanced runs |
| LevenbergMarquardt | Slowest | Best | High-accuracy, GPU-enabled |
| Exact | Very Slow | Best | Research, small problems |

**Tuning marginalization:**
- **Decrease `max_keyframes`** → smaller window = faster marginalization = less accuracy
- **Increase `damping`** (1e-7 → 1e-5) → more stable but slower convergence
- **Switch to "Diagonal" Hessian** → 2-3x faster marginalization, acceptable accuracy loss

---

## Performance Expectations

### Release Build Optimization
Always run benchmarks with `--release` and `target-cpu=native`:

```bash
RUSTFLAGS="-C target-cpu=native -C llvm-args=-mcpu=native" \
cargo test --release --tests -- --nocapture test_slam_vs_vio_benchmarking
```

**Speed improvements (vs debug mode):**
- Debug: ~0.2 fps (extremely slow)
- Release (generic): ~3.5 fps
- Release + native CPU: ~4.0-6.0 fps (25-30% faster on x86)

### Frame Processing Breakdown (Profile: Balanced, 100 frames, release+native)
```
IO (image decode):        4-5 ms  (I/O bound)
IMU fetch:                0 ms    (cached)
Estimator (hot path):   170-180 ms (dominant, feature detection/matching/BA)
Total per frame:        174-185 ms
FPS:                    5.2-5.8
```

Estimator breakdown (estimated, from instrumentation):
- Feature detection: 20-30 ms
- Stereo matching: 30-50 ms
- Triangulation: 10-15 ms
- PnP solve: 10-15 ms
- Bundle adjustment: 60-80 ms (depends on window size and BA iterations)
- Marginalization: 5-10 ms (depends on window size and approximator)

---

## Trajectory Alignment & Evaluation

### Ground Truth Frame Mismatch
The benchmark automatically **aligns estimated trajectories to ground truth** using the first matched pose:

```
aligned_pose[t] = GT_frame[t0] × inv(VIO_frame[t0]) × VIO_pose[t]
```

This corrects for:
- Camera-body frame convention mismatches
- Coordinate system handedness differences
- Constant rotation offsets due to calibration errors

**Result:** Rotation errors drop from ~90° (orthogonal mismatch) to ~0-50° (alignment-corrected RPE rotation).

### Metrics Reported
After alignment:
- **ATE (Absolute Trajectory Error):** RMSE of position error at each timestamp
  - Typical range: 0.5-2.0 m for indoor TUM-VI
  - Lower is better; <1.5 m considered good
- **RPE Translation:** RMSE of relative pose error between consecutive frames
  - Typical range: 0.01-0.7 m
  - Lower indicates smooth, consistent motion tracking
- **RPE Rotation:** RMSE of rotation error (in radians, printed as degrees)
  - After alignment: 0-50° is expected; >70° indicates unresolved frame mismatch

---

## Troubleshooting

### Problem: Test hangs or extremely slow (>1000 s for 200 frames)
**Cause:** Marginalization becoming ill-conditioned; sliding window too large or damping too low.

**Fix:**
1. Reduce `keyframe_window_size` (15 → 10)
2. Reduce `marginalization.max_keyframes` to match
3. Increase `marginalization.damping` (1e-7 → 1e-6)
4. Switch to `hessian_approximator: "Diagonal"`
5. Use `tum_vi_safe_longrun.yaml` for long runs

### Problem: faer AMD panic at frame 200+
**Cause:** Sparse solver encountering singular matrix due to ill-conditioning.

**Fix:**
- Increase damping significantly (1e-5)
- Reduce window size (8-12 frames)
- Disable IMU prior
- Use safe long-run profile

### Problem: High RPE rotation (~90°)
**Cause:** Ground truth frame convention mismatch (not a runtime issue).

**Status:** Benchmark automatically aligns; after alignment, RPE rotation should drop to <50°.

If still high (>60° after alignment), check:
1. Extrinsics (`T_B_Cl`, `T_B_Cr`) match calibration reference
2. GT file format (`.tum` space-separated vs `.csv` comma-separated)
3. Timestamp synchronization between GT and images

---

## Configuration Reference

### Recommended Presets

**Speed-focused (for CI/CD):**
```bash
RS_VIO_CONFIG_PATH="config/tum_vi_fast.yaml" RS_VIO_MAX_FRAMES=300
```

**General purpose (recommended):**
```bash
RS_VIO_CONFIG_PATH="config/tum_vi_balanced.yaml" RS_VIO_MAX_FRAMES=150
```

**Accuracy-focused:**
```bash
RS_VIO_CONFIG_PATH="config/tum_vi_accuracy.yaml" RS_VIO_MAX_FRAMES=100
```

**Long-run robustness (500+ frames):**
```bash
RS_VIO_CONFIG_PATH="config/tum_vi_safe_longrun.yaml" RS_VIO_MAX_FRAMES=500
```

---

## Custom Configuration

To create your own profile:

1. Copy an existing config:
   ```bash
   cp config/tum_vi_balanced.yaml config/my_profile.yaml
   ```

2. Adjust parameters incrementally:
   - Start with balanced baseline
   - Change one section at a time
   - Run a 50-100 frame test to validate

3. Test with:
   ```bash
   RS_VIO_CONFIG_PATH="config/my_profile.yaml" RS_VIO_MAX_FRAMES=50 \
   cargo test --release --tests -- --nocapture test_slam_vs_vio_benchmarking
   ```

4. Monitor the `[Perf]` output for per-stage times:
   - `avg_est > 500 ms` → marginalization becoming slow; reduce window
   - `total fps < 1` → need faster config; reduce BA/PnP iterations
   - Consistent times → stable, safe to scale to longer runs

---

## Example: Creating a Speed Profile

Goal: Run 1000 frames at ~8 fps without crashing.

1. Start with `tum_vi_safe_longrun.yaml`
2. Increase features slightly:
   ```yaml
   feature_detection:
     grid_size: 14           # +2
     max_features_per_grid: 4  # +1
   ```
3. Slightly relax KF thresholds to add more constraints:
   ```yaml
   keyframe_management:
     keyframe_window_size: 10  # +2
   ```
4. Keep window tight:
   ```yaml
   marginalization:
     max_keyframes: 10
   ```
5. Test: `RS_VIO_MAX_FRAMES=200` → verify stable ~4-5 fps
6. Scale: `RS_VIO_MAX_FRAMES=1000` → should maintain throughput

---

## Next Steps

1. **Validate your setup:**
   ```bash
   RS_VIO_CONFIG_PATH="config/tum_vi_balanced.yaml" RS_VIO_MAX_FRAMES=100 \
   cargo test --release --tests -- --nocapture test_slam_vs_vio_benchmarking
   ```

2. **Compare profiles:**
   - Run fast, balanced, and safe_longrun on same dataset
   - Compare ATE/RPE/FPS outputs
   - Choose profile matching your use case

3. **Extend to your dataset:**
   - Set `RS_VIO_TUMVI_PATH` to your sequence
   - Adjust frame cap for full sequence
   - Monitor `[Perf]` output to watch per-stage costs

4. **Tune for production:**
   - Measure typical frame rate needed
   - Pick profile with 20% headroom (if target is 5 fps, use profile with 6+ fps)
   - Validate on longest sequence available

---

## Contact & Debugging

For crashes or anomalies:
- Check [COMPREHENSIVE_TEST_SESSION.md](COMPREHENSIVE_TEST_SESSION.md) for session logs
- Run with `RUST_BACKTRACE=1 cargo test ...` to get full panic traces
- Profile with `cargo test --release -- --nocapture 2>&1 | grep '\[Perf\]'` to monitor stages
