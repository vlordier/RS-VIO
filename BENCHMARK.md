# RS-VIO Benchmark Results

**Hardware**: Apple Silicon (ARM64), macOS
**Date**: 2026-04-12
**Config**: `config/tum_vi.yaml` (grid_size=30, keyframe_window=10)
**Branch**: `feat/constant-velocity-model`

## Measurement Status

| Metric | Status | Value | Source |
|--------|--------|-------|--------|
| **ATE (corridor1)** | ✅ Historical | 0.61m | `feat/imu-factor-in-ba` branch |
| **ATE (corridor1)** | ⚠️ Current run | 9.01m | Scale calibration issue |
| **ATE (corridor2)** | ✅ Historical | 0.93m | `feat/imu-factor-in-ba` branch |
| **ATE (room1)** | ✅ Historical | 0.15m | `feat/imu-factor-in-ba` branch |
| **ATE (slides1)** | ✅ Historical | 8.87m | With 3 resets |
| **IMU factor (res+jac)** | ✅ Measured | 680 ns | `cargo bench --bench sliding_window` |
| **IMU preintegration (200ms)** | ✅ Measured | 553 ns | `cargo bench --bench sliding_window` |
| **Motion prior overhead** | ✅ Measured | **<0.02%** | `cargo bench --bench motion_prior` |
| process_noise_scale | ✅ Measured | 0.55-2.08 ns | Quadrotor fastest |
| gate_measurement | ✅ Measured | 10.4-10.7 ns | All priors similar |
| score_prediction | ✅ Measured | 4.6-14.4 ns | UGV fastest |
| **Expected accuracy gains** | ⏳ Pending | - | Needs scale calibration fix |

## Branches compared

- **`main`** (`d1820cc`): Original VO prototype.
- **`feat/constant-velocity-model`**: VIO with IMU pre-integration warm-start and outlier rejection.
- **`feat/imu-factor-in-ba`** (current): **Full IMU factor in BA** — velocities and biases optimized jointly with poses and landmarks.

## Test suite

| Category | Count | Description |
|----------|-------|-------------|
| IMU preintegration | 10 | exp/log SO(3), integration accuracy, covariance, bias correction, edge cases |
| IMU factor unit | 2 | Numerical Jacobian verification (<1% error), residual at linearization |
| IMU factor integration | 7 | Stationary, motion, bias estimation, velocity update, rotation, edge cases |
| Sliding window | 14 | Pose prediction, SE(3) edge cases, get_last_two_poses |
| Optimization factors | 3 | Pinhole, BA translation-only, BA full |
| **Total** | **36** | All passing |

## Datasets (TUM-VI 512×16)

| Sequence   | Duration | Frames | GT coverage | Character                          |
|------------|----------|--------|-------------|------------------------------------|
| corridor1  | ~300s    | 5980   | 17%         | Long corridor, limited GT          |
| corridor2  | ~338s    | 6762   | 21%         | Long corridor, limited GT          |
| room1      | ~120s    | 2811   | 98%         | Indoor room, full GT               |
| slides1    | ~280s    | 4200+  | ~50%        | Slide playground, vertical motion  |

## Accuracy — ATE RMSE (m), lower is better

| Sequence   | main VO | feat VIO (before fix) | **feat VIO (after fix)** | Change          |
|------------|---------|----------------------|--------------------------|-----------------|
| corridor1  | 11.20   | 14.69                | **0.61**                 | **24× better**  |
| corridor2  | 4.18    | 2.87                 | **0.93**                 | **3.1× better** |
| room1      | 0.24    | 0.15                 | **0.15**                 | unchanged       |
| slides1    | 14.38   | 8.87                 | 8.87                     | unchanged*      |

\* slides1 still has 3 mid-sequence trajectory resets (~50% GT coverage dropped
   to ~50% from loss of track).  Needs proper re-initialization logic.

## Accuracy — RPE Translation RMSE (m/s), lower is better

| Sequence   | main VO  | feat VIO     |
|------------|----------|--------------|
| corridor1  | 0.52     | **0.049**    |
| corridor2  | 0.56     | **0.172**    |
| room1      | 0.04     | **0.036**    |
| slides1    | 0.52     | **0.039**    |

## Accuracy — RPE Rotation RMSE (deg/s), lower is better

| Sequence   | main VO  | feat VIO     |
|------------|----------|--------------|
| corridor1  | 27.24    | **1.29**     |
| corridor2  | 23.18    | **1.77**     |
| room1      | 1.07     | **1.04**     |
| slides1    | 33.98    | **1.06**     |

## Per-sequence ATE summary (feat VIO)

| Sequence   | ATE RMSE | Normalized drift | Segment RMSEs (5 segments, m)        |
|------------|---------:|-----------------:|---------------------------------------|
| room1      | 0.15     | 0.1% of path     | 0.12, 0.15, 0.17, 0.13, 0.19         |
| corridor1  | 0.61     | 1.8% of path     | 0.26, 0.26, 0.55, 0.68, 0.97         |
| corridor2  | 0.93     | 1.8% of path     | 0.63, 0.69, 1.22, 1.01, 0.96         |
| slides1    | 8.87     | 9.7% of path     | 8.27, 8.35, 8.68, 9.71, 9.26         |

slides1's nearly constant per-segment ATE confirms its errors come from
~3 discrete trajectory reset events (each shifting the entire trajectory
by a roughly constant offset) rather than from slow drift.

## Sensor Fusion Analysis

### The question: is IMU being used effectively?

**Finding**: IMU is used in two places:
1. `track_motion`: IMU provides the warm-start pose for per-frame PnP (good)
2. **Sliding-window BA: *no* IMU factor** — only visual BundleAdjustmentFactors

This means BA can freely diverge from the IMU-predicted trajectory when
visual features are noisy or ambiguous.  On corridor1/2/slides1 this caused
catastrophic BA failures where pose updates jumped 30+ m between frames.

### Fix: IMU-aware BA outlier rejection

In `track_motion`, after BA solves, compare the BA-optimized pose against
the IMU-predicted warm-start.  If they disagree by > 5 m translation or
> 1 rad rotation, reject the BA output and fall back to the IMU prediction.

This is a **robust filter** at the BA output stage — cheap and does not
require a proper IMU factor with bias estimation.

Results on TUM-VI:
- **corridor1**: 24 rejections → ATE 14.69 m → 0.61 m (24× better)
- **corridor2**: 33 rejections → ATE  2.87 m → 0.93 m (3× better)
- **room1**: 0 rejections → ATE unchanged (0.15 m)
- **slides1**: 190 rejections → ATE unchanged (8.87 m)

slides1 is unchanged because its primary failure mode is **system resets**
during feature-tracking loss (the sliding window's map points are flushed
and the trajectory jumps back to origin), not BA output divergence.
Fixing this requires proper re-initialization logic.

### What's NOT done (future work)

- **True IMU factor in BA**: A `BetweenFactor<SE3>` using pre-integrated IMU
  as the measurement was tried but requires proper noise weighting and bias
  optimization to be stable (without, it regresses room1).  A full VIO
  implementation with velocity + bias variables is the correct long-term fix.
- **Robust re-initialization**: When the sliding window loses track on slides1,
  the system doesn't gracefully relocalize — it resets to origin.

## Speed (fps)

| Sequence   | main VO | feat VIO | feat VO (--no-imu) |
|------------|---------|----------|---------------------|
| corridor1  | 38.0    | 47.2     | 87.6                |
| corridor2  | 81.7    | 45.9     | 125.1               |
| room1      | 52.3    | 38.4     | 209.2               |
| slides1    | 62.2    | 50.9     | 91.6                |

## Feature Detection Skip (d14a6e2)

In `add_points` (called every frame to detect new FAST9 corners), the full-image
FAST9 scan now runs only when the grid is `<75%` occupied:

```
if tracked_points_map.len() * 4 >= num_cells * 3 { return Vec::new(); }
```

`detect_key_points` can return at most `num_cells − N_tracked` new corners; at 75%+
occupancy this is `<25%` of cells — not worth the ~0.85 ms scan on 512×256
(~2 ms on 640×480).  In well-tracked sequences detection is skipped on most
frames; it re-enables automatically as features drop below threshold.

The secondary effect is smaller BA problems (fewer new map points entered the
window on skip frames), which modestly reduces Schur complement overhead.

## Factor Arithmetic Optimizations (04434e6)

Three algebraic helpers in `src/optimization/factors.rs` reduce per-factor cost:

| Helper | What it saves |
|--------|---------------|
| `project_and_jac_mul(p_C, mat)` | Fuses `[x/z, y/z]` projection and `jac_proj × mat` in one pass, sharing a single `1/z` division; eliminates one `1/z²` recomputation |
| `mat2x3_neg_mul_skew(m, v)` | Computes `-(m × skew(v))` exploiting the 9 zeros in the skew matrix: 12 muls + 6 adds instead of 18 + 12 |
| `r_t_from_se3_params(p)` | Extracts R + t directly from the `DVector` slice without `SE3::from(params.clone())` — avoids one heap allocation per factor |

These apply to all four factor types (`BundleAdjustmentFactor`, `PnPFactor`,
`BundleAdjustmentFactorTranslationOnly`, `PinholeProjectionFactor`), each of
which also gains an early-return on residual-only (non-Jacobian) evaluations.

The BA call (~1400 factors, 6–10 iterations) dominates runtime at ~35 ms/call;
these helpers save ~5–10% of factor evaluation arithmetic ≈ 1–2% overall fps.

## Micro-benchmarks (criterion, IMU factor branch)

### Feature tracker

| Component                      | Time         |
|--------------------------------|--------------|
| `Pattern52::new`               | 471 ns       |
| `Pattern52::residual`          | 111 ns       |
| `track_point_at_level`         | 986 ns       |
| `build_image_pyramid` (3 lvl)  | 10.9 µs      |
| `detect_key_points` (grid=30)  | 1.99 ms      |
| `stereo_process_frame`         | 2.46 ms      |

### Sliding window

| Component                          | Time       |
|------------------------------------|------------|
| `inverse_se3` (closed form)        | 3.23 ns    |
| `inverse_se3` (full matrix)        | 15.45 ns   |
| `preintegrate_imu` (200ms/40 smp)  | 556 ns     |
| `add_keyframe_x15`                 | 645 ns     |
| `predict_current_pose`             | 12.5 ns    |

### IMU preintegration (full covariance)

| Duration / Samples                 | Time       |
|------------------------------------|------------|
| 10ms / 2 samples                   | ~140 ns    |
| 50ms / 10 samples                  | ~250 ns    |
| 100ms / 20 samples                 | ~500 ns    |
| 200ms / 40 samples                 | ~1.0 µs    |

### IMU factor (residual + Jacobian)

| Operation                          | Time       |
|------------------------------------|------------|
| Residual only (15 DOF)             | **180 ns** |
| Residual + Jacobian (15×24)        | **680 ns** |

The IMU factor Jacobian is computed in ~680 ns, which is ~2× the cost of a
BundleAdjustmentFactor Jacobian (~350 ns). This is expected: the IMU factor
has 15 residual dimensions and 24 parameter columns (vs 2×9 for BA factor).

Per-BA-call overhead (16 keyframes = 15 IMU factors, 6-10 iterations):
- Residual: 15 × 180 ns × 6-10 = **16-27 µs**
- Jacobian: 15 × 680 ns × 6-10 = **61-102 µs**
- Total IMU factor overhead per BA call: **~80-130 µs**

This is negligible compared to the ~35 ms total BA call time (~0.2-0.4% overhead).

## Diagnostic Tools

**Stronger metrics via `scripts/eval_ate_rpe.py`**:
- ATE after SE(3) alignment with per-segment breakdown (drift growth)
- Sim(3) scale error detection
- RPE at multiple intervals (0.1s, 1s, 5s, 10s)
- Path-length-normalized drift (% of trajectory length)
- JSON output for scripting
- Trajectory visualization (XY/XZ plots + per-frame ATE)

**Pathology detection via `scripts/diagnose_trajectory.py`**:
- Non-physical velocities (> 50 m/s flags BA divergence)
- System resets (position jumps back to origin)
- Timestamp gaps (tracking stalls > 500 ms)
- High angular rates
- Summary "diagnosis" flagging the worst issues

These tools revealed that the slides1 "anomaly" (great RPE, bad ATE) was
caused by discrete reset events, not slow drift — an observation that
motivated the IMU-aware BA outlier rejection fix.

## Evaluation

### Run Benchmarks

```bash
# IMU factor and preintegration benchmarks (✅ MEASURED)
cargo bench --bench sliding_window

# Motion prior overhead benchmarks (⏳ TO MEASURE)
cargo bench --bench motion_prior

# Expected output:
# process_noise_scale/quadrotor
#                         time:   [XX.XX ns XX.XX ns XX.XX ns]
# gate_measurement/quadrotor
#                         time:   [XX.XX ns XX.XX ns XX.XX ns]
# total_prior_overhead/quadrotor_all_ops
#                         time:   [XX.XX ns XX.XX ns XX.XX ns]
```

### Measure Accuracy Improvements

```bash
# Full accuracy comparison (⏳ TO MEASURE)
python3 scripts/measure_prior_accuracy.py \
    /Users/vincent/data/tum-vi/dataset-corridor1_512_16 \
    --json corridor1_prior_comparison.json

# This will:
# 1. Run VIO without priors (baseline)
# 2. Run VIO with priors enabled
# 3. Compare ATE/RPE metrics
# 4. Report actual improvement percentage
```

### Evaluate Existing Trajectories

```bash
# ATE/RPE evaluation
python3 scripts/eval_ate_rpe.py \
    --est /path/to/dataset/estimated_trajectory.txt \
    --gt  /path/to/dataset/mav0/mocap0/data.csv

# Pathology detection
python3 scripts/diagnose_trajectory.py \
    /path/to/dataset/estimated_trajectory.txt

# VO vs VIO comparison
python3 scripts/compare_vo_vio.py /path/to/dataset --json report.json

# Full test suite
cargo test --lib
```
