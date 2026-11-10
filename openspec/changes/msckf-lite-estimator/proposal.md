## Why

The current sliding window bundle adjustment estimator is computationally hypersensitive: it times out on `slides1` (requires ~720s vs 600s limit) and any modification to factor weights or parameters catastrophically breaks the two tightly-tuned corridor sequences. After 4 failed optimization attempts, incremental tuning has reached a ceiling. A fundamentally different algorithm—MSCKF-lite—is needed to achieve robustness in feature-poor segments without sacrificing real-time performance.

## What Changes

- **New**: `src/msckf/` module — complete error-state EKF estimator with cloned camera poses
- **New**: MSCKF pipeline binary `run_tum_msckf` for TUM-VI evaluation
- **New**: Explicit visual degradation state machine (NORMAL → DEGRADED → RECOVERY)
- **New**: Nullspace-based landmark elimination (features used as constraints, not state)
- **Kept**: Existing `src/estimator/` sliding window BA unchanged (parallel implementation)
- **Kept**: Existing feature tracker (`src/feature_tracker/`) reused by MSCKF frontend

## Capabilities

### New Capabilities

- `imu-propagation`: Error-state EKF IMU propagation with midpoint integration and 15×15 covariance update including clone cross-terms
- `camera-clone-management`: Clone augmentation when a new stereo frame arrives, marginalization of oldest clone when window is full (max 8 clones, 63-dim state)
- `msckf-update`: Multi-view feature track constraints via nullspace landmark elimination — forms H_x/H_f Jacobians, projects out H_f, runs EKF update
- `visual-degradation`: State machine that monitors track count, stereo inlier ratio, and reprojection error to switch between NORMAL and DEGRADED modes; IMU-only propagation during degraded segments
- `msckf-initialization`: Staged init — gyro bias from static segment, gravity alignment from accelerometer average, visual bootstrap from stereo geometry

### Modified Capabilities

## Impact

- New module `src/msckf/` (~10 files, ~2000 lines)
- New binary `src/bin/run_tum_msckf.rs`
- Reuses `src/feature_tracker/` (no changes)
- Reuses `src/optimization/imu_preintegration.rs` noise model (no changes)
- No changes to existing estimator — both run in parallel on same branch
- Dependency: `nalgebra` already present; no new crates needed
