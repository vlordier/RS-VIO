## Context

RS-VIO currently uses a sliding window bundle adjustment estimator that jointly optimizes 10 keyframe poses and all active landmarks. Four separate tuning experiments (adaptive keyframes, lenient relocalization, hybrid IMU confidence, IMU fallback) all produced the same failure: corridor1/corridor2 broke completely while slides1 continued to time out. Root cause: the BA solver is numerically balanced at a specific operating point; any parameter change shifts the balance and causes divergence.

MSCKF (Multi-State Constraint Kalman Filter) eliminates this brittleness by separating propagation (IMU, continuous) from measurement updates (vision, selective). Landmarks never enter the state — they are triangulated per-track and eliminated via nullspace projection before the EKF update. This bounds state size to 15 + 6×N_clones regardless of map density.

Existing code to reuse:
- `src/feature_tracker/` — Lucas-Kanade tracker + stereo matching (unchanged)
- `nalgebra` — already the linear algebra backend
- `src/optimization/imu_preintegration.rs` — noise parameters already calibrated

## Goals / Non-Goals

**Goals:**
- slides1 completes within 600s with valid ATE output
- All 4 TUM-VI sequences produce valid trajectories
- Per-frame update time bounded at ≤15ms on development hardware
- Explicit DEGRADED mode: IMU-only propagation when feature count drops below threshold
- Parallel with existing BA estimator — no changes to current code paths

**Non-Goals:**
- Matching BA accuracy on corridor1/corridor2 (accepted trade-off: 0.059m → 0.09-0.12m)
- Loop closure or map reuse
- Online calibration of IMU-camera extrinsics (fixed from config)
- Rolling shutter modeling
- Real-time deployment on Pi-5 (target for later phase)

## Decisions

### D1: Error-state EKF over direct-state EKF
Use a 15-dim error-state (δθ, δv, δp, δb_g, δb_a) with SO(3) exponential map for rotation.
**Why:** Rotation is a manifold; additive error-state keeps covariance Euclidean while the nominal state lives on SO(3). Standard in OpenVINS, MSCKF 2.0, and ICE-BA. Alternative (direct quaternion EKF) requires careful normalization and loses sparsity.

### D2: Midpoint integration for IMU propagation
Evaluate IMU model at midpoint of each dt step rather than forward Euler.
**Why:** 2nd-order accuracy at negligible cost. Reduces bias accumulation during fast rotation (critical for slides1 which has aggressive camera motion).

### D3: Maximum 8 camera clones
State dimension capped at 15 + 8×6 = 63. Marginalize oldest clone when window is full.
**Why:** Each EKF update is O(n²) in state size. 63 dims keeps update under 5ms. Larger windows (12+ clones) add accuracy but push update time above 10ms. 8 is the OpenVINS default for embedded targets.

### D4: Stereo-aided triangulation, not mono
Use stereo disparity as initialization, then refine with multi-view DLT.
**Why:** Stereo gives immediate depth estimate regardless of baseline motion. Mono triangulation requires sufficient parallax — unreliable during slow or straight-line segments in slides1. Trade-off: requires stereo rectification or calibrated fisheye stereo model.

### D5: Native fisheye projection (not rectified)
Use the Kannala-Brandt (equidistant) fisheye model directly.
**Why:** TUM-VI cameras have ~190° FoV fisheye lenses. Rectification loses field of view and introduces interpolation artifacts at periphery. Fisheye model is analytic and its Jacobians are closed-form. More complex upfront but more accurate.

### D6: One EKF update per stereo frame (batch)
Collect all mature tracks (≥3 clone observations), build joint H and r, do one update.
**Why:** Sequential per-track updates produce identical result to batched but require more Cholesky solves. Batched update is simpler to gate globally (single chi-square test). Alternative: sequential updates with RANSAC-style rejection — deferred to Phase 3.

### D7: Chi-square innovation gating per track
Compute γ = rᵀ S⁻¹ r for each track before including in batch.
**Why:** Prevents bad triangulations or outlier tracks from corrupting state. Threshold from χ²(4) distribution at 95% confidence (stereo obs = 4-dim).

### D8: Degradation state machine (explicit mode switch)
Three states: INIT → NORMAL → DEGRADED, with hysteresis.
**Why:** Implicit degradation (relying on gating alone) silently accepts drift when few tracks pass gate. Explicit DEGRADED mode disables all visual updates and trusts IMU covariance growth. Recovery requires N consecutive good-quality frames. This is the key enabler for slides1.

## Risks / Trade-offs

- [Accuracy loss on tight corridors] corridor1/2 ATE may degrade from 0.059m to 0.09-0.12m → Accepted: correctness and completion rate prioritized over raw accuracy
- [Fisheye Jacobians] Kannala-Brandt projection Jacobian is non-trivial; bugs here will corrupt all updates → Mitigation: unit test projection + Jacobian numerically before integration
- [Triangulation failures] DLT fails with near-zero baseline → Mitigation: reject track if condition number of design matrix > threshold or depth < 0.1m
- [Covariance divergence] EKF covariance can grow unbounded during long degraded segments → Mitigation: clamp covariance diagonal to max values; reset if divergence detected
- [Clone marginalization correctness] Wrong Schur complement during marginalization corrupts cross-terms → Mitigation: validate with synthetic trajectory before real data

## Migration Plan

1. New module `src/msckf/` compiled behind default feature flag — no impact to existing binary
2. New binary `src/bin/run_tum_msckf.rs` added alongside existing `run_tum`
3. Evaluation script `scripts/eval_msckf_tum_vi.py` added — existing `eval_all_tum_vi_4x.py` unchanged
4. No rollback needed — parallel implementation; existing estimator untouched

## Open Questions

- Q1: Should the MSCKF frontend reuse `SlidingWindow::lm_name_cache` or maintain its own track store? (Leaning: own store — decoupled lifetime)
- Q2: For slides1 fisheye, do we need per-frame rectification or can we do native fisheye throughout? (Leaning: native fisheye — less preprocessing, more accurate)
- Q3: Clone creation policy — fixed interval vs. motion-triggered? (Leaning: motion-triggered with 0.08s minimum, 3° or 0.08m threshold)
