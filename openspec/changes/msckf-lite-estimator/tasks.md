## 1. Foundation — State & Module Structure

- [x] 1.1 Create `src/msckf/mod.rs` with module declarations and public re-exports
- [x] 1.2 Define `ImuState`, `CameraClone`, `FilterState` structs in `src/msckf/state.rs`
- [x] 1.3 Define `VisualMode` enum (INIT/NORMAL/DEGRADED) and `FrameQuality` struct in `src/msckf/degrade.rs`
- [x] 1.4 Add `msckf` module to `src/lib.rs` and verify `cargo check` passes

## 2. IMU Propagation

- [x] 2.1 Implement midpoint IMU integration for nominal state in `src/msckf/propagation.rs`
- [x] 2.2 Implement 15×15 IMU state transition Jacobian Φ_imu
- [x] 2.3 Implement process noise matrix Q and its discretization per dt step
- [x] 2.4 Implement full covariance propagation preserving IMU↔clone cross-terms
- [x] 2.5 Unit test: propagate static IMU (a=g, ω=0) for 1s, verify p/v/R unchanged

## 3. Camera Clone Management

- [x] 3.1 Implement clone creation trigger logic (time/rotation/translation thresholds) in `src/msckf/clone.rs`
- [x] 3.2 Implement covariance augmentation when a clone is added (derive clone uncertainty from IMU block)
- [x] 3.3 Implement oldest-clone marginalization via Schur complement
- [x] 3.4 Enforce max 8 clone limit with auto-marginalize before new clone insertion
- [x] 3.5 Unit test: add 10 clones sequentially, verify state dimension never exceeds 63

## 4. Feature Track Management

- [x] 4.1 Define `StereoObservation` and `FeatureTrack` structs in `src/msckf/frontend.rs`
- [x] 4.2 Implement `TrackManager` with add/update/expire track lifecycle
- [x] 4.3 Wire `TrackManager` to existing `src/feature_tracker/` output (reuse KLT tracks)
- [x] 4.4 Implement stereo observation pairing (left↔right epipolar match per clone)
- [x] 4.5 Implement track maturity check (≥3 clone observations required for update)

## 5. Triangulation

- [x] 5.1 Implement multi-view DLT triangulation in `src/msckf/triangulate.rs`
- [x] 5.2 Implement stereo-based depth initialization as DLT starting point
- [x] 5.3 Implement rejection criteria: negative depth, condition number > 100, reprojection RMSE > 2.0px
- [x] 5.4 Unit test: triangulate point at known depth from synthetic stereo observations, verify < 1cm error

## 6. MSCKF Residual & Nullspace

- [x] 6.1 Implement fisheye (Kannala-Brandt) projection and its Jacobian in `src/msckf/residual.rs`
- [x] 6.2 Numerically validate fisheye Jacobian against finite differences
- [x] 6.3 Implement H_x (residual Jacobian w.r.t. clone poses) for one track
- [x] 6.4 Implement H_f (residual Jacobian w.r.t. landmark position) for one track
- [x] 6.5 Implement nullspace projection in `src/msckf/nullspace.rs` (QR decomposition of H_f)
- [x] 6.6 Unit test: verify H_o = Qᵀ·H_x has zero product with H_f (nullspace condition)

## 7. EKF Update

- [x] 7.1 Implement chi-square innovation gate per track in `src/msckf/update.rs`
- [x] 7.2 Implement batch stacking of gated (H_o, r_o) across all tracks
- [x] 7.3 Implement Kalman gain computation and state correction application
- [x] 7.4 Implement covariance update (Joseph form for numerical stability)
- [x] 7.5 Unit test: update with synthetic track residuals on known state, verify correction converges

## 8. Visual Degradation State Machine

- [x] 8.1 Implement `FrameQuality` computation (track count, stereo inlier ratio, median reprojection error)
- [x] 8.2 Implement NORMAL→DEGRADED transition logic with thresholds
- [x] 8.3 Implement DEGRADED→NORMAL hysteresis (5 consecutive good frames)
- [x] 8.4 Implement conservative update path for DEGRADED mode (strict gate, ≥5 tracks)
- [x] 8.5 Integration test: inject 50 frames of zero features, verify mode switches to DEGRADED and back

## 9. Initialization

- [x] 9.1 Implement gyro bias estimation from static startup segment in `src/msckf/init.rs`
- [x] 9.2 Implement gravity alignment from accelerometer average (roll/pitch init)
- [x] 9.3 Implement visual bootstrap: initial velocity from first two stereo frames with ≥20 tracks
- [x] 9.4 Implement non-static fallback: zero bias with inflated initial covariance
- [x] 9.5 Implement INIT→NORMAL mode transition after all init stages complete

## 10. Pipeline Integration & Evaluation

- [x] 10.1 Create `src/bin/run_tum_msckf.rs` binary wiring IMU + stereo streams to `MsckfPipeline`
- [x] 10.2 Create `src/msckf/pipeline.rs` orchestrating the per-frame loop (propagate → clone → track → update)
- [x] 10.3 Create `scripts/eval_msckf_tum_vi.py` mirroring existing eval script with 600s timeout
- [x] 10.4 Run slides1 first — verify completes within 600s with valid ATE output
- [x] 10.5 Run full 4-sequence evaluation and record ATE for all sequences
- [x] 10.6 Tune clone count, track selection count, and gating thresholds based on eval results
