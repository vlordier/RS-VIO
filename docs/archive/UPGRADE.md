What modern robust VIO/SLAM systems actually do

They combine, in order:

- Quality-ranked matches
- PROSAC sampling
- MAGSAC++ (robust scoring)
- M-estimator refinement
- BA / PGO

Key takeaway: the pipeline matters more than any single algorithm.

Default recommendation for embedded drone VIO/SLAM

- PROSAC + MAGSAC++ for geometry estimation
- SE3 refinement with Huber loss
- Use wide thresholds (avoid aggressive rejection) but keep strict geometric verification
- If CPU is very limited: PROSAC + well-tuned RANSAC, then Huber refinement


Here is an engineering-grade, step-by-step upgrade plan to evolve a basic VIO into a robust embedded VI-SLAM. Work top-down: each phase has a goal, concrete tasks, and exit checks. Stop at any phase once acceptance is met and the system stays real-time. This version is optimized for execution by a coding LLM agent: each task includes implementation notes, suggested defaults, test hints, and required artifacts.

---

# VIO -> VI-SLAM Upgrade Plan (Embedded Drone)

**How to use**
- Execute phases in order; do not skip prerequisites.
- Each task lists Do / Verify / Artifacts. Ship when all exit checks pass on representative flights.
- Keep control loop isolated: apply SLAM corrections only to the global map, never directly to the flight controller.

**Guiding principles**
- Never destabilize real-time control.
- Prefer estimator-aware mitigation over blind filtering.
- Add global consistency after local robustness.
- Every change must be measurable, toggleable, and revertible.

---

## Phase 0 — Baseline hardening (mandatory)

Goal: trust sensors and timing before adding complexity.

### 0.1 Calibration and sync
- Do: verify intrinsics, stereo extrinsics, camera <-> IMU extrinsics, and hardware sync; re-run rectification QA.
- Verify: straight lines stay straight; gravity alignment unbiased; short-trajectory scale stable.
- Test hints: checkerboard sweep with varying distances; IMU vs gravity alignment on static stand; record stereo bag (ROS bag/flight log) and run offline epipolar error histograms.
- Implementation notes: expect YAML or TOML cal files; add CLI `tools/calib_check --bag <bag> --output calib_report.json`; enforce timestamp monotonicity in sensor ingest.
- Defaults: stereo rms < 0.3 px; extrinsics reproj < 0.5 px; sync offset < 2 ms.
- Artifacts: calibration report, rectified sample images, sync offset log.

---

## Phase 1 — Vision front-end robustness

Goal: cleaner tracks and depths to feed all downstream estimators.

### 1.1 Sub-pixel feature tracking
- Do: enable KLT/LK sub-pixel refinement; track left(t) -> left(t+1) and left(t) -> right(t); drop tracks with high photometric or geometric error.
- Verify: lower reprojection error variance; longer track lifetimes; reduced pose jitter.
- Test hints: replay a handheld indoor bag (ROS bag/flight log) with motion blur; compare track length CDF and per-frame reprojection RMS before/after.
- Implementation notes: add a `tracker::Params { window=7, levels=4, max_err_px=1.5, min_eig=1e-4, max_iters=30, eps=1e-3 }`; run pyramidal LK; use forward-backward check.
- Defaults: accept tracks with photometric error < 30 (normalized); geometric epipolar error < 1.5 px.
- Metrics: track length CDF, per-frame inlier ratio, reprojection RMS.
- Artifacts: tracking metrics timeseries, reprojection error histogram, failure cases log.

### 1.2 Sub-pixel stereo refinement
- Do: after initial correspondence, refine disparity with patch alignment along the epipolar line (Gauss–Newton or quadratic peak fit); bound updates.
- Verify: tighter depth distribution; better far-point consistency.
- Test hints: static scene at multiple ranges; compute per-depth-bin noise; check far-point consistency on long hallway.
- Implementation notes: 1D search along epipolar with window=5-7 px; clamp sub-pixel update to +/-0.5 px; propagate refined disparity to depth covariance.
- Defaults: photometric threshold 25-35; max iterations 10; damping 1e-3.
- Artifacts: disparity refinement before/after stats, depth noise plot.

---

## Phase 2 — Modern robust geometry (replace plain RANSAC)

Goal: higher inlier quality and resilience to blur/vibration.

### 2.1 PROSAC + MAGSAC++
- Do: rank matches by quality; run PROSAC sampling with MAGSAC++ scoring; keep fallback RANSAC; apply to stereo, PnP, loop verification.
- Verify: higher inlier ratio; fewer false geometries; pose stability under induced blur.
- Test hints: run synthetic blur/noise injection on a recorded bag (ROS bag/flight log); log inlier ratios vs baseline RANSAC; evaluate pose drift on looped trajectory.
- Implementation notes: expose sampler interface `SamplePolicy::Prosac { ranked_matches }`; for PnP use P3P + PROSAC; use termination on score plateau; keep classic RANSAC path under feature-poor conditions.
- Defaults: max iterations 2000; confidence 0.995; MAGSAC++ sigma_start 2.0 px; min inliers stereo 25, PnP 12.
- Artifacts: inlier curves, pose drift comparison, fallback trigger counts.

### 2.2 Robust refinement everywhere
- Do: use Huber or Cauchy losses in pose refinement, local BA, and IMU–vision residuals.
- Verify: fewer estimator snaps; graceful degradation under injected noise.
- Test hints: add synthetic outliers (occasional bad features) in replay; measure pose jump frequency and residual distributions.
- Implementation notes: parameterize loss per residual type; start with Huber delta 1.0 px (vision) and 2 deg/s, 0.5 m/s^2 (IMU); expose runtime switch `--loss {huber,cauchy}`.
- Defaults: Huber delta 1.0 px (vision), 2 deg/s (gyro), 0.5 m/s^2 (accel); Cauchy scale 1.5 px if enabled.
- Artifacts: loss configuration, robustness A/B logs.

---

## Phase 3 — IMU vibration handling (estimator-safe)

Goal: mitigate vibration without harming timing.

### 3.1 Vibration metrics
- Do: compute per-window band-limited RMS (gyro + accel) and optional short FFT peak energy.
- Verify: metrics correlate with throttle/RPM/attitude changes.
- Test hints: bench test with motor spin-ups at different throttles; plot RMS/FFT peaks vs ESC telemetry or manual tachometer.
- Implementation notes: window 100-200 ms; band-limit gyro 20-200 Hz, accel 20-120 Hz; compute peak freq and magnitude from short FFT (e.g., 256-point, Hann window).
- Defaults: flag high vibration when gyro RMS > 0.1 rad/s or accel RMS > 0.4 m/s^2.
- Artifacts: vibration vs throttle plots, threshold proposal.

### 3.2 Adaptive covariance (primary)
- Do: inflate IMU covariance dynamically when vibration is high; keep signal timing unchanged.
- Verify: reduced divergence during high throttle; improved pose consistency.
- Test hints: fly aggressive throttle steps; count estimator resets/divergences vs baseline; inspect covariance scaling traces.
- Implementation notes: scale Q by factor s = clamp(1 + k*(rms/threshold - 1), 1, s_max); apply separately for gyro/accel; no filtering delay on scale.
- Defaults: k=1.5, s_max=5.0; threshold from 3.1.
- Artifacts: covariance scale log, divergence incidents before/after.

### 3.3 RPM-tracked notch (optional)
- Do: if ESC telemetry (bdshot/CAN) exists, notch motor fundamental + first harmonic with low order; pair with covariance adaptation. If no RPM, use wide notch guided by IMU FFT peaks.
- Verify: reduced high-frequency IMU noise without estimator lag.
- Test hints: bench spin with varying RPM; compare PSD before/after notch; measure estimator latency/phase using step attitude inputs.
- Implementation notes: biquad notch, Q=10-20, order 2; update center freq each telemetry packet; guard against freq jitter with low-pass on RPM.
- Defaults: latency budget < 0.2 ms per axis; fallback wide notch at dominant peak if no RPM.
- Artifacts: notch frequencies log, latency measurement, residual noise plot.

---

## Phase 4 — Mapping and keyframes (VIO -> SLAM transition)

Goal: maintain a stable local map while bounding resources.

### 4.1 Keyframe system
- Do: promote frames by parallax/motion/feature-drop; store pose, features, descriptors, IMU link; enforce bounds on keyframe growth.
- Verify: stable local map size; no real-time regressions.
- Test hints: long indoor loop replay; track keyframe count vs distance; profile CPU to ensure no deadline misses.
- Implementation notes: promotion rules (any): parallax > 1.5 deg or > 0.3 m; feature count drop > 30%; elapsed time > 0.7 s. Cap active keyframes to N=5-7 in sliding window, archive older with marginalization.
- Defaults: parallax threshold 1.5 deg or 0.3 m; feature drop 30%; time 0.7 s; active keyframe cap 5-7.
- Metrics: keyframe count vs distance, promotion rate, CPU per frame.
- Artifacts: keyframe promotion policy, map size traces.

### 4.2 Persistent landmarks
- Do: promote tracks to map points; keep observation lists; cull by parallax and reprojection error.
- Verify: long-lived landmarks; reduced map churn.
- Test hints: replay with stop-and-go motion; inspect landmark lifetime histogram and churn rate vs baseline.
- Implementation notes: promote when seen in >=3 keyframes with baseline > 0.2 m; store inverse depth + covariance; cull if reprojection > 3 px or viewing angle < 1 deg.
- Defaults: min observations 3; min baseline 0.2 m; cull threshold 3 px or 1 deg viewing angle.
- Metrics: landmark lifetime CDF, observations per landmark, churn rate.
- Artifacts: landmark lifetime histogram, culling stats.

---

## Phase 5 — Loop closure (CPU-only, safe)

Goal: add global consistency without risking flight control.

### 5.1 Place recognition
- Do: compute BoW or equivalent global descriptor per keyframe; maintain DB with temporal exclusion.
- Verify: high recall with low false positives.
- Test hints: offline loop dataset with known repeats; sweep DB parameters; plot precision-recall.
- Implementation notes: ORB/BRIEF descriptors -> DBOW2/FBow; query top K=30; temporal exclusion 30 frames; store descriptor vectors on disk.
- Defaults: top K=30 candidates; temporal exclusion 30 frames; min BoW score 0.01.
- Metrics: recall, precision, query time.
- Artifacts: PR curves, false-positive log.

### 5.2 Geometric loop verification
- Do: descriptor matching; PROSAC + MAGSAC++; minimum inlier and consistency gates.
- Verify: zero catastrophic false loops.
- Test hints: run on place-recognition candidates; inject synthetic mismatches to ensure rejection; monitor inlier histograms.
- Implementation notes: use PnP or 5-point essential depending on modality; require min inliers 25 and reprojection RMS < 2.5 px; add temporal/gps prior if available.
- Defaults: min inliers 25; max reprojection RMS 2.5 px; max geometric error 0.05 m.
- Metrics: acceptance rate, inlier count, reprojection error.
- Artifacts: accepted/rejected loop stats, inlier distributions.

### 5.3 Pose graph optimization
- Do: build pose graph (keyframe nodes, odometry + loop edges); optimize asynchronously; apply corrections only to the global frame.
- Verify: map drift reduced; no disturbances to local VIO used by controller.
- Test hints: replay long loop; compare absolute drift before/after PGO; observe controller-facing local pose remains smooth.
- Implementation notes: g2o/ceres pose graph; edge robust loss (Huber delta 1.0); optimization at 0.5-1.0 Hz; publish correction as slowly varying transform T_map_local.
- Defaults: optimization rate 0.5-1.0 Hz; Huber delta 1.0; max iterations 50.
- Metrics: drift reduction, correction magnitude, optimization time.
- Artifacts: pose-graph configs, correction magnitude logs, timing profile.

---

## Phase 6 — Optional advanced improvements

Goal: incremental gains once core is solid.

### 6.1 Learned vibration scheduler
- Do: model inputs (throttle, RPM, attitude, vibration metrics) to output covariance scale and notch bandwidth; constrain monotonicity and bounds.
- Verify: smoother covariance scaling; no oscillatory behavior.
- Test hints: offline evaluate on labeled throttle/RPM logs; ensure outputs are bounded and monotone; run HIL to check stability.
- Implementation notes: start with small MLP (2 layers, 32 units, ReLU) or monotone spline; train offline with supervised targets from heuristic scaler; clip outputs to [1, s_max] and notch Q to [5, 25].
- Defaults: MLP 2 layers, 32 units; output bounds s ∈ [1, 5], Q ∈ [5, 25].
- Artifacts: model spec, validation plots.

### 6.2 Rolling shutter compensation
- Do: IMU-aware per-row pose correction; integrate with sub-pixel alignment.
- Verify: reduced motion distortion; cleaner high-speed tracks.
- Test hints: fast pan/tilt replay; measure rolling-shutter wobble reduction in reprojection metrics; check track stability during rapid yaw.
- Implementation notes: fit per-row pose using IMU preintegration over exposure; warp patches before LK; budget CPU by batching rows.
- Defaults: row batch size 4-8; max angular velocity for compensation 3 rad/s.
- Artifacts: before/after reprojection and track stability plots.

---

## Final architecture (target)

```
Sensors
 ├─ Stereo (sub-pixel tracked)
 ├─ IMU (adaptive covariance, optional notch)
 └─ ESC telemetry (RPM)

VIO Frontend (real-time)
 ├─ PROSAC + MAGSAC++
 ├─ Robust losses
 └─ Sliding window BA

SLAM Backend (async)
 ├─ Keyframes + landmarks
 ├─ Place recognition
 ├─ Loop verification
 └─ Pose graph optimization
```

---

Below is a clean block diagram you can drop into documentation, followed by a short explanation of each block. This reflects the upgraded embedded VIO -> VI-SLAM architecture we discussed.

![Embedded VIO/SLAM reference 1](https://www.mathworks.com/help/examples/vision/win64/MonocularVisualInertialSLAMExample_01.png)

![Embedded VIO/SLAM reference 2](https://www.researchgate.net/publication/360976186/figure/fig2/AS%3A1181907781976086%401658800182501/Diagram-of-the-architecture-of-visual-SLAM-systems.png)

![Embedded VIO/SLAM reference 3](https://www.researchgate.net/publication/347108258/figure/fig1/AS%3A1022671001231363%401620835175819/Pipeline-of-the-proposed-visual-inertial-odometry-system.png)

---

# Embedded VIO -> VI-SLAM Block Diagram

```
 +---------------------------------------------+
 |                 Sensors                     |
 |                                             |
 |  Stereo Cameras      IMU          ESC       |
 |  (Left / Right)   (Gyro+Accel)   Telemetry  |
 |                                             |
 +-----------+-------------+-----------+-------+
             |             |           |
             |             |           |
 +-----------v-------------+           |
 |       Image Frontend                |
 | - Rectify                           |
 | - Subpixel Tracking                 |
 | - Stereo KLT                        |
 +-----------+-------------------------+
             |             |
             |             |
             |   +---------v---------+
             |   |    IMU Frontend   |
             |   | - Sync            |
             |   | - Bias init       |
             |   | - Vib metrics     |
             |   +---------+---------+
             |             |
             |             |
             |             |
             |   +---------v---------+
             |   |  RPM / Power      |
             |   |  Telemetry        |
             |   | - RPM             |
             |   | - Current         |
             |   | - Voltage         |
             |   +---------+---------+
             |             |
             |             |
 +-----------v-------------v-----------+
 |       Estimator Conditioning        |
 | - Adaptive IMU Covariance           |
 | - Optional RPM Notch Filters        |
 +-----------+-------------------------+
             |
 +-----------v-------------------------+
 |    VIO Core (Real-Time, Tight)      |
 | - Feature / Track Selection         |
 | - PROSAC + MAGSAC++ Geometry        |
 | - Sliding Window Opt (BA + IMU)     |
 | - Robust Losses (Huber / Cauchy)    |
 | Outputs: Local Pose, Vel, Bias, KF  |
 +-----------+-------------------------+
             |
```

**Control uses this output only** (loop closure and global map corrections stay isolated)

```
 +-----------v-------------------------+
 |   Keyframe & Local Mapping Layer    |
 | - Keyframe Selection                |
 | - Persistent Landmarks              |
 | - Local BA / Culling                |
 +-----------+-------------------------+
             |
 +-----------v-------------------------+
 | Place Recognition & Loop Detection  |
 | - Global Descriptor / BoW           |
 | - Keyframe Database                 |
 | - Candidate Retrieval               |
 +-----------+-------------------------+
             |
 +-----------v-------------------------+
 |     Loop Verification (Robust)      |
 | - Descriptor Matching               |
 | - PROSAC + MAGSAC++                 |
 | - Geometric Consistency Checks      |
 +-----------+-------------------------+
             |
 +-----------v-------------------------+
 |   Pose Graph Optimization (Async)   |
 | - Nodes: Keyframes                  |
 | - Edges: Odom + Loop Closures       |
 | - Global Drift Correction           |
 +-----------+-------------------------+
             |
 +-----------v-------------------------+
 |   Global Map & Visualization        |
 | - Corrected Trajectory              |
 | - Consistent Map                    |
 | - Relocalization Reference          |
 +-------------------------------------+
```

---

## How to read this diagram (important)

### 1. Hard separation of responsibilities

- Top half = sensing + conditioning
- Middle = real-time estimation (must never block)
- Bottom = global consistency (slow, async)

This separation is non-negotiable on drones.

### 2. Control safety rule (critical)

Flight control uses only:

```
VIO Core -> Local Pose
```

Loop closure and pose graph never directly affect control. They only correct:

- the map
- the global frame
- visualization / long-term navigation

### 3. Where each upgrade fits

| Upgrade               | Block                            |
| --------------------- | -------------------------------- |
| Sub-pixel tracking    | Image Frontend                   |
| Stereo refinement     | Image Frontend                   |
| PROSAC + MAGSAC++     | VIO Core + Loop Verification     |
| Robust losses         | VIO Core                         |
| Covariance adaptation | Estimator Conditioning           |
| RPM-tracked notch     | Estimator Conditioning           |
| Keyframes             | Keyframe & Local Mapping         |
| Loop closure          | Place Recognition + Verification |
| Global correction     | Pose Graph Optimization          |

### 4. Why this scales well on embedded

- Real-time path is short and deterministic
- Heavy work is asynchronous
- Failures in loop closure cannot crash tracking
- Easy to disable features selectively

## One-sentence summary (doc-friendly)

> The system combines a real-time, tightly coupled VIO front-end with an asynchronous SLAM back-end that performs loop closure and global optimization, ensuring local stability for control and long-term global consistency for mapping.


## Priority if time-limited

1. Sub-pixel tracking
2. PROSAC + MAGSAC++
3. IMU covariance adaptation
4. Keyframes + persistent map
5. Loop closure
6. RPM-tracked notch filtering
7. Learned vibration scheduling
8. Rolling shutter compensation
