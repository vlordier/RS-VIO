# RS-VIO Implementation Roadmap & Feature Tracking

**Current Status**: Fusion module complete & production-ready
**Last Updated**: January 21, 2026
**Current Phase**: Prioritizing next major features

---

## Implementation Status Overview

### ✅ Completed Phases

| Phase | Scope | Status | Key Deliverables |
|-------|-------|--------|------------------|
| **Phase 1: Core VIO** | Basic SLAM pipeline | ✅ Complete | Feature tracking, stereo, BA, loop closure |
| **Phase 2: IMU Integration** | Preintegration, denoising, optimization | ✅ Complete | IMU factors, denoising filters, higher-order terms |
| **Phase 3: Super-Resolution** | Stereo SR refinement | ✅ Complete | Subpixel matching, confidence scoring, depth hypotheses |
| **Phase 4: Fusion System** | Multi-frame fusion strategies | ✅ Complete | Depth-aware + rotation-only strategies, config-driven selection |

### 🔄 In-Progress / Planned

| Phase | Scope | Priority | Estimated Effort |
|-------|-------|----------|------------------|
| **Phase 5: Auto-Calibration** | Manual-only calibration workflows | ⭐⭐⭐ High | 40-60 hours |
| **Phase 6: Feature Detection SOTA** | Track-first, detect-to-fill, learned descriptors | ⭐⭐⭐ High | 50-70 hours |
| **Phase 7: Real-Time Budget** | Adaptive scheduling, frame drop handling | ⭐⭐ Medium | 30-40 hours |
| **Phase 8: Advanced Geometric SR** | Plane segmentation, EPI-focused accumulation | ⭐⭐ Medium | 35-50 hours |
| **Phase 9: Integration Testing** | EuRoC benchmarks, comparative validation | ⭐⭐⭐ High | 20-30 hours |

---

## Detailed Feature Backlog

### Phase 5: Auto-Calibration System (High Priority)

**Objective**: Implement manual-only, ground-based calibration workflows for IMU, cameras, and stereo.

#### 5.1 IMU Self-Calibration
- **Status**: ✅ Complete
- **Description**: Six-pose method for bias, scale, and noise estimation
- **Implementation**: [src/calibration/imu_calibration.rs](src/calibration/imu_calibration.rs)
- **Features**:
  - State machine for guided calibration workflow
  - Per-pose statistics computation (mean, std, clipping detection)
  - Gyro bias + accel bias + scale estimation
  - Noise density calculation
  - Quality gates with configurable thresholds
  - Human-readable quality reports
- **Tests**: 3 unit tests (basic flow, statistics, clipping detection)
- **Lines of Code**: ~480 lines
- **Time Spent**: ~4 hours (under estimate)

#### 5.2 Camera Intrinsics Calibration
- **Status**: 🔴 Not Started
- **Description**: Monocular BA-based K + distortion solving
- **Requirements**:
  - Operator performs gentle motion in front of textured scene/checkerboard
  - System detects features, tracks across frames
  - Solves: focal length, principal point, radial/tangential distortion
  - For rolling shutter: estimate readout time
  - Regularization + outlier rejection via robust cost
- **Acceptance Criteria**:
  - Reprojection RMS < 0.3px (global) or 0.5px (rolling)
  - Distortion coefficients stable across retries
  - Rolling shutter readout time estimated if detected
  - Temperature/environment logged with solution
- **Dependencies**: Feature tracking, BA framework (exists)
- **Estimated Effort**: 15-18 hours

#### 5.3 Camera-IMU Time Offset
- **Status**: 🔴 Not Started
- **Description**: Cross-correlation of optical flow vs gyro rotation
- **Requirements**:
  - Operator performs slow pan/tilt with known angular velocity
  - System extracts optical flow and integrates gyro
  - Cross-correlates over time-offset range, refines with BA
  - Validates via gyro-to-flow consistency
- **Acceptance Criteria**:
  - Time offset uncertainty < 2ms
  - Correlation peak sharpness (SNR) > 3
  - Consistency with independent checks
- **Dependencies**: Feature tracking, optical flow (exists)
- **Estimated Effort**: 10-12 hours

#### 5.4 Stereo Extrinsics Calibration
- **Status**: 🔴 Not Started
- **Description**: Relative pose (R, t) + baseline via epipolar geometry
- **Requirements**:
  - Operator performs figure-8 or small translations
  - System tracks features in both cameras
  - Jointly optimizes: baseline, relative rotation, relative translation
  - Regularization with prior on baseline (±5%)
  - Validates with epipolar inlier ratio + angular error
- **Acceptance Criteria**:
  - Epipolar inlier ratio > 95%
  - Median epipolar error < 1px
  - Baseline uncertainty < 5% of magnitude
  - Consistency hold-out test: re-solve on unseen frames, divergence < 10%
- **Dependencies**: Stereo matching, epipolar geometry (exists)
- **Estimated Effort**: 12-15 hours

#### 5.5 Quality Gates & Operational Workflows
- **Status**: 🔴 Not Started
- **Description**: Automated acceptance criteria, operator overrides, versioning
- **Requirements**:
  - Pre-flight self-check (bias drift detection, geometry sanity)
  - In-flight monitoring only (no auto-recalibration)
  - Post-flight analysis and recalibration scheduling
  - Versioning: `calibration_v1.0.yaml`, `calibration_v1.1.yaml`, etc.
  - Integrity check: SHA256 hash + metadata validation
  - Guided prompts for each calibration step
- **Acceptance Criteria**:
  - All quality gates documented and tunable
  - Operator override workflow clear and logged
  - Calibration history fully recoverable
  - Pre-arm checks prevent flight with stale/invalid calibration
- **Dependencies**: Config system, logging (exists)
- **Estimated Effort**: 8-10 hours

**Total Estimated Effort (Phase 5)**: 57-70 hours
**Priority**: ⭐⭐⭐ HIGH (blocking production deployment)

---

### Phase 6: Feature Detection SOTA (High Priority)

**Objective**: Implement modern feature detection + tracking + description with compute budget awareness.

#### 6.1 Track-First, Detect-to-Fill Pattern
- **Status**: 🔴 Not Started
- **Description**: Maintain tracks on existing features; only detect when coverage drops
- **Requirements**:
  - Keep tracks alive across frames (KLT prolongation)
  - Spatial grid with coverage tracking
  - Detect new features only in under-populated cells
  - Per-feature uncertainty from tracking residuals + image gradients
  - Grid density / ANMS enforcement for spatial diversity
- **Acceptance Criteria**:
  - Track persistence: 80%+ of features tracked > 5 frames
  - Uniform spatial distribution (cell occupancy variance < 20%)
  - Uncertainty estimates correlate with re-detection rate (R² > 0.7)
  - CPU cost stable even with high feature counts
- **Dependencies**: Feature tracker, optical flow (exists)
- **Estimated Effort**: 15-18 hours

#### 6.2 Per-Feature Uncertainty Tracking
- **Status**: 🔴 Not Started
- **Description**: Uncertainty from image gradients, tracking residuals, and photometric noise
- **Requirements**:
  - Compute Laplacian/Hessian eigenvalues at feature location
  - Track residual magnitude as confidence measure
  - Accumulate photometric noise model per region
  - Propagate uncertainty through stereo matching
  - Use in BA weighting: lower uncertainty = higher weight
- **Acceptance Criteria**:
  - Uncertainty estimates > 0, consistent with feature quality
  - Features with high uncertainty downweighted in BA
  - Outlier rate < 5% for high-confidence features
  - Uncertainty correlates with reprojection error (R² > 0.65)
- **Dependencies**: Feature scoring, BA weighting (exists)
- **Estimated Effort**: 12-14 hours

#### 6.3 SuperPoint Integration (GPU-Conditional)
- **Status**: 🔴 Not Started
- **Description**: Learned keypoint detector + descriptor on GPU (Jetson/i7+)
- **Requirements**:
  - TensorRT model loading + inference pipeline
  - Conditional compilation: CPU-only build skips SuperPoint
  - GPU acceleration only on keyframes (not every frame)
  - Fallback to ORB if GPU unavailable or models missing
  - Descriptor database for relocalization
- **Acceptance Criteria**:
  - Inference time < 50ms on full resolution (Jetson Xavier)
  - Feature count comparable to ORB (200-500 points)
  - Better repeatability under blur/scale vs ORB
  - Graceful fallback on GPU OOM or inference failure
- **Dependencies**: TensorRT library, GPU drivers
- **Estimated Effort**: 18-22 hours

#### 6.4 LightGlue Matching (Loop Closure, Relocalization)
- **Status**: 🔴 Not Started
- **Description**: Learned matcher for keyframe-to-keyframe + relocalization
- **Requirements**:
  - SuperPoint keypoints as input
  - Efficient matching graph for loop closure queries
  - Runs only on keyframe creation / loop closure attempts (not every frame)
  - Robust to viewpoint/appearance changes
  - Optional: online learned descriptor refinement
- **Acceptance Criteria**:
  - Loop closure recall > 85% on standard benchmarks
  - Inlier ratio > 80% for correct matches
  - Relocalization success rate > 90% within 5m / 30°
  - Does not block real-time frame processing
- **Dependencies**: SuperPoint (6.3), GPU inference
- **Estimated Effort**: 12-16 hours

#### 6.5 ORB Loop Closure Fallback
- **Status**: 🔴 Not Started (ORB exists, loop closure exists, needs refinement)
- **Description**: Improve ORB-based loop closure robustness
- **Requirements**:
  - BoW (bag-of-words) database with inverted index
  - Faster candidate retrieval for loop closure queries
  - Geometric verification: Essential matrix + RANSAC
  - Multi-frame consistency check
- **Acceptance Criteria**:
  - Candidate retrieval < 50ms for 10k keyframes
  - False positive rate < 5%
  - Complements SuperPoint+LightGlue fallback
- **Dependencies**: Existing ORB + loop closure (partially exists)
- **Estimated Effort**: 8-10 hours

**Total Estimated Effort (Phase 6)**: 65-80 hours
**Priority**: ⭐⭐⭐ HIGH (enables robust relocalization + compute flexibility)

---

### Phase 7: Real-Time Budget Optimization (Medium Priority)

**Objective**: Adaptive scheduling, frame drop detection, and graceful degradation.

#### 7.1 Frame Processing Budget Tracker
- **Status**: 🔴 Not Started
- **Description**: Per-module timing + budget headroom monitoring
- **Requirements**:
  - Measure: feature detection, tracking, stereo matching, SR, fusion, BA times
  - Track moving average + peak latency
  - Predict frame deadline vs actual completion time
  - Alert if budget exceeded, log to telemetry
- **Acceptance Criteria**:
  - Timing accurate to ±1ms
  - Detects budget overruns before frame drop
  - No performance regression from instrumentation
- **Dependencies**: Core pipeline (exists)
- **Estimated Effort**: 6-8 hours

#### 7.2 Module Scheduling & Priority
- **Status**: 🔴 Not Started
- **Description**: Conditional execution based on compute headroom
- **Requirements**:
  - SuperPoint/LightGlue: only run if headroom > 10ms
  - Fusion: skip if next frame arriving soon
  - BA: skip if no new keyframe
  - Loop closure: background thread with time quota
  - Prioritize frame tracking (never skip)
- **Acceptance Criteria**:
  - Frame tracking latency stable even if BA heavy
  - Loop closure doesn't cause frame drops
  - Configuration controls module thresholds
- **Dependencies**: Budget tracker (7.1), module integration
- **Estimated Effort**: 12-15 hours

#### 7.3 Graceful Degradation & Frame Drop Handling
- **Status**: 🔴 Not Started
- **Description**: Detect, log, and recover from frame drops
- **Requirements**:
  - Track frame timestamp gaps
  - Log dropped frames with reason (BA overrun, GPU OOM, etc.)
  - Adjust IMU preintegration if frames missing
  - Telemetry report: drop rate, impact on trajectory
- **Acceptance Criteria**:
  - Drops detected with < 1 frame latency
  - IMU accounting correct despite gaps
  - Drop rate < 2% on target platform at target FPS
- **Dependencies**: IMU preintegration, logging (exists)
- **Estimated Effort**: 8-10 hours

#### 7.4 Compute-Aware Mode Selection
- **Status**: 🔴 Not Started
- **Description**: Auto-select fusion strategy, feature detector based on platform
- **Requirements**:
  - Detect available GPU / compute class at startup
  - Auto-select: SuperPoint on Jetson/GPU, GFTT on CPU-only
  - Auto-select: depth-aware fusion if budget, rotation-only otherwise
  - User override via config
- **Acceptance Criteria**:
  - Auto-selection matches manual tuning within 10%
  - No hard failures on undersized platforms
  - Clear log output of mode decisions
- **Dependencies**: Budget tracker (7.1), module scheduling (7.2)
- **Estimated Effort**: 6-8 hours

**Total Estimated Effort (Phase 7)**: 32-41 hours
**Priority**: ⭐⭐ MEDIUM (optimization, nice-to-have before production)

---

### Phase 8: Advanced Geometric SR (Medium Priority)

**Objective**: Plane segmentation, EPI-focused accumulation, and direct/semi-direct methods.

#### 8.1 Plane-Segmented Fusion
- **Status**: 🔴 Not Started
- **Description**: RANSAC-based plane detection + per-plane homography fusion
- **Requirements**:
  - Detect planar regions in scene (ground, building facades)
  - Per-plane homography from stereo depth + pose
  - Warp + fuse patches within each plane separately
  - Handle occlusions and moving objects gracefully
- **Acceptance Criteria**:
  - > 90% of frames have ≥ 2 planes detected
  - Fusion quality + 5% over non-segmented in structured scenes
  - Computational cost < 5ms per frame
- **Dependencies**: Depth estimation, stereo matching (exists)
- **Estimated Effort**: 15-18 hours

#### 8.2 EPI-Focused Stereo Accumulation
- **Status**: 🔴 Not Started
- **Description**: Multi-frame stereo cost volume with pose compensation
- **Requirements**:
  - Accumulate disparity hypotheses along epipolar lines
  - Compensate for camera pose across frames
  - Reduces fake corners from temporal noise
  - Improves disparity precision at no extra feature count
- **Acceptance Criteria**:
  - Disparity uncertainty < 0.5px (from 1-2px baseline)
  - Feature detection reliability + 8%
  - Computational cost < 3ms per frame
- **Dependencies**: Multi-frame fusion, stereo (exists)
- **Estimated Effort**: 12-15 hours

#### 8.3 Semi-Direct (SVO-Style) Patch Tracking
- **Status**: 🔴 Not Started
- **Description**: Photometric alignment of patches with depth filter
- **Requirements**:
  - Track patches (not just corners) with direct alignment
  - Estimate depth via Bayesian filter from epipolar geometry
  - Works well with good IMU and exposure control
  - Complements keypoint tracking for robustness
- **Acceptance Criteria**:
  - Tracks last 2x longer than KLT in low-texture regions
  - Depth estimates within ±20% of stereo in 80% of frames
  - Latency < 2ms per tracked patch
- **Dependencies**: IMU (exists), photometric calibration (8.4)
- **Estimated Effort**: 18-22 hours

#### 8.4 Photometric Calibration
- **Status**: 🔴 Not Started
- **Description**: Online vignetting + exposure model estimation
- **Requirements**:
  - Estimate per-pixel vignette map
  - Estimate exposure time / gain
  - Detect rolling shutter effects and correct
  - Use in photometric residual weighting
- **Acceptance Criteria**:
  - Photometric residuals more uniform across image
  - Direct method convergence improved
  - < 1ms overhead per frame
- **Dependencies**: Semi-direct patches (8.3), BA (exists)
- **Estimated Effort**: 10-12 hours

**Total Estimated Effort (Phase 8)**: 55-67 hours
**Priority**: ⭐⭐ MEDIUM (advances to SOTA, nice-to-have)

---

### Phase 9: Integration Testing & Benchmarking (High Priority)

**Objective**: Comprehensive validation on standard datasets with ground truth.

#### 9.1 EuRoC Benchmark Suite
- **Status**: 🔴 Not Started
- **Description**: Automated runs on all EuRoC sequences with ground truth comparison
- **Requirements**:
  - Load all EuRoC MH01-MH05, V102-V203
  - Run each with all fusion strategies
  - Log trajectory + compute metrics (RMSE, ATE, RPE)
  - Generate comparison tables / plots
- **Acceptance Criteria**:
  - ATE < 10cm for MH sequences
  - ATE < 20cm for V sequences
  - Depth-aware fusion shows measurable improvement over baseline
  - Rotation-only shows 50-70% of depth-aware benefit at lower cost
- **Dependencies**: Dataset loading (exists), metrics (to implement)
- **Estimated Effort**: 10-12 hours

#### 9.2 Metrics & Reporting Framework
- **Status**: 🔴 Not Started
- **Description**: Trajectory error metrics, timing profiling, memory profiling
- **Requirements**:
  - Absolute trajectory error (ATE)
  - Relative pose error (RPE)
  - Per-module execution times
  - Memory peak / average usage
  - Feature count, tracking rate, loop closure success
  - Generate JSON reports + markdown summaries
- **Acceptance Criteria**:
  - Metrics match standard TUM evaluation code
  - Reports reproducible across runs
  - < 1% metric variance with fixed seed
- **Dependencies**: Benchmark infrastructure
- **Estimated Effort**: 8-10 hours

#### 9.3 Regression Testing & CI
- **Status**: 🔴 Not Started
- **Description**: Automated tests on subset of datasets, CI/CD integration
- **Requirements**:
  - Run on MH01 + V102 per PR / commit
  - Track metrics over time (git history)
  - Alert on regressions (> 5% ATE increase)
  - GitHub Actions integration
- **Acceptance Criteria**:
  - CI runs in < 5 minutes
  - Regressions detected before merge
  - Historical trend plots available
- **Dependencies**: Metrics framework (9.2)
- **Estimated Effort**: 6-8 hours

#### 9.4 Platform-Specific Validation
- **Status**: 🔴 Not Started
- **Description**: Test on actual target hardware (Jetson Nano/Xavier, i7)
- **Requirements**:
  - Jetson Nano: verify rotation-only + ORB fits real-time budget
  - Jetson Xavier: verify depth-aware + SuperPoint feasible
  - Desktop (i7): baseline for comparison
  - Log: actual frame rates, latencies, memory
- **Acceptance Criteria**:
  - Nano achieves 30 FPS with rotation-only fusion
  - Xavier achieves 20 FPS with depth-aware + SuperPoint
  - Memory footprint < 1GB on Nano, < 2GB on Xavier
- **Dependencies**: All other phases
- **Estimated Effort**: 8-10 hours

#### 9.5 Stress Testing & Edge Cases
- **Status**: 🔴 Not Started
- **Description**: Test robustness to challenging conditions
- **Requirements**:
  - Low light, motion blur, fast rotations
  - Camera shake, rolling shutter artifacts
  - Feature-poor scenes (glass, sky)
  - Quick occlusions and re-emergence
  - Calibration drift detection
- **Acceptance Criteria**:
  - System doesn't crash on any input
  - Graceful degradation when features sparse
  - Recovery within 1-2 frames after occlusion
- **Dependencies**: Core pipeline
- **Estimated Effort**: 6-8 hours

**Total Estimated Effort (Phase 9)**: 38-48 hours
**Priority**: ⭐⭐⭐ HIGH (validates production readiness)

---

## Summary: Feature Prioritization & Timeline

### By Priority & Effort

| Phase | Priority | Est. Hours | Blocking | Sequential Dependencies |
|-------|----------|-----------|----------|-------------------------|
| Phase 5: Auto-Calibration | ⭐⭐⭐ | 57-70 | Yes (production) | Independent |
| Phase 9: Integration Testing | ⭐⭐⭐ | 38-48 | Yes (validation) | Needs other phases complete |
| Phase 6: Feature Detection SOTA | ⭐⭐⭐ | 65-80 | Partial | Independent (some deps) |
| Phase 7: Real-Time Budget | ⭐⭐ | 32-41 | No | Can start anytime |
| Phase 8: Advanced Geometric SR | ⭐⭐ | 55-67 | No | Independent |

### Recommended Sequence

**Path 1: Shortest to Production (4-5 weeks, 1-2 developers)**
1. Phase 5: Auto-Calibration (weeks 1-2, 57-70h)
2. Phase 9: Integration Testing (week 3, 38-48h)
3. Phase 6: Feature Detection SOTA (weeks 4-5, 65-80h)

**Path 2: Maximum Quality (8-10 weeks, 2 developers)**
1. Phase 5: Auto-Calibration (weeks 1-2)
2. Phase 9: Integration Testing (week 3)
3. Phase 6: Feature Detection SOTA (weeks 4-5)
4. Phase 7: Real-Time Budget (week 6, parallel with 6)
5. Phase 8: Advanced Geometric SR (weeks 7-8)

**Path 3: Parallel Development (6-7 weeks, 3+ developers)**
- Assign Phase 5 to one developer
- Assign Phase 6 to one developer
- Assign Phases 7-8 to one developer
- Phase 9 in parallel once earlier phases stabilize

---

## Known Technical Debt & Limitations

### Currently Documented (in code)

- [ ] `src/fusion/rotation_stabilizer.rs:80, 126`: TODO integrate gyro warping into fuse() production path
- [ ] `src/feature_tracker/feature_tracker/stereo_tracker.rs:549`: TODO connect to actual fusion/estimator velocity estimates
- [ ] `src/estimator/sliding_window/optimization.rs:868`: Reserved for advanced triangulation with motion compensation
- [ ] Rolling shutter handling: partially implemented, needs full correction in photometric alignment
- [ ] Online intrinsics refinement: exists but not wired to operator feedback loop
- [ ] Loop closure relocalization: basic ORB+BoW, needs LightGlue integration for robustness

### Performance Gaps

- [ ] Frame processing latency on Jetson Nano: unknown (need platform validation)
- [ ] Feature detection repeatability under rolling shutter: not quantified
- [ ] Fusion overhead on low-texture scenes: no metrics yet
- [ ] Multi-scale feature detection: reserved but not implemented
- [ ] Online parameter adaptation: only manual calibration implemented

### Missing Integration Points

- [ ] Calibration system ↔ flight software
- [ ] Real-time budget ↔ module scheduler
- [ ] Photometric calibration ↔ BA weighting
- [ ] Loop closure ↔ relocalization decision logic
- [ ] Plane segmentation ↔ fusion strategy selection

---

## Open Questions & Design Decisions Pending

1. **GPU Model Deployment**: How to package SuperPoint/LightGlue models?
   - Option A: Embed in binary (larger binary size)
   - Option B: Download at first run
   - Option C: Part of separate calibration/setup phase

2. **Plane Segmentation Robustness**: How to handle moving objects / dynamic scenes?
   - Current design assumes static scene
   - Need motion-aware segmentation or skip fusion in dynamic regions

3. **Calibration Versioning**: How many old calibrations to keep?
   - Current plan: keep 5-10 most recent
   - Need policy for very old platforms (1 year+)

4. **Loop Closure Frequency**: How often to run expensive matcher?
   - Current: only on keyframe creation
   - Alternative: background thread with time quota

5. **Time Budget Allocation**: Fixed vs adaptive quotas per module?
   - Fixed: predictable, tuned per platform
   - Adaptive: responds to scene complexity

---

## Metrics & Success Criteria

### By End of Phase 5 (Auto-Calibration)
- ✓ IMU calibration: bias uncertainty < 2%
- ✓ Camera intrinsics: reprojection RMS < 0.3px
- ✓ Stereo extrinsics: epipolar error < 1px
- ✓ System prevents flight with stale calibration

### By End of Phase 6 (Feature Detection)
- ✓ Track persistence: > 80% of features tracked > 5 frames
- ✓ SuperPoint inference: < 50ms on Jetson Xavier
- ✓ Loop closure recall: > 85% on standard benchmarks
- ✓ Graceful CPU-only fallback

### By End of Phase 9 (Integration Testing)
- ✓ EuRoC ATE: < 10cm (MH), < 20cm (V)
- ✓ Regression testing integrated to CI
- ✓ Jetson Nano: 30 FPS with rotation-only fusion
- ✓ Jetson Xavier: 20 FPS with depth-aware + SuperPoint

---

## References & Related Docs

- [FUSION_ARCHITECTURE.md](FUSION_ARCHITECTURE.md) — Completed fusion module
- [FUSION_QUICKSTART.md](FUSION_QUICKSTART.md) — User deployment guide
- [Untitled-4](../Untitled-4) — Original technical notes (comprehensive)
- [SESSION_COMPLETION_FUSION.md](SESSION_COMPLETION_FUSION.md) — Phase 4 completion

---

**Maintainers**: RS-VIO Team
**Last Reviewed**: January 21, 2026
**Next Review**: After Phase 5 starts or upon major design decision
