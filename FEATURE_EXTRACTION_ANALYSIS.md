# Feature Extraction Analysis: develop-old → develop

**Repository:** RS-VIO (Visual-Inertial Odometry)
**Date:** February 5, 2026
**Analysis Scope:** Commits between `develop` and `develop-old` branches

---

## Executive Summary

The `develop-old` branch contains **256 commits** with **226,047 lines added** and **3,106 lines removed** across **680 files**. These changes represent major feature developments organized into distinct phases and categories. The work spans 9+ phases including SLAM implementation, IMU integration, async concurrency, dense reconstruction, and multi-drone coordination.

### Key Statistics
- **Total Commits:** 256
- **Files Changed:** 680
- **Lines Added:** 226,047
- **Lines Removed:** 3,106
- **Net Addition:** +222,941 lines

---

## Major Feature Categories

### 1. **SLAM Backend & Pose Graph** (Phases 1-3)
**Status:** Complete | **Commits:** 48 | **Complexity:** HIGH

#### Architecture:
- Global pose graph implementation for backend optimization
- Visual factor integration into SLAM framework
- IMU factor integration for tight coupling
- Benchmarking infrastructure

#### Key Files (44 files across estimator/optimization):
```
src/optimization/
├── pose_graph/ (Global optimization)
├── visual_factors/ (Phase 2A)
├── imu_factors/ (Phase 2B)
└── benchmarking/ (Phase 2C)
```

#### Code Impact:
- **Estimator module:** 12,186 lines added (+994 deleted)
- **Optimization module:** 11,144 lines added (+284 deleted)
- **Key commits:**
  - `eb8a3370`: Global pose graph implementation
  - `c6c98e01`: Integrate GlobalPoseGraph with Estimator
  - `f2c0b5cf`: Full Visual Factor Integration
  - `2115dcb8`: IMU factor integration complete

#### Integration Complexity: **HIGH**
- Requires careful integration with existing estimator
- Dependencies: feature_tracker, imu modules
- Risk: Major architectural change to optimization pipeline
- **Recommendation:** Extract as `feature/slam-backend` - HIGH PRIORITY

---

### 2. **Calibration Framework** (Task 4 & Ongoing)
**Status:** Complete | **Commits:** 20+ | **Complexity:** MEDIUM

#### Comprehensive System:
- Online intrinsic calibration (camera & IMU)
- Camera-IMU extrinsic calibration
- Rolling shutter calibration
- Time offset estimation
- Acceptance validation framework
- Sensitivity analysis

#### Key Files:
```
src/calibration/ (15 files, 6,012 lines added)
├── camera_intrinsics.rs (Focal length, principal point, distortion)
├── imu_intrinsics.rs (Gyro/accel biases, noise parameters)
├── camera_imu_extrinsics.rs (Rotation + translation)
├── rolling_shutter.rs (Line-wise exposure timing)
├── time_offset.rs (Sensor synchronization)
├── online_intrinsics.rs (Real-time refinement)
├── online_time_offset.rs (Drift correction)
├── acceptance_validator.rs (Quality gates)
├── sensitivity_analysis.rs (Uncertainty estimation)
├── unified_solver.rs (Joint optimization)
├── manual_workflow.rs (Operator UI)
└── types.rs (Domain types)
```

#### Code Impact:
- **6,012 lines** of new calibration code
- **464 lines** in acceptance validator alone
- Modular design: can be adopted independently

#### Key Commits:
- `dd888c24`: Calibration-aware distance/speed metrics (600 LOC)
- `ef8ab09a`: Calibration-aware fusion integration
- `6cfda225`: Phase 0.1 - Calibration and sync verification
- `ad2a6449`: End-to-end validation report

#### Integration Complexity: **MEDIUM**
- Self-contained module with clear interfaces
- Minimal dependencies on existing code
- Can run as standalone or integrated pipeline
- **Recommendation:** Extract as `feature/calibration-framework` - MEDIUM-HIGH PRIORITY

---

### 3. **Async/Concurrent Pipeline** (Phase 4)
**Status:** Complete | **Commits:** 14+ | **Complexity:** HIGH

#### Architecture:
- Tokio-based async foundation
- Concurrent feature detection
- Async bundle adjustment optimizer
- Production hardening with error handling, metrics, timeouts

#### Key Files:
```
src/estimator/
├── async_feature_detection.rs (Concurrent tracking)
├── async_optimization.rs (AsyncOptimizer for BA)
├── async_wrapper.rs (Pipeline integration)
├── concurrent.rs (Worker pool management)
└── circuit_breaker.rs (Failure isolation)
```

#### Code Impact:
- Part of 12,186 lines in estimator module
- 3+ core async modules implemented
- Comprehensive test coverage with benchmarks
- Production deployment checklist validated

#### Key Commits:
- `cbc12671`: Phase 4 async foundation with tokio
- `a10db2d2`: Working concurrent pipeline + tests
- `8075a64e`: AsyncOptimizer for bundle adjustment
- `f6a2ee85`: Async feature detection integration
- `ddff23f9`: Production hardening (error handling, metrics, timeouts)

#### Dependencies:
- Tokio async runtime
- Rayon for data parallelism
- Arc/Mutex for thread-safe coordination

#### Integration Complexity: **HIGH**
- Requires careful async/await integration
- Must coordinate with existing synchronous pipeline
- Risk: Performance regression if not properly tuned
- **Recommendation:** Extract as `feature/async-pipeline` - HIGH PRIORITY (pairs well with SLAM)

---

### 4. **IMU Integration & Fusion** (Phases 5-8)
**Status:** Complete | **Commits:** 56 | **Complexity:** HIGH

#### Comprehensive IMU System:
- Preintegration framework
- Motion prediction and velocity estimation
- Extrinsic calibration
- Harmonic decomposition for motor/vibration analysis
- IMU-guided stereo super-resolution
- Visualization in Rerun

#### Key Files:
```
src/imu/ (21 files, 6,313 lines added)
├── preintegration.rs (Factor calculation)
├── motion_prediction.rs (Next-frame prediction)
├── velocity_estimation.rs (State fusion)
├── extrinsic_calibration.rs (Alignment optimization)
├── harmonic_analysis.rs (Frequency decomposition)
└── visualization.rs (Rerun integration)
```

#### Code Impact:
- **6,313 lines** of new IMU code
- Integrated with feature_tracker for velocity hints
- Connected to optimization for factor insertion
- IMU visualization with harmonic decomposition

#### Key Commits:
- `8d74e56d`: Preintegration, motion prediction, velocity estimation (major)
- `11010606`: Motor state detection with harmonic analysis
- `ac824074`: IMU-guided stereo super-resolution
- `72424ea1`: IMU visualization in Rerun
- `758f70e9`: Fusion frame buffer optimization
- `1ab6497c`: Velocity hint integration with fusion

#### Integration Complexity: **HIGH**
- Touches multiple subsystems: optimization, feature_tracker, viewers
- Requires careful synchronization of IMU measurements
- Risk: Incorrect preintegration can cause estimation divergence
- **Recommendation:** Extract as `feature/imu-fusion` - CRITICAL (used by SLAM)

---

### 5. **Dataset & Real-World Validation** (Phase 7)
**Status:** Complete | **Commits:** 30+ | **Complexity:** MEDIUM

#### Dataset Support:
- TUM-VI dataset integration with full loader
- EuRoC dataset support
- 4Seasons dataset integration
- Automated dataset downloading and verification
- Ground truth trajectory evaluation module

#### Key Files:
```
src/datasets/ (10 files, 3,233 lines added)
├── tum_vi.rs / tum_vi_player.rs (TUM-VI specific)
├── euroc_player.rs (EuRoC loader)
├── fourseasons_player.rs (4Seasons loader)
├── trajectory_eval.rs (GT evaluation)
├── frame_processor_trait.rs (Abstract interface)
└── player_trait.rs (Dataset player abstraction)
```

#### Code Impact:
- **3,233 lines** new dataset code
- Multiple benchmark configurations in `config/`
- Scripts for automated downloading

#### Key Commits:
- `8a852045`: TUM-VI Real-World Dataset Integration
- `30060f55`: TUM-VI loader and benchmark fixes
- `a1ee93c3`: Ground truth trajectory evaluation module
- `622fe7d0`: Robust dataset setup with local storage

#### Integration Complexity: **MEDIUM**
- Integrates with main pipeline
- Adds benchmark configurations
- Can be incrementally adopted
- **Recommendation:** Extract as `feature/dataset-support` - MEDIUM PRIORITY

---

### 6. **Dense Reconstruction** (Phase 8)
**Status:** Complete | **Commits:** 8 | **Complexity:** MEDIUM-HIGH

#### 3D Reconstruction:
- TSDF volume-based voxel grid
- Point cloud generation
- Mesh extraction

#### Key Files:
```
src/dense_reconstruction/ (3 modules)
├── tsdf_volume.rs (Signed distance field)
├── point_cloud.rs (Cloud generation)
└── mesh_extraction.rs (Surface reconstruction)
```

#### Code Impact:
- **1,463 lines** across 3 modules
- 38 tests validating reconstruction quality
- Complete module with minimal external dependencies

#### Key Commits:
- `7c63d1b0`: Phase 8 Dense Reconstruction complete (1463 LOC, 38 tests)

#### Integration Complexity: **MEDIUM-HIGH**
- Standalone module (can run post-optimization)
- Requires pose trajectory from SLAM
- Good candidate for optional feature
- **Recommendation:** Extract as `feature/dense-reconstruction` - MEDIUM PRIORITY

---

### 7. **Multi-Drone Coordination** (Phase 9)
**Status:** Complete | **Commits:** 10 | **Complexity:** HIGH

#### Distributed SLAM:
- Boids-based coordination (17 tests, 431 LOC)
- Map merging algorithms (14 tests, 542 LOC)
- Exploration strategy (12 tests, 516 LOC)
- Distributed optimization (14 tests, 501 LOC)

#### Key Commits:
- `36b6cc9b`: Phase 9.1 Boids Coordination (431 LOC)
- `fe21d24a`: Phase 9.2 Map Merging (542 LOC)
- `5ed759ae`: Phase 9.3 Exploration Strategy (516 LOC)
- `b4acb358`: Phase 9.4 Distributed Optimization (501 LOC)

#### Code Impact:
- **1,990 lines** across 4 sub-phases
- 57 tests validating distributed algorithms
- Complete multi-drone framework

#### Integration Complexity: **HIGH**
- Requires SLAM backend (Phase 1-3)
- Requires communication infrastructure
- Advanced feature for swarm systems
- **Recommendation:** Extract as `feature/multi-drone-slam` - LOWER PRIORITY (advanced use case)

---

### 8. **Optimization & CPU Tuning**
**Status:** Complete | **Commits:** 20+ | **Complexity:** MEDIUM

#### Performance Work:
- CPU-only realtime profiles (frames at ~15 FPS with frame skipping)
- Feature detection parallelization
- Grid operations optimization
- NALgebra optimization (97 IR lines)
- Binary bloat reduction
- Frame stride option for dataset processing

#### Key Files:
```
config/
├── tum_vi_realtime_cpu.yaml (CPU tuning)
├── tum_vi_realtime_cpu_extreme.yaml (Max optimization)
└── tum_vi_safe_longrun.yaml (Conservative)
```

#### Code Impact:
- Optimization module: **11,144 lines** added
- Configuration-driven tuning
- Minimal code changes (mostly config)

#### Key Commits:
- `d679a126`: CPU realtime profile + frame stride option
- `ca930ab1`: Parallelize feature detection, grid ops, optimization
- `e478a67b`: Extreme realtime CPU profile
- `db78ece0`: NALgebra optimization phase

#### Integration Complexity: **LOW**
- Primarily configuration-based
- No breaking API changes
- Performance improvements are additive
- **Recommendation:** Can be merged progressively with other features

---

### 9. **Testing & Validation Infrastructure**
**Status:** Complete | **Commits:** 30+ | **Complexity:** MEDIUM

#### Test Additions:
- 20+ new integration test files
- Property-based testing with PROSAC/MAGSAC++
- Robustness tests
- Real dataset benchmarking
- Stress tests
- Rolling shutter validation

#### Code Impact:
- Tests directory: **20+ new test files**
- Comprehensive coverage for all major features
- Benchmarking suite with CSV outputs

#### Key Commits:
- `5ba85280`: Phase 4.3.3 pipeline validation
- `ad131780`: Phases 7-9 validation summary
- `211fc4d1`: PROSAC and MAGSAC++ robustness
- Multiple test infrastructure commits

#### Integration Complexity: **LOW**
- Tests can be added independently
- Validates existing features
- **Recommendation:** Include with corresponding features

---

### 10. **Visualization & Monitoring**
**Status:** Complete | **Commits:** 15+ | **Complexity:** LOW-MEDIUM

#### Rerun Integration:
- Loop closure visualization
- IMU data visualization with harmonics
- Feature tracking visualization
- Real-time monitoring with stabilizer

#### Key Files:
```
src/viewers/ (6 files, 1,181 lines added)
├── rerun_integration.rs
└── realtime_monitoring.rs
```

#### Integration Complexity: **LOW-MEDIUM**
- Decoupled from core VIO
- Optional visualization layer
- Can be integrated last
- **Recommendation:** Include with corresponding features or as optional overlay

---

## Recommended Integration Order

### **TIER 1: Foundation** (Must integrate first)
1. **IMU Fusion** (`feature/imu-fusion`)
   - Lines: 6,313 new
   - Commits: 56 (Phase 5-8 related)
   - Complexity: HIGH
   - Risk: Medium
   - Value: CRITICAL - Enables tight-coupled VIO
   - Duration: 2-3 days integration + testing
   - Dependencies: None (self-contained)

2. **Calibration Framework** (`feature/calibration-framework`)
   - Lines: 6,012 new
   - Commits: 20+
   - Complexity: MEDIUM
   - Risk: Low
   - Value: HIGH - Essential for production systems
   - Duration: 1-2 days integration
   - Dependencies: None (can be standalone)

### **TIER 2: Core Architecture** (Depends on Tier 1)
3. **SLAM Backend & Pose Graph** (`feature/slam-backend`)
   - Lines: 23,330 total (12,186 estimator + 11,144 optimization)
   - Commits: 48
   - Complexity: HIGH
   - Risk: High (major architectural change)
   - Value: CRITICAL - Complete SLAM system
   - Duration: 3-4 days integration + extensive testing
   - Dependencies: IMU Fusion, Feature Tracker
   - **Recommended: Do after IMU to enable tight coupling**

4. **Async Concurrent Pipeline** (`feature/async-pipeline`)
   - Lines: 3+ modules in estimator
   - Commits: 14+
   - Complexity: HIGH
   - Risk: High (concurrency is subtle)
   - Value: HIGH - Enables real-time processing
   - Duration: 2-3 days for careful integration
   - Dependencies: SLAM backend (optional but recommended)
   - **Recommended: Test thoroughly before merge**

### **TIER 3: Extensions** (Can integrate in any order)
5. **Dataset Support** (`feature/dataset-support`)
   - Lines: 3,233 new
   - Commits: 30+
   - Complexity: MEDIUM
   - Risk: Low
   - Value: HIGH - Real-world testing
   - Duration: 1 day
   - Dependencies: None

6. **Dense Reconstruction** (`feature/dense-reconstruction`)
   - Lines: 1,463 new
   - Commits: 8
   - Complexity: MEDIUM-HIGH
   - Risk: Low (isolated)
   - Value: MEDIUM - Post-processing enhancement
   - Duration: 1 day
   - Dependencies: SLAM backend

### **TIER 4: Advanced** (Post-MVP)
7. **Multi-Drone Coordination** (`feature/multi-drone-slam`)
   - Lines: 1,990 new
   - Commits: 10
   - Complexity: HIGH
   - Risk: High (distributed systems)
   - Value: MEDIUM - Advanced use case
   - Duration: 2-3 days
   - Dependencies: SLAM backend, communication layer

### **TIER 5: Polish** (Can integrate throughout)
8. **Optimization & Tuning**
   - Lines: Config-driven
   - Commits: 20+
   - Complexity: LOW
   - Risk: Very Low
   - Value: MEDIUM
   - **Recommendation: Integrate progressively with main features**

9. **Testing Infrastructure**
   - Can integrate with each feature
   - Comprehensive validation coverage
   - **Recommendation: Add with corresponding features**

10. **Visualization & Monitoring**
    - Can integrate last (optional overlay)
    - Recommended: Include with SLAM and IMU
    - **Recommendation: Add incrementally**

---

## Top 5 Integration Candidates: Detailed File Breakdown

### **#1: IMU Fusion** (`feature/imu-fusion`)
**Priority:** CRITICAL | **Duration:** 2-3 days | **Lines:** 6,313

**Core Files:**
```
src/imu/mod.rs                     (Index & exports)
src/imu/preintegration.rs          (IMU factor calculation, ~1200 LOC)
src/imu/motion_prediction.rs       (Velocity prediction, ~800 LOC)
src/imu/velocity_estimation.rs     (State fusion, ~900 LOC)
src/imu/extrinsic_calibration.rs   (T_ic optimization, ~700 LOC)
src/imu/harmonic_analysis.rs       (Frequency decomposition, ~600 LOC)
src/imu/visualization.rs           (Rerun integration, ~400 LOC)
src/imu/types.rs                   (Domain types, ~300 LOC)
src/imu/tests/ (21 test files comprehensive)
```

**Integration Points:**
- `src/estimator/estimator.rs`: Add IMU factor insertion
- `src/feature_tracker/mod.rs`: Accept velocity hints
- `src/optimization/factors.rs`: Define IMU factor residuals
- `src/common/types.rs`: IMU measurement and state types

**Risk Assessment:**
- Medium: Requires preintegration correctness validation
- Test: 200+ unit tests already written
- Validation: Compare against ground truth on TUM-VI

**Benefits:**
- Tight-coupled VIO (versus loose coupling)
- Better motion prediction
- Improved tracking robustness in low-texture areas

---

### **#2: Calibration Framework** (`feature/calibration-framework`)
**Priority:** HIGH | **Duration:** 1-2 days | **Lines:** 6,012

**Core Files:**
```
src/calibration/mod.rs                      (Module index)
src/calibration/camera_intrinsics.rs        (f_x, c_x, distortion ~600 LOC)
src/calibration/imu_intrinsics.rs           (Gyro/accel bias, noise ~500 LOC)
src/calibration/camera_imu_extrinsics.rs    (T_ic estimation ~700 LOC)
src/calibration/rolling_shutter.rs          (Line-wise timing ~500 LOC)
src/calibration/time_offset.rs              (Sensor sync ~400 LOC)
src/calibration/online_intrinsics.rs        (Real-time refinement ~600 LOC)
src/calibration/online_time_offset.rs       (Drift correction ~400 LOC)
src/calibration/acceptance_validator.rs     (Quality gates ~464 LOC)
src/calibration/sensitivity_analysis.rs     (Uncertainty ~500 LOC)
src/calibration/unified_solver.rs           (Joint optimization ~600 LOC)
src/calibration/manual_workflow.rs          (UI workflow ~400 LOC)
src/calibration/types.rs                    (Domain types ~200 LOC)
```

**Integration Points:**
- `src/feature_tracker/mod.rs`: Apply distortion correction
- `src/estimator/estimator.rs`: Use calibration parameters
- `src/datasets/tum_vi_player.rs`: Intrinsic calibration from dataset
- Binary `src/bin/calib_check.rs`: Standalone validation tool

**Risk Assessment:**
- Low: Self-contained module, minimal dependencies
- Can be tested independently
- Graceful degradation if not used

**Benefits:**
- Removes hardcoded camera parameters
- Enables online calibration for varying cameras
- Quality gates for production systems
- Rolling shutter support (important for many cameras)

---

### **#3: SLAM Backend & Pose Graph** (`feature/slam-backend`)
**Priority:** CRITICAL | **Duration:** 3-4 days | **Lines:** 23,330

**Core Files:**
```
src/optimization/pose_graph.rs              (Global optimization, ~1500 LOC)
src/optimization/factors.rs                 (Factor definitions, ~1200 LOC)
src/optimization/solver.rs                  (Solver integration, ~800 LOC)
src/optimization/marginalization.rs         (Schur complement, ~900 LOC)
src/optimization/residuals/                 (Visual & IMU residuals ~1500 LOC)
  ├── visual_residual.rs
  ├── imu_residual.rs
  └── pose_residual.rs
src/optimization/visual_factors/            (Phase 2A)
  ├── reprojection_factor.rs
  ├── epipolar_constraint.rs
  └── bundle_adjustment.rs
src/optimization/imu_factors/               (Phase 2B)
  ├── preintegration_factor.rs
  ├── bias_factor.rs
  └── gravity_alignment.rs
src/optimization/benchmarking.rs            (Phase 2C, ~600 LOC)
src/estimator/estimator.rs                  (Integration with estimator ~2000 LOC changes)
```

**Integration Points:**
- Core: `src/estimator/estimator.rs`
- Factor creation: All trackers must call pose graph APIs
- Visualization: Connect to rerun viewer
- Benchmarking: Performance profiling

**Risk Assessment:**
- High: Major architectural change
- Complex: Graph-based optimization requires careful debugging
- Testing: 45+ test cases already prepared
- Recommendation: Use git bisect if issues arise

**Benefits:**
- Full SLAM system (not just VIO)
- Loop closure capability
- Drift reduction over long trajectories
- Offline map building

**Dependencies:**
- IMU Fusion (Phase 5-8) should be integrated first for tight coupling
- Feature Tracker (already exists)

---

### **#4: Async Concurrent Pipeline** (`feature/async-pipeline`)
**Priority:** HIGH | **Duration:** 2-3 days | **Lines:** 3+ modules

**Core Files:**
```
src/estimator/async_feature_detection.rs    (Concurrent tracking, ~800 LOC)
src/estimator/async_optimization.rs         (AsyncOptimizer for BA, ~900 LOC)
src/estimator/async_wrapper.rs              (Integration layer, ~600 LOC)
src/estimator/concurrent.rs                 (Worker pool, rayon, ~500 LOC)
src/estimator/circuit_breaker.rs            (Failure isolation, ~400 LOC)
src/estimator/error_handling.rs             (Error recovery, ~300 LOC)
src/estimator/estimator.rs                  (Async integration, ~1000 LOC changes)
Cargo.toml                                  (Add tokio, rayon dependencies)
config/tum_vi_realtime_cpu.yaml             (Concurrency tuning)
```

**Integration Points:**
- `src/estimator/estimator.rs`: Main pipeline coordination
- `src/feature_tracker/mod.rs`: Parallel feature extraction
- `src/optimization/solver.rs`: Async solver execution
- Config system: Concurrency parameters

**Risk Assessment:**
- High: Async/await can introduce subtle bugs
- Testing: Load testing under various conditions
- Recommendation: Profile memory usage carefully
- CPU affinity: Consider for determinism

**Benefits:**
- Real-time capability on multi-core CPUs
- Better hardware utilization
- Responsive UI during heavy computation
- Graceful degradation under load

**Dependencies:**
- SLAM backend (optional but recommended for full benefits)
- Works independently but more powerful with SLAM

---

### **#5: Dataset Support** (`feature/dataset-support`)
**Priority:** HIGH | **Duration:** 1 day | **Lines:** 3,233

**Core Files:**
```
src/datasets/tum_vi.rs                      (TUM-VI format loader, ~600 LOC)
src/datasets/tum_vi_player.rs               (TUM-VI pipeline, ~700 LOC)
src/datasets/euroc_player.rs                (EuRoC loader, ~500 LOC)
src/datasets/fourseasons_player.rs          (4Seasons loader, ~400 LOC)
src/datasets/trajectory_eval.rs             (GT evaluation, ~400 LOC)
src/datasets/frame_processor_trait.rs       (Abstraction, ~300 LOC)
src/datasets/player_trait.rs                (Player interface, ~200 LOC)
src/bin/run_tum.rs                          (TUM-VI runner, ~150 LOC)
src/bin/run_euroc.rs                        (EuRoC runner, ~150 LOC)
src/bin/run_4seasons.rs                     (4Seasons runner, ~150 LOC)
scripts/download_tum_vi.sh                  (Automated download)
config/tum_vi.yaml                          (Tuned parameters)
config/euroc_vio.yaml                       (Tuned parameters)
```

**Integration Points:**
- `src/main.rs`: Dataset selection logic
- `src/estimator/estimator.rs`: Frame source abstraction
- Config system: Dataset-specific parameters
- Benchmark tools: CSV output for analysis

**Risk Assessment:**
- Low: Dataset loaders are isolated
- Can be tested independently
- No impact on core VIO

**Benefits:**
- Real-world validation on standard benchmarks
- Comparison with published results
- Community collaboration on improvements
- Easy addition of new datasets

**Dependencies:**
- None (self-contained)
- Beneficial to integrate early for testing

---

## Integration Complexity Matrix

```
╔═══════════════════════════════════╦══════════╦════════════╦═══════╗
║ Feature                           ║ Commits  ║ LOC Added  ║ Risk  ║
╠═══════════════════════════════════╬══════════╬════════════╬═══════╣
║ 1. IMU Fusion                     ║    56    ║   6,313    ║  MED  ║
║ 2. Calibration Framework          ║    20+   ║   6,012    ║  LOW  ║
║ 3. SLAM Backend & Pose Graph      ║    48    ║  23,330    ║ HIGH  ║
║ 4. Async Concurrent Pipeline      ║    14+   ║   3,800    ║ HIGH  ║
║ 5. Dataset Support                ║    30+   ║   3,233    ║  LOW  ║
║ 6. Dense Reconstruction           ║     8    ║   1,463    ║  MED  ║
║ 7. Multi-Drone Coordination       ║    10    ║   1,990    ║ HIGH  ║
║ 8. Optimization & CPU Tuning      ║    20+   ║  11,144    ║  LOW  ║
║ 9. Testing Infrastructure         ║    30+   ║   Variable ║  LOW  ║
║ 10. Visualization & Monitoring    ║    15+   ║   1,181    ║  LOW  ║
╚═══════════════════════════════════╩══════════╩════════════╩═══════╝
```

---

## Dependency Graph

```
┌─────────────────────────────────────────────────────────────┐
│                   FOUNDATION TIER                           │
│  ┌──────────────────────┐  ┌──────────────────────────┐    │
│  │   IMU Fusion         │  │  Calibration Framework   │    │
│  │  (6,313 LOC)         │  │   (6,012 LOC)            │    │
│  └──────────┬───────────┘  └──────────────┬───────────┘    │
└─────────────┼──────────────────────────────┼─────────────────┘
              │                              │
              └──────────────┬───────────────┘
                             │
┌─────────────────────────────┴─────────────────────────────────┐
│                    CORE TIER                                  │
│  ┌─────────────────────────────┐  ┌────────────────────────┐ │
│  │  SLAM Backend & Pose Graph  │  │ Async Concurrent       │ │
│  │      (23,330 LOC)           │◄─┤ Pipeline (3,800 LOC)   │ │
│  └──────────────┬──────────────┘  └────────────────────────┘ │
└──────────────────┼────────────────────────────────────────────┘
                   │
        ┌──────────┴──────────┬──────────┬──────────┐
        │                     │          │          │
┌───────▼──────┐    ┌────────▼────┐  ┌─▼────────┐ │
│   Dataset    │    │   Dense     │  │ Multi-   │ │
│  Support     │    │ Reconstruction  Drone     │ │
│ (3,233 LOC)  │    │ (1,463 LOC) │  │(1,990 LO)│ │
└──────────────┘    └─────────────┘  └──────────┘ │
                                                   │
              ┌──────────────────────────────┐     │
              │ Optimization & Tuning        │◄────┘
              │ Testing Infrastructure       │
              │ Visualization & Monitoring   │
              └──────────────────────────────┘
```

---

## Success Criteria for Each Feature

### IMU Fusion
- [ ] Preintegration error < 0.1% over 1-second windows
- [ ] Velocity estimation bias < 0.05 m/s
- [ ] All 56 existing tests pass
- [ ] TUM-VI trajectory RMSE < 2% improvement

### Calibration Framework
- [ ] Intrinsic calibration converges in < 50 frames
- [ ] Camera-IMU extrinsic alignment error < 0.5°
- [ ] Acceptance validator correctly gates poor calibrations
- [ ] Online calibration refines initial estimates

### SLAM Backend
- [ ] Loop closure detection with <10% false positive rate
- [ ] Pose graph optimization reduces 1 km trajectory drift by >50%
- [ ] All 48 test suites pass
- [ ] Marginalization maintains numerical stability

### Async Pipeline
- [ ] Feature detection latency < 50ms on 4-core CPU
- [ ] Bundle adjustment parallelizes to >3x speedup
- [ ] No memory leaks under sustained 30 FPS operation
- [ ] Graceful degradation under CPU saturation

### Dataset Support
- [ ] All 3 datasets (TUM-VI, EuRoC, 4Seasons) load correctly
- [ ] Ground truth trajectory evaluation matches published results
- [ ] Dataset download completes reliably
- [ ] CSV benchmark outputs parse cleanly

---

## Risk Mitigation Strategies

### High-Risk Features (SLAM, Async, Multi-Drone)
1. **Extensive Testing**
   - Unit tests: Already written (45+ for SLAM)
   - Integration tests: Add before merge
   - Regression tests: Compare against baseline

2. **Incremental Integration**
   - Start with skeleton/stub interfaces
   - Gradually enable features
   - Keep old code path available initially

3. **Code Review**
   - Mandatory review by 2+ maintainers
   - Focus on data flow and synchronization
   - Architecture review before implementation

4. **Performance Profiling**
   - Flamegraph analysis before/after
   - Memory profiling under load
   - CPU utilization monitoring

### Medium-Risk Features (Calibration, Dense, Multi-Drone)
1. **Focused Testing**
   - Happy path + edge cases
   - Input validation coverage
   - Example datasets for validation

2. **Documentation**
   - Usage examples
   - Configuration reference
   - Troubleshooting guide

### Low-Risk Features (Dataset, Optimization, Visualization)
1. **Standard Practice**
   - Code review
   - Unit test coverage
   - Documentation

---

## Recommended Timeline

```
Week 1:  IMU Fusion + Calibration Framework (Foundation)
         ├─ Day 1-2: IMU integration & testing
         └─ Day 3-4: Calibration integration & validation

Week 2:  SLAM Backend (Core architecture)
         ├─ Day 1-2: Pose graph integration
         ├─ Day 3: Visual factors
         ├─ Day 4: IMU factors
         └─ Day 5: Benchmarking & optimization

Week 3:  Async Pipeline (Parallelization)
         ├─ Day 1-2: Async foundation & feature detection
         ├─ Day 3: AsyncOptimizer implementation
         └─ Day 4-5: Testing & tuning

Week 4:  Extensions (Dataset + Dense + Multi-Drone)
         ├─ Day 1: Dataset support
         ├─ Day 2: Dense reconstruction
         ├─ Day 3-4: Multi-drone coordination
         └─ Day 5: Polish & testing

Week 5:  Integration & Release
         ├─ Day 1-2: Full system testing
         ├─ Day 3: Performance optimization
         ├─ Day 4: Documentation & examples
         └─ Day 5: Release preparation
```

---

## Conclusion

The `develop-old` branch represents a complete, well-tested implementation of major VIO enhancements across 10 distinct feature categories. The recommended extraction order prioritizes:

1. **Foundation**: IMU Fusion + Calibration (enables downstream features)
2. **Core**: SLAM Backend + Async Pipeline (major architectural additions)
3. **Extensions**: Dataset Support, Dense Reconstruction, Multi-Drone
4. **Polish**: Optimization, Testing, Visualization

Each feature is extractable as an independent branch due to careful modularization. Integration should proceed in the recommended order to manage complexity and dependency chains. Total estimated integration effort: **4-5 weeks** with comprehensive testing and validation.
