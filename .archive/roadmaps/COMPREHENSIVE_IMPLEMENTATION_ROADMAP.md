# RS-VIO Comprehensive Implementation Roadmap

**Based on:** Design document for best practical SOTA stack for onboard realtime drone
**Date:** Current Session
**Status:** Planning & Implementation Phase

## Overview

This roadmap translates the design recommendations into a concrete implementation plan for RS-VIO. It covers:
- Multi-frame geometric fusion approaches
- Feature detection and tracking strategies
- Auto-calibration system (manual-guided)
- Compute-aware pipeline selection

---

## Phase 1: Feature Detection & Tracking Module ⏳

### 1.1 Traits & Abstractions

**File:** `src/feature_detection/mod.rs`

```rust
pub trait KeypointDetector: Send + Sync {
    fn detect(&self, image: &[u8], width: u32, height: u32) -> Vec<Keypoint>;
    fn name(&self) -> &str;
}

pub trait Descriptor: Send + Sync {
    fn compute(&self, image: &[u8], keypoints: &[Keypoint]) -> Vec<DescriptorData>;
    fn distance(&self, desc1: &DescriptorData, desc2: &DescriptorData) -> u32;
    fn name(&self) -> &str;
}

pub trait FeatureTracker: Send + Sync {
    fn track(&mut self, frame: &Frame, prev_frame: &Frame) -> Vec<FeatureTrack>;
    fn reset(&mut self);
}
```

### 1.2 Classical Detectors

**Shi-Tomasi (GFTT) + Pyramidal KLT**
- File: `src/feature_detection/gftt_klt.rs`
- Grid-based spatial distribution
- Track-first, detect-to-fill strategy
- Pyramid levels for scale robustness
- Uncertainty from tracking residuals

**FAST/AGAST Corners**
- File: `src/feature_detection/fast_corners.rs`
- High-speed corner detection
- Non-maximum suppression
- Optional descriptor pairing

**ORB (ORiented BRIEF)**
- File: `src/feature_detection/orb_descriptor.rs`
- Binary descriptors for fast matching
- Rotation invariance
- Bag-of-Words integration for loop closure

**AKAZE**
- File: `src/feature_detection/akaze.rs`
- MLDB binary descriptors
- Scale-space pyramid
- Blur/rotation robustness

### 1.3 Learned Detectors (Placeholder for GPU)

**SuperPoint Integration**
- File: `src/feature_detection/superpoint_integration.rs`
- GPU-based (requires ONNX Runtime or TensorRT)
- Keypoint + descriptor in one pass
- TensorRT optimization for Jetson
- Runtime selection via config

**DISK / R2D2** (Optional future)
- Placeholder interface
- Deferred implementation

### 1.4 Configuration & Selection

**File:** `src/feature_detection/config.rs`

```yaml
feature_detection:
  detector: gftt          # or: fast, orb, akaze, superpoint
  tracker: klt            # or: optical-flow
  descriptor: none        # or: orb, akaze, superpoint

  gftt:
    quality_level: 0.01
    min_distance: 10
    grid_size: 30
    max_per_grid: 200

  superpoint:
    enabled: false
    model_path: models/superpoint.onnx
    confidence_threshold: 0.015
    nms_radius: 4
    max_keypoints: 1000

  tracking:
    pyramid_levels: 4
    window_size: 15
    max_iterations: 30
    convergence_threshold: 0.001
```

---

## Phase 2: Enhanced Fusion Module ⏳

### 2.1 Additional Geometric Fusion Approaches

**1. SE(3) Warp with Planar Model**
- File: `src/fusion/se3_planar_fusion.rs`
- IMU-propagated pose + single plane assumption
- Homography-like warp
- Cost: cheap, works well at range
- Limitation: parallax breaks it

**2. Full Depth-Aware Fusion (Two-Pass)**
- File: `src/fusion/depth_aware_two_pass.rs`
- Pass A: coarse stereo/sparse depth
- Pass B: per-pixel warp using depth + pose
- Occlusion-aware fusion
- Optional re-run stereo with finer params

**3. Plane-Segmented Fusion**
- File: `src/fusion/plane_segmented_fusion.rs`
- RANSAC plane detection from sparse depth
- Per-plane homography fusion
- Excellent in urban scenes
- Robust to moving objects (plane-masked)

**4. EPI-Focused Accumulation**
- File: `src/fusion/epi_cost_volume_fusion.rs`
- Multi-frame cost volume accumulation
- Direct disparity precision improvement
- Less fake-corner artifacts
- More memory, but very effective

**5. Keyframe Mosaic / Super-Sampled Map**
- File: `src/fusion/keyframe_mosaic.rs`
- Higher-res stabilized mosaic per keyframe
- Detect features on "reference" mosaic
- High repeatability
- Asynchronous (amortizes cost over time)

### 2.2 Fusion Strategy Enum

```rust
pub enum FusionStrategy {
    None,
    RotationOnly,              // Current (fast, stable)
    SE3Planar,                 // New (cheap, range-limited)
    DepthAwarePatches,         // Current (best compute/benefit)
    DepthAwareFull,            // New (best quality)
    PlaneSegmented,            // New (urban scenes)
    EPICostVolume,             // New (disparity precision)
    KeyframeMosaic,            // New (asynchronous)
}
```

### 2.3 Fusion Configuration

```yaml
fusion:
  strategy: rotation-only      # or: se3-planar, depth-patches, depth-full, plane-segmented, epi-volume, keyframe-mosaic

  rotation_only:
    num_frames: 3
    weighting_strategy: exponential

  se3_planar:
    num_frames: 4
    plane_distance: 10.0       # Assume scene at 10m

  depth_aware_full:
    two_pass: true
    num_frames: 5
    occlusion_handling: true

  plane_segmented:
    ransac_iterations: 1000
    inlier_threshold: 0.1
    min_plane_size: 100

  keyframe_mosaic:
    mosaic_scale: 2.0          # 2x resolution
    detect_on_mosaic: true
```

---

## Phase 3: Auto-Calibration System ⏳

### 3.1 Manual-Guided Calibration Framework

**File:** `src/calibration/mod.rs`

```rust
pub trait ManualCalibrationGuide: Send + Sync {
    fn name(&self) -> &str;
    fn description(&self) -> &str;
    fn motion_cue(&self) -> &str;           // "Rotate on X axis slowly..."
    fn expected_duration_sec(&self) -> u32;
    fn validate_capture(&self, frames: &[Frame]) -> CalibrationResult;
    fn solve(&self, frames: &[Frame]) -> CalibrationSolution;
}
```

### 3.2 Individual Calibration Modules

**IMU Self-Calibration (Static)**
- File: `src/calibration/imu_static.rs`
- 6 face-down orientations (or 3-axis rotations)
- Estimates: bias, scale, cross-axis factors
- Detects clipping/saturation history
- Output: bias, scale matrix, noise density, health flags

**Camera Intrinsics (Monocular BA)**
- File: `src/calibration/camera_intrinsics.rs`
- Operator moves in front of textured scene
- Solves: f, cx/cy, distortion, rolling-shutter readout
- Regularization prevents drift
- Output: K matrix, distortion, rolling-shutter time, RMS, uncertainty

**Camera-IMU Time Offset**
- File: `src/calibration/camera_imu_sync.rs`
- Slow pan motion with steady angular velocity
- Cross-correlates optical flow vs gyro signal
- Rolls over time-offset range, finds peak
- Refines with BA
- Output: offset (ms), uncertainty, quality score

**Stereo Extrinsics**
- File: `src/calibration/stereo_extrinsics.rs`
- Figure-8 or small translation motions
- Epipolar geometry optimization
- Validates with inlier ratio, angular error
- Output: relative pose, baseline, geometry metrics

**Rolling vs Global Shutter**
- File: `src/calibration/shutter_type.rs`
- Confirmed during intrinsics solve
- Checks for row-skew artifacts in epipolar error
- Output: shutter type, readout time (rolling), confidence

**Multi-Camera / Heterogeneous Rigs**
- File: `src/calibration/multicamera.rs`
- Per-camera intrinsics independently
- Per-pair extrinsics independently
- Optional: joint graph over all cameras + IMU
- Output: all per-camera K, all per-pair poses, consistency metrics

### 3.3 Quality Gates & Acceptance

```rust
pub struct CalibrationQualityGates {
    pub max_reprojection_rms: f32,      // e.g., 0.3 px global, 0.5 px rolling
    pub min_epipolar_inlier_ratio: f32, // e.g., 0.95
    pub max_epipolar_error: f32,        // e.g., 1.0 px
    pub max_baseline_uncertainty: f32,  // e.g., 0.05 (5%)
    pub max_time_offset_uncertainty: f32, // e.g., 0.002 (2 ms)
    pub max_consistency_divergence: f32,  // e.g., 0.1 (10%)
    pub allow_manual_override: bool,
}
```

### 3.4 Operational Workflow

**Ground Mode:**
- File: `src/calibration/ground_mode.rs`
- CLI: `rs-vio calibrate-imu`, `calibrate-cameras`, `calibrate-stereo`
- Guided motion prompts with visual feedback
- Real-time quality monitoring
- Accept/reject with operator override option

**Pre-Flight Self-Check:**
- File: `src/calibration/preflight_check.rs`
- Load last-good calibration
- Bias creep detection
- Gross geometry sanity check
- Warn if quality confidence below threshold

**In-Flight Monitoring** (no adjustment):
- File: `src/calibration/flight_monitor.rs`
- Log anomalies only
- Track bias drift trend
- Monitor epipolar error trend
- Flag photometric residual spikes

**Persistence & Versioning:**
- File: `src/calibration/persistence.rs`
- Timestamp, sensor SN, mount ID, platform ID
- Versioned: `calibration_v1.0.yaml`, `v1.1.yaml`
- SHA256 integrity check on load
- Multiple profiles (pre-crash, post-repair, etc.)

---

## Phase 4: Compute-Aware Pipeline Configuration ⏳

### 4.1 Platform Profiles

**File:** `src/pipeline/platform_profiles.rs`

```yaml
platform_profiles:

  cpu_only:  # Raspberry Pi 5, small ARM
    detector: gftt
    tracker: klt
    descriptor: orb        # Only on keyframes
    fusion: rotation_only
    loop_closure: orb-bow
    target_fps: 15
    target_resolution: 640x480

  jetson:    # Jetson Nano/Xavier
    detector: gftt         # KLT fast, SuperPoint on keyframes
    tracker: klt
    descriptor: superpoint # On keyframes only
    fusion: rotation_only + selective_depth_patches
    loop_closure: superpoint+lightglue
    target_fps: 30
    target_resolution: 1280x720

  tiny_budget:  # Hard realtime, minimal compute
    detector: fast
    tracker: klt
    descriptor: none
    fusion: rotation_only
    loop_closure: patch_matching (keyframes only)
    target_fps: 20
    target_resolution: 320x240

  offline:   # Server, no constraint
    detector: superpoint
    tracker: klt
    descriptor: superpoint
    fusion: depth_aware_full + plane_segmented
    loop_closure: superpoint+lightglue
    target_fps: unlimited
    target_resolution: 1920x1080
```

### 4.2 Adaptive Parameter Tuning

**File:** `src/pipeline/adaptive_tuning.rs`

Based on observed performance:
- Adjust N (frames to fuse) to stay within budget
- Reduce feature count if frame drops
- Switch fusion strategy if latency exceeds budget
- Downsample resolution if necessary
- Gate heavy modules (SuperPoint/LightGlue) to keyframes

**Monitoring:**
- Per-frame execution time breakdown
- Feature count trends
- Tracking success rate
- Pose estimation uncertainty
- Sensor health metrics

---

## Phase 5: Integration & Deployment Path

### 5.1 Configuration-Based Pipeline

```yaml
# config/prod_onboard_drone.yaml
platform: jetson

feature_detection:
  detector: gftt
  tracking:
    pyramid_levels: 4
    max_features: 500

fusion:
  strategy: rotation_only
  num_frames: 3
  weighting: exponential

loop_closure:
  enabled: true
  module: superpoint+lightglue  # Keyframes only
  trigger_interval: 50          # Every 50 frames

calibration:
  manual_only: true
  preflight_check: true
  monitor_flight: true
  allow_manual_override: false

deployment:
  check_calibration_age: 10     # Warn if >10 flights old
  check_thermal_shock: true
  check_sensor_health: true
```

### 5.2 Deployment Checklist

```rust
pub struct DeploymentChecklist {
    pub calibration_recent: bool,
    pub quality_gates_passed: bool,
    pub operator_override_documented: bool,
    pub temperature_similar: bool,
    pub sensor_health_ok: bool,
    pub consistency_check_ok: bool,
    pub ready_to_arm: bool,
}
```

---

## Phase 6: Documentation & Examples

### 6.1 Architecture Guides

- **Feature_Detection_Guide.md** - Strategy selection, tracking, uncertainty
- **Fusion_Strategies_Reference.md** - All 7 approaches, use cases, costs
- **AutoCalibration_Manual.md** - Ground-mode operation, quality gates, troubleshooting
- **Platform_Tuning_Guide.md** - CPU/Jetson/embedded profiles, FPS tuning
- **Deployment_Checklist.md** - Pre-flight, in-flight, post-flight workflows

### 6.2 Configuration Examples

- `config/raspberry_pi_5.yaml` - CPU-only, minimal compute
- `config/jetson_xavier.yaml` - GPU-accelerated, good compute
- `config/offline_processing.yaml` - No constraint, max quality
- `config/hard_realtime.yaml` - Minimal latency, all features

---

## Implementation Priority

### ✅ Completed (Phase 1 Partial)
- Fusion module foundation (RotationStabilizer, DepthAwareFusion)
- Fusion configuration framework

### ⏳ Priority 1 (This Session)
1. Feature detection module (traits, GFTT/KLT, FAST, ORB)
2. Auto-calibration system (ground-mode framework)
3. Fusion enum expansion to 7 approaches

### ⏳ Priority 2 (Next Session)
1. SE(3) planar fusion implementation
2. Plane-segmented fusion
3. Keyframe mosaic approach
4. Calibration implementations (IMU, intrinsics, stereo)

### ⏳ Priority 3 (Future)
1. SuperPoint integration (GPU placeholder)
2. EPI cost-volume fusion
3. Adaptive pipeline tuning
4. Platform profiles with runtime selection
5. Deployment checklist automation

---

## Success Criteria

| Goal | Target | Status |
|------|--------|--------|
| Feature detection module | Pluggable traits + 4 detectors | ⏳ Phase 1 |
| Fusion strategies | 7 approaches implemented | ⏳ Phase 2 |
| Auto-calibration | Ground-mode + quality gates | ⏳ Phase 3 |
| Platform profiles | CPU, Jetson, embedded, offline | ⏳ Phase 4 |
| Documentation | Comprehensive guides + examples | ⏳ Phase 6 |
| Tests | Unit + integration for all modules | ⏳ Ongoing |
| Code quality | Zero unsafe, full error handling | ⏳ Ongoing |

---

## Estimated Timeline

| Phase | Components | Estimated Time | Status |
|-------|-----------|-----------------|--------|
| 1 | Feature detection (traits + 4 detectors) | 2-3 hours | ⏳ |
| 2 | Fusion strategies (7 approaches) | 4-5 hours | ⏳ |
| 3 | Auto-calibration (framework + modules) | 3-4 hours | ⏳ |
| 4 | Compute-aware pipeline | 2-3 hours | ⏳ |
| 5 | Integration & deployment | 1-2 hours | ⏳ |
| 6 | Documentation | 2-3 hours | ⏳ |
| **Total** | **Full implementation** | **14-20 hours** | ⏳ |

---

## Next Steps

1. Start Phase 1: Feature detection module
2. Start Phase 3: Auto-calibration framework
3. Expand Phase 2: Fusion strategies to 7 approaches
4. Create configuration examples for each platform
5. Write comprehensive guides for deployment

---

**This roadmap translates theory into practice. Each phase is modular and independent. Start with feature detection (Phase 1) to enable better tracking, then expand fusion (Phase 2) for quality, then add auto-calibration (Phase 3) for operational resilience.**

**Goal: A production-ready, deployment-aware VIO system that can adapt to different hardware (Raspberry Pi → Jetson) and operational modes (real-time onboard → offline processing).**
