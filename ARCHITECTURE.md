# RS-VIO Architecture Guide

This document describes the high-level architecture, module organization, and design decisions for the RS-VIO project.

## Table of Contents

1. [System Overview](#system-overview)
2. [Module Organization](#module-organization)
3. [Data Flow](#data-flow)
4. [Key Algorithms](#key-algorithms)
5. [Design Patterns](#design-patterns)
6. [Performance Considerations](#performance-considerations)
7. [Safety & Validation](#safety--validation)
8. [API Reference](#api-reference)

## System Overview

RS-VIO is a stereo visual-inertial odometry (VIO) system that estimates camera pose and builds 3D maps from stereo image sequences. The system operates in real-time with deterministic performance suitable for embedded systems.

### High-Level Pipeline

```
Input: Stereo Image Pair (left, right)
          ↓
    ┌─────────────────────────────┐
    │   Feature Detection & Track │  (feature_tracker module)
    │   - FAST corner detection   │
    │   - Lucas-Kanade optical    │
    │   - Stereo matching         │
    └─────────────────────────────┘
          ↓
    ┌─────────────────────────────┐
    │   3D Point Triangulation    │  (estimator module)
    │   - Depth computation       │
    │   - Map point creation      │
    └─────────────────────────────┘
          ↓
    ┌─────────────────────────────┐
    │   Pose Estimation           │  (estimator module)
    │   - PnP solver              │
    │   - Motion model            │
    └─────────────────────────────┘
          ↓
    ┌─────────────────────────────┐
    │   Keyframe Management       │  (estimator::sliding_window)
    │   - Keyframe selection      │
    │   - Window maintenance      │
    └─────────────────────────────┘
          ↓
    ┌─────────────────────────────┐
    │   Bundle Adjustment         │  (optimization module)
    │   - LM optimization         │
    │   - Pose refinement         │
    │   - Landmark refinement     │
    └─────────────────────────────┘
          ↓
    Output: Camera Pose + Map Points
```

## Module Organization

### `src/lib.rs` - Library Root
Provides public API and module re-exports.

### `src/types.rs` - Float Precision Configuration
Configurable floating-point types (f32/f64) for embedded flexibility.

**Key Types:**
- `Float` - Configurable via `use_f32` feature
- `Vector2/3`, `Matrix3/4` - Linear algebra types
- `Pose`, `Transform` - SE(3) representations

### `src/feature_tracker/` - Feature Detection & Tracking

**Modules:**
- `feature_tracker.rs` - Main tracker and stereo matching
- `patch.rs` - 52-point pattern for optical flow
- `image_utilities.rs` - Image pyramid and utility functions

**Key Types:**
- `Feature` - Individual detected feature with ID and coordinates
- `Pattern52` - 52-point patch for Lucas-Kanade tracking
- `StereoPatchTracker` - Stereo feature tracking

**Algorithm:**
```
Frame N → Feature Detection (FAST)
       ↓
       Optical Flow Tracking (52-point LK)
       ↓
       Left-Right Matching (stereo)
       ↓
       Valid Tracked Features
       ↓
Frame N+1 (continues tracking previous features)
```

**Complexity:** O(grid_size² × patch_size × iterations) ≈ 10-30ms

### `src/estimator/` - VIO Pipeline

**Modules:**
- `estimator.rs` - Main VIO pipeline orchestrator
- `frame.rs` - Frame representation and feature storage
- `state.rs` - System state (poses and landmarks)
- `sliding_window.rs` - Keyframe management and optimization

**Pipeline Architecture:**

1. **Frame Processing** (`estimator.rs`)
   - Receives stereo pair
   - Initializes frame representation
   - Orchestrates feature tracking

2. **Feature Association** (`frame.rs`)
   - Stores features from both cameras
   - Manages feature-landmark associations
   - Handles feature visibility

3. **State Management** (`state.rs`)
   - Maintains camera poses (history)
   - Maintains 3D map points
   - Provides state access for optimization

4. **Sliding Window** (`sliding_window.rs`)
   - Manages keyframe selection (translation/rotation thresholds)
   - Maintains fixed-size optimization window
   - Triggers bundle adjustment

**Key Types:**
- `Estimator` - Main pipeline orchestrator
- `Frame` - Frame with left/right features
- `State` - System state container
- `SlidingWindow` - Keyframe management

### `src/optimization/` - Bundle Adjustment

**Modules:**
- `factors.rs` - Reprojection error factors
- `observer.rs` - Optimization diagnostics
- `tests.rs` - Optimization verification

**Factors:**

1. **PinholeProjectionFactor**
   - Reprojection error: `||proj(T_C_W · p_W) - obs||²`
   - Variables: 3D point position (3 DOF)
   - Parameters: Camera pose, observation (fixed)
   - Use: Optimize landmarks with fixed poses

2. **BundleAdjustmentFactor**
   - Full multi-view bundle adjustment
   - Variables: Camera poses (6 DOF via SE(3) tangent)
   - Parameters: Landmarks, observations (fixed)
   - Use: Simultaneous pose and landmark optimization

**Solver:** Levenberg-Marquardt with sparse Cholesky

**Complexity:** O(window_size³ + n_landmarks) ≈ 1-10ms

### `src/datasets/` - Data Loading & Configuration

**Modules:**
- `config.rs` - YAML configuration loader
- `euroc_player.rs` - EuRoC dataset player
- `tum_vi_player.rs` - TUM-VI dataset player
- `fourseasons_player.rs` - 4Seasons dataset player

**Data Structures:**
- `Config` - System configuration from YAML
- `ImageData` - Timestamped image with path
- `ImuData` - IMU measurements (future)
- `FrameContext` - Processing state
- `PlayerResult` - Execution statistics

### `src/viewers/` - Visualization

**Modules:**
- `viewer.rs` - Visualization trait
- `rerun.rs` - Rerun.io backend

**Features:**
- 3D camera trajectory visualization
- 3D landmark point clouds
- Camera frustum visualization
- Feature tracking overlay
- Real-time streaming

### `src/validation.rs` - Numerical Safety

**Utilities:**
- `validate_vector()` - Check for NaN/Inf
- `validate_matrix()` - Check matrix validity
- `validate_point()` - Check depth ranges
- `validate_rotation()` - Check orthonormality
- `validate_se3()` - Check SE(3) transform validity
- `safe_divide()` - Protected division
- `float_eq()` - Epsilon comparison

**Depth Ranges:**
- Minimum: 0.1m (prevents near-plane issues)
- Maximum: 1000m (prevents overflow)

## Data Flow

### Frame Processing Flow

```
Input Stereo Pair
      ↓
FeatureTracker.process_frame()
      ├─ Detect features (FAST)
      ├─ Track features (Lucas-Kanade)
      └─ Stereo match (left-right consistency)
      ↓
Estimator.process_frame()
      ├─ Create Frame
      ├─ Add tracked features
      ├─ Estimate pose (PnP/motion model)
      └─ Triangulate new points
      ↓
SlidingWindow.add_frame()
      ├─ Check keyframe criteria
      ├─ Add to optimization window
      └─ Remove old frames if needed
      ↓
SlidingWindow.optimize()
      ├─ Build factors
      ├─ Run LM solver
      └─ Update poses & landmarks
      ↓
Output: Pose + Map
```

### State Lifecycle

```
Frame N:   [Pose_1, ..., Pose_N]
           [Points_1, ..., Points_M]

Frame N+1: [Pose_2, ..., Pose_N+1]  (oldest pose removed)
           [Points_1, ..., Points_M, Points_new]  (new points added)

Optimization: Refine [Pose_2..N+1] and [Points] via BA
```

## Key Algorithms

### Feature Detection (FAST)
- Detects fast-moving corners
- Grid-based distribution for uniform coverage
- Multi-pyramid levels for scale invariance
- Time: O(width × height) ≈ 2-5ms

### Lucas-Kanade Optical Flow
- 52-point pattern for tracking
- Iterative refinement (typically 3-5 iterations)
- Converges when gradient < threshold
- Time: O(n_features × 52 × iterations) ≈ 8-20ms

### Stereo Matching
- Left-right consistency check
- Epipolar constraint enforcement
- Disparity computation via baseline
- Time: O(n_features) ≈ 1-2ms

### PnP Pose Estimation
- Perspective-n-Point solver
- Uses 3D-2D correspondences
- Motion model refinement
- Time: O(n_features) ≈ 3-5ms

### Bundle Adjustment
- Levenberg-Marquardt optimization
- Sparse Cholesky linear solver
- SE(3) manifold representation
- Time: O(window_size³) ≈ 1-10ms

## Design Patterns

### 1. Separation of Concerns
Each module handles one aspect:
- **feature_tracker**: Pixel-level feature tracking
- **estimator**: Pose and map estimation
- **optimization**: Bundle adjustment
- **datasets**: Data loading
- **viewers**: Visualization

### 2. Type Safety
- No `unsafe` code (enforced by `unsafe_code = forbid`)
- Compile-time feature flags for f32/f64
- Configuration validation at load time
- Result types for error handling

### 3. Real-Time Predictability
- Deterministic sequential processing (no parallelism)
- Fixed-size sliding window (bounded memory)
- Timeout-based iteration limits
- No dynamic allocations in hot paths

### 4. Configuration Management
- YAML-based configuration
- Loaded once at startup
- Shared ownership via references
- Viewer ownership semantics

## Performance Considerations

### Computational Budget (640×480 @ 30 FPS = 33ms)

| Component | Budget | Typical |
|-----------|--------|---------|
| Feature Detection | 10ms | 5-8ms |
| Optical Flow | 15ms | 10-15ms |
| Pose Estimation | 5ms | 3-5ms |
| Optimization | 15ms | 1-10ms |
| **Total** | **45ms** | **20-40ms** |

Notes:
- Budget exceeds 33ms due to I-frame (high feature count)
- Normal frames: 20-30ms
- 60 FPS target requires aggressive optimization (not yet met)

### Memory Considerations

**Per-Frame Memory:**
- Left image: 640×480 = 307KB
- Right image: 640×480 = 307KB
- Image pyramids: ~614KB
- Features (200/frame): 8KB
- Total per frame: ~1.2MB

**Sliding Window (5 frames):**
- Poses: 5 × 128B = 640B
- Landmarks (500): 500 × 24B = 12KB
- Feature associations: ~10KB
- Total: ~40KB state
- With images: ~6MB peak

**Memory Growth:** Linear until window fills, then constant.

### Cache Efficiency

**Hot Path Operations:**
1. Optical flow (sequential pixel access)
   - Row-major image layout ✓
   - Image pyramid hierarchy ✓

2. Bundle adjustment
   - Sparse Hessian structure ✓
   - Batch factor evaluation ✓

3. Feature tracking
   - Contiguous feature storage ✓
   - HashMap for ID lookups ✗ (acceptable overhead)

## Safety & Validation

### Numerical Safety

All VIO computations validated:
- **Depth checks**: 0.1m ≤ z ≤ 1000m
- **Rotation validation**: Orthonormality check
- **Transform validation**: SE(3) consistency
- **Division safety**: Epsilon checks before division
- **Finite checks**: NaN/Inf detection and rejection

### Error Handling

- No unwrap() in production code
- Result types propagate errors
- Graceful degradation (skip invalid features)
- Detailed error context for debugging

### Safe Code Guarantees

```toml
[lints.rust]
unsafe_code = "forbid"
expect_used = "deny"
unwrap_used = "deny"
panic = "deny"
```

## API Reference

### Main Entry Points

**Library Usage:**
```rust
use rs_vio::{Estimator, datasets::Config};

let config = Config::load("config.yaml")?;
let mut estimator = Estimator::new(config, None);
estimator.process_frame(&left_img, &right_img);
```

**Dataset Playback:**
```rust
use rs_vio::datasets::{EurocPlayer, PlayerConfig};

let player = EurocPlayer;
let result = player.run(config)?;
```

### Key Type Signatures

**Estimator Pipeline:**
```rust
impl Estimator {
    pub fn new(config: Config, viewer: Option<Box<dyn Viewer>>) -> Self
    pub fn process_frame(&mut self, left: &GrayImage, right: &GrayImage)
    pub fn get_pose(&self) -> Option<SE3>
    pub fn get_map_points(&self) -> Vec<Vector3<Float>>
}
```

**Feature Tracker:**
```rust
impl<const LEVELS: u32> StereoPatchTracker<LEVELS> {
    pub fn new(grid_size: u32, max_iters: u32, threshold: f64) -> Self
    pub fn process_frame(&mut self, left: &GrayImage, right: &GrayImage, frame: &mut Frame)
    pub fn get_left_track_points(&self) -> HashMap<usize, (f32, f32)>
    pub fn get_right_track_points(&self) -> HashMap<usize, (f32, f32)>
}
```

### Configuration Structure

```yaml
camera:
  image_width: 640
  image_height: 480
  left_intrinsics: [fx, fy, cx, cy]
  left_distortion: [k1, k2, p1, p2]
  left_model: "pinhole-radtan" | "eucm"
  right_intrinsics: [fx, fy, cx, cy]
  right_distortion: [k1, k2, p1, p2]
  right_model: "pinhole-radtan" | "eucm"
  T_B_Cl: [4x4 matrix]  # Body to left camera
  T_B_Cr: [4x4 matrix]  # Body to right camera

keyframe_management:
  keyframe_window_size: 5
  translation_threshold: 0.05
  rotation_threshold: 0.1

feature_detection:
  grid_size: 15
  max_features: 200

optimization:
  max_iterations: 50
  convergence_threshold: 0.01
  linear_solver: "sparse_cholesky"
```

## Further Reading

- [BENCHMARKING.md](BENCHMARKING.md) - Performance testing guide
- [CONTRIBUTING.md](CONTRIBUTING.md) - Development setup
- [src/lib.rs](src/lib.rs) - Module documentation
- Generated API docs: `cargo doc --open`

