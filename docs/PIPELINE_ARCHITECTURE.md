# RS-VIO Pipeline Architecture

## Overview

RS-VIO (Rolling Shutter Visual-Inertial Odometry) is a Rust implementation of stereo visual-inertial odometry that processes synchronized stereo camera images with IMU measurements to estimate camera motion and build 3D maps.

## Pipeline Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           INPUT SOURCES                                      │
├───────────────────────────────┬─────────────────────────────────────────────┤
│     STEREO CAMERAS            │              IMU                             │
│   (30-60 FPS, 640×480)        │          (200 Hz typical)                    │
└───────────────┬───────────────┴─────────────────────┬───────────────────────┘
                │                                     │
                ▼                                     ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                        FRONT-END PROCESSING                                 ──────────────────────────────────────────── │
├─────────────────────────────────┤
│                                                                              │
│  ┌─────────────────────┐    ┌─────────────────────┐    ┌──────────────────┐ │
│  │   FEATURE TRACKER   │    │    IMU DENOISING    │    │  SIGNAL ANALYSIS │ │
│  │                     │    │                     │    │                  │ │
│  │ • FAST corners      │    │ • High-pass filter  │    │ • Motor state    │ │
│  │ • 52-pt patch LK    │    │ • Notch filters     │    │ • Harmonic decomp│ │
│  │ • Left-right check  │    │ • Low-pass filter   │    │ • Vibration det  │ │
│  │ • SIMD accelerated  │    │ • Spike rejection   │    │ • f0 confidence  │ │
│  │ • Adaptive frame    │    │ • Clipping weight   │    │                  │ │
│  │   skipping          │    │ • Motion modes      │    │                  │ │
│  └──────────┬──────────┘    └──────────┬──────────┘    └────────┬─────────┘ │
│             │                          │                          │           │
│             ▼                          ▼                          ▼           │
│  ┌──────────────────────────────────────────────────────────────────────┐   │
│  │              VISION-IMU FUSION & ADAPTIVE PROCESSING                  │   │
│  │                                                                      │   │
│  │  ┌──────────────────┐  ┌──────────────────┐  ┌────────────────────┐  │   │
│  │  │ IMU-aided        │  │ Motion-aware     │  │ Rolling shutter    │  │   │
│  │  │ feature tracking │  │ super resolution │  │ correction         │  │   │
│  │  │                  │  │                  │  │                    │  │   │
│  │  │ • IMU motion pred│  │ • Adaptive patch │  │ • Row timestamps   │  │   │
│  │  │ • Gyro integration│  │   size/iter     │  │ • Pose interp      │  │   │
│  │  │ • Velocity est   │  │ • Pyramid ref    │  │ • Bundle adjust    │  │   │
│  │  │ • Bias estimate  │  │ • Motion comp    │  │                    │  │   │
│  │  └──────────────────┘  └──────────────────┘  └────────────────────┘  │   │
│  └──────────────────────────────────────────────────────────────────────┘   │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                      SLIDING WINDOW ESTIMATOR                                │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  ┌──────────────────────────────────────────────────────────────────────┐   │
│  │                     STATE ESTIMATION                                  │   │
│  │                                                                      │   │
│  │  ┌───────────────┐  ┌───────────────┐  ┌─────────────────────────┐  │   │
│  │  │    PnP/RANSAC │  │   IMU         │  │     Keyframe            │  │   │
│  │  │   Pose Est    │  │  Preintegration│  │     Selection          │  │   │
│  │  │               │  │               │  │                         │  │   │
│  │  │ • Essential   │  │ • ΔR, Δv, Δp  │  │ • Translation thresh   │  │   │
│  │  │   matrix      │  │ • Covariance  │  │ • Rotation thresh      │  │   │
│  │  │ • 3D-2D correspondences │  │ • Jacobians │  │ • IMU-visual agree │  │   │
│  │  │ • Outlier rejection│  │ • Bias correction│                    │  │   │
│  │  └───────────────┘  └───────────────┘  └─────────────────────────┘  │   │
│  │                                                                      │   │
│  │  ┌──────────────────────────────────────────────────────────────┐   │   │
│  │  │              BUNDLE ADJUSTMENT (BA)                           │   │   │
│  │  │                                                               │   │   │
│  │  │  Minimize: Σ ||z_ij - h(T_i, P_j)||² + Σ ||IMU prior||²       │   │   │
│  │  │                                                               │   │   │
│  │  │  • 6DOF poses (T_W_Bi) per keyframe                           │   │   │
│  │  │  • 3DOF point positions (P_j)                                 │   │   │
│  │  │  • 6DOF IMU-camera extrinsics                                 │   │   │
│  │  │  • 6DOF velocity priors (from IMU)                            │   │   │
│  │  │  • 3DOF gyro bias, 3DOF accel bias                            │   │   │
│  │  │                                                               │   │   │
│  │  └──────────────────────────────────────────────────────────────┘   │   │
│  │                                                                      │   │
│  └──────────────────────────────────────────────────────────────────────┘   │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                        MARGINALIZATION & LOOP CLOSURE                        │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  ┌──────────────────────────────────────────────────────────────────────┐   │
│  │                  MARGINALIZATION MANAGER                              │   │
│  │                                                                      │   │
│  │  Oldest frame marginalized:                                          │   │
│  │  • Prior formed from marginalized states                             │   │
│  │  • FEJ (First-Estimate Jacobians) for consistency                   │   │
│  │  • Schur complement for efficiency                                   │   │
│  │                                                                      │   │
│  └──────────────────────────────────────────────────────────────────────┘   │
│                                                                              │
│  ┌──────────────────────────────────────────────────────────────────────┐   │
│  │                    LOOP CLOSURE DETECTION                             │   │
│  │                                                                      │   │
│  │  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────────┐   │   │
│  │  │   BoW Retriever │  │   Descriptor    │  │   PnP Verification  │   │   │
│  │  │                 │  │   Matching      │  │                     │   │   │
│  │  │ • ORB vocabulary│  │ • FLANN/L2 norm │  │ • RANSAC + EPnP     │   │   │
│  │  │ • DBow3 index   │  │ • Ratio test    │  │ • 6DOF pose match   │   │   │
│  │  │ • Temporal gap  │  │ • Hamming dist  │  │ • Information mat   │   │   │
│  │  └─────────────────┘  └─────────────────┘  └─────────────────────┘   │   │
│  │                                                                      │   │
│  └──────────────────────────────────────────────────────────────────────┘   │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                              OUTPUTS                                         │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────────────────┐ │
│  │  Trajectory     │  │   Sparse Map    │  │      Calibration Quality    │ │
│  │  T_W_B(t)       │  │   3D Points     │  │      Monitoring             │ │
│  │                 │  │                 │  │                             │ │
│  │ • SE(3) poses   │  │ • Triangulated  │  │ • Reprojection error       │ │
│  │ • Velocities    │  │   landmarks     │  │ • IMU noise estimates      │ │
│  │ • Covariances   │  │ • Track length  │  │ • Time offset drift       │ │
│  └─────────────────┘  └─────────────────┘  └─────────────────────────────┘ │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

## Key Processing Stages

### 1. IMU Preprocessing (`imu/`)

| Component | Purpose | Key Features |
|-----------|---------|--------------|
| `DenoiseFilter` | Raw IMU cleaning | HPF (0.5Hz), notch filters (0.06Hz, 1.46Hz), LPF (50Hz), spike rejection, motion-adaptive modes |
| `HigherOrderFilter` | Jerk/snap computation | Derivative filtering, f0 confidence weighting, spike detection |
| `VibrationFilter` | Resonance suppression | Per-axis notch filters, adaptive Q |
| `SignalAnalyzer` | Motor state detection | Harmonic decomposition, vibration analysis |
| `Preintegrator` | Motion integration | ΔR, Δv, Δp with Jacobians, bias correction |

### 2. Vision Processing (`vision/`)

| Component | Purpose | Key Features |
|-----------|---------|--------------|
| `StereoSuperResolver` | Subpixel disparity | Confidence-weighted refinement, pyramid levels, motion compensation |
| `SubpixelDisparity` | Patch matching | Image pyramid, Gauss-Newton optimization |
| `IMUAidedTracker` | Feature tracking | Gyro motion prediction, velocity integration |
| `RollingShutterCorrection` | RS compensation | Row timestamps, pose interpolation, BA |
| `AdaptiveFusion` | Multi-sensor fusion | Quality-based weighting, depth optimization |

### 3. Feature Tracking (`feature_tracker/`)

- **Detection**: FAST corners with grid-based distribution
- **Tracking**: 52-point patch pattern, Lucas-Kanade optimization
- **Stereo Matching**: Left-right consistency check (epipolar constraint)
- **Optimization**: SIMD-accelerated patch residuals (AVX2/SSE4.1)
- **Adaptive**: Frame skipping under load

### 4. State Estimation (`estimator/`)

- **Sliding Window**: Fixed-size window of keyframes
- **IMU Preintegration**: Efficient relative motion computation
- **Bundle Adjustment**: Nonlinear optimization of poses + points
- **IMU Prior**: Motion constraints from preintegrated IMU
- **Marginalization**: First-estimate Jacobian consistency

### 5. Loop Closure (`optimization/loop_closure/`)

- **BoW Retrieval**: ORB vocabulary with DBow3 indexing
- **Descriptor Matching**: FLANN-based, ratio test, Hamming distance
- **PnP Verification**: RANSAC + EPnP with essential matrix check
- **Enhanced Verifier**: Geometric consistency, information matrix

### 6. Calibration (`calibration/`)

| Module | Purpose |
|--------|---------|
| `CameraIntrinsics` | Pinhole, EUCM, OpenCV models |
| `StereoExtrinsics` | Baseline, rotation between cameras |
| `ImuIntrinsics` | Noise densities, bias random walks |
| `CameraImuExtrinsics` | T_BC transformation |
| `OnlineTimeOffset` | Camera-IMU time drift |
| `RollingShutter` | Readout time estimation |
| `UnifiedSolver` | Joint optimization |

## Configuration

### IMU Parameters
```yaml
imu:
  gyro_noise_density: 1.0e-4      # rad/s/√Hz
  accel_noise_density: 1.0e-2     # m/s²/√Hz
  gyro_bias_random_walk: 1.0e-5   # rad/s²/√Hz
  accel_bias_random_walk: 1.0e-4  # m/s³/√Hz
  gravity: [0.0, 0.0, -9.81]      # m/s²
```

### Denoise Filter
```yaml
denoise:
  imu_sample_rate: 200.0          # Hz
  camera_frame_rate: 30.0         # Hz
  highpass_cutoff: 0.5           # Hz
  lowpass_cutoff: 50.0           # Hz
  enable_notch_filter: true
  notch_frequencies: [0.06, 1.46] # Hz
  adaptive_notch_q: true
  motion_modes:
    hover_rms_thresh: 0.25        # rad/s
    aggressive_rms_thresh: 0.8    # rad/s
```

### Feature Tracker
```yaml
tracking:
  max_features: 500
  grid_rows: 5
  grid_cols: 5
  min_distance: 10.0              # pixels
  pyramid_levels: 3
  patch_size: 31
  iterations: 30
  termination_threshold: 0.01
```

## Performance Targets

| Metric | Target | Notes |
|--------|--------|-------|
| Frame latency | <33ms | @ 30 FPS |
| Feature detection | 5-20ms | Depends on grid size |
| Feature tracking | 10-30ms | Typical motion |
| Bundle adjustment | 10-50ms | 20 keyframes |
| Memory (window) | <50MB | 50 frames, 500 features |

## Supported Datasets

- **EuRoC MAV**: Vicon room sequences, ASMLab sequences
- **4Seasons**: Outdoor driving, seasonal changes
- **TUM-VI**: Handheld, indoor/outdoor

## References

1. Forster et al., "IMU Preintegration on Manifold", RSS 2017
2. Lupton & Sukkarieh, "Visual-Inertial-Aided Navigation", 2011
3. Mur-Artal et al., "ORB-SLAM2", 2017
4. Qin et al., "VINS-Mono", 2018
5. Geier et al., "Shift-and-add registration", 2013
