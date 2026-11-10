# MSCKF Architecture for RS-VIO

## Overview

This document describes the MSCKF (Multi-State Constraint Kalman Filter) architecture for RS-VIO, designed for embedded platforms like Raspberry Pi 5.

## Why MSCKF?

| Aspect | Plain EKF | MSCKF | Full BA |
|--------|-----------|-------|---------|
| Multi-frame geometry | ❌ | ✅ | ✅ |
| State size | 15 + landmarks | 15 + 6N clones | 6N + 3M |
| CPU (Pi 5) | ~5ms | ~15-25ms | ~50-100ms |
| Robustness | Medium | High | Highest |

**MSCKF is the sweet spot** for embedded multi-vehicle VIO.

## Architecture

```
IMU (200Hz) → Propagate state + covariance
                    ↓
Stereo (20-30Hz) → Track features → Augment clone
                        ↓
                Select mature tracks (12-24)
                        ↓
                Triangulate → Residualize
                        ↓
                Nullspace projection (eliminate landmarks)
                        ↓
                Platform-aware gating
                        ↓
                EKF update → Prune clones
```

## Core State

```rust
// 15 DOF IMU core state
pub struct ImuState {
    pub q_wb: UnitQuaternion<f32>,  // Orientation
    pub p_wb: Vector3<f32>,         // Position
    pub v_wb: Vector3<f32>,         // Velocity
    pub bg: Vector3<f32>,           // Gyro bias
    pub ba: Vector3<f32>,           // Accel bias
}

// 6 DOF per camera clone
pub struct CameraClone {
    pub id: u64,
    pub timestamp_s: f64,
    pub q_wc: UnitQuaternion<f32>,
    pub p_wc: Vector3<f32>,
}

// Total: 15 + 6*N (N = 8-12 clones typical)
pub struct MsckfState {
    pub imu: ImuState,
    pub clones: Vec<CameraClone>,
}
```

## Platform Priors (4 Insertion Points)

### 1. Process Noise Shaping
```rust
fn process_noise_scale(&self, imu: &ImuState, dt: f32) -> ProcessNoiseScale {
    // Quad: inflate during aggressive motion
    // Fixed-wing: reduce in steady cruise
    // UGV: lower vertical noise
    // VTOL: inflate all in transition
}
```

### 2. Prediction Scoring
```rust
fn score_prediction(&self, prev: &ImuState, predicted: &ImuState) -> PriorScore {
    // Check speed, acceleration, tilt limits
    // Return score (0-1) and flags
}
```

### 3. Update Gating
```rust
fn gate_msckf_update(&self, summary: &VisualUpdateSummary) -> GateDecision {
    // Quad: allow high agility, reject absurd jumps
    // Fixed-wing: reject lateral/vertical nonsense
    // UGV: strongest gating on vertical motion
    // VTOL: mode-dependent
}
```

### 4. Covariance Shaping
```rust
fn measurement_covariance_scale(&self, summary: &VisualUpdateSummary) -> f32 {
    // Return alpha > 1 to downweight suspicious updates
}
```

## Per-Vehicle Behavior

### Quadrotor
- **Motion model**: Agile 6-DoF
- **Gating**: Allow high rates, reject impossible accel
- **Best for**: Aggressive motion, hover

### Fixed-Wing
- **Motion model**: Forward flight, curvature bounded
- **Gating**: Strong on lateral/vertical, smooth turns
- **Best for**: High-speed, structured motion

### VTOL
- **Motion model**: Mode-switched (hover/cruise/transition)
- **Gating**: Quad in hover, fixed-wing in cruise, loose in transition
- **Best for**: Hybrid vehicles

### UGV
- **Motion model**: Planar, nonholonomic
- **Gating**: Strongest on vertical/lateral
- **Best for**: Ground vehicles

## Implementation Status

The MSCKF architecture is designed but not yet fully implemented in RS-VIO. The current system uses:
- Sliding window BA with IMU factors
- Motion prior gating in PnP path
- Platform-specific priors (QuadrotorPrior, UgvPrior, etc.)

## Next Steps

1. **Phase 1**: Implement core MSCKF state/covariance
2. **Phase 2**: Add IMU propagation with clone augmentation
3. **Phase 3**: Implement MSCKF update (triangulation + nullspace)
4. **Phase 4**: Integrate platform priors at 4 insertion points
5. **Phase 5**: Tune and benchmark on Pi 5

## References

- Mourikis & Roumeliotis, "A Multi-State Constraint Kalman Filter for Vision-Aided Inertial Navigation", ICRA 2007
- Li & Mourikis, "High-precision, consistent EKF-based visual-inertial odometry", IJRR 2013
- Eckenhoff et al., "Continuous-Inertial-Timing Visual-Inertial Odometry", RSS 2018
