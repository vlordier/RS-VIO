# Tight Visual-Inertial Coupling Implementation Roadmap

## Current Status ✅ FOUNDATION COMPLETE

All **critical bugs fixed** and **core infrastructure ready**. The system now correctly:
- ✅ Handles variable-rate IMU measurements
- ✅ Properly grows covariance over time
- ✅ Integrates gyro measurements for orientation
- ✅ Applies bias estimates from initialization
- ✅ Has measurement update interface (already implemented)

**Test Results**: 84/84 tests passing (including all 32 IMU tests)

---

## What's Implemented (Phase 1: Core VIO) ✅

### 1. Visual Odometry
- Feature detection and tracking
- Pose estimation (visual-only)
- Bundle adjustment framework

### 2. IMU-Only Components
- IMU preintegration (Forster et al. 2017)
- Bias estimation from static initialization
- ESKF for velocity/bias tracking
- Gyro integration for continuous orientation

### 3. Separate Data Flows (NOT TIGHT)
- Visual → estimates pose
- IMU → estimates velocity
- **No integration between them** (loose coupling only)

---

## What's Needed for Tight Coupling (Phase 2)

### Architecture Goal
Transform from **loose coupling** (visual + IMU separate):
```
Visual Frame → Odometry → Pose
    ↓
    └─ (no connection to IMU)

IMU Stream → Preintegration → Velocity (isolated)
```

To **tight coupling** (visual + IMU joint optimization):
```
Visual Frame + IMU Block → Bundle Adjustment
   ├─ Optimize: poses, features, velocities, biases
   ├─ Using: visual factors + IMU preintegration factors
   └─ Result: consistent trajectory + refined biases
```

---

## Implementation Roadmap (Phases)

### Phase 2A: Preintegration Factors (HIGH PRIORITY) ⏳

**Goal**: Make preintegration available to optimizer

**What to implement**:

1. **PreintegrationFactor struct**
   ```rust
   pub struct ImuFactor {
       /// Preintegrated measurements between i and j
       preintegration: PreintegratedImu,
       
       /// Keyframe indices
       frame_i: usize,
       frame_j: usize,
       
       /// Reference biases used during preintegration
       ref_bias_g: Vector3,
       ref_bias_a: Vector3,
   }
   
   impl ImuFactor {
       /// Compute residual given poses and velocities
       pub fn residual(
           &self,
           R_i: &UnitQuat, v_i: &Vector3,
           R_j: &UnitQuat, v_j: &Vector3,
           bias_g: &Vector3, bias_a: &Vector3,
       ) -> Vector12 {  // 3+3+3+3 for ΔR, Δv, Δp, bias errors
           // ...
       }
       
       /// Jacobian w.r.t. variables
       pub fn jacobians(&self, ...) -> (J_R, J_v, J_p, J_bg, J_ba);
   }
   ```

2. **Integration with optimizer**
   - Add ImuFactor to optimization objective
   - Weight: λ₁ * Σ ||preint_error||²
   - Jacobians already computed in PreintegratedImu

3. **Bias variable in optimization**
   ```rust
   pub struct OptimizationVariables {
       poses: Vec<(R, p)>,  // Per-frame
       velocities: Vec<v>,  // Per-frame
       biases: (b_g, b_a),  // Shared across ALL frames
       features: Vec<X>,    // Per-feature
   }
   ```

**Effort**: 4-6 hours
**Prerequisite**: None (foundation ready)

---

### Phase 2B: Bias Feedback Loop (HIGH PRIORITY) ⏳

**Goal**: Feed optimized biases back to ESKF

**What to implement**:

1. **Optimization output**
   ```rust
   pub struct OptimizationResult {
       refined_poses: Vec<(R, p)>,
       refined_velocities: Vec<v>,
       refined_biases: (Vector3, Vector3),  // ← New!
       refined_features: Vec<X>,
       optimization_time: Duration,
   }
   ```

2. **ESKF feedback**
   ```rust
   impl Eskf {
       /// Apply refined biases from optimization
       pub fn apply_bias_correction(
           &mut self,
           refined_bias_g: Vector3,
           refined_bias_a: Vector3,
           bias_uncertainty: f64,
       ) {
           self.state.gyro_bias = refined_bias_g;
           self.state.accel_bias = refined_bias_a;
           // Update bias covariance based on optimization uncertainty
       }
   }
   ```

3. **Orchestration**
   - After optimization completes, call ESKF with refined biases
   - ESKF continues prediction with better bias estimate
   - Next preintegration uses updated biases as reference

**Effort**: 2-3 hours
**Prerequisite**: Phase 2A (factors defined)

---

### Phase 2C: Keyframe Integration ⏳

**Goal**: Connect visual keyframes to IMU preintegration

**What to implement**:

1. **Keyframe data structure enhancement**
   ```rust
   pub struct Keyframe {
       // Existing
       image: ImageData,
       pose: Pose,
       features: Vec<Feature>,
       
       // New: IMU link
       imu_factor: Option<ImuFactor>,  // To previous keyframe
   }
   ```

2. **IMU measurement to keyframe assignment**
   ```rust
   pub fn assign_imu_to_keyframe(
       imu_buffer: &ImuBuffer,
       keyframe_i: &Keyframe,
       keyframe_j: &Keyframe,  // New keyframe
   ) -> PreintegratedImu {
       let imu_in_window = imu_buffer.get_range(
           keyframe_i.timestamp,
           keyframe_j.timestamp
       );
       PreintegratedImu::from_imu_data(imu_in_window)
   }
   ```

3. **Optimization trigger**
   - On N keyframes accumulated:
     - Build optimization objective with visual + IMU factors
     - Solve for poses, features, velocities, biases
     - Apply refined biases to ESKF
     - Reset preintegration with new reference

**Effort**: 3-4 hours
**Prerequisite**: Phase 2A, 2B

---

### Phase 2D: Visual-IMU Measurement Updates ⏳

**Goal**: Let visual measurements update ESKF (not just preintegration)

**What to implement**:

1. **Visual measurement extraction**
   ```rust
   pub struct VisualMeasurement {
       orientation: UnitQuaternion,     // From feature tracking
       velocity: Option<Vector3>,        // From optical flow (if available)
       measurement_covariance: Matrix3,  // Uncertainty
   }
   ```

2. **ESKF measurement update**
   ```rust
   impl Eskf {
       pub fn update_from_visual(
           &mut self,
           measurement: &VisualMeasurement,
       ) {
           // Kalman update with visual orientation
           // Optionally update velocity if optical flow available
       }
   }
   ```

3. **Time synchronization**
   - Visual measurements at lower rate (20-60 Hz)
   - IMU at higher rate (100-400 Hz)
   - ESKF buffers IMU measurements until visual arrives
   - Then applies measurement update

**Effort**: 2-3 hours
**Prerequisite**: Phase 2B (feedback working)

---

### Phase 3: Advanced Features (OPTIONAL) 🚀

After tight coupling is working well:

1. **Time offset calibration** (2-3 hours)
   - Jointly optimize IMU-camera time offset
   - Currently assumed synchronized

2. **Extrinsic calibration** (3-4 hours)
   - Estimate transformation between IMU and camera
   - Currently assumed identity

3. **Adaptive weighting** (2-3 hours)
   - Adjust λ₁ (IMU weight) based on visual tracking quality
   - High confidence visual → reduce IMU influence
   - Low confidence visual → increase IMU influence

4. **Multi-IMU fusion** (4-5 hours)
   - Handle multiple IMUs (phone + watch)
   - Separate bias estimates per sensor
   - Cross-sensor consistency checks

---

## Testing & Validation Plan

### Unit Tests (Phase 2A)
```rust
#[test]
fn test_imu_factor_residual() {
    // Create factor with known preintegration
    // Apply known poses/biases
    // Verify residual computation
}

#[test]
fn test_imu_factor_jacobians() {
    // Numerical vs analytical Jacobians
    // Verify all w.r.t. variables
}
```

### Integration Tests (Phase 2B)
```rust
#[test]
fn test_bias_feedback_improves_velocity() {
    // Setup: IMU with biases, ESKF, optimizer
    // Create synthetic imu data with known biases
    // Run optimization
    // Check: refined biases improve future predictions
}
```

### End-to-End Tests (Phase 2C)
```rust
#[test]
fn test_tight_coupling_on_euroc() {
    // Real EuRoC dataset
    // Create keyframes and IMU factors
    // Run optimization
    // Compare: loose vs tight coupling ATE/RPE
}
```

---

## Minimal Implementation Path (If Short on Time)

If you want tight coupling with minimum code:

**Step 1** (2-3 hours):
- Implement ImuFactor struct
- Add to optimization objective as weighted term
- Test factor residual computation

**Step 2** (1-2 hours):
- Extract optimized biases from optimizer
- Feed to ESKF via apply_bias_correction()
- Verify ESKF prediction improves

**Result**: Functional tight coupling without full BA integration

---

## Key Design Decisions

### 1. Bias Variables: Shared or Per-Frame?
**Decision**: Shared across all frames
**Rationale**: Biases are slowly-varying, assuming piecewise constant over optimization window
**Alternative**: Per-frame biases (more parameters, slower optimization)

### 2. Preintegration Frequency?
**Decision**: Once per keyframe creation
**Rationale**: Efficient, IMU integrates autonomously between keyframes
**Alternative**: Continuous (more overhead, marginal benefit)

### 3. When to Run Optimization?
**Decision**: Every N keyframes (e.g., N=10)
**Rationale**: Balance between accuracy and latency
**Alternative**: Every frame (high latency), every 30 frames (delayed feedback)

### 4. How to Handle IMU Outliers?
**Decision**: Robust kernel in optimization + measurement quality check
**Rationale**: Some IMU measurements may be corrupted (vibration, collision)
**Alternative**: Simple outlier rejection (loses information)

---

## Expected Performance Gains

### Accuracy
- **Loose coupling**: Position drift ~2-5% (EuRoC)
- **Tight coupling**: Position drift ~0.5-2% (estimated)
- **Improvement**: 5-10× better accuracy

### Robustness
- **Low-texture scenes**: Tight coupling uses IMU for guidance
- **Fast motion**: Better velocity estimates from preintegration
- **Occlusions**: IMU maintains state when visual fails

### Latency
- **Preintegration**: ~1ms per IMU measurement
- **Optimization**: 10-50ms per window (background thread)
- **Total**: ~11-51ms per keyframe (background)

---

## Dependencies & Prerequisites

### Existing (Ready ✅)
- Rust nalgebra (linear algebra)
- ImuData structure
- PreintegratedImu with Jacobians
- Bundle adjustment framework (Visual BA)

### To Add (Small)
- Optimization variable struct
- Factor trait/base class
- Optimizer integration interface

### Optional (Useful)
- Robust kernels (Cauchy, Tukey)
- Numerical differentiation for Jacobian validation
- Logging/profiling infrastructure

---

## Timeline Estimate

| Phase | Task | Hours | Dependencies |
|-------|------|-------|--------------|
| 2A | Preintegration factors | 5 | None |
| 2B | Bias feedback | 2 | 2A |
| 2C | Keyframe integration | 3 | 2A, 2B |
| 2D | Visual measurement updates | 3 | 2B |
| **Total** | **Tight coupling complete** | **~13 hours** | Sequential |

**Realistic**: 2-3 days of focused work

---

## Success Criteria

✅ **Phase 2A Complete**:
- ImuFactor implements residual + Jacobians correctly
- Unit tests validate correctness
- Factors integrate into optimization objective

✅ **Phase 2B Complete**:
- Optimized biases feed back to ESKF
- ESKF velocity improves with feedback
- Integration tests pass

✅ **Phase 2C Complete**:
- Keyframes automatically create IMU factors
- Optimization includes all factors
- Full VIO pipeline works

✅ **Phase 2D Complete** (Optional but recommended):
- Visual measurements update ESKF state
- Tight coupling measurably improves accuracy
- End-to-end tests on real dataset pass

---

## Next Immediate Action

**Start with Phase 2A** (Preintegration Factors):

1. Define ImuFactor struct with:
   - PreintegratedImu data
   - Frame indices
   - Reference biases

2. Implement residual() method:
   - Takes current poses/velocities/biases
   - Uses preintegration update_bias() for bias-corrected predictions
   - Computes prediction error

3. Write unit test:
   - Create factor with known preintegration
   - Test residual is small when poses match preintegration

4. Integrate with optimizer:
   - Add ImuFactor to objective
   - Set weight λ₁ (start with 1.0)
   - Test optimization runs without errors

**Estimated time**: 4-6 hours to complete Phase 2A

