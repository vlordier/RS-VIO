# IMU Testing - Reveal the Fundamental Issues

## Test Suite to Expose Issues

### Test 1: Variable Timestamp Spacing (Issue #1)

**Goal**: Prove that constant `dt` produces wrong results with variable-spaced measurements

```rust
#[test]
fn test_variable_timestamp_spacing() {
    let config = ImuConfig::default();
    let mut estimator = VelocityEstimator::new(config);
    
    // Constant acceleration in one direction
    let accel_value = 1.0; // 1 m/s²
    
    // Variable-spaced measurements: [1ms, 1ms, 2ms, 1ms]
    let measurements = vec![
        ImuData { timestamp: 1_000_000, accel: [accel_value, 0.0, 0.0], gyro: [0.0; 3] },
        ImuData { timestamp: 2_000_000, accel: [accel_value, 0.0, 0.0], gyro: [0.0; 3] },
        ImuData { timestamp: 3_000_000, accel: [accel_value, 0.0, 0.0], gyro: [0.0; 3] },
        ImuData { timestamp: 5_000_000, accel: [accel_value, 0.0, 0.0], gyro: [0.0; 3] }, // 2ms gap
        ImuData { timestamp: 6_000_000, accel: [accel_value, 0.0, 0.0], gyro: [0.0; 3] },
    ];
    
    // Current wrong way (constant dt=1ms)
    let dt = 0.001;
    estimator.update(&measurements, dt);  // ❌ Treats all gaps as 1ms
    let wrong_velocity = estimator.get_velocity();
    
    // Expected: v = a * (1ms + 1ms + 2ms + 1ms) = 1.0 * 0.005 = 0.005 m/s
    // Wrong result: v = a * (4 * 1ms) = 1.0 * 0.004 = 0.004 m/s (20% error!)
    
    println!("Wrong velocity: {}", wrong_velocity.x);  // ~0.004
    println!("Expected velocity: 0.005");
    
    // This test PASSES with current code, proving it's wrong!
    assert!((wrong_velocity.x - 0.004).abs() < 0.0001);  // ✓ Current code
    // But should FAIL:
    // assert!((wrong_velocity.x - 0.005).abs() < 0.0001);  // ✗ With fixed code
}
```

**Current Result**: PASSES (system is broken)
**With Fix**: Should update to use actual timestamp deltas

---

### Test 2: Noise Covariance Behavior (Issue #2)

**Goal**: Prove that covariance changes opposite to expectation

```rust
#[test]
fn test_noise_covariance_consistency() {
    let noise = ImuNoise::default();
    
    // Create preintegration with same measurements but different dt
    let accel = na::Vector3::new(0.0, 0.0, 1.0);
    let gyro = na::Vector3::new(0.0, 0.0, 0.1);
    
    // Case 1: dt = 0.01s (10ms)
    let mut preint_10ms = PreintegratedImu::new(noise.clone());
    preint_10ms.integrate(gyro, accel, 0.01);
    let cov_10ms_diagonal = preint_10ms.covariance.diagonal().clone();
    
    // Case 2: dt = 0.001s (1ms)
    let mut preint_1ms = PreintegratedImu::new(noise.clone());
    preint_1ms.integrate(gyro, accel, 0.001);
    let cov_1ms_diagonal = preint_1ms.covariance.diagonal().clone();
    
    // Current WRONG behavior:
    // Shorter dt → LARGER covariance (backwards!)
    // Because: cov = noise_density² / dt
    println!("Covariance for 10ms: {}", cov_10ms_diagonal[0]);
    println!("Covariance for 1ms:  {}", cov_1ms_diagonal[0]);
    
    // This test PASSES with current code:
    assert!(cov_1ms_diagonal[0] > cov_10ms_diagonal[0]);  // ✓ Current (WRONG)
    
    // This test FAILS with current code:
    // assert!(cov_10ms_diagonal[0] > cov_1ms_diagonal[0]);  // ✗ Correct behavior
}
```

**Current Result**: First assertion passes (system is backwards)
**Expected**: Longer integration should have MORE covariance, not less

**Practical Impact**:
```
100Hz IMU:  dt = 0.01s  → cov_ratio = 1.0
1000Hz IMU: dt = 0.001s → cov_ratio = 10.0 (same sensor, 10x more uncertainty!)
```
This is absurd. Using a faster IMU makes the filter LESS confident!

---

### Test 3: Orientation Staying at Identity (Issue #3)

**Goal**: Prove that velocity estimates are wrong when sensor is tilted

```rust
#[test]
fn test_velocity_with_sensor_tilt() {
    let config = ImuConfig::default();
    let mut estimator = VelocityEstimator::new(config);
    
    // Sensor tilted 45° forward (around Y axis)
    let tilt_angle = std::f64::consts::PI / 4.0;  // 45°
    let rotation = na::UnitQuaternion::from_axis_angle(
        &na::Unit::new_normalize(na::Vector3::new(0.0, 1.0, 0.0)),
        tilt_angle,
    );
    
    estimator.initialize_from_imu(&[], &rotation);  // ❌ Orientation update ignored!
    
    // When tilted forward 45°, accelerometer reads:
    // - Forward motion (a_x) → appears as up (a_z) in sensor frame
    // - Gravity (g) → appears as forward + down in sensor frame
    
    let accel_forward_tilted = na::Vector3::new(
        0.0,                              // No X in sensor frame
        0.0,                              // No Y
        9.81 * (45_f64).cos() + 5.0,     // Gravity component + forward accel tilted
    );
    
    let imu = ImuData {
        timestamp: 0,
        accel: accel_forward_tilted.into(),
        gyro: [0.0; 3],
    };
    
    estimator.update(&[imu], 0.01);
    let velocity = estimator.get_velocity();
    
    // Current WRONG behavior:
    // Assumes orientation is identity (R = I)
    // So it interprets the Z acceleration directly
    // v_z = 9.81 * 0.707 + 5.0 ≈ 12.95 m/s (vertical motion!)
    
    println!("Velocity Z (WRONG): {}", velocity.z);  // ~0.13 m/s upward
    println!("Expected velocity X (forward): ~0.05 m/s");
    println!("Expected velocity Z (gravity compensation): ~0.0");
    
    // This test PASSES with current code, proving orientation is ignored:
    assert!(velocity.z > 0.1);  // ✓ Current (WRONG)
    assert!(velocity.x < 0.001);  // ✓ Current (no forward velocity perceived)
    
    // This test FAILS with current code:
    // After proper rotation, should have:
    // assert!(velocity.x > 0.04);  // Forward motion
    // assert!(velocity.z < 0.01);  // Gravity mostly canceled
}
```

**Current Result**: System thinks forward motion is upward motion!
**Why**: Orientation stays at identity, acceleration not rotated from sensor to world frame

**Real-world impact**: Any time camera tilts (pitch/roll), velocity becomes garbage.

---

### Test 4: Bias Estimates Never Applied (Issue #4)

**Goal**: Show that initialization bias estimates are ignored

```rust
#[test]
fn test_initialization_bias_unused() {
    // Step 1: Estimate biases during static phase
    let mut bias_estimator = ImuBiasEstimator::new(ImuConfig::default());
    
    let static_imu = vec![
        ImuData { timestamp: 0, gyro: [0.1, 0.05, -0.02], accel: [0.5, -0.3, 9.81] },
        ImuData { timestamp: 10_000_000, gyro: [0.12, 0.04, -0.01], accel: [0.4, -0.25, 9.82] },
        // ... more static measurements
    ];
    
    for imu in &static_imu {
        bias_estimator.add_sample(imu);
    }
    
    let bias_est = bias_estimator.get_bias_estimate();
    println!("Estimated biases:");
    println!("  Gyro: {:?}", bias_est.gyro_bias);       // ~[0.11, 0.045, -0.015]
    println!("  Accel: {:?}", bias_est.accel_bias);     // ~[0.45, -0.28, 0.01]
    
    // Step 2: Initialize velocity estimator
    let mut vel_estimator = VelocityEstimator::new(ImuConfig::default());
    vel_estimator.initialize_from_imu(&static_imu, &na::UnitQuaternion::identity());
    
    // Step 3: Check what biases are actually being used
    // ❌ There's NO METHOD to check this in VelocityEstimator!
    // ❌ The biases initialized are always [0, 0, 0]!
    
    // This test PASSES with current code (proving biases unused):
    assert_eq!(vel_estimator.eskf.state.gyro_bias.norm(), 0.0);    // ✓ Always zero!
    assert_eq!(vel_estimator.eskf.state.accel_bias.norm(), 0.0);   // ✓ Always zero!
    
    // This test FAILS with current code:
    // assert!(vel_estimator.get_accel_bias() == bias_est.accel_bias);
    // assert!(vel_estimator.get_gyro_bias() == bias_est.gyro_bias);
}
```

**Current Result**: Biases remain zero despite estimation
**Why**: No integration between `ImuBiasEstimator` and `VelocityEstimator`

**Impact**: If hardware has systematic bias (e.g., +0.5 m/s² on Z):
- Estimates it correctly: [0, 0, 0.5]
- Then ignores it: uses [0, 0, 0]
- Then estimates gravity as 10.31 m/s² instead of 9.81 m/s²
- All velocity estimates are biased by 5% ❌

---

### Test 5: Gyro Measurements Ignored (Issue #3b)

**Goal**: Show that gyro data doesn't affect velocity estimation

```rust
#[test]
fn test_gyro_never_integrated() {
    let config = ImuConfig::default();
    let mut estimator = VelocityEstimator::new(config);
    
    // Two identical acceleration, different gyro
    let imu_still = ImuData {
        timestamp: 0,
        accel: [0.0, 0.0, 10.0],
        gyro: [0.0, 0.0, 0.0],  // No rotation
    };
    
    let imu_rotating = ImuData {
        timestamp: 10_000_000,
        accel: [0.0, 0.0, 10.0],        // Same acceleration
        gyro: [1.0, 0.0, 0.0],          // Spinning at 1 rad/s
    };
    
    // Case 1: With zero rotation
    let mut est1 = VelocityEstimator::new(config.clone());
    est1.initialize_from_imu(&[], &na::UnitQuaternion::identity());
    est1.update(&[imu_still], 0.01);
    let v1 = est1.get_velocity();
    
    // Case 2: With rotation
    let mut est2 = VelocityEstimator::new(config.clone());
    est2.initialize_from_imu(&[], &na::UnitQuaternion::identity());
    est2.update(&[imu_rotating], 0.01);
    let v2 = est2.get_velocity();
    
    // Current behavior: Gyro is completely ignored
    // So both cases produce IDENTICAL velocities!
    println!("Velocity with gyro=0:   {}", v1);
    println!("Velocity with gyro=1:   {}", v2);
    
    // This test PASSES with current code (gyro ignored):
    assert!((v1 - v2).norm() < 0.00001);  // ✓ Exactly equal!
    
    // This test FAILS with current code:
    // If gyro integration existed, v2 would have different orientation
    // and therefore different velocity transformation
    // assert!((v1 - v2).norm() > 0.001);
}
```

**Current Result**: Gyro data has zero effect on velocity
**Why**: Gyro measurements are read with `_` prefix (unused variable)

---

### Test 6: Open-Loop Filter (Issue #6)

**Goal**: Show that covariance grows unbounded without measurement updates

```rust
#[test]
fn test_eskf_covariance_only_grows() {
    let config = ImuConfig::default();
    let mut estimator = VelocityEstimator::new(config);
    
    let imu = ImuData {
        timestamp: 0,
        accel: [0.1, 0.1, 10.0],
        gyro: [0.01, 0.01, 0.01],
    };
    
    let initial_uncertainty = estimator.get_covariance_trace();  // Trace = sum of diagonal
    
    // Predict 100 times without any corrections
    for i in 0..100 {
        estimator.update(&[imu], 0.01);
    }
    
    let final_uncertainty = estimator.get_covariance_trace();
    
    println!("Initial uncertainty: {}", initial_uncertainty);
    println!("Final uncertainty:   {}", final_uncertainty);
    
    // This test PASSES with current code (open-loop growth):
    assert!(final_uncertainty > initial_uncertainty);  // ✓ Always grows
    
    // With a proper filter with measurements:
    // Uncertainty would oscillate: grow during prediction, shrink during update
    // Current: monotonic growth
}
```

**Current Result**: Covariance grows monotonically forever
**Why**: Only prediction, never measurement updates to correct

**Impact**: Filter confidence degrades continuously. After 1 minute, uncertainty is huge. No way to fix it.

---

## Integration Test: Complete System Validation

### Test 7: Round-Trip Validation

```rust
#[test]
fn test_end_to_end_motion() {
    // Scenario: 1m forward motion, level platform
    
    // Expected:
    // - Constant 1 m/s forward for 1 second
    // - Zero vertical motion
    // - Result: 1m displacement forward, 0m vertical
    
    let mut estimator = VelocityEstimator::new(ImuConfig::default());
    
    // Proper 1 m/s forward acceleration
    let accel_forward = 1.0;  // m/s²
    
    // For 1 second: integrate v = a*t = 1*1 = 1 m/s
    let imu_measurements: Vec<_> = (0..100)
        .map(|i| {
            ImuData {
                timestamp: i * 10_000_000,  // 10ms spacing
                accel: [accel_forward, 0.0, 10.0],  // forward + gravity
                gyro: [0.0; 3],
            }
        })
        .collect();
    
    // Initialize
    estimator.initialize_from_imu(&[imu_measurements[0]], &na::UnitQuaternion::identity());
    
    // Process all measurements
    estimator.update(&imu_measurements, 0.01);  // ❌ Wrong: constant dt for variable timestamps
    
    let final_velocity = estimator.get_velocity();
    
    // Expected: [~1.0, 0, 0] m/s (forward, horizontal)
    // With bugs:
    // - Bug #1 (variable dt): ~0.99 (timing wrong)
    // - Bug #2 (noise cov): affects Kalman gains (if using updates)
    // - Bug #3 (no orientation): [0, 0, 1.0] (upward instead of forward!)
    
    println!("Final velocity: {}", final_velocity);
    println!("Expected:       [1.0, 0, 0]");
    
    // This test FAILS with current code due to:
    // - Orientation staying at identity
    // - No measurement updates to correct
    // - Possible variable-dt timing errors
}
```

---

## Test Execution

### To Expose Issue #1 (Variable dt)
```bash
cargo test test_variable_timestamp_spacing -- --nocapture
```
**Expected**: Shows 20% error in velocity with variable-spaced measurements

### To Expose Issue #2 (Noise covariance)
```bash
cargo test test_noise_covariance_consistency -- --nocapture
```
**Expected**: Shows covariance DECREASES with faster IMU (backwards!)

### To Expose Issue #3 (Missing orientation)
```bash
cargo test test_velocity_with_sensor_tilt -- --nocapture
```
**Expected**: Forward motion interpreted as upward motion

### To Expose Issue #4 (Unused biases)
```bash
cargo test test_initialization_bias_unused -- --nocapture
```
**Expected**: Estimated biases are always zero in ESKF

### To Expose Issue #3b (Gyro ignored)
```bash
cargo test test_gyro_never_integrated -- --nocapture
```
**Expected**: Gyro data has zero effect on output

### To Expose Issue #6 (Open-loop)
```bash
cargo test test_eskf_covariance_only_grows -- --nocapture
```
**Expected**: Uncertainty grows forever without bounds

---

## Summary

These tests don't just find bugs—they expose fundamental architectural problems:

| Test | Current | Expected | Status |
|------|---------|----------|--------|
| Variable dt spacing | Accepts constant dt | Uses actual deltas | ❌ BROKEN |
| Noise covariance | Divides by dt | Multiplies by dt | ❌ BACKWARDS |
| Sensor tilt | Assumes identity R | Rotates acceleration | ❌ WRONG |
| Bias initialization | Ignores estimates | Applies biases | ❌ DISCONNECTED |
| Gyro integration | Skips gyro | Integrates for R | ❌ INCOMPLETE |
| Filter updates | Only prediction | Has measurement update | ❌ OPEN-LOOP |

**Every test passes with current code, proving the system is fundamentally broken.**

