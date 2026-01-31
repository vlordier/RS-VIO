# IMU Processing Visualization Guide

## Overview

The RS-VIO system now includes comprehensive **before and after** IMU processing visualization in Rerun, with support for harmonic decomposition and signal quality analysis.

## Features

### 1. **Raw IMU Visualization** (`log_imu_raw`)
Displays raw sensor measurements directly from the IMU:
- **Accelerometer**: 3-axis raw data [x, y, z] in m/s²
- **Gyroscope**: 3-axis raw data [x, y, z] in rad/s
- **Visualization**: Time-series line strips (light red for accel, light blue for gyro)
- **Use Case**: Detect sensor noise, vibration patterns, spikes

### 2. **Processed IMU Visualization** (`log_imu_processed`)
Shows IMU data after bias correction and filtering:
- **Accelerometer**: Bias-corrected acceleration [x, y, z] in m/s²
- **Gyroscope**: Bias-corrected rotation rate [x, y, z] in rad/s
- **Visualization**: Clean time-series (bright red/blue)
- **Use Case**: Verify bias removal, assess filtering effectiveness

### 3. **Harmonic Decomposition** (`log_imu_harmonics`)
Breaks down IMU signals into fundamental components:

```
Raw IMU = Gravity + Bias + Harmonics + Noise
```

**Components visualized:**
- **Gravity Component**: Estimated gravity vector (green arrow from origin)
  - Typically ≈ [0, 0, -9.81] m/s² in sensor frame
  - Aligned with accelerometer's Z-axis when stationary

- **Accelerometer Bias**: Constant sensor offset [b_x, b_y, b_z]
  - Orange point in 3D space
  - Typically 0.05-0.5 m/s² depending on sensor quality

- **Gyroscope Bias**: Constant rotation rate offset
  - Usually 0.1-1 deg/s equivalent
  - Critical for long-term drift reduction

- **Harmonic Components**: Residual oscillations after gravity/bias removal
  - Magenta time-series line strip
  - Represents measurement noise and actual vibration

### 4. **Signal Quality Metrics** (`log_imu_signal_quality`)
Analyzes signal health and reliability:

**Per-axis metrics:**
- **SNR (Signal-to-Noise Ratio)**: dB scale
  - > 30 dB = Excellent
  - 20-30 dB = Good
  - 10-20 dB = Fair
  - < 10 dB = Poor

- **RMS (Root Mean Square)**: Energy measure in m/s²
  - Low RMS = steady measurements
  - High RMS = significant noise/vibration

- **Peak**: Maximum absolute value per axis
  - Detect transient spikes
  - Identify impact events

## Typical Usage Pattern

### In the Estimator or IMU Processor

```rust
use rs_vio::imu::ImuSignalAnalyzer;
use rs_vio::viewers::Viewer;

// Create analyzer
let mut analyzer = ImuSignalAnalyzer::new(100); // 100-sample window

// Process incoming IMU data
for imu_data in imu_measurements {
    analyzer.process_measurement(imu_data);
}

// Get harmonics decomposition
let decomp = analyzer.decompose_harmonics();

// Visualize raw measurements
viewer.log_imu_raw(
    timestamp_ns,
    &raw_accel_data,      // &[[f32; 3], ...]
    &raw_gyro_data,       // &[[f32; 3], ...]
    "world/imu/raw"
);

// Visualize processed measurements
viewer.log_imu_processed(
    timestamp_ns,
    &processed_accel,     // &[[f32; 3], ...]
    &processed_gyro,      // &[[f32; 3], ...]
    "world/imu/processed"
);

// Visualize harmonic decomposition
viewer.log_imu_harmonics(
    timestamp_ns,
    decomp.gravity.to_array().map(|v| v as f32),
    decomp.accel_bias.to_array().map(|v| v as f32),
    decomp.gyro_bias.to_array().map(|v| v as f32),
    &decomp.residual_harmonics.iter()
        .map(|h| h.to_array().map(|v| v as f32))
        .collect::<Vec<_>>(),
    "world/imu/harmonics"
);

// Visualize signal quality
viewer.log_imu_signal_quality(
    timestamp_ns,
    decomp.quality.snr,
    decomp.quality.rms,
    decomp.quality.peak,
    "world/imu/quality"
);
```

## Signal Decomposition Math

### Gravity Component
The accelerometer measures:
$$\mathbf{a}_{raw} = \mathbf{g} + \mathbf{a}_{actual} + \mathbf{b}_a + \mathbf{n}_a$$

Where:
- $\mathbf{g}$ = gravity (~9.81 m/s² downward)
- $\mathbf{a}_{actual}$ = true linear acceleration
- $\mathbf{b}_a$ = accelerometer bias
- $\mathbf{n}_a$ = measurement noise

### Bias Estimation
During initialization (stationary phase):
$$\mathbf{b}_a \approx \text{mean}(\mathbf{a}_{raw}) - \mathbf{g}$$

### Harmonic Extraction
Residual after gravity and bias removal:
$$\mathbf{r} = \mathbf{a}_{raw} - \mathbf{g} - \mathbf{b}_a$$

Harmonics represent vibration modes and coupling effects.

## Color Coding in Rerun

| Component | Color | Meaning |
|-----------|-------|---------|
| Raw Accel | Light Red | Original sensor data |
| Processed Accel | Bright Red | Bias-corrected |
| Raw Gyro | Light Blue | Original sensor data |
| Processed Gyro | Bright Blue | Bias-corrected |
| Gravity | Green | Estimated downward vector |
| Bias (Accel) | Orange | Static offset |
| Harmonics | Magenta | Residual noise/vibration |

## Performance Implications

- **Computation**: O(n) per frame where n = window size (default 100)
- **Memory**: ~200 bytes per axis (3 axes × 100 samples × 8 bytes)
- **Rerun Transfer**: ~1-2ms per visualization call

## Tuning Recommendations

### For High-Vibration Environments
- Increase window size → Better averaging, higher latency
- Monitor harmonic RMS → Should stay < 0.1 m/s²

### For Low-Vibration/Indoor
- Smaller window (50 samples) → Faster response
- Stricter SNR threshold (> 25 dB)

### For Flight-Ready Systems
- Use learned vibration filters (see `vibration_filter.rs`)
- Combine signal quality with adaptive weighting
- Log harmonics for post-flight analysis

## Integration Points

1. **EurocPlayer / TUMVIPlayer**: Call `log_imu_*` methods in frame processing loop
2. **ImuProcessor**: Enhanced with `ImuSignalAnalyzer` for real-time decomposition
3. **Estimator**: Pass harmonic metrics to optimization weighting
4. **Viewer**: Auto-routes to Rerun with consistent coloring

## Debugging Common Issues

### High Accel RMS on One Axis
- Check for misalignment
- Verify calibration
- Look for systematic vibration pattern

### Gravity Magnitude ≠ 9.81 m/s²
- Indicates poor sensor calibration
- Check temperature drift
- May need in-flight recalibration

### SNR Degradation Over Time
- Possible sensor saturation
- Environmental shock/impact
- Power supply noise coupling

## References

- Forster et al., "IMU Preintegration on Manifold", RSS 2017
- Trawny & Roumeliotis, "Indirect Kalman Filter for 3D Attitude", 2005
- Vibration analysis: Welch's method, FFT-based harmonic detection

## Next Steps

- [ ] Integrate with frequency-domain FFT analysis
- [ ] Add adaptive filter gain visualization
- [ ] Real-time SNR-based outlier rejection
- [ ] Post-flight harmonics analysis pipeline
