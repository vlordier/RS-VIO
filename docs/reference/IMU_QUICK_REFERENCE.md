# IMU Visualization Quick Reference

## 📊 What You Can See in Rerun

### Raw vs. Processed Comparison
```
Raw Accel (Light Red)  ─────────────>  Processed Accel (Bright Red)
Raw Gyro (Light Blue)  ─────────────>  Processed Gyro (Bright Blue)
```
**Shows:** Effect of bias removal and filtering

### Harmonic Decomposition
```
Gravity Component      Green Arrow      (~9.81 m/s² downward)
Accel Bias            Orange Point      (constant sensor offset)
Gyro Bias             Text Annotation   (rotation rate offset)
Harmonics/Noise       Magenta Lines     (residual oscillations)
```

### Signal Quality Dashboard
```
SNR (Signal-to-Noise)  Bar Chart  Excellent >30dB | Good 20-30dB | Fair 10-20dB
RMS (Energy)          Bar Chart  Per-axis root mean square
Peak (Max Value)      Bar Chart  Transient spike detection
```

## 🚀 Quick Usage

### Basic Integration
```rust
use rs_vio::imu::ImuSignalAnalyzer;

// Create analyzer
let mut analyzer = ImuSignalAnalyzer::new(100);

// Process measurements
for imu_data in incoming_imu {
    analyzer.process_measurement(imu_data);
}

// Get decomposition
let decomp = analyzer.decompose_harmonics();

// Visualize (in order)
viewer.log_imu_raw(ts, &raw_a, &raw_g, "imu/raw");
viewer.log_imu_processed(ts, &proc_a, &proc_g, "imu/processed");
viewer.log_imu_harmonics(ts, decomp.gravity, decomp.accel_bias,
                         decomp.gyro_bias, &decomp.residual_harmonics, "imu/harmonics");
viewer.log_imu_signal_quality(ts, decomp.quality.snr,
                              decomp.quality.rms, decomp.quality.peak, "imu/quality");
```

## 🎯 Signal Decomposition Formula

```
Raw IMU = Gravity + Bias + Fundamental Harmonic + Higher Harmonics + Noise

Where:
- Gravity ≈ [0, 0, -9.81] m/s² (downward)
- Bias = constant sensor offset (0.05-0.5 m/s² typical)
- Harmonics = oscillatory components (f0, 2f0, 3f0, ...)
- Noise = random measurement error
```

## 🔧 Tuning Tips

| Scenario | Action |
|----------|--------|
| High vibration | Increase window size (200+), monitor RMS |
| Low SNR (<15dB) | Increase measurement noise in filter |
| Gravity ≠ 9.81 | Sensor calibration needed |
| High bias drift | Check for temperature effects |
| Spiky measurements | Harmonic RMS should stay < 0.1 m/s² |

## 📍 Color Reference

| Color | Component | Meaning |
|-------|-----------|---------|
| 🔴 Light Red | Raw Accel | Original sensor data |
| 🔴 Bright Red | Processed Accel | Bias-corrected |
| 🔵 Light Blue | Raw Gyro | Original sensor data |
| 🔵 Bright Blue | Processed Gyro | Bias-corrected |
| 🟢 Green | Gravity | Estimated downward direction |
| 🟠 Orange | Bias | Static sensor offset |
| 🟣 Magenta | Harmonics | Residual noise/vibration |

## 📊 Quality Levels

### SNR (dB)
- **>30**: Excellent - Low noise, clean signal
- **20-30**: Good - Acceptable for most applications
- **10-20**: Fair - Noisy but usable, may need filtering
- **<10**: Poor - High noise, reliability concerns

### RMS (m/s²)
- **<0.05**: Very stable (excellent)
- **0.05-0.1**: Good signal quality
- **0.1-0.2**: Acceptable with adaptive weighting
- **>0.2**: High vibration, needs attention

## 🔍 Debugging Checklist

- [ ] Gravity magnitude close to 9.81 m/s²?
- [ ] Bias estimates reasonable (< 0.5 m/s²)?
- [ ] SNR > 15 dB on all axes?
- [ ] RMS stable over time?
- [ ] Harmonic components < 0.1 m/s² RMS?
- [ ] Raw vs. processed shows clear improvement?

## 📚 Documentation Files

- **IMU_VISUALIZATION_GUIDE.md** - Comprehensive guide with examples
- **IMU_VISUALIZATION_IMPLEMENTATION.md** - Technical implementation details
- **examples/imu_visualization_example.rs** - Code examples
- **DELIVERY_SUMMARY.md** - What was delivered and how

## 🎓 Learning Resources

The visualization teaches you:
1. How much bias your sensor has
2. What the gravity vector looks like in sensor frame
3. How much noise is in your measurements
4. What harmonics (vibration modes) are present
5. How effective your bias removal is

## 🚀 Next Steps

1. Run with actual IMU data in Rerun
2. Observe gravity alignment and bias estimates
3. Tune SNR thresholds for your environment
4. Use quality metrics to weight measurements
5. Validate against ground truth if available

---

**Remember:** You're looking at 4 separate visualizations:
- ✅ Raw data (light colors)
- ✅ Processed data (bright colors)
- ✅ Decomposed components (gravity, bias, harmonics)
- ✅ Quality metrics (SNR, RMS, peak)

All synchronized in the same Rerun viewer!
