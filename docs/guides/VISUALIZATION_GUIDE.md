# VIO Optimization Visualization Guide

This guide explains how to use the visualization utilities to compare VIO optimizations with and without various enhancements.

## Overview

The visualization module provides tools to generate comparison plots showing the benefits of:

1. **IMU-Aided Tracking** vs baseline KLT
2. **Sub-Pixel Disparity Refinement** vs integer disparity
3. **Rolling Shutter Correction** vs global shutter model

## Quick Start

### 1. Run the Demo Example

```bash
cargo run --example plot_vio_comparisons
```

This generates:
- CSV data files with comparison metrics
- Python plotting script
- Statistical summaries

### 2. Generate Plots

```bash
# Install dependencies (first time only)
pip install pandas matplotlib numpy

# Generate plots
python3 ./plot_output/plot_comparisons.py
```

This creates three PNG files:
- `tracking_comparison.png` - IMU-aided tracking benefits
- `disparity_comparison.png` - Sub-pixel refinement accuracy
- `rolling_shutter_comparison.png` - RS correction effectiveness

## Using in Your Code

### Import the Module

```rust
use rs_vio::vision::{
    TrackingComparison, DisparityComparison, RollingShutterComparison,
    generate_plot_script,
};
```

### 1. Tracking Comparison

```rust
let mut tracking = TrackingComparison::new();

// For each frame, record baseline vs IMU-aided metrics
for frame_idx in 0..num_frames {
    let baseline_features = baseline_tracker.feature_count();
    let imu_aided_features = imu_tracker.feature_count();
    let baseline_avg_len = baseline_tracker.avg_track_length();
    let imu_avg_len = imu_tracker.avg_track_length();
    let pred_error = imu_tracker.prediction_error_px();

    tracking.add_frame(
        frame_idx,
        baseline_features,
        imu_aided_features,
        baseline_avg_len,
        imu_avg_len,
        pred_error
    );
}

// Export and analyze
tracking.export_csv("tracking_comparison.csv")?;
let stats = tracking.compute_stats();
println!("Improvement: {:.1}%", stats.improvement_percent);
```

**Metrics Tracked**:
- Feature count (with/without IMU)
- Average track length (persistence)
- IMU prediction error (pixels)

**Expected Results**:
- ×2–3 increase in tracked features during rotation
- ×1.5–2 longer track persistence
- Prediction error < 2 pixels

### 2. Disparity Comparison

```rust
let mut disparity = DisparityComparison::new();

// For each feature, compare integer vs sub-pixel disparity
for (feature_id, feature) in features.iter().enumerate() {
    let int_disp = feature.integer_disparity();
    let sub_disp = feature.subpixel_disparity();

    // Compute depths (baseline * focal / disparity)
    let int_depth = (baseline * focal) / int_disp;
    let sub_depth = (baseline * focal) / sub_disp;

    // Photometric errors
    let int_err = compute_photometric_error(feature, int_disp);
    let sub_err = compute_photometric_error(feature, sub_disp);

    disparity.add_feature(
        feature_id,
        int_disp, sub_disp,
        int_depth, sub_depth,
        int_err, sub_err
    );
}

disparity.export_csv("disparity_comparison.csv")?;
let stats = disparity.compute_stats();
println!("Error reduction: {:.1}%", stats.error_reduction_percent);
```

**Metrics Tracked**:
- Integer vs sub-pixel disparity
- Depth accuracy improvement
- Photometric error reduction

**Expected Results**:
- ×5–10 depth accuracy improvement
- 50–70% photometric error reduction
- Depth variance σ(Z) ~ Z² behavior preserved but shifted down

### 3. Rolling Shutter Comparison

```rust
let mut rs_comp = RollingShutterComparison::new();

// For each frame, compare GS vs RS correction
for frame_idx in 0..num_frames {
    let ang_vel = imu.angular_velocity_magnitude();

    // Compute reprojection errors with both models
    let gs_error = compute_reprojection_error_gs(frame);
    let rs_error = compute_reprojection_error_rs(frame);
    let feature_count = frame.features.len();

    rs_comp.add_frame(
        frame_idx,
        ang_vel,
        gs_error,
        rs_error,
        feature_count
    );
}

rs_comp.export_csv("rolling_shutter_comparison.csv")?;
let stats = rs_comp.compute_stats();
println!("Error reduction: {:.1}%", stats.error_reduction_percent);
println!("High-vel improvement: {:.2} px → {:.2} px",
         stats.high_velocity_gs_error_px,
         stats.high_velocity_rs_error_px);
```

**Metrics Tracked**:
- Angular velocity (rad/s)
- Reprojection error (GS model)
- Reprojection error (RS corrected)
- Feature count stability

**Expected Results**:
- ×2 error reduction overall
- ×3–5 improvement at high angular velocity (>0.5 rad/s)
- Error no longer grows with rotation rate

## Generated Plots

### 1. Tracking Comparison (`tracking_comparison.png`)

Four subplots:
- **Top-left**: Feature count over time (baseline vs IMU-aided)
- **Top-right**: Average track length (persistence metric)
- **Bottom-left**: Improvement percentage per frame
- **Bottom-right**: IMU prediction error accuracy

### 2. Disparity Comparison (`disparity_comparison.png`)

Four subplots:
- **Top-left**: Integer vs sub-pixel disparity scatter
- **Top-right**: Integer vs sub-pixel depth accuracy
- **Bottom-left**: Photometric error distribution
- **Bottom-right**: Sub-pixel offset histogram

### 3. Rolling Shutter Comparison (`rolling_shutter_comparison.png`)

Four subplots:
- **Top-left**: Error vs angular velocity (GS model)
- **Top-right**: Error vs angular velocity (RS corrected)
- **Bottom-left**: Error over time (direct comparison)
- **Bottom-right**: Error reduction percentage

## Statistical Output

Each comparison computes summary statistics:

```rust
// Tracking
pub struct TrackingStats {
    pub avg_baseline_features: f64,      // Average feature count (no IMU)
    pub avg_imu_aided_features: f64,     // Average feature count (with IMU)
    pub improvement_percent: f64,        // % increase
    pub avg_prediction_error_px: f64,    // IMU prediction accuracy
}

// Disparity
pub struct DisparityStats {
    pub avg_depth_difference_m: f64,     // Mean depth error improvement
    pub avg_integer_error_px: f64,       // Photometric error (integer)
    pub avg_subpixel_error_px: f64,      // Photometric error (sub-pixel)
    pub error_reduction_percent: f64,    // % improvement
}

// Rolling Shutter
pub struct RollingShutterStats {
    pub avg_gs_error_px: f64,                 // Mean GS error
    pub avg_rs_error_px: f64,                 // Mean RS error
    pub error_reduction_percent: f64,         // % improvement
    pub high_velocity_gs_error_px: f64,       // GS error at high rotation
    pub high_velocity_rs_error_px: f64,       // RS error at high rotation
}
```

## Validation Metrics

Use these thresholds to validate implementations:

### IMU-Aided Tracking
- ✅ **Good**: 20–40% feature count increase
- ✅ **Excellent**: >40% increase
- ✅ **Prediction error**: <2 pixels RMS
- ❌ **Problem**: <10% improvement or >3 pixel error

### Sub-Pixel Disparity
- ✅ **Good**: 40–60% error reduction
- ✅ **Excellent**: >60% error reduction
- ✅ **Depth variance**: Clearly tighter σ(Z)
- ❌ **Problem**: <30% improvement

### Rolling Shutter Correction
- ✅ **Good**: 40–60% error reduction
- ✅ **Excellent**: >60% error reduction at high ω
- ✅ **Stability**: Error flat vs angular velocity
- ❌ **Problem**: <30% improvement or error still grows with ω

## Integration with Real Data

To use with real sensor data:

```rust
// Read your dataset
let images = load_images("dataset/images")?;
let imu_data = load_imu("dataset/imu.csv")?;

// Process with both methods
let mut tracking = TrackingComparison::new();
let baseline_tracker = KLTTracker::new();
let imu_tracker = IMUAidedTracker::new();

for (idx, image) in images.iter().enumerate() {
    // Baseline processing
    baseline_tracker.track_features(&image);

    // IMU-aided processing
    let imu_window = get_imu_measurements(&imu_data, idx);
    imu_tracker.add_imu_measurements(imu_window);
    imu_tracker.track_features(&image);

    // Record comparison
    tracking.add_frame(/* ... */);
}

// Generate visualizations
tracking.export_csv("results/tracking.csv")?;
generate_plot_script("results")?;
```

## Customizing Plots

The generated Python script can be customized:

```python
# Edit plot_output/plot_comparisons.py

# Change figure size
fig, axes = plt.subplots(2, 2, figsize=(16, 12))  # Larger

# Change colors
axes[0, 0].plot(..., color='#1f77b4', linewidth=3)

# Add grid
axes[0, 0].grid(True, alpha=0.5, linestyle='--')

# Save different format
plt.savefig('output.pdf', dpi=300)  # PDF instead of PNG
```

## Troubleshooting

### No visible improvement

**Problem**: Plots show minimal difference between baseline and optimized

**Solutions**:
- Check time synchronization (IMU ↔ camera)
- Verify calibration quality (intrinsics, extrinsics, time offset)
- Ensure motion is sufficient (need rotation for IMU-aided, parallax for disparity)
- Check for implementation bugs (run unit tests)

### Python plotting errors

**Problem**: `ModuleNotFoundError` or import errors

**Solution**:
```bash
pip install --upgrade pandas matplotlib numpy
# Or use conda
conda install pandas matplotlib numpy
```

### Large prediction errors

**Problem**: IMU prediction error >5 pixels

**Solutions**:
- Check IMU calibration (bias, scale, noise)
- Verify time offset calibration
- Check for IMU saturation/clipping
- Reduce time between frames

## Advanced Usage

### Real-time Monitoring

```rust
// Create live dashboard
use std::time::Instant;

let mut last_report = Instant::now();
let report_interval = Duration::from_secs(1);

loop {
    // Process frame...

    if last_report.elapsed() >= report_interval {
        let stats = tracking.compute_stats();
        println!("Live stats: improvement={:.1}%, error={:.2}px",
                 stats.improvement_percent,
                 stats.avg_prediction_error_px);
        last_report = Instant::now();
    }
}
```

### Batch Processing

```rust
// Process multiple sequences
for sequence in &["indoor", "outdoor", "fast_motion"] {
    let mut tracking = TrackingComparison::new();
    // ... process ...
    tracking.export_csv(&format!("results/{}_tracking.csv", sequence))?;
}

// Generate combined plot
generate_plot_script("results")?;
```

## References

- Implementation details: [PHASE_2_IMPLEMENTATION_COMPLETE.md](PHASE_2_IMPLEMENTATION_COMPLETE.md)
- API reference: [PHASE_2_QUICK_REFERENCE.md](PHASE_2_QUICK_REFERENCE.md)
- Theoretical background: See user-provided optimization guide
