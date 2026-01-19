# Calibration-Aware Distance/Speed Metrics

## Overview

Task 4 adds **calibration-aware performance tracking** to the distance/speed metrics system. This measures how calibration quality affects tracking accuracy across different distance and speed conditions.

The key insight: *Calibration quality directly impacts fusion confidence, which should improve residual accuracy and feature tracking stability.*

## What's New

### CalibrationAwareMetrics

Main metrics container that compares **weighted vs unweighted** performance:

```rust
pub struct CalibrationAwareMetrics {
    pub weighted_residuals: WeightedResidualStats,      // With calibration weighting
    pub unweighted_residuals: WeightedResidualStats,    // Baseline (no weighting)
    pub track_survival: TrackSurvivalStats,              // Feature survival rates
    pub distance_bin_improvements: DistanceBinnedImprovement,
    pub speed_bin_improvements: SpeedBinnedImprovement,
}
```

### WeightedResidualStats

Comprehensive residual analysis with outlier detection:

```rust
pub struct WeightedResidualStats {
    pub visual_rms: f64,          // RMS of visual (reprojection) residuals
    pub imu_rms: f64,             // RMS of IMU preintegration residuals
    pub total_rms: f64,           // Combined RMS
    pub sample_count: usize,      // Number of measurements
    pub outlier_rate: f32,        // % of residuals > 3×median
    pub mean_residual: f64,
    pub median_residual: f64,
    pub std_residual: f64,
}

impl WeightedResidualStats {
    /// Calculate improvement percentage vs baseline
    pub fn improvement_vs(&self, baseline: &WeightedResidualStats) -> f32 {
        ((baseline.total_rms - self.total_rms) / baseline.total_rms * 100.0) as f32
    }
}
```

**Interpretation**:
- If `weighted_rms < unweighted_rms`: Calibration weighting helped
- `improvement_vs()` shows % reduction: 10% = 10% better accuracy
- `outlier_rate` should be ~5% (normal) or lower (good gating)

### TrackSurvivalStats

Feature tracking longevity metrics:

```rust
pub struct TrackSurvivalStats {
    pub median_track_length: f32,  // Median # of frames a feature survives
    pub mean_track_length: f32,
    pub max_track_length: usize,
    pub survival_rate_5: f32,      // % of features surviving > 5 frames
    pub survival_rate_10: f32,     // % of features surviving > 10 frames
    pub total_features: usize,
}
```

**Interpretation**:
- Higher median/mean = better feature stability (IMU-aided tracking working)
- `survival_rate_5` > 50% is good for typical VIO
- IMU initialization should increase survival by 20-30%

### Distance Bin Improvements

Per-distance-range performance:

```rust
pub struct DistanceBinnedImprovement {
    pub near_field: (f64, f64, f32),    // (weighted_rms, unweighted_rms, improvement %)
    pub mid_field: (f64, f64, f32),     // 1-3 m
    pub far_field: (f64, f64, f32),     // 3-10 m
    pub very_far_field: (f64, f64, f32), // 10m+
}
```

**Expected patterns**:
- Near field: High improvement (50-70%) because calibration dominates at close range
- Far field: Lower improvement (10-20%) because depth uncertainty dominates

### Speed Bin Improvements

Per-motion-speed performance:

```rust
pub struct SpeedBinnedImprovement {
    pub static_scene: (f64, f64, f32),       // < 0.1 m/s
    pub slow_motion: (f64, f64, f32),        // 0.1-0.5 m/s
    pub normal_motion: (f64, f64, f32),      // 0.5-2.0 m/s
    pub fast_motion: (f64, f64, f32),        // 2.0-5.0 m/s
    pub very_fast_motion: (f64, f64, f32),   // 5.0+ m/s
}
```

**Expected patterns**:
- Fast motion: Higher improvement (40-60%) because RS correction matters
- Static: Moderate improvement (20-35%) - less motion uncertainty, more reliance on calibration

## CalibrationAwareAnalyzer

Main API for collecting and analyzing metrics:

```rust
pub struct CalibrationAwareAnalyzer {
    // ... internal
}

impl CalibrationAwareAnalyzer {
    pub fn new() -> Self
    
    /// Record weighted residuals (with calibration confidence applied)
    pub fn record_weighted_visual_residual(&mut self, distance: f64, speed: f32, residual: f32)
    pub fn record_weighted_imu_residual(&mut self, distance: f64, speed: f32, residual: f32)
    
    /// Record unweighted residuals (for baseline comparison)
    pub fn record_unweighted_visual_residual(&mut self, distance: f64, speed: f32, residual: f32)
    pub fn record_unweighted_imu_residual(&mut self, distance: f64, speed: f32, residual: f32)
    
    /// Record feature track length
    pub fn record_track_length(&mut self, length: usize)
    
    /// Generate comprehensive analysis
    pub fn analyze(&self) -> CalibrationAwareMetrics
}
```

## Usage Example

### Basic Workflow

```rust
use rs_vio::evaluation::{CalibrationAwareAnalyzer, CalibrationAwareMetrics};

fn main() {
    let mut analyzer = CalibrationAwareAnalyzer::new();
    
    // During VIO tracking loop
    for frame in frames {
        for feature in &frame.features {
            let distance = landmark.position.norm();
            let speed = state.velocity.norm() as f32;
            
            // Compute residuals with and without calibration weighting
            let unweighted_residual = reprojection_error(feature);
            let weighted_residual = unweighted_residual * calibration_weight;
            
            // Record for distance bin
            analyzer.record_weighted_visual_residual(distance, speed, weighted_residual);
            analyzer.record_unweighted_visual_residual(distance, speed, unweighted_residual);
        }
        
        // Track feature survival
        for track in &feature_manager.active_tracks {
            if track.frame_count > 0 {
                analyzer.record_track_length(track.frame_count);
            }
        }
    }
    
    // Generate report
    let metrics = analyzer.analyze();
    print_report(&metrics);
}

fn print_report(metrics: &CalibrationAwareMetrics) {
    println!("=== CALIBRATION AWARENESS IMPACT ===\n");
    
    println!("OVERALL RESIDUAL RMS:");
    println!("  Weighted (with calibration):   {:.4} px", metrics.weighted_residuals.total_rms);
    println!("  Unweighted (baseline):         {:.4} px", metrics.unweighted_residuals.total_rms);
    println!("  Improvement:                   {:.1}%", 
        metrics.weighted_residuals.improvement_vs(&metrics.unweighted_residuals));
    
    println!("\nVISUAL vs IMU BALANCE:");
    println!("  Visual RMS (weighted):         {:.4} px", metrics.weighted_residuals.visual_rms);
    println!("  IMU RMS (weighted):            {:.4} rad/s", metrics.weighted_residuals.imu_rms);
    println!("  Outlier rate (weighted):       {:.1}%", metrics.weighted_residuals.outlier_rate);
    
    println!("\nFEATURE TRACKING QUALITY:");
    println!("  Median track length:           {:.1} frames", metrics.track_survival.median_track_length);
    println!("  Mean track length:             {:.1} frames", metrics.track_survival.mean_track_length);
    println!("  Features lasting > 5 frames:   {:.1}%", metrics.track_survival.survival_rate_5);
    println!("  Features lasting > 10 frames:  {:.1}%", metrics.track_survival.survival_rate_10);
    
    println!("\nDISTANCE BIN IMPROVEMENTS:");
    let (w, u, imp) = metrics.distance_bin_improvements.near_field;
    println!("  Near field (0-1m):             {:.4} px → {:.4} px ({:.1}%)", u, w, imp);
    
    let (w, u, imp) = metrics.distance_bin_improvements.mid_field;
    println!("  Mid field (1-3m):              {:.4} px → {:.4} px ({:.1}%)", u, w, imp);
    
    let (w, u, imp) = metrics.distance_bin_improvements.far_field;
    println!("  Far field (3-10m):             {:.4} px → {:.4} px ({:.1}%)", u, w, imp);
    
    let (w, u, imp) = metrics.distance_bin_improvements.very_far_field;
    println!("  Very far (10m+):               {:.4} px → {:.4} px ({:.1}%)", u, w, imp);
    
    println!("\nSPEED BIN IMPROVEMENTS:");
    let (w, u, imp) = metrics.speed_bin_improvements.static_scene;
    println!("  Static (<0.1 m/s):             {:.4} px → {:.4} px ({:.1}%)", u, w, imp);
    
    let (w, u, imp) = metrics.speed_bin_improvements.normal_motion;
    println!("  Normal (0.5-2.0 m/s):          {:.4} px → {:.4} px ({:.1}%)", u, w, imp);
    
    let (w, u, imp) = metrics.speed_bin_improvements.very_fast_motion;
    println!("  Very fast (5.0+ m/s):          {:.4} px → {:.4} px ({:.1}%)", u, w, imp);
}
```

## Integration with Fusion Algorithm

How to use `CalibrationAwareAnalyzer` in your VIO pipeline:

### 1. During Feature Tracking

```rust
let fusion = OptimizedFusionAlgorithm::with_calibration(config, calibration_result);

for frame in frames {
    for feature in &frame.features {
        let distance = triangulated_landmark.position.norm();
        let speed = state.velocity.norm() as f32;
        
        // Get calibration-aware weights
        let visual_weight = fusion.visual_residual_weight();  // e.g., 0.8
        let imu_weight = fusion.imu_residual_weight();        // e.g., 0.75
        
        // Compute residuals
        let visual_residual = compute_reprojection_error(feature);
        let imu_residual = compute_imu_error(state);
        
        // Apply weighting
        let weighted_visual = visual_residual * visual_weight as f32;
        let weighted_imu = imu_residual * imu_weight as f32;
        
        // Record for analysis
        analyzer.record_weighted_visual_residual(distance, speed, weighted_visual);
        analyzer.record_unweighted_visual_residual(distance, speed, visual_residual);
        
        analyzer.record_weighted_imu_residual(distance, speed, weighted_imu);
        analyzer.record_unweighted_imu_residual(distance, speed, imu_residual);
    }
}
```

### 2. Track Feature Survival

```rust
// Update feature manager
for (feature_id, track) in &feature_manager.tracks {
    if !track.active {
        // Record final track length when feature dies
        analyzer.record_track_length(track.frame_count);
    }
}
```

### 3. Interpret Results

**Good calibration scenario** (excellent score 0.95):
```
Weighted RMS:    0.28 px
Unweighted RMS:  0.42 px
Improvement:     33% ✓

Median track:    8.5 frames
Survival > 5:    68%
Survival > 10:   45%
```

**Poor calibration scenario** (poor score 0.30):
```
Weighted RMS:    0.51 px
Unweighted RMS:  0.54 px
Improvement:     6% ✗ (weighting doesn't help much)

Median track:    3.2 frames
Survival > 5:    18%
Survival > 10:   2%
```

## Validation Targets (From User's Guide)

The guide says these are the minimal acceptance metrics:

| Metric | Baseline | With Good Cal | Target |
|--------|----------|---------------|--------|
| **Feature track survival** | 3-4 frames | 8-10 frames | ×2-3× improvement |
| **Reprojection RMS (static)** | 0.5-0.7 px | 0.25-0.35 px | ×2-2.5× improvement |
| **Reprojection RMS (fast)** | 0.8-1.2 px | 0.4-0.6 px | ×2-2.5× improvement |
| **Depth stability** | σ(Z) = 0.05Z | σ(Z) = 0.02Z | ×2.5× improvement |

## Test Cases

All test cases included validate:

1. **test_calibration_aware_metrics**: Weighted < unweighted
2. **test_track_survival_computation**: Track length statistics
3. **test_distance_bin_improvements**: Per-distance performance
4. **test_speed_bin_improvements**: Per-speed performance
5. **test_outlier_detection**: Outlier rate calculation
6. **test_distance_binning**: Original distance metrics (unchanged)
7. **test_speed_binning**: Original speed metrics (unchanged)

## Files Changed

**New components in src/evaluation/distance_speed_metrics.rs**:
- `CalibrationAwareMetrics` (20 lines)
- `WeightedResidualStats` (50 lines)
- `TrackSurvivalStats` (30 lines)
- `DistanceBinnedImprovement` (20 lines)
- `SpeedBinnedImprovement` (20 lines)
- `CalibrationAwareAnalyzer` (400+ lines)
- 5 new comprehensive tests

**Updated src/evaluation/mod.rs**:
- Export CalibrationAwareMetrics, CalibrationAwareAnalyzer, and helper types

**Total additions**: ~600 lines (analyzer + comprehensive tests)

## Performance Notes

- `CalibrationAwareAnalyzer` uses Vec storage (no allocations per record, just appends)
- `analyze()` is O(n log n) due to sorting for median computation
- Memory usage: ~40 bytes per residual sample + ~100 bytes for tracks
- Suitable for real-time use: compute metrics post-session, not per-frame

## Next Steps

1. **Integration with BA/Fusion**: Call analyzer in sliding window BA loop
2. **Visualization**: Plot distance/speed heatmaps showing improvement
3. **Thresholding**: Define "good calibration" based on improvement thresholds
4. **Closed-loop control**: Recalibrate if improvement < 20% across bins

## References

- `CALIBRATION_INTEGRATION_COMPLETE.md`: How calibration quality affects confidence
- User's guide: Minimal acceptance metrics for each multiplier
- `src/evaluation/calibration_quality.rs`: How quality → confidence weighting
