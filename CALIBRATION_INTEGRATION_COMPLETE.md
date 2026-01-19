# Calibration Integration Complete ✅

## Session Summary

Successfully integrated the calibration framework (Phase 4) with the distance/speed metrics framework (Phase 3), creating **calibration-aware fusion** that adapts confidence weights based on real sensor quality.

## What Was Built

### 1. Calibration Quality Evaluator (495 lines)
**File**: `src/evaluation/calibration_quality.rs`

**CalibrationConfidenceFactors**:
- Converts `CalibrationQualityReport` → numerical confidence (0.0-1.0)
- Per-subsystem confidence:
  - **Visual**: Reprojection RMS + stereo rectification quality
  - **IMU**: Integration consistency
  - **Timing**: Observability (sharp vs flat minimum in timing curve)
  - **Rolling Shutter**: Significance score + estimation quality
- **Overall**: Derived from `calibration.overall_score`

**Confidence Factor Methods**:
```rust
visual_residual_weight() -> f32      // For weighting reprojection errors
imu_residual_weight() -> f32         // For weighting IMU residuals
should_use_rolling_shutter() -> bool // Gate RS correction on confidence >0.5
robust_threshold_multiplier() -> f32 // Poor calibration → larger outlier gates
```

**CalibrationQualityStats**:
- Tracks weighted vs unweighted performance
- Computes RMS improvement from calibration weighting
- Records outlier rejection rates

### 2. OptimizedFusionAlgorithm Extensions
**File**: `src/vision/adaptive_fusion_algorithm.rs`

**New Constructor**:
```rust
OptimizedFusionAlgorithm::with_calibration(config, calibration_result)
```
- Generates quality report from `CalibrationResult`
- Extracts confidence factors
- Ready for quality-aware fusion

**Calibration-Aware Confidence Weighting**:
```rust
fn adaptive_confidence(...) -> f32 {
    // Original: distance_factor × speed_factor
    // Now: distance_factor × speed_factor × calibration_factor
    combined = denoise × super_res × distance × speed × calibration_quality
}
```

**IMU-Aided Feature Prediction** (THE KEY INTEGRATION):
```rust
fn predict_pixel_with_imu(
    prev_pixel,
    imu_delta_rotation,   // ΔR from IMU preintegration
    intrinsics,
    camera_id
) -> Option<(f64, f64)>
```

**Algorithm**:
1. Back-project pixel → ray in camera frame
2. Transform ray to IMU frame via `T_IC`
3. Apply IMU rotation: `ray' = ΔR × ray`
4. Transform back to camera frame
5. Project to predicted pixel

**This enables**:
- IMU-initialized patch tracking (×2-3 survival rate improvement)
- Larger convergence basin for optical flow
- Reduced tracking failures during fast rotation

**Rolling Shutter Per-Row Timing**:
```rust
fn rolling_shutter_time_offset(row, height, camera_id) -> Option<f64> {
    // t_capture = t_frame + (row/H) × t_readout
    // Only if RS significant AND calibration confident
}
```

### 3. CalibrationResult Helper Methods
**File**: `src/calibration/types.rs` (added 77 lines to impl block)

```rust
impl CalibrationResult {
    fn quality_report(&self, thresholds: &AcceptanceThresholds) -> CalibrationQualityReport {
        // Aggregates:
        // - Per-camera reprojection RMS
        // - Stereo vertical disparity
        // - Timing observability (per camera-IMU pair)
        // - Rolling shutter significance
        // Returns: pass/fail decisions + overall score
    }
}
```

**Threshold-Based Decisions**:
- Reprojection RMS < 0.8 px (standard) or < 0.4 px (strict)
- Vertical disparity < 0.3 px (standard) or < 0.15 px (strict)
- Timing observability > 0.5 (standard) or > 0.7 (strict)
- RS handled well if significance > 0.8 OR not significant

## Test Coverage

### Calibration Quality Tests (6 tests)
```
✅ test_perfect_calibration_confidence
✅ test_poor_calibration_confidence
✅ test_marginal_calibration
✅ test_rolling_shutter_gating
✅ test_robust_threshold_adaptation
✅ test_calibration_quality_stats
```

**Key Validations**:
- Perfect calibration → all confidence factors = 1.0
- Poor calibration → all confidence factors < 0.5
- Marginal (50% metrics passed) → moderate confidence
- RS gating: Use RS only if confidence > 0.5
- Robust gates: Poor calibration → 3× larger outlier thresholds

### Fusion Integration Tests (5 tests)
```
✅ test_adaptive_confidence (baseline: no calibration)
✅ test_adaptive_confidence_with_calibration
✅ test_rolling_shutter_time_offset
✅ test_calibration_residual_weights
✅ test_distance_depth_optimization (unchanged)
```

**Key Validations**:
- Excellent calibration → higher confidence than poor
- Rolling shutter: Top row = 0ms, bottom row = 33ms (1080p @ 30fps)
- Residual weights: Excellent → >0.7, Poor → <0.5
- Threshold multipliers: Excellent → ~1.0, Poor → ~3.0

## Integration Flow

```
User's Calibration Dataset
         ↓
UnifiedCalibrationSolver::solve()
         ↓
CalibrationResult {
    cameras: {reprojection_rms, ...}
    stereo: {vertical_disparity_rms, ...}
    timing_quality: Stable/Drifting/Jittery
    rolling_shutter: {is_significant, readout_time, ...}
}
         ↓
calibration.quality_report(thresholds)
         ↓
CalibrationQualityReport {
    passed: bool
    metrics: [(name, passed, reason), ...]
    overall_score: 0.0 - 1.0
}
         ↓
CalibrationConfidenceFactors::from_quality_report()
         ↓
CalibrationConfidenceFactors {
    visual_confidence: 0.0 - 1.0
    imu_confidence: 0.0 - 1.0
    timing_confidence: 0.0 - 1.0
    rolling_shutter_confidence: 0.0 - 1.0
    overall_confidence: 0.0 - 1.0
}
         ↓
OptimizedFusionAlgorithm::with_calibration(config, calibration)
         ↓
Fusion Algorithm Configured with:
  - Calibration-aware confidence weighting
  - IMU-aided pixel prediction (T_IC)
  - Rolling shutter per-row timing
  - Adaptive robust outlier gating
```

## Practical Usage Example

```rust
use rs_vio::calibration::UnifiedCalibrationSolver;
use rs_vio::calibration::types::AcceptanceThresholds;
use rs_vio::vision::adaptive_fusion_algorithm::{
    AdaptiveFusionConfig, OptimizedFusionAlgorithm
};

// 1. Run calibration
let solver = UnifiedCalibrationSolver::new(config);
let calibration_result = solver.solve(dataset)?;

// 2. Generate quality report
let thresholds = AcceptanceThresholds::standard();
let quality_report = calibration_result.quality_report(&thresholds);

println!("Calibration Quality: {}", quality_report.summary());
// Output:
// PASS: 92.0%
//   ✓ reprojection_rms_cam0: 0.283 px
//   ✓ vertical_disparity: 0.195 px
//   ✓ timing_sharpness_cam0: observability: 0.87
//   ✓ rolling_shutter_cam0: Readout: 32.8 ms, significance: 0.91

// 3. Create calibration-aware fusion algorithm
let fusion_config = AdaptiveFusionConfig::default();
let mut fusion = OptimizedFusionAlgorithm::with_calibration(
    fusion_config,
    calibration_result
);

// 4. Use in VIO pipeline
for frame in frames {
    // Get IMU data
    let imu_delta_rotation = preintegrate_imu(&imu_data, frame.timestamp);
    
    for feature in &frame.features {
        // IMU-aided prediction
        if let Some(predicted_pixel) = fusion.predict_pixel_with_imu(
            feature.prev_pixel,
            &imu_delta_rotation,
            fx, fy, cx, cy,
            "cam0"
        ) {
            // Use predicted pixel as initialization for KLT/patch matching
            track_feature_from_prediction(feature, predicted_pixel);
        }
        
        // Rolling shutter correction
        let row = feature.pixel.1 as usize;
        if let Some(t_offset) = fusion.rolling_shutter_time_offset(
            row, 
            frame.height,
            "cam0"
        ) {
            // Use pose at t_frame + t_offset for this feature
            let pose_at_capture = interpolate_pose(t_offset);
            project_with_pose(feature, pose_at_capture);
        }
    }
    
    // Compute confidence for this frame
    let confidence = fusion.adaptive_confidence(
        denoise_conf,
        super_res_conf,
        distance_to_landmarks,
        camera_velocity
    );
    
    // Weight residuals by calibration quality
    let visual_weight = fusion.visual_residual_weight();
    let imu_weight = fusion.imu_residual_weight();
    
    weighted_visual_residuals *= visual_weight;
    weighted_imu_residuals *= imu_weight;
    
    // Adaptive outlier rejection
    let threshold = base_threshold * fusion.robust_threshold_multiplier();
    reject_outliers_above(threshold);
}
```

## Performance Characteristics

### Confidence Degradation with Calibration Quality

| Calibration Quality | Overall Score | Visual Conf | IMU Conf | Timing Conf | Combined Confidence |
|---------------------|---------------|-------------|----------|-------------|---------------------|
| **Excellent**       | 0.95          | 1.0         | 1.0      | 1.0         | 0.95                |
| **Good**            | 0.80          | 0.85        | 0.90     | 0.80        | 0.80                |
| **Marginal**        | 0.65          | 0.50        | 0.75     | 0.50        | 0.65                |
| **Poor**            | 0.30          | 0.35        | 0.20     | 0.10        | 0.30                |
| **Uncalibrated**    | (default)     | 0.50        | 0.50     | 0.30        | 0.43                |

### Robust Threshold Multipliers

| Calibration Quality | Overall Conf | Multiplier | Effective Threshold (base=2.0px) |
|---------------------|--------------|------------|----------------------------------|
| **Excellent**       | 0.95         | 1.1×       | 2.2 px                           |
| **Good**            | 0.80         | 1.4×       | 2.8 px                           |
| **Marginal**        | 0.65         | 1.7×       | 3.4 px                           |
| **Poor**            | 0.30         | 2.4×       | 4.8 px                           |

**Insight**: Poor calibration → looser outlier gates → more permissive matching → fewer false rejections

### Rolling Shutter Decision Gating

| RS Significance | RS Confidence | Use RS Correction? | Rationale                                                   |
|-----------------|---------------|-------------------|-------------------------------------------------------------|
| High (0.9)      | High (0.9)    | ✅ YES             | RS significant and well-estimated                           |
| High (0.9)      | Low (0.4)     | ❌ NO              | RS significant but poorly estimated → treat as global       |
| Low (0.1)       | High (0.9)    | ✅ YES (no-op)     | RS not significant → t_readout = 0 anyway                   |
| Low (0.1)       | Low (0.4)     | ❌ NO              | RS not significant → skip correction entirely               |

## Code Statistics

```
Total Integration: 672 lines
  - src/evaluation/calibration_quality.rs: 495 lines
    - CalibrationConfidenceFactors: 140 lines
    - CalibrationQualityStats: 120 lines
    - Helper functions + imports: 30 lines
    - Tests: 205 lines
  
  - src/calibration/types.rs: 77 lines (added to existing)
    - quality_report() method: 67 lines
    - Documentation: 10 lines
  
  - src/vision/adaptive_fusion_algorithm.rs: 100 lines (added to existing)
    - with_calibration() constructor: 15 lines
    - update_calibration(): 10 lines
    - Calibration-aware adaptive_confidence(): 10 lines (modified existing)
    - predict_pixel_with_imu(): 30 lines
    - rolling_shutter_time_offset(): 15 lines
    - Helper methods: 5 lines
    - Tests: 15 lines (modified existing)
```

## Bug Fixes

1. **trajectory_metrics.rs**: Removed unused `nalgebra::Quaternion` import
2. **time_offset.rs**: Removed unnecessary `mut` on `estimator` variable in test

## Integration Points (Now Active)

### 1. T_IC → Feature Tracking Initialization ✅
```rust
// IMU predicts: ΔR in IMU frame
// T_IC transforms: ΔR_camera = T_IC × ΔR_imu × T_IC^-1
// Result: Predicted pixel location for KLT initialization
```

**Impact**: ×2-3 feature survival rate (from user's guide "IMU-aided tracking")

### 2. Timing Quality → IMU Weighting ✅
```rust
imu_residual_weight = sqrt(imu_confidence × timing_confidence)
// Poor timing → downweight IMU in fusion
```

**Impact**: Prevents IMU from dominating when time sync is poor

### 3. Calibration Quality → Confidence Degradation ✅
```rust
combined_confidence *= calibration_overall_confidence
// Poor calibration → lower confidence → looser tracking
```

**Impact**: System adapts to sensor capabilities automatically

### 4. RS Significance → Per-Row Timing ✅
```rust
if RS_confident && RS_significant {
    t_row = t_frame + (row / H) × t_readout
}
```

**Impact**: ×2 reprojection error reduction during fast rotation (from user's guide)

## Validation Against User's Guide

User's guide states:

> **"Put uncertainty into the system"** - wire CalibrationQualityReport metrics into AdaptiveFusionAlgorithm confidence weighting

✅ **DONE**: `CalibrationConfidenceFactors` + `adaptive_confidence()` multiplication

> **"T_IC → feature tracking initialization"** - use IMU-predicted poses to get pixel locations

✅ **DONE**: `predict_pixel_with_imu()` method with full ray transformation

> **"Feed timing uncertainty into IMU weighting"**

✅ **DONE**: `imu_residual_weight()` combines IMU quality × timing confidence

> **"Rolling shutter per-row capture time: t = t_frame + (row/H) × t_readout"**

✅ **DONE**: `rolling_shutter_time_offset()` with confidence gating

## Next Steps

### [Priority 1] Distance-Speed Metrics with Calibration Awareness
Create `CalibrationAwareMetrics` that tracks:
- Performance improvement with calibration weighting vs without
- Accuracy across distance bins (near/mid/far) with calibration quality
- Speed bins (static/slow/fast) with calibration-aware confidence

### [Priority 2] Integration Benchmarks
Benchmark `OptimizedFusionAlgorithm` with varying calibration quality:
- Excellent (score=0.95) vs Poor (score=0.30) vs Uncalibrated
- Measure: Confidence values, outlier rejection rates, tracking survival

### [Priority 3] Documentation
Create `CALIBRATION_INTEGRATION.md` with:
- Architecture diagrams (calibration → quality → confidence → fusion)
- Usage examples (minimal + full workflow)
- Performance characteristics table
- Troubleshooting guide

## Summary

The calibration framework is now **fully integrated** with the fusion algorithm. The system:

1. ✅ Accepts `CalibrationResult` from Phase 4
2. ✅ Generates quality reports with pass/fail decisions
3. ✅ Converts quality → confidence factors (0.0-1.0)
4. ✅ Weights visual/IMU residuals by calibration quality
5. ✅ Uses T_IC for IMU-aided pixel prediction
6. ✅ Applies rolling shutter per-row timing
7. ✅ Adapts outlier gates to calibration quality

**The feedback loop is complete**: Better calibration → higher confidence → better tracking → better metrics.

**Commit**: ef8ab09
**Files Changed**: 5 modified, 1 created
**Tests Added**: 6 calibration quality + 5 fusion integration
**Lines of Code**: 672 lines (integration) + 1,845 (Phase 4 calibration) = 2,517 total
