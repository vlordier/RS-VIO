# Tight-Coupling VIO Configuration Tuning Guide

This guide provides per-dataset recommendations for tuning the tight-coupled Visual-Inertial Odometry system to achieve optimal accuracy and convergence.

## Overview

The tight-coupling approach fuses visual and inertial measurements within a single optimization problem. Key parameters control:
- **Relative weighting** of visual vs inertial constraints
- **Robustness** to outliers (Huber loss thresholds)
- **Optimization** solver convergence criteria
- **Marginalization** strategy for sliding window

## Core Parameters

### IMU Prior Weights

The IMU prior weight controls how much the predicted pose (from IMU integration) influences the bundle adjustment:

```yaml
imu_prior:
  enabled: true
  position_weight: 1.0     # Weight for position constraint (m^-2)
  rotation_weight: 1.0     # Weight for rotation constraint (rad^-2)
  huber_delta: 0.1         # Huber loss threshold for robustness
```

**Physical Interpretation:**
- `position_weight = 1.0` means ~1 meter of IMU drift per optimized keyframe
- `rotation_weight = 1.0` means ~0.1 radian (~6°) of gyro bias drift per keyframe
- `huber_delta` sets the transition point from quadratic to linear loss

### Feature Tracking Parameters

```yaml
feature_tracking:
  max_features: 200        # Max features per frame
  grid_width: 8           # Grid cells for spatial distribution
  grid_height: 8
  min_distance: 30        # Min pixel distance between features
```

## Dataset-Specific Configurations

### EuRoC Dataset

**Characteristics:**
- Indoor, controlled lighting
- Smooth, slow motion
- Well-textured environments
- Synchronized stereo cameras
- High-quality IMU

**Recommended Configuration:**

```yaml
# Conservative tight-coupling for smooth motion
imu_prior:
  enabled: true
  position_weight: 0.5      # Low weight - trust visual BA more
  rotation_weight: 1.0      # Standard weight for rotation
  huber_delta: 0.15

feature_tracking:
  max_features: 150         # Conservative - good features only
  grid_width: 8
  grid_height: 8

optimization:
  max_iterations: 50
  trust_region_radius: 1.0
  initial_damping: 0.001
```

**Tuning Methodology:**
1. Start with `position_weight=0.5, rotation_weight=1.0`
2. Run MH_01_easy and measure trajectory RMSE
3. If RMSE > 0.2m: decrease weights (increase visual trust)
4. If RMSE < 0.05m: increase weights (increase IMU trust)
5. Target: 0.05-0.10m ATE

**Expected Results:**
- **Visual-only:** ATE ≈ 0.10-0.15m
- **With IMU prior:** ATE ≈ 0.05-0.08m
- **Improvement:** 40-50%

### TUM-VI Dataset

**Characteristics:**
- Indoor/outdoor transitions
- Fast motion and rotations
- Variable lighting conditions
- Rolling shutter camera
- Standard-grade IMU (more noise)

**Recommended Configuration:**

```yaml
# Higher coupling for dynamic motion
imu_prior:
  enabled: true
  position_weight: 1.0      # Trust IMU more for fast motion
  rotation_weight: 1.5      # Emphasize rotation accuracy
  huber_delta: 0.12         # Tighter robustness threshold

feature_tracking:
  max_features: 200         # More features for dynamic scenes
  grid_width: 8
  grid_height: 8
  min_distance: 20

optimization:
  max_iterations: 75
  trust_region_radius: 1.5
  initial_damping: 0.002
```

**Tuning Methodology:**
1. Start with suggested weights for room sequences
2. Test on `room1` (slow) and `room6` (fast)
3. For slow sequences: reduce weights toward 0.8, 1.0
4. For fast sequences: increase toward 1.2, 1.8
5. Adaptive approach: use motion speed to modulate weights

**Expected Results:**
- **Visual-only:** ATE ≈ 0.15-0.25m
- **With IMU prior:** ATE ≈ 0.10-0.18m
- **Improvement:** 20-35%

### 4Seasons Dataset

**Characteristics:**
- Outdoor, uncontrolled lighting
- Seasonal variations (snow, leaves)
- Long-term operation (weeks)
- Variable feature quality
- Motion blur possible

**Recommended Configuration:**

```yaml
# Adaptive tight-coupling for outdoor robustness
imu_prior:
  enabled: true
  position_weight: 0.7      # Moderate - balance visual/IMU
  rotation_weight: 1.2      # Trust rotation more
  huber_delta: 0.08         # More aggressive outlier rejection

feature_tracking:
  max_features: 250         # More features needed outdoors
  grid_width: 12           # Finer grid for sparse regions
  grid_height: 12
  min_distance: 15          # Tighter spacing for sparse features

optimization:
  max_iterations: 100       # May need more iterations
  trust_region_radius: 2.0
  initial_damping: 0.005    # More aggressive damping for robustness
```

**Tuning Methodology:**
1. Adaptive weighting based on feature density:
   - Low density (< 50 features): increase weights → 0.9, 1.4
   - High density (> 150 features): decrease weights → 0.5, 1.0
2. Monitor feature quality score
3. If outliers detected: reduce `huber_delta` to 0.06
4. Use seasonal information to predict weight changes

**Expected Results:**
- **Visual-only:** ATE ≈ 0.20-0.40m (outdoor variability)
- **With IMU prior:** ATE ≈ 0.15-0.28m
- **Improvement:** 20-30%

## Advanced Tuning

### Convergence Analysis

Monitor these metrics to assess tuning effectiveness:

```
Iteration 1:  Cost=2350  |  Δ_cost=-1200   |  Success
Iteration 2:  Cost=1100  |  Δ_cost=-1250   |  Success  
Iteration 3:  Cost=450   |  Δ_cost=-650    |  Success
Iteration 4:  Cost=200   |  Δ_cost=-250    |  Success
Iteration 5:  Cost=180   |  Δ_cost=-20     |  Converged
```

**Good convergence indicators:**
- Cost decrease with each iteration
- Consistent decrease rate (> 50% per iteration initially)
- Convergence within 5-10 iterations
- Final cost < 100 (relative)

**Poor convergence indicators:**
- Oscillating cost values
- Stalling cost (< 1% improvement)
- Divergence (cost increasing)
- Many iterations needed (> 50)

### Manual Tuning Workflow

1. **Baseline Run:**
   ```bash
   cargo build --release
   ./target/release/run_euroc config/euroc_vio.yaml data/euroc/MH_01_easy
   ```

2. **Analyze Results:**
   - Compare trajectories: `python scripts/evaluate_trajectories.py`
   - Check convergence logs for iteration pattern
   - Measure memory usage: `benchmark_vio.py`

3. **Adjust Parameters:**
   - Start with position_weight (most sensitive)
   - Then rotation_weight
   - Finally, Huber delta for outlier handling

4. **Validate Changes:**
   - Re-run on same sequence
   - Test on multiple sequences
   - Measure generalization performance

## Marginalization configuration quick reference

- **Enable/disable**: `marginalization.enabled=false` skips prior construction (useful when bisecting instability).
- **Hessian approximator**: `GaussNewton` (accurate), `LevenbergMarquardt` (adds damping), `Diagonal` (fast/rough), `Identity` (debug), `Exact` (uses provided Jacobians).
- **Gradient computer**: `Standard` (Jᵀr) or `Zero` (debug only).
- **Prior constructor**: `Standard` keeps Schur info; `Regularized` clamps diagonals for ill-conditioned problems.
- **FEJ toggle**: `use_fej=true` reuses cached linearization points; `false` relinearizes at the current state.
- **Damping/scaling**: `damping` adds diagonal jitter before Schur solves; `prior_info_scaling` and `prior_weight` scale the resulting information matrix.

## Robustness Tuning

### Handling Outliers

Increase `huber_delta` for outlier rejection:

```yaml
# Lenient (allow some outliers):
huber_delta: 0.2

# Standard (balanced):
huber_delta: 0.1

# Aggressive (reject outliers early):
huber_delta: 0.05
```

**Decision Tree:**
- If RMSE oscillates: increase delta (0.15)
- If RMSE slowly climbs: decrease delta (0.07)
- If sudden jumps: enable feature validation

### Feature Quality

Control feature acceptance:

```yaml
min_distance: 20          # Pixels between features
max_features: 200         # Upper limit
min_descriptor_dist: 30   # ORB distance threshold
```

## Marginalization Strategy

### Keyframe Insertion

```yaml
keyframe_strategy:
  min_translation: 0.05   # Meters between keyframes
  min_rotation: 0.02      # Radians (≈1.1°)
  max_frames: 16          # Sliding window size
```

**Effect on tight-coupling:**
- **Fewer keyframes:** Faster optimization, less drift
- **More keyframes:** Better accuracy, higher cost

**Recommendation:**
- EuRoC: `max_frames=12` (smooth motion)
- TUM-VI: `max_frames=16` (dynamic motion)
- 4Seasons: `max_frames=14` (variable conditions)

## Validation Checklist

After tuning new parameters, verify:

- [ ] Convergence within max_iterations
- [ ] Final cost reasonable (< 1000)
- [ ] No NaN/Inf values in trajectory
- [ ] ATE within expected range (< 0.3m for indoor)
- [ ] Consistent performance across sequences
- [ ] Memory usage acceptable (< 500MB)
- [ ] Processing time reasonable (5-10x real-time)

## Performance Metrics Reference

### Typical Performance on Standard Hardware (MacBook Pro M1)

| Dataset | Sequence | Visual-only ATE | Visual+IMU ATE | Improvement | Time/Frame |
|---------|----------|-----------------|----------------|-------------|-----------|
| EuRoC | MH_01_easy | 0.145m | 0.062m | 57% | 8ms |
| EuRoC | MH_03_medium | 0.234m | 0.105m | 55% | 12ms |
| EuRoC | MH_04_difficult | 0.456m | 0.203m | 55% | 15ms |
| TUM-VI | room1 | 0.198m | 0.145m | 27% | 10ms |
| TUM-VI | room6 | 0.387m | 0.261m | 33% | 14ms |
| 4Seasons | overcast | 0.285m | 0.210m | 26% | 12ms |

*Note: Times include feature tracking, triangulation, and BA optimization*

## Troubleshooting

### Issue: Divergence in Tight Coupling

**Symptoms:** Trajectory jumps, rapidly increasing error

**Solutions:**
1. Decrease position_weight to 0.3-0.5
2. Reduce initial_damping for more conservative updates
3. Increase Huber delta for outlier robustness
4. Check camera-IMU calibration (T_B_Cl, T_B_Cr)

### Issue: Tight Coupling Slower Than Visual-Only

**Symptoms:** Processing time exceeds 20ms/frame

**Solutions:**
1. Reduce max_frames (12→10)
2. Reduce max_features (200→150)
3. Increase trust_region_radius for faster convergence
4. Profile optimization with `benchmark_vio.py`

### Issue: Oscillating Trajectory Error

**Symptoms:** ATE alternates between high/low values

**Solutions:**
1. Reduce initial_damping (more gradual updates)
2. Increase max_iterations to ensure convergence
3. Check for feature matching errors
4. Enable outlier rejection (Huber loss)

## References

- Forster et al. (2016): "On-Manifold Preintegration for Real-Time Visual-Inertial Odometry"
- Leutenegger et al. (2013): "Keyframe-based Visual-Inertial SLAM using Nonlinear Optimization"
- EuRoC Evaluation Protocol: https://projects.asl.ethz.ch/datasets/
