# Fusion Quick-Start Guide

Get RS-VIO fusion up and running in 5 minutes.

## What is Fusion?

Fusion combines information from multiple video frames to improve feature quality and trajectory stability. RS-VIO provides three fusion strategies with different tradeoffs:

| Strategy | Overhead | Requirements | Use Case |
|----------|----------|--------------|----------|
| **Depth-Aware** | 2-3ms | Image capture | High accuracy (Jetson Xavier, i7+) |
| **Rotation-Only** | 0.5-1ms | IMU only | Mobile/edge platforms |
| **None** | 0ms | — | Real-time / benchmarking |

## Quick Start

### 1. Choose Your Strategy

```bash
# For maximum accuracy (high-end platform)
cp config/fusion_with_image_capture.yaml config/my_config.yaml

# For mobile/edge (Jetson Nano, ARM)
cp config/fusion_rotation_only.yaml config/my_config.yaml

# For baseline comparison
cp config/fusion_disabled_baseline.yaml config/my_config.yaml
```

### 2. Edit Camera Parameters

Open `config/my_config.yaml` and update:
- `camera.image_width` / `image_height`
- `camera.left_intrinsics` (fx, fy, cx, cy)
- `camera.left_distortion` (radial + tangential coefficients)
- `camera.T_B_Cl` / `T_B_Cr` (camera-to-body transforms)

See [config/fusion_with_image_capture.yaml](config/fusion_with_image_capture.yaml) for EuRoC example values.

### 3. Build & Run

```bash
# Build release (optimized)
cargo build --release

# Run with your config
./target/release/rs-vio config/my_config.yaml

# Or with test dataset
./target/release/rs-vio config/my_config.yaml --dataset euroc --sequence_dir /path/to/euroc_mh01
```

### 4. Verify It's Working

Check logs for fusion messages:
```
[INFO] Fusion strategy: DepthAwareFusion
[INFO] Fusion buffer size: 2 / 5
[INFO] Fused 250 features with confidence blending
```

## Configuration Reference

### For Depth-Aware Fusion

```yaml
debug:
  capture_left_image_for_fusion: true  # Enable image storage
  fusion_strategy: "depth-aware"        # Use depth-aware strategy
  use_imu: true                         # IMU required for depth hypotheses
```

**Performance**: ~2-3ms per frame (752×480 image)

**When to use**: High-end GPUs, servers, research platforms

### For Rotation-Only Fusion

```yaml
debug:
  capture_left_image_for_fusion: false  # No image needed
  fusion_strategy: "rotation"           # Use rotation stabilization
  use_imu: true                         # IMU required
```

**Performance**: ~0.5-1ms per frame

**When to use**: Mobile platforms, edge devices, latency-sensitive apps

### For Baseline (No Fusion)

```yaml
debug:
  capture_left_image_for_fusion: false
  fusion_strategy: "none"               # Disable fusion
  use_imu: true                         # IMU still used for pre-integration
```

**Performance**: No overhead

**When to use**: Benchmarking, real-time constraints, research baseline

## Latency Budget

Add fusion latency to your frame processing budget:

| Target FPS | Frame Time | Fusion | Total |
|-----------|-----------|--------|-------|
| 30 Hz | 33.3ms | 2-3ms | ~36ms ✓ |
| 20 Hz | 50ms | 0.5-1ms | ~51ms ✓ |
| 10 Hz | 100ms | 2-3ms | ~103ms ✓ |
| 5 Hz | 200ms | 2-3ms | ~203ms ✓ |

If your frame time is tight, use rotation-only or disable fusion.

## Common Issues

### Q: How do I disable fusion?
**A**: Set `fusion_strategy: "none"` in config debug section.

### Q: I'm getting "Image plane not available" errors
**A**: You're using depth-aware fusion but `capture_left_image_for_fusion: false`. Either:
- Change `capture_left_image_for_fusion: true`, OR
- Switch to `fusion_strategy: "rotation"`

### Q: How much does fusion improve accuracy?
**A**: Typical improvements are 5-10% on EuRoC (depth-aware) and 2-3% (rotation-only).

### Q: Can I use fusion on Jetson Nano?
**A**: Yes, use rotation-only fusion (`fusion_rotation_only.yaml`). Depth-aware may exceed your budget.

### Q: What if I only have gyro, no depth?
**A**: Use rotation-only fusion. It only needs IMU and doesn't require image analysis.

## Next Steps

1. **Benchmark**: Compare all three strategies on your dataset
   ```bash
   # Run each config and compare trajectory RMSE
   for config in fusion_disabled_baseline fusion_rotation_only fusion_with_image_capture; do
     ./target/release/rs-vio config/${config}.yaml --dataset your_dataset
   done
   ```

2. **Tune**: Adjust hyperparameters in [FUSION_ARCHITECTURE.md](FUSION_ARCHITECTURE.md)
   - `quality_threshold` for depth-aware patch quality cutoff
   - `gyro_uncertainty_threshold` for rotation confidence

3. **Integrate**: Use fusion results in your application
   ```rust
   // Feature confidence is automatically blended in processor
   let confidence = frame.left_features[i].quality.confidence;
   ```

## Resources

- **Architecture Deep-Dive**: [FUSION_ARCHITECTURE.md](FUSION_ARCHITECTURE.md)
- **Config Examples**:
  - [config/fusion_with_image_capture.yaml](config/fusion_with_image_capture.yaml) — depth-aware
  - [config/fusion_rotation_only.yaml](config/fusion_rotation_only.yaml) — rotation
  - [config/fusion_disabled_baseline.yaml](config/fusion_disabled_baseline.yaml) — baseline
- **Implementation**:
  - Depth-Aware: [src/fusion/depth_aware_fusion.rs](src/fusion/depth_aware_fusion.rs)
  - Rotation: [src/fusion/rotation_stabilizer.rs](src/fusion/rotation_stabilizer.rs)

---

**Want to contribute a new fusion strategy?** See "Adding a New Fusion Strategy" in [FUSION_ARCHITECTURE.md](FUSION_ARCHITECTURE.md).
