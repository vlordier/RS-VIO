# VIO Pipeline Configuration Guide

## Overview

RS-VIO now features a comprehensive, platform-aware configuration system that allows you to optimize the entire VIO pipeline for your specific hardware and performance requirements.

## Quick Start

### 1. Select a Platform Preset

Use the interactive configuration helper:

```bash
python3 scripts/configure_vio.py --interactive
```

Or select directly:

```bash
python3 scripts/configure_vio.py --platform cpu_only --recommend
```

### 2. Load Configuration in Code

```rust
use rs_vio::feature_tracker::VIOPipelineConfig;

// Load from file
let config = VIOPipelineConfig::load_toml("configs/cpu_only.toml")?;

// Validate before use
config.validate()?;

// Use with your tracker
```

### 3. Use Built-in Presets

```rust
// CPU-only (Raspberry Pi 5, ARM)
let config = VIOPipelineConfig::cpu_only();

// GPU-enabled (Jetson Nano/Xavier/Orin)
let config = VIOPipelineConfig::gpu_enabled();

// Hard realtime (60 FPS, minimal latency)
let config = VIOPipelineConfig::hard_realtime();

// Balanced (default)
let config = VIOPipelineConfig::balanced();
```

## Available Platform Configurations

### CPU-Only (Raspberry Pi 5 / ARM)

**Target**: 20 FPS, 256 MB RAM

**Optimizations**:
- Rotation-only frame stabilization (3 frames)
- Shi-Tomasi + KLT tracking
- ORB descriptors for loop closure
- Reduced pyramid levels (2)
- Track-first detection (100-200 features)

**Use when**:
- Embedded ARM processors
- Limited memory budget
- No GPU available
- Battery-powered systems

**Config file**: `configs/cpu_only.toml`

### GPU-Enabled (Jetson Nano/Xavier/Orin)

**Target**: 30 FPS, 1024 MB RAM

**Optimizations**:
- Adaptive fusion strategy
- Depth-aware fusion for challenging scenes
- SuperPoint descriptors on keyframes
- Higher feature counts (150-300)
- 3-level pyramids

**Use when**:
- GPU/NPU available
- Higher accuracy requirements
- Can afford more compute
- Loop closure critical

**Config file**: `configs/gpu_enabled.toml`

### Hard Realtime (Minimal Latency)

**Target**: 60 FPS, 128 MB RAM

**Optimizations**:
- No frame fusion (single frame)
- FAST detector (fastest available)
- No descriptors or loop closure
- Minimal features (80-150)
- Reduced disparity search (64 pixels)
- No bundle adjustment

**Use when**:
- Strict timing constraints
- Latency-critical applications
- Racing drones / high-speed flight
- Resource-constrained systems

**Config file**: `configs/hard_realtime.toml`

### Balanced (Default)

**Target**: 30 FPS, 512 MB RAM

**Optimizations**:
- Rotation-only stabilization (5 frames)
- Shi-Tomasi + track-first pattern
- ORB descriptors for matching
- 3-level pyramids
- 150-300 features

**Use when**:
- Typical drone applications
- Good quality/performance balance
- CPU-only with moderate resources
- Starting point for customization

**Config file**: `configs/balanced.toml`

## Configuration Components

### Performance Budget

Controls overall system resource usage:

```toml
[performance]
target_fps = 30.0
max_latency_ms = 33.0
max_cpu_usage = 0.75
max_memory_mb = 512
enable_gpu = false
```

### Fusion Strategy

Multi-frame geometric super-resolution:

```toml
[fusion]
strategy = "RotationOnly"  # None, RotationOnly, PlanarSe3, DepthAware, PatchLevel, Adaptive
num_frames = 5

[fusion.stabilizer]
enabled = true
buffer_size = 5
accumulation_weight = 0.7
min_rotation_threshold = 0.001
```

**Strategies**:
- `None`: No fusion, single frame processing
- `RotationOnly`: Gyro-based stabilization (recommended for drones)
- `PlanarSe3`: Homography-based warping
- `DepthAware`: Two-pass with stereo depth
- `PatchLevel`: Fusion around tracked features only
- `Adaptive`: Choose based on scene characteristics

### Feature Detection

```toml
[detection]
backend = "ShiTomasi"  # ShiTomasi, Fast, Agast, SuperPoint, Disk
tracking_strategy = "TrackFirst"  # DetectEveryFrame, TrackFirst, SemiDirect
min_quality = 0.01
use_pyramid = true
pyramid_levels = 3
enforce_spatial_distribution = true
grid_cell_size = 32

[detection.track_first]
min_features = 150
max_features = 300
grid_cell_size = 32
min_features_per_cell = 2
corner_quality_threshold = 0.01
min_feature_distance = 10.0
```

**Backends**:
- `ShiTomasi`: Robust, well-tested (best for CPU)
- `Fast`: Fastest detector, lower quality
- `Agast`: Adaptive FAST
- `SuperPoint`: Learned detector (requires GPU)
- `Disk`: Learned detector (requires GPU)

**Tracking Strategies**:
- `DetectEveryFrame`: Simple but expensive
- `TrackFirst`: Long tracks, detect only when needed (recommended)
- `SemiDirect`: Photometric alignment (experimental)

### Feature Matching

```toml
[matching]
descriptor_type = "Orb"  # None, Orb, Brief, Akaze, SuperPoint, LightGlue
keyframes_only = true
max_descriptor_distance = 50.0
use_ratio_test = true
ratio_test_threshold = 0.8
```

**Descriptor Types**:
- `None`: No matching (tracking-only)
- `Orb`: Fast binary descriptor (best for CPU)
- `Brief`: Very fast binary
- `Akaze`: More robust, slower
- `SuperPoint`: Learned descriptor (GPU)
- `LightGlue`: Learned matching (GPU)

### Stereo Configuration

```toml
[stereo]
baseline = 0.11  # meters
max_disparity = 128
subpixel_refinement = true
cost_window_size = 5
uniqueness_ratio = 0.15
texture_threshold = 10.0
```

## Customization Guide

### Tuning for Your Platform

1. **Start with a preset**:
   ```rust
   let mut config = VIOPipelineConfig::cpu_only();
   ```

2. **Adjust feature counts**:
   ```rust
   if let Some(ref mut tf) = config.detection.track_first {
       tf.min_features = 120;
       tf.max_features = 250;
   }
   ```

3. **Tune fusion**:
   ```rust
   config.fusion.num_frames = 4;  // More = smoother, slower
   if let Some(ref mut stab) = config.fusion.stabilizer {
       stab.accumulation_weight = 0.8;  // Higher = more reactive
   }
   ```

4. **Validate**:
   ```rust
   config.validate()?;
   ```

5. **Save custom configuration**:
   ```rust
   config.save_toml("my_custom_config.toml")?;
   ```

### Performance Tuning Tips

**If FPS is too low**:
- Reduce `max_features` (150 → 100)
- Decrease `pyramid_levels` (3 → 2)
- Reduce `fusion.num_frames` (5 → 3)
- Use simpler detector (`ShiTomasi` → `Fast`)
- Disable loop closure
- Reduce stereo `max_disparity` (128 → 64)

**If quality is poor**:
- Increase `max_features` (200 → 300)
- Enable fusion (`strategy = "RotationOnly"`)
- Increase `pyramid_levels` (2 → 3)
- Improve feature quality (`min_quality = 0.005`)
- Enable subpixel refinement

**If memory usage is high**:
- Reduce `max_features`
- Decrease `fusion.num_frames`
- Reduce `pyramid_levels`
- Lower `max_memory_mb` budget

## Integration Example

```rust
use rs_vio::feature_tracker::{
    VIOPipelineConfig, FrameStabilizer, TrackFirstDetector
};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Load configuration
    let config = VIOPipelineConfig::load_toml("configs/balanced.toml")?;
    config.validate()?;
    
    println!("{}", config.summary());
    
    // Create stabilizer (if enabled)
    let mut stabilizer = if config.fusion.strategy == FusionStrategy::RotationOnly {
        config.fusion.stabilizer.map(|stab_config| {
            FrameStabilizer::new(stab_config, fx, fy, cx, cy)
        })
    } else {
        None
    };
    
    // Create track-first detector (if enabled)
    let mut detector = if config.detection.tracking_strategy == TrackingStrategy::TrackFirst {
        config.detection.track_first.map(|tf_config| {
            TrackFirstDetector::new(tf_config, image_width, image_height)
        })
    } else {
        None
    };
    
    // Use in tracking loop
    for (frame, rotation, timestamp) in frames {
        // Optional stabilization
        let processed_frame = if let Some(ref mut stab) = stabilizer {
            stab.process_frame(&frame, rotation, timestamp)
        } else {
            frame
        };
        
        // Track-first detection
        if let Some(ref mut det) = detector {
            det.update_tracks(&tracked_points, &processed_frame, residuals);
        }
        
        // ... rest of tracking pipeline
    }
    
    Ok(())
}
```

## Configuration Validation

The system performs automatic validation:

```rust
let config = VIOPipelineConfig::cpu_only();

// This checks:
// - GPU features require enable_gpu = true
// - Fusion strategies have required parameters
// - Feature counts are sensible
// - Performance budgets are positive
config.validate()?;
```

**Common Validation Errors**:
- `SuperPoint/DISK require GPU enabled`
- `Rotation-only fusion requires stabilizer config`
- `min_features must be <= max_features`
- `target_fps must be positive`

## CLI Configuration Tool

Interactive helper for selecting configurations:

```bash
# List all available platforms
python3 scripts/configure_vio.py --list

# Interactive selection
python3 scripts/configure_vio.py --interactive

# Direct selection with recommendations
python3 scripts/configure_vio.py --platform gpu_enabled --recommend
```

## Best Practices

1. **Start with a preset** that matches your hardware
2. **Profile your application** to find bottlenecks
3. **Tune incrementally** - change one parameter at a time
4. **Validate after changes** to catch misconfigurations
5. **Save custom configs** for reproducibility
6. **Document your tuning** for team members

## Module Toggle Pattern

All major modules can be enabled/disabled:

```rust
// Disable fusion for minimal latency
config.fusion.strategy = FusionStrategy::None;

// Disable loop closure for realtime
config.enable_loop_closure = false;

// Disable bundle adjustment for speed
config.enable_bundle_adjustment = false;

// Disable IMU integration (visual-only)
config.enable_imu = false;
```

## Advanced: Custom Platform

For specialized hardware:

```rust
let config = VIOPipelineConfig {
    platform: TargetPlatform::Custom,
    performance: PerformanceBudget {
        target_fps: 25.0,
        max_latency_ms: 40.0,
        max_cpu_usage: 0.85,
        max_memory_mb: 384,
        enable_gpu: false,
    },
    // ... customize all components
    ..Default::default()
};
```

## Summary

The configuration system provides:

- ✅ **4 platform presets** (CPU-only, GPU, hard-realtime, balanced)
- ✅ **Modular design** - enable/disable any component
- ✅ **Performance budgets** - FPS, latency, memory, CPU
- ✅ **Automatic validation** - catch errors before runtime
- ✅ **TOML serialization** - easy to edit and version control
- ✅ **Interactive CLI** - guided configuration selection
- ✅ **Runtime tuning** - adjust parameters on the fly
- ✅ **Platform-aware** - optimized for specific hardware

This makes RS-VIO adaptable to any platform from resource-constrained embedded systems to high-performance GPU platforms.
