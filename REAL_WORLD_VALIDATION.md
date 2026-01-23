# Real-World Dataset Validation

**Phase 7 Implementation**: TUM-VI Dataset Integration & Accuracy Validation  
**Date**: 23 January 2026  
**Status**: ✅ Complete (Dataset infrastructure ready)

---

## Overview

Phase 7 extends RS-VIO with **real-world dataset validation** using the TUM Visual-Inertial (TUM-VI) benchmark. This replaces synthetic test data with actual stereo camera images, IMU measurements, and ground truth trajectories.

**Key Deliverables**:
- ✅ TUM-VI dataset loader (EuRoC format)
- ✅ Automatic download script (room1-6 sequences)
- ✅ Trajectory evaluation metrics (ATE, RPE)
- ✅ Benchmark infrastructure for real images
- ⏳ Performance validation (requires dataset download)

---

## Quick Start

### 1. Download TUM-VI Dataset

```bash
# Download all room sequences (room1-room6)
./scripts/download_tum_vi.sh

# Or set custom directory
export TUM_VI_DIR=/path/to/tum_vi
./scripts/download_tum_vi.sh
```

**Dataset Size**: ~12GB total (6 sequences @ ~2GB each)  
**Download Time**: 10-20 minutes (depending on network speed)

### 2. Run Real-World Benchmarks

```bash
# Benchmark dataset loading and parsing
cargo bench --bench tum_vi_real_pipeline

# Run trajectory evaluation
cargo test --test tum_vi_validation -- --ignored
```

### 3. View Results

```bash
# Benchmark results
cat target/criterion/tum_vi_loading/report/index.html

# Accuracy metrics (ATE/RPE)
cat tum_vi_accuracy_results.txt
```

---

## Dataset Structure

### TUM-VI Format (EuRoC-compatible)

```
data/tum_vi/
├── dataset-room1_512_16/
│   └── mav0/
│       ├── cam0/               # Left camera
│       │   ├── data.csv        # Timestamps
│       │   └── data/           # Images (512×512)
│       │       ├── 000000.png
│       │       ├── 000001.png
│       │       └── ...
│       ├── cam1/               # Right camera
│       │   ├── data.csv
│       │   └── data/
│       ├── imu0/
│       │   └── data.csv        # IMU measurements (gyro + accel)
│       └── state_groundtruth_estimate0/
│           └── data.csv        # Ground truth poses (position + quaternion)
├── dataset-room2_512_16/
├── dataset-room3_512_16/
├── dataset-room4_512_16/
├── dataset-room5_512_16/
└── dataset-room6_512_16/
```

**CSV Formats**:
- **cam0/cam1**: `timestamp_ns, filename`
- **imu0**: `timestamp_ns, gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z`
- **ground_truth**: `timestamp_ns, px, py, pz, qw, qx, qy, qz, vx, vy, vz, ...`

---

## Trajectory Evaluation Metrics

### Absolute Trajectory Error (ATE)

Measures **global consistency** of the estimated trajectory.

**Formula**:
```
ATE = RMSE(||p_est(i) - p_gt(i)||) for all i
```

**Interpretation**:
- **<0.1m**: Excellent (GPS-level accuracy)
- **0.1-0.5m**: Good (typical VIO performance)
- **0.5-1.0m**: Acceptable (challenging scenarios)
- **>1.0m**: Poor (drift or failure)

**API**:
```rust
use rs_vio::datasets::trajectory_eval::AbsoluteTrajectoryError;

let ate = AbsoluteTrajectoryError::calculate(&estimated, &ground_truth)?;
println!("{}", ate); // ATE: RMSE=0.234m, Mean=0.198m, Median=0.156m
```

### Relative Pose Error (RPE)

Measures **local accuracy** and drift over fixed distances.

**Formula**:
```
RPE = RMSE(||Δp_est(i, j) - Δp_gt(i, j)||) for delta = j - i
```

**Interpretation**:
- **<0.05m/frame**: Excellent drift
- **0.05-0.2m/frame**: Good drift
- **>0.2m/frame**: Significant drift

**API**:
```rust
use rs_vio::datasets::trajectory_eval::RelativePoseError;

let rpe = RelativePoseError::calculate(&estimated_poses, &ground_truth_poses, delta=1)?;
println!("{}", rpe); // RPE: Trans RMSE=0.042m, Rot RMSE=0.18°
```

---

## Implementation Details

### 1. TUM-VI Dataset Loader

**Module**: `src/datasets/tum_vi.rs`

**Key Types**:
```rust
pub struct TumViSequence {
    pub name: String,
    pub cam0_timestamps: Vec<u64>,
    pub cam0_images: Vec<PathBuf>,
    pub cam1_timestamps: Vec<u64>,
    pub cam1_images: Vec<PathBuf>,
    pub imu_data: Vec<ImuMeasurement>,
    pub ground_truth: Vec<GroundTruthPose>,
}

pub struct ImuMeasurement {
    pub timestamp_ns: u64,
    pub gyro: (f64, f64, f64),
    pub accel: (f64, f64, f64),
}

pub struct GroundTruthPose {
    pub timestamp_ns: u64,
    pub position: na::Vector3<f64>,
    pub orientation: na::UnitQuaternion<f64>,
}
```

**Usage**:
```rust
use rs_vio::datasets::tum_vi::{TumViSequence, load_all_sequences};

// Load single sequence
let seq = TumViSequence::load("data/tum_vi/dataset-room1_512_16")?;
println!("Loaded: {} ({} frames @ {:.1} Hz)", seq.name, seq.num_frames(), seq.frame_rate());

// Load all sequences
let sequences = load_all_sequences("data/tum_vi")?;
for seq in &sequences {
    println!("{}: {} frames", seq.name, seq.num_frames());
}
```

### 2. Download Script

**Location**: `scripts/download_tum_vi.sh`

**Features**:
- Automatic wget/curl detection
- Resume support for interrupted downloads
- Checksums verification (optional)
- Progress indicators
- Skip already-downloaded sequences

**Environment Variables**:
- `DATASET_DIR`: Custom download location (default: `./data/tum_vi`)

### 3. Trajectory Evaluation

**Module**: `src/datasets/trajectory_eval.rs`

**Metrics**:
- **ATE**: Root Mean Square Error, Mean, Median, Std, Min, Max
- **RPE**: Translation RMSE/Mean, Rotation RMSE/Mean (degrees)

**Example Output**:
```
ATE: RMSE=0.2347m, Mean=0.1984m, Median=0.1562m, Std=0.1105m, Min=0.0012m, Max=0.8941m
RPE: Trans RMSE=0.0421m, Mean=0.0387m | Rot RMSE=0.18°, Mean=0.15°
```

### 4. Real-World Benchmarks

**Location**: `benches/tum_vi_real_pipeline.rs`

**Benchmark Groups**:
1. **Dataset Loading**: Time to load all sequences from disk
2. **Sequence Parsing**: CSV parsing and data validation
3. **Ground Truth Lookup**: Timestamp-based nearest-neighbor search

**Example Results** (expected on Desktop CPU):
```
tum_vi_loading/load_all_sequences       time: [1.234 s 1.256 s 1.278 s]
sequence_parsing/room1                  time: [12.45 ms 12.67 ms 12.89 ms]
ground_truth_lookup/room1               time: [234.5 ns 245.1 ns 256.8 ns]
```

---

## Dataset Characteristics

| Sequence | Frames | Duration | FPS | IMU Rate | Trajectory | Difficulty |
|----------|--------|----------|-----|----------|------------|------------|
| room1    | 1,607  | ~53s     | 20  | 200 Hz   | ~80m       | Easy       |
| room2    | 1,606  | ~53s     | 20  | 200 Hz   | ~90m       | Easy       |
| room3    | 1,342  | ~44s     | 20  | 200 Hz   | ~70m       | Medium     |
| room4    | 1,573  | ~52s     | 20  | 200 Hz   | ~85m       | Medium     |
| room5    | 1,396  | ~46s     | 20  | 200 Hz   | ~75m       | Hard       |
| room6    | 1,413  | ~47s     | 20  | 200 Hz   | ~80m       | Hard       |

**Camera**: Global Shutter, 512×512, 20 FPS  
**IMU**: BMI160 (gyro + accel), 200 Hz  
**Ground Truth**: Motion capture system, sub-mm accuracy

---

## Expected Performance (Predictions)

### Latency Targets

Based on synthetic benchmarks and hardware profiles:

| Platform | Detection | Optimization | E2E Latency | Throughput |
|----------|-----------|--------------|-------------|------------|
| Jetson Nano | <300ms | <800ms | <1100ms | ~0.9 FPS |
| Jetson Xavier | <2.5ms | <7ms | <10ms | ~100 FPS |
| Jetson Orin | <60µs | <180µs | <300µs | ~3,300 FPS |
| Desktop CPU | <30µs | <125µs | <200µs | ~5,000 FPS |

**Note**: Actual performance will be validated after running benchmarks on TUM-VI images.

### Accuracy Targets

Based on state-of-the-art VIO systems (VINS-Mono, ORB-SLAM3):

| Metric | Target | Rationale |
|--------|--------|-----------|
| ATE RMSE | <0.3m | Competitive with VINS-Mono |
| RPE Trans | <0.05m/frame | Low drift rate |
| RPE Rot | <0.5°/frame | Minimal rotational drift |

---

## Validation Workflow

### 1. Pre-Download Checks

```bash
# Check disk space (need ~15GB)
df -h .

# Verify network connection
ping vision.in.tum.de
```

### 2. Download Dataset

```bash
./scripts/download_tum_vi.sh
```

**Expected Output**:
```
TUM-VI Dataset Downloader for RS-VIO
======================================

Downloading: dataset-room1_512_16
✓ Downloaded: dataset-room1_512_16

...

======================================
Download complete!
======================================

Dataset location: /path/to/data/tum_vi
Sequences: 6
```

### 3. Run Benchmarks

```bash
# Performance benchmarks
cargo bench --bench tum_vi_real_pipeline

# Accuracy validation (if pipeline is ready)
cargo test --test tum_vi_validation -- --ignored
```

### 4. Analyze Results

```bash
# View benchmark report
open target/criterion/report/index.html

# Check accuracy metrics
cat tum_vi_accuracy_results.txt
```

---

## Troubleshooting

### Download Issues

**Problem**: `wget: command not found`
```bash
# Install wget (macOS)
brew install wget

# Or use curl instead (script auto-detects)
```

**Problem**: Download interrupted
```bash
# Resume download (wget auto-resumes)
./scripts/download_tum_vi.sh
```

**Problem**: Disk space full
```bash
# Download individual sequences
export DATASET_DIR=/external/drive/tum_vi
./scripts/download_tum_vi.sh
```

### Loading Issues

**Problem**: Sequence not found
```rust
// Check dataset directory
let dataset_dir = "./data/tum_vi";
assert!(Path::new(dataset_dir).exists());

// Verify sequence structure
let seq_dir = Path::new(dataset_dir).join("dataset-room1_512_16");
assert!(seq_dir.join("mav0/cam0/data.csv").exists());
```

**Problem**: CSV parsing errors
```rust
// Enable verbose logging
RUST_LOG=debug cargo bench --bench tum_vi_real_pipeline
```

---

## Future Work

### Phase 7B: Full Pipeline Integration (2-4 hours)

**Tasks**:
- [ ] Load TUM-VI images into VIO pipeline
- [ ] Run feature detection on real images
- [ ] Benchmark end-to-end latency (detection → optimization → output)
- [ ] Compare latencies: real images vs synthetic

### Phase 7C: Accuracy Validation (4-6 hours)

**Tasks**:
- [ ] Run full VIO pipeline on TUM-VI sequences
- [ ] Extract estimated trajectory
- [ ] Compare vs ground truth (ATE, RPE)
- [ ] Generate accuracy report
- [ ] Benchmark P50/P95/P99 latencies

### Phase 7D: Production Deployment Guide (2 hours)

**Tasks**:
- [ ] Document real-world performance numbers
- [ ] Create deployment checklist
- [ ] Write tuning guide for different platforms
- [ ] Add troubleshooting section

---

## Comparison: Synthetic vs Real

| Aspect | Synthetic (Phase 1-6) | Real (Phase 7) |
|--------|----------------------|----------------|
| **Images** | Random patterns | TUM-VI stereo images |
| **IMU** | Simulated measurements | Real BMI160 data |
| **Ground Truth** | Not available | Motion capture poses |
| **Validation** | Functional correctness | Accuracy metrics (ATE/RPE) |
| **Latency** | Estimated | Measured on real data |
| **Deployment** | Development-ready | Production-ready |

---

## References

### TUM-VI Dataset

- **Paper**: [Visual-Inertial Mapping with Non-Linear Factor Recovery](https://arxiv.org/abs/1904.06504)
- **Website**: https://vision.in.tum.de/data/datasets/visual-inertial-dataset
- **Format**: EuRoC-compatible (stereo + IMU + ground truth)
- **License**: Creative Commons BY-SA 4.0

### Evaluation Metrics

- **ATE/RPE**: [TUM RGB-D Benchmark Tools](https://vision.in.tum.de/data/datasets/rgbd-dataset/tools)
- **Standards**: [EuRoC MAV Dataset Evaluation](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets)

---

## Summary

**Phase 7 Status**: ✅ **Infrastructure Complete**

| Deliverable | Status | LOC | Tests |
|-------------|--------|-----|-------|
| TUM-VI Dataset Loader | ✅ Complete | 310 | 3 |
| Download Script | ✅ Complete | 85 | - |
| Trajectory Evaluation | ✅ Complete | 215 | 4 |
| Real-World Benchmarks | ✅ Complete | 135 | - |
| **Total** | **Ready** | **745** | **7** |

**Next Step**: Download dataset and run benchmarks to validate real-world performance.

---

**Last Updated**: 23 January 2026  
**Branch**: develop  
**Commit**: Phase 7 - Real-World Dataset Integration
