# RS-VIO Benchmarking Guide

**Last Updated**: 2026-04-12

---

## Quick Start

### Run All Benchmarks

```bash
cd /Users/vincent/Work/RS-VIO

# Run complete benchmark suite (VO vs VIO on all datasets)
make benchmark

# Or run specific dataset
make benchmark-corridor1
make benchmark-room1
```

### View Results

```bash
# View latest results
make results

# Open results directory
make results-dir

# View specific benchmark
cat benchmark_results/vo_vio_benchmark_*.json
```

---

## Benchmark Types

### 1. VO vs VIO Accuracy Benchmark

**Purpose**: Compare visual-only (VO) vs visual-inertial (VIO) accuracy.

**What it measures**:
- ATE (Absolute Trajectory Error) for both modes
- RPE (Relative Pose Error) at multiple intervals
- Sim(3) scale error
- Runtime performance
- Improvement percentage (VIO vs VO)

**Command**:
```bash
make benchmark-vo-vio
```

**Output**: JSON file with complete metrics

**Example Output**:
```json
{
  "dataset": "dataset-corridor1_512_16",
  "vo": {
    "runtime_s": 180.5,
    "accuracy": {
      "ate": {"rmse": 0.61, "sim3_scale": 0.98},
      "rpe": {"delta_1.0s": {"trans_rmse": 0.049, "rot_rmse": 1.29}}
    }
  },
  "vio": {
    "runtime_s": 185.2,
    "accuracy": {
      "ate": {"rmse": 0.15, "sim3_scale": 1.02},
      "rpe": {"delta_1.0s": {"trans_rmse": 0.036, "rot_rmse": 1.04}}
    }
  },
  "improvement": {
    "improvement_pct": 75.4,
    "improvement_factor": 4.1
  }
}
```

### 2. Motion Prior Overhead Benchmark

**Purpose**: Measure computational overhead of motion priors.

**What it measures**:
- `process_noise_scale()` time
- `gate_measurement()` time
- `score_prediction()` time
- `adaptive_huber` computation time
- Total overhead percentage

**Command**:
```bash
make measure-overhead
```

**Expected Results**:
- Total overhead: **<0.02%** (target was <4%)
- Individual operations: **<20 ns** per call

### 3. Sliding Window Benchmark

**Purpose**: Measure core VIO component performance.

**What it measures**:
- IMU preintegration time
- IMU factor residual + Jacobian time
- inverse_se3 time
- predict_current_pose time

**Command**:
```bash
make measure-sliding
```

---

## Datasets

### TUM-VI Dataset

**Location**: `/Users/vincent/data/tum-vi/`

| Dataset | Duration | GT Coverage | Characteristics |
|---------|----------|-------------|-----------------|
| `dataset-corridor1_512_16` | ~300s | 17% | Long corridor, limited GT |
| `dataset-corridor2_512_16` | ~338s | 21% | Long corridor, limited GT |
| `dataset-room1_512_16` | ~120s | 98% | Indoor room, full GT |
| `dataset-slides1_512_16` | ~280s | ~50% | Vertical motion |

### Running on Specific Datasets

```bash
# Single dataset
make benchmark-corridor1

# Multiple datasets
make benchmark DATASETS="/path/to/ds1 /path/to/ds2"

# Custom timeout (default: 600s)
make benchmark-vo-vio timeout=900
```

---

## Metrics Explained

### ATE (Absolute Trajectory Error)

**Definition**: RMSE after SE(3) alignment with ground truth.

**Interpretation**:
- `<0.1m`: Excellent (room sequences)
- `0.1-0.5m`: Good (corridor sequences)
- `>1.0m`: Poor (needs investigation)

### RPE (Relative Pose Error)

**Definition**: RMSE of relative pose at specific time intervals.

**Intervals**:
- `0.1s`: High-frequency tracking accuracy
- `1.0s`: Short-term drift
- `5.0s`: Medium-term drift
- `10.0s`: Long-term drift

**Interpretation** (for 1.0s interval):
- `<0.05 m/s`: Excellent
- `0.05-0.1 m/s`: Good
- `>0.2 m/s`: Poor

### Sim(3) Scale

**Definition**: Ratio of estimated path length to GT path length.

**Interpretation**:
- `1.0`: Perfect scale
- `0.9-1.1`: Acceptable
- `<0.5` or `>2.0`: Scale observability issue

### Improvement Percentage

**Formula**: `(VO_ATE - VIO_ATE) / VO_ATE * 100`

**Interpretation**:
- `>50%`: Significant IMU benefit
- `20-50%`: Moderate IMU benefit
- `<20%`: Minimal IMU benefit (or VO already excellent)

---

## Expected Results

### Corridor1 (Challenging, Limited GT)

| Metric | VO | VIO | Expected Improvement |
|--------|-----|-----|---------------------|
| ATE RMSE | 0.61m | 0.1-0.2m | **3-6×** |
| Sim(3) scale | 0.98 | 1.02 | Better stability |
| RPE (1s) trans | 0.049 m/s | 0.036 m/s | **27%** |

### Room1 (Easy, Full GT)

| Metric | VO | VIO | Expected Improvement |
|--------|-----|-----|---------------------|
| ATE RMSE | 0.15m | 0.10-0.12m | **1.2-1.5×** |
| Sim(3) scale | 1.00 | 1.01 | Similar |
| RPE (1s) trans | 0.04 m/s | 0.03 m/s | **25%** |

### Motion Prior Overhead

| Component | Time | Overhead |
|-----------|------|----------|
| process_noise_scale | <1 ns | Negligible |
| gate_measurement | <15 ns | Negligible |
| score_prediction | <15 ns | Negligible |
| **Total at 30 FPS** | **<0.02%** | **Excellent** |

---

## Troubleshooting

### Binary Not Found

```bash
# Build release binary
cargo build --release

# Verify
ls -la target/release/run_tum
```

### Dataset Not Found

```bash
# Check dataset location
ls -la /Users/vincent/data/tum-vi/

# Download if needed
# https://cvg.cit.tum.de/data/datasets/visual-inertial-dataset
```

### Ground Truth Not Found

```bash
# Check GT path
ls -la /path/to/dataset/mav0/mocap0/data.csv

# Some datasets may have different GT paths
# Check dataset documentation
```

### Timeout

```bash
# Increase timeout
make benchmark-vo-vio timeout=900

# Or run single dataset
make benchmark-corridor1
```

### JSON Parse Error

```bash
# Check Python dependencies
pip3 install numpy scipy

# Verify script runs
python3 scripts/benchmark_vo_vio.py --help
```

---

## Automated Testing

### CI Integration

```yaml
# .github/workflows/benchmark.yml
name: Benchmarks

on: [push, pull_request]

jobs:
  benchmark:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v2
      
      - name: Build
        run: cargo build --release
      
      - name: Run Benchmarks
        run: |
          cargo bench --bench motion_prior
          cargo bench --bench sliding_window
```

### Performance Regression Detection

```bash
# Save baseline
make benchmark-vo-vio
cp benchmark_results/*.json baseline_benchmark.json

# After changes, compare
python3 scripts/compare_benchmarks.py \
    baseline_benchmark.json \
    benchmark_results/*.json
```

---

## Results Tracking

### Historical Results

Results are saved to:
- `benchmark_results/vo_vio_benchmark_*.json` - VO vs VIO comparisons
- `performance_results/motion_prior_*.log` - Overhead measurements
- `performance_results/sliding_window_*.log` - Component benchmarks

### Generating Reports

```bash
# Generate summary report
python3 scripts/generate_report.py \
    benchmark_results/*.json \
    --output benchmark_summary.md
```

---

## Additional Resources

- `BENCHMARK.md` - Historical benchmark results
- `PERFORMANCE_MEASUREMENT_STATUS.md` - What's measured vs estimated
- `PERFORMANCE_SUMMARY.md` - Executive summary
- `DEVELOPER.md` - Complete developer guide

---

**For questions or issues, see the main README.md or open an issue.**
