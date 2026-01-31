# Benchmarking Guide for RS-VIO

This document provides guidance on running, interpreting, and comparing benchmarks for RS-VIO. It covers everything from quick starts to detailed performance analysis and continuous improvement workflows.

## Table of Contents

1. [Prerequisites](#prerequisites)
2. [Quick Start](#quick-start)
3. [Running Benchmarks](#running-benchmarks)
4. [Understanding Results](#understanding-results)
5. [Visualization Workflow](#visualization-workflow)
6. [Performance Analysis](#performance-analysis)
7. [Benchmark Details](#benchmark-details)
8. [Branch Comparison](#branch-comparison)
9. [Advanced Topics](#advanced-topics)
10. [Troubleshooting](#troubleshooting)

## Prerequisites

**System Requirements:**
- Rust 1.70+ with `cargo` (see [CONTRIBUTING.md](CONTRIBUTING.md))
- macOS or Linux (Windows requires WSL2)
- ~2GB disk space for `target/criterion/` and plots
- Python 3.8+ (optional, for visualization)

**Optional Dependencies (for plotting):**
```bash
pip install -r scripts/requirements-benchmarking.txt  # matplotlib, numpy
```

**Build Profiles Tested:**
- `release` (default, ~10-15 min for full suite)
- `embedded-safe` (with debug checks, ~12-18 min)
- `ultra-critical` (maximum safety, ~12-18 min)

## Quick Start

Get baseline metrics in 5 minutes:

```bash
# 1. Run baseline (includes plotting if matplotlib available)
./scripts/benchmark.sh --baseline v0.2.0 --save

# 2. View plots (generated in target/benchmark_plots/)
open target/benchmark_plots/benchmark_comparison.png

# 3. Run comparison after code changes
./scripts/benchmark.sh --baseline v0.2.0

# Expected output time: 10-15 minutes (full suite with ~20 benchmarks)
```

For Python plotting specifically:
```bash
# Install dependencies
pip install matplotlib numpy

# Run all benchmarks
cargo bench --release --all

# Generate visualizations
python3 scripts/plot_benchmarks.py
```

## Overview

RS-VIO includes comprehensive benchmarks for all major components:

- **Feature Tracker** (`benches/feature_tracker.rs`) - Feature detection, tracking, and grid operations
- **Estimator** (`benches/estimator.rs`) - Frame processing, initialization, and resolution scaling
- **Optimization** (`benches/optimization.rs`) - State management and sliding window operations
- **Pipeline** (`benches/pipeline.rs`) - End-to-end VIO pipeline latency and throughput

## Running Benchmarks

**Timing Expectations:**
| Command | Time | Notes |
|---------|------|-------|
| Single benchmark | 2-5 min | e.g., `--bench estimator` |
| All benchmarks | 10-15 min | `--all` on release profile |
| With comparison | 15-20 min | Extra time for baseline retrieval |
| With plotting | +2-3 min | Python post-processing |

### Run All Benchmarks
```bash
cargo bench --release --all
```

### Run Specific Benchmark Suite
```bash
# Feature tracker benchmarks
cargo bench --release --bench feature_tracker

# Estimator benchmarks
cargo bench --release --bench estimator

# Optimization benchmarks
cargo bench --release --bench optimization

# Pipeline benchmarks
cargo bench --release --bench pipeline
```

### Run Specific Benchmark
```bash
# Run only feature detection benchmarks
cargo bench --release --bench feature_tracker -- feature_detection

# Run only pipeline latency benchmarks
cargo bench --release --bench pipeline -- vio_pipeline_latency
```

### Baseline Comparison Mode
```bash
# Save baseline for future comparison
cargo bench --release --bench estimator -- --save-baseline v0.2.0

# Compare against saved baseline
cargo bench --release --bench estimator -- --baseline v0.2.0

# Compare multiple baselines
cargo bench --release --bench estimator -- --baseline optimization-v1
```

## Understanding Results

### Criterion Output Format

When you run benchmarks, Criterion produces output like:

```
Feature Detection (5×5 grid)           time:   [4.2342 ms 4.2891 ms 4.3456 ms]
                                       change: [-1.23% +0.45% +2.10%]
                                       slope   [4.2129 ms 4.3102 ms] R² = 0.9987
Found 2 outliers among 100 measurements (2.00%)
  1 (1.00%) high mild
  1 (1.00%) low mild
```

**Key Metrics Explained:**

| Metric | Meaning | Example |
|--------|---------|---------|
| `time` | Measured execution time with 95% confidence interval | `[4.23 ms 4.29 ms 4.35 ms]` means 4.29ms ±0.06ms |
| `change` | Percent difference vs. saved baseline | `[-1.23% +0.45% +2.10%]` means -1.23% to +2.10% change |
| `slope` | Linear regression of time vs. iteration count | Indicates if variance is systematic |
| `outliers` | Measurements outside normal distribution | Usually <5% is normal |
| `R²` | Goodness of fit for regression | >0.99 indicates reliable results |

### Interpreting Statistical Significance

**Performance Change Classification:**

| Change | Interpretation | Action |
|--------|-----------------|--------|
| **< ±5%** | Within measurement noise | No action needed |
| **±5% to ±10%** | Marginal (investigate if consistent) | Run 3× more to confirm |
| **> ±10% improvement** | Significant improvement ✅ | Document and merge |
| **> ±10% regression** | Significant regression ⚠️ | Investigate before merge |

**Example Decision Tree:**
```
Criterion shows: change [-2.1% +1.3% +4.2%]
├─ Confidence interval includes 0? YES
├─ Max change < ±5%? YES
└─ Decision: Within noise, approve change ✅

Criterion shows: change [-0.5% +8.3% +15.1%]
├─ Confidence interval includes 0? NO
├─ All measurements > 5%? YES
└─ Decision: Investigate (outliers?) 🔍 → re-run with --sample-size 100
```

### Criterion Output Files

Benchmark results are saved in `target/criterion/`:

```
target/criterion/
├── estimator/
│   ├── process_single_frame_baseline/
│   │   ├── base/
│   │   │   ├── base.json        # Raw measurements
│   │   │   ├── raw.json         # Outlier analysis
│   │   │   └── ...
│   │   └── benchmark.json       # Summary statistics
│   └── [other benchmarks]/
├── feature_tracker/
└── ...
```

Use these for post-processing: `python3 scripts/plot_benchmarks.py`

## Automated Benchmarking with Plotting

### Quick Start
```bash
# Make script executable
chmod +x scripts/benchmark.sh

# Install plotting dependencies
pip install -r scripts/requirements-benchmarking.txt

# Save baseline for current version
./scripts/benchmark.sh --baseline v0.2.0 --save

# Run all benchmarks and generate plots
./scripts/benchmark.sh --baseline v0.2.0
```

### Benchmark Script Usage

**Run specific benchmark:**
```bash
./scripts/benchmark.sh --bench estimator
```

**Run with different profile:**
```bash
./scripts/benchmark.sh --profile ultra-critical
./scripts/benchmark.sh --profile embedded-safe
```

**Save and compare baselines:**
```bash
# Save baseline
./scripts/benchmark.sh --baseline v0.2.0 --save

# Later, compare against it
./scripts/benchmark.sh --baseline v0.2.0

# Compare optimization changes
./scripts/benchmark.sh --baseline optimization-attempt-1 --save
./scripts/benchmark.sh --baseline optimization-attempt-1
```

**Specify output directories:**
```bash
./scripts/benchmark.sh --output-dir /tmp/bench --plot-dir /tmp/plots
```

### Generated Plots

The plotting script generates four types of visualizations:

#### 1. Benchmark Comparison (`benchmark_comparison.png`)
- Bar chart of all benchmarks
- Shows mean time with error bars (std deviation)
- Easy identification of slowest operations

**Use case:** Identify performance bottlenecks across all components

#### 2. Resolution Scaling (`resolution_scaling.png`)
- Line chart showing latency vs image resolution
- Tests: 320×240, 640×480, 1280×960
- Helps identify quadratic pixel scaling

**Use case:** Determine feasibility for different resolution targets

#### 3. Grid Size Scaling (`grid_scaling.png`)
- Performance impact of grid granularity
- Tests: 5×5, 10×10, 15×15, 20×20
- Optimize grid parameters

**Use case:** Balance feature density vs. speed

#### 4. Category Breakdown (`category_breakdown.png`)
- Average latency by benchmark category
- Feature detection, tracking, estimation, optimization
- Compare subsystem performance

**Use case:** Allocate optimization efforts by impact

### Plotting Script Options

```bash
python3 scripts/plot_benchmarks.py [OPTIONS]

Options:
  --criterion-dir DIR      Path to Criterion output (default: target/criterion)
  --output-dir DIR         Output directory for plots (default: target/benchmark_plots)
  --plot-types TYPES       Plots to generate (default: all)
                           Options: comparison, resolution, grid, category, all
```

**Examples:**
```bash
# Generate all plots
python3 scripts/plot_benchmarks.py

# Generate only resolution and grid plots
python3 scripts/plot_benchmarks.py --plot-types resolution grid

# Use custom directories
python3 scripts/plot_benchmarks.py \
  --criterion-dir /tmp/results \
  --output-dir ./my_plots
```

## Visualization Workflow

### Step-by-Step: Optimization Cycle with Plots

1. **Establish baseline:**
   ```bash
   ./scripts/benchmark.sh --baseline v0.2.0-initial --save
   ```
   - Generates plots in `target/benchmark_plots/`
   - Saves Criterion baseline for comparison

2. **Examine plots:**
   ```bash
   open target/benchmark_plots/benchmark_comparison.png    # Find bottlenecks
   open target/benchmark_plots/resolution_scaling.png      # Check scaling
   open target/benchmark_plots/grid_scaling.png            # Optimize parameters
   ```

3. **Make optimization changes** to hot paths identified in plots

4. **Run comparison benchmarks:**
   ```bash
   ./scripts/benchmark.sh --baseline v0.2.0-initial
   ```
   - Compares metrics against baseline
   - Regenerates plots with new data

5. **Review changes:**
   - Compare old vs. new plots visually
   - Check Criterion output for statistical significance
   - Look for >5% improvements before merging

6. **Save successful optimization:**
   ```bash
   ./scripts/benchmark.sh --baseline v0.2.0-optimized --save
   ```

### Example: Optimizing Feature Tracking

```bash
# Initial baseline
./scripts/benchmark.sh --bench feature_tracker --baseline tracking-v1 --save

# Examine results
open target/benchmark_plots/grid_scaling.png

# Analysis:
# - 20×20 grid is significantly slower
# - Feature tracking plateaus at 10×10
# Decision: Investigate 15×15 optimization

# (Make optimization changes...)

# Test improvement
./scripts/benchmark.sh --bench feature_tracker --baseline tracking-v1

# Output: 15×15 is now 10% faster ✓
# Save as new baseline
./scripts/benchmark.sh --bench feature_tracker --baseline tracking-v2 --save
```

### Reading Plot Outputs

**Benchmark Comparison Chart:**
```
┌─────────────────────────────────────────┐
│ Benchmark Performance Comparison        │
│                                         │
│      ▁▂▃▄▅▆▇█ (fast to slow)           │
│ process_frame_baseline        ███ 25ms │
│ sequential_frames_5           ████ 28ms│
│ feature_tracking              ██ 15ms  │
│ grid_allocation               ▁ 0.8ms  │
│                                         │
└─────────────────────────────────────────┘
```

**Resolution Scaling (linear indicates optimization needed):**
```
Latency
  ▲
  │     1280×960
  │        ●
  │       /
  │      /
  │     ● 640×480
  │    /
  │   /
  │  ● 320×240
  │ /
  └─────────────────► Resolution (pixels)
```

**Grid Size Scaling (should plateau):**
```
Latency
  ▲
  │ ●──────┐
  │  \     │
  │   ●    │ Optimal region
  │    \   │
  │     ●  │
  │      ──┘
  │
  └─────────────────► Grid Size
```

## Performance Analysis Tips

### Interpreting Plot Patterns

**Flat/plateau in scaling curve**
- Indicates algorithm is memory-bound
- Further optimizations limited by memory bandwidth
- Consider data structure or memory layout improvements

**Linear scaling in resolution**
- Expected for pixel-processing algorithms
- Doubling resolution should increase latency 4×
- Indicates good algorithmic scaling

**Exponential growth**
- Likely indicates O(n²) or worse algorithm
- High priority for optimization
- Consider algorithmic improvements (e.g., spatial indexing)

**High variance in plots**
- System is under load or competing processes active
- Rerun benchmarks with fewer background applications
- Consider running on isolated hardware for production release

### Using Plots for Decision Making

1. **Identify bottlenecks** - Look at `benchmark_comparison.png` for slowest operations
2. **Check scaling** - Review `resolution_scaling.png` for quadratic vs. linear growth
3. **Optimize parameters** - Use `grid_scaling.png` to find sweet spot
4. **Track improvements** - Save baselines before/after optimizations

### Baseline Drift Monitoring

Track baseline drift over time to catch performance regressions:

```bash
# Weekly baseline tracking
for week in $(seq 1 10); do
  ./scripts/benchmark.sh --baseline "week-$week" --save
  # Compare to previous week:
  # ./scripts/benchmark.sh --baseline "week-$((week-1))"
done
```

## CI/CD Integration

### GitHub Actions Workflow

To run benchmarks automatically on every push:

```yaml
# .github/workflows/benchmark.yml
name: Benchmarks

on:
  push:
    branches: [main, feature/**]
  pull_request:
    branches: [main]

jobs:
  benchmark:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
        with:
          fetch-depth: 0  # Full history for baseline comparison

      - uses: actions-rs/toolchain@v1
        with:
          toolchain: stable

      # Download previous main branch baseline
      - run: |
          git fetch origin main:main
          git show main:target/criterion/main-baseline.json > /tmp/baseline.json || true

      # Run benchmarks
      - run: cargo bench --release --all 2>&1 | tee /tmp/bench.log

      # Check for regressions
      - name: Analyze Results
        run: |
          if grep -q "regression" /tmp/bench.log; then
            echo "⚠️ Performance regression detected!"
            exit 1
          fi
```

### Local Baseline Management

```bash
# Save baseline for current commit
./scripts/benchmark.sh --baseline "$(git rev-parse --short HEAD)" --save

# Compare all PR branches against main
for branch in $(git branch | grep feature/); do
  git checkout "$branch"
  ./scripts/benchmark.sh --baseline "main"
  git checkout -
done
```

## Benchmark Details

### Feature Tracker Benchmarks

**Feature Detection** - Measures time to detect features in a single image
- Tests grid sizes: 5×5, 10×10, 15×15, 20×20
- Impact of grid granularity on detection performance

**Feature Tracking** - Measures optical flow tracking between frames
- Tests different grid resolutions
- Simulates 10-pixel translation (typical frame-to-frame motion)

**Noisy Tracking** - Measures robustness with Gaussian noise
- Realistic sensor conditions
- Identifies noise sensitivity

**Grid Allocation** - Measures overhead of grid initialization
- Different grid sizes
- Memory allocation patterns

**Expected Results** (baseline: 640×480 resolution):
- Feature detection: 5-20ms depending on grid size
- Feature tracking: 10-30ms for typical translation
- Grid allocation: <1ms

### Estimator Benchmarks

**Single Frame Processing** - Baseline latency per frame
- Checkerboard pattern input
- Standard 640×480 resolution
- Should be <33ms for 30 FPS target

**Sequential Frame Processing** - Multi-frame pipeline behavior
- 5, 10, 20 frames
- Accumulation of state over time
- Identifies performance degradation

**Camera Model Creation** - Initialization overhead
- One-time cost
- Not in critical path for real-time operation

**Estimator Initialization** - Setup cost
- State allocation
- Configuration parsing

**Resolution Scaling** - Impact of image resolution
- 320×240 (low-resolution)
- 640×480 (standard)
- 1280×960 (high-resolution)
- Identifies quadratic relationship with pixel count

**Expected Results** (baseline: 640×480):
- Single frame: 15-25ms
- Camera model creation: 1-2ms
- Initialization: 2-5ms
- 1280×960 frames: 3-4× slower than 640×480

### Optimization Benchmarks

**State Management** - Core data structure operations
- Adding frame poses
- Adding map points
- Low-level performance metrics

**Sliding Window Operations** - Keyframe management
- Window sizes: 3, 5, 10 frames
- State retrieval
- Pose access patterns

**State Serialization** - Copying and cloning
- Different numbers of map points
- Memory overhead per point

**Observation Management** - Feature observation recording
- Typical per-feature cost

**Pose Recovery** - Fast pose lookup
- Access patterns for pose history

**Expected Results** (baseline: 5-frame window, 50 map points):
- Add frame pose: <0.1ms
- Add map point: <0.1ms
- Get poses: <0.1ms per call
- Record observation: <0.01ms

### Pipeline Benchmarks

**VIO Pipeline Latency** - End-to-end frame processing time
- 320×240 resolution: 8-12ms
- 640×480 resolution: 15-25ms
- Includes feature tracking + optimization

**VIO Stream Throughput** - Multi-frame sustained performance
- 10, 30, 60 frame streams
- Identifies state growth effects
- Should be consistent with latency × frame count

**Initialization Phase** - First 5 frames (critical for convergence)
- Longer than steady-state frames
- Establishes scale and orientation

**Memory Overhead** - State growth over time
- 5, 10, 20, 50 frames
- Identifies memory leaks or unbounded growth

**Realtime Deadline Compliance** - 30 FPS and 60 FPS targets
- 30 FPS target: <33ms per frame
- 60 FPS target: <16ms per frame
- Critical for embedded deployment

**Expected Results** (baseline: 640×480, first 5 frames):
- Single frame latency: 15-25ms
- 30 FPS deadline: ✅ Met
- 60 FPS deadline: ❌ Not met (would require 16ms)
- Memory growth: <50MB for 50 frames

## Branch Comparison

Testing optimization changes across branches with statistical rigor.

### Workflow: Testing an Optimization

1. **Save baseline on current branch:**
   ```bash
   cargo bench --release --bench estimator -- --save-baseline current-implementation
   ```

2. **Switch to optimization branch:**
   ```bash
   git checkout feature/optimization-branch
   cargo build --release
   ```

3. **Run benchmark with comparison:**
   ```bash
   cargo bench --release --bench estimator -- --baseline current-implementation
   ```

4. **Review output:**
   - Green = faster (improvement)
   - Red = slower (regression)
   - Yellow = within noise

### Interpreting Comparison Output

```
process_single_frame_baseline        time:   [18.523 ms 18.821 ms 19.142 ms]
                                     change: [-2.34% +1.23% +5.12%] (within noise)
                        vs baseline: [18.234 ms 18.412 ms 18.601 ms]
```

- **time**: Current measured time with confidence interval
- **change**: Relative difference from baseline
- **vs baseline**: Previous baseline measurement

### Guidelines for Meaningful Changes

- **Noise margin**: ±5% is typically within measurement noise
- **Significant improvement**: >5% consistent improvement across runs
- **Significant regression**: >5% consistent regression
- **Sample size**: Run 10+ times for stable results

## Performance Targets

### Embedded VIO Requirements

| Target | Requirement | Status |
|--------|-------------|--------|
| Single frame latency | <33ms @ 30 FPS | ✅ |
| Single frame latency | <16ms @ 60 FPS | ⚠️ (needs optimization) |
| Memory per 50 frames | <50MB | ✅ |
| Feature detection | <20ms @ 640×480 | ✅ |
| Deterministic performance | <10% variance | ✅ |

## Advanced Topics

### Auto-Generated Documentation

Benchmark code is automatically documented using Rust's `rustdoc` system. The documentation includes:

**What's Documented:**
- Purpose and metrics for each benchmark
- Test parameters and ranges
- Expected results and performance targets
- Links to this detailed BENCHMARKING.md guide

**Generate Documentation:**

```bash
# Generate API documentation (includes benchmarks)
cargo doc --lib --no-deps

# With private items and internals
cargo doc --lib --no-deps --document-private-items

# Open in browser (macOS/Linux)
cargo doc --lib --no-deps --open

# Using the helper script
./scripts/generate-docs.sh
./scripts/generate-docs.sh --private --open  # Include private items
```

**View Generated Docs:**

```
target/doc/rs_vio/
├── index.html              # Main library documentation
├── feature_tracker/        # Feature tracking module
├── estimator/              # Frame estimator module
└── optimization/           # State optimization module
```

**Documentation Structure:**

Each benchmark file has a module-level doc comment (`//!`) that documents:

```rust
//! # Feature Tracker Benchmarks
//!
//! Comprehensive performance benchmarks for the feature tracking subsystem.
//!
//! ## Benchmarks
//! - Feature Detection - Time to detect features in a single image
//! - Feature Tracking - Optical flow tracking between frames
//! ...
```

**CI/CD Integration:**

Documentation is automatically generated and validated on every push:

```yaml
# .github/workflows/docs.yml runs:
1. Generate documentation with 'cargo doc'
2. Check for doc comment warnings
3. Validate benchmark doc comments exist
4. Test documentation examples
5. Deploy to GitHub Pages (on main branch)
```

**Best Practices for Documenting Benchmarks:**

When adding new benchmarks, include:

```rust
/// Measure foo operation performance
///
/// Tests the performance of foo with different parameters.
/// This is used to track regressions in the hot path.
///
/// # Metrics
/// - Time to complete N iterations
/// - Memory usage per operation
///
/// # Expected Results
/// Expected time < 1ms per operation
///
/// See [BENCHMARKING.md](../BENCHMARKING.md#foo-benchmarks) for context
fn benchmark_foo(c: &mut Criterion) {
    // benchmark code...
}
```

### Profiling with Flamegraph

For detailed performance analysis:

```bash
# Install flamegraph (if needed)
cargo install flamegraph

# Run single benchmark with profiling
CARGO_PROFILE_RELEASE_DEBUG=true cargo flamegraph --bench estimator

# Generates flame graph in flamegraph.svg
open flamegraph.svg
```

## Memory Profiling

```bash
# With valgrind (Linux)
cargo build --release --bench estimator
valgrind --tool=massif ./target/release/deps/estimator-* 2>&1

# With instruments (macOS)
cargo build --release --bench estimator
xcrun xctrace record --template "System Trace" ./target/release/deps/estimator-*
```

## Safe Builds Only

All benchmarks compile with `unsafe_code = forbid`, ensuring:
- No unsafe code paths
- Safe-only optimizations tested
- Production-ready performance

## Continuous Improvement Workflow

1. **Establish baseline**: `cargo bench --release --all -- --save-baseline v0.2.0`
2. **Make optimization changes**
3. **Run comparison**: `cargo bench --release --all -- --baseline v0.2.0`
4. **If improvement >5%**: Include in release, update baseline
5. **If regression**: Investigate and revert
6. **Document changes** in CHANGELOG.md with performance impact

## Troubleshooting

**See also:** [scripts/README.md](scripts/README.md) for script-specific issues

### Benchmarks show high variance
- Close other applications
- Disable CPU frequency scaling: `sudo cpupower frequency-set -g performance`
- Run multiple times with `--sample-size 50`

### Memory benchmark shows growth
- Check for unbounded collections in state management
- Verify sliding window is correctly removing old frames
- Profile with valgrind to identify allocation patterns

### Latency spikes
- Check for GC-like behavior in state transitions
- Look for O(n²) algorithms in optimization
- Profile with flamegraph to find hot paths

### Plots not generating
```bash
# Verify matplotlib is installed
python3 -c "import matplotlib; print('OK')"

# If missing:
pip install -r scripts/requirements-benchmarking.txt

# Manually run plotting script
python3 scripts/plot_benchmarks.py --criterion-dir target/criterion
```

### Baseline file corruption
```bash
# Remove corrupted baselines
rm -rf target/criterion/*/base/

# Regenerate
./scripts/benchmark.sh --baseline v0.2.0 --save
```

### Permission errors on scripts
```bash
# Make executable
chmod +x scripts/benchmark.sh scripts/plot_benchmarks.py

# Or run directly
python3 scripts/plot_benchmarks.py
bash scripts/benchmark.sh
```

### Criterion database locked
```bash
# Remove lock file if process crashed
rm -f target/criterion/.lock

# Rerun benchmarks
cargo bench --release --all
```

## Related Documentation

- [CONTRIBUTING.md](CONTRIBUTING.md) - Build profile setup
- [README.md](README.md#safety--embedded-systems) - Safety guarantees
- [scripts/README.md](scripts/README.md) - Script documentation
- [SECURITY.md](SECURITY.md) - Safety-critical deployment

## Next Steps

After establishing baseline metrics:
1. Profile hotspots with flamegraph
2. Identify architecture improvements
3. Test optimization changes against baseline
4. Document performance gains for release notes
5. Update performance targets as improvements stabilize
