# RS-VIO Scripts

Utility scripts for benchmarking, testing, and deployment.

## Table of Contents

1. [Benchmarking](#benchmarking)
2. [Dataset Management](#dataset-management)
3. [Development Tools](#development-tools)
4. [Contributing](#contributing)

## Benchmarking

Scripts for performance analysis and visualization.

### Quick Start

```bash
# Install dependencies once
pip install -r requirements-benchmarking.txt

# Make scripts executable
chmod +x benchmark.sh

# Run baseline + plots (10-15 minutes)
./benchmark.sh --baseline v0.2.0 --save
```

### generate-docs.sh - Documentation Generator

**Purpose:** Automatically generate HTML API documentation from source code comments.

**Features:**
- Generates rustdoc from Rust doc comments (`///`, `//!`)
- Includes benchmark documentation with parameters and results
- Validates documentation completeness
- Can serve documentation locally
- CI/CD integration ready

**Basic Usage:**

```bash
# Generate all documentation
./generate-docs.sh

# Generate and open in browser
./generate-docs.sh --open

# Include private items
./generate-docs.sh --private --open

# Serve locally on http://localhost:8000
./generate-docs.sh --server
```

**Generated Output:**

```
target/doc/
├── rs_vio/                     # Main library documentation
│   ├── feature_tracker/        # Feature tracking module
│   ├── estimator/              # Frame estimator module
│   ├── optimization/           # State optimization module
│   └── datasets/               # Dataset players
└── index.html                  # Redirects to rs_vio/
```

**Options:**

- `--help` - Show help message
- `--all` - Generate all docs (default)
- `--rustdoc` - Generate only API docs
- `--benchmarks` - Generate only benchmark docs
- `--private` - Include private items in documentation
- `--open` - Open documentation in browser
- `--clean` - Remove generated documentation
- `--server` - Serve documentation locally

**Examples:**

```bash
# Clean and regenerate
./generate-docs.sh --clean && ./generate-docs.sh

# Generate with private implementation details
./generate-docs.sh --private

# Serve for documentation review
./generate-docs.sh --server

# Generate benchmarks only
./generate-docs.sh --benchmarks --open
```

**Documentation Workflow:**

1. **Write code with doc comments:**
   ```rust
   /// Measure feature detection performance
   ///
   /// Tests feature detection at various grid sizes.
   fn benchmark_detection(c: &mut Criterion) { }
   ```

2. **Generate documentation:**
   ```bash
   ./scripts/generate-docs.sh
   ```

3. **Review in browser:**
   ```bash
   ./scripts/generate-docs.sh --open
   ```

4. **Deploy via CI/CD:**
   - Automatically runs on every push
   - Validates documentation completeness
   - Publishes to GitHub Pages

**See:** [BENCHMARKING.md](../BENCHMARKING.md#auto-generated-documentation)

## benchmark.sh - Automated Benchmark Runner

**Purpose:** Unified interface for running benchmarks with baseline comparison and plot generation.

**Features:**
- ✅ Run individual or all benchmarks
- ✅ Save and compare baselines
- ✅ Automatic plot generation
- ✅ Multiple build profiles
- ✅ Color-coded output

**Basic Usage:**

```bash
# Save baseline
./benchmark.sh --baseline v0.2.0 --save

# Run comparison against baseline
./benchmark.sh --baseline v0.2.0

# Run specific benchmark
./benchmark.sh --bench estimator

# Use different profile (release [default], embedded-safe, ultra-critical)
./benchmark.sh --profile ultra-critical

# Combine options
./benchmark.sh --bench feature_tracker --baseline tracking-v1 --save
```

**Advanced Usage:**

```bash
# Custom output directories
./benchmark.sh --output-dir /tmp/results --plot-dir /tmp/plots

# Run multiple benchmarks with comparison
for bench in estimator optimization feature_tracker pipeline; do
  ./benchmark.sh --bench "$bench" --baseline v0.2.0
done

# Generate plots without running benchmarks
# (if you already have Criterion results in target/criterion/)
python3 plot_benchmarks.py
```

**Output Structure:**
```
target/
├── criterion/              # Criterion benchmark data (JSON)
├── benchmark_results/      # Logged output from benchmark runs
└── benchmark_plots/        # Generated PNG visualizations
    ├── benchmark_comparison.png
    ├── resolution_scaling.png
    ├── grid_scaling.png
    └── category_breakdown.png
```

**Exit Codes:**
- `0` - Success
- `1` - Benchmarks failed to compile/run
- `2` - Python/matplotlib not available (plotting skipped)

### plot_benchmarks.py - Benchmark Visualization

**Purpose:** Convert Criterion JSON output into performance visualizations.

**Features:**
- Reads Criterion benchmark results from `target/criterion/`
- Generates 4 plot types with matplotlib
- Configurable output directory
- Standalone script (no special dependencies beyond matplotlib/numpy)

**Basic Usage:**

```bash
# Generate all plots
python3 plot_benchmarks.py

# Generate specific plot types
python3 plot_benchmarks.py --plot-types resolution grid

# Custom directories
python3 plot_benchmarks.py \
  --criterion-dir /tmp/criterion \
  --output-dir ./my_plots

# Get help
python3 plot_benchmarks.py --help
```

**Plot Types:**

| Plot | File | Shows |
|------|------|-------|
| **Comparison** | `benchmark_comparison.png` | All benchmarks side-by-side |
| **Resolution** | `resolution_scaling.png` | 320×240 → 1280×960 latency trend |
| **Grid** | `grid_scaling.png` | 5×5 → 20×20 grid performance |
| **Category** | `category_breakdown.png` | Average latency by component |

**Examples:**

```bash
# After running benchmarks, visualize results
cargo bench --release --all
python3 plot_benchmarks.py

# Compare plots from different baselines
python3 plot_benchmarks.py --criterion-dir target/criterion
# Then compare visually with previous plots

# Export plots to specific location for reports
python3 plot_benchmarks.py --output-dir ./performance_report
tar czf report.tar.gz ./performance_report/
```

### requirements-benchmarking.txt

Python dependencies for plotting scripts:
```
matplotlib>=3.5.0
numpy>=1.20.0
```

**Installation:**
```bash
pip install -r requirements-benchmarking.txt
```

**Troubleshooting:**
```bash
# Verify installation
python3 -c "import matplotlib; import numpy; print('OK')"

# If missing, install system-wide
pip3 install --user -r requirements-benchmarking.txt

# Or use virtual environment (recommended)
python3 -m venv venv
source venv/bin/activate
pip install -r requirements-benchmarking.txt
```

## Dataset Management

Scripts for downloading and running benchmark datasets.

### run_euroc.sh, run_tum-vi.sh, run_4seasons.sh

**Purpose:** Run VIO pipeline on standard benchmark datasets.

**Usage:**
```bash
./run_euroc.sh              # MH_01_easy dataset
./run_tum-vi.sh             # dataset-calib-imu_on sequence
./run_4seasons.sh           # parking_garage sequence
```

**See:** [CONTRIBUTING.md](../CONTRIBUTING.md#testing-with-real-datasets)

## Development Tools

### bump-version.sh

Semantic version bumping for releases.

```bash
./bump-version.sh minor    # 0.1.0 → 0.2.0
./bump-version.sh patch    # 0.1.0 → 0.1.1
./bump-version.sh major    # 0.1.0 → 1.0.0
```

## Contributing

All scripts should follow these guidelines:

**Requirements:**
- ✅ Shebang and description at top (`#!/bin/bash`)
- ✅ Error handling (`set -euo pipefail`)
- ✅ Color-coded output (success/error/info)
- ✅ Help text (`--help` or `-h` flag)
- ✅ CI/CD compatible (non-interactive by default)
- ✅ Compatible with macOS and Linux
- ✅ Comments for non-obvious sections

**Example Script Template:**

```bash
#!/bin/bash
# Purpose: Brief description of what this script does
# Usage: script-name.sh [OPTIONS]

set -euo pipefail

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m'  # No Color

# Functions
print_success() { echo -e "${GREEN}✓ $1${NC}"; }
print_error() { echo -e "${RED}✗ $1${NC}"; }
print_info() { echo -e "${BLUE}ℹ $1${NC}"; }

print_help() {
  cat << EOF
Usage: script-name.sh [OPTIONS]

Options:
  --help     Show this message
  --verbose  Enable verbose output
EOF
}

# Main
if [[ "${1:-}" == "--help" ]]; then
  print_help
  exit 0
fi

# Your code here...
```

## See Also

- [BENCHMARKING.md](../BENCHMARKING.md) - Comprehensive benchmarking guide
- [CONTRIBUTING.md](../CONTRIBUTING.md) - Development setup and workflow

## Testing Scripts

Test scripts locally before committing:

```bash
# Benchmarking
./scripts/benchmark.sh --bench estimator

# Plotting
python3 scripts/plot_benchmarks.py --plot-types all
```
