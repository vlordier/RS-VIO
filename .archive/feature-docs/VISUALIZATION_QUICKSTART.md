# Quick Start: Visualization Commands

## Using Makefile (Recommended)

### Generate Everything
```bash
make viz                    # All: demo + TUM-VI + plots
```

### Individual Steps
```bash
make viz-demo              # Generate synthetic demo data
make viz-tum               # Process TUM-VI dataset
make viz-install-python    # Install Python dependencies
make viz-plots             # Generate plots from demo data
make viz-plots-tum         # Generate plots from TUM-VI data
```

### Example Workflow
```bash
# Full workflow
make viz-demo && make viz-plots

# Or for TUM-VI
make viz-tum && make viz-plots-tum

# Or everything at once
make viz
```

## Using Shell Script

### Quick Commands
```bash
# Generate everything
./scripts/visualize.sh --all

# Demo only
./scripts/visualize.sh --demo --plots

# TUM-VI only
./scripts/visualize.sh --tum-vi --plots

# Install Python dependencies
./scripts/visualize.sh --install-deps

# Clean generated files
./scripts/visualize.sh --clean
```

### Custom Dataset Path
```bash
export DATASET_DIR=/path/to/your/datasets
./scripts/visualize.sh --tum-vi --plots
```

## Manual Commands

### Synthetic Demo
```bash
# 1. Generate data
cargo run --example plot_vio_comparisons

# 2. Install Python deps (first time only)
pip install pandas matplotlib numpy

# 3. Generate plots
python3 ./plot_output/plot_comparisons.py
```

### TUM-VI Dataset
```bash
# 1. Generate data
cargo run --example plot_tum_vi_comparison -- /tmp/rs-vio-samples/tum_vi/room1

# 2. Install Python deps (first time only)
pip install pandas matplotlib numpy

# 3. Generate plots
python3 ./tum_vi_results/plot_comparisons.py
```

## Output Files

### Synthetic Demo
```
plot_output/
├── tracking_comparison.csv
├── disparity_comparison.csv
├── rolling_shutter_comparison.csv
├── plot_comparisons.py
├── tracking_comparison.png           (after plots)
├── disparity_comparison.png          (after plots)
└── rolling_shutter_comparison.png    (after plots)
```

### TUM-VI Dataset
```
tum_vi_results/
├── tracking_comparison.csv
├── disparity_comparison.csv
├── rolling_shutter_comparison.csv
├── plot_comparisons.py
├── tracking_comparison.png           (after plots)
├── disparity_comparison.png          (after plots)
└── rolling_shutter_comparison.png    (after plots)
```

## Makefile Targets Reference

| Target | Description |
|--------|-------------|
| `make viz` | Generate all visualizations (demo + TUM-VI + plots) |
| `make viz-demo` | Generate synthetic demo data |
| `make viz-tum` | Process TUM-VI dataset |
| `make viz-install-python` | Install Python plotting dependencies |
| `make viz-plots` | Generate plots from demo data |
| `make viz-plots-tum` | Generate plots from TUM-VI data |

## Script Options Reference

| Option | Description |
|--------|-------------|
| `--demo` | Generate synthetic demo only |
| `--tum-vi` | Generate TUM-VI visualization only |
| `--plots` | Generate plots from existing data |
| `--all` | Generate everything |
| `--install-deps` | Install Python dependencies only |
| `--clean` | Clean generated files |
| `--help` | Show help message |

## Common Issues

### TUM-VI dataset not found
```bash
# Download dataset
wget https://vision.in.tum.de/tumvi/exported/euroc/512_16/dataset-room1_512_16.tar

# Extract
mkdir -p /tmp/rs-vio-samples/tum_vi
tar xf dataset-room1_512_16.tar -C /tmp/rs-vio-samples/tum_vi/

# Or use custom path
export DATASET_DIR=/your/path
make viz-tum
```

### Python dependencies not installed
```bash
# Quick install
make viz-install-python

# Or manually
pip install pandas matplotlib numpy
```

### Plots not generating
```bash
# Check Python installation
python3 -c "import pandas, matplotlib, numpy; print('OK')"

# Reinstall if needed
pip3 install --upgrade pandas matplotlib numpy

# Run with verbose output
cd plot_output && python3 plot_comparisons.py
```

## Performance

**Synthetic Demo**:
- Generation: ~2 seconds
- Plotting: ~1 second
- Total: ~3 seconds

**TUM-VI (200 frames)**:
- Data loading: ~0.1 seconds
- Processing: ~1.5 seconds
- Plotting: ~1 second
- Total: ~2.6 seconds

**Full TUM-VI (2,821 frames)**:
- Edit line 62 in `examples/plot_tum_vi_comparison.rs`: `let max_frames = 2821;`
- Expected time: ~20 seconds

## Tips

### Faster Builds
```bash
# Use release mode for faster execution
cargo build --release --example plot_vio_comparisons
cargo build --release --example plot_tum_vi_comparison
```

### Parallel Generation
```bash
# Generate both in parallel
make viz-demo & make viz-tum & wait
make viz-plots & make viz-plots-tum & wait
```

### Custom Output Directories
Edit the examples to change output paths:
- `examples/plot_vio_comparisons.rs` line 67
- `examples/plot_tum_vi_comparison.rs` line 67

### Integration with CI
```bash
# Add to CI pipeline
make viz-demo
make viz-plots
# Upload artifacts: plot_output/*.png
```

## See Also

- [VISUALIZATION_GUIDE.md](VISUALIZATION_GUIDE.md) - Complete API documentation
- [TUM_VI_RESULTS.md](TUM_VI_RESULTS.md) - Detailed results analysis
- [RUN_DATASET_COMPARISON.md](RUN_DATASET_COMPARISON.md) - Dataset usage guide
