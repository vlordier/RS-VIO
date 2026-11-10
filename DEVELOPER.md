# RS-VIO Developer Guide

## Quick Reference

```bash
# Most common commands
make build          # Build release binary
make test           # Run all tests
make bench          # Run all benchmarks
make measure        # Run performance measurements
make help           # Show all commands
```

## Development Workflow

### 1. Daily Development

```bash
# Format code, run tests, and lint
make dev

# Or individually:
make fmt      # Format with rustfmt
make test     # Run tests
make lint     # Run clippy
```

### 2. Before Committing

```bash
# Full CI-style check
make ci

# This runs:
# - fmt-check (verify formatting)
# - test (all tests pass)
# - lint (clippy warnings)
```

### 3. Performance Measurement

```bash
# Complete measurement suite (~20 min)
make measure DATASET=/Users/vincent/data/tum-vi/dataset-corridor1_512_16

# Just motion prior overhead (~3 min)
make measure-overhead

# Just accuracy comparison (~15 min)
make measure-accuracy DATASET=/path/to/dataset

# View results
make results

# Open results directory
make results-dir
```

### 4. Benchmarks

```bash
# All benchmarks
make bench

# Individual benchmarks
make bench-motion    # Motion prior overhead
make bench-sliding   # Sliding window
make bench-feature   # Feature tracker
```

## Command Reference

### Build Commands

| Command | Description | Time |
|---------|-------------|------|
| `make build` | Build release binary | ~2 min |
| `make debug` | Build debug binary | ~30 sec |
| `make clean` | Clean build artifacts | ~5 sec |

### Test Commands

| Command | Description | Time |
|---------|-------------|------|
| `make test` | Run all tests | ~5 min |
| `make test-fast` | Run fast tests only | ~1 min |
| `make test-overhead` | Run overhead measurement | ~30 sec |

### Benchmark Commands

| Command | Description | Time | Output |
|---------|-------------|------|--------|
| `make bench` | All benchmarks | ~10 min | Console + logs |
| `make bench-motion` | Motion prior overhead | ~3 min | `performance_results/motion_prior_*.log` |
| `make bench-sliding` | Sliding window | ~5 min | `performance_results/sliding_window_*.log` |
| `make bench-feature` | Feature tracker | ~2 min | `performance_results/feature_*.log` |

### Measurement Commands

| Command | Description | Time | Output |
|---------|-------------|------|--------|
| `make measure` | Complete measurement | ~20 min | `performance_results/` |
| `make measure-overhead` | Motion prior only | ~3 min | `performance_results/motion_prior_*.log` |
| `make measure-accuracy` | Accuracy comparison | ~15 min | `performance_results/accuracy_*.json` |

**Note:** `make measure-accuracy` requires a dataset path:
```bash
make measure-accuracy DATASET=/Users/vincent/data/tum-vi/dataset-corridor1_512_16
```

### Code Quality Commands

| Command | Description | Auto-fix |
|---------|-------------|----------|
| `make fmt` | Format code | ✅ |
| `make fmt-check` | Check formatting | ❌ |
| `make lint` | Run clippy | ❌ |
| `make lint-fix` | Auto-fix clippy | ✅ |

### Documentation Commands

| Command | Description |
|---------|-------------|
| `make docs` | Generate docs |
| `make docs-open` | Generate and open docs |

### Utility Commands

| Command | Description |
|---------|-------------|
| `make results` | View latest results |
| `make results-dir` | Open results directory |
| `make commit-measure` | Commit measurement infrastructure |
| `make help` | Show all commands |

## Performance Measurement Workflow

### Step 1: Run Benchmarks

```bash
# Run all benchmarks
make bench

# Output shows:
# process_noise_scale/quadrotor
#                         time:   [XX.XX ns XX.XX ns XX.XX ns]
# gate_measurement/quadrotor
#                         time:   [XX.XX ns XX.XX ns XX.XX ns]
```

### Step 2: Run Accuracy Comparison

```bash
# Run on corridor1 (most important)
make measure-accuracy DATASET=/Users/vincent/data/tum-vi/dataset-corridor1_512_16

# Output shows:
# ATE RMSE: 0.61m → 0.XXm
# Improvement: XX.X% (X.X× better)
```

### Step 3: View Results

```bash
# View summary
make results

# Open directory
make results-dir

# View specific file
cat performance_results/motion_prior_*.log
cat performance_results/accuracy_*.json
```

### Step 4: Update Documentation

Edit these files with actual measured numbers:
- `PERFORMANCE_MEASUREMENT_STATUS.md`
- `BENCHMARK.md`
- `PLATFORM_PRIORS_COMPLETE.md`

## Troubleshooting

### Build Issues

```bash
# Clean and rebuild
make clean && make build

# Update dependencies
cargo update
```

### Test Failures

```bash
# Run specific test
cargo test --lib <test_name>

# Run with output
cargo test --lib -- --nocapture
```

### Benchmark Issues

```bash
# Run with verbose output
cargo bench --bench motion_prior -- --verbose

# Run single benchmark
cargo bench --bench motion_prior process_noise_scale
```

### Measurement Issues

```bash
# Check dataset exists
ls -la /Users/vincent/data/tum-vi/dataset-corridor1_512_16

# Run script manually
python3 scripts/measure_prior_accuracy.py \
    /Users/vincent/data/tum-vi/dataset-corridor1_512_16 \
    --json test.json
```

## Customization

### Override Default Dataset

```bash
# Set in command
make measure DATASET=/path/to/other/dataset

# Or export environment variable
export DATASET=/path/to/other/dataset
make measure
```

### Custom Benchmark Config

Edit `benches/motion_prior.rs` to add custom benchmarks.

### Custom Make Targets

Add to `Makefile`:
```makefile
my-custom-target:
	@echo "Custom command"
	cargo my-custom-command
```

## CI/CD Integration

The Makefile is designed for CI integration:

```yaml
# GitHub Actions example
jobs:
  test:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v2
      - name: Run tests
        run: make ci
  
  bench:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v2
      - name: Run benchmarks
        run: make bench
```

## Performance Tips

1. **Use release mode** for accurate timing:
   ```bash
   make build  # Uses --release
   ```

2. **Run benchmarks multiple times** for stable results:
   ```bash
   for i in {1..3}; do make bench-motion; done
   ```

3. **Close other applications** during measurements for consistent results.

4. **Use SSD** for dataset storage to avoid I/O bottlenecks.

## Additional Resources

- `BENCHMARK.md` - Detailed benchmark results
- `PERFORMANCE_MEASUREMENT_STATUS.md` - What's measured vs estimated
- `PLATFORM_PRIORS_COMPLETE.md` - Motion prior implementation details
- `MSCKF_ARCHITECTURE.md` - Future MSCKF architecture

---

**For questions or issues, see the main README.md or open an issue.**
