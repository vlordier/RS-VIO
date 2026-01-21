# Migration from Make to Just

## Overview

RS-VIO has been migrated from GNU Make to [Just](https://github.com/casey/just) as the primary task automation tool. Just is a modern, Rust-native task runner that aligns better with the Rust ecosystem and provides superior recipe syntax and error handling.

## Installation

### macOS
```bash
brew install just
```

### Linux (Ubuntu/Debian)
```bash
apt install just
```

### Other systems
See https://github.com/casey/just#installation

## Quick Start

List all available recipes:
```bash
just -l
```

Run a recipe:
```bash
just <recipe-name>
```

Common recipes:
```bash
just build              # Build debug binary
just release           # Build release (optimized)
just test              # Run all tests
just check             # Full code quality checks
just setup-datasets    # Download and setup datasets
just run-euroc         # Run EuRoC benchmark
just ci                # Full CI pipeline
```

## Recipe Categories

### Build & Test
- `build` - Debug build
- `release` - Optimized release build
- `test` - Run all debug tests
- `test-release` - Run all release tests

### Code Quality
- `fmt` - Auto-format code (rustfmt)
- `clippy` - Lint with clippy
- `clippy-fix` - Auto-fix clippy warnings
- `audit` - Security audit
- `check` - All quality checks (fmt + clippy + audit + lint)
- `quality` - Extended quality pipeline (includes deps, test, doc)

### Datasets & Benchmarks
- `download-datasets` - Download EuRoC, TUM-VI, 4Seasons
- `setup-datasets` - Download and verify datasets
- `verify-datasets` - Verify dataset integrity
- `run-euroc` - Run EuRoC benchmark
- `run-tum` - Run TUM-VI benchmark
- `run-4seasons` - Run 4Seasons benchmark
- `run` - Run all benchmarks
- `benchmark-all` - Complete benchmark suite

### Visualization
- `viz` - Generate all visualizations
- `viz-demo` - Synthetic demo visualization
- `viz-tum` - TUM-VI dataset visualization
- `viz-install-python` - Install Python dependencies
- `viz-plots` - Generate plots from demo
- `viz-plots-tum` - Generate plots from TUM-VI

### Docker
- `docker` / `docker-build` - Build Docker image
- `docker-test` - Build and test Docker image
- `docker-push` - Push Docker image to registry

### Advanced Quality Tools
- `deps` - Check all dependencies (unused, outdated, licenses, duplicates)
- `coverage` - Generate code coverage report
- `doc` - Generate documentation
- `miri` - Run MIRI static analysis (nightly)
- `unsafe` - Analyze unsafe code
- `mutants` - Run mutation testing
- `bloat` - Analyze binary size
- `semver` - Check semantic versioning
- `msrv` - Check MSRV (Minimum Supported Rust Version)

### Maintenance
- `clean` - Clean build artifacts
- `clean-all` - Deep clean (including Docker)
- `install-tools` - Install all quality tools

## Configuration

Dataset directory can be customized via environment variable:
```bash
DATASET_DIR=/path/to/datasets just setup-datasets
```

Default: `/tmp/rs-vio-samples`

## Migration Details

### What Changed
- ✅ Removed: `Makefile` → archived to `.archived/Makefile.bak`
- ✅ Removed: `Makefile.quality` → archived to `.archived/Makefile.quality.bak`
- ✅ Created: `justfile` with all recipes
- ✅ Updated: Documentation references (DATASETS.md, QUICKSTART.md, etc.)
- ✅ Updated: Scripts with just references

### Recipe Equivalents

| Make | Just |
|------|------|
| `make build` | `just build` |
| `make release` | `just release` |
| `make test` | `just test` |
| `make test-release` | `just test-release` |
| `make check` | `just check` |
| `make fmt` | `just fmt` |
| `make clippy` | `just clippy` |
| `make clippy-fix` | `just clippy-fix` |
| `make audit` | `just audit` |
| `make quality` | `just quality` |
| `make setup-datasets` | `just setup-datasets` |
| `make run-euroc` | `just run-euroc` |
| `make ci` | `just ci` |
| `make docker` | `just docker` |
| `make clean` | `just clean` |

## Benefits of Just

1. **Rust-native** - Built in Rust, aligns with our ecosystem
2. **Better syntax** - YAML-free, shell-friendly recipe definitions
3. **Superior error handling** - Clear error messages and failure propagation
4. **Shellcheck integration** - Recipes are validated shell scripts
5. **Environment variables** - First-class support via `{{ var }}`
6. **Dependencies** - Explicit recipe dependencies with `@require`
7. **Cross-platform** - Works on Windows, macOS, Linux uniformly
8. **No implicit targets** - Clearer intention; recipes are explicit

## For CI/CD Systems

GitHub Actions, GitLab CI, and other systems can use just directly:
```yaml
# GitHub Actions example
- name: Run quality checks
  run: just check

- name: Run tests
  run: just test-release

- name: Build and push Docker
  run: just docker-push
```

## Troubleshooting

### "just: command not found"
Install just: `brew install just` (macOS) or `apt install just` (Linux)

### Recipe fails with "No such file or directory"
Some recipes (e.g., `run-euroc`) require datasets to be pre-downloaded. Run:
```bash
just setup-datasets
```

### Custom dataset directory
Set `DATASET_DIR` before running:
```bash
export DATASET_DIR=/data/rs-vio
just run-euroc
```

## See Also

- [justfile](justfile) - Complete recipe definitions
- [Just documentation](https://just.systems/)
- Original Makefiles archived in [.archived/](.archived/)

---

**Migrated**: January 21, 2026  
**Status**: All recipes tested and validated
