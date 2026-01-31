# Scripts Directory Improvement Plan

## Executive Summary

After a thorough review of the `/scripts` directory, I've identified several critical issues affecting maintainability, usability, and reliability. This document provides a comprehensive critique and actionable improvement plan.

---

## Critical Findings

### ✅ **What Works Well**

1. **Core Runner Scripts** (`run_euroc.sh`, `run_tum-vi.sh`, `run_4seasons.sh`)
   - ✅ Well-structured, proper error handling (`set -euo pipefail`)
   - ✅ Correctly invoke Rust binaries via `cargo run --release --bin <name>`
   - ✅ Environment variable support for RUST_LOG and CONFIG_PATH
   - ✅ Clean argument passing
   - **Status**: **KEEP AS-IS** (minimal improvements needed)

2. **Development Tools**
   - `lint_shell.sh` - Simple, effective shellcheck integration
   - `bump-version.sh` - Automated version management with CHANGELOG updates
   - `generate-docs.sh` - Comprehensive documentation generation
   - **Status**: **KEEP** (minor improvements needed)

3. **Dataset Management**
   - `generate_test_datasets.sh` - Creates synthetic datasets for testing
   - `setup-datasets.sh` - Comprehensive dataset download/setup automation
   - **Status**: **KEEP** (quality improvements needed)

---

## 🚨 Critical Issues

### 1. **Massive Duplication in Plotting/Analysis Scripts**

**Problem**: THREE separate Python scripts perform overlapping visualization tasks:

| Script | Lines | Purpose | Duplication Level |
|--------|-------|---------|------------------|
| `evaluate_and_plot.py` | ~200 | Run VIO + generate plots | **HIGH** |
| `create_detailed_plots.py` | ~300 | Generate performance plots | **HIGH** |
| `plot_benchmarks.py` | ~250 | Parse Criterion results + plot | **MEDIUM** |

**Evidence**:
```python
# All three use similar matplotlib boilerplate:
import matplotlib.pyplot as plt
fig, axes = plt.subplots(...)
axes.plot(...)
plt.savefig(...)
```

**Impact**:
- Maintenance burden (bug fixes need 3× effort)
- Inconsistent plot styling
- Confusion about which script to use
- ~750 lines of redundant code

---

### 2. **Hardcoded Paths Throughout**

**Problem**: Multiple scripts contain hardcoded absolute paths:

```python
# evaluate_and_plot.py
DATASET_PATH = "/Users/vincent/Work/RS-VIO/datasets/euroc/MH_01_easy"

# create_detailed_plots.py
results_file = "/tmp/rs-vio-samples/evaluation_results.json"

# benchmark_vio.py
default_binary = "/Users/vincent/Work/RS-VIO/target/release/run_euroc"
```

**Impact**:
- Scripts fail on different machines
- Not portable for CI/CD
- Requires manual editing for each user

---

### 3. **Unclear Script Usage & Documentation Mismatch**

**Problem**: `README.md` documents extensive workflows, but actual usage is unclear:

```bash
# README.md says:
./scripts/evaluate_and_plot.py --dataset /path/to/euroc

# But script has:
DATASET_PATH = "/Users/vincent/Work/RS-VIO/datasets/euroc/MH_01_easy"  # Hardcoded!
```

**Evidence from documentation**:
- `EXECUTION_GUIDE.md` references scripts 20+ times
- `OPTIMIZATION_REPORT.md` shows script outputs
- But no clear "quick start" for new users

---

### 4. **Unused/Deprecated Scripts**

**Candidates for removal**:

1. **`benchmark_datasets.sh`** (53 lines)
   - Appears to be superseded by `orchestrate.sh`
   - No references in documentation

2. **`evaluate_imu_prior.sh`** (102 lines)
   - Contains inline Python instead of calling Python scripts
   - Duplicates functionality in `evaluate_and_plot.py`
   - Confusing dual-language approach

3. **`docker-automation.sh`** (204 lines)
   - Comprehensive but no active Docker workflows in CI/CD
   - `Dockerfile` exists but unclear if used

---

### 5. **Python Script Quality Issues**

**Type Safety**:
- Recently added type hints, but many are incomplete:
  ```python
  def plot_trajectory(data):  # No type hints
      pass

  def run_benchmark(config: dict) -> None:  # dict is too generic
      pass
  ```

**Error Handling**:
```python
# evaluate_and_plot.py
result = subprocess.run([binary, ...])  # No check of returncode
data = json.load(f)  # No exception handling
```

**Testing**:
- ❌ Zero unit tests for Python scripts
- ❌ No integration tests
- ❌ No CI/CD validation

---

### 6. **Shell Script Quality Issues**

**`orchestrate.sh` (346 lines)**:
- ⚠️ Extremely long, does too many things:
  - Build management
  - Testing orchestration
  - Dataset running
  - Benchmark execution
  - Report generation
- Violates single responsibility principle
- Difficult to debug failures (which step failed?)

**`setup-datasets.sh` (427 lines)**:
- Very comprehensive but:
  - No progress indicators for long downloads
  - No checksum verification (security risk)
  - No resume capability for failed downloads

---

## Improvement Plan

### Phase 1: Consolidation (High Priority)

#### 1.1 Merge Python Plotting Scripts
**Action**: Create single `scripts/vio_analysis.py` module

```python
# New structure:
scripts/vio_analysis.py
  ├── class BenchmarkRunner
  ├── class TrajectoryPlotter
  ├── class CriterionParser
  └── main() with CLI
```

**Benefits**:
- Single source of truth
- Consistent plotting style
- Reduced maintenance
- Better testability

**Migration**:
```bash
# Deprecate and redirect:
evaluate_and_plot.py -> vio_analysis.py --mode trajectory
create_detailed_plots.py -> vio_analysis.py --mode detailed
plot_benchmarks.py -> vio_analysis.py --mode criterion
```

#### 1.2 Remove Deprecated Scripts
**Delete**:
- `benchmark_datasets.sh` (superseded by `orchestrate.sh`)
- `evaluate_imu_prior.sh` (use Python scripts instead)

**Archive** (move to `scripts/archive/`):
- `docker-automation.sh` (keep for future Docker work)

---

### Phase 2: Path Standardization (High Priority)

#### 2.1 Environment Variable Convention
**Add to all scripts**:
```bash
# Shell scripts
PROJECT_ROOT="${PROJECT_ROOT:-$(git rev-parse --show-toplevel)}"
DATASETS_DIR="${DATASETS_DIR:-${PROJECT_ROOT}/datasets}"
```

```python
# Python scripts
import os
PROJECT_ROOT = os.environ.get('RS_VIO_ROOT', Path(__file__).parent.parent)
DATASETS_DIR = Path(os.environ.get('RS_VIO_DATASETS', PROJECT_ROOT / 'datasets'))
```

#### 2.2 Configuration File
**Create**: `scripts/config.yaml`
```yaml
paths:
  datasets: "${HOME}/rs-vio-datasets"
  results: "${PROJECT_ROOT}/evaluation_results"

binaries:
  run_euroc: "target/release/run_euroc"
  run_tum: "target/release/run_tum"
  run_4seasons: "target/release/run_4seasons"

plotting:
  dpi: 300
  style: "seaborn-v0_8-darkgrid"
  figsize: [12, 8]
```

---

### Phase 3: Documentation & Usability (Medium Priority)

#### 3.1 Update README.md
**Restructure** `scripts/README.md`:

```markdown
# RS-VIO Scripts

## Quick Start (80% use case)
./scripts/run_euroc.sh /path/to/dataset

## Running Benchmarks
./scripts/vio_analysis.py --help

## Dataset Setup
./scripts/setup-datasets.sh ~/datasets

## Development Tools
./scripts/lint_shell.sh
./scripts/bump-version.sh patch
```

#### 3.2 Add Examples Directory
**Create**: `scripts/examples/`
```
scripts/examples/
├── basic_euroc_run.sh
├── full_benchmark_workflow.sh
└── plotting_custom.py
```

---

### Phase 4: Quality Improvements (Medium Priority)

#### 4.1 Add Python Testing
**Create**: `scripts/tests/`
```python
# tests/test_vio_analysis.py
import pytest
from vio_analysis import BenchmarkRunner, TrajectoryPlotter

def test_benchmark_runner_init():
    runner = BenchmarkRunner(binary="dummy")
    assert runner.binary == "dummy"

def test_trajectory_plotter_formats():
    plotter = TrajectoryPlotter()
    assert "euroc" in plotter.supported_formats()
```

**Add to CI/CD**:
```yaml
# .github/workflows/test.yml
- name: Test Python scripts
  run: |
    cd scripts
    python -m pytest tests/
```

#### 4.2 Shell Script Improvements

**`orchestrate.sh` - Refactor into modules**:
```bash
scripts/orchestrate/
├── build.sh
├── test.sh
├── benchmark.sh
└── orchestrate.sh (main dispatcher)
```

**`setup-datasets.sh` - Add safety**:
```bash
# Add checksum verification
download_and_verify() {
  local url="$1" sha256="$2" filepath="$3"
  download_file "$url" "$filepath"
  echo "$sha256  $filepath" | sha256sum -c -
}
```

---

### Phase 5: Advanced Improvements (Low Priority)

#### 5.1 Python Package Structure
**Convert to proper package**:
```
scripts/
├── setup.py
├── rs_vio_tools/
│   ├── __init__.py
│   ├── analysis.py
│   ├── plotting.py
│   └── benchmarking.py
└── tests/
```

**Install as editable package**:
```bash
pip install -e scripts/
rs-vio-analysis --help  # Clean CLI
```

#### 5.2 Add Pre-commit Hooks
**Create**: `.pre-commit-config.yaml`
```yaml
repos:
  - repo: https://github.com/koalaman/shellcheck-precommit
    hooks:
      - id: shellcheck

  - repo: https://github.com/astral-sh/ruff-pre-commit
    hooks:
      - id: ruff
        args: [--fix]
      - id: ruff-format
```

---

## Prioritized Action Items

### 🔴 **Critical (Do Now)**

1. ✅ **Create backup branch**: `git checkout -b refactor/scripts` ✓ (Already done)
2. **Merge plotting scripts** into `vio_analysis.py`
3. **Remove hardcoded paths** - use environment variables
4. **Delete deprecated scripts**: `benchmark_datasets.sh`, `evaluate_imu_prior.sh`

### 🟡 **Important (This Week)**

5. **Update `README.md`** with clear quick start
6. **Add path configuration** via `config.yaml`
7. **Refactor `orchestrate.sh`** into modular components
8. **Add checksums** to `setup-datasets.sh`

### 🟢 **Nice to Have (This Month)**

9. **Add Python tests** with pytest
10. **Create examples/** directory with common workflows
11. **Add CI/CD validation** for scripts
12. **Improve type hints** across Python scripts

---

## Migration Strategy

### Step 1: Preparation
```bash
# Create backup
git checkout -b refactor/scripts-backup

# Document current usage
git grep -l "scripts/" > scripts_usage.txt
```

### Step 2: Consolidation
```bash
# Create unified analysis script
touch scripts/vio_analysis.py
# Migrate code from evaluate_and_plot.py, create_detailed_plots.py, plot_benchmarks.py

# Test new script
python scripts/vio_analysis.py --help
python scripts/vio_analysis.py --mode trajectory --dataset /path/to/euroc
```

### Step 3: Cleanup
```bash
# Archive old scripts
mkdir -p scripts/archive
git mv scripts/evaluate_and_plot.py scripts/archive/
git mv scripts/create_detailed_plots.py scripts/archive/
git mv scripts/plot_benchmarks.py scripts/archive/

# Delete deprecated
git rm scripts/benchmark_datasets.sh
git rm scripts/evaluate_imu_prior.sh
```

### Step 4: Documentation
```bash
# Update README
vim scripts/README.md

# Update references in docs
grep -rl "evaluate_and_plot" docs/ | xargs sed -i 's/evaluate_and_plot/vio_analysis/g'
```

---

## Success Metrics

After refactoring, we should achieve:

| Metric | Before | Target | Measurement |
|--------|--------|--------|-------------|
| Total Python LOC | ~1200 | ~600 | `cloc scripts/*.py` |
| Script count | 23 | ~15 | `ls scripts/*.{sh,py} \| wc -l` |
| Hardcoded paths | 15+ | 0 | `grep -r "/Users/vincent" scripts/` |
| Test coverage | 0% | 80% | `pytest --cov` |
| Documentation completeness | 60% | 95% | Manual review |

---

## Risks & Mitigations

| Risk | Impact | Mitigation |
|------|--------|------------|
| Breaking existing workflows | HIGH | Keep old scripts in `archive/` for 1 release |
| Users have custom modifications | MEDIUM | Document migration in `CHANGELOG.md` |
| CI/CD depends on old paths | MEDIUM | Update CI/CD before merging |
| Performance regression | LOW | Benchmark before/after |

---

## Conclusion

The scripts directory has **significant technical debt** but is fundamentally useful. The core runner scripts are excellent, but the analysis/plotting ecosystem needs consolidation. By following this phased approach, we can:

1. **Reduce complexity** by 50% (fewer files, clearer responsibilities)
2. **Improve portability** (no hardcoded paths)
3. **Increase reliability** (tests, type safety)
4. **Better documentation** (clearer usage patterns)

**Recommended Next Step**: Start with **Phase 1 (Consolidation)** - merge the three plotting scripts into `vio_analysis.py`. This single change will eliminate ~750 lines of duplicated code and provide immediate value.
