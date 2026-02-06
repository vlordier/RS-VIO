# RS-VIO automation via just
set shell := ["bash", "-euo", "pipefail", "-c"]

default := "help"

# Paths and binaries
build_dir := "target"
release_dir := "{{build_dir}}/release"
debug_dir := "{{build_dir}}/debug"
config_dir := "config"
script_dir := "scripts"
dataset_dir := env_var_or_default("DATASET_DIR", "datasets")

euroc_bin := "{{release_dir}}/run_euroc"
tum_bin := "{{release_dir}}/run_tum"
four_seasons_bin := "{{release_dir}}/run_4seasons"

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
help:
    echo "RS-VIO just recipes"
    echo "Dataset dir: {{dataset_dir}}"
    echo "Use 'just -l' to list all recipes."

info:
    echo "Rust version:" && rustc --version
    echo "Cargo version:" && cargo --version
    echo "OS: $(uname -s)" && echo "Arch: $(uname -m)"
    echo "Build dir: {{build_dir}}"

# ---------------------------------------------------------------------------
# Build & Test
# ---------------------------------------------------------------------------
build:
    echo "Building debug binary..."
    cargo build

release:
    echo "Building release binary (optimized)..."
    cargo build --release

test: build
    echo "Running debug tests..."
    cargo test --all

test-release: release
    echo "Running release tests..."
    cargo test --all --release

# ---------------------------------------------------------------------------
# Code Quality
# ---------------------------------------------------------------------------
fmt:
    echo "Formatting code..."
    cargo fmt --all

clippy: release
    echo "Linting with clippy..."
    cargo clippy --all --release -- -D warnings

clippy-fix:
    echo "Auto-fixing clippy warnings..."
    cargo clippy --fix --all-targets --all-features --allow-dirty

audit:
    echo "Running security audit..."
    cargo audit

lint:
    echo "Linting shell scripts..."
    if [ -x "{{script_dir}}/lint_shell.sh" ]; then \
      {{script_dir}}/lint_shell.sh; \
    else \
      echo "Warning: lint_shell.sh not found or not executable"; \
    fi

check: fmt clippy audit lint
    echo "All code quality checks passed."

ci: check test-release lint
    echo "Full CI pipeline passed."

# ---------------------------------------------------------------------------
# Dataset Management
# ---------------------------------------------------------------------------
download-datasets:
    echo "Downloading datasets to {{dataset_dir}}..."
    mkdir -p {{dataset_dir}}
    bash {{script_dir}}/download_datasets.sh {{dataset_dir}} all
    echo "Download complete."

setup-datasets:
    echo "Setting up datasets with verification..."
    if [ -x "{{script_dir}}/setup-datasets.sh" ]; then \
      bash {{script_dir}}/setup-datasets.sh; \
    else \
      echo "setup-datasets.sh not found. Falling back to download."; \
      just download-datasets; \
    fi
    echo "Dataset setup complete."

verify-datasets:
    echo "Verifying datasets in {{dataset_dir}}..."
    for dataset in euroc tum_vi 4seasons; do \
      echo "Checking $$dataset..."; \
      if [ -d "{{dataset_dir}}/$$dataset" ]; then \
        echo "  ✓ $$dataset found"; \
      else \
        echo "  ⚠ $$dataset not found"; \
      fi; \
    done

# ---------------------------------------------------------------------------
# Benchmarks
# ---------------------------------------------------------------------------
run: run-euroc run-tum run-4seasons
    echo "All benchmarks completed."

run-euroc: release
    echo "Running EuRoC benchmark..."
    if [ -d "{{dataset_dir}}/euroc/MH_01_easy" ]; then \
      timeout 120 {{euroc_bin}} {{config_dir}}/euroc_vio.yaml {{dataset_dir}}/euroc/MH_01_easy || true; \
      echo "EuRoC benchmark complete."; \
    else \
      echo "EuRoC dataset not found. Run: just setup-datasets"; \
      exit 1; \
    fi

run-tum: release
    echo "Running TUM-VI benchmark..."
    if [ -d "{{dataset_dir}}/tum_vi" ]; then \
      timeout 120 {{tum_bin}} {{config_dir}}/tum_vi.yaml {{dataset_dir}}/tum_vi || true; \
      echo "TUM-VI benchmark complete."; \
    else \
      echo "TUM-VI dataset not found. Run: just setup-datasets"; \
      exit 1; \
    fi

run-4seasons: release
    echo "Running 4Seasons benchmark..."
    if [ -d "{{dataset_dir}}/4seasons" ]; then \
      recording_dir=$(ls -d {{dataset_dir}}/4seasons/recording_* 2>/dev/null | head -1); \
      if [ -z "$$recording_dir" ]; then \
        echo "No 4Seasons recordings found."; \
        exit 1; \
      fi; \
      echo "Using recording: $(basename $$recording_dir)"; \
      timeout 120 {{euroc_bin}} {{config_dir}}/4seasons.yaml "$$recording_dir" || true; \
      echo "4Seasons benchmark complete."; \
    else \
      echo "4Seasons dataset not found. Run: just setup-datasets"; \
      exit 1; \
    fi

setup-4seasons:
    echo "4Seasons Dataset Setup Guide"
    echo "  1. Register at https://www.4seasons-dataset.com/"
  echo "  2. Download a recording ZIP to {{dataset_dir}}/downloads"
    echo "  3. Run: ./scripts/setup-datasets.sh"
    echo "Check current ZIPs:"
  ls -lh {{dataset_dir}}/downloads/recording_*.zip 2>/dev/null || echo "  No 4Seasons ZIP found in {{dataset_dir}}/downloads/"
    if [ -d "{{dataset_dir}}/4seasons" ]; then \
      echo "Extracted datasets:"; \
      ls -d {{dataset_dir}}/4seasons/*/ 2>/dev/null; \
    else \
      echo "No extracted datasets yet."; \
    fi

benchmark-all:
    echo "Running complete benchmark suite..."
    if [ -x "{{script_dir}}/orchestrate.sh" ]; then \
      bash {{script_dir}}/orchestrate.sh all; \
    else \
      echo "orchestrate.sh not found."; \
      exit 1; \
    fi

# ---------------------------------------------------------------------------
# Docker
# ---------------------------------------------------------------------------
docker: docker-build
    echo "Docker image built: rs-vio:latest"

docker-build:
    echo "Building Docker image..."
    docker build -t rs-vio:latest .

docker-test: docker-build
    echo "Testing Docker image..."
    docker run --rm rs-vio:latest --help
    echo "Docker smoke test passed."

docker-push:
    echo "Pushing Docker image..."
    read -p "Enter Docker registry (e.g., docker.io/username): " registry; \
    docker tag rs-vio:latest $$registry/rs-vio:latest; \
    docker push $$registry/rs-vio:latest; \
    echo "Push complete."

# ---------------------------------------------------------------------------
# Visualization
# ---------------------------------------------------------------------------
viz: viz-demo viz-tum viz-plots viz-plots-tum
    echo "All visualizations generated."

viz-demo:
    echo "Generating synthetic demonstration data..."
    cargo run --example plot_vio_comparisons
    echo "Demo data generated in ./plot_output/"

viz-tum:
    echo "Processing TUM-VI dataset..."
    if [ -d "{{dataset_dir}}/tum_vi/room1" ]; then \
      cargo run --example plot_tum_vi_comparison -- {{dataset_dir}}/tum_vi/room1; \
      echo "TUM-VI data generated in ./tum_vi_results/"; \
    else \
      echo "TUM-VI dataset not found at {{dataset_dir}}/tum_vi/room1"; \
      echo "Download from https://vision.in.tum.de/data/datasets/visual-inertial-dataset"; \
      echo "Or run: just download-datasets"; \
      exit 1; \
    fi

viz-install-python:
    echo "Installing Python plotting dependencies..."
    if command -v pip3 >/dev/null 2>&1; then \
      pip3 install pandas matplotlib numpy; \
    elif command -v pip >/dev/null 2>&1; then \
      pip install pandas matplotlib numpy; \
    else \
      echo "pip not found. Please install Python 3."; \
      exit 1; \
    fi

viz-plots:
    echo "Generating plots from demo data..."
    if [ -f "./plot_output/plot_comparisons.py" ]; then \
      cd plot_output && python3 plot_comparisons.py; \
      echo "Plots generated in ./plot_output/"; \
      ls -lh ./plot_output/*.png 2>/dev/null || echo "No PNG files generated (check for errors)"; \
    else \
      echo "Demo data not found. Run 'just viz-demo' first."; \
      exit 1; \
    fi

viz-plots-tum:
    echo "Generating plots from TUM-VI data..."
    if [ -f "./tum_vi_results/plot_comparisons.py" ]; then \
      cd tum_vi_results && python3 plot_comparisons.py; \
      echo "Plots generated in ./tum_vi_results/"; \
      ls -lh ./tum_vi_results/*.png 2>/dev/null || echo "No PNG files generated (check for errors)"; \
    else \
      echo "TUM-VI data not found. Run 'just viz-tum' first."; \
      exit 1; \
    fi

# ---------------------------------------------------------------------------
# Cleanup
# ---------------------------------------------------------------------------
clean:
    echo "Cleaning build artifacts..."
    cargo clean
    rm -rf {{build_dir}} Cargo.lock

clean-all: clean
    echo "Removing Docker containers and images..."
    docker rmi rs-vio:latest 2>/dev/null || true
    docker system prune -f 2>/dev/null || true
    echo "Deep clean complete."

# ---------------------------------------------------------------------------
# Extended Quality Pipeline (from Makefile.quality)
# ---------------------------------------------------------------------------
format: fmt

lint-all: lint clippy

check-all: check

deps-unused:
    echo "Checking for unused dependencies (nightly)..."
    if command -v cargo >/dev/null 2>&1; then \
      cargo +nightly udeps --all-targets --all-features 2>/dev/null || echo "Install nightly: rustup install nightly"; \
    fi

deps-outdated:
    echo "Checking for outdated dependencies..."
    if command -v cargo-outdated >/dev/null 2>&1; then \
      cargo outdated --root-deps-only; \
    else \
      echo "Install: cargo install cargo-outdated"; \
    fi

deps-license:
    echo "Checking license compliance..."
    if command -v cargo-deny >/dev/null 2>&1; then \
      cargo deny check licenses; \
    else \
      echo "Install: cargo install cargo-deny"; \
    fi

deps-dup:
    echo "Checking for duplicate dependencies..."
    cargo tree --duplicates

deps: deps-unused deps-outdated deps-license deps-dup

check-fast:
    echo "Running cargo check (all targets)..."
    cargo check --all-targets --all-features
    cargo check --all-features --manifest-path Cargo.toml

test-nextest:
    echo "Running tests with nextest..."
    if command -v cargo-nextest >/dev/null 2>&1; then \
      cargo nextest run --all-features; \
    else \
      echo "Install: cargo install cargo-nextest"; \
      cargo test --all-features; \
    fi

coverage:
    echo "Generating coverage report..."
    if command -v cargo-tarpaulin >/dev/null 2>&1; then \
      cargo tarpaulin --all-features --out Html --report-name coverage; \
      echo "Coverage report: target/tarpaulin/tarpaulin.html"; \
    else \
      echo "Install: cargo install cargo-tarpaulin"; \
    fi

doc:
    echo "Generating documentation..."
    cargo doc --all-features --no-deps
    echo "Documentation: target/doc/index.html"

doc-links:
    echo "Checking documentation links..."
    if command -v cargo-deadlinks >/dev/null 2>&1; then \
      cargo deadlinks --dir target/doc; \
    else \
      echo "Install: cargo install cargo-deadlinks"; \
    fi

miri:
    echo "Running MIRI (requires nightly)..."
    if command -v cargo >/dev/null 2>&1; then \
      cargo +nightly miri test --all-features 2>&1 | head -100; \
    else \
      echo "Install nightly: rustup install nightly"; \
    fi

unsafe:
    echo "Analyzing unsafe code usage..."
    if command -v cargo-geiger >/dev/null 2>&1; then \
      cargo geiger --all-features --format markdown; \
    else \
      echo "Install: cargo install cargo-geiger"; \
    fi

mutants:
    echo "Running mutation testing..."
    if command -v cargo-mutants >/dev/null 2>&1; then \
      cargo mutants --all-features; \
    else \
      echo "Install: cargo install cargo-mutants"; \
    fi

features:
    echo "Testing all feature combinations..."
    if command -v cargo-hack >/dev/null 2>&1; then \
      cargo hack --each-feature --no-dev-deps check; \
    else \
      echo "Install: cargo install cargo-hack"; \
    fi

bloat:
    echo "Analyzing binary size..."
    if command -v cargo-bloat >/dev/null 2>&1; then \
      cargo bloat --release --features lightglue -- -A | head -50; \
    else \
      echo "Install: cargo install cargo-bloat"; \
    fi

llvm-lines:
    echo "Analyzing LLVM output..."
    if command -v cargo-llvm-lines >/dev/null 2>&1; then \
      cargo llvm-lines --release --all-features | head -30; \
    else \
      echo "Install: cargo install cargo-llvm-lines"; \
    fi

semver:
    echo "Checking semantic versioning..."
    if command -v cargo-semver >/dev/null 2>&1; then \
      cargo semver-checks; \
    else \
      echo "Install: cargo install cargo-semver"; \
    fi

msrv:
    echo "Checking Minimum Supported Rust Version..."
    if command -v cargo-msrv >/dev/null 2>&1; then \
      cargo msrv verify; \
    else \
      echo "Install: cargo install cargo-msrv"; \
    fi

quality: fmt check-fast lint audit deps test doc
    echo "Quality pipeline completed."

install-tools:
    echo "Installing quality tools..."
    cargo install cargo-audit --locked
    cargo install cargo-deny --locked
    cargo install cargo-udeps --locked
    cargo install cargo-outdated --locked
    cargo install cargo-nextest --locked
    cargo install cargo-tarpaulin --locked
    cargo install cargo-geiger --locked
    cargo install cargo-bloat --locked
    cargo install cargo-llvm-lines --locked
    cargo install cargo-mutants --locked
    cargo install cargo-hack --locked
    cargo install cargo-sort --locked
    cargo install cargo-deadlinks --locked
    echo "All tools installed."

quick:
    echo "Running quick quality check..."
    cargo fmt --all --check
    cargo clippy --all-targets --all-features -- -D warnings
    cargo test --all-features --lib
    echo "Quick check complete."

ci-quality: fmt check-fast lint audit
    echo "CI quality check complete."
