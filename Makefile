# RS-VIO Makefile
#
# Developer-friendly commands for building, testing, and measuring performance.
#
# Usage:
#   make              - Show help
#   make bench        - Run all benchmarks
#   make measure      - Run performance measurements
#   make test         - Run all tests
#   make build        - Build release binary

.PHONY: help build test bench measure measure-accuracy measure-overhead clean docs lint fmt

# Default target
help:
	@echo "========================================================================"
	@echo "RS-VIO Development Commands"
	@echo "========================================================================"
	@echo ""
	@echo "Build:"
	@echo "  make build          Build release binary"
	@echo "  make debug          Build debug binary"
	@echo "  make clean          Clean build artifacts"
	@echo ""
	@echo "Test:"
	@echo "  make test           Run all tests"
	@echo "  make test-fast      Run tests without slow/ignored tests"
	@echo "  make test-overhead  Run motion prior overhead test"
	@echo ""
	@echo "Benchmarks:"
	@echo "  make bench          Run all benchmarks"
	@echo "  make bench-motion   Run motion prior overhead benchmarks"
	@echo "  make bench-sliding  Run sliding window benchmarks"
	@echo "  make bench-feature  Run feature tracker benchmarks"
	@echo ""
	@echo "Performance Measurement:"
	@echo "  make measure        Run complete performance measurement"
	@echo "  make measure-accuracy  Run accuracy comparison (requires dataset)"
	@echo "  make measure-overhead  Run motion prior overhead only"
	@echo ""
	@echo "  Example:"
	@echo "    make measure DATASET=/Users/vincent/data/tum-vi/dataset-corridor1_512_16"
	@echo ""
	@echo "Code Quality:"
	@echo "  make fmt            Format code with rustfmt"
	@echo "  make fmt-check      Check code formatting"
	@echo "  make lint           Run clippy linter"
	@echo "  make lint-fix       Auto-fix clippy warnings"
	@echo ""
	@echo "Documentation:"
	@echo "  make docs           Generate documentation"
	@echo "  make docs-open      Generate and open documentation"
	@echo ""
	@echo "Utilities:"
	@echo "  make results        View latest measurement results"
	@echo "  make results-dir    Open results directory"
	@echo "  make commit-measure  Commit measurement infrastructure"
	@echo ""

# ============================================================================
# Build
# ============================================================================

build:
	@echo "Building release binary..."
	cargo build --release

debug:
	@echo "Building debug binary..."
	cargo build

clean:
	@echo "Cleaning build artifacts..."
	cargo clean
	rm -rf performance_results/

# ============================================================================
# Test
# ============================================================================

test:
	@echo "Running all tests..."
	cargo test --lib

test-fast:
	@echo "Running fast tests (excluding slow/ignored)..."
	cargo test --lib -- --skip measure_prior_overhead

test-overhead:
	@echo "Running motion prior overhead measurement test..."
	cargo test measure_prior_overhead -- --ignored --nocapture

# ============================================================================
# Benchmarks
# ============================================================================

bench: bench-motion bench-sliding
	@echo ""
	@echo "========================================================================"
	@echo "All benchmarks complete!"
	@echo "========================================================================"

bench-motion:
	@echo "========================================================================"
	@echo "Running Motion Prior Overhead Benchmarks"
	@echo "========================================================================"
	cargo bench --bench motion_prior

bench-sliding:
	@echo "========================================================================"
	@echo "Running Sliding Window Benchmarks"
	@echo "========================================================================"
	cargo bench --bench sliding_window

bench-feature:
	@echo "========================================================================"
	@echo "Running Feature Tracker Benchmarks"
	@echo "========================================================================"
	cargo bench --bench feature_tracker

# ============================================================================
# Performance Measurement
# ============================================================================

# Default dataset (can be overridden with make measure DATASET=...)
DATASET ?= /Users/vincent/data/tum-vi/dataset-corridor1_512_16
DATASETS ?= /Users/vincent/data/tum-vi/dataset-corridor1_512_16 \
            /Users/vincent/data/tum-vi/dataset-corridor2_512_16 \
            /Users/vincent/data/tum-vi/dataset-room1_512_16

measure: measure-overhead measure-sliding measure-accuracy
	@echo ""
	@echo "========================================================================"
	@echo "Performance Measurement Complete!"
	@echo "========================================================================"
	@echo ""
	@echo "Results saved to: performance_results/"
	@echo "View results: make results"
	@echo ""

measure-overhead:
	@echo "========================================================================"
	@echo "Measuring Motion Prior Overhead"
	@echo "========================================================================"
	@mkdir -p performance_results
	cargo bench --bench motion_prior 2>&1 | tee performance_results/motion_prior_$$(date +%Y%m%d_%H%M%S).log

measure-sliding:
	@echo "========================================================================"
	@echo "Measuring Sliding Window Performance"
	@echo "========================================================================"
	@mkdir -p performance_results
	cargo bench --bench sliding_window 2>&1 | tee performance_results/sliding_window_$$(date +%Y%m%d_%H%M%S).log

measure-accuracy:
	@echo "========================================================================"
	@echo "Measuring Accuracy Improvements"
	@echo "========================================================================"
	@echo "Dataset: $(DATASET)"
	@if [ ! -d "$(DATASET)" ]; then \
		echo "Error: Dataset not found at $(DATASET)"; \
		echo "Set correct path with: make measure DATASET=/path/to/dataset"; \
		exit 1; \
	fi
	@mkdir -p performance_results
	python3 scripts/measure_prior_accuracy.py \
		"$(DATASET)" \
		--config config/tum_vi.yaml \
		--json performance_results/accuracy_comparison_$$(date +%Y%m%d_%H%M%S).json \
		2>&1 | tee performance_results/accuracy_$$(date +%Y%m%d_%H%M%S).log

# ============================================================================
# VO vs VIO Benchmarking
# ============================================================================

benchmark: benchmark-vo-vio
	@echo ""
	@echo "========================================================================"
	@echo "Benchmarking Complete!"
	@echo "========================================================================"

benchmark-vo-vio:
	@echo "========================================================================"
	@echo "Running VO vs VIO Benchmark Suite"
	@echo "========================================================================"
	@mkdir -p benchmark_results
	python3 scripts/benchmark_vo_vio.py \
		--binary target/release/run_tum \
		--config config/tum_vi.yaml \
		--datasets $(DATASETS) \
		--output benchmark_results/vo_vio_benchmark_$$(date +%Y%m%d_%H%M%S).json \
		--timeout 600

benchmark-corridor1:
	@echo "========================================================================"
	@echo "Benchmarking Corridor1 (VO vs VIO)"
	@echo "========================================================================"
	@mkdir -p benchmark_results
	python3 scripts/benchmark_vo_vio.py \
		--binary target/release/run_tum \
		--config config/tum_vi.yaml \
		--datasets /Users/vincent/data/tum-vi/dataset-corridor1_512_16 \
		--output benchmark_results/corridor1_benchmark_$$(date +%Y%m%d_%H%M%S).json

benchmark-room1:
	@echo "========================================================================"
	@echo "Benchmarking Room1 (VO vs VIO)"
	@echo "========================================================================"
	@mkdir -p benchmark_results
	python3 scripts/benchmark_vo_vio.py \
		--binary target/release/run_tum \
		--config config/tum_vi.yaml \
		--datasets /Users/vincent/data/tum-vi/dataset-room1_512_16 \
		--output benchmark_results/room1_benchmark_$$(date +%Y%m%d_%H%M%S).json

# ============================================================================
# Code Quality
# ============================================================================

fmt:
	@echo "Formatting code..."
	cargo fmt

fmt-check:
	@echo "Checking code formatting..."
	cargo fmt --check

lint:
	@echo "Running clippy linter..."
	cargo clippy --lib

lint-fix:
	@echo "Auto-fixing clippy warnings..."
	cargo clippy --lib --fix --allow-dirty --allow-staged

# ============================================================================
# Documentation
# ============================================================================

docs:
	@echo "Generating documentation..."
	cargo doc --no-deps

docs-open:
	@echo "Generating and opening documentation..."
	cargo doc --no-deps --open

# ============================================================================
# Utilities
# ============================================================================

results:
	@echo "Latest measurement results:"
	@echo ""
	@if [ -d performance_results ]; then \
		ls -lt performance_results/ | head -10; \
		echo ""; \
		echo "To view a specific file:"; \
		echo "  cat performance_results/<filename>"; \
	else \
		echo "No results found. Run: make measure"; \
	fi

results-dir:
	@echo "Opening results directory..."
	@if [ -d performance_results ]; then \
		open performance_results 2>/dev/null || xdg-open performance_results 2>/dev/null || echo "Results dir: $$(pwd)/performance_results"; \
	else \
		echo "No results found. Run: make measure"; \
	fi

commit-measure:
	@echo "Committing measurement infrastructure..."
	./scripts/commit_measurements.sh

# ============================================================================
# Quick Development Cycle
# ============================================================================

# Quick test + lint for development
dev: fmt test lint
	@echo ""
	@echo "✓ Development checks complete"

# Full CI-style check
ci: fmt-check test lint
	@echo ""
	@echo "✓ CI checks passed"
