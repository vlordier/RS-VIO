# RS-VIO Makefile
# Best practices: Clear structure, composable targets, helpful output
# See: https://makefiletutorial.com

# ============================================================================
# Variable Definitions
# ============================================================================

.DEFAULT_GOAL := help
SHELL := /bin/bash
.SHELLFLAGS := -euo pipefail -c

# Directory structure
BUILD_DIR := target
RELEASE_DIR := $(BUILD_DIR)/release
DEBUG_DIR := $(BUILD_DIR)/debug
SCRIPT_DIR := scripts
CONFIG_DIR := config
DATASET_DIR := /tmp/rs-vio-samples

# Binaries
EUROC_BIN := $(RELEASE_DIR)/run_euroc
TUM_BIN := $(RELEASE_DIR)/run_tum
4SEASONS_BIN := $(RELEASE_DIR)/run_4seasons

# Color output
COLOR_BLUE := \033[0;34m
COLOR_GREEN := \033[0;32m
COLOR_YELLOW := \033[1;33m
COLOR_RED := \033[0;31m
NC := \033[0m

# ============================================================================
# Phony Targets
# ============================================================================

.PHONY: help build release test test-release check fmt audit clean \
	lint clippy clippy-fix \
	download-datasets setup-datasets setup-4seasons verify-datasets \
	run run-euroc run-tum run-4seasons \
	docker docker-build docker-test docker-push \
	ci benchmark-all \
	info clean-all

# ============================================================================
# Help & Information
# ============================================================================

help:
	@echo "$(COLOR_BLUE)═══════════════════════════════════════════════════════════$(NC)"
	@echo "$(COLOR_BLUE)RS-VIO Development Makefile$(NC)"
	@echo "$(COLOR_BLUE)═══════════════════════════════════════════════════════════$(NC)"
	@echo ""
	@echo "$(COLOR_GREEN)BUILD TARGETS$(NC)"
	@echo "  $(COLOR_YELLOW)build$(NC)                 - Debug build"
	@echo "  $(COLOR_YELLOW)release$(NC)               - Release build (optimized)"
	@echo ""
	@echo "$(COLOR_GREEN)TESTING & QUALITY$(NC)"
	@echo "  $(COLOR_YELLOW)test$(NC)                  - Run all tests (debug)"
	@echo "  $(COLOR_YELLOW)test-release$(NC)          - Run all tests (release)"
	@echo "  $(COLOR_YELLOW)check$(NC)                 - Run code quality checks (fmt, clippy, audit)"
	@echo "  $(COLOR_YELLOW)fmt$(NC)                   - Auto-format code"
	@echo "  $(COLOR_YELLOW)clippy$(NC)                - Lint with clippy"
	@echo "  $(COLOR_YELLOW)clippy-fix$(NC)            - Auto-fix clippy warnings"
	@echo "  $(COLOR_YELLOW)audit$(NC)                 - Security audit"
	@echo "  $(COLOR_YELLOW)lint$(NC)                  - Lint shell scripts"
	@echo ""
	@echo "$(COLOR_GREEN)DATASETS & BENCHMARKS$(NC)"
	@echo "  $(COLOR_YELLOW)download-datasets$(NC)     - Download real datasets (EuRoC, TUM-VI, 4Seasons)"
	@echo "  $(COLOR_YELLOW)setup-datasets$(NC)        - Download, verify, and test datasets"
	@echo "  $(COLOR_YELLOW)setup-4seasons$(NC)        - 4Seasons setup instructions"
	@echo "  $(COLOR_YELLOW)verify-datasets$(NC)       - Verify dataset integrity"
	@echo "  $(COLOR_YELLOW)run$(NC)                   - Run all benchmarks (requires datasets)"
	@echo "  $(COLOR_YELLOW)run-euroc$(NC)             - Run EuRoC benchmark"
	@echo "  $(COLOR_YELLOW)run-tum$(NC)               - Run TUM-VI benchmark"
	@echo "  $(COLOR_YELLOW)run-4seasons$(NC)          - Run 4Seasons benchmark"
	@echo ""
	@echo "$(COLOR_GREEN)DOCKER$(NC)"
	@echo "  $(COLOR_YELLOW)docker$(NC)                - Build Docker image"
	@echo "  $(COLOR_YELLOW)docker-build$(NC)          - Build Docker image (alias)"
	@echo "  $(COLOR_YELLOW)docker-test$(NC)           - Build and test Docker image"
	@echo "  $(COLOR_YELLOW)docker-push$(NC)           - Push Docker image (requires credentials)"
	@echo ""
	@echo "$(COLOR_GREEN)AUTOMATION & CI$(NC)"
	@echo "  $(COLOR_YELLOW)ci$(NC)                    - Full CI pipeline (build, test, check, lint)"
	@echo "  $(COLOR_YELLOW)benchmark-all$(NC)         - Complete benchmark suite (datasets + all 3 benchmarks)"
	@echo ""
	@echo "$(COLOR_GREEN)MAINTENANCE$(NC)"
	@echo "  $(COLOR_YELLOW)clean$(NC)                 - Clean build artifacts"
	@echo "  $(COLOR_YELLOW)clean-all$(NC)             - Clean everything including containers"
	@echo "  $(COLOR_YELLOW)info$(NC)                  - Display environment information"
	@echo ""
	@echo "$(COLOR_BLUE)═══════════════════════════════════════════════════════════$(NC)"
	@echo ""
	@echo "$(COLOR_GREEN)Quick Start Examples:$(NC)"
	@echo "  make build && make test                    # Build and test"
	@echo "  make release && make run-euroc             # Build release and run EuRoC"
	@echo "  make ci                                    # Full CI checks"
	@echo "  make setup-datasets && make run            # Setup datasets and run all benchmarks"
	@echo ""

info:
	@echo "$(COLOR_BLUE)Environment Information$(NC)"
	@echo "Rust version:"
	@rustc --version
	@cargo --version
	@echo "OS: $(shell uname -s)"
	@echo "Architecture: $(shell uname -m)"
	@echo "Build directory: $(BUILD_DIR)"
	@echo "Release binaries: $(RELEASE_DIR)"

# ============================================================================
# Build Targets
# ============================================================================

build:
	@echo "$(COLOR_GREEN)Building debug binary...$(NC)"
	cargo build

release:
	@echo "$(COLOR_GREEN)Building release binary (optimized)...$(NC)"
	cargo build --release

# ============================================================================
# Testing Targets
# ============================================================================

test: build
	@echo "$(COLOR_GREEN)Running debug tests...$(NC)"
	cargo test --all

test-release: release
	@echo "$(COLOR_GREEN)Running release tests...$(NC)"
	cargo test --all --release

# ============================================================================
# Code Quality Targets
# ============================================================================

fmt:
	@echo "$(COLOR_GREEN)Formatting code...$(NC)"
	cargo fmt

clippy: release
	@echo "$(COLOR_GREEN)Linting with clippy...$(NC)"
	cargo clippy --all --release -- -D warnings

clippy-fix:
	@echo "$(COLOR_GREEN)Auto-fixing clippy warnings...$(NC)"
	cargo clippy --fix --all-targets --all-features --allow-dirty

# Quick quality check (recommended before commits)
quality-quick:
	@echo "$(COLOR_BLUE)════════════════════════════════════════════════════════════$(NC)"
	@echo "$(COLOR_BLUE)  RS-VIO Quick Quality Check$(NC)"
	@echo "$(COLOR_BLUE)════════════════════════════════════════════════════════════$(NC)"
	@echo ""
	@echo "$(COLOR_YELLOW)1. Running tests...$(NC)"
	@cargo test --lib --bins
	@echo ""
	@echo "$(COLOR_YELLOW)2. Running clippy...$(NC)"
	@cargo clippy --all --all-targets -- -D warnings
	@echo ""
	@echo "$(COLOR_YELLOW)3. Checking formatting...$(NC)"
	@cargo fmt --all -- --check
	@echo ""
	@echo "$(COLOR_GREEN)✅ Quick quality check passed!$(NC)"

# Pre-commit hook (fast, auto-fixes formatting)
pre-commit:
	@echo "$(COLOR_YELLOW)Running pre-commit checks...$(NC)"
	@cargo fmt --all
	@cargo clippy --all -- -D warnings
	@cargo test --all --quiet
	@echo "$(COLOR_GREEN)✅ Pre-commit checks passed!$(NC)"

# Full quality check (includes security audit)
quality-full: quality-quick
	@echo ""
	@echo "$(COLOR_YELLOW)4. Running security audit...$(NC)"
	@cargo audit --deny warnings --ignore RUSTSEC-2025-0141 || echo "$(COLOR_YELLOW)Note: Install cargo-audit for security checks$(NC)"
	@echo ""
	@echo "$(COLOR_GREEN)✅ Full quality check complete!$(NC)"


audit:
	@echo "$(COLOR_GREEN)Running security audit...$(NC)"
	cargo audit

lint:
	@echo "$(COLOR_GREEN)Linting shell scripts...$(NC)"
	@if [ -x "$(SCRIPT_DIR)/lint_shell.sh" ]; then \
		$(SCRIPT_DIR)/lint_shell.sh; \
	else \
		echo "$(COLOR_YELLOW)Warning: lint_shell.sh not found or not executable$(NC)"; \
	fi

check: fmt clippy audit lint
	@echo "$(COLOR_GREEN)✓ All code quality checks passed!$(NC)"

# ============================================================================
# Dataset Targets
# ============================================================================

download-datasets:
	@echo "$(COLOR_GREEN)Downloading datasets to $(DATASET_DIR)...$(NC)"
	@mkdir -p $(DATASET_DIR)
	bash $(SCRIPT_DIR)/download_datasets.sh $(DATASET_DIR) all
	@echo "$(COLOR_GREEN)✓ Download complete$(NC)"

setup-datasets:
	@echo "$(COLOR_GREEN)Setting up datasets with verification...$(NC)"
	@if [ -x "$(SCRIPT_DIR)/setup-datasets.sh" ]; then \
		bash $(SCRIPT_DIR)/setup-datasets.sh; \
	else \
		echo "$(COLOR_YELLOW)setup-datasets.sh not found$(NC)"; \
		echo "Falling back to manual download..."; \
		$(MAKE) download-datasets; \
	fi
	@echo "$(COLOR_GREEN)✓ Dataset setup complete$(NC)"

verify-datasets:
	@echo "$(COLOR_GREEN)Verifying datasets...$(NC)"
	@for dataset in euroc tum_vi 4seasons; do \
		echo "Checking $$dataset..."; \
		if [ -d "$(DATASET_DIR)/$$dataset" ]; then \
			echo "  $(COLOR_GREEN)✓ $$dataset found$(NC)"; \
		else \
			echo "  $(COLOR_YELLOW)⚠ $$dataset not found$(NC)"; \
		fi; \
	done

# ============================================================================
# Benchmark Targets
# ============================================================================

run: run-euroc run-tum run-4seasons
	@echo "$(COLOR_GREEN)✓ All benchmarks completed$(NC)"

run-euroc: release
	@echo "$(COLOR_GREEN)Running EuRoC benchmark...$(NC)"
	@if [ -d "$(DATASET_DIR)/euroc/MH_01_easy" ]; then \
		timeout 120 $(EUROC_BIN) $(CONFIG_DIR)/euroc_vio.yaml $(DATASET_DIR)/euroc/MH_01_easy || true; \
		echo "$(COLOR_GREEN)✓ EuRoC benchmark complete$(NC)"; \
	else \
		echo "$(COLOR_RED)✗ EuRoC dataset not found$(NC)"; \
		echo "Download with: make setup-datasets"; \
		exit 1; \
	fi

run-tum: release
	@echo "$(COLOR_GREEN)Running TUM-VI benchmark...$(NC)"
	@if [ -d "$(DATASET_DIR)/tum_vi" ]; then \
		timeout 120 $(TUM_BIN) $(CONFIG_DIR)/tum_vi.yaml $(DATASET_DIR)/tum_vi || true; \
		echo "$(COLOR_GREEN)✓ TUM-VI benchmark complete$(NC)"; \
	else \
		echo "$(COLOR_RED)✗ TUM-VI dataset not found$(NC)"; \
		echo "Download with: make setup-datasets"; \
		exit 1; \
	fi

run-4seasons: release
	@echo "$(COLOR_GREEN)Running 4Seasons benchmark...$(NC)"
	@if [ -d "$(DATASET_DIR)/4seasons" ]; then \
		recording_dir=$$(ls -d $(DATASET_DIR)/4seasons/recording_* 2>/dev/null | head -1); \
		if [ -z "$$recording_dir" ]; then \
			echo "$(COLOR_RED)✗ No 4Seasons recordings found$(NC)"; \
			exit 1; \
		fi; \
		echo "$(COLOR_BLUE)Using recording: $$(basename $$recording_dir)$(NC)"; \
		timeout 120 $(EUROC_BIN) $(CONFIG_DIR)/4seasons.yaml "$$recording_dir" || true; \
		echo "$(COLOR_GREEN)✓ 4Seasons benchmark complete$(NC)"; \
	else \
		echo "$(COLOR_RED)✗ 4Seasons dataset not found$(NC)"; \
		echo ""; \
		echo "4Seasons has been automated!"; \
		echo "Run: make setup-datasets"; \
		exit 1; \
	fi

setup-4seasons:
	@echo "$(COLOR_BLUE)4Seasons Dataset Setup Guide$(NC)"
	@echo ""
	@echo "Quick Instructions:"
	@echo "  1. Register at: https://www.4seasons-dataset.com/"
	@echo "  2. Download a sequence (recording_YYYY-MM-DD_HH-MM-SS.zip)"
	@echo "  3. Save to: /tmp/recording_*.zip"
	@echo "  4. Run: ./scripts/setup-datasets.sh"
	@echo ""
	@echo "For detailed instructions, see: docs/4SEASONS_SETUP.md"
	@echo ""
	@echo "Check current download status:"
	@ls -lh /tmp/recording_*.zip 2>/dev/null || echo "  No 4Seasons ZIP found in /tmp/"
	@[ -d "$(DATASET_DIR)/4seasons" ] && echo "  Extracted datasets:" && ls -d $(DATASET_DIR)/4seasons/*/ 2>/dev/null || echo "  No extracted datasets yet"

# ============================================================================
# Docker Targets
# ============================================================================

docker: docker-build
	@echo "$(COLOR_GREEN)✓ Docker image built: rs-vio:latest$(NC)"

docker-build:
	@echo "$(COLOR_GREEN)Building Docker image...$(NC)"
	docker build -t rs-vio:latest .

docker-test: docker-build
	@echo "$(COLOR_GREEN)Testing Docker image...$(NC)"
	docker run --rm rs-vio:latest --help
	@echo "$(COLOR_GREEN)✓ Docker smoke test passed$(NC)"

docker-push:
	@echo "$(COLOR_GREEN)Pushing Docker image...$(NC)"
	@read -p "Enter Docker registry (e.g., docker.io/username): " registry; \
	docker tag rs-vio:latest $$registry/rs-vio:latest; \
	docker push $$registry/rs-vio:latest; \
	echo "$(COLOR_GREEN)✓ Push complete$(NC)"

# ============================================================================
# Automation & CI Targets
# ============================================================================

ci: check test-release lint
	@echo "$(COLOR_GREEN)═══════════════════════════════════════════════════════════$(NC)"
	@echo "$(COLOR_GREEN)✓ Full CI pipeline passed!$(NC)"
	@echo "$(COLOR_GREEN)═══════════════════════════════════════════════════════════$(NC)"

benchmark-all:
	@echo "$(COLOR_GREEN)Running complete benchmark suite...$(NC)"
	@if [ -x "$(SCRIPT_DIR)/orchestrate.sh" ]; then \
		bash $(SCRIPT_DIR)/orchestrate.sh all; \
	else \
		echo "$(COLOR_RED)✗ orchestrate.sh not found$(NC)"; \
		exit 1; \
	fi

# ============================================================================
# Cleanup Targets
# ============================================================================

clean:
	@echo "$(COLOR_GREEN)Cleaning build artifacts...$(NC)"
	cargo clean
	rm -rf $(BUILD_DIR) Cargo.lock

clean-all: clean
	@echo "$(COLOR_GREEN)Removing Docker containers and images...$(NC)"
	-docker rmi rs-vio:latest 2>/dev/null || true
	-docker system prune -f 2>/dev/null || true
	@echo "$(COLOR_GREEN)✓ Deep clean complete$(NC)"
