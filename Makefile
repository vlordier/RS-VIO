.PHONY: help build release test test-release clippy fmt fmt-check audit clean lint-shell download-datasets generate-test-data run-euroc run-tum run-4seasons docker-build docker-smoke-test all

help:
	@echo "RS-VIO Makefile targets:"
	@echo "  make build              - Debug build"
	@echo "  make release            - Release build (optimized)"
	@echo "  make test               - Run all tests (debug)"
	@echo "  make test-release       - Run all tests (release)"
	@echo "  make clippy             - Lint with clippy"
	@echo "  make fmt                - Auto-format code"
	@echo "  make fmt-check          - Check formatting without changes"
	@echo "  make audit              - Security audit"
	@echo "  make lint-shell         - Lint all shell scripts (requires shellcheck)"
	@echo "  make download-datasets  - Download sample datasets to /tmp/rs-vio-samples"
	@echo "  make generate-test-data - Generate synthetic test data in /tmp/rs-vio-test-data"
	@echo "  make run-euroc          - Build release and run EuRoC binary with test data"
	@echo "  make run-tum            - Build release and run TUM-VI binary with test data"
	@echo "  make run-4seasons       - Build release and run 4Seasons binary with test data"
	@echo "  make docker-build       - Build Docker image (rs-vio:latest)"
	@echo "  make docker-smoke-test  - Smoke test Docker image"
	@echo "  make clean              - Clean build artifacts"
	@echo "  make all                - Build, test, lint, audit (full CI)"
	@echo ""
	@echo "Example: make test-release"

build:
	cargo build

release:
	cargo build --release

test:
	cargo test --all

test-release:
	cargo test --all --release

clippy:
	cargo clippy --all --release -- -D warnings

fmt:
	cargo fmt

fmt-check:
	cargo fmt -- --check

audit:
	cargo audit

lint-shell:
	./scripts/lint_shell.sh

download-datasets:
	@echo "Downloading sample datasets to /tmp/rs-vio-samples..."
	./scripts/download_datasets.sh /tmp/rs-vio-samples all
	@echo "Done. Use -v /tmp/rs-vio-samples/<dataset> with Docker."

generate-test-data:
	@echo "Generating synthetic test data to /tmp/rs-vio-test-data..."
	bash scripts/download_datasets.sh /tmp/rs-vio-test-data all
	@echo "✓ Test data ready for local testing"

run-euroc: release generate-test-data
	@echo "Running EuRoC estimator with synthetic test data..."
	timeout 10 ./target/release/run_euroc config/euroc_vio.yaml /tmp/rs-vio-test-data/euroc/MH_01_easy || true
	@echo "✓ EuRoC run completed"

run-tum: release generate-test-data
	@echo "Running TUM-VI estimator with synthetic test data..."
	timeout 10 ./target/release/run_tum config/tum_vi.yaml /tmp/rs-vio-test-data/tum_vi/room1 || true
	@echo "✓ TUM-VI run completed"

run-4seasons: release generate-test-data
	@echo "Running 4Seasons estimator with synthetic test data..."
	timeout 10 ./target/release/run_4seasons config/4seasons.yaml /tmp/rs-vio-test-data/4seasons/recording_2021-01-07_13-03-56 || true
	@echo "✓ 4Seasons run completed"

docker-build:
	docker build -t rs-vio:latest .

docker-smoke-test: docker-build
	@echo "Testing rs-vio:latest..."
	docker run --rm rs-vio:latest --help
	@echo "✓ Docker smoke test passed"

clean:
	cargo clean
	rm -rf target/

all: fmt-check audit lint-shell test-release clippy
	@echo "✓ All checks passed!"

.DEFAULT_GOAL := help
