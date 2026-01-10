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
	@echo "  make download-datasets  - Download real datasets to /tmp/rs-vio-samples"
	@echo "  make run-euroc          - Build release and run EuRoC with real data"
	@echo "  make run-tum            - Build release and run TUM-VI with real data"
	@echo "  make run-4seasons       - Build release and run 4Seasons with real data"
	@echo "  make docker-build       - Build Docker image (rs-vio:latest)"
	@echo "  make docker-smoke-test  - Smoke test Docker image"
	@echo "  make clean              - Clean build artifacts"
	@echo "  make all                - Build, test, lint, audit (full CI)"
	@echo ""
	@echo "Example: make download-datasets && make run-euroc"

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

run-euroc: release
	@if [ -d "/tmp/rs-vio-samples/euroc/MH_01_easy" ]; then \
		echo "Running EuRoC estimator with real dataset..."; \
		timeout 60 ./target/release/run_euroc config/euroc_vio.yaml /tmp/rs-vio-samples/euroc/MH_01_easy || true; \
	else \
		echo "EuRoC dataset not found. Download first with: make download-datasets"; \
		exit 1; \
	fi
	@echo "✓ EuRoC run completed"

run-tum: release
	@if [ -d "/tmp/rs-vio-samples/tum_vi" ]; then \
		echo "Running TUM-VI estimator with real dataset..."; \
		timeout 60 ./target/release/run_tum config/tum_vi.yaml /tmp/rs-vio-samples/tum_vi || true; \
	else \
		echo "TUM-VI dataset not found. Download first with: make download-datasets"; \
		exit 1; \
	fi
	@echo "✓ TUM-VI run completed"

run-4seasons: release
	@if [ -d "/tmp/rs-vio-samples/4seasons" ]; then \
		echo "Running 4Seasons estimator with real dataset..."; \
		timeout 60 ./target/release/run_4seasons config/4seasons.yaml /tmp/rs-vio-samples/4seasons/* || true; \
	else \
		echo "4Seasons dataset not found. Download first with: make download-datasets"; \
		exit 1; \
	fi
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
