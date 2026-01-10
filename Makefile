.PHONY: help build release test test-release clippy fmt fmt-check audit clean lint-shell download-datasets docker-build docker-smoke-test all

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
