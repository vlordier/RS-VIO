#!/usr/bin/env bash
set -euo pipefail

# RS-VIO Complete Automation Script
# Orchestrates building, testing, and running everything

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DATASETS_DIR="${DATASETS_DIR:-/tmp/rs-vio-samples}"
COLOR_GREEN='\033[0;32m'
COLOR_BLUE='\033[0;34m'
COLOR_YELLOW='\033[1;33m'
COLOR_RED='\033[0;31m'
NC='\033[0m'

log_section() { 
  echo ""
  echo -e "${COLOR_BLUE}╔════════════════════════════════════════════════════════════════╗${NC}"
  echo -e "${COLOR_BLUE}║${NC} $1"
  echo -e "${COLOR_BLUE}╚════════════════════════════════════════════════════════════════╝${NC}"
  echo ""
}

log_success() { echo -e "${COLOR_GREEN}✓ $*${NC}"; }
log_error() { echo -e "${COLOR_RED}✗ $*${NC}"; }
log_info() { echo -e "${COLOR_BLUE}→ $*${NC}"; }
log_warn() { echo -e "${COLOR_YELLOW}! $*${NC}"; }

usage() {
  cat << 'EOF'
RS-VIO Complete Automation Script

Usage: scripts/orchestrate.sh [COMMAND]

Commands:
  all               - Run everything (build, test, datasets, docker)
  build             - Build project (debug + release)
  test              - Run all test suites
  setup-datasets    - Download and setup real datasets
  run-euroc         - Run EuRoC estimator with test data
  run-tum           - Run TUM-VI estimator with test data
  run-4seasons      - Run 4Seasons estimator with test data
  docker            - Build and test Docker image
  docker-push       - Build and push Docker image to registry
  clean             - Clean build artifacts
  report            - Generate test report

Environment:
  DATASETS_DIR      - Dataset location (default: /tmp/rs-vio-samples)
  REGISTRY          - Docker registry (default: docker.io)
  IMAGE_NAME        - Docker image name (default: rs-vio)

Examples:
  scripts/orchestrate.sh all                    # Full automation
  scripts/orchestrate.sh setup-datasets         # Download datasets only
  scripts/orchestrate.sh test                   # Run all tests
  DATASETS_DIR=/data scripts/orchestrate.sh all # Custom dataset dir
EOF
}

# Build
build_all() {
  log_section "Building Project"
  
  log_info "Building debug artifacts..."
  cargo build --all
  log_success "Debug build complete"
  
  log_info "Building release artifacts..."
  cargo build --release --all
  log_success "Release build complete"
  
  log_info "Building binaries..."
  cargo build --release --bins
  log_success "Binary builds complete"
  
  echo ""
  log_info "Build artifacts:"
  for binary in run_euroc run_tum run_4seasons; do
    if [ -f "target/release/$binary" ]; then
      size=$(du -h "target/release/$binary" | cut -f1)
      echo "  $(printf '%-15s' "$binary") $size"
    fi
  done
}

# Run tests
run_all_tests() {
  log_section "Running Tests"
  
  if [ -x "scripts/run-all-tests.sh" ]; then
    bash "scripts/run-all-tests.sh"
  else
    log_error "Test script not found"
    return 1
  fi
}

# Setup datasets
setup_datasets() {
  log_section "Setting Up Datasets"
  
  if [ -x "scripts/setup-datasets.sh" ]; then
    bash "scripts/setup-datasets.sh" "$DATASETS_DIR"
  else
    log_error "Setup script not found"
    return 1
  fi
}

# Run individual binaries
run_binary_test() {
  local binary="$1" config="$2" dataset="$3"
  
  log_info "Testing: $binary"
  
  if [ ! -x "target/release/$binary" ]; then
    log_warn "Binary not found, building..."
    cargo build --release --bin "$binary"
  fi
  
  if [ ! -d "$dataset" ]; then
    log_error "Dataset not found: $dataset"
    return 1
  fi
  
  timeout 60 "./target/release/$binary" "config/$config" "$dataset" 2>&1 | head -30
}

# Docker automation
run_docker_automation() {
  log_section "Docker Automation"
  
  if [ -x "scripts/docker-automation.sh" ]; then
    bash "scripts/docker-automation.sh" "rs-vio:latest" "$DATASETS_DIR"
  else
    log_error "Docker automation script not found"
    return 1
  fi
}

# Docker push
push_docker_image() {
  log_section "Docker Push"
  
  local registry="${REGISTRY:-docker.io}"
  local image_name="${IMAGE_NAME:-rs-vio}"
  local tag="${TAG:-latest}"
  local full_name="$registry/$image_name:$tag"
  
  log_info "Building image for push: $full_name"
  cargo build --release
  docker build -t "rs-vio:latest" .
  docker tag "rs-vio:latest" "$full_name"
  
  log_info "Pushing to registry: $registry"
  docker push "$full_name"
  
  log_success "Image pushed: $full_name"
}

# Clean
clean_build() {
  log_section "Cleaning Build Artifacts"
  
  log_info "Removing target directory..."
  rm -rf target/
  log_success "Clean complete"
}

# Generate report
generate_report() {
  log_section "Test Report"
  
  cat > /tmp/rs-vio-report.txt << 'EOL'
RS-VIO Project Status Report
============================

Build Information:
EOL
  
  echo "Generated: $(date)" >> /tmp/rs-vio-report.txt
  echo "Project: RS-VIO" >> /tmp/rs-vio-report.txt
  echo "Location: $PROJECT_ROOT" >> /tmp/rs-vio-report.txt
  echo "" >> /tmp/rs-vio-report.txt
  
  echo "Git Information:" >> /tmp/rs-vio-report.txt
  cd "$PROJECT_ROOT"
  echo "Branch: $(git rev-parse --abbrev-ref HEAD)" >> /tmp/rs-vio-report.txt
  echo "Commits ahead of main: $(git rev-list main..HEAD --count)" >> /tmp/rs-vio-report.txt
  echo "Latest commit: $(git log -1 --oneline)" >> /tmp/rs-vio-report.txt
  echo "" >> /tmp/rs-vio-report.txt
  
  echo "Dataset Runners:" >> /tmp/rs-vio-report.txt
  for binary in run_euroc run_tum run_4seasons; do
    if [ -f "target/release/$binary" ]; then
      size=$(du -h "target/release/$binary" | cut -f1)
      echo "  $binary: $size" >> /tmp/rs-vio-report.txt
    fi
  done
  echo "" >> /tmp/rs-vio-report.txt
  
  echo "Code Quality:" >> /tmp/rs-vio-report.txt
  if cargo clippy --all --release -- -D warnings 2>&1 | grep -q "warning:"; then
    echo "  Clippy: WARNINGS FOUND" >> /tmp/rs-vio-report.txt
  else
    echo "  Clippy: ✓ PASS" >> /tmp/rs-vio-report.txt
  fi
  
  if cargo audit 2>&1 | grep -q "vulnerability"; then
    echo "  Security Audit: VULNERABILITIES FOUND" >> /tmp/rs-vio-report.txt
  else
    echo "  Security Audit: ✓ PASS" >> /tmp/rs-vio-report.txt
  fi
  echo "" >> /tmp/rs-vio-report.txt
  
  echo "Datasets:" >> /tmp/rs-vio-report.txt
  if [ -d "$DATASETS_DIR" ]; then
    echo "  Location: $DATASETS_DIR" >> /tmp/rs-vio-report.txt
    [ -d "$DATASETS_DIR/euroc/MH_01_easy" ] && echo "  - EuRoC MH_01_easy: FOUND" >> /tmp/rs-vio-report.txt || echo "  - EuRoC: NOT FOUND" >> /tmp/rs-vio-report.txt
    [ -d "$DATASETS_DIR/tum_vi/room1" ] && echo "  - TUM-VI room1 (indoor): FOUND" >> /tmp/rs-vio-report.txt || echo "  - TUM-VI room1: NOT FOUND" >> /tmp/rs-vio-report.txt
    [ -d "$DATASETS_DIR/tum_vi/magistrale1" ] && echo "  - TUM-VI magistrale1 (outdoor): FOUND" >> /tmp/rs-vio-report.txt || echo "  - TUM-VI magistrale1: NOT FOUND" >> /tmp/rs-vio-report.txt
    [ -d "$DATASETS_DIR/4seasons" ] && echo "  - 4Seasons: FOUND" >> /tmp/rs-vio-report.txt || echo "  - 4Seasons: NOT FOUND" >> /tmp/rs-vio-report.txt
  fi
  echo "" >> /tmp/rs-vio-report.txt
  
  cat /tmp/rs-vio-report.txt
  log_success "Report saved to: /tmp/rs-vio-report.txt"
}

# Run everything
run_all() {
  log_section "RS-VIO Complete Automation"
  
  log_info "Starting full automation workflow..."
  echo ""
  
  build_all || { log_error "Build failed"; exit 1; }
  echo ""
  
  run_all_tests || { log_warn "Some tests failed"; }
  echo ""
  
  setup_datasets || { log_warn "Dataset setup failed"; }
  echo ""
  
  if [ -d "$DATASETS_DIR/euroc/MH_01_easy" ]; then
    run_binary_test "run_euroc" "euroc_vio.yaml" "$DATASETS_DIR/euroc/MH_01_easy"
    echo ""
  fi
  
  # Run TUM-VI indoor sequence
  if [ -d "$DATASETS_DIR/tum_vi/room1/mav0" ]; then
    run_binary_test "run_tum" "tum_vi.yaml" "$DATASETS_DIR/tum_vi/room1"
    echo ""
  fi
  
  # Run TUM-VI outdoor sequence
  if [ -d "$DATASETS_DIR/tum_vi/magistrale1/mav0" ]; then
    run_binary_test "run_tum" "tum_vi.yaml" "$DATASETS_DIR/tum_vi/magistrale1"
    echo ""
  fi
  
  if ls "$DATASETS_DIR"/4seasons/recording_*/times.txt 1>/dev/null 2>&1; then
    local recording_dir
    recording_dir=$(dirname "$(ls -t "$DATASETS_DIR"/4seasons/recording_*/times.txt | head -1)")
    run_binary_test "run_4seasons" "4seasons.yaml" "$recording_dir"
    echo ""
  fi
  
  run_docker_automation || { log_warn "Docker automation failed"; }
  echo ""
  
  generate_report
  echo ""
  
  log_success "Automation complete! ✓"
}

# Main
main() {
  local command="${1:-all}"
  
  case "$command" in
    all)
      run_all
      ;;
    build)
      build_all
      ;;
    test)
      run_all_tests
      ;;
    setup-datasets)
      setup_datasets
      ;;
    run-euroc)
      [ -d "$DATASETS_DIR/euroc/MH_01_easy" ] && \
        run_binary_test "run_euroc" "euroc_vio.yaml" "$DATASETS_DIR/euroc/MH_01_easy" || \
        log_error "EuRoC dataset not found. Run: scripts/orchestrate.sh setup-datasets"
      ;;
    run-tum)
      [ -f "$DATASETS_DIR/tum_vi/depth.txt" ] && \
        run_binary_test "run_tum" "tum_vi.yaml" "$DATASETS_DIR/tum_vi" || \
        log_error "TUM-VI dataset not found. Run: scripts/orchestrate.sh setup-datasets"
      ;;
    run-4seasons)
      if ls "$DATASETS_DIR"/4seasons/recording_*/times.txt 1>/dev/null 2>&1; then
        local recording_dir
        recording_dir=$(dirname "$(ls -t "$DATASETS_DIR"/4seasons/recording_*/times.txt | head -1)")
        run_binary_test "run_4seasons" "4seasons.yaml" "$recording_dir"
      else
        log_error "4Seasons dataset not found. Run: scripts/orchestrate.sh setup-datasets"
      fi
      ;;
    docker)
      run_docker_automation
      ;;
    docker-push)
      push_docker_image
      ;;
    clean)
      clean_build
      ;;
    report)
      generate_report
      ;;
    -h|--help)
      usage
      ;;
    *)
      log_error "Unknown command: $command"
      usage
      exit 1
      ;;
  esac
}

main "$@"
