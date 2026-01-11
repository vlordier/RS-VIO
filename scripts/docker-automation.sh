#!/usr/bin/env bash
set -euo pipefail

# RS-VIO Docker Automation Script
# Builds, tests, and runs Docker image with datasets

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE_NAME="${1:-rs-vio:latest}"
DATASETS_DIR="${2:-/tmp/rs-vio-samples}"
COLOR_GREEN='\033[0;32m'
COLOR_BLUE='\033[0;34m'
COLOR_RED='\033[0;31m'
NC='\033[0m'

log_info() { echo -e "${COLOR_BLUE}[INFO]${NC} $*"; }
log_success() { echo -e "${COLOR_GREEN}[✓]${NC} $*"; }
log_error() { echo -e "${COLOR_RED}[✗]${NC} $*"; }
log_section() { echo -e "\n${COLOR_BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"; echo -e "${COLOR_BLUE}$1${NC}"; echo -e "${COLOR_BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}\n"; }

# Check prerequisites
check_docker() {
  if ! command -v docker &>/dev/null; then
    log_error "Docker is not installed"
    exit 1
  fi
  log_success "Docker is installed"
}

# Build Docker image
build_image() {
  log_section "Building Docker Image: $IMAGE_NAME"
  
  cd "$PROJECT_ROOT"
  
  if docker build -t "$IMAGE_NAME" .; then
    log_success "Docker image built successfully"
    local size
    size=$(docker images "$IMAGE_NAME" --format "{{.Size}}")
    log_info "Image size: $size"
    return 0
  else
    log_error "Failed to build Docker image"
    return 1
  fi
}

# Run smoke test
run_smoke_test() {
  log_section "Running Docker Smoke Test"
  
  log_info "Testing image with --help"
  if docker run --rm "$IMAGE_NAME" --help &>/dev/null; then
    log_success "Smoke test passed"
    return 0
  else
    log_error "Smoke test failed"
    return 1
  fi
}

# Run with dataset
run_with_dataset() {
  local binary="$1"
  local config="$2"
  local dataset_path="$3"
  local timeout="${4:-60}"
  
  log_info "Running $binary with $(basename "$dataset_path")"
  
  if [ ! -d "$dataset_path" ]; then
    log_error "Dataset not found: $dataset_path"
    return 1
  fi
  
  local mount_point="/data"
  
  if docker run --rm \
    --volume "$dataset_path:$mount_point:ro" \
    --entrypoint "/usr/local/bin/$binary" \
    --timeout "$timeout" \
    "$IMAGE_NAME" \
    "config/$config" "$mount_point" 2>&1 | head -30; then
    
    log_success "$binary completed"
    return 0
  else
    log_error "$binary failed or timed out"
    return 1
  fi
}

# Test all datasets
run_datasets() {
  log_section "Testing Datasets with Docker"
  
  local success_count=0
  local fail_count=0
  
  if [ -d "$DATASETS_DIR/euroc/MH_01_easy" ]; then
    if run_with_dataset "run_euroc" "euroc_vio.yaml" "$DATASETS_DIR/euroc/MH_01_easy" 60; then
      ((success_count++))
    else
      ((fail_count++))
    fi
    echo ""
  else
    log_info "Skipping EuRoC (not found at $DATASETS_DIR/euroc)"
  fi
  
  if [ -f "$DATASETS_DIR/tum_vi/depth.txt" ]; then
    if run_with_dataset "run_tum" "tum_vi.yaml" "$DATASETS_DIR/tum_vi" 60; then
      ((success_count++))
    else
      ((fail_count++))
    fi
    echo ""
  else
    log_info "Skipping TUM-VI (not found at $DATASETS_DIR/tum_vi)"
  fi
  
  if find "$DATASETS_DIR"/4seasons -name 'times.txt' -type f | grep -q .; then
    local recording_dir
    recording_dir=$(dirname "$(find "$DATASETS_DIR"/4seasons -name 'times.txt' -type f -printf '%T@ %p\n' | sort -rn | head -1 | cut -d' ' -f2-)")
    if run_with_dataset "run_4seasons" "4seasons.yaml" "$recording_dir" 60; then
      ((success_count++))
    else
      ((fail_count++))
    fi
    echo ""
  else
    log_info "Skipping 4Seasons (not found at $DATASETS_DIR/4seasons)"
  fi
  
  log_info "Dataset tests: $success_count passed, $fail_count failed"
  if [ "$fail_count" -eq 0 ]; then
    return 0
  else
    return 1
  fi
}

# Show image info
show_image_info() {
  log_section "Docker Image Information"
  
  echo "Image: $IMAGE_NAME"
  docker inspect "$IMAGE_NAME" --format='
Name: {{.RepoTags}}
Created: {{.Created}}
Size: {{.Size}} bytes
Architecture: {{.Architecture}}
OS: {{.Os}}
Entrypoint: {{.Config.Entrypoint}}
Volumes: {{range $key, $val := .Config.Volumes}}{{$key}} {{end}}
' 2>/dev/null || log_info "Could not retrieve full image info"
  
  echo ""
  log_info "Available binaries in image:"
  docker run --rm --entrypoint sh "$IMAGE_NAME" -c "ls -lh /usr/local/bin/run_*" 2>/dev/null || echo "  (Could not list binaries)"
}

# Cleanup
cleanup() {
  log_section "Cleanup Options"
  
  echo "To remove the Docker image:"
  echo "  docker rmi $IMAGE_NAME"
  echo ""
  echo "To remove all RS-VIO images:"
  echo "  docker rmi -f \$(docker images 'rs-vio*' -q)"
}

# Main
main() {
  log_section "RS-VIO Docker Automation Script"
  
  check_docker
  echo ""
  
  build_image || exit 1
  echo ""
  
  run_smoke_test || exit 1
  echo ""
  
  show_image_info
  echo ""
  
  if [ -d "$DATASETS_DIR" ] && [ -n "$(ls -A "$DATASETS_DIR" 2>/dev/null)" ]; then
    run_datasets
  else
    log_info "Skipping dataset tests (no datasets found at $DATASETS_DIR)"
    log_info "To test with datasets, run:"
    log_info "  $0 $IMAGE_NAME /path/to/datasets"
  fi
  echo ""
  
  cleanup
  
  log_success "Docker automation complete!"
}

main "$@"
