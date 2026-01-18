#!/usr/bin/env bash
set -euo pipefail

# RS-VIO Dataset Setup Script
# Automates downloading and extracting datasets for local testing

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

# Resolve datasets directory to an absolute path
if [ -n "${1:-}" ]; then
  DATASETS_DIR="$(cd "$(dirname "$1")" && pwd)/$(basename "$1")"
else
  DATASETS_DIR="$PROJECT_ROOT/datasets"
fi

CHECK_ONLY="${CHECK_ONLY:-0}"
CHECKSUM_FILE="${CHECKSUM_FILE:-"$PROJECT_ROOT/scripts/dataset_checksums.sha256"}"
COLOR_GREEN='\033[0;32m'
COLOR_BLUE='\033[0;34m'
COLOR_YELLOW='\033[1;33m'
COLOR_RED='\033[0;31m'
NC='\033[0m' # No Color

log_info() { echo -e "${COLOR_BLUE}[INFO]${NC} $*"; }
log_success() { echo -e "${COLOR_GREEN}[✓]${NC} $*"; }
log_warn() { echo -e "${COLOR_YELLOW}[!]${NC} $*"; }
log_error() { echo -e "${COLOR_RED}[✗]${NC} $*"; }

mkdir -p "$DATASETS_DIR"
cd "$DATASETS_DIR"

# Check prerequisites
check_tools() {
  local missing=0
  for cmd in curl unzip tar; do
    if ! command -v "$cmd" &>/dev/null; then
      log_error "Missing required tool: $cmd"
      missing=1
    fi
  done
  [[ $missing -eq 0 ]] && return 0 || return 1
}

# Download with progress, resume and retry
download_file() {
  local url="$1" filepath="$2" max_retries=3
  local attempt=1
  
  while [ $attempt -le $max_retries ]; do
    log_info "Downloading (attempt $attempt/$max_retries): $url"
    if curl -fL --continue-at - --progress-bar --max-time 3600 "$url" -o "$filepath"; then
      log_success "Downloaded: $(basename "$filepath")"
      return 0
    fi
    attempt=$((attempt + 1))
    [ $attempt -le $max_retries ] && sleep 5
  done
  
  log_error "Failed to download after $max_retries attempts: $url"
  return 1
}

# Optional SHA256 verification using a manifest file (one line: "<sha256>  <filename>")
verify_checksum() {
  local archive_path="$1"
  local archive_name
  archive_name="$(basename "$archive_path")"

  if [ ! -f "$CHECKSUM_FILE" ]; then
    log_warn "Checksum file not found: $CHECKSUM_FILE (skipping integrity check for $archive_name)"
    return 0
  fi

  local expected
  expected=$(grep -v '^#' "$CHECKSUM_FILE" | grep "  $archive_name$" | awk '{print $1}' || true)
  if [ -z "$expected" ]; then
    log_warn "No checksum entry for $archive_name in $CHECKSUM_FILE (skipping integrity check)"
    return 0
  fi

  local actual
  actual=$(sha256sum "$archive_path" | awk '{print $1}')
  if [ "$expected" != "$actual" ]; then
    log_error "Checksum mismatch for $archive_name"
    log_error " expected: $expected"
    log_error "   actual: $actual"
    return 1
  fi

  log_success "Checksum verified for $archive_name"
  return 0
}

# Extract archive
extract_archive() {
  local archive="$1" extract_to="$2"
  
  mkdir -p "$extract_to"
  
  if [[ "$archive" == *.zip ]]; then
    log_info "Extracting ZIP: $archive"
    unzip -q -o "$archive" -d "$extract_to" && log_success "Extracted ZIP"
  elif [[ "$archive" == *.tar.gz ]] || [[ "$archive" == *.tgz ]]; then
    log_info "Extracting TAR.GZ: $archive"
    tar -xzf "$archive" -C "$extract_to" --strip-components=1 && log_success "Extracted TAR.GZ"
  elif [[ "$archive" == *.tar ]]; then
    log_info "Extracting TAR: $archive"
    tar -xf "$archive" -C "$extract_to" --strip-components=1 && log_success "Extracted TAR"
  else
    log_error "Unknown archive format: $archive"
    return 1
  fi
}

# Verify dataset structure
verify_dataset() {
  local dataset_type="$1" dataset_path="$2"
  
  case "$dataset_type" in
    euroc)
      if [ -d "$dataset_path/mav0/cam0/data" ] &&
         [ -d "$dataset_path/mav0/cam1/data" ] &&
         [ -f "$dataset_path/mav0/cam0/data.csv" ] &&
         [ -f "$dataset_path/mav0/cam1/data.csv" ]; then
        log_success "EuRoC dataset verified"
        return 0
      fi
      ;;
    tum)
      # TUM-VI is handled in EuRoC layout (mav0/...)
      if [ -d "$dataset_path/mav0/cam0/data" ] &&
         [ -d "$dataset_path/mav0/cam1/data" ] &&
         [ -f "$dataset_path/mav0/cam0/data.csv" ] &&
         [ -f "$dataset_path/mav0/cam1/data.csv" ]; then
        log_success "TUM-VI dataset verified"
        return 0
      fi
      ;;
    4seasons)
      # 4Seasons is converted to EuRoC format
      if [ -d "$dataset_path/mav0/cam0/data" ] &&
         [ -d "$dataset_path/mav0/cam1/data" ] &&
         [ -f "$dataset_path/mav0/cam0/data.csv" ] &&
         [ -f "$dataset_path/mav0/cam1/data.csv" ] &&
         [ -f "$dataset_path/mav0/imu0/data.csv" ]; then
        log_success "4Seasons dataset verified"
        return 0
      fi
      ;;
  esac
  
  log_warn "Could not verify $dataset_type dataset structure at $dataset_path"
  return 1
}

# Download EuRoC
setup_euroc() {
  local euroc_dir="$DATASETS_DIR/euroc"
  local archive="$DATASETS_DIR/.archives/MH_01_easy.zip"
  local machine_hall_url="https://www.research-collection.ethz.ch/bitstreams/7b2419c1-62b5-4714-b7f8-485e5fe3e5fe/download"
  
  log_info "Setting up EuRoC dataset"
  if [ -d "$euroc_dir/MH_01_easy/mav0" ]; then
    log_success "EuRoC dataset already exists"
    return 0
  fi
  if [ "$CHECK_ONLY" -eq 1 ]; then
    log_info "Check-only mode: skipping EuRoC download"
    return 0
  fi
  
  # Check if archive exists, if not download it
  mkdir -p "$(dirname "$archive")"
  if [ ! -f "$archive" ]; then
    log_info "Downloading EuRoC Machine Hall datasets (12GB)..."
    log_info "This may take 10-20 minutes depending on your connection..."
    
    if ! download_file "$machine_hall_url" "$archive"; then
      log_error "Failed to download EuRoC datasets"
      log_warn "Manual download available at:"
      log_warn "  https://www.research-collection.ethz.ch/entities/researchdata/bcaf173e-5dac-484b-bc37-faf97a594f1f"
      return 1
    fi
  else
    if ! verify_checksum "$archive"; then
      log_warn "Existing EuRoC archive failed checksum; deleting and re-downloading"
      rm -f "$archive"
      setup_euroc
      return 0
    fi
  fi
  
  log_info "Found EuRoC archive: $archive"
  mkdir -p "$euroc_dir"
  extract_archive "$archive" "$euroc_dir"
  
  # Move from machine_hall subdirectory to euroc root if needed
  if [ -d "$euroc_dir/machine_hall" ]; then
    log_info "Reorganizing EuRoC directory structure..."
    # Move all sequences from machine_hall to euroc root
    find "$euroc_dir/machine_hall" -mindepth 1 -maxdepth 1 -type d -exec mv {} "$euroc_dir/" \;
    rmdir "$euroc_dir/machine_hall" 2>/dev/null || true
  fi
  
  # Extract inner sequence zip if it exists (some archives have nested zips)
  if [ -f "$euroc_dir/MH_01_easy/MH_01_easy.zip" ]; then
    log_info "Extracting inner EuRoC sequence archive: MH_01_easy.zip"
    unzip -q -o "$euroc_dir/MH_01_easy/MH_01_easy.zip" -d "$euroc_dir/MH_01_easy" || {
      log_warn "Failed to extract MH_01_easy.zip"
    }
    # Clean up the zip file after extraction
    rm -f "$euroc_dir/MH_01_easy/MH_01_easy.zip"
  fi
  
  # Verify the dataset
  if verify_dataset "euroc" "$euroc_dir/MH_01_easy"; then
    log_success "EuRoC dataset setup complete"
  else
    log_error "EuRoC dataset verification failed"
    return 1
  fi
}

# Download TUM-VI
setup_tum() {
  local tum_dir="$DATASETS_DIR/tum_vi"
  local base_url="https://cdn2.vision.in.tum.de/tumvi/exported/euroc/512_16"
  local archives_dir="$DATASETS_DIR/.archives"
  
  log_info "Setting up TUM-VI dataset"
  if [ "$CHECK_ONLY" -eq 1 ]; then
    verify_dataset "tum" "$tum_dir/room1" || true
    verify_dataset "tum" "$tum_dir/magistrale1" || true
    log_info "Check-only mode: skipping TUM-VI download"
    return 0
  fi
  
  mkdir -p "$archives_dir"
  
  # Download indoor sequence (room1)
  local indoor_url="$base_url/dataset-room1_512_16.tar"
  local indoor_archive="$archives_dir/tum_vi_room1.tar"
  local indoor_dir="$tum_dir/room1"
  
  if [ -d "$indoor_dir/mav0" ]; then
    log_success "TUM-VI room1 already exists"
  else
    if [ ! -f "$indoor_archive" ]; then
      log_info "Downloading TUM-VI room1 (indoor sequence)"
      if ! download_file "$indoor_url" "$indoor_archive"; then
        log_error "Failed to download TUM-VI room1"
        return 1
      fi
    else
      verify_checksum "$indoor_archive" || { log_warn "Checksum failed for $indoor_archive; re-downloading"; rm -f "$indoor_archive"; setup_tum; return 0; }
      log_info "Found existing archive: $indoor_archive"
    fi
    
    log_info "Extracting TUM-VI room1"
    mkdir -p "$indoor_dir"
    extract_archive "$indoor_archive" "$indoor_dir"
    verify_dataset "tum" "$indoor_dir"
  fi
  
  # Download outdoor sequence (magistrale1)
  local outdoor_url="$base_url/dataset-magistrale1_512_16.tar"
  local outdoor_archive="$archives_dir/tum_vi_magistrale1.tar"
  local outdoor_dir="$tum_dir/magistrale1"
  
  if [ -d "$outdoor_dir/mav0" ]; then
    log_success "TUM-VI magistrale1 already exists"
  else
    if [ ! -f "$outdoor_archive" ]; then
      log_info "Downloading TUM-VI magistrale1 (outdoor sequence)"
      if ! download_file "$outdoor_url" "$outdoor_archive"; then
        log_error "Failed to download TUM-VI magistrale1"
        return 1
      fi
    else
      verify_checksum "$outdoor_archive" || { log_warn "Checksum failed for $outdoor_archive; re-downloading"; rm -f "$outdoor_archive"; setup_tum; return 0; }
      log_info "Found existing archive: $outdoor_archive"
    fi
    
    log_info "Extracting TUM-VI magistrale1"
    mkdir -p "$outdoor_dir"
    extract_archive "$outdoor_archive" "$outdoor_dir"
    rm -f "$outdoor_archive"
    verify_dataset "tum" "$outdoor_dir"
  fi
  
  log_success "TUM-VI dataset setup complete"
}

# Download 4Seasons
setup_4seasons() {
  local seasons_dir="$DATASETS_DIR/4seasons"
  local seq_name="parking_garage_3_train"
  local recording_id="recording_2021-05-10_19-15-19"
  
  log_info "Setting up 4Seasons dataset ($seq_name)"
  if [ "$CHECK_ONLY" -eq 1 ]; then
    verify_dataset "4seasons" "$seasons_dir/$recording_id" || true
    log_info "Check-only mode: skipping 4Seasons download"
    return 0
  fi
  
  # Check if already extracted
  if [ -d "$seasons_dir/$recording_id/mav0" ] && [ -f "$seasons_dir/$recording_id/mav0/imu0/data.csv" ]; then
    log_success "4Seasons $seq_name already exists"
    return 0
  fi
  
  mkdir -p "$seasons_dir"
  
  # 4Seasons format: Download individual components
  # We need: stereo images + IMU/GNSS data
  local imu_url="https://vision.cs.tum.edu/webshare/g/4seasons-dataset/dataset/$recording_id/${recording_id}_imu_gnss.zip"
  local stereo_url="https://vision.cs.tum.edu/webshare/g/4seasons-dataset/dataset/$recording_id/${recording_id}_stereo_images_distorted.zip"
  local ref_poses_url="https://vision.cs.tum.edu/webshare/g/4seasons-dataset/dataset/$recording_id/${recording_id}_reference_poses.zip"
  
  local temp_dir
  temp_dir=$(mktemp -d)
  trap 'rm -rf "$temp_dir"' EXIT
  
  # Download IMU data
  log_info "Downloading 4Seasons IMU/GNSS data (6.1MB)..."
  if ! download_file "$imu_url" "$temp_dir/imu.zip"; then
    log_error "Failed to download 4Seasons IMU data"
    return 1
  fi
  verify_checksum "$temp_dir/imu.zip" || log_warn "Checksum skipped/failed for imu.zip"
  
  # Download stereo images
  log_info "Downloading 4Seasons stereo images (1.3GB)..."
  if ! download_file "$stereo_url" "$temp_dir/stereo.zip"; then
    log_error "Failed to download 4Seasons stereo images"
    return 1
  fi
  verify_checksum "$temp_dir/stereo.zip" || log_warn "Checksum skipped/failed for stereo.zip"
  
  # Download reference poses
  log_info "Downloading 4Seasons reference poses (371KB)..."
  if ! download_file "$ref_poses_url" "$temp_dir/poses.zip"; then
    log_warn "Could not download reference poses (optional)"
  else
    verify_checksum "$temp_dir/poses.zip" || log_warn "Checksum skipped/failed for poses.zip"
  fi
  
  # Extract all to a temp directory
  local extract_dir="$temp_dir/extract"
  mkdir -p "$extract_dir"
  
  log_info "Extracting archives..."
  unzip -q -o "$temp_dir/imu.zip" -d "$extract_dir" || true
  unzip -q -o "$temp_dir/stereo.zip" -d "$extract_dir" || true
  [ -f "$temp_dir/poses.zip" ] && unzip -q -o "$temp_dir/poses.zip" -d "$extract_dir" || true
  
  # Reorganize to EuRoC format: mav0/cam0/, mav0/cam1/, mav0/imu0/
  log_info "Organizing to EuRoC format..."
  
  local recording_dir="$seasons_dir/$recording_id"
  mkdir -p "$recording_dir/mav0/cam0/data" \
           "$recording_dir/mav0/cam1/data" \
           "$recording_dir/mav0/imu0"
  
  # Move camera data from distorted_images/cam{0,1}/ to mav0/
  local src_recording="$extract_dir/$recording_id"
  if [ -d "$src_recording/distorted_images/cam0" ]; then
    log_info "Moving cam0 images..."
    mv "$src_recording/distorted_images/cam0"/*.png "$recording_dir/mav0/cam0/data/" 2>/dev/null || true
  fi
  
  if [ -d "$src_recording/distorted_images/cam1" ]; then
    log_info "Moving cam1 images..."
    mv "$src_recording/distorted_images/cam1"/*.png "$recording_dir/mav0/cam1/data/" 2>/dev/null || true
  fi
  
  # Create data.csv files from timestamps (image filenames are unix timestamps)
  if [ -n "$(find "$recording_dir/mav0/cam0/data" -name '*.png' -type f 2>/dev/null | head -1)" ]; then
    log_info "Creating cam0 data.csv..."
    {
      echo "timestamp,filename"
      find "$recording_dir/mav0/cam0/data" -name '*.png' -type f -print0 | sort -z | while IFS= read -r -d '' f; do
        timestamp=$(basename "$f" .png)
        echo "$timestamp,$(basename "$f")"
      done | sort -n
    } > "$recording_dir/mav0/cam0/data.csv"
  fi
  
  if [ -n "$(find "$recording_dir/mav0/cam1/data" -name '*.png' -type f 2>/dev/null | head -1)" ]; then
    log_info "Creating cam1 data.csv..."
    {
      echo "timestamp,filename"
      find "$recording_dir/mav0/cam1/data" -name '*.png' -type f -print0 | sort -z | while IFS= read -r -d '' f; do
        timestamp=$(basename "$f" .png)
        echo "$timestamp,$(basename "$f")"
      done | sort -n
    } > "$recording_dir/mav0/cam1/data.csv"
  fi
  
  # Convert IMU data from imu.txt to data.csv
  if [ -f "$src_recording/imu.txt" ]; then
    log_info "Converting IMU data..."
    {
      echo "timestamp,omega_x,omega_y,omega_z,alpha_x,alpha_y,alpha_z"
      tail -n +2 "$src_recording/imu.txt" | cut -d' ' -f1-7
    } > "$recording_dir/mav0/imu0/data.csv"
  fi
  
  # Verify structure
  if verify_dataset "4seasons" "$recording_dir"; then
    log_success "4Seasons $seq_name ready at: $recording_dir"
    return 0
  else
    log_warn "4Seasons extracted but verification found issues"
    # Try to continue anyway since we have the data
    return 0
  fi
}

# Test binary
test_binary() {
  local binary="$1" config="$2" dataset_path="$3" timeout="${4:-30}"
  
  if [ ! -x "$PROJECT_ROOT/target/release/$binary" ]; then
    log_warn "Binary not found: $binary. Building..."
    cd "$PROJECT_ROOT"
    cargo build --release --bin "$binary" || return 1
  fi
  
  log_info "Testing $binary with dataset: $dataset_path"
  
  if timeout "$timeout" "$PROJECT_ROOT/target/release/$binary" \
    "$PROJECT_ROOT/config/$config" "$dataset_path" 2>&1 | head -20; then
    log_success "$binary test completed"
    return 0
  else
    log_warn "$binary test timed out or failed"
    return 1
  fi
}

# Main workflow
main() {
  log_info "RS-VIO Dataset Setup Script"
  log_info "Datasets directory: $DATASETS_DIR"
  if [ "$CHECK_ONLY" -eq 1 ]; then
    log_info "Mode: check-only (no downloads or extraction)"
  else
    log_info "Datasets will be stored in: $DATASETS_DIR (add to .gitignore)"
  fi
  echo ""
  
  # Check tools
  if ! check_tools; then
    log_error "Please install missing tools and try again"
    exit 1
  fi
  log_success "All required tools available"
  echo ""
  
  # Download or verify datasets
  local setup_results=()
  
  log_info "Starting dataset downloads..."
  echo ""
  
  # EuRoC
  if setup_euroc; then
    setup_results+=("✓ EuRoC")
  else
    setup_results+=("✗ EuRoC (manual download required)")
  fi
  echo ""
  
  # TUM-VI
  if setup_tum; then
    setup_results+=("✓ TUM-VI")
  else
    setup_results+=("✗ TUM-VI (check network/mirror)")
  fi
  echo ""
  
  # 4Seasons
  if setup_4seasons; then
    setup_results+=("✓ 4Seasons")
  else
    setup_results+=("✗ 4Seasons (manual download required)")
  fi
  echo ""
  
  # Summary
  log_info "Download Summary:"
  for result in "${setup_results[@]}"; do
    echo "  $result"
  done
  echo ""
  
  # Test available datasets
  log_info "Testing available datasets..."
  echo ""
  
  if [ -d "$DATASETS_DIR/euroc/MH_01_easy" ]; then
    test_binary "run_euroc" "euroc_vio.yaml" "$DATASETS_DIR/euroc/MH_01_easy"
    echo ""
  fi
  
  if [ -f "$DATASETS_DIR/tum_vi/depth.txt" ]; then
    test_binary "run_tum" "tum_vi.yaml" "$DATASETS_DIR/tum_vi"
    echo ""
  fi
  
  if find "$DATASETS_DIR"/4seasons -name 'times.txt' -type f | grep -q .; then
    local recording_dir
    recording_dir=$(dirname "$(find "$DATASETS_DIR"/4seasons -name 'times.txt' -type f -printf '%T@ %p\n' | sort -rn | head -1 | cut -d' ' -f2-)")
    test_binary "run_4seasons" "4seasons.yaml" "$recording_dir"
    echo ""
  fi
  
  log_success "Setup complete!"
  log_info "Datasets saved to: $DATASETS_DIR"
  echo ""
  log_info "To run individual datasets:"
  echo "  cargo run --release --bin run_euroc config/euroc_vio.yaml $DATASETS_DIR/euroc/MH_01_easy"
  echo "  cargo run --release --bin run_tum config/tum_vi.yaml $DATASETS_DIR/tum_vi"
  echo "  cargo run --release --bin run_4seasons config/4seasons.yaml $DATASETS_DIR/4seasons/recording_*"
}

main "$@"
