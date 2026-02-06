#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: scripts/download_datasets.sh <target-dir> [euroc|tum|4seasons|all]

Downloads or prepares datasets for EuRoC, TUM-VI, and 4Seasons.
- target-dir: destination root directory (created if missing)
- second arg: which dataset to fetch (default: all)

Dataset Availability:
- EuRoC: Registration required at https://projects.asl.ethz.ch/datasets/euroc-mav/
  * Download MH_01_easy.zip manually and place in /tmp/MH_01_easy.zip
  * Script will extract it to <target-dir>/euroc/

- TUM-VI: Free download at https://vision.in.tum.de/data/datasets/visual-inertial-dataset
  * This script downloads the walking_xyz sequence automatically
  * ~1.5 GB download

- 4Seasons: Free download at https://www.4seasons-dataset.com/
  * Download one or more recording ZIPs manually
  * Extract to <target-dir>/4seasons/

Notes:
- TUM-VI downloads automatically (~1.5 GB)
- EuRoC and 4Seasons require manual downloads
- Extracted datasets will be 2-10 GB per sequence
EOF
}

require_cmd() {
  command -v "$1" >/dev/null 2>&1 || { echo "Missing required tool: $1" >&2; exit 1; }
}

fetch() {
  local url="$1" out="$2"
  echo "Downloading $url -> $out"
  mkdir -p "$(dirname "$out")"
  curl -L --fail --retry 5 --retry-delay 5 --retry-all-errors --continue-at - "$url" -o "$out"
}

extract_zip() {
  local archive="$1" dest="$2"
  echo "Extracting $archive -> $dest"
  mkdir -p "$dest"
  unzip -q "$archive" -d "$dest"
}

fetch_euroc() {
  local root="$1/euroc"
  echo "=== EuRoC MH_01_easy ==="
  echo "Note: EuRoC requires registration at https://projects.asl.ethz.ch/datasets/euroc-mav/"
  echo "Please download MH_01_easy.zip manually and extract to: $root"

  local archive="$1/.archives/MH_01_easy.zip"
  if [ -f "$archive" ]; then
    extract_zip "$archive" "$root"
    rm "$archive"
    echo "EuRoC MH_01_easy extracted to $root"
  else
    echo "Skipping EuRoC (requires manual download to $archive)"
  fi
}

fetch_tum() {
  local root="$1/tum_vi"
  local archives_dir="$1/.archives"
  echo "=== TUM-VI room1 (512_16) ==="
  local base_url="https://cdn2.vision.in.tum.de/tumvi/exported/euroc/512_16"
  local url="$base_url/dataset-room1_512_16.tar"
  local archive="$archives_dir/tum_vi_room1.tar"
  local out_dir="$root/room1"

  mkdir -p "$archives_dir" "$out_dir"

  if [ -d "$out_dir/mav0" ]; then
    echo "TUM-VI room1 already exists at $out_dir"
    return 0
  fi

  fetch "$url" "$archive"
  tar -xf "$archive" -C "$out_dir" --strip-components=1
  echo "TUM-VI room1 extracted to $out_dir"
}

fetch_4seasons() {
  local root="$1/4seasons"
  echo "=== 4Seasons Dataset ==="
  echo "Note: 4Seasons requires download from https://www.4seasons-dataset.com/"
  echo "Please download a recording ZIP and extract to: $root"

  # Fallback for development: create minimal structure if nothing exists
  if [ ! -d "$root" ] || [ -z "$(ls -A "$root" 2>/dev/null)" ]; then
    echo "4Seasons data not found. Please download manually from https://www.4seasons-dataset.com/"
    echo "Extract to: $root"
  fi
}

main() {
  if [[ ${1:-} == "-h" || ${1:-} == "--help" || $# -lt 1 ]]; then
    usage; exit 0
  fi

  require_cmd echo

  local target="$1"
  local which=${2:-all}
  mkdir -p "$target"

  case "$which" in
    euroc) fetch_euroc "$target" ;;
    tum) fetch_tum "$target" ;;
    4seasons) fetch_4seasons "$target" ;;
    all) fetch_euroc "$target"; fetch_tum "$target"; fetch_4seasons "$target" ;;
    *) echo "Unknown dataset selector: $which" >&2; usage; exit 1 ;;
  esac

  echo "Done. Samples stored under $target"
}

main "$@"
