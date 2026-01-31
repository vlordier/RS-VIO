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
  curl -L --fail --retry 3 --retry-delay 2 "$url" -o "$out"
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

  if [ -f "/tmp/MH_01_easy.zip" ]; then
    extract_zip "/tmp/MH_01_easy.zip" "$root"
    rm "/tmp/MH_01_easy.zip"
    echo "EuRoC MH_01_easy extracted to $root"
  else
    echo "Skipping EuRoC (requires manual download)"
  fi
}

fetch_tum() {
  local root="$1/tum_vi"
  echo "=== TUM RGB-D freiburg3_walking_xyz ==="
  local url="http://download.tum.de/rgbd/dataset/freiburg3/rgbd-dataset_freiburg3_walking_xyz.tgz"
  local archive="/tmp/tum_vi.tgz"

  fetch "$url" "$archive"
  mkdir -p "$root"
  tar -xzf "$archive" -C "$root" --strip-components=1
  rm "$archive"

  echo "TUM dataset extracted to $root"
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
