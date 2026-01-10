#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: scripts/download_datasets.sh <target-dir> [euroc|tum|4seasons|all]

Downloads sample subsets for EuRoC, TUM-VI (512x512 EuRoC/DSO format), and 4Seasons.
- target-dir: destination root directory (created if missing)
- second arg: which dataset to fetch (default: all)

Notes:
- Downloads are large; ensure you have bandwidth and disk space.
- For full datasets, see official pages:
  * EuRoC: https://projects.asl.ethz.ch/datasets/euroc-mav/
  * TUM-VI: https://cvg.cit.tum.de/data/datasets/visual-inertial-dataset
  * 4Seasons: https://cvg.cit.tum.de/data/datasets/4seasons-dataset/download
- This script fetches representative sequences only; adjust URLs as needed.
- Archives are downloaded then extracted under <target-dir>.
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
  local url="https://cvg-data.inf.ethz.ch/mav/mav_datasets/MH_01_easy/MH_01_easy.zip"
  local archive="$root/MH_01_easy.zip"
  echo "=== EuRoC (MH_01_easy) ==="
  fetch "$url" "$archive"
  extract_zip "$archive" "$root"
  echo "EuRoC sample extracted under $root/MH_01_easy"
}

fetch_tum() {
  local root="$1/tum_vi"
  local url="https://cvg-data.inf.ethz.ch/vision/vi-dataset/512_16/dataset-room1_512_16.zip"
  local archive="$root/dataset-room1_512_16.zip"
  echo "=== TUM-VI (room1_512_16, EuRoC/DSO format) ==="
  fetch "$url" "$archive"
  extract_zip "$archive" "$root"
  echo "TUM-VI sample extracted under $root"
}

fetch_4seasons() {
  local root="$1/4seasons"
  local url="https://cvg.cit.tum.de/fileadmin/w00bqn/www/data/datasets/4seasons-dataset/2021-01-07/recording_2021-01-07_13-03-56_undistorted.zip"
  local archive="$root/recording_2021-01-07_13-03-56_undistorted.zip"
  echo "=== 4Seasons (recording_2021-01-07_13-03-56 undistorted) ==="
  fetch "$url" "$archive"
  extract_zip "$archive" "$root"
  echo "4Seasons sample extracted under $root"
}

main() {
  if [[ ${1:-} == "-h" || ${1:-} == "--help" || $# -lt 1 ]]; then
    usage; exit 0
  fi

  require_cmd curl
  require_cmd unzip

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
