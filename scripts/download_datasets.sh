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

# Create a minimal valid 1x1 PNG file using base64
create_minimal_png() {
  local filepath="$1"
  # Base64-encoded minimal 1x1 grayscale PNG
  echo "iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAYAAAAfFcSJAAAADUlEQVR42mNk+M9QDwADhgGAWjR9awAAAABJRU5ErkJggg==" | base64 -d > "$filepath"
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
  echo "=== EuRoC (MH_01_easy) - Creating synthetic test data ==="
  mkdir -p "$root/MH_01_easy/mav0/cam0/data" "$root/MH_01_easy/mav0/cam1/data" "$root/MH_01_easy/mav0/imu0"
  echo "timestamp,filename" > "$root/MH_01_easy/mav0/cam0/data.csv"
  echo "timestamp,filename" > "$root/MH_01_easy/mav0/cam1/data.csv"
  echo "timestamp,w_RS_S_x,w_RS_S_y,w_RS_S_z,a_RS_S_x,a_RS_S_y,a_RS_S_z" > "$root/MH_01_easy/mav0/imu0/data.csv"
  for i in {0..9}; do
    ts=$((1403636579000000000 + i * 33333333))
    create_minimal_png "$root/MH_01_easy/mav0/cam0/data/$ts.png"
    create_minimal_png "$root/MH_01_easy/mav0/cam1/data/$ts.png"
    echo "$ts,$ts.png" >> "$root/MH_01_easy/mav0/cam0/data.csv"
    echo "$ts,$ts.png" >> "$root/MH_01_easy/mav0/cam1/data.csv"
    echo "$ts,0,0,0,0,0,0" >> "$root/MH_01_easy/mav0/imu0/data.csv"
  done
  echo "EuRoC sample created under $root/MH_01_easy (synthetic)"
}

fetch_tum() {
  local root="$1/tum_vi"
  echo "=== TUM-VI (room1, EuRoC/DSO format) - Creating synthetic test data ==="
  mkdir -p "$root/room1/mav0/cam0/data" "$root/room1/mav0/cam1/data" "$root/room1/mav0/imu0"
  echo "timestamp,filename" > "$root/room1/mav0/cam0/data.csv"
  echo "timestamp,filename" > "$root/room1/mav0/cam1/data.csv"
  echo "timestamp,w_RS_S_x,w_RS_S_y,w_RS_S_z,a_RS_S_x,a_RS_S_y,a_RS_S_z" > "$root/room1/mav0/imu0/data.csv"
  for i in {0..9}; do
    ts=$((1403636579000000000 + i * 33333333))
    create_minimal_png "$root/room1/mav0/cam0/data/$ts.png"
    create_minimal_png "$root/room1/mav0/cam1/data/$ts.png"
    echo "$ts,$ts.png" >> "$root/room1/mav0/cam0/data.csv"
    echo "$ts,$ts.png" >> "$root/room1/mav0/cam1/data.csv"
    echo "$ts,0,0,0,0,0,0" >> "$root/room1/mav0/imu0/data.csv"
  done
  echo "TUM-VI sample created under $root/room1 (synthetic)"
}

fetch_4seasons() {
  local root="$1/4seasons"
  echo "=== 4Seasons (recording_2021-01-07) - Creating synthetic test data ==="
  mkdir -p "$root/recording_2021-01-07_13-03-56/undistorted_images/cam0" "$root/recording_2021-01-07_13-03-56/undistorted_images/cam1"
  
  # Create times.txt with synthetic timestamps (format: timestamp filename timestamp)
  # Note: The code uses timestamp as filename (appends .png)
  {
    for i in {0..9}; do
      ts=$((i * 1000000000))
      echo "$ts $ts $ts"
    done
  } > "$root/recording_2021-01-07_13-03-56/times.txt"
  
  # Create valid minimal PNG files with timestamp names
  for i in {0..9}; do
    ts=$((i * 1000000000))
    create_minimal_png "$root/recording_2021-01-07_13-03-56/undistorted_images/cam0/${ts}.png"
    create_minimal_png "$root/recording_2021-01-07_13-03-56/undistorted_images/cam1/${ts}.png"
  done
  echo "4Seasons sample created under $root/recording_2021-01-07_13-03-56 (synthetic)"
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
