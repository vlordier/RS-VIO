#!/usr/bin/env bash
set -euo pipefail

# Create a minimal valid 1x1 PNG file
create_minimal_png() {
  local filepath="$1"
  echo "iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAYAAAAfFcSJAAAADUlEQVR42mNk+M9QDwADhgGAWjR9awAAAABJRU5ErkJggg==" | base64 -d > "$filepath"
}

# Generate EuRoC test data
generate_euroc() {
  local root="$1/euroc"
  echo "Creating EuRoC test data..."
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

  echo "✓ EuRoC created at $root/MH_01_easy"
}

# Generate TUM-VI test data
generate_tum() {
  local root="$1/tum_vi"
  echo "Creating TUM-VI test data..."
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

  echo "✓ TUM-VI created at $root/room1"
}

# Generate 4Seasons test data
generate_4seasons() {
  local root="$1/4seasons"
  echo "Creating 4Seasons test data..."
  mkdir -p "$root/recording_2021-01-07_13-03-56/rgb"

  {
    for i in {0..9}; do
      echo "0.$i"
    done
  } > "$root/recording_2021-01-07_13-03-56/times.txt"

  for i in {0..9}; do
    create_minimal_png "$root/recording_2021-01-07_13-03-56/rgb/00$i.png"
  done

  echo "✓ 4Seasons created at $root/recording_2021-01-07_13-03-56"
}

# Main entry point
main() {
  if [[ $# -lt 1 ]]; then
    echo "Usage: $0 <output-dir> [euroc|tum|4seasons|all]"
    echo "Creates synthetic test datasets for RS-VIO"
    exit 1
  fi

  local target="$1"
  local which="${2:-all}"
  mkdir -p "$target"

  case "$which" in
    euroc) generate_euroc "$target" ;;
    tum) generate_tum "$target" ;;
    4seasons) generate_4seasons "$target" ;;
    all)
      generate_euroc "$target"
      generate_tum "$target"
      generate_4seasons "$target"
      ;;
    *)
      echo "Unknown dataset: $which" >&2
      exit 1
      ;;
  esac

  echo "✓ All datasets created in $target"
}

main "$@"
