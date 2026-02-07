#!/usr/bin/env bash
# Default logging: quiet rerun internals, keep our crate at debug.
export RUST_LOG="info,rs_vio=debug,rerun=warn,re_log=warn,re_sdk=warn,re_ws_comms=warn"

cd "$(dirname "${BASH_SOURCE[0]}")/.." || exit 1

# Use DATASET_DIR if set, otherwise find first 4Seasons recording
if [ -n "$DATASET_DIR" ]; then
  dataset_dir="$DATASET_DIR"
else
  dataset_dir=$(find datasets/4seasons -maxdepth 1 -type d -name 'recording_*' 2>/dev/null | head -1)
  if [ -z "$dataset_dir" ]; then
    echo "Error: No 4Seasons recordings found in datasets/4seasons/"
    exit 1
  fi
fi

cargo run --release --bin run_4seasons config/4seasons.yaml "$dataset_dir" "$@"
