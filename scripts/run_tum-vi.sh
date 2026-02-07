#!/usr/bin/env bash
# Default logging: quiet rerun internals, keep our crate at debug.
export RUST_LOG="info,rs_vio=debug,rerun=warn,re_log=warn,re_sdk=warn,re_ws_comms=warn"

cd "$(dirname "${BASH_SOURCE[0]}")/.." || exit 1

# Use DATASET_DIR if set, otherwise default to local datasets/tum_vi/room1
dataset_dir="${DATASET_DIR:-datasets/tum_vi/room1}"

cargo run --release --bin run_tum config/tum_vi.yaml "$dataset_dir" "$@"
