#!/usr/bin/env bash
set -euo pipefail

# Default logging: quiet rerun internals, keep our crate at debug.
export RUST_LOG="${RUST_LOG:-info,rs_vio=debug,rerun=warn,re_log=warn,re_sdk=warn,re_ws_comms=warn}"

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
ROOT_DIR="$(cd -- "$SCRIPT_DIR/.." && pwd -P)"

CONFIG_PATH="${CONFIG_PATH:-config/euroc_vio.yaml}"

if [[ $# -lt 1 ]]; then
	echo "Usage: $0 <euroc_dataset_path> [extra args...]" >&2
	exit 1
fi

DATASET_PATH="$1"
shift

cd "$ROOT_DIR"
exec cargo run --release --bin run_euroc "$CONFIG_PATH" "$DATASET_PATH" "$@"
