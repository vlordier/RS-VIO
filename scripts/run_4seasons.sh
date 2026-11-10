#!/usr/bin/env bash
# Default logging: quiet rerun internals, keep our crate at debug.
export RUST_LOG="info,rs_vio=debug,rerun=warn,re_log=warn,re_sdk=warn,re_ws_comms=warn"

cd "$(dirname "${BASH_SOURCE[0]}")/.."
cargo run --release --bin run_4seasons config/4seasons.yaml /home/charles/Workspace/data/4seasons/some/  "$@"
