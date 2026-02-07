#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
ROOT_DIR="$(cd -- "$SCRIPT_DIR/.." && pwd -P)"

if ! command -v shellcheck >/dev/null 2>&1; then
  echo "shellcheck is required. Install via: brew install shellcheck (macOS) or apt-get install shellcheck (Debian/Ubuntu)." >&2
  exit 1
fi

cd "$ROOT_DIR"
shellcheck scripts/*.sh
