#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BASE_BRANCH="${1:-develop}"
FEATURE_REF="${2:-HEAD}"

if [[ -n "$(git -C "$ROOT_DIR" status --porcelain)" ]]; then
  echo "Working tree is not clean. Please commit or stash changes before running." >&2
  exit 1
fi

current_branch="$(git -C "$ROOT_DIR" rev-parse --abbrev-ref HEAD)"
if [[ "$FEATURE_REF" == "HEAD" ]]; then
  FEATURE_REF="$current_branch"
fi

run_eval_tests() {
  local ref="$1"
  echo ""
  echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
  echo "Trajectory evaluation tests: $ref"
  echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
  git -C "$ROOT_DIR" checkout -q "$ref"
  /usr/bin/time -p cargo test --lib trajectory_evaluation --release
}

cleanup() {
  git -C "$ROOT_DIR" checkout -q "$current_branch"
}
trap cleanup EXIT

run_eval_tests "$BASE_BRANCH"
run_eval_tests "$FEATURE_REF"
