#!/bin/bash
set -e

echo "Running Rust CI checks..."

# Run tests
echo "Running cargo test..."
cargo test --quiet

# Run clippy
echo "Running cargo clippy..."
cargo clippy --all-targets -- -D warnings

# Run rustfmt check
echo "Running cargo fmt check..."
cargo fmt --all -- --check

echo "All checks passed!"
