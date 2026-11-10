#!/bin/bash
# Commit Makefile and developer UX improvements
#
# Usage: ./scripts/commit_makefile.sh

set -e

cd /Users/vincent/Work/RS-VIO

echo "========================================================================"
echo "Committing Makefile and Developer UX Improvements"
echo "========================================================================"
echo ""

# Show what will be committed
echo "Files to commit:"
git status --short
echo ""

# Commit
git add -A
git commit -m "feat: Add Makefile for improved developer UX

Makefile Targets:

Build:
  make build        - Build release binary
  make debug        - Build debug binary
  make clean        - Clean build artifacts

Test:
  make test         - Run all tests
  make test-fast    - Run fast tests only
  make test-overhead - Run motion prior overhead test

Benchmarks:
  make bench        - Run all benchmarks
  make bench-motion - Motion prior overhead (~3 min)
  make bench-sliding - Sliding window (~5 min)
  make bench-feature - Feature tracker (~2 min)

Performance Measurement:
  make measure      - Complete measurement (~20 min)
  make measure-overhead - Motion prior only (~3 min)
  make measure-accuracy - Accuracy comparison (~15 min/dataset)
  
  Example:
    make measure DATASET=/path/to/dataset

Code Quality:
  make fmt          - Format code
  make fmt-check    - Check formatting
  make lint         - Run clippy
  make lint-fix     - Auto-fix clippy

Documentation:
  make docs         - Generate docs
  make docs-open    - Generate and open docs

Utilities:
  make results      - View latest results
  make results-dir  - Open results directory
  make help         - Show all commands

Developer Workflow:
  make dev          - Quick dev cycle (fmt + test + lint)
  make ci           - Full CI check (fmt-check + test + lint)

Documentation:
- DEVELOPER.md: Complete developer guide with examples
- README.md: Updated with make commands
- Makefile: Self-documenting with help target

Benefits:
- Simplified command interface
- Consistent development workflow
- Easy performance measurement
- CI-ready commands"

echo ""
echo "✓ Commit complete"
echo ""
echo "Quick start:"
echo "  make help     - Show all commands"
echo "  make dev      - Quick development cycle"
echo "  make measure  - Run performance measurements"
