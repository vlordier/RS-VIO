#!/bin/bash
# DHAT Heap Profiling for RS-VIO
# ===============================
# DHAT finds pathological allocation patterns and memory leaks.
# Requires: cargo install dhat-rs or jemalloc
#
# Usage:
#   ./scripts/run_dhat.sh              # Profile all tests
#   ./scripts/run_dhat.sh --lib        # Profile library tests only
#   ./scripts/run_dhat.sh estimator    # Profile specific module

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
cd "$PROJECT_ROOT"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

log_info() { echo -e "${GREEN}[INFO]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }

# Default settings
OUTPUT_DIR="dhat-output"
TEST_ARGS=""

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --lib)
            TEST_ARGS="--lib"
            shift
            ;;
        --output)
            OUTPUT_DIR="$2"
            shift 2
            ;;
        --help|-h)
            echo "DHAT Heap Profiling for RS-VIO"
            echo ""
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --lib          Profile library tests only"
            echo "  --output DIR   Output directory (default: dhat-output)"
            echo "  --help         Show this help"
            echo ""
            echo "Examples:"
            echo "  $0                     # Profile all tests"
            echo "  $0 --lib              # Profile lib only"
            echo "  $0 estimator          # Profile estimator tests"
            exit 0
            ;;
        *)
            TEST_ARGS="$TEST_ARGS $1"
            shift
            ;;
    esac
done

# Create output directory
mkdir -p "$OUTPUT_DIR"

log_info "Running DHAT heap profiling..."
log_info "Output directory: $OUTPUT_DIR"

# Check if dhat-rs is available
if ! cargo test --features dhat --no-run 2>&1 | grep -q "Finished"; then
    log_warn "Building with DHAT feature..."
fi

# Run tests with DHAT
# DHAT outputs to a JSON file on drop
export DHAT="file=$OUTPUT_DIR/dhat-$(date +%Y%m%d-%H%M%S).json"

log_info "Executing tests with heap profiling..."

cargo test --features dhat --release $TEST_ARGS 2>&1 | tee "$OUTPUT_DIR/test-output.log"

log_info "Heap profiling complete!"
log_info ""
log_info "Results saved to: $OUTPUT_DIR/"
echo ""
echo "To analyze results:"
echo "  1. Open the JSON file in a browser"
echo "  2. Or use: dhat $OUTPUT_DIR/*.json"
echo ""
echo "Top allocation sites are sorted by:"
echo "  - Bytes allocated (excluding deallocations)"
echo "  - Number of allocations"
echo ""

# Print summary of output files
if ls "$OUTPUT_DIR"/*.json &>/dev/null; then
    echo "Generated DHAT files:"
    ls -lh "$OUTPUT_DIR"/*.json | awk '{print "  " $9 " (" $5 ")"}'
fi
