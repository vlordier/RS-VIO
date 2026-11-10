#!/bin/bash
# Complete Performance Measurement Script for RS-VIO
# 
# This script measures:
# 1. Motion prior overhead (criterion benchmarks)
# 2. Accuracy improvements (ATE/RPE comparison)
# 3. Existing benchmarks (for comparison)
#
# Usage: ./scripts/run_all_measurements.sh [dataset_path]
#
# Example: ./scripts/run_all_measurements.sh /Users/vincent/data/tum-vi/dataset-corridor1_512_16

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
cd "$PROJECT_DIR"

echo "========================================================================"
echo "RS-VIO Complete Performance Measurement"
echo "========================================================================"
echo "Project directory: $PROJECT_DIR"
echo "Date: $(date)"
echo ""

# Create results directory
RESULTS_DIR="$PROJECT_DIR/performance_results"
mkdir -p "$RESULTS_DIR"

DATASET_PATH="${1:-}"

# ============================================================================
# 1. Motion Prior Overhead Benchmarks
# ============================================================================
echo "========================================================================"
echo "1. MEASURING MOTION PRIOR OVERHEAD"
echo "========================================================================"

MOTION_PRIOR_LOG="$RESULTS_DIR/motion_prior_$(date +%Y%m%d_%H%M%S).log"
echo "Output: $MOTION_PRIOR_LOG"

cargo bench --bench motion_prior 2>&1 | tee "$MOTION_PRIOR_LOG"

echo ""
echo "✓ Motion prior overhead measured"
echo ""

# ============================================================================
# 2. Sliding Window Benchmarks (for comparison)
# ============================================================================
echo "========================================================================"
echo "2. MEASURING SLIDING WINDOW BENCHMARKS"
echo "========================================================================"

SLIDING_WINDOW_LOG="$RESULTS_DIR/sliding_window_$(date +%Y%m%d_%H%M%S).log"
echo "Output: $SLIDING_WINDOW_LOG"

cargo bench --bench sliding_window 2>&1 | tee "$SLIDING_WINDOW_LOG"

echo ""
echo "✓ Sliding window benchmarks measured"
echo ""

# ============================================================================
# 3. Accuracy Comparison (if dataset provided)
# ============================================================================
if [ -n "$DATASET_PATH" ]; then
    echo "========================================================================"
    echo "3. MEASURING ACCURACY IMPROVEMENTS"
    echo "========================================================================"
    echo "Dataset: $DATASET_PATH"
    
    ACCURACY_LOG="$RESULTS_DIR/accuracy_$(date +%Y%m%d_%H%M%S).log"
    ACCURACY_JSON="$RESULTS_DIR/accuracy_comparison_$(date +%Y%m%d_%H%M%S).json"
    
    echo "Output: $ACCURACY_LOG"
    echo "JSON: $ACCURACY_JSON"
    
    python3 scripts/measure_prior_accuracy.py \
        "$DATASET_PATH" \
        --config config/tum_vi.yaml \
        --json "$ACCURACY_JSON" 2>&1 | tee "$ACCURACY_LOG"
    
    echo ""
    echo "✓ Accuracy improvements measured"
else
    echo "========================================================================"
    echo "3. SKIPPING ACCURACY MEASUREMENT (no dataset path provided)"
    echo "========================================================================"
    echo "To measure accuracy improvements, run:"
    echo "  $0 /path/to/dataset"
    echo ""
    echo "Example datasets:"
    echo "  /Users/vincent/data/tum-vi/dataset-corridor1_512_16"
    echo "  /Users/vincent/data/tum-vi/dataset-corridor2_512_16"
    echo "  /Users/vincent/data/tum-vi/dataset-room1_512_16"
fi

# ============================================================================
# Summary
# ============================================================================
echo ""
echo "========================================================================"
echo "MEASUREMENT COMPLETE"
echo "========================================================================"
echo ""
echo "Results saved to:"
echo "  $RESULTS_DIR/"
echo ""
echo "Files:"
ls -lh "$RESULTS_DIR"
echo ""
echo "Next steps:"
echo "1. Review motion prior overhead in: $MOTION_PRIOR_LOG"
echo "2. Review accuracy improvements in: $ACCURACY_JSON"
echo "3. Update PERFORMANCE_MEASUREMENT_STATUS.md with actual numbers"
echo ""
echo "To run on another dataset:"
echo "  $0 /path/to/another/dataset"
echo ""
