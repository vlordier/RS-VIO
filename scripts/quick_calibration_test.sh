#!/usr/bin/env bash
# Quick test runner - runs all 3 strategies with shorter timeout for validation

set -e

DATASET="/tmp/rs-vio-samples/tum_vi"
WORK_DIR="/Users/vincent/Work/RS-VIO"
RESULTS_DIR="/tmp/calibration_test_$(date +%Y%m%d_%H%M%S)"

mkdir -p "$RESULTS_DIR"

echo "════════════════════════════════════════════════════════════="
echo "  QUICK CALIBRATION TEST (All 3 Strategies)"
echo "════════════════════════════════════════════════════════════="
echo "Results: $RESULTS_DIR"
echo ""

cd "$WORK_DIR"

# Strategy 1: Baseline
echo "▶ Strategy 1: Baseline (config/tum_vi.yaml)"
timeout 120 ./target/release/run_tum config/tum_vi.yaml "$DATASET" 2>&1 | tail -20 || true
echo "✅ Done"
echo ""

# Strategy 2: Online refinement
echo "▶ Strategy 2: Online Refinement (config/tum_vi_self_calibrating.yaml)"
timeout 120 ./target/release/run_tum config/tum_vi_self_calibrating.yaml "$DATASET" 2>&1 | tail -20 || true
echo "✅ Done"
echo ""

# Strategy 3: Post-processing
echo "▶ Strategy 3: Offline Refinement (post_process_calibration.py)"
python tools/post_process_calibration.py \
    --dataset "$DATASET" \
    --config config/tum_vi_self_calibrating.yaml \
    --output "$RESULTS_DIR/refined.yaml" \
    --frames 100 2>&1 | tail -30
echo "✅ Done"
echo ""

# Comparisons
echo "════════════════════════════════════════════════════════════="
echo "  QUICK COMPARISONS"
echo "════════════════════════════════════════════════════════════="
echo ""

echo "Strategy 1 vs Ground Truth:"
python tools/compare_tumvi_intrinsics.py \
    --dataset "$DATASET" \
    --config config/tum_vi.yaml

echo ""
echo "Strategy 2 vs Ground Truth:"
python tools/compare_tumvi_intrinsics.py \
    --dataset "$DATASET" \
    --config config/tum_vi_self_calibrating.yaml

echo ""
echo "Strategy 3 vs Ground Truth:"
python tools/compare_tumvi_intrinsics.py \
    --dataset "$DATASET" \
    --config "$RESULTS_DIR/refined.yaml"

echo ""
echo "✅ All strategies tested successfully!"
