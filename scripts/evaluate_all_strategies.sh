#!/usr/bin/env bash

# Complete calibration evaluation pipeline
# Runs all three strategies and compares results

set -e

DATASET="/tmp/rs-vio-samples/tum_vi"
WORK_DIR="/Users/vincent/Work/RS-VIO"
RESULTS_DIR="/tmp/calibration_evaluation_$(date +%Y%m%d_%H%M%S)"

mkdir -p "$RESULTS_DIR"

echo "═════════════════════════════════════════════════════════════"
echo "  TUM-VI INTRINSICS CALIBRATION EVALUATION"
echo "═════════════════════════════════════════════════════════════"
echo ""
echo "Dataset: $DATASET"
echo "Results: $RESULTS_DIR"
echo ""

# ============================================================================
# STRATEGY 1: Baseline (current system, default config)
# ============================================================================
echo "[STRATEGY 1] Running baseline on TUM-VI with default config..."
echo "  Config: config/tum_vi.yaml"
echo "  Expected: ~0.4% focal length error"
echo ""

cd "$WORK_DIR"

CONFIG1="config/tum_vi.yaml"
OUTPUT1="$RESULTS_DIR/strategy1_baseline"
mkdir -p "$OUTPUT1"

timeout 600 ./target/release/run_tum "$CONFIG1" "$DATASET" \
    2>&1 | tee "$OUTPUT1/run.log" || true

if [ -f "$DATASET/statistics.txt" ]; then
    cp "$DATASET/statistics.txt" "$OUTPUT1/statistics.txt"
fi

echo "✅ Strategy 1 complete. Results in $OUTPUT1"
echo ""

# ============================================================================
# STRATEGY 2: Online intrinsics refinement
# ============================================================================
echo "[STRATEGY 2] Running with online intrinsics refinement..."
echo "  Config: config/tum_vi_self_calibrating.yaml"
echo "  Expected: 0.1-0.2% focal length error"
echo ""

CONFIG2="config/tum_vi_self_calibrating.yaml"
OUTPUT2="$RESULTS_DIR/strategy2_online_refinement"
mkdir -p "$OUTPUT2"

timeout 600 ./target/release/run_tum "$CONFIG2" "$DATASET" \
    2>&1 | tee "$OUTPUT2/run.log" || true

if [ -f "$DATASET/statistics.txt" ]; then
    cp "$DATASET/statistics.txt" "$OUTPUT2/statistics.txt"
fi

echo "✅ Strategy 2 complete. Results in $OUTPUT2"
echo ""

# ============================================================================
# STRATEGY 3: Offline stereo calibrator refinement
# ============================================================================
echo "[STRATEGY 3] Running post-process calibration refinement..."
echo "  Method: Offline stereo calibrator on 500 frames"
echo "  Expected: <0.1% focal length error"
echo ""

OUTPUT3="$RESULTS_DIR/strategy3_offline_refinement"
mkdir -p "$OUTPUT3"

python "$WORK_DIR/tools/post_process_calibration.py" \
    --dataset "$DATASET" \
    --config "$CONFIG2" \
    --output "$OUTPUT3/refined.yaml" \
    --frames 500 \
    2>&1 | tee "$OUTPUT3/refinement.log"

echo "✅ Strategy 3 complete. Results in $OUTPUT3"
echo ""

# ============================================================================
# COMPARISON
# ============================================================================
echo "═════════════════════════════════════════════════════════════"
echo "  INTRINSICS COMPARISON"
echo "═════════════════════════════════════════════════════════════"
echo ""

echo "[BASELINE vs TUM-VI GROUND TRUTH]"
python "$WORK_DIR/tools/compare_tumvi_intrinsics.py" \
    --dataset "$DATASET" \
    --config "$CONFIG1" 2>&1 | tee "$RESULTS_DIR/comparison_baseline.txt"
echo ""

echo "[STRATEGY 2 vs TUM-VI GROUND TRUTH]"
python "$WORK_DIR/tools/compare_tumvi_intrinsics.py" \
    --dataset "$DATASET" \
    --config "$CONFIG2" 2>&1 | tee "$RESULTS_DIR/comparison_strategy2.txt"
echo ""

echo "[STRATEGY 3 (REFINED) vs TUM-VI GROUND TRUTH]"
python "$WORK_DIR/tools/compare_tumvi_intrinsics.py" \
    --dataset "$DATASET" \
    --config "$OUTPUT3/refined.yaml" 2>&1 | tee "$RESULTS_DIR/comparison_strategy3.txt"
echo ""

# ============================================================================
# SUMMARY
# ============================================================================
echo "═════════════════════════════════════════════════════════════"
echo "  EVALUATION SUMMARY"
echo "═════════════════════════════════════════════════════════════"
echo ""
echo "All results saved to: $RESULTS_DIR"
echo ""
echo "Outputs:"
echo "  Strategy 1 (Baseline):              $OUTPUT1"
echo "  Strategy 2 (Online Refinement):     $OUTPUT2"
echo "  Strategy 3 (Offline Refinement):    $OUTPUT3"
echo ""
echo "Comparison files:"
echo "  - $RESULTS_DIR/comparison_baseline.txt"
echo "  - $RESULTS_DIR/comparison_strategy2.txt"
echo "  - $RESULTS_DIR/comparison_strategy3.txt"
echo ""
echo "Next: Review the comparison files to see which strategy gives best accuracy!"
echo ""
