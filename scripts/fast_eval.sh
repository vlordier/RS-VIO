#!/bin/bash
# Fast evaluation mode - trades some accuracy for rapid iteration
# Typical speedup: 2.5-3x (lower resolution, fewer features)

set -e

usage() {
  echo "Usage: $0 <sequence> [--quick|--ultra]"
  echo ""
  echo "Modes:"
  echo "  --quick  : 384×384 resolution, 2 features/grid (default: 2-2.5x faster)"
  echo "  --ultra  : 256×256 resolution, 1 feature/grid (3-4x faster, less accurate)"
  echo ""
  echo "Examples:"
  echo "  $0 room1                # Quick mode (default)"
  echo "  $0 slides1 --ultra      # Ultra-fast mode for rapid iteration"
}

if [ -z "$1" ]; then
  usage
  exit 1
fi

SEQUENCE=$1
MODE=${2:-"--quick"}
BINARY="/Users/vincent/Work/RS-VIO/target/release/run_tum"
CONFIG_BASE="/Users/vincent/Work/RS-VIO/config/tum_vi.yaml"
DATASET="/Users/vincent/data/tum-vi/dataset-${SEQUENCE}_512_16"
OUTPUT_EST="$DATASET/estimated_trajectory_fast.txt"
OUTPUT_GT="$DATASET/mav0/mocap0/data.csv"

if [ ! -d "$DATASET" ]; then
  echo "Error: Dataset not found: $DATASET"
  exit 1
fi

echo "================================================================================"
echo "Fast Evaluation Mode: $SEQUENCE $MODE"
echo "================================================================================"

# Create temporary config based on mode
TEMP_CONFIG=$(mktemp)
trap "rm -f $TEMP_CONFIG" EXIT

cp "$CONFIG_BASE" "$TEMP_CONFIG"

case "$MODE" in
  --quick)
    # 384×384, 2 features per grid = ~2.5x faster
    echo "Mode: QUICK (384×384, 2 feat/grid)"
    sed -i '' 's/image_width: 512/image_width: 384/' "$TEMP_CONFIG"
    sed -i '' 's/image_height: 512/image_height: 384/' "$TEMP_CONFIG"
    sed -i '' 's/max_features_per_grid: 1/max_features_per_grid: 2/' "$TEMP_CONFIG"
    ;;
  --ultra)
    # 256×256, 1 feature per grid = ~4x faster
    echo "Mode: ULTRA (256×256, 1 feat/grid)"
    sed -i '' 's/image_width: 512/image_width: 256/' "$TEMP_CONFIG"
    sed -i '' 's/image_height: 512/image_height: 256/' "$TEMP_CONFIG"
    sed -i '' 's/max_features_per_grid: 1/max_features_per_grid: 1/' "$TEMP_CONFIG"
    ;;
  *)
    echo "Unknown mode: $MODE"
    usage
    exit 1
    ;;
esac

# Run VIO
echo ""
echo "Running VIO..."
rm -f "$OUTPUT_EST"

time "$BINARY" "$TEMP_CONFIG" "$DATASET" || {
  echo "VIO failed"
  exit 1
}

if [ ! -f "$OUTPUT_EST" ]; then
  echo "No trajectory generated"
  exit 1
fi

# Evaluate
echo ""
echo "Evaluating..."
python3 /Users/vincent/Work/RS-VIO/scripts/eval_ate_rpe.py \
  --est "$OUTPUT_EST" \
  --gt "$OUTPUT_GT" 2>&1 | grep -E "Sim\(3\)|RMSE|Path length" | head -5

echo ""
echo "================================================================================"
