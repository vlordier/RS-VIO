#!/bin/bash
# Auto-generated script to export ground truth data for VIO training
# This runs RS-VIO export_teacher on multiple datasets

set -e  # Exit on error

WORK_DIR="/Users/vincent/Work/RS-VIO"
cd "$WORK_DIR"

echo "======================================================================="
echo "GROUND TRUTH EXPORT FOR VIO TRAINING"
echo "======================================================================="
echo "RS-VIO binary: $WORK_DIR/target/release/export_teacher"
echo "Output directory: $WORK_DIR/data/ground_truth"
echo ""

# Create output directory
mkdir -p data/ground_truth

# Function to export a dataset
export_dataset() {
    local dataset_path=$1
    local output_name=$2
    local max_frames=$3

    echo ""
    echo "======================================================================="
    echo "Exporting: $output_name"
    echo "Dataset: $dataset_path"
    echo "Max frames: $max_frames"
    echo "======================================================================="

    ./target/release/export_teacher \
      --dataset-path "$dataset_path" \
      --output-dir "data/ground_truth/$output_name" \
      --max-frames "$max_frames" \
      --config config/teacher_tumvi_export.yaml

    if [ $? -eq 0 ]; then
        echo "✅ $output_name completed"
    else
        echo "❌ $output_name failed"
        return 1
    fi
}

# Export TUM-VI datasets
export_dataset "datasets/tum_vi/room1" "tumvi_room1" 1500
export_dataset "datasets/tum_vi/dataset-magistrale1_512_16" "tumvi_magistrale1" 1000

# Export EuRoC datasets
export_dataset "datasets/euroc/MH_01_easy" "euroc_mh01_easy" 1000
export_dataset "datasets/euroc/MH_02_easy" "euroc_mh02_easy" 800
export_dataset "datasets/euroc/MH_03_medium" "euroc_mh03_medium" 800

echo ""
echo "======================================================================="
echo "✅ ALL EXPORTS COMPLETE!"
echo "======================================================================="
echo "Output directory: $WORK_DIR/data/ground_truth"
echo ""
echo "Generated datasets:"
ls -lh data/ground_truth/
echo ""
echo "Next steps:"
echo "  1. Return to Jupyter notebook"
echo "  2. Run Cell 15 (dataloader creation) - will detect real data"
echo "  3. Continue with training cells"
echo ""
echo "💡 The training will automatically use real data instead of synthetic"
