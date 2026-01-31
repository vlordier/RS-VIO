#!/bin/bash
# Export multiple datasets for multi-domain training

set -e

DATASETS_ROOT="/Users/vincent/Work/RS-VIO/datasets"
OUTPUT_ROOT="/Users/vincent/Work/RS-VIO/exported_data_multi"

echo "======================================================================"
echo "MULTI-DATASET EXPORT FOR DOMAIN-ROBUST TRAINING"
echo "======================================================================"
echo ""

# Create output directory
mkdir -p "$OUTPUT_ROOT"

# Counter
total_exported=0

# Function to export a dataset
export_dataset() {
    local dataset_path="$1"
    local dataset_name="$2"
    local max_frames="${3:-1000}"  # Default 1000 frames per dataset

    echo "----------------------------------------------------------------------"
    echo "Exporting: $dataset_name"
    echo "Path: $dataset_path"
    echo "Max frames: $max_frames"
    echo "----------------------------------------------------------------------"

    if [ ! -d "$dataset_path" ]; then
        echo "⚠️  Dataset not found: $dataset_path"
        echo "   Skipping..."
        return
    fi

    # Create dataset-specific output directory
    output_dir="$OUTPUT_ROOT/$dataset_name"
    mkdir -p "$output_dir"

    # Run export
    cargo run --release --bin export_teacher --features export-teacher -- \
        --dataset-path "$dataset_path" \
        --output-dir "$output_dir" \
        --max-frames "$max_frames"

    if [ $? -eq 0 ]; then
        echo "✅ Successfully exported $dataset_name"

        # Count frames
        if [ -f "$output_dir/metadata.json" ]; then
            frame_count=$(python3 -c "import json; print(len(json.load(open('$output_dir/metadata.json'))))")
            echo "   Frames: $frame_count"
            total_exported=$((total_exported + frame_count))
        fi
    else
        echo "❌ Failed to export $dataset_name"
    fi

    echo ""
}

# Export TUM-VI datasets
echo "📦 TUM-VI Datasets"
export_dataset "$DATASETS_ROOT/tum_vi/room1" "tumvi_room1" 1500
export_dataset "$DATASETS_ROOT/tum_vi/magistrale1" "tumvi_magistrale1" 1000

# Export EuRoC datasets
echo "📦 EuRoC MAV Datasets"
export_dataset "$DATASETS_ROOT/euroc/MH_01_easy" "euroc_mh01_easy" 1000
export_dataset "$DATASETS_ROOT/euroc/MH_02_easy" "euroc_mh02_easy" 800
export_dataset "$DATASETS_ROOT/euroc/MH_03_medium" "euroc_mh03_medium" 800
export_dataset "$DATASETS_ROOT/euroc/MH_04_difficult" "euroc_mh04_diff" 600

# Export 4Seasons dataset
echo "📦 4Seasons Dataset"
export_dataset "$DATASETS_ROOT/4seasons/recording_2021-05-10_19-15-19" "4seasons_outdoor" 1000

echo "======================================================================"
echo "EXPORT SUMMARY"
echo "======================================================================"
echo "Total frames exported: $total_exported"
echo "Output directory: $OUTPUT_ROOT"
echo ""
echo "Next steps:"
echo "  1. Verify exports: ls -lh $OUTPUT_ROOT/*/metadata.json"
echo "  2. Train multi-domain: python3 train_multi_domain_refinement.py \\"
echo "       --datasets tumvi_room1 tumvi_magistrale1 euroc_mh01_easy \\"
echo "       --epochs 30"
echo "======================================================================"
