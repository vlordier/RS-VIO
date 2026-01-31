#!/bin/bash
# Generate teacher training data from TUM-VI dataset
# Usage: ./tools/generate_teacher_dataset.sh /path/to/tum-vi-root output-dir

set -euo pipefail

# Configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
CARGO_FEATURE="export-teacher"

# Command line arguments
TUM_VI_ROOT="${1:-.}"
OUTPUT_DIR="${2:./teacher_data}"

# TUM-VI sequences to process
SEQUENCES=("room1" "room2" "room3" "room4" "room5" "room6")

# Check if TUM-VI dataset exists
if [ ! -d "$TUM_VI_ROOT" ]; then
    echo "❌ TUM-VI root directory not found: $TUM_VI_ROOT"
    echo "Usage: $0 /path/to/tum-vi-root [output-dir]"
    exit 1
fi

# Create output directory
mkdir -p "$OUTPUT_DIR"
echo "📁 Output directory: $OUTPUT_DIR"

# Build export binary
echo "🔨 Building export binary..."
cd "$REPO_ROOT"
cargo build --bin export_teacher --features "$CARGO_FEATURE" --release 2>&1 | grep -E "error|warning:|Finished" || true
if [ ! -f "target/release/export_teacher" ]; then
    echo "❌ Failed to build export_teacher binary"
    exit 1
fi
echo "✅ Build successful"

EXPORT_BINARY="$REPO_ROOT/target/release/export_teacher"

# Process each sequence
SUCCESSFUL=0
FAILED=0

for seq in "${SEQUENCES[@]}"; do
    SEQ_PATH="$TUM_VI_ROOT/$seq"

    if [ ! -d "$SEQ_PATH" ]; then
        echo "⚠️  Skipping $seq - directory not found"
        continue
    fi

    echo ""
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "Processing: $seq"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

    # Create sequence-specific output directory
    SEQ_OUTPUT="$OUTPUT_DIR/$seq"
    mkdir -p "$SEQ_OUTPUT"

    # Run export with proper configuration
    echo "🚀 Starting export..."
    if "$EXPORT_BINARY" \
        --config "$REPO_ROOT/config/teacher_tumvi_export.yaml" \
        --dataset-path "$SEQ_PATH" \
        --output-dir "$SEQ_OUTPUT" \
        --sequence-name "$seq" \
        2>&1 | tee "$SEQ_OUTPUT/export.log"; then

        echo "✅ Export successful for $seq"
        ((SUCCESSFUL++))

        # Validate exported data
        echo "🔍 Validating export..."
        if command -v python3 &> /dev/null; then
            python3 "$REPO_ROOT/tools/validate_export.py" "$SEQ_OUTPUT/$seq.h5" --verbose || true
        fi

    else
        echo "❌ Export failed for $seq"
        ((FAILED++))
    fi
done

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "SUMMARY"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Successful: $SUCCESSFUL"
echo "Failed:     $FAILED"
echo "Output:     $OUTPUT_DIR"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# Exit with failure if any exports failed
[ $FAILED -eq 0 ] && exit 0 || exit 1
