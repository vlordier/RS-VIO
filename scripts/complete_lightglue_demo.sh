#!/bin/bash
# Complete RS-VIO + LightGlue demonstration with real datasets

set -e

echo "🚀 RS-VIO + LightGlue: Complete Dataset Workflow Demo"
echo "====================================================="

# Check prerequisites
echo "🔧 Checking prerequisites..."

if ! command -v curl &>/dev/null; then
    echo "❌ curl not found. Please install curl."
    exit 1
fi

if ! command -v unzip &>/dev/null; then
    echo "❌ unzip not found. Please install unzip."
    exit 1
fi

echo "✅ Prerequisites OK"

# Check LightGlue setup
echo ""
echo "🎯 Checking LightGlue setup..."

if [ ! -f "models/lightglue_superpoint.onnx" ]; then
    echo "⚠️  LightGlue model not found. Downloading..."
    ./scripts/setup_lightglue.sh
else
    echo "✅ LightGlue model found"
fi

# Build binaries
echo ""
echo "🔨 Building VIO binaries with LightGlue..."
if ! cargo build --features lightglue --release --quiet; then
    echo "❌ Build failed"
    exit 1
fi
echo "✅ Build successful"

# Show available datasets
echo ""
echo "📦 Available Datasets (use ./scripts/setup-datasets.sh to download):"
echo "  • EuRoC MH_01_easy (recommended for first tests)"
echo "  • TUM-VI room1 (indoor sequence)"
echo "  • 4Seasons parking_garage_3_train (outdoor sequence)"
echo ""

# Show dataset sources with correct URLs
echo "🔗 Dataset Download Links:"
echo "  • EuRoC: https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets"
echo "  • TUM-VI: https://vision.in.tum.de/data/datasets/visual-inertial-dataset"
echo "  • 4Seasons: https://vision.cs.tum.edu/4seasons/"
echo ""

# Show example commands
echo "🚀 Example Commands (after downloading datasets):"
echo ""
echo "# EuRoC with LightGlue"
echo "./target/release/run_euroc \\"
echo "  --config-file config/euroc_vio.yaml \\"
echo "  --dataset-path /path/to/euroc/MH_01_easy"
echo ""
echo "# TUM-VI with LightGlue"
echo "./target/release/run_tum \\"
echo "  --config-file config/tum_vi.yaml \\"
echo "  --dataset-path /path/to/tumvi/dataset-room1_512_16"
echo ""
echo "# 4Seasons with LightGlue"
echo "./target/release/run_4seasons \\"
echo "  --config-file config/4seasons.yaml \\"
echo "  --dataset-path /path/to/4seasons/recording_2021-05-10_19-15-19"
echo ""

echo "💡 LightGlue Integration Features:"
echo "  • Deep learning-based feature matching"
echo "  • Robust to lighting changes and motion blur"
echo "  • Better loop closure recall"
echo "  • Hybrid ORB + LightGlue approach"
echo "  • Automatic fallback when needed"
echo ""

echo "🎉 Ready to run VIO with LightGlue-enhanced loop closure!"
echo "   Download a dataset and run the commands above."
