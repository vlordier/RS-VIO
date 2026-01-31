#!/bin/bash
# Comprehensive demo: Running RS-VIO with LightGlue on real datasets

set -e

echo "🚀 RS-VIO + LightGlue: Full Pipeline Demo"
echo "=========================================="

# Check if LightGlue model exists
if [ ! -f "models/lightglue_superpoint.onnx" ]; then
    echo "❌ LightGlue model not found. Run setup script first:"
    echo "   ./scripts/setup_lightglue.sh"
    exit 1
fi

echo "✅ LightGlue model found: $(ls -lh models/lightglue_superpoint.onnx)"

# Check if compiled with LightGlue
echo ""
echo "🔧 Checking LightGlue compilation..."
if ! cargo build --features lightglue --release --quiet 2>/dev/null; then
    echo "❌ Compilation failed. Check dependencies."
    exit 1
fi

echo "✅ Compilation successful with LightGlue support"

# Show available binaries
echo ""
echo "📦 Available VIO binaries:"
ls -la target/release/run_* 2>/dev/null || echo "   Binaries not built yet"

echo ""
echo "🎯 Dataset Sources:"
echo "  • EuRoC: https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets"
echo "  • TUM-VI: https://vision.in.tum.de/data/datasets/visual-inertial-dataset"
echo "  • 4Seasons: https://vision.cs.tum.edu/4seasons/"

echo ""
echo "🚀 Ready to run commands:"
echo ""
echo "  # EuRoC MAV dataset"
echo "  ./target/release/run_euroc \\"
echo "    --config-file config/euroc_vio.yaml \\"
echo "    --dataset-path /path/to/euroc/MH_01_easy"
echo ""
echo "  # TUM-VI dataset"
echo "  ./target/release/run_tum \\"
echo "    --config-file config/tum_vi.yaml \\"
echo "    --dataset-path /path/to/tumvi/dataset-room1_512_16"
echo ""
echo "  # 4Seasons dataset"
echo "  ./target/release/run_4seasons \\"
echo "    --config-file config/4seasons.yaml \\"
echo "    --dataset-path /path/to/4seasons/office_0"

echo ""
echo "💡 LightGlue Integration Details:"
echo "  • Loop closure detection uses both ORB (fast) + LightGlue (accurate)"
echo "  • LightGlue provides deep learning feature matching"
echo "  • Robust to appearance changes, lighting, motion blur"
echo "  • Automatic fallback if geometric verification fails"
echo "  • ~500MB memory overhead, ~10-50ms per matching operation"

echo ""
echo "🎉 Setup complete! Download a dataset and run VIO with LightGlue-enhanced loop closure!"
