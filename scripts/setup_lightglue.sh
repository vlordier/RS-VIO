#!/bin/bash
# Script to download and test LightGlue ONNX models

set -e

echo "LightGlue Model Setup Script"
echo "============================"

# Create models directory
mkdir -p models

# Download LightGlue SuperPoint model (v2.0 with dynamic batch support)
MODEL_URL="https://github.com/fabio-sim/LightGlue-ONNX/releases/download/v2.0/superpoint_lightglue_pipeline.ort.onnx"
MODEL_PATH="models/lightglue_superpoint.onnx"

echo "Available LightGlue Models (v2.0 - Dynamic Batch Support):"
echo "  1. superpoint_lightglue_pipeline.ort.onnx (ONNX Runtime CPU/CUDA - RECOMMENDED)"
echo "  2. superpoint_lightglue_pipeline.onnx (ONNX-only)"
echo "  3. disk_lightglue_pipeline.ort.onnx (DISK extractor instead of SuperPoint)"
echo "  4. *_lightglue_pipeline.trt.onnx (TensorRT optimized)"
echo ""

if [ ! -f "$MODEL_PATH" ]; then
    echo "Downloading LightGlue SuperPoint fused model..."
    echo "URL: $MODEL_URL"
    echo "Destination: $MODEL_PATH"
    echo ""

    if command -v curl &> /dev/null; then
        curl -L -o "$MODEL_PATH" "$MODEL_URL"
    elif command -v wget &> /dev/null; then
        wget -O "$MODEL_PATH" "$MODEL_URL"
    else
        echo "Error: Neither curl nor wget found. Please install one of them."
        exit 1
    fi

    if [ -f "$MODEL_PATH" ]; then
        echo "✅ Model downloaded successfully to $MODEL_PATH"
        ls -lh "$MODEL_PATH"
    else
        echo "❌ Download failed"
        exit 1
    fi
else
    echo "✅ Model already exists at $MODEL_PATH"
    ls -lh "$MODEL_PATH"
fi

# Test the model loading
echo ""
echo "Testing LightGlue model loading..."
if cargo test test_lightglue_model_loading --features lightglue -- --nocapture --quiet; then
    echo "✅ LightGlue model loading test PASSED"
else
    echo "⚠️  LightGlue model loading test failed (expected if no model weights)"
fi

echo ""
echo "========================================"
echo "🎉 LightGlue setup complete!"
echo "========================================"
echo ""
echo "📦 Available Datasets & Binaries:"
echo "  • EuRoC: ./target/release/run_euroc --config-file config/euroc_vio.yaml --dataset-path /path/to/euroc/MH_01_easy"
echo "  • TUM-VI: ./target/release/run_tum --config-file config/tum_vi.yaml --dataset-path /path/to/tumvi/dataset-room1_512_16"
echo "  • 4Seasons: ./target/release/run_4seasons --config-file config/4seasons.yaml --dataset-path /path/to/4seasons/office_0"
echo ""
echo "🚀 Usage Examples:"
echo "  # Compile with LightGlue support"
echo "  cargo build --features lightglue --release"
echo ""
echo "  # Run EuRoC VIO with LightGlue-enabled loop closure"
echo "  ./target/release/run_euroc --config-file config/euroc_vio.yaml --dataset-path /path/to/euroc/MH_01_easy"
echo ""
echo "  # Run TUM-VI with LightGlue"
echo "  ./target/release/run_tum --config-file config/tum_vi.yaml --dataset-path /path/to/tumvi/dataset-room1_512_16"
echo ""
echo "  # Run 4Seasons with LightGlue"
echo "  ./target/release/run_4seasons --config-file config/4seasons.yaml --dataset-path /path/to/4seasons/office_0"
echo ""
echo "💡 Note: LightGlue will automatically be used for loop closure detection"
echo "   when the feature is enabled and model weights are available."
echo ""
echo "📦 Dataset Download: Use ./scripts/setup-datasets.sh to download datasets"