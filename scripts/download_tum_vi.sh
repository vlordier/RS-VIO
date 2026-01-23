#!/bin/bash
# Download TUM-VI dataset for real-world VIO validation
# Dataset: https://vision.in.tum.de/data/datasets/visual-inertial-dataset

set -e

DATASET_DIR="${DATASET_DIR:-./data/tum_vi}"
DATASET_BASE_URL="https://vision.in.tum.de/tumvi/exported/euroc/512_16"

# Color output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}TUM-VI Dataset Downloader for RS-VIO${NC}"
echo "======================================"
echo ""

# Create directory structure
mkdir -p "$DATASET_DIR"
cd "$DATASET_DIR"

# TUM-VI sequences to download (room1-room6)
SEQUENCES=(
    "dataset-room1_512_16"
    "dataset-room2_512_16"
    "dataset-room3_512_16"
    "dataset-room4_512_16"
    "dataset-room5_512_16"
    "dataset-room6_512_16"
)

download_sequence() {
    local seq=$1
    local url="${DATASET_BASE_URL}/${seq}/${seq}.tar"
    
    echo -e "${YELLOW}Downloading: ${seq}${NC}"
    
    # Check if already downloaded
    if [ -d "$seq" ]; then
        echo -e "${GREEN}✓ Already exists: $seq${NC}"
        return 0
    fi
    
    # Download
    if command -v wget &> /dev/null; then
        wget -q --show-progress "$url" -O "${seq}.tar"
    elif command -v curl &> /dev/null; then
        curl -L --progress-bar "$url" -o "${seq}.tar"
    else
        echo -e "${RED}Error: Neither wget nor curl found${NC}"
        exit 1
    fi
    
    # Extract
    echo -e "${YELLOW}Extracting: ${seq}${NC}"
    tar -xf "${seq}.tar"
    rm "${seq}.tar"
    
    echo -e "${GREEN}✓ Downloaded: ${seq}${NC}"
}

# Download all sequences
for seq in "${SEQUENCES[@]}"; do
    download_sequence "$seq"
done

echo ""
echo -e "${GREEN}======================================"
echo "Download complete!"
echo "======================================"
echo ""
echo "Dataset location: $(pwd)"
echo "Sequences: ${#SEQUENCES[@]}"
echo ""
echo "Directory structure:"
tree -L 2 -d . 2>/dev/null || ls -la

echo ""
echo -e "${YELLOW}Next steps:${NC}"
echo "1. Run benchmarks: cargo bench --bench tum_vi_real_pipeline"
echo "2. View results: cat benches/tum_vi_results.txt"
echo ""
