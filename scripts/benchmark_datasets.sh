#!/bin/bash
# Comprehensive Real-World Dataset Benchmarks
# Tests performance on EuRoC, TUM-VI, and 4Seasons datasets

set -e

WORKSPACE_DIR="/Users/vincent/Work/RS-VIO"
RESULTS_FILE="${WORKSPACE_DIR}/REAL_DATASET_BENCHMARKS.md"

# Color codes
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}Real-World Dataset Performance Benchmarks${NC}"
echo -e "${BLUE}========================================${NC}\n"

# Initialize results file
cat > "$RESULTS_FILE" << 'EOF'
# Real-World Dataset Benchmarks

## Summary

Comprehensive performance benchmarks on real VIO datasets comparing sequential vs parallel feature tracking.

---

EOF

# Function to run a single benchmark
run_benchmark() {
    local dataset_name=$1
    local dataset_path=$2
    local config_path=$3
    
    echo -e "${YELLOW}Benchmarking: ${dataset_name}${NC}"
    echo "Dataset: ${dataset_path}" | tee -a "$RESULTS_FILE"
    
    # Run the benchmark with timing
    start_time=$(date +%s%N)
    output=$("$WORKSPACE_DIR/target/release/run_euroc" "$config_path" "$dataset_path" 2>&1 || true)
    end_time=$(date +%s%N)
    
    # Extract metrics from output
    frames=$(echo "$output" | grep "Total Frames Processed:" | tail -1 | grep -oE '[0-9]+' | head -1)
    avg_time=$(echo "$output" | grep "average" | tail -1 | sed 's/.*average //' | sed 's/ per.*//')
    fps=$(echo "$output" | grep "average" | tail -1 | awk '{print $(NF-2)}' | tr -d 'ms' || echo "N/A")
    
    total_ms=$((($end_time - $start_time) / 1000000))
    total_sec=$(echo "scale=2; $total_ms / 1000" | bc)
    
    echo -e "  Frames: ${frames}"
    echo -e "  Avg time/frame: ${avg_time}"
    echo -e "  Wall clock: ${total_sec}s"
    echo -e "  Details: ${output}" >> "$RESULTS_FILE"
    echo ""
    
    # Append to results
    cat >> "$RESULTS_FILE" << EOF

| Dataset | Frames | Avg Time | Wall Clock |
|---------|--------|----------|------------|
| ${dataset_name} | ${frames} | ${avg_time} | ${total_sec}s |

EOF
}

# Build release binary first
echo -e "${BLUE}Building release binary...${NC}"
cd "$WORKSPACE_DIR"
cargo build --release 2>&1 | tail -3

echo -e "\n${BLUE}Running benchmarks on real datasets...${NC}\n"

# EuRoC Benchmarks
echo -e "${BLUE}[1/5] EuRoC MH_01_easy${NC}"
run_benchmark "EuRoC MH_01_easy" \
    "/tmp/rs-vio-samples/euroc/MH_01_easy" \
    "config/euroc_vio.yaml"

echo -e "${BLUE}[2/5] EuRoC MH_02_easy${NC}"
run_benchmark "EuRoC MH_02_easy" \
    "/tmp/rs-vio-samples/euroc/MH_02_easy" \
    "config/euroc_vio.yaml"

echo -e "${BLUE}[3/5] EuRoC MH_03_medium${NC}"
run_benchmark "EuRoC MH_03_medium" \
    "/tmp/rs-vio-samples/euroc/MH_03_medium" \
    "config/euroc_vio.yaml"

# TUM-VI Benchmarks
echo -e "${BLUE}[4/5] TUM-VI room1${NC}"
run_benchmark "TUM-VI room1" \
    "/tmp/rs-vio-samples/tum_vi/room1" \
    "config/tum_vi.yaml"

# 4Seasons Benchmark
echo -e "${BLUE}[5/5] 4Seasons recording${NC}"
run_benchmark "4Seasons recording_2021-05-10" \
    "/tmp/rs-vio-samples/4seasons/recording_2021-05-10_19-15-19" \
    "config/4seasons.yaml"

echo -e "\n${GREEN}✓ Benchmarks complete!${NC}"
echo -e "${GREEN}Results saved to: ${RESULTS_FILE}${NC}"

# Print summary
echo -e "\n${BLUE}Summary of Results:${NC}"
cat "$RESULTS_FILE"
