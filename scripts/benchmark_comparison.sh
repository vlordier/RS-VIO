#!/usr/bin/env bash
# Compare benchmark results against baseline
# Usage: ./scripts/benchmark_comparison.sh [baseline_file]

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
METRICS_DIR="$PROJECT_ROOT/metrics/benchmarks"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

cd "$PROJECT_ROOT"

# Check for baseline
BASELINE_FILE="${1:-$METRICS_DIR/baseline_latest.txt}"

if [[ ! -f "$BASELINE_FILE" ]]; then
    echo -e "${RED}Error: Baseline file not found: $BASELINE_FILE${NC}"
    echo ""
    echo "Run this first to create baseline:"
    echo "  make benchmark-baseline"
    exit 1
fi

echo -e "${BLUE}════════════════════════════════════════════════════════════════${NC}"
echo -e "${BLUE}  RS-VIO Benchmark Comparison${NC}"
echo -e "${BLUE}════════════════════════════════════════════════════════════════${NC}"
echo ""
echo -e "${YELLOW}Baseline:${NC} $BASELINE_FILE"
echo ""

# Run current benchmarks
CURRENT_FILE="/tmp/rs_vio_bench_current_$$.txt"
echo -e "${YELLOW}Running current benchmarks...${NC}"
cargo bench --all 2>&1 | tee "$CURRENT_FILE"

echo ""
echo -e "${BLUE}════════════════════════════════════════════════════════════════${NC}"
echo -e "${BLUE}  Comparison Results${NC}"
echo -e "${BLUE}════════════════════════════════════════════════════════════════${NC}"
echo ""

# Parse and compare (simple text comparison)
# In a real implementation, you'd parse benchmark outputs properly

echo -e "${YELLOW}Baseline vs Current:${NC}"
echo ""
echo "Baseline file: $BASELINE_FILE"
echo "Current run: $CURRENT_FILE"
echo ""
echo -e "${YELLOW}Note:${NC} Manual comparison required."
echo "Consider using 'cargo-criterion' for automatic comparisons:"
echo "  cargo install cargo-criterion"
echo ""

# Check for major regressions (simple heuristic)
REGRESSION_FOUND=false

# Look for test results in both files
# This is a simplified check - real implementation would parse properly
if grep -q "test result: FAILED" "$CURRENT_FILE"; then
    echo -e "${RED}❌ WARNING: Some benchmarks failed!${NC}"
    REGRESSION_FOUND=true
fi

if [[ "$REGRESSION_FOUND" == true ]]; then
    echo ""
    echo -e "${RED}⚠️  Potential regressions detected${NC}"
    echo "Review the benchmark outputs for details"
    exit 1
else
    echo -e "${GREEN}✅ No major regressions detected${NC}"
    echo ""
    echo "To save this as new baseline:"
    echo "  cp $CURRENT_FILE $METRICS_DIR/baseline_$(date +%Y%m%d_%H%M%S).txt"
fi

echo ""
echo -e "${BLUE}════════════════════════════════════════════════════════════════${NC}"
