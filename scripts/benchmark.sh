#!/bin/bash
# Benchmark automation script
# Runs benchmarks and generates plots for analysis

set -euo pipefail

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
BASELINE_NAME=""
SAVE_BASELINE=false
OUTPUT_DIR="target/benchmark_results"
PLOT_DIR="target/benchmark_plots"
BENCHES=("estimator" "optimization" "feature_tracker" "pipeline")

# Functions
print_header() {
    echo -e "${BLUE}=== $1 ===${NC}"
}

print_success() {
    echo -e "${GREEN}✓ $1${NC}"
}

print_error() {
    echo -e "${RED}✗ $1${NC}"
}

print_info() {
    echo -e "${YELLOW}→ $1${NC}"
}

show_help() {
    cat << EOF
Usage: ./scripts/benchmark.sh [OPTIONS]

Options:
    --baseline NAME         Save/compare against baseline NAME
    --save                  Save baseline instead of comparing
    --all                   Run all benchmarks (default)
    --bench NAME            Run specific benchmark (estimator, optimization, etc.)
    --output-dir DIR        Output directory for results
    --plot-dir DIR          Output directory for plots
    --profile PROFILE       Build profile (release, release-with-debug)
    --help                  Show this help message

Examples:
    # Save baseline for current version
    ./scripts/benchmark.sh --baseline v0.2.0 --save

    # Run benchmarks and compare against baseline
    ./scripts/benchmark.sh --baseline v0.2.0

    # Run only estimator benchmarks
    ./scripts/benchmark.sh --bench estimator

    # Run all benchmarks and generate plots
    ./scripts/benchmark.sh --all

EOF
}

# Parse arguments
PROFILE="release"
while [[ $# -gt 0 ]]; do
    case $1 in
        --baseline)
            BASELINE_NAME="$2"
            shift 2
            ;;
        --save)
            SAVE_BASELINE=true
            shift
            ;;
        --bench)
            BENCHES=("$2")
            shift 2
            ;;
        --output-dir)
            OUTPUT_DIR="$2"
            shift 2
            ;;
        --plot-dir)
            PLOT_DIR="$2"
            shift 2
            ;;
        --profile)
            PROFILE="$2"
            shift 2
            ;;
        --help)
            show_help
            exit 0
            ;;
        *)
            print_error "Unknown option: $1"
            show_help
            exit 1
            ;;
    esac
done

# Create output directories
mkdir -p "$OUTPUT_DIR" "$PLOT_DIR"

# Validate profile
case "$PROFILE" in
    release|release-with-debug)
        ;;
    *)
        print_error "Invalid profile: $PROFILE"
        exit 1
        ;;
esac

print_header "RS-VIO Benchmark Suite"
print_info "Profile: $PROFILE"
print_info "Benchmarks: ${BENCHES[*]}"

# Run benchmarks
print_header "Running Benchmarks"
for bench in "${BENCHES[@]}"; do
    print_info "Running $bench..."

    if [ -n "$BASELINE_NAME" ] && [ "$SAVE_BASELINE" = false ]; then
        # Compare against baseline
        cargo bench --profile "$PROFILE" --bench "$bench" \
            -- --baseline "$BASELINE_NAME" \
            || print_error "Benchmark $bench failed"
    elif [ "$SAVE_BASELINE" = true ] && [ -n "$BASELINE_NAME" ]; then
        # Save new baseline
        cargo bench --profile "$PROFILE" --bench "$bench" \
            -- --save-baseline "$BASELINE_NAME" \
            || print_error "Benchmark $bench failed"
    else
        # Run without baseline comparison
        cargo bench --profile "$PROFILE" --bench "$bench" \
            || print_error "Benchmark $bench failed"
    fi

    print_success "$bench completed"
done

# Generate plots
print_header "Generating Plots"
if command -v python3 &> /dev/null; then
    if python3 -c "import matplotlib" 2>/dev/null; then
        print_info "Generating benchmark plots..."
        python3 scripts/plot_benchmarks.py \
            --output-dir "$PLOT_DIR" \
            --plot-types all \
            || print_error "Plot generation failed"
        print_success "Plots generated in $PLOT_DIR"
    else
        print_error "matplotlib not installed"
        print_info "Install with: pip install matplotlib"
    fi
else
    print_error "python3 not found, skipping plots"
fi

# Summary
print_header "Summary"
print_success "Benchmarks completed"
print_info "Results: target/criterion/"
print_info "Plots: $PLOT_DIR"

if [ "$SAVE_BASELINE" = true ] && [ -n "$BASELINE_NAME" ]; then
    print_success "Baseline saved: $BASELINE_NAME"
    echo ""
    echo "To compare future runs:"
    echo "  ./scripts/benchmark.sh --baseline $BASELINE_NAME"
fi

echo ""
print_info "View results with:"
echo "  open $PLOT_DIR/benchmark_comparison.png"
echo "  open $PLOT_DIR/resolution_scaling.png"
echo "  open $PLOT_DIR/grid_scaling.png"
