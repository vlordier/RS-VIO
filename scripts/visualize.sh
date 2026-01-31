#!/usr/bin/env bash
# VIO Optimization Visualization Script
# Automates generation of comparison plots for Phase 2 optimizations

set -euo pipefail

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
DATASET_DIR="${DATASET_DIR:-/tmp/rs-vio-samples}"
TUM_VI_PATH="${DATASET_DIR}/tum_vi/room1"

print_header() {
    echo -e "${BLUE}═══════════════════════════════════════════════════════════${NC}"
    echo -e "${BLUE}$1${NC}"
    echo -e "${BLUE}═══════════════════════════════════════════════════════════${NC}"
}

print_success() {
    echo -e "${GREEN}✓ $1${NC}"
}

print_error() {
    echo -e "${RED}✗ $1${NC}"
}

print_warning() {
    echo -e "${YELLOW}⚠ $1${NC}"
}

print_info() {
    echo -e "${BLUE}ℹ $1${NC}"
}

check_command() {
    if ! command -v "$1" &> /dev/null; then
        print_error "$1 not found"
        return 1
    fi
    return 0
}

check_python_deps() {
    print_info "Checking Python dependencies..."

    if ! python3 -c "import pandas, matplotlib, numpy" 2>/dev/null; then
        print_warning "Python dependencies not installed"
        echo ""
        echo "Installing pandas, matplotlib, numpy..."

        if command -v pip3 &> /dev/null; then
            pip3 install pandas matplotlib numpy
        elif command -v pip &> /dev/null; then
            pip install pandas matplotlib numpy
        else
            print_error "pip not found. Please install Python 3"
            exit 1
        fi
    fi

    print_success "Python dependencies ready"
}

generate_demo() {
    print_header "Generating Synthetic Demo Visualization"

    echo "Running synthetic example..."
    cargo run --example plot_vio_comparisons

    print_success "Demo data generated in ./plot_output/"
}

generate_tum_vi() {
    print_header "Generating TUM-VI Dataset Visualization"

    if [ ! -d "$TUM_VI_PATH" ]; then
        print_error "TUM-VI dataset not found at $TUM_VI_PATH"
        echo ""
        echo "Please download TUM-VI dataset:"
        echo "  https://vision.in.tum.de/data/datasets/visual-inertial-dataset"
        echo ""
        echo "Or set DATASET_DIR environment variable:"
        echo "  export DATASET_DIR=/path/to/datasets"
        exit 1
    fi

    echo "Processing TUM-VI room1..."
    cargo run --example plot_tum_vi_comparison -- "$TUM_VI_PATH"

    print_success "TUM-VI data generated in ./tum_vi_results/"
}

generate_plots() {
    local output_dir="$1"
    local name="$2"

    if [ ! -f "$output_dir/plot_comparisons.py" ]; then
        print_error "No plotting script found in $output_dir"
        echo "Run visualization generation first"
        return 1
    fi

    echo "Generating $name plots..."
    cd "$output_dir" && python3 plot_comparisons.py
    cd - > /dev/null

    # Check if plots were generated
    if ls "$output_dir"/*.png &> /dev/null; then
        print_success "Plots generated:"
        ls -lh "$output_dir"/*.png
    else
        print_warning "No PNG files found (check Python output for errors)"
    fi
}

show_results() {
    print_header "Visualization Results"

    echo ""
    echo -e "${GREEN}Synthetic Demo:${NC}"
    if [ -d "./plot_output" ]; then
        echo "  Data: ./plot_output/*.csv"
        if ls ./plot_output/*.png &> /dev/null; then
            echo "  Plots: ./plot_output/*.png"
        else
            echo "  Plots: Not generated (run with --plots)"
        fi
    else
        echo "  Not generated"
    fi

    echo ""
    echo -e "${GREEN}TUM-VI Dataset:${NC}"
    if [ -d "./tum_vi_results" ]; then
        echo "  Data: ./tum_vi_results/*.csv"
        if ls ./tum_vi_results/*.png &> /dev/null; then
            echo "  Plots: ./tum_vi_results/*.png"
        else
            echo "  Plots: Not generated (run with --plots)"
        fi
    else
        echo "  Not generated"
    fi

    echo ""
}

usage() {
    cat << EOF
Usage: $0 [OPTIONS]

Generate VIO optimization comparison visualizations.

OPTIONS:
    --demo              Generate synthetic demo only
    --tum-vi            Generate TUM-VI dataset visualization only
    --plots             Generate plots from existing data
    --all               Generate everything (demo + TUM-VI + plots)
    --install-deps      Install Python dependencies only
    --clean             Clean generated files
    --help              Show this help message

EXAMPLES:
    $0 --all                    # Generate everything
    $0 --demo --plots           # Demo data + plots
    $0 --tum-vi --plots         # TUM-VI data + plots
    $0 --install-deps           # Install Python deps

ENVIRONMENT:
    DATASET_DIR         Path to datasets (default: /tmp/rs-vio-samples)

OUTPUT:
    ./plot_output/              Synthetic demo results
    ./tum_vi_results/           TUM-VI dataset results
EOF
}

main() {
    local do_demo=false
    local do_tum=false
    local do_plots=false
    local do_install=false
    local do_clean=false
    local do_all=false

    # Parse arguments
    if [ $# -eq 0 ]; then
        usage
        exit 0
    fi

    while [ $# -gt 0 ]; do
        case "$1" in
            --demo)
                do_demo=true
                ;;
            --tum-vi)
                do_tum=true
                ;;
            --plots)
                do_plots=true
                ;;
            --all)
                do_all=true
                ;;
            --install-deps)
                do_install=true
                ;;
            --clean)
                do_clean=true
                ;;
            --help|-h)
                usage
                exit 0
                ;;
            *)
                print_error "Unknown option: $1"
                usage
                exit 1
                ;;
        esac
        shift
    done

    # Clean if requested
    if [ "$do_clean" = true ]; then
        print_header "Cleaning Generated Files"
        rm -rf ./plot_output ./tum_vi_results
        print_success "Cleaned"
        exit 0
    fi

    # Install dependencies if requested
    if [ "$do_install" = true ]; then
        check_python_deps
        exit 0
    fi

    # Check prerequisites
    check_command cargo || exit 1
    check_command python3 || exit 1

    # Handle --all flag
    if [ "$do_all" = true ]; then
        do_demo=true
        do_tum=true
        do_plots=true
    fi

    # Check Python deps if we're going to plot
    if [ "$do_plots" = true ]; then
        check_python_deps
    fi

    # Generate demo
    if [ "$do_demo" = true ]; then
        generate_demo
        echo ""
    fi

    # Generate TUM-VI
    if [ "$do_tum" = true ]; then
        generate_tum_vi
        echo ""
    fi

    # Generate plots
    if [ "$do_plots" = true ]; then
        print_header "Generating Plots"

        if [ "$do_demo" = true ] || [ -d "./plot_output" ]; then
            echo ""
            generate_plots "./plot_output" "demo"
        fi

        if [ "$do_tum" = true ] || [ -d "./tum_vi_results" ]; then
            echo ""
            generate_plots "./tum_vi_results" "TUM-VI"
        fi

        echo ""
    fi

    # Show results
    show_results

    print_header "Complete"

    if [ "$do_plots" = false ]; then
        echo ""
        print_info "To generate plots, run:"
        echo "  $0 --plots"
        echo ""
        echo "Or manually:"
        echo "  python3 ./plot_output/plot_comparisons.py"
        echo "  python3 ./tum_vi_results/plot_comparisons.py"
    fi
}

main "$@"
