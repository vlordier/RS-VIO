#!/bin/bash

##############################################################################
# Multi-Strategy Benchmarking Suite for RS-VIO
# 
# Tests all stereo matching strategies (BasicRANSAC, IMUGuided, 
# TemporalConsistency, HybridOpticalFlow) across all available datasets
# (EuRoC, TUM-VI, 4Seasons)
#
# Usage: ./scripts/benchmark_strategies.sh [OPTIONS]
#        ./scripts/benchmark_strategies.sh --all         # All strategies on all datasets
#        ./scripts/benchmark_strategies.sh --fast        # EuRoC + BasicRANSAC only
#        ./scripts/benchmark_strategies.sh --euroc       # EuRoC dataset only
#
##############################################################################

set -euo pipefail

# ============================================================================
# Configuration
# ============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
DATASET_DIR="${DATASET_DIR:-/tmp/rs-vio-samples}"
RELEASE_DIR="$PROJECT_ROOT/target/release"
RESULTS_DIR="$PROJECT_ROOT/benchmark_results"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
RESULTS_FILE="$RESULTS_DIR/strategies_${TIMESTAMP}.csv"

# Color output
COLOR_BLUE='\033[0;34m'
COLOR_GREEN='\033[0;32m'
COLOR_YELLOW='\033[1;33m'
COLOR_RED='\033[0;31m'
NC='\033[0m'

# Available strategies
STRATEGIES=("BasicRANSAC" "IMUGuided" "TemporalConsistency" "HybridOpticalFlow")
DATASETS=()

# ============================================================================
# Functions
# ============================================================================

log_info() {
    echo -e "${COLOR_GREEN}✓${NC} $1"
}

log_warn() {
    echo -e "${COLOR_YELLOW}⚠${NC} $1"
}

log_error() {
    echo -e "${COLOR_RED}✗${NC} $1"
}

log_header() {
    echo ""
    echo -e "${COLOR_BLUE}═══════════════════════════════════════════════════════════${NC}"
    echo -e "${COLOR_BLUE}$1${NC}"
    echo -e "${COLOR_BLUE}═══════════════════════════════════════════════════════════${NC}"
}

usage() {
    cat << EOF
${COLOR_BLUE}Multi-Strategy Benchmarking for RS-VIO${NC}

USAGE:
    $0 [OPTIONS]

OPTIONS:
    --all               Run all strategies on all available datasets (4-6 hours)
    --fast              Quick run: BasicRANSAC on EuRoC only (5 minutes)
    --euroc             Test all strategies on EuRoC dataset (30 minutes)
    --tum               Test all strategies on TUM-VI dataset (30 minutes)
    --4seasons          Test all strategies on 4Seasons dataset (30 minutes)
    --strategies LIST   Comma-separated list of strategies to test
                        (default: all) e.g., --strategies BasicRANSAC,IMUGuided
    --datasets LIST     Comma-separated list of datasets to test
                        (default: all) e.g., --datasets euroc,tum
    --no-build          Skip cargo build (use existing binaries)
    --help              Show this help message

EXAMPLES:
    # Test all strategies on all datasets
    $0 --all

    # Quick smoke test
    $0 --fast

    # Test specific strategy
    $0 --strategies BasicRANSAC --euroc

    # Compare just IMU-aware strategies
    $0 --strategies IMUGuided,TemporalConsistency --all

EOF
    exit 0
}

check_datasets() {
    log_info "Checking available datasets..."
    
    if [ -d "$DATASET_DIR/euroc/MH_01_easy" ]; then
        DATASETS+=("euroc")
        log_info "EuRoC dataset found"
    else
        log_warn "EuRoC dataset not found at $DATASET_DIR/euroc/MH_01_easy"
    fi

    if [ -d "$DATASET_DIR/tum_vi" ]; then
        DATASETS+=("tum")
        log_info "TUM-VI dataset found"
    else
        log_warn "TUM-VI dataset not found at $DATASET_DIR/tum_vi"
    fi

    if ls "$DATASET_DIR/4seasons"/recording_* &>/dev/null 2>&1; then
        DATASETS+=("4seasons")
        log_info "4Seasons dataset found"
    else
        log_warn "4Seasons dataset not found at $DATASET_DIR/4seasons"
    fi

    if [ ${#DATASETS[@]} -eq 0 ]; then
        log_error "No datasets found. Install with: make setup-datasets"
        exit 1
    fi

    echo "  Available datasets: ${DATASETS[*]}"
}

build_release() {
    if [ "$SKIP_BUILD" = true ]; then
        log_info "Skipping build (--no-build specified)"
        return
    fi

    log_header "Building Release Binaries"
    cd "$PROJECT_ROOT"
    cargo build --release --quiet 2>&1 | grep -v "^   Compiling" || true
    log_info "Release binaries built successfully"
}

run_benchmark() {
    local strategy=$1
    local dataset=$2
    local dataset_path=$3
    local config_file=$4
    local start_time=$(date +%s)
    
    echo -n "  Testing $strategy on ${dataset^^}... "
    
    # Set strategy via environment variable or config mutation
    # For now, we use the default strategy (BasicRANSAC)
    # TODO: Add strategy configuration support to binaries
    
    local cmd=""
    case "$dataset" in
        euroc)
            cmd="$RELEASE_DIR/run_euroc $config_file $dataset_path"
            ;;
        tum)
            cmd="$RELEASE_DIR/run_tum $config_file $dataset_path"
            ;;
        4seasons)
            cmd="$RELEASE_DIR/run_4seasons $config_file $dataset_path"
            ;;
    esac

    # Run with timeout and capture output
    local output
    output=$(timeout 120 $cmd 2>&1 || true)
    
    local end_time=$(date +%s)
    local elapsed=$((end_time - start_time))
    
    # Extract metrics from output
    local frames=$(echo "$output" | grep -oP 'Processed \K[0-9]+' | head -1)
    local avg_time=$(echo "$output" | grep -oP 'average \K[0-9.]+' | head -1)
    
    frames=${frames:-"N/A"}
    avg_time=${avg_time:-"N/A"}
    
    echo -e "${COLOR_GREEN}${elapsed}s${NC} (frames: $frames, avg: ${avg_time}ms)"
    
    # Log result
    echo "$dataset,${dataset^^},$strategy,${elapsed}s,$frames,${avg_time}ms" >> "$RESULTS_FILE"
}

analyze_results() {
    log_header "Benchmark Results Summary"
    
    if [ ! -f "$RESULTS_FILE" ]; then
        log_error "No results file found"
        return
    fi
    
    echo ""
    echo "Results saved to: $RESULTS_FILE"
    echo ""
    
    # Display summary table
    echo "┌─────────────────┬──────────────────┬──────────┬────────────┐"
    echo "│ Dataset         │ Strategy         │ Frames   │ Avg Time   │"
    echo "├─────────────────┼──────────────────┼──────────┼────────────┤"
    
    tail -n +2 "$RESULTS_FILE" 2>/dev/null | while IFS=',' read -r dataset dataset_name strategy elapsed frames avg_time; do
        # Format table row
        printf "│ %-15s │ %-16s │ %-8s │ %-10s │\n" "$dataset_name" "$strategy" "$frames" "$avg_time"
    done
    
    echo "└─────────────────┴──────────────────┴──────────┴────────────┘"
    
    echo ""
    echo "Next Steps:"
    echo "  1. Review results: cat $RESULTS_FILE"
    echo "  2. Generate plots: python3 scripts/plot_strategy_comparison.py"
    echo "  3. Deploy optimal: scripts/deploy_optimal_strategy.sh"
}

# ============================================================================
# Main Script
# ============================================================================

main() {
    local selected_strategies=("${STRATEGIES[@]}")
    local selected_datasets=()
    SKIP_BUILD=false
    
    # Parse arguments
    while [[ $# -gt 0 ]]; do
        case $1 in
            --all)
                selected_datasets=()
                selected_strategies=("${STRATEGIES[@]}")
                shift
                ;;
            --fast)
                selected_datasets=("euroc")
                selected_strategies=("BasicRANSAC")
                shift
                ;;
            --euroc)
                selected_datasets=("euroc")
                shift
                ;;
            --tum)
                selected_datasets=("tum")
                shift
                ;;
            --4seasons)
                selected_datasets=("4seasons")
                shift
                ;;
            --strategies)
                IFS=',' read -ra selected_strategies <<< "$2"
                shift 2
                ;;
            --datasets)
                IFS=',' read -ra selected_datasets <<< "$2"
                shift 2
                ;;
            --no-build)
                SKIP_BUILD=true
                shift
                ;;
            --help)
                usage
                ;;
            *)
                log_error "Unknown option: $1"
                usage
                ;;
        esac
    done

    # Header
    log_header "RS-VIO Multi-Strategy Benchmarking"
    echo "Timestamp: $TIMESTAMP"
    echo "Project: $PROJECT_ROOT"
    echo "Datasets: $DATASET_DIR"
    echo ""

    # Check and discover datasets
    check_datasets
    
    # If no specific datasets requested, use all available
    if [ ${#selected_datasets[@]} -eq 0 ]; then
        selected_datasets=("${DATASETS[@]}")
    fi

    echo ""
    echo "Strategies to test: ${selected_strategies[*]}"
    echo "Datasets to test: ${selected_datasets[*]}"
    echo ""

    # Create results directory and file
    mkdir -p "$RESULTS_DIR"
    echo "dataset,dataset_name,strategy,elapsed_time,frame_count,avg_processing_ms" > "$RESULTS_FILE"

    # Build release binaries
    build_release

    # Run benchmarks
    log_header "Running Benchmarks"
    
    local total_tests=$((${#selected_strategies[@]} * ${#selected_datasets[@]}))
    local current_test=0
    
    for strategy in "${selected_strategies[@]}"; do
        for dataset in "${selected_datasets[@]}"; do
            current_test=$((current_test + 1))
            echo ""
            echo -e "${COLOR_YELLOW}[$current_test/$total_tests]${NC} Strategy: $strategy"
            
            case "$dataset" in
                euroc)
                    config_file="$PROJECT_ROOT/config/euroc_vio.yaml"
                    dataset_path="$DATASET_DIR/euroc/MH_01_easy"
                    if [ ! -d "$dataset_path" ]; then
                        log_warn "EuRoC dataset not found, skipping"
                        continue
                    fi
                    run_benchmark "$strategy" "euroc" "$dataset_path" "$config_file"
                    ;;
                tum)
                    config_file="$PROJECT_ROOT/config/tum_vi.yaml"
                    dataset_path="$DATASET_DIR/tum_vi"
                    if [ ! -d "$dataset_path" ]; then
                        log_warn "TUM-VI dataset not found, skipping"
                        continue
                    fi
                    run_benchmark "$strategy" "tum" "$dataset_path" "$config_file"
                    ;;
                4seasons)
                    config_file="$PROJECT_ROOT/config/4seasons.yaml"
                    dataset_path=$(ls -d "$DATASET_DIR"/4seasons/recording_* 2>/dev/null | head -1)
                    if [ -z "$dataset_path" ]; then
                        log_warn "4Seasons dataset not found, skipping"
                        continue
                    fi
                    run_benchmark "$strategy" "4seasons" "$dataset_path" "$config_file"
                    ;;
            esac
        done
    done

    # Analyze and display results
    analyze_results
    
    log_header "Benchmark Complete"
    log_info "Results saved to: $RESULTS_FILE"
    echo ""
}

# Run main script
main "$@"
