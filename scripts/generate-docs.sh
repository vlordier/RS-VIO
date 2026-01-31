#!/bin/bash
# Generate and manage documentation for RS-VIO
# Includes rustdoc API documentation and benchmark docs

set -euo pipefail

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m'  # No Color

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
    echo -e "${BLUE}ℹ $1${NC}"
}

print_help() {
    cat << EOF
Generate and manage documentation for RS-VIO

Usage: generate-docs.sh [OPTIONS]

OPTIONS:
  --help           Show this help message
  --all            Generate all documentation (default)
  --rustdoc        Generate rustdoc API documentation
  --benchmarks     Generate benchmark documentation
  --private        Include private items in rustdoc (--document-private-items)
  --open           Open generated documentation in browser
  --clean          Remove generated documentation
  --server         Serve documentation locally (requires simple-http-server)

EXAMPLES:
  # Generate all documentation
  ./generate-docs.sh

  # Generate with private items and open in browser
  ./generate-docs.sh --private --open

  # Generate benchmark docs only
  ./generate-docs.sh --benchmarks

  # Clean and regenerate
  ./generate-docs.sh --clean && ./generate-docs.sh

  # Serve documentation locally
  ./generate-docs.sh --all --server

DOCUMENTATION OUTPUT:
  - API Docs: target/doc/rs_vio/
  - Benchmark Docs: target/doc/
  - README: See BENCHMARKING.md for manual docs
EOF
}

generate_rustdoc() {
    print_header "Generating Rustdoc API Documentation"

    local args="--lib"
    if [[ "${PRIVATE_ITEMS}" == "true" ]]; then
        args="$args --document-private-items"
        print_info "Including private items in documentation"
    fi

    # shellcheck disable=SC2086
    if cargo doc $args --no-deps 2>&1; then
        print_success "Rustdoc generated: target/doc/rs_vio/"
        echo ""
        echo "View documentation:"
        echo "  - Main library: target/doc/rs_vio/index.html"
        echo "  - Feature Tracker: target/doc/rs_vio/feature_tracker/index.html"
        echo "  - Estimator: target/doc/rs_vio/estimator/index.html"
        echo "  - Optimization: target/doc/rs_vio/optimization/index.html"
        echo ""
        return 0
    else
        print_error "Failed to generate rustdoc"
        return 1
    fi
}

generate_benchmark_docs() {
    print_header "Generating Benchmark Documentation"

    # Benchmark files with documentation
    local bench_files=(
        "benches/feature_tracker.rs"
        "benches/estimator.rs"
        "benches/optimization.rs"
        "benches/pipeline.rs"
    )

    print_info "Building benchmarks (this extracts documentation from code)"
    if cargo test --benches --no-run --release 2>&1 | grep -q "Compiling"; then
        print_success "Benchmark code documented"
        echo ""
        echo "Benchmark documentation is embedded in source files:"
        for file in "${bench_files[@]}"; do
            echo "  - $file (view with: cargo doc --open)"
        done
        echo ""
        echo "Each benchmark includes:"
        echo "  - Purpose and metrics"
        echo "  - Test parameters and ranges"
        echo "  - Expected results and baselines"
        echo "  - Links to detailed BENCHMARKING.md guide"
        return 0
    else
        print_error "Failed to build benchmarks"
        return 1
    fi
}

open_documentation() {
    print_header "Opening Documentation"

    local doc_path="target/doc/rs_vio/index.html"

    if [[ -f "$doc_path" ]]; then
        print_info "Opening $doc_path"
        if command -v open &> /dev/null; then
            open "$doc_path"
        elif command -v xdg-open &> /dev/null; then
            xdg-open "$doc_path"
        else
            print_error "Could not open documentation (open/xdg-open not found)"
            print_info "Open manually: $doc_path"
            return 1
        fi
        print_success "Documentation opened in browser"
    else
        print_error "Documentation not found. Run: cargo doc --lib"
        return 1
    fi
}

serve_documentation() {
    print_header "Serving Documentation Locally"

    if ! command -v simple-http-server &> /dev/null; then
        print_info "Installing simple-http-server..."
        cargo install simple-http-server
    fi

    local port="${DOC_PORT:-8000}"
    print_info "Starting documentation server on http://localhost:$port"
    echo ""
    echo "Serving from: target/doc/"
    echo "API Docs: http://localhost:$port/rs_vio/index.html"
    echo "Press Ctrl+C to stop"
    echo ""

    cd target/doc
    simple-http-server --port "$port" --index
}

clean_documentation() {
    print_header "Cleaning Documentation"

    if [[ -d "target/doc" ]]; then
        rm -rf target/doc
        print_success "Removed target/doc/"
    fi

    if cargo clean 2>&1; then
        print_success "Cleaned build artifacts"
    fi
}

# Parse arguments
GENERATE_RUSTDOC=false
GENERATE_BENCHMARKS=false
OPEN_DOCS=false
CLEAN_DOCS=false
SERVE_DOCS=false
PRIVATE_ITEMS=false
GENERATE_ALL=true

while [[ $# -gt 0 ]]; do
    case $1 in
        --help)
            print_help
            exit 0
            ;;
        --all)
            GENERATE_ALL=true
            shift
            ;;
        --rustdoc)
            GENERATE_ALL=false
            GENERATE_RUSTDOC=true
            shift
            ;;
        --benchmarks)
            GENERATE_ALL=false
            GENERATE_BENCHMARKS=true
            shift
            ;;
        --private)
            PRIVATE_ITEMS=true
            shift
            ;;
        --open)
            OPEN_DOCS=true
            shift
            ;;
        --clean)
            CLEAN_DOCS=true
            shift
            ;;
        --server)
            GENERATE_ALL=false
            SERVE_DOCS=true
            shift
            ;;
        *)
            print_error "Unknown option: $1"
            print_help
            exit 1
            ;;
    esac
done

# Main execution
if [[ "$CLEAN_DOCS" == "true" ]]; then
    clean_documentation
    if [[ "$GENERATE_ALL" == "false" ]] && [[ "$SERVE_DOCS" == "false" ]]; then
        exit 0
    fi
fi

if [[ "$SERVE_DOCS" == "true" ]]; then
    # Generate docs before serving if they don't exist
    if [[ ! -d "target/doc/rs_vio" ]]; then
        generate_rustdoc
    fi
    serve_documentation
    exit $?
fi

if [[ "$GENERATE_ALL" == "true" ]]; then
    GENERATE_RUSTDOC=true
    GENERATE_BENCHMARKS=true
fi

# Generate documentation
EXIT_CODE=0
if [[ "$GENERATE_RUSTDOC" == "true" ]]; then
    if ! generate_rustdoc; then
        EXIT_CODE=1
    fi
fi

if [[ "$GENERATE_BENCHMARKS" == "true" ]]; then
    if ! generate_benchmark_docs; then
        EXIT_CODE=1
    fi
fi

# Open documentation if requested
if [[ "$OPEN_DOCS" == "true" ]] && [[ $EXIT_CODE -eq 0 ]]; then
    echo ""
    open_documentation
fi

if [[ $EXIT_CODE -eq 0 ]]; then
    print_success "Documentation generation complete!"
    echo ""
    echo "Next steps:"
    echo "  1. Review generated documentation: target/doc/rs_vio/"
    echo "  2. Open in browser: ./scripts/generate-docs.sh --open"
    echo "  3. Serve locally: ./scripts/generate-docs.sh --server"
    echo "  4. See BENCHMARKING.md for detailed benchmark information"
else
    print_error "Documentation generation failed"
fi

exit $EXIT_CODE
