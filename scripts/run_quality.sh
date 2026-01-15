#!/bin/bash
# RS-VIO Quality Pipeline Runner
# ==============================
# This script runs the complete quality pipeline for RS-VIO.
# Usage: ./scripts/run_quality.sh [--quick] [--tool <tool>]

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
cd "$PROJECT_ROOT"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Logging functions
log_info() { echo -e "${BLUE}[INFO]${NC} $1"; }
log_success() { echo -e "${GREEN}[✓]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[⚠]${NC} $1"; }
log_error() { echo -e "${RED}[✗]${NC} $1"; }

# Print header
print_header() {
    echo ""
    echo "========================================"
    echo "  RS-VIO Rust Quality Pipeline"
    echo "========================================"
    echo ""
}

# Print section header
print_section() {
    echo ""
    echo "----------------------------------------"
    echo "  $1"
    echo "----------------------------------------"
}

# Run a command and report status
run_cmd() {
    local name="$1"
    shift
    log_info "Running: $name"
    if "$@" 2>&1; then
        log_success "$name passed"
        return 0
    else
        log_error "$name failed"
        return 1
    fi
}

# Quick mode - only core checks
QUICK_MODE=false
TOOL_MODE=""

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --quick)
            QUICK_MODE=true
            shift
            ;;
        --tool)
            TOOL_MODE="$2"
            shift 2
            ;;
        --help|-h)
            echo "Usage: $0 [--quick] [--tool <tool>]"
            echo ""
            echo "Options:"
            echo "  --quick    Run only core checks (format, clippy, test)"
            echo "  --tool     Run only a specific tool (clippy, fmt, audit, etc.)"
            echo ""
            echo "Available tools:"
            echo "  fmt, clippy, check, audit, deps, test, coverage, miri, unsafe, mutants, bloat, semver"
            exit 0
            ;;
        *)
            log_error "Unknown option: $1"
            exit 1
            ;;
    esac
done

# Run specific tool
run_specific_tool() {
    case "$TOOL_MODE" in
        fmt|format)
            run_cmd "Formatting (rustfmt)" cargo fmt --all
            ;;
        clippy)
            run_cmd "Linting (clippy)" cargo clippy --all-targets --all-features -- -D warnings
            ;;
        check)
            run_cmd "Cargo check" cargo check --all-targets --all-features
            ;;
        audit)
            run_cmd "Security audit (cargo-audit)" cargo audit --deny warnings
            ;;
        deps)
            run_cmd "Dependency check (cargo-tree)" cargo tree --duplicates
            ;;
        test)
            run_cmd "Tests" cargo test --all-features --lib
            ;;
        coverage)
            if command -v cargo-tarpaulin &> /dev/null; then
                run_cmd "Coverage" cargo tarpaulin --all-features --out Html --report-name coverage
            else
                log_warn "cargo-tarpaulin not installed. Run: cargo install cargo-tarpaulin"
            fi
            ;;
        miri)
            if command -v cargo &> /dev/null && rustup show 2>/dev/null | grep -q "nightly"; then
                run_cmd "MIRI" cargo +nightly miri test --all-features
            else
                log_warn "MIRI requires nightly Rust. Run: rustup install nightly"
            fi
            ;;
        unsafe)
            if command -v cargo-geiger &> /dev/null; then
                run_cmd "Unsafe code analysis" cargo geiger --all-features --format markdown
            else
                log_warn "cargo-geiger not installed. Run: cargo install cargo-geiger"
            fi
            ;;
        mutants)
            if command -v cargo-mutants &> /dev/null; then
                run_cmd "Mutation testing" cargo mutants --all-features
            else
                log_warn "cargo-mutants not installed. Run: cargo install cargo-mutants"
            fi
            ;;
        bloat)
            if command -v cargo-bloat &> /dev/null; then
                run_cmd "Binary size analysis" cargo bloat --release --features lightglue -- -A | head -50
            else
                log_warn "cargo-bloat not installed. Run: cargo install cargo-bloat"
            fi
            ;;
        semver)
            if command -v cargo-semver &> /dev/null; then
                run_cmd "Semver check" cargo semver-checks
            else
                log_warn "cargo-semver not installed. Run: cargo install cargo-semver"
            fi
            ;;
        *)
            log_error "Unknown tool: $TOOL_MODE"
            exit 1
            ;;
    esac
}

# Main quality pipeline
run_quality_pipeline() {
    local failed=0

    print_header

    # =========================================================================
    # STAGE 1: Formatting
    # =========================================================================
    print_section "Stage 1: Formatting"
    if ! run_cmd "Formatting" cargo fmt --all; then
        ((failed++))
    fi

    # =========================================================================
    # STAGE 2: Linting
    # =========================================================================
    print_section "Stage 2: Linting"
    if ! run_cmd "Clippy" cargo clippy --all-targets --all-features -- -D warnings; then
        ((failed++))
    fi

    if ! run_cmd "Cargo check" cargo check --all-targets --all-features; then
        ((failed++))
    fi

    # Exit early in quick mode
    if $QUICK_MODE; then
        print_section "Quick Mode Complete"
        echo "Run without --quick for full pipeline."
        exit $failed
    fi

    # =========================================================================
    # STAGE 3: Security
    # =========================================================================
    print_section "Stage 3: Security"
    if ! run_cmd "Cargo audit" cargo audit --deny warnings 2>&1; then
        ((failed++))
    fi

    # =========================================================================
    # STAGE 4: Dependencies
    # =========================================================================
    print_section "Stage 4: Dependencies"
    if ! run_cmd "Dependency tree" cargo tree --duplicates 2>&1; then
        log_warn "Duplicate dependencies found"
    fi

    # =========================================================================
    # STAGE 5: Tests
    # =========================================================================
    print_section "Stage 5: Tests"
    if ! run_cmd "Unit tests" cargo test --all-features --lib; then
        ((failed++))
    fi

    # =========================================================================
    # STAGE 6: Documentation
    # =========================================================================
    print_section "Stage 6: Documentation"
    if ! run_cmd "Doc generation" cargo doc --all-features --no-deps; then
        log_warn "Documentation generation had issues"
    fi

    # =========================================================================
    # Summary
    # =========================================================================
    print_section "Quality Pipeline Summary"

    if (( failed == 0 )); then
        log_success "All quality checks passed!"
        echo ""
        echo "For additional checks, run:"
        echo "  $0 --tool miri        # Static analysis"
        echo "  $0 --tool unsafe      # Unsafe code analysis"
        echo "  $0 --tool coverage    # Code coverage"
        echo "  $0 --tool mutants     # Mutation testing"
        echo "  $0 --tool bloat       # Binary size analysis"
        echo "  $0 --tool semver      # Semver compliance"
    else
        log_error "$failed check(s) failed!"
        exit $failed
    fi
}

# Run
if [[ -n "$TOOL_MODE" ]]; then
    run_specific_tool
else
    run_quality_pipeline
fi
