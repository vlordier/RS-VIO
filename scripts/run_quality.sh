#!/bin/bash
# RS-VIO Quality Pipeline Runner - Embedded Drone Edition
# =======================================================
# Harshest configuration for safety-critical drone real-time VIO
# Usage: ./scripts/run_quality.sh [--quick] [--tool <tool>] [--embedded]

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
cd "$PROJECT_ROOT"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Logging functions
log_info() { echo -e "${BLUE}[INFO]${NC} $1"; }
log_success() { echo -e "${GREEN}[✓]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[⚠]${NC} $1"; }
log_error() { echo -e "${RED}[✗]${NC} $1"; }
log_embedded() { echo -e "${CYAN}[DRONE]${NC} $1"; }

# Print header
print_header() {
    echo ""
    echo "========================================"
    echo "  RS-VIO Embedded Drone Quality Pipeline"
    echo "  Safety-Critical Real-Time VIO"
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
EMBEDDED_MODE=false

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
        --embedded|-e)
            EMBEDDED_MODE=true
            shift
            ;;
        --help|-h)
            echo "Usage: $0 [--quick] [--tool <tool>] [--embedded]"
            echo ""
            echo "Options:"
            echo "  --quick    Run only core checks (format, clippy, test)"
            echo "  --tool     Run only a specific tool"
            echo "  --embedded Use embedded-safe profile for maximum safety"
            echo ""
            echo "Available tools:"
            echo "  fmt, clippy, check, audit, deps, test, coverage, miri, unsafe, mutants, bloat, semver"
            echo ""
            echo "Embedded profiles:"
            echo "  --embedded        Use embedded-safe profile"
            echo "  cargo build --profile ultra-critical  # Maximum safety"
            echo "  cargo build --profile bare-metal      # Bare-metal STM32"
            exit 0
            ;;
        *)
            log_error "Unknown option: $1"
            exit 1
            ;;
    esac
done

# Get cargo profile for embedded mode
get_profile() {
    if $EMBEDDED_MODE; then
        echo "embedded-safe"
    else
        echo "release"
    fi
}

# Run specific tool
run_specific_tool() {
    local profile
    profile=$(get_profile)
    case "$TOOL_MODE" in
        fmt|format)
            run_cmd "Formatting (rustfmt)" cargo fmt --all
            ;;
        clippy)
            if $EMBEDDED_MODE; then
                log_embedded "Using embedded-safe profile for clippy"
                run_cmd "Linting (clippy - embedded)" cargo clippy --profile "$profile" --all-targets --all-features -- -D warnings -A clippy::cast_precision_loss -A clippy::float_cmp
            else
                run_cmd "Linting (clippy)" cargo clippy --all-targets --all-features -- -D warnings
            fi
            ;;
        check)
            if $EMBEDDED_MODE; then
                run_cmd "Cargo check (embedded-safe)" cargo check --profile "$profile" --all-targets --all-features
            else
                run_cmd "Cargo check" cargo check --all-targets --all-features
            fi
            ;;
        audit)
            run_cmd "Security audit (cargo-audit)" cargo audit --deny warnings
            ;;
        deps)
            run_cmd "Dependency check (cargo-tree)" cargo tree --duplicates
            ;;
        test)
            if $EMBEDDED_MODE; then
                run_cmd "Tests (embedded-safe)" cargo test --profile "$profile" --all-features --lib
            else
                run_cmd "Tests" cargo test --all-features --lib
            fi
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
    local profile
    profile=$(get_profile)

    print_header

    if $EMBEDDED_MODE; then
        log_embedded "Using embedded-safe profile: $profile"
        log_embedded "Features: overflow-checks=true, debug-assertions=true, lto=fat"
    fi

    # =========================================================================
    # STAGE 1: Formatting (non-negotiable for embedded)
    # =========================================================================
    print_section "Stage 1: Formatting"
    if ! run_cmd "Formatting" cargo fmt --all; then
        ((failed++))
    fi

    # =========================================================================
    # STAGE 2: Linting (harshest for embedded safety)
    # =========================================================================
    print_section "Stage 2: Linting"
    if $EMBEDDED_MODE; then
        # Allow VIO-specific patterns but deny all others
        if ! run_cmd "Clippy (embedded-safe)" cargo clippy --profile "$profile" --all-targets --all-features -- -D warnings -A clippy::cast_precision_loss -A clippy::float_cmp -A clippy::unwrap_used -A clippy::expect_used; then
            ((failed++))
        fi
    else
        if ! run_cmd "Clippy" cargo clippy --all-targets --all-features -- -D warnings; then
            ((failed++))
        fi
    fi

    if ! run_cmd "Cargo check" cargo check --all-targets --all-features; then
        ((failed++))
    fi

    # Exit early in quick mode
    if $QUICK_MODE; then
        print_section "Quick Mode Complete"
        echo "Run without --quick for full pipeline."
        echo "For embedded safety: $0 --quick --embedded"
        exit $failed
    fi

    # =========================================================================
    # STAGE 3: Security (critical for drone safety)
    # =========================================================================
    print_section "Stage 3: Security"
    if ! run_cmd "Cargo audit" cargo audit --deny warnings 2>&1; then
        ((failed++))
    fi

    # =========================================================================
    # STAGE 4: Dependencies (supply chain security)
    # =========================================================================
    print_section "Stage 4: Dependencies"
    if ! run_cmd "Dependency tree" cargo tree --duplicates 2>&1; then
        log_warn "Duplicate dependencies found"
    fi

    if ! run_cmd "Unneeded dependencies" cargo +nightly udeps --all-targets 2>&1; then
        log_warn "Unneeded dependencies detected"
    fi

    # =========================================================================
    # STAGE 5: Tests (100% coverage target for embedded)
    # =========================================================================
    print_section "Stage 5: Tests"
    if $EMBEDDED_MODE; then
        if ! run_cmd "Unit tests (embedded-safe)" cargo test --profile "$profile" --all-features --lib; then
            ((failed++))
        fi
    else
        if ! run_cmd "Unit tests" cargo test --all-features --lib; then
            ((failed++))
        fi
    fi

    # =========================================================================
    # STAGE 6: Documentation (safety-critical docs)
    # =========================================================================
    print_section "Stage 6: Documentation"
    if ! run_cmd "Doc generation" cargo doc --all-features --no-deps; then
        log_warn "Documentation generation had issues"
    fi

    if ! run_cmd "Dead links" cargo deadlinks --dir target/doc 2>&1; then
        log_warn "Broken documentation links found"
    fi

    # =========================================================================
    # STAGE 7: Embedded Safety Checks
    # =========================================================================
    if $EMBEDDED_MODE; then
        print_section "Stage 7: Embedded Safety"
        log_embedded "Verifying embedded profile settings..."

        # Check that embedded profile has correct settings
        if grep -q "overflow-checks = true" Cargo.toml && \
           grep -q "panic = \"abort\"" Cargo.toml && \
           grep -q "lto = \"fat\"" Cargo.toml; then
            log_success "Embedded profile correctly configured"
        else
            log_warn "Embedded profile may not be optimally configured"
        fi
    fi

    # =========================================================================
    # Summary
    # =========================================================================
    print_section "Quality Pipeline Summary"

    if (( failed == 0 )); then
        log_success "All quality checks passed!"
        echo ""
        echo "For embedded drone safety, run:"
        echo "  $0 --embedded                    # Full pipeline with embedded-safe"
        echo "  $0 --quick --embedded            # Quick embedded check"
        echo ""
        echo "Build commands:"
        echo "  cargo build --profile embedded-safe   # Embedded-safe build"
        echo "  cargo build --profile ultra-critical  # Maximum safety build"
        echo "  cargo build --profile bare-metal      # Bare-metal STM32 build"
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
