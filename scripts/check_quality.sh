#!/usr/bin/env bash
# Comprehensive quality check script for RS-VIO
# Usage: ./scripts/check_quality.sh [--full]

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

cd "$PROJECT_ROOT"

echo "════════════════════════════════════════════════════════════════"
echo "  RS-VIO Quality Assurance Check"
echo "════════════════════════════════════════════════════════════════"
echo ""

FULL_CHECK=false
if [[ "$1" == "--full" ]]; then
    FULL_CHECK=true
fi

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Track failures
FAILURES=0

run_check() {
    local name="$1"
    local command="$2"
    
    echo -e "${YELLOW}▶${NC} Running: $name"
    if eval "$command"; then
        echo -e "${GREEN}✅${NC} $name: PASSED"
        echo ""
        return 0
    else
        echo -e "${RED}❌${NC} $name: FAILED"
        echo ""
        ((FAILURES++))
        return 1
    fi
}

# 1. Run tests
run_check "Test Suite" "cargo test --all --verbose"

# 2. Run clippy
run_check "Clippy Linting" "cargo clippy --all --all-targets -- -D warnings"

# 3. Check formatting
run_check "Code Formatting" "cargo fmt --all -- --check"

# 4. Build release
run_check "Release Build" "cargo build --release --verbose"

# 5. Security audit (full check only)
if [[ "$FULL_CHECK" == true ]]; then
    echo -e "${YELLOW}▶${NC} Running: Security Audit"
    if cargo audit --version >/dev/null 2>&1; then
        run_check "Security Audit" "cargo audit --deny warnings --ignore RUSTSEC-2025-0141"
    else
        echo -e "${YELLOW}⚠${NC}  cargo-audit not installed, skipping security check"
        echo "   Install with: cargo install cargo-audit"
        echo ""
    fi
    
    # 6. Check for outdated dependencies
    echo -e "${YELLOW}▶${NC} Checking for outdated dependencies"
    if cargo outdated --version >/dev/null 2>&1; then
        cargo outdated --workspace --format list || true
        echo ""
    else
        echo -e "${YELLOW}⚠${NC}  cargo-outdated not installed, skipping"
        echo "   Install with: cargo install cargo-outdated"
        echo ""
    fi
fi

# Summary
echo "════════════════════════════════════════════════════════════════"
if [[ $FAILURES -eq 0 ]]; then
    echo -e "${GREEN}✅ All quality checks passed!${NC}"
    echo "════════════════════════════════════════════════════════════════"
    exit 0
else
    echo -e "${RED}❌ $FAILURES check(s) failed${NC}"
    echo "════════════════════════════════════════════════════════════════"
    exit 1
fi
