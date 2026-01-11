#!/usr/bin/env bash
set -euo pipefail

# RS-VIO Complete Testing Script
# Runs all tests and benchmarks

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COLOR_GREEN='\033[0;32m'
COLOR_BLUE='\033[0;34m'
COLOR_YELLOW='\033[1;33m'
COLOR_RED='\033[0;31m'
NC='\033[0m'

log_section() { echo -e "${COLOR_BLUE}═══════════════════════════════════════════════════════${NC}"; echo -e "${COLOR_BLUE}$1${NC}"; echo -e "${COLOR_BLUE}═══════════════════════════════════════════════════════${NC}"; }
log_success() { echo -e "${COLOR_GREEN}✓ $*${NC}"; }
log_error() { echo -e "${COLOR_RED}✗ $*${NC}"; }
log_info() { echo -e "${COLOR_BLUE}→ $*${NC}"; }

cd "$PROJECT_ROOT"

# Test counters
tests_passed=0
tests_failed=0

run_test() {
  local name="$1"
  shift
  
  log_info "$name"
  if "$@"; then
    log_success "$name passed"
    ((tests_passed++))
    return 0
  else
    log_error "$name failed"
    ((tests_failed++))
    return 1
  fi
}

# Tests
log_section "RS-VIO Complete Test Suite"
echo ""

log_section "1. Code Quality"
run_test "Formatting check" cargo fmt -- --check
run_test "Clippy linting (release)" cargo clippy --all --release -- -D warnings
run_test "Security audit" cargo audit
run_test "Shell script linting" ./scripts/lint_shell.sh
echo ""

log_section "2. Unit Tests"
run_test "Unit tests (debug)" cargo test --lib --all
run_test "Unit tests (release)" cargo test --lib --all --release
echo ""

log_section "3. Integration Tests"
run_test "Integration tests" cargo test --test '*' --release
echo ""

log_section "4. Property & Stress Tests"
run_test "Property tests" cargo test --test 'property_tests' --release
run_test "Stress tests" cargo test --test 'stress_test' --release
echo ""

log_section "5. Documentation Tests"
run_test "Doc tests" cargo test --doc
echo ""

log_section "6. Build Artifacts"
run_test "Debug build" cargo build --all
run_test "Release build (binaries)" cargo build --release --bins
run_test "Release build (all)" cargo build --release --all
echo ""

log_section "7. Binary Size Check"
for binary in run_euroc run_tum run_4seasons; do
  if [ -f "target/release/$binary" ]; then
    size=$(du -h "target/release/$binary" | cut -f1)
    echo -e "${COLOR_GREEN}✓${NC} $binary: $size"
  fi
done
echo ""

log_section "8. Benchmarks (Optional)"
if command -v cargo-criterion &>/dev/null; then
  log_info "Running benchmarks..."
  cargo bench --bench estimator --release 2>&1 | head -20 || true
  log_success "Benchmarks completed"
else
  log_info "Skipping benchmarks (criterion not installed)"
fi
echo ""

log_section "Summary"
total=$((tests_passed + tests_failed))
echo "Tests run: $total"
echo -e "  ${COLOR_GREEN}Passed: $tests_passed${NC}"
if [ $tests_failed -gt 0 ]; then
  echo -e "  ${COLOR_RED}Failed: $tests_failed${NC}"
  exit 1
else
  echo -e "  ${COLOR_GREEN}Failed: $tests_failed${NC}"
fi
echo ""

if [ $total -eq 0 ]; then
  log_error "No tests found to run"
  exit 1
fi

if [ $tests_failed -eq 0 ]; then
  log_success "All tests passed! ✓"
  exit 0
else
  log_error "Some tests failed!"
  exit 1
fi
