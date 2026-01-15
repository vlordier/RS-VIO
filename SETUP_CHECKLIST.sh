# RS-VIO Quality Pipeline Setup Checklist
# ========================================
# Run this to verify your setup is complete

## 1. Pre-commit Hooks Setup
echo "=== Pre-commit Setup ==="
if [ -f .pre-commit-config.yaml ]; then
    echo "✅ .pre-commit-config.yaml exists"
    if command -v pre-commit &> /dev/null; then
        echo "✅ pre-commit installed"
        pre-commit --version
        echo ""
        echo "To install hooks, run:"
        echo "  pre-commit install"
        echo "  pre-commit install -t commit-msg"
    else
        echo "❌ pre-commit not installed"
        echo "Install: pip install pre-commit"
    fi
else
    echo "❌ .pre-commit-config.yaml missing"
fi

## 2. CI/CD Workflows
echo ""
echo "=== CI Workflows ==="
for wf in .github/workflows/*.yml; do
    if [ -f "$wf" ]; then
        echo "✅ $(basename $wf)"
    else
        echo "❌ $(basename $wf) missing"
    fi
done

## 3. Quality Configuration Files
echo ""
echo "=== Quality Config Files ==="
for f in rustfmt.toml deny.toml miri.toml shears.toml .cargo/quality.toml; do
    if [ -f "$f" ]; then
        echo "✅ $f"
    else
        echo "⚠️  $f missing (optional)"
    fi
done

## 4. Scripts
echo ""
echo "=== Quality Scripts ==="
for f in scripts/run_quality.sh; do
    if [ -f "$f" ] && [ -x "$f" ]; then
        echo "✅ $f (executable)"
    elif [ -f "$f" ]; then
        echo "⚠️  $f (not executable - run: chmod +x $f)"
    else
        echo "❌ $f missing"
    fi
done

## 5. Makefile
echo ""
echo "=== Makefile Targets ==="
if [ -f Makefile.quality ]; then
    echo "✅ Makefile.quality exists"
    echo "Available targets:"
    grep -E "^[a-z-]+:" Makefile.quality | head -20
else
    echo "❌ Makefile.quality missing"
fi

## 6. Documentation
echo ""
echo "=== Documentation ==="
if [ -f RUST_QUALITY.md ]; then
    echo "✅ RUST_QUALITY.md"
else
    echo "⚠️  RUST_QUALITY.md missing"
fi

## 7. Rust Toolchain
echo ""
echo "=== Rust Toolchain ==="
rustc --version
cargo --version
echo ""
echo "Install nightly for MIRI/fuzzing:"
echo "  rustup toolchain install nightly"
echo "  rustup +nightly component add miri rust-src"

## 8. Core Quality Tools Check
echo ""
echo "=== Core Tools ==="
# Check clippy via cargo
if cargo clippy --version &>/dev/null; then
    echo "✅ clippy"
else
    echo "⚠️  clippy not installed"
    echo "   Install: rustup component add clippy"
fi
# Check cargo tools
for tool in audit deny; do
    if command -v $tool &> /dev/null; then
        echo "✅ $tool"
    else
        echo "⚠️  $tool not installed (cargo install cargo-$tool)"
    fi
done

## 9. Advanced Tools (Optional)
echo ""
echo "=== Advanced Tools ==="
for tool in nextest tarpaulin geiger bloat llvm-lines mutants hack shear deadlinks fuzz; do
    if command -v $tool &> /dev/null; then
        echo "✅ $tool"
    else
        echo "⚠️  $tool (optional: cargo install $tool)"
    fi
done

## 10. Compilation & Tests
echo ""
echo "=== Build Test ==="
if cargo check --lib 2>&1 | grep -q "Finished"; then
    echo "✅ Compilation successful"
else
    echo "❌ Compilation failed"
fi

if cargo test --lib 2>&1 | grep -q "test result: ok"; then
    echo "✅ Tests passing"
else
    echo "❌ Tests failing"
fi

## Summary
echo ""
echo "========================================"
echo "  Setup Status Summary"
echo "========================================"
echo ""
echo "To get started:"
echo ""
echo "1. Install pre-commit:"
echo "   pip install pre-commit"
echo "   pre-commit install"
echo "   pre-commit install -t commit-msg"
echo ""
echo "2. Install core cargo tools:"
echo "   make -f Makefile.quality install-tools"
echo ""
echo "3. Run quality check:"
echo "   ./scripts/run_quality.sh --quick"
echo ""
echo "4. For full pipeline (weekly):"
echo "   ./scripts/run_quality.sh"
echo ""
