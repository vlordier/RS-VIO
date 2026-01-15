# Quality Tooling Status

**Last Updated**: 2026-01-15  
**Branch**: develop  
**Rust Version**: 1.80+

## ✅ Passing Quality Checks

### Core Quality Tools (Production Ready)
All core quality tools pass without warnings:

| Tool | Status | Details |
|------|--------|---------|
| **cargo fmt** | ✅ PASS | All files formatted according to rustfmt.toml |
| **cargo clippy** | ✅ PASS | 0 warnings with `--all-targets` |
| **cargo test** | ✅ PASS | 385/385 tests passing (4 skipped expected) |
| **cargo audit** | ✅ PASS | 4 allowed low-risk warnings (documented in CI) |
| **cargo deny** | ✅ PASS | All checks passing (advisories, bans, licenses, sources) |

**Commits Applied**:
- `273cfe8` - chore: apply cargo fmt formatting
- `3926071` - fix: resolve all clippy warnings for lib and bins
- `88959f7` - fix: resolve all clippy warnings for tests and benchmarks
- `9d28713` - fix: update deny.toml for cargo-deny 0.19 compatibility

### Advanced Quality Tools

| Tool | Status | Notes |
|------|--------|-------|
| **cargo-outdated** | ✅ WORKING | 9 dependencies have updates available |
| **cargo-bloat** | ✅ WORKING | Binary size: 5.6MiB .text section |
| **cargo-nextest** | ✅ WORKING | All 385 tests pass (faster than cargo test) |
| **cargo-hack** | ✅ WORKING | Feature matrix testing for default features |

## ⚠️ Known Limitations

### cargo-geiger (Unsafe Code Audit)
**Status**: ⚠️ TOOL LIMITATION

```bash
# Symptom
cargo geiger  # Hangs or fails with package matching errors
```

**Root Cause**: Known cargo-geiger issue with large dependency trees and certain package versions (time@0.3.45, valuable@0.1.1, allocator-api2, foldhash)

**Mitigation**: 
- Project uses `#![forbid(unsafe_code)]` at crate level
- Manual verification: `rg "unsafe" src/` returns no hits in production code
- Dependencies audited via cargo-deny and cargo-audit

**Resolution**: Not blocking - project enforces memory safety via compiler

---

### cargo-udeps (Unused Dependencies)
**Status**: ⚠️ NIGHTLY REQUIRED

```bash
# Error
cargo udeps  # Fails: "error: the option `Z` is only accepted on the nightly compiler"
```

**Root Cause**: cargo-udeps uses unstable `-Z binary-dep-depinfo` flag

**Workaround**: Install nightly and use `rustup run nightly cargo udeps`

**Known False Positives**: Two nalgebra versions (0.33, 0.34) intentional for camera-intrinsic-model compatibility

**Resolution**: Not urgent - dependencies manually reviewed and minimal

---

### use_f32 Feature
**Status**: ⚠️ INCOMPLETE (70+ errors)

```bash
# Test
cargo check --no-default-features --features use_f32
# Result: 70+ type mismatch errors
```

**Affected Modules**:
- `src/estimator/estimator.rs` (16+ errors)
- `src/estimator/sliding_window.rs` (12+ errors)
- `src/estimator/state.rs` (5+ errors)
- `src/optimization/tight_coupling.rs` (10+ errors)
- Others throughout codebase

**Core Issue**: Fundamental f32/f64 type mismatches:
1. **Config parsing**: YAML deserialization creates `Vec<f64>`
2. **Literal floats**: `0.0`, `1.0` inferred as f64
3. **Math operations**: `f64::consts::PI`, `angle.sin()` return f64
4. **External APIs**: camera-intrinsic-model requires f64

**Partial Fixes Applied** (`0464db0`, `0a8b2ee`):
- ✅ Added `float_const` module (ZERO, ONE, TWO, HALF, PI, EPSILON)
- ✅ Fixed `OpenCVModel5::new()` with explicit casts to f64
- ✅ Created infrastructure for gradual migration

**Complete Fix Requires**:
1. Generic-over-float config parsing (serde deserialize to `T: Float`)
2. Systematic replacement of float literals with `float_const::`
3. Cast insertion at API boundaries (70+ locations)
4. Estimated effort: 8-12 hours

**Recommendation**: **Defer** - f64 is correct choice for SLAM (precision matters)

---

### lightglue Feature  
**Status**: ⚠️ ORT API MIGRATION NEEDED

```bash
# Test
cargo check --features lightglue
# Result: 5 ORT API errors
```

**Affected File**: `src/optimization/loop_closure/lightglue.rs`

**Core Issue**: Upgrade from ORT 1.x to ORT 2.0 broke API:

1. **Import structure changed**:
   ```rust
   // Old (ORT 1.x)
   use ort::{ExecutionProvider, GraphOptimizationLevel, Session, Value};
   
   // New (ORT 2.0)
   use ort::{
       session::{builder::GraphOptimizationLevel, Session},
       value::Value,
   };
   ```

2. **Tensor creation API changed**:
   ```rust
   // Old
   Value::from_array(array.clone())
   
   // New (needs investigation)
   Value::from_array(???)  // OwnedTensorArrayData trait issues
   ```

3. **Tensor extraction API changed**:
   ```rust
   // Old
   tensor.view()[[i, j]]
   
   // New
   let (shape, data) = tensor.try_extract_tensor::<T>()?;
   data[i * stride + j]
   ```

**Partial Fixes Applied** (`0464db0`):
- ✅ Updated imports to ORT 2.0 structure
- ✅ Fixed return type (MatchMetrics instead of MatchResult)
- ✅ Removed unused imports

**Remaining Work**:
1. Fix `Value::from_array()` - needs ndarray → ORT tensor conversion
2. Update inference API - `session.run()` signature changed
3. Test with actual ONNX model

**Estimated Effort**: 2-4 hours with ORT docs

**Recommendation**: **Defer** - feature is experimental, not used in production

---

## 📦 Dependency Updates Available

From `cargo outdated --root-deps-only`:

| Dependency | Current | Latest | Type | Breaking? |
|------------|---------|--------|------|-----------|
| camera-intrinsic-model | 0.7.3 | 0.8.0 | Normal | Minor (likely safe) |
| criterion | 0.5.1 | 0.8.1 | Dev | Major (bench API) |
| imageproc | 0.25.0 | 0.26.0 | Normal | Minor |
| nalgebra | 0.33.2 | 0.34.1 | Normal | **Already using 0.34** |
| ndarray | 0.16.1 | 0.17.2 | Normal | Minor |
| rand | 0.8.5 | 0.9.2 | Normal | Major (API changes) |
| rerun | 0.25.1 | 0.28.2 | Normal | Minor (patch) |
| thiserror | 1.0.69 | 2.0.17 | Normal | Major (error trait) |

**Recommended Updates** (low risk):
```bash
cargo update camera-intrinsic-model
cargo update imageproc
cargo update rerun
cargo update ndarray
```

**Defer** (major version bumps):
- criterion (test-only, stable API sufficient)
- rand (API audit needed)
- thiserror (Error trait changes need review)

---

## 📊 Quality Metrics

### Test Coverage
- **Total Tests**: 385
- **Unit Tests**: ~216 (lib)
- **Integration Tests**: ~165 (tests/)
- **Benchmark Tests**: 4 (benches/)
- **Pass Rate**: 100% (4 skipped intentional)
- **Execution Time**: 4.12s (cargo nextest)

### Code Quality
- **Clippy Warnings**: 0
- **Clippy Denies**: 18 safety-critical lints
- **Unsafe Code**: 0 blocks (enforced via `#![forbid(unsafe_code)]`)
- **License Compliance**: ✅ All dependencies checked
- **Security Advisories**: 4 allowed (low severity, transitive)

### Binary Size (release)
- **Total .text**: 5.6 MiB
- **Largest Symbol**: `alloc::raw_vec::RawVecInner` (65.9 KiB, 1.1%)
- **RS-VIO Code**: ~400 KiB (7% of binary)
- **Dependencies**: ~5.2 MiB (93% - mostly rerun, arrow)

---

## 🚀 Recommendation

**Production Readiness**: ✅ **READY**

The codebase passes all essential quality checks for the default feature set:
- Zero clippy warnings with strict lints
- 100% test pass rate (385 tests)
- No unsafe code
- Dependency licenses compliant
- Security advisories reviewed and allowed

**Optional Features**: ⚠️ **EXPERIMENTAL**
- `use_f32`: Significant work needed (70+ fixes)
- `lightglue`: ORT API migration needed (5+ fixes)

**Action Items**:
1. ✅ **Merge current quality improvements** to develop
2. 📦 **Update low-risk dependencies** (camera-intrinsic-model, imageproc, rerun)
3. 📝 **Document use_f32/lightglue** as experimental in README
4. 🔄 **Re-run quality suite** before releases: `make quality`

---

## 🔧 Running Quality Checks

```bash
# Core checks (required before merge)
cargo fmt --check
cargo clippy --all-targets
cargo test
cargo audit
cargo deny check

# Advanced checks (optional)
cargo outdated --root-deps-only
cargo bloat --release -n 20
cargo nextest run
cargo hack --feature-powerset --exclude-features lightglue,use_f32

# Full quality suite
make quality  # See Makefile.quality
```

---

## 📚 References

- **CLIPPY_LINTS.toml**: 106 lines, 18 deny-level lints
- **deny.toml**: cargo-deny v0.19 configuration
- **.github/workflows/quality.yml**: CI pipeline
- **Makefile.quality**: 287-line quality automation
