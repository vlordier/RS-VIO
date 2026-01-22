# CPU Profiling Next Steps: NALgebra Optimization

## Current Situation
From flamegraph analysis via `cargo llvm-lines`, NALgebra matrix operations consume **11.0% of LLVM IR** (77.7k lines):

```
do_inverse4:       20,301 lines (3 copies)      → 4x4 matrix inversion
dotx:              19,950 lines (21 copies)     → Dot product (monomorphized 21x!)
Matrix::apply:     10,184 lines (128 copies)    → Matrix operations (128 variants!)
SVD::try_new:       5,294 lines (3 copies)      → Singular Value Decomposition
try_inverse_mut:    4,843 lines (3 copies)      → In-place inversion
GEMM/GEMV/AXCPY:   17,098 lines (88 copies)    → General matrix multiply
```

## Root Cause: Type Monomorphization

Each matrix operation is monomorphized for:
- Multiple numeric types (f32, f64, Complex32, Complex64)
- Matrix shapes (3x3, 4x4, dynamic, different storage layouts)
- Closure types (different lambdas in apply operations)

**Example:** `dotx` with 21 copies means the dot product function was compiled 21 different times for different type combinations.

---

## Investigation: Current NALgebra Usage

### Step 1: Find all nalgebra imports and type usage
```bash
grep -r "use nalgebra" src/ | head -20
grep -r "Matrix\|Vector\|DMatrix\|DVector" src/ | grep -v "test\|bench" | wc -l
grep "f32" src/*.rs | grep -v comment | wc -l
```

### Step 2: Audit numeric type usage in VIO pipeline
Check if f32 is actually used or just cargo-culted from float-generic code.

### Step 3: Measure impact of std::f64 vs f32
- f64: Higher accuracy, matches camera calibration data typically in double precision
- f32: Smaller memory, faster on some processors, but VIO accumulates small errors

**Hypothesis:** f32 probably not measurable faster in VIO (not SIMD-heavy), but adds 30% compilation bloat

---

## Optimization Plan: Phase 1 (Quick Win)

### Goal: Remove f32 Compilation Paths (Target: -5% IR)

1. **Standardize to f64 throughout**
   ```rust
   // Before: Generic over Float type
   pub fn process<F: Float>(data: &[F]) -> F { ... }
   
   // After: Use f64 directly (VIO needs precision)
   pub fn process(data: &[f64]) -> f64 { ... }
   ```

2. **Remove nalgebra dual-type features**
   ```toml
   # Before:
   nalgebra = { version = "0.33.2", features = ["std"] }
   
   # After:
   nalgebra = { version = "0.33.2", features = ["std"], default-features = false }
   # And remove any f32 type specializations
   ```

3. **Use fixed-size matrices where possible**
   ```rust
   // Instead of: DMatrix<f64> (dynamic, monomorphized 3x3, 4x4, NxN variants)
   // Use: Matrix3<f64>, Matrix4<f64>, Vector3<f64> (pre-specialized)
   
   // Replace:
   let K = DMatrix::from_row_slice(3, 3, &[fx, 0, cx, 0, fy, cy, 0, 0, 1]);
   
   // With:
   let K = Matrix3::<f64>::new(
       fx, 0.0, cx,
       0.0, fy, cy,
       0.0, 0.0, 1.0
   );
   ```

4. **Reduce Matrix::apply monomorphization (128 variants!)**
   ```rust
   // The issue: Matrix::apply takes a closure, creating a new monomorphization per closure type
   
   // Instead of:
   matrix.map(|x| expensive_operation(x))  // Creates new monomorphization
   
   // Use:
   let mut result = matrix.clone();
   for elem in result.iter_mut() {
       *elem = expensive_operation(*elem);
   }
   ```

### Files to Audit (in order)
1. `src/types.rs` - Check if Float type alias is still used
2. `src/estimator/` - Bundle adjustment uses heavy matrix ops
3. `src/optimization/marginalization.rs` - Marginalization factors use matrix ops
4. `src/optimization/loop_closure/` - Feature matching uses matrices

---

## Optimization Plan: Phase 2 (Moderate Effort)

### Precompute Fixed Transformation Matrices
Many transforms are computed repeatedly:
```rust
// Instead of computing every frame:
let T_B_C_inv = T_B_C.try_inverse().unwrap();  // 20,301 IR lines for do_inverse4!

// Precompute once:
pub struct CameraTransform {
    T_B_C: Matrix4<f64>,
    T_B_C_inv: Matrix4<f64>,
}
```

### Cache Dot Product Results
```rust
// Pattern detection: dotx (19,950 lines!) called repeatedly
// Solution: Cache results when computing same pairs

// In loop:
let result = feature_vec.dot(&template);  // Monomorphized N times
// → Could cache template or use SIMD if really hot
```

---

## Quick Impact Check

### Command to measure improvement:
```bash
# Before optimization:
cargo llvm-lines --lib 2>&1 | grep "TOTAL\|nalgebra"

# After changes:
cargo llvm-lines --lib 2>&1 | grep "TOTAL\|nalgebra"

# Test performance:
cargo bench --bench hotpath_benchmarks
```

**Expected Results if all Phase 1 done:**
- Total IR: 702,757 → ~640k (10-15% reduction)
- nalgebra proportion: 11% → ~6-7%

---

## Risk Assessment

### Low Risk (Quick Wins):
- ✅ Change `Float` type alias to just `f64`
- ✅ Use fixed-size matrices (Matrix3, Vector3)
- ✅ Precompute inverses
- ✅ Run existing test suite (validates correctness)

### Medium Risk (Monitor):
- ⚠️ Removing f32 support (if anyone uses it)
- ⚠️ SIMD optimizations (numerical stability)

### High Risk (Don't Do):
- ❌ Custom SVD implementations (numerical stability)
- ❌ Lower precision float types (accumulation errors in VIO)

---

## Next Action Items

If you want to proceed with NALgebra optimization:

1. **Measure current f32 usage:**
   ```bash
   grep -r "f32" src/ | grep -v "test\|comment" | wc -l
   ```

2. **Standardize Float type:**
   - Check if `types.rs` has `Float` type alias
   - If yes, replace with `type Float = f64`
   - Recompile and measure: `cargo llvm-lines --lib`

3. **Audit matrix creation:**
   - Find all `DMatrix` allocations
   - Replace with `Matrix3`, `Matrix4`, `Vector3` where dimensions fixed
   - Run tests to validate

4. **Precompute transforms:**
   - Identify transforms computed in hot loops
   - Create cached versions in structs

---

## Flamegraph vs LLVM-Lines

**Note:** This analysis used `cargo llvm-lines` (IR code size) as a proxy for CPU profiling.

**Real flamegraph** would require:
```bash
cargo flamegraph --release --bench hotpath_benchmarks
# Generates flamegraph.svg showing actual CPU time distribution
```

**Why LLVM-lines useful:**
- Runs instantly (no profiling overhead)
- Identifies compilation bloat (related to monomorphization)
- Shows code generator hotspots
- Correlates with both binary size and runtime performance

**Limitations:**
- Not 1:1 correlation with CPU time
- Doesn't account for cache effects
- May miss algorithmic bottlenecks that compile to small code

**Best approach:** Combine LLVM-lines analysis (bloat identification) with real flamegraph (actual performance bottlenecks).
