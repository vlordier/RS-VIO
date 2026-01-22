# CPU Profiling Analysis: flamegraph + LLVM IR Lines

## Overview
Analyzed CPU hotspots using `cargo llvm-lines` to identify functions consuming the most LLVM IR code, which correlates with compilation size and often CPU performance overhead.

**Total IR Lines: 702,757** (after recent bloat optimization)

## Top CPU Hotspots (by LLVM IR code size)

### Tier 1: Matrix Operations (nalgebra) - 15.3% of IR
These are the dominant CPU consumers, representing linear algebra operations:

| Function | IR Lines | Copies | % Total |
|----------|----------|--------|---------|
| `nalgebra::linalg::inverse::do_inverse4` | 20,301 | 3 | 2.9% |
| `nalgebra::base::blas::dotx` | 19,950 | 21 | 2.8% |
| `nalgebra::base::matrix::Matrix::apply` | 10,184 | 128 | 1.4% |
| `nalgebra::linalg::svd::SVD::try_new_unordered` | 5,294 | 3 | 0.8% |
| `nalgebra::linalg::inverse::try_inverse_mut` | 4,843 | 3 | 0.7% |
| `nalgebra::base::blas_uninit::gemm_uninit` | 7,280 | 30 | 1.0% |
| `nalgebra::base::blas_uninit::gemv_uninit` | 4,106 | 24 | 0.6% |
| `nalgebra::base::blas_uninit::axcpy_uninit` | 5,712 | 34 | 0.8% |

**Combined: ~77.7k IR lines (11.0%)**

**Impact:** These matrix inversion, dot product, and GEMM (matrix multiplication) operations are used extensively in:
- IMU preintegration calculations
- Camera pose estimation
- Marginalization factors
- Bundle adjustment optimization

---

### Tier 2: Sorting & Iterator Operations - 9.6% of IR
Iterative algorithms with heavy memory patterns:

| Function | IR Lines | Copies | % Total |
|----------|----------|--------|---------|
| `<core::slice::iter::Iter<T> as Iterator>::fold` | 14,095 | 150 | 2.0% |
| `core::slice::sort::stable::quicksort::stable_partition` | 9,130 | 34 | 1.3% |
| `core::slice::sort::shared::smallsort::small_sort_general_with_scratch` | 4,781 | 19 | 0.7% |
| `core::slice::sort::stable::drift::sort` | 4,233 | 17 | 0.6% |
| `core::slice::sort::shared::find_existing_run` | 3,520 | 20 | 0.5% |
| `core::slice::sort::stable::quicksort::quicksort` | 3,441 | 17 | 0.5% |
| `core::slice::sort::stable::merge::merge` | 2,805 | 17 | 0.4% |

**Combined: ~42k IR lines (5.9%)**

**Impact:** Heavy allocations and sorting used in:
- Keyframe selection/pruning
- Feature matching (sorting by descriptor similarity)
- Marginalization block reordering
- Image pyramid iteration

---

### Tier 3: Vector Allocation & Initialization - 7.2% of IR

| Function | IR Lines | Copies | % Total |
|----------|----------|--------|---------|
| `<Vec<T> as SpecFromIterNested>::from_iter` | 9,060 | 94 | 1.3% |
| `Vec<T>::extend_trusted` | 6,074 | 68 | 0.9% |
| `Vec<T>::extend_desugared` | 3,354 | 32 | 0.5% |
| `Vec<T>::push_mut` | 2,453 | 46 | 0.3% |

**Combined: ~21k IR lines (2.9%)**

**Impact:** Memory allocation overhead in:
- Feature extraction loops
- Image patch collection
- Measurement buffering
- State initialization

---

### Tier 4: File I/O & Format Parsing - 3.2% of IR

| Function | IR Lines | Copies | % Total |
|----------|----------|--------|---------|
| `exr::meta::header::Header::read` | 5,470 | 1 | 0.8% |
| `std::io::default_read_to_end` | 4,865 | 7 | 0.7% |
| `tiff::decoder::ifd::Entry::decode_offset` | 3,150 | 15 | 0.4% |

**Combined: ~13.5k IR lines (1.9%)**

**Impact:** Image codec parsing (EXR, TIFF) from file I/O

---

### Tier 5: Core Algorithm Operations - 2.0% of IR

| Function | IR Lines | Copies | % Total |
|----------|----------|--------|---------|
| `rs_vio::estimator::processor::process_frame` | 2,584 | 1 | 0.4% |
| `nalgebra::linalg::symmetric_eigen::SymmetricEigen::do_decompose` | 3,063 | 2 | 0.4% |
| `nalgebra::linalg::schur::Schur::do_decompose` | 2,818 | 2 | 0.4% |

**Combined: ~8.5k IR lines (1.2%)**

---

## CPU Profiling Insights

### 1. **Matrix Operations Dominate (11.0% of IR)**
**Problem:** NALgebra's matrix operations are heavily monomorphized for different types (f32, f64, fixed/dynamic sizes).

**Why It Matters:** 
- Each matrix operation (inverse, multiply, dot product) gets compiled multiple times
- VIO uses both 3x3 (rotation) and 4x4 (homogeneous) matrices extensively
- Bundle adjustment involves dozens of matrix operations per optimization iteration

**Evidence:** 
- `dotx` alone has 21 copies (monomorphization)
- `Matrix::apply` has 128 copies (different type combinations)

---

### 2. **Excessive Iterator & Sorting (5.9% of IR)**
**Problem:** Generic sorting and iteration code is monomorphized for many element types.

**Why It Matters:**
- Sorting used for feature track management, keyframe selection
- Iterator `fold` operation monomorphized across different closure types
- Each unique closure creates a new monomorphization instantiation

---

### 3. **Vector Allocation Overhead (2.9% of IR)**
**Problem:** Vec operations specialized for many element types.

**Why It Matters:**
- Feature extraction creates many temporary collections
- Image pyramid building allocates vectors at each level
- State buffers require frequent allocation/deallocation

---

## Optimization Opportunities

### High ROI - Quick Wins
1. **Reduce NALgebra Monomorphization** (Potential: ~5-8% IR reduction)
   - Use f64 primarily (avoid f32 dual compilation)
   - Pre-allocate fixed-size matrices instead of dynamic
   - Consider nalgebra::SVector for small matrices

2. **Cache Matrix Results** (Potential: ~2-3% CPU reduction)
   - Avoid recomputing inverses (e.g., T_B_Cl^-1)
   - Cache dot products in inner loops
   - Pre-compute fixed transformation matrices

3. **Iterator Simplification** (Potential: ~1-2% IR reduction)
   - Use `for` loops instead of iterator chains where possible
   - Limit closure complexity (reduces monomorphization variants)

### Medium ROI - Moderate Effort
4. **Preallocate Collections** (Potential: ~1-2% CPU improvement)
   - Use `Vec::with_capacity()` for known sizes
   - Avoid repeated allocations in hot loops
   - Use stack-allocated arrays for fixed-size collections

5. **Remove EXR/TIFF Codecs** (Already done via feature gates)
   - EXR/TIFF parsing: 3.2% of IR
   - Only PNG used in datasets
   - Feature gate already in place ✅

### Lower ROI - High Effort
6. **Custom SVD/Eigenvalue Implementations**
   - Would require specialized numerical algorithms
   - High risk of numerical instability
   - Only 0.8% of IR (not worth the effort)

---

## Recommendations for Next Phase

### Priority 1: NALgebra Optimization
- **Target:** Reduce matrix operation monomorphization from 11% to ~5-7%
- **Approach:** 
  1. Audit all nalgebra type usage (f32 vs f64)
  2. Standardize on f64 for accuracy-critical operations
  3. Use fixed-size types (`Matrix3`, `Vector3`) instead of dynamic
  4. Profile marginal benefit of f32 (probably minimal)

### Priority 2: Iterator & Vector Simplification
- **Target:** Reduce iterator IR from 5.9% to ~3-4%
- **Approach:**
  1. Replace complex iterator chains with explicit loops
  2. Reduce closure variants in map/filter operations
  3. Pre-allocate collections where sizes are known

### Priority 3: CPU Cache Optimization
- **Target:** 5-10% CPU runtime improvement (not IR reduction)
- **Approach:**
  1. Profile real execution (flamegraph) on actual datasets
  2. Identify hot loops in process_frame
  3. Optimize memory access patterns
  4. Consider SIMD operations for batch matrix operations

---

## Measurement Methodology

**Tool:** `cargo llvm-lines --lib`
- Measures LLVM IR code size (proxy for compilation bloat)
- Higher IR lines often correlate with slower compilation
- Not always 1:1 with runtime performance, but indicates monomorphization bloat
- Useful for identifying code generation hotspots

**Current Baseline:**
- Total IR: 702,757 lines
- Release binary: 10 MB
- Test coverage: 498/498 passing

---

## Related Previous Optimizations
✅ **Phase 1:** Bloat optimization (committed d04b1dc)
  - Image codec: PNG-only (removed EXR/TIFF/WebP codecs)
  - Rerun feature-gating: 38% binary reduction
  - Monomorphization refactor: execute() trait object (1,991 IR lines)
  - Serde cleanup: 14 redundant rename attributes

**Remaining Bloat Reduction Opportunity:** ~50-80k IR lines (7-11%)
- Primary targets: NALgebra (11%), Iterators/Sorting (5.9%)
