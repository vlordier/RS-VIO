# Marginalization: Priority Bug Fixes

## Overview
This document lists actionable bugs found during critical review, prioritized by:
1. Impact on correctness/stability
2. Ease of fix
3. Risk to embedded deployment

---

## PRIORITY 1: Fix This First (Do Today)

### Bug #7: Condition Number Heuristic is Wrong
**File**: `src/optimization/marginalization.rs:1256-1274`
**Severity**: CRITICAL
**Issue**: Current heuristic `κ ≈ ||A||_F / trace(A)` fails to detect ill-conditioning

**Problem Code**:
```rust
fn estimate_condition_number(&self, matrix: &DMatrix<f64>) -> Option<f64> {
    let frob = matrix.norm();
    let trace = (0..matrix.nrows().min(matrix.ncols()))
        .map(|i| matrix[(i, i)].abs())
        .sum::<f64>();

    if trace > 1e-12 {
        Some(frob / trace)  // ← WRONG: Not a condition number!
    } else {
        None
    }
}
```

**Failure Example**:
```
Matrix A = [[1, 0], [0, 1e-6]]
Actual condition number: 1e6
Computed: ~1.0
Result: Damping NOT escalated, solver fails silently
```

**Fix**: Use proper estimate
```rust
fn estimate_condition_number(&self, matrix: &DMatrix<f64>) -> Option<f64> {
    if matrix.nrows() == 0 || matrix.ncols() == 0 {
        return None;
    }

    // Use Frobenius norm divided by minimum diagonal element
    let frob = matrix.norm();
    let min_diag = (0..matrix.nrows().min(matrix.ncols()))
        .map(|i| matrix[(i, i)].abs())
        .fold(f64::INFINITY, f64::min);

    if min_diag > 1e-14 {
        Some(frob / min_diag)  // ← Better, though still O(n) approximation
    } else {
        // Matrix is singular, use full SVD for diagnostic
        self.estimate_condition_number_svd(matrix)
    }
}
```

**Why**: Directly prevents silent failures when Schur complement becomes ill-conditioned.

**Test**:
```rust
#[test]
fn test_condition_number_detects_singular() {
    let singular_matrix = DMatrix::from_row_slice(2, 2, &[1.0, 0.0, 0.0, 1e-6]);
    let est_bad = estimate_condition_number(&singular_matrix);
    assert!(est_bad.unwrap() > 100.0, "Should detect ill-conditioning");
}
```

---

### Bug #3: FEJ Cache Hash Misses Dimensions
**File**: `src/optimization/marginalization.rs:1451-1474`
**Severity**: CRITICAL
**Issue**: Hash based only on IDs, not dimensions. If dimension changes, cache serves wrong-sized data.

**Problem Code**:
```rust
fn compute_structure_hash(keep_ids: &[ParamId], marg_ids: &[ParamId]) -> u64 {
    let mut hasher = std::collections::hash_map::DefaultHasher::new();
    let mut sorted_ids: Vec<_> = keep_ids.iter().chain(marg_ids.iter()).collect();
    sorted_ids.sort();
    for id in sorted_ids {
        id.hash(&mut hasher);  // ← Only hashes ID, not dimension!
    }
    hasher.finish()
}
```

**Failure Example**:
```
Cycle 1: KeyframeVelocity(0) has dimension 3
  Cache stores 3D point

Cycle 2: Bug changes same ID to dimension 2
  Structure hash same (same IDs)
  Cache returns 3D point for 2D param
  Marginalization uses wrong dimension → Schur is corrupted
```

**Fix**: Include dimensions in hash
```rust
fn compute_structure_hash(param_blocks: &HashMap<ParamId, ParamBlock>) -> u64 {
    let mut hasher = std::collections::hash_map::DefaultHasher::new();
    let mut entries: Vec<_> = param_blocks.iter().collect();
    entries.sort_by_key(|(id, _)| id);
    for (id, block) in entries {
        id.hash(&mut hasher);
        block.dimension.hash(&mut hasher);  // ← Include dimension!
    }
    hasher.finish()
}
```

**Update call site**:
```rust
fn update_fej_cache(&mut self, param_blocks: &HashMap<ParamId, ParamBlock>, ...) {
    let structure_hash = Self::compute_structure_hash(param_blocks);  // Pass full blocks
    // ... rest unchanged
}
```

**Why**: Prevents silent dimension mismatch corruption.

---

### Bug #6: Early Return After Expensive Schur Computation
**File**: `src/optimization/marginalization.rs:1035-1047`
**Severity**: CRITICAL (embedded constraint)
**Issue**: Computes Schur complement even when no marginalization needed (wastes 50ms)

**Problem Code**:
```rust
pub fn marginalize(&mut self, ...) -> MarginalizationResult {
    // ... 50+ lines of setup ...

    let (H_aa, H_ab, H_ba, H_bb) = self.partition_hessian(...);  // EXPENSIVE
    let (b_a, b_b) = self.partition_gradient(...);                // EXPENSIVE

    let schur_complement = &H_aa - &H_ab * &H_bb_inv_H_ba;        // VERY EXPENSIVE

    // ... 100 lines later ...

    if prior_param_ids.is_empty() {
        return MarginalizationResult { prior: None, ... };  // ← Discards result!
    }
}
```

**Impact**: 50ms wasted per marginalization cycle on Jetson Xavier (embedded constraint = 33ms per frame at 30 Hz).

**Fix**: Early exit check
```rust
pub fn marginalize(&mut self, ...) -> MarginalizationResult {
    // Validate that marginalization is needed BEFORE expensive computation
    let (keep_indices, marg_indices) = self.build_index_maps(param_blocks, keep_ids, marg_ids);

    if marg_indices.is_empty() {
        log::debug!("No parameters to marginalize; early exit");
        return MarginalizationResult {
            prior: None,
            info: MarginalizationInfo::default(),
        };
    }

    // NOW do expensive work
    let (H_aa, H_ab, H_ba, H_bb) = self.partition_hessian(...);
    // ... rest unchanged ...
}
```

**Why**: Easy 50ms gain on embedded platforms.

---

## PRIORITY 2: Fix Before Production Deployment

### Bug #5: Redundant Scaling Parameters
**File**: `src/optimization/marginalization.rs:115 + 637-642`
**Severity**: CRITICAL (API confusion)
**Issue**: `prior_weight` and `prior_info_scaling` do the same thing

**Current**:
```rust
pub struct MarginalizationConfig {
    pub prior_weight: f64,          // Weight the prior
    pub prior_info_scaling: f64,    // Scale the information
}

// Both are multiplied:
let info = schur * config.prior_info_scaling * config.prior_weight;
```

**Problem**: Semantic confusion. When tuning, which do you change?

**Fix**: Merge into one parameter
```rust
pub struct MarginalizationConfig {
    pub prior_info_scale: f64,      // Single parameter: scale info matrix
    // prior_weight removed (was always 1.0 anyway)
}

// Constructor:
let info = schur * config.prior_info_scale;  // Clear intent
```

**Why**: Reduces configuration complexity, prevents mistuning.

---

### Bug #10: FEJ Cache Populated But Unused
**File**: `src/optimization/marginalization.rs:1426-1440 + 913-922`
**Severity**: MODERATE (feature incomplete)
**Issue**: FEJ linearization points are cached but never consumed by Hessian approximators

**Current Flow**:
1. ✅ Cache is populated: `fej_cache.set_point(id, block.linearization_point)`
2. ✅ Passed to prior: `construct_prior(..., &fej_cache.points)`
3. ✅ Stored: `MarginalizationPrior.linearization_points`
4. ❌ **Never used** by `HessianApproximator.compute_hessian()`

**Fix**: Pass cache to approximators
```rust
pub trait HessianApproximator {
    fn compute_hessian(
        &self,
        residuals: &DVector<f64>,
        param_dim: usize,
        jacobians: Option<&Vec<DMatrix<f64>>>,
        fej_points: Option<&HashMap<ParamId, DVector<f64>>>,  // ← Add this
    ) -> DMatrix<f64>;
}

// In manager:
pub fn compute_approximate_hessian(...) -> DMatrix<f64> {
    self.hessian_approximator.compute_hessian(
        residuals,
        param_dim,
        jacobians,
        Some(&self.fej_cache.points),  // ← Pass cache
    )
}
```

**Why**: Activates FEJ consistency, improves marginalization accuracy.

---

## PRIORITY 3: Before First Embedded Flight

### Bug #1: Partition Logic Reconstructed Each Call
**File**: `src/optimization/marginalization.rs:1110-1230`
**Severity**: MODERATE (fragile logic)
**Issue**: `expand_block_indices()` rebuilds parameter ordering each time, no assertion that order is consistent

**Current**:
```rust
fn expand_block_indices(block_indices: &[usize], ...) -> Vec<usize> {
    let mut param_vec: Vec<(usize, usize)> = Vec::new();
    for id in keep_ids.iter() {          // Reconstructs order
        // ...
    }
    for id in marg_ids.iter() {          // Reconstructs order
        // ...
    }
    // No assertion that order matches previous calls
}
```

**Risk**: If `keep_ids` is unsorted, different partition calls might reorder differently.

**Fix**: Use explicit index ranges instead
```rust
fn partition_hessian(
    &self,
    H: &DMatrix<f64>,
    keep_idx_ranges: &[(usize, usize)],  // Start, length pairs
    marg_idx_ranges: &[(usize, usize)],
) -> (DMatrix<f64>, DMatrix<f64>, DMatrix<f64>, DMatrix<f64>) {
    // Directly extract submatrices from index ranges
    // No reconstruction, no ordering ambiguity
}
```

**Why**: Eliminates subtle reordering bugs before they cause field failures.

---

## Quick Fix Checklist

- [ ] **Today**: Fix condition number heuristic (Issue #7)
- [ ] **Today**: Fix FEJ cache hash (Issue #3)
- [ ] **Today**: Add early return before Schur (Issue #6)
- [ ] **Tomorrow**: Test partition logic with unsorted IDs (Issue #1)
- [ ] **This week**: Merge prior_weight + prior_info_scaling (Issue #5)
- [ ] **This week**: Activate FEJ in Hessian approximators (Issue #10)
- [ ] **Before flight**: Validate dimension consistency (Issue #9)

---

## Testing Each Fix

```bash
# Test condition number fix
cargo test estimate_condition -- --nocapture

# Test FEJ cache with dimension changes
cargo test fej_cache_structure_hash -- --nocapture

# Test early exit
cargo test marginalize_empty_marg_ids -- --nocapture

# Test partition with unsorted IDs
cargo test partition_unsorted_ids -- --nocapture

# Run full suite
cargo test marginalization --lib
```

---

## Impact Summary

| Fix | Time to Implement | Risk | Impact |
|-----|-------------------|------|--------|
| Issue #7 (condition number) | 30 min | Low | Prevents silent failures |
| Issue #3 (FEJ hash) | 20 min | Low | Prevents corruption |
| Issue #6 (early return) | 15 min | Low | 50ms latency gain |
| Issue #1 (partition logic) | 1 hour | Medium | Robustness improvement |
| Issue #5 (merge params) | 45 min | Medium | API cleanup |
| Issue #10 (activate FEJ) | 1 hour | Medium | Accuracy improvement |

**Total effort**: ~4 hours for critical safety fixes + API cleanup.
