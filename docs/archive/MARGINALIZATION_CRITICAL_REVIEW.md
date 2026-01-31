# Marginalization Implementation: Critical Review Summary

## Quick Stats

- **14 Issues Found** (5 CRITICAL, 5 MODERATE, 4 MINOR)
- **False Positives**: 1 (Solver damping bounds are actually correct)
- **Severity**: Multiple bugs can cause silent failures (ill-conditioned matrices not detected, dimension mismatches)

---

## Critical Issues at a Glance

| # | Issue | Severity | Impact | Example |
|---|-------|----------|--------|---------|
| 7 | Condition number heuristic wrong | 🔴 CRITICAL | Misses ill-conditioning entirely | Matrix κ=1e6 appears well-conditioned |
| 3 | FEJ cache hash misses dimensions | 🔴 CRITICAL | Dimension mismatch → corrupt Schur | Param changes 3D→2D, cache serves 3D point |
| 5 | Dual scaling parameters (weight + scaling) | 🔴 CRITICAL | API confusion, redundant code | What's difference between prior_weight=1.0 and prior_info_scaling=0.9? |
| 6 | Early return after Schur computation | 🔴 CRITICAL | Wasted 50ms per cycle (embedded constraint) | Computes Schur even when marg_ids=[] |
| 1 | Partition ordering reconstructed each time | 🔴 CRITICAL | Fragile index management, no assertion | Unsorted IDs → different partitions on different calls? |

---

## Key Findings

### 1. **Condition Number Estimation is Fundamentally Broken** (Issue #7)
The current heuristic `κ ≈ ||A||_F / trace(A)` is **not** a condition number:

```
Example: A = [[1, 0], [0, 1e-6]]

True κ = 1/1e-6 = 1e6 (singular!)
Computed κ = 1.0 / 1.0 ≈ 1.0 (claims well-conditioned!) ❌

This breaks damping escalation logic — the solver can't detect
when the matrix is actually ill-conditioned.
```

**Impact**: Silent failures when Schur complement becomes singular. No warnings logged.

---

### 2. **FEJ Cache Stores Data But Never Uses It** (Issue #10)
Linearization points are cached (to maintain First-Estimate Jacobian property) but:
- The cache is populated
- Passed to prior constructor
- Stored in MarginalizationPrior
- **Never actually used by HessianApproximator** ❌

This means FEJ consistency guarantees are broken — Jacobians are re-evaluated at current linearization point instead of first point.

---

### 3. **Two Parameters Do the Same Thing** (Issue #5)
```rust
let mut info = schur_complement * config.prior_info_scaling;  // Multiply by 0.9
info = info * config.prior_weight;                           // Multiply by 1.0

// Result: multiply by 0.9
// But which parameter is "weight" and which is "scale"? Both multiply...
```

Semantic confusion in API. Should be one parameter.

---

### 4. **Partition Logic is Fragile** (Issue #1)
```rust
fn expand_block_indices(block_indices: &[usize], ...) -> Vec<usize> {
    // Rebuilds param_vec from SCRATCH by iterating keep_ids then marg_ids
    // But what if the IDs aren't in a consistent order?
    // What if keep_ids changes between calls?
}
```

No guarantee that partition is consistent across Hessian and gradient. One calls with `keep_indices`, another with `marg_indices`, but the reconstruction happens in `keep_ids` order.

---

### 5. **Pseudo-Inverse Tolerance Doesn't Scale** (Issue #4)
Threshold is fixed at `1e-10 × max_sv`, but should scale with matrix size:

```
Small matrix (10×10): tolerance = 1e-10
Large matrix (200×200): tolerance = 1e-10 (same!)

Result: Large matrices get LESS regularization per DOF,
        potentially inverting smaller singular values
```

---

## What Works Well ✅

1. **Solver fallback pipeline** is correct (Cholesky → LU → pseudo-inverse)
2. **Damping escalation** correctly bounds at 1e3× (was wrong on first read)
3. **Test coverage** is comprehensive (57 tests passing)
4. **Configuration system** is flexible and well-structured

---

## What Needs Fixing 🔧

### Week 1 (High Impact)
1. Replace condition number heuristic with SVD-based estimate or proper proxy
2. Add dimensions to FEJ cache hash
3. Move early return check before Schur computation
4. Test partition logic with unsorted parameter IDs

### Week 2 (Clean Up)
1. Merge prior_weight and prior_info_scaling into one parameter
2. Actually use FEJ linearization points in Hessian approximators
3. Remove or document ZeroGradientComputer

### Week 3+ (Polish)
1. Add dimension validation
2. Fix keyframe selection to use age, not array index
3. Document num_marginalize_per_step behavior

---

## Testing Recommendations

```bash
# Test condition number estimation
cargo test estimate_condition -- --nocapture

# Test with ill-conditioned Schur complements
cargo test marginalization_ill_conditioned -- --nocapture

# Test FEJ cache with changing dimensions
cargo test fej_cache_dimension_change -- --nocapture

# Profile partition logic with unsorted IDs
cargo bench partition_ordering
```

---

## Read the Full Analysis

See [MARGINALIZATION_CRITICAL_ANALYSIS.md](./MARGINALIZATION_CRITICAL_ANALYSIS.md) for detailed explanations of each issue, code examples, and specific fix recommendations.
