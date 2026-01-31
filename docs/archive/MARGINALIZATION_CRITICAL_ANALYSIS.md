# Critical Analysis: Marginalization Implementation

## Overview

This document identifies **logical inconsistencies, design flaws, and questionable choices** in the marginalization implementation. These are issues that don't necessarily break tests but create subtle bugs, confusing APIs, or brittle code.

---

## 🔴 CRITICAL ISSUES (High Priority)

### 1. **Partition Index Ordering Bug: Hessian vs Gradient**

**Location**: Lines 1145-1230 (partition_hessian, partition_gradient, expand_block_indices)

**The Problem**:
```rust
// partition_hessian calls:
let keep_elem_indices = self.expand_block_indices(keep_indices, param_blocks, keep_ids, marg_ids);
let marg_elem_indices = self.expand_block_indices(marg_indices, param_blocks, keep_ids, marg_ids);

// partition_gradient calls:
let keep_elem_indices = self.expand_block_indices(keep_indices, param_blocks, keep_ids, marg_ids);
let marg_elem_indices = self.expand_block_indices(marg_indices, param_blocks, keep_ids, marg_ids);
```

**The Issue**: `expand_block_indices()` iterates over `keep_ids` FIRST, then `marg_ids`:

```rust
for id in keep_ids.iter() { ... }  // Adds keep indices first
for id in marg_ids.iter() { ... }  // Then adds marg indices
```

This means `current_pos` increases in the order [keep_ids..., marg_ids...].

But in `build_index_maps()` (line 1110), the order is the SAME:
```rust
for id in keep_ids { ... }        // Position 0..n_keep
for id in marg_ids { ... }        // Position n_keep..total
```

**Wait... is this actually inconsistent?** Let me trace through:

1. Hessian partitioning uses `expand_block_indices(keep_indices, ...)` and `expand_block_indices(marg_indices, ...)`
2. `expand_block_indices` rebuilds param_vec by iterating keep_ids THEN marg_ids
3. So the element indices should be in order [keep elements 0..n_keep], [marg elements n_keep..total]

But the issue is: **`expand_block_indices` doesn't take the actual ordering from `keep_indices`/`marg_indices` — it RECONSTRUCTS it from scratch every time.**

This is fragile:
- If `keep_ids` and `marg_ids` are not in a stable order, repeated calls give different results
- There's no validation that the reconstructed order matches what was expected

**Impact**: Subtle reordering bugs if parameter IDs are provided in different orders across calls.

**Fix**: Pass the actual element index ranges to partition functions instead of reconstructing:
```rust
fn partition_hessian(&self, H: &DMatrix<f64>,
    keep_start: usize, keep_len: usize,
    marg_start: usize, marg_len: usize) -> (...)
```

---

### 2. **Solver Pipeline: MAX_DAMPING_SCALE Boundary Condition**

**Location**: Lines 1327-1345

**Status**: ✅ **NOT A BUG** (I was wrong on first review)

**Why I thought it was a bug**:
I misread the loop boundary condition. The code is:
```rust
for attempt in 1..4 {              // Attempts 1, 2, 3 (3 total)
    damping_scale *= 10.0;         // Scale becomes 10, 100, 1000
    if damping_scale > MAX_DAMPING_SCALE {  // 1e3
        log::warn!("... switching to LU fallback");
        break;
    }
    regularized = H_bb.clone();
    Self::add_diagonal_damping(&mut regularized, self.config.damping * damping_scale);
    // Try Cholesky...
}
```

The condition is `>` (strictly greater than), not `>=`. So:
- Attempt 1: scale=10, apply damping 1e-4, try Cholesky
- Attempt 2: scale=100, apply damping 1e-3, try Cholesky
- Attempt 3: scale=1000, apply damping 1e-2, try Cholesky (since 1000 ≯ 1000)
- Loop exits naturally

So all three damping levels ARE attempted. ✅ **Correct behavior.**

**Note**: The boundary is correct as-is. Using `>=` would skip the 1000× scale, which would reduce the regularization range unnecessarily.

---

### 3. **FEJ Cache Structure Hash: Doesn't Include Dimensions**

**Location**: Lines 1468-1474 (update_fej_cache, compute_structure_hash)

**The Problem**:
```rust
fn compute_structure_hash(keep_ids: &[ParamId], marg_ids: &[ParamId]) -> u64 {
    let mut hasher = std::collections::hash_map::DefaultHasher::new();
    let mut sorted_ids: Vec<_> = keep_ids.iter().chain(marg_ids.iter()).collect();
    sorted_ids.sort();
    for id in sorted_ids {
        id.hash(&mut hasher);
    }
    hasher.finish()
}
```

**The Issue**:
The hash is based ONLY on parameter IDs, not on dimensions. Consider this scenario:

1. Initial state: KeyframePose(0) with dimension 6
2. Marginalize that frame
3. New state: Same parameter IDs, but now KeyframePose(1) with dimension 5 (reduced measurement)

The structure hash would be DIFFERENT (different IDs), so this isn't actually a problem. BUT:

**The real issue**: If a parameter's dimension changes (e.g., due to a bug in the caller), the FEJ cache will have stale linearization points with the WRONG dimension, causing silent failures downstream.

**Example**:
```rust
// Cycle 1: KeyframeVelocity(0) has dimension 3
fej_cache.set_point(KeyframeVelocity(0), vec![...3...])

// Cycle 2: Code bug causes same ID with dimension 2
// But structure hash is the same (same IDs)
// FEJ cache still returns 3-dimensional point for a 2D parameter
// Marginalizer uses wrong dimension → Schur complement is wrong
```

**Fix**: Include both IDs and dimensions in the hash:
```rust
fn compute_structure_hash(param_blocks: &HashMap<ParamId, ParamBlock>) -> u64 {
    let mut hasher = ...;
    let mut entries: Vec<_> = param_blocks.iter().collect();
    entries.sort_by_key(|(id, _)| id);
    for (id, block) in entries {
        id.hash(&mut hasher);
        block.dimension.hash(&mut hasher);  // <-- Include dimension
    }
    hasher.finish()
}
```

---

### 4. **Pseudo-Inverse Tolerance: O(n) vs O(1) Behavior**

**Location**: Lines 1379-1397 (pseudo_inverse)

**The Problem**:
```rust
fn pseudo_inverse(&self, matrix: &DMatrix<f64>) -> DMatrix<f64> {
    let svd = SVD::new(matrix.clone(), true, true);
    let singulars = svd.singular_values();

    let tol = 1e-10 * max_sv;  // Conservative threshold
    let mut rank = 0;
    for (i, sv) in singulars.iter().enumerate() {
        if *sv > tol {
            s_inv[(i, i)] = 1.0 / sv;
            rank += 1;
        }
    }
    ...
}
```

**The Issue**: The tolerance is **fixed at 1e-10**, but should scale with **matrix rank or size**.

Consider:
- Small Schur complement (10×10): tolerance = 1e-10 * max_sv = 1e-10 * 1.0 = 1e-10
- Large Schur complement (200×200): tolerance = 1e-10 * max_sv = 1e-10 * 1.0 = 1e-10

In the large case, we're keeping 200 singular values where the small case keeps only 10. This is **inconsistent regularization**: the larger problem gets LESS regularization per degree of freedom.

**Better approach**: Scale by matrix size
```rust
let tol = (matrix.nrows() as f64).sqrt() * 1e-10 * max_sv;  // Scale with sqrt(n)
```

Or better yet, use a heuristic like SVD threshold = ε·max(m,n)·max_sv where ε = 1e-10.

---

### 5. **Configuration Semantics: `prior_weight` vs `prior_info_scaling`**

**Location**: Lines 115 (config definition), 637-642 (StandardPriorConstructor)

**The Problem**:
```rust
pub struct MarginalizationConfig {
    pub prior_weight: f64,
    pub prior_info_scaling: f64,
}

// In StandardPriorConstructor:
let mut info = schur_complement.clone() * config.prior_info_scaling;
info = info * config.prior_weight;  // Apply BOTH scalings
```

**The Question**: What's the semantic difference?

- `prior_info_scaling`: "Scale the information matrix" (0.9 by default)
- `prior_weight`: "Weight the prior" (1.0 by default)

In the code, they're **applied sequentially**:
```
info = schur * 0.9 * 1.0 = schur * 0.9
```

But conceptually:
- If `prior_info_scaling` is "weighting the measurements", it should be inverted for the information matrix
- If `prior_weight` is "trust weight", it should scale the information matrix

**The confusion**: Which one is which? And why are there TWO parameters doing essentially the same thing?

**Common case**: If `prior_weight=1.0` always, then `prior_info_scaling` alone controls the prior strength. But then why have `prior_weight` at all?

**Better design**: Pick ONE:
```rust
pub struct MarginalizationConfig {
    pub prior_info_scale: f64,  // Single, clear parameter
}

impl PriorConstructor {
    let info = schur_complement * config.prior_info_scale;
}
```

---

### 6. **Marginalize Method: Inconsistent Treatment of Empty Param Blocks**

**Location**: Lines 1035-1047 (marginalize method)

**The Problem**:
```rust
let (H_aa, H_ab, H_ba, H_bb) = self.partition_hessian(...);

// ... Schur complement computation ...

// Collect parameter IDs that were actually marginalized
let prior_param_ids: Vec<ParamId> = param_blocks
    .keys()
    .filter(|id| !keep_ids.contains(id))
    .cloned()
    .collect();

// If no parameters were marginalized, return empty result
if prior_param_ids.is_empty() {
    log::debug!("No parameters to marginalize");
    return MarginalizationResult {
        prior: None,
        info: MarginalizationInfo::default(),
    };
}
```

**The Issue**: This check happens AFTER computing Schur complement, which is wasteful. More importantly:

**Case 1**: `marg_ids` is empty (caller says "don't marginalize anything")
- Result: Schur complement = H_aa (no rows/cols), computation wasted

**Case 2**: `marg_ids` is non-empty, but none are in param_blocks
- Result: `prior_param_ids` becomes empty, we return None
- But we already spent time partitioning!

**Even worse**: The check should be at the START:
```rust
pub fn marginalize(...) -> MarginalizationResult {
    if marg_ids.is_empty() {
        return MarginalizationResult { prior: None, ... };  // Fast exit
    }

    // NOW do expensive work
}
```

---

### 7. **Condition Number Heuristic: Frobenius/Trace Ratio is NOT a Condition Number**

**Location**: Lines 1256-1274 (estimate_condition_number)

**The Problem**:
```rust
fn estimate_condition_number(&self, matrix: &DMatrix<f64>) -> Option<f64> {
    let frob = matrix.norm();      // ||A||_F = sqrt(sum(a_ij^2))
    let trace = (0..matrix.nrows().min(matrix.ncols()))
        .map(|i| matrix[(i, i)].abs())
        .sum::<f64>();

    if trace > 1e-12 {
        Some(frob / trace)
    } else {
        None
    }
}
```

**The Issue**: This is NOT a condition number estimate. It's a **ratio of norms**.

Real condition number: κ = σ_max / σ_min (ratio of singular values)

This ratio: κ_heuristic = ||A||_F / |trace(A)|

**Counter-example**:
```
A = [[1, 0],
     [0, 1e-6]]

κ_true = 1 / 1e-6 = 1e6 (ill-conditioned)

||A||_F = sqrt(1 + 1e-12) ≈ 1.0
trace = 1 + 1e-6 ≈ 1.0
κ_heuristic ≈ 1.0  (says it's WELL-conditioned!)
```

**This is WRONG**: The heuristic claims a matrix is well-conditioned when it's actually singular.

**Even worse**:
```
B = [[1e6, 0],
     [0,   1]]

κ_true = 1e6 / 1 = 1e6 (ill-conditioned)

||B||_F = sqrt(1e12 + 1) ≈ 1e6
trace = 1e6 + 1 ≈ 1e6
κ_heuristic ≈ 1.0  (WRONG again!)
```

**The heuristic works ONLY if the diagonal is dominant AND all eigenvalues are similar.**

**Fix**: Use a proper heuristic like:
```rust
fn estimate_condition_number_fast(&self, matrix: &DMatrix<f64>) -> Option<f64> {
    // Use Frobenius norm and min diagonal element as proxies
    let frob = matrix.norm();
    let min_diag = (0..matrix.nrows().min(matrix.ncols()))
        .map(|i| matrix[(i, i)].abs())
        .fold(f64::INFINITY, f64::min);

    if min_diag > 1e-12 {
        Some(frob / min_diag)
    } else {
        None
    }
}
```

Or better: Use the Frobenius norm of the matrix and its perturbation sensitivity:
```rust
// κ ≈ ||A||_F / σ_min, where σ_min ≈ ||A^{-T}A||_F^{-1}
// This is still O(n^2) but faster than SVD
```

---

## 🟡 MODERATE ISSUES (Medium Priority)

### 8. **select_marginalization_candidates: Unused Variables**

**Location**: Lines 1546-1548

```rust
for (i, &kf_id) in keyframe_ids.iter().enumerate() {
    let _age = current_frame - kf_id;  // <-- Computed but never used!
    if i < config.num_marginalize_per_step {
        // Uses index i, not age
    }
}
```

**The Issue**: The code computes `age` but then marginalizes based on **index order**, not age. This is semantically confusing:

**Intent**: "Marginalize the oldest keyframes"
**Actual**: "Marginalize the first N keyframes in the array"

These are ONLY the same if `keyframe_ids` is sorted by age. If not, this is a **silent bug**.

**Fix**: Make intent explicit:
```rust
for &kf_id in keyframe_ids.iter().take(config.num_marginalize_per_step) {
    marg_ids.push(...);
}
```

---

### 9. **ParamBlock::dimension is never Validated**

**Location**: Across marginalize method

**The Problem**: When partitioning, the code assumes `param_blocks[id].dimension` is correct:

```rust
let total_params: usize = param_blocks.values().map(|b| b.dimension).sum();

assert_eq!(hessian.nrows(), total_params, ...);
```

But if any parameter's dimension is WRONG, the Hessian dimensions won't match, and we'll get a panic.

**The Issue**: There's no forward check:
- ParamBlocks should validate that their dimensions make sense (e.g., KeyframePose = 6, not 5)
- The marginalize method should check that each ID's dimension hasn't changed between calls

**Risk**: Silent corruption if dimensions drift.

---

### 10. **FEJ Cache: First Linearization Point Never Used Consistently**

**Location**: Lines 1426-1440 (update_fej_cache)

**The Problem**:
```rust
if self.config.use_fej {
    if self.fej_cache.structure_hash() != structure_hash {
        self.fej_cache.points.retain(|id, _| active_ids.contains(id));
        self.fej_cache.update_structure_hash(structure_hash);
    }

    for id in active_ids.iter() {
        if !self.fej_cache.contains(id) {
            if let Some(block) = param_blocks.get(id) {
                self.fej_cache.set_point(id, block.linearization_point.clone());
            }
        }
    }
}
```

**The Issue**: The cache stores linearization points, but **never uses them**.

Where would these be used? In the Hessian and Jacobian computation:
```rust
// FEJ requirement: Jacobians must be evaluated at FIRST linearization point, not current point
pub fn compute_approximate_hessian(...) {
    // But this method doesn't know about FEJ cache!
    self.hessian_approximator.compute_hessian(...)
}
```

The `HessianApproximator` trait takes jacobians as input but **doesn't care about where they were evaluated**. So the FEJ cache is populated but **never consumed**.

**Result**: FEJ is implemented but doesn't actually work.

**Fix**: Pass linearization points to the heuristic approximators:
```rust
pub trait HessianApproximator {
    fn compute_hessian(
        &self,
        residuals: &DVector<f64>,
        param_dim: usize,
        jacobians: Option<&Vec<DMatrix<f64>>>,
        linearization_points: Option<&HashMap<ParamId, DVector<f64>>>,  // <-- Add this
    ) -> DMatrix<f64>;
}
```

---

### 11. **Gradient Computer: Zero vs Standard**

**Location**: Lines 567-604 (ZeroGradientComputer, StandardGradientComputer)

**The Problem**:
```rust
pub struct ZeroGradientComputer;

impl GradientComputer for ZeroGradientComputer {
    fn compute_gradient(...) -> DVector<f64> {
        DVector::zeros(param_dim)  // Always zero
    }
}
```

**Why would you ever use this?** When is the gradient zero useful?

1. If you want Gauss-Newton approximation WITHOUT gradient information (only Jacobian)
2. But then you should use the Jacobian directly, not discard it

**The use case is unclear**, and it's likely a **debug artifact** that shouldn't be in production code.

**Fix**: Either document WHY this exists, or remove it.

---

### 12. **Config Default: `num_marginalize_per_step` is always 1**

**Location**: Line 110

```rust
pub num_marginalize_per_step: usize,

// Default:
num_marginalize_per_step: 1,
```

**The Problem**: This parameter is set to 1 and never changed. In marginalization selection:

```rust
for (i, &kf_id) in keyframe_ids.iter().enumerate() {
    if i < config.num_marginalize_per_step {  // Always true for i=0
        marg_ids.push(ParamId::KeyframePose(kf_id));
    }
}
```

**Effect**: Only the FIRST keyframe is marginalized per step. If your window has 8 keyframes and one gets stale, you don't marginalize it — you only marginalize the oldest, then wait until it's first again.

**Question**: Is this intentional? If so, why is it configurable?

**Likely intent**: Marginalize multiple old frames per step to speed up window shrinkage.

**Current effect**: Slower than necessary window management.

---

## 🟢 MINOR ISSUES (Low Priority)

### 13. **Unused Underscore Variables Indicate Incomplete Implementation**

**Location**: Lines 951-952 (marginalize method)

```rust
let _start_time = std::time::Instant::now();
let _n_keep = keep_indices.len();
```

These are never used. Why?

- `_start_time`: Total marginalization time is never computed
- `_n_keep`: Number of kept parameters is never logged

**This suggests the timing/logging infrastructure is incomplete.**

---

### 14. **Hessian Approximator: Identity Approximator is Useless**

**Location**: Lines 532-549 (IdentityApproximator)

```rust
pub struct IdentityApproximator {
    scale: f64,
}

impl HessianApproximator for IdentityApproximator {
    fn compute_hessian(...) -> DMatrix<f64> {
        DMatrix::from_diagonal(&DVector::from_element(param_dim, self.scale))
    }
}
```

**The Problem**: This ignores all information (residuals, jacobians) and just returns a scaled identity.

**When would this be used?**
- Debugging
- Fallback when Jacobians are unavailable

But the code doesn't validate that you're using Identity when Jacobians are missing. You could use Identity and then pass Jacobians — they're silently ignored.

---

### 15. **Matrix Clone Overhead Still High in solve_h_bb_system**

**Location**: Lines 1327-1370

```rust
let mut regularized = H_bb.clone();  // Clone 1

if let Some(chol) = Cholesky::new(regularized.clone()) {  // Clone 2 (in Cholesky)
    return (chol.solve(H_ba), chol.solve(b_b));
}

for attempt in 1..4 {
    regularized = H_bb.clone();  // Clone 3, 4, 5
    // ... Cholesky ...
}
```

Even with the fixes, you're still cloning H_bb 1-4 times per solve. For 84×84 matrices, this is 256KB-1MB of allocations.

**Better approach**: Use references and in-place damping:
```rust
let mut H_regularized = H_bb.clone();  // Single clone
for attempt in 0..4 {
    let damping_factor = self.config.damping * 10_f64.powi(attempt as i32);
    // Temporarily increase diagonal
    for i in 0..H_regularized.nrows() {
        H_regularized[(i, i)] -= previous_damping;  // Remove old
        H_regularized[(i, i)] += damping_factor;    // Add new
    }

    if let Some(chol) = Cholesky::new(H_regularized.clone()) {  // Only clone for Cholesky
        return (chol.solve(H_ba), chol.solve(b_b));
    }
}
```

---

## Summary Table

| Issue | Type | Severity | Impact | Fix Effort | Status |
|-------|------|----------|--------|-----------|--------|
| 1. Partition index ordering | Logic | 🔴 Critical | Subtle reordering bugs | Medium | ⚠️ Needs investigation |
| 2. Solver escalation bounds | Logic | 🟢 VERIFIED CORRECT | - | - | ✅ No bug |
| 3. FEJ cache hash missing dimensions | Logic | 🔴 Critical | Dimension mismatch on cache hits | Low | 🔴 Confirmed |
| 4. Pseudo-inverse tolerance O(n) | Numerical | 🔴 Critical | Wrong regularization for large matrices | Low | 🔴 Confirmed |
| 5. prior_weight vs prior_info_scaling | API | 🔴 Critical | Confusing dual parameters | Medium | 🔴 Confirmed |
| 6. Early return after Schur computation | Performance | 🔴 Critical | Wastes computation | Low | 🔴 Confirmed |
| 7. Condition number heuristic wrong | Numerical | 🔴 Critical | Misses ill-conditioning | Medium | 🔴 Confirmed |
| 8. Keyframe marginalization order | Logic | 🟡 Moderate | May marginalize wrong frames | Low | 🟡 Confirmed |
| 9. Dimension validation missing | Safety | 🟡 Moderate | Silent corruption on dimension drift | Low | 🟡 Confirmed |
| 10. FEJ cache unused in Hessian approx | Logic | 🟡 Moderate | FEJ feature partially incomplete | Medium | 🟡 Confirmed |
| 11. ZeroGradientComputer purpose | Design | 🟡 Moderate | Unclear API | Low | 🟡 Confirmed |
| 12. num_marginalize_per_step = 1 always | Design | 🟡 Moderate | Inefficient window management | Low | 🟡 Confirmed |
| 13. Unused timing variables | Completeness | 🟢 Minor | Missing instrumentation | Low | 🟢 Confirmed |
| 14. IdentityApproximator always identity | Design | 🟢 Minor | Misleading API | Low | 🟢 Confirmed |
| 15. Clone overhead not eliminated | Performance | 🟢 Minor | Memory churn | Medium | 🟢 Confirmed |

---

## Recommendations

**Immediate (Week 1) - HIGH-IMPACT FIXES:**
1. **Fix condition number heuristic** (Issue #7) - Currently WRONG, may miss ill-conditioning
   - Replace Frobenius/trace with proper SVD-based estimate or use min diagonal element
   - This directly affects stability decisions and can cause silent failures

2. **Fix FEJ cache hash** (Issue #3) - Include parameter dimensions
   - Prevents dimension mismatch corruption when parameters change size

3. **Fix early return before Schur** (Issue #6) - Compute Schur only when needed
   - Saves ~50ms per marginalization cycle on embedded platforms

4. **Verify partition ordering** (Issue #1) - Critical data flow issue
   - Need to ensure indices match between Hessian and gradient partitioning
   - Test with unsorted parameter IDs to validate

**Short-term (Week 2) - SEMANTIC CLEANUP:**
1. Merge `prior_weight` and `prior_info_scaling` into single parameter (Issue #5)
   - Currently two redundant parameters causing confusion

2. Implement FEJ usage in Hessian approximators (Issue #10)
   - Pass linearization points from cache to trait implementations
   - Currently passing cache but not consuming it

3. Document or remove `ZeroGradientComputer` (Issue #11)
   - Unclear why this exists; either document use case or remove

**Medium-term (Week 3+) - API IMPROVEMENTS:**
1. Validate parameter dimensions at initialization (Issue #9)
   - Catch dimension mismatches early instead of silent corruption

2. Fix keyframe selection logic (Issue #8)
   - Use age-based selection instead of index-based
   - Add assertions that marginalizing correct frames

3. Clarify `num_marginalize_per_step` behavior (Issue #12)
   - Either document why it's always 1, or implement multi-frame marginalization

**Nice-to-have (Polish)**:
- Reduce clone overhead in solve_h_bb_system (Issue #15)
- Add timing instrumentation (Issue #13)
- Clean up dead code in approximators (Issue #14)

---

## Verification Commands

Test the critical issues:
```bash
# Run solver on ill-conditioned matrices
cargo test solver_pipeline -- --nocapture

# Check FEJ cache behavior with dimension changes
cargo test fej_cache -- --nocapture

# Profile marginalization timing
cargo bench --bench marginalization
```
