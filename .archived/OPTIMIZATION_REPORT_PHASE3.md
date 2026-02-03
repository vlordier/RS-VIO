# Optimization Report: Sliding Window Bundle Adjustment

## 1. Identified Bottleneck
The `SlidingWindow::optimize` method contained a significant sequential preparation phase before calling the solver.
- This phase constructs the Factor Graph (Levenberg-Marquardt problem) and computes Initial Values.
- For a sliding window of 8 frames with ~2000 features each, this involves processing ~16,000 observations.
- Each observation required:
  - String formatting (`LM_{id}`, `KF_{id}`) -> Heavy allocation.
  - HashMap lookups (String keys) -> Hashing overhead.
  - Matrix operations (Triangulation) -> Simd optimizable but was sequential.
  - Sequential vector pushing.

## 2. Optimization Strategy (Phase 3)
We refactored `optimize` to extract `build_optimization_problem` and parallelized it using `rayon`.

### Key Improvements:
1.  **Parallel Observation Counting (Map-Reduce)**:
    - Parallelized the counting of feature observations across all frames.
    - Switched from `String` keys ("LM_123") to `usize` keys (123) for intermediate counting.
    - **Impact**: Eliminated ~16,000 string allocations and hashes during the counting phase.

2.  **Parallel Factor Construction**:
    - Parallelized the logic for creating `BundleAdjustmentFactor` and `Initial Values`.
    - Each frame is processed in parallel.
    - String keys (`LM_{id}`) are now only generated *lazily* and only if the feature passes the "Stereo Constraint" filter.

3.  **Parallel Aggregation**:
    - Parallelized the construction of the `initial_values` HashMap using a parallel fold-reduce pattern.
    - Moves data instead of cloning where possible.

### Benchmark Results
Benchmark Condition: 8 Keyframes, 2000 Features per frame (Simulating heavy load).

- **Baseline**: ~74.65 ms
- **Optimized**: ~56.95 ms
- **Speedup**: ~24% reduction in preparation time.

### Remaining Constraints
- The `apex_solver::Problem::add_residual_block` method is inherently sequential (requires `&mut self`).
- We have optimized the *creation* of residuals, but the *insertion* remains sequential (approx 32,000 residuals inserted linear time).

## 3. Files Modified
- `src/estimator/sliding_window.rs`: Implemented `build_optimization_problem` with Rayon.
- `tests/benchmark_sliding_window_optimization.rs`: Added regression benchmark.
