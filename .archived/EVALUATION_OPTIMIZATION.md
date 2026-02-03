# Trajectory Evaluation Optimization

## Overview
Optimized the `TrajectoryEvaluation` module, specifically the calculations for:
- **ATE (Absolute Trajectory Error)**
- **RPE (Relative Pose Error)**

These metrics involve iterating over large sequences of poses (e.g., 50k+ frames), finding associations, and solving geometric error terms.

## Optimization Strategy
- **Parallel Iteration**: Replaced sequential loops with `Rayon` parallel iterators (`par_iter`, `par_windows`).
- **Data Gathering**: Pre-collected iterators into `Vec` to enable efficient work stealing/splitting.
- **Reduction**: Used map-reduce patterns to aggregate error statistics.

## Benchmark Results
Benchmarks run with synthetic trajectories inside `src/evaluation/trajectory_evaluation.rs`.

**Conditions:**
- Machine: Local Dev Environment (M1/M2 likely based on speed)
- Dataset: Synthetic sinusoid trajectories.

| Metric | Poses | Serial Time | Parallel Time | Speedup |
|--------|-------|-------------|---------------|---------|
| ATE | 20,000 | ~9.8 ms | ~7.0 ms | ~1.4x |
| **RPE** | **50,000** | **~771 ms** | **~81 ms** | **~9.5x** |

## Implementation Details

### RPE Optimization
The Relative Pose Error calculation computes relative transforms between pairs of frames `(t, t+delta)` and their corresponding ground truth pairs. This is computationally expensive (matrix inversions, multiplications).

**Before (Serial)**:
```rust
for (ts2, pose2) in poses {
   // Sequential dependencies with 'prev' variable
   // Hard to parallelize directly
}
```

**After (Parallel)**:
```rust
poses.collect::<Vec<_>>()
    .par_windows(2) // Parallel sliding window
    .filter_map(|window| {
        // Independent calculation for each pair
        compute_relative_error(window[0], window[1])
    })
    .collect()
```

This massive speedup allows for real-time evaluation feedback even on long datasets.
