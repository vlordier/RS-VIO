# Feature Detection Optimization

## Overview
As part of the CPU parallelization initiative, the `detect_key_points` function in `src/feature_tracker/image_utilities.rs` was identified as a bottleneck. This function performs FAST corner detection on a grid over the entire image.

## Optimization Strategy
- **Parallelization**: Converted the nested loops (iterating over grid cells) into a parallel iterator task list.
- **Library**: Used `rayon` for data parallelism.
- **Granularity**: Each grid cell (e.g., 30x30 pixels) is processed independently.

## Benchmark Results
Unit test: `benchmark_detect_key_points_performance` in `src/feature_tracker/image_utilities.rs`.

**Conditions:**
- Image Size: 1200x800 (Euroc/Standard resolution)
- Grid Size: 30px
- Activity: 20% of cells pre-occupied, others running FAST detection.

**Performance:**
- **Serial (baseline)**: ~240 ms
- **Parallel (Rayon)**: ~28 ms
- **Speedup**: ~8.5x on local machine.

## Implementation Details
The grid traversal was refactored from:
```rust
for x in (x_start..x_stop) {
    for y in (y_start..y_stop) {
        // ...
        corners_fast9(...)
    }
}
```
To:
```rust
let tasks: Vec<_> = ...; // Collect valid cells
tasks.par_iter().map(|(x,y)| {
    // ...
    corners_fast9(...)
}).collect()
```
