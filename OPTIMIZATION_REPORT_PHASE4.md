# Optimization Report: Stereo Feature Tracker Parallelization

## 1. Identified Bottleneck
The `StereoPatchTracker::process_frame` function performs two major independent tasks sequentially:
1.  **Image Pyramid Construction**: Builds multi-level image pyramids for both Left and Right cameras.
2.  **Temporal Tracking**: Tracks existing features from the previous frame to the current frame for both Left and Right cameras independently.

In a stereo setup, these operations are symmetric and independent, making them ideal candidates for Task Parallelism.

## 2. Optimization Strategy (Phase 4)
We utilized `rayon::join` to execute symmetric tasks in parallel.

### Key Improvements:
1.  **Parallel Pyramid Construction**:
    - Simultaneously builds the Gaussian pyramid for Left and Right images.
    - Although individual pyramid levels were already built in parallel, running both pyramids concurrently increases CPU saturation, especially for low level counts (3-5 levels).

2.  **Parallel Temporal Tracking**:
    - Simultaneously executes Lukas-Kanade optical flow for Left and Right camera streams.
    - Each stream tracks ~1000-2000 points.
    - This effectively halves the latency of the tracking phase, which is often the dominant cost in the frontend.

### Benchmark Results
Benchmark Condition: Synthetic image sequence, 640x480 resolution, 4 pyramid levels.

- **Baseline**: ~155.10 ms per frame
- **Optimized**: ~96.24 ms per frame
- **Speedup**: ~1.61x (38% execution time reduction)

## 3. Files Modified
- `src/feature_tracker/feature_tracker.rs`: Implemented `rayon::join` for pyramid building and tracking.
- `tests/benchmark_stereo_tracker.rs`: Added micro-benchmark for tracking performance.
