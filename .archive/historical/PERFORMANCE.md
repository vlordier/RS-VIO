# Performance Baseline

This document captures performance metrics for the RS-VIO library as of the feature/realtime-performance branch.

## Test Suite Performance

All tests pass with clean compilation and no warnings.

### Unit & Integration Tests
- **Library tests:** 77 passed; 0 failed; **~0.56s**
- **Comprehensive integration tests:** 24 passed; 0 failed
- **End-to-end VIO tests:** 15 passed; 0 failed; **~3.7s**
- **Total test suite:** ~180+ tests; **all passing**

### Real Dataset Smoke Tests
- **EuRoC smoke tests:** 2 tests (skipped if `RS_VIO_EUROC_PATH` not set)
- **TUM-VI smoke tests:** 2 tests (skipped if `RS_VIO_TUMVI_PATH` not set)

## Performance Benchmarks

Benchmarks run on Release build (`-C opt-level=3 -C lto=fat`).

### Real-World Pipeline Performance
*Measured with `cargo bench --bench real_world_performance`*

#### Feature Tracking (Mono)
- **640×480 resolution:** ~7.68–7.95 ms per frame (~126–130 FPS)
- **320×240 resolution:** ~1.89–1.97 ms per frame (~508–528 FPS)
- **1280×720 resolution:** ~23.95–24.65 ms per frame (~41–42 FPS)

#### Feature Tracking (Stereo)
- **640×480 resolution:** ~9.25–9.54 ms per frame (~105–108 FPS)

### Low-Level Optimization Performance
*Measured with `cargo bench --bench performance_optimizations`*

#### Patch Operations
- **Residual computation (scalar):** ~28–29 ns (consistent across patch sizes)
- **Stats accumulation:** ~81–82 ns
- **Alternative stats variant:** ~22–23 ns

### Real Dataset Processing (4Seasons Sample)
*Benchmark run: 5257 frames from 4Seasons recording_2021-05-10*

**Per-Frame Breakdown:**
- Average processing time: **57.47 ms**
- Average frame rate: **17.4 FPS**
- Frame creation: ~0.05 ms
- Patch tracking: ~10.5 ms (±0.5 ms)
- Motion tracking: ~1.0 ms (±0.1 ms)
- Bundle adjustment: **~9.4 ms** (triggered on keyframes every ~4–5 frames)

**Total Dataset Time:** 5257 frames processed in ~304 seconds wall-clock time

## Interpretation

### Realtime Constraints
- For **30 FPS VIO:** need ~33 ms per frame
  - Stereo tracking alone (9.5 ms) + motion tracking (1 ms) + optimization amortization leaves ~22 ms headroom
  - 4Seasons results (17.4 FPS) suggest need for further optimization or parameter tuning
- For **20 FPS VIO:** need ~50 ms per frame
  - Current 4Seasons performance fits comfortably

### Bottlenecks
1. **Patch tracking:** ~10 ms (dominant non-optimization cost)
2. **Bundle adjustment:** ~9 ms per keyframe (sparse, but impactful)
3. **Motion tracking:** ~1 ms (efficient)

## CI Integration

- **Performance smoke tests:** Included in PR CI via lightweight bench runs (`--warm-up-time 1 --measurement-time 3`)
- **Full benchmarks:** Gated behind `ENABLE_FULL_BENCH` environment variable; run on-demand or main branch
- **Dataset benchmarks:** Gated behind `RS_VIO_*_PATH` environment variables; skipped by default in CI

## Future Tuning Opportunities

1. **Multi-scale feature tracking:** Hierarchical patch correlation to reduce per-frame cost
2. **Keyframe density tuning:** Adjust `translation_threshold` and `rotation_threshold` to reduce optimization frequency
3. **Motion tracking acceleration:** Use SIMD or GPU-accelerated optical flow
4. **Bundle adjustment pruning:** Limit landmark count in sliding window to maintain 30 FPS

## Last Updated
- **Date:** 2026-01-12
- **Branch:** feature/realtime-performance
- **Build:** Release with LTO enabled
- **Platform:** macOS (Apple Silicon)
