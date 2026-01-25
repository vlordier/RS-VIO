# CPU-Only Realtime Plan

Goal: Achieve realtime VIO/SLAM on CPU-only hardware. Today’s bottleneck is per-frame estimator time; I/O and IMU are negligible.

## Current Status (TUM-VI room1)
- Config: `config/tum_vi_realtime_cpu.yaml` (512x512)
- Test: `cargo test --release --tests -- --nocapture test_slam_vs_vio_benchmarking`
- Env: `RS_VIO_TUMVI_PATH=...`, `RUSTFLAGS=-C target-cpu=native`, `RUST_LOG=warn`

Results:
- Stride=2: ~8.3–8.5 fps processed (not realtime for 20 Hz → 10 Hz decimated).
- Stride=3: ~12.8–12.9 fps processed (realtime for 20 Hz → 6.7 Hz decimated).
- Accuracy (150 frames): ATE RMSE ~0.46–0.60 m. RPE reported 0.0, likely a measurement bug to investigate.

## Short-Term Options (No Code Changes)
- Accept stride=3 for a realtime stopgap at ~6–7 Hz output.
- Try the more aggressive config `config/tum_vi_realtime_cpu_extreme.yaml`:
  - Window 8 → less BA work
  - Fewer features (~36 total) and reduced OF iterations → less tracking work
  - PnP/BA iterations = 1 → less solver time
  - Damping 1e-5 → stability with fewer iterations

Commands:
- Stride=2: `RS_VIO_CONFIG_PATH=config/tum_vi_realtime_cpu_extreme.yaml RS_VIO_FRAME_STRIDE=2 ...`
- Stride=3: `RS_VIO_CONFIG_PATH=config/tum_vi_realtime_cpu.yaml RS_VIO_FRAME_STRIDE=3 ...`

## Engineering Plan (Code-Level Speedups)
1) Parallelism (Rayon)
- Parallelize per-frame independent work:
  - Image pyramid construction across levels
  - Grid-based feature detection/tracking across cells
  - Stereo match evaluation across candidate pairs
  - Residual/Jacobian evaluation across landmarks and observations

2) SIMD Acceleration
- Use `std::simd` or explicit SSE/AVX for:
  - Warp/resample, gradient and patch operations
  - Projection residuals and Jacobians
  - Robust loss evaluation (Huber/soft L1)

3) Memory & Allocation
- Preallocate and reuse buffers for features, tracks, pyramids, residuals
- Use structure-of-arrays for tight inner loops
- Avoid per-frame small allocations (Vec::with_capacity, arenas, bump allocators)

4) Keyframes & Decimation
- Increase keyframe thresholds so non-keyframes do IMU-only propagation + lightweight tracking
- Run BA on keyframes only; skip or downscale optimization on non-keyframes
- Evaluate lower-cost marginalization schedule (less frequent, smaller priors)

5) Solver Optimizations
- Keep diagonal Hessian for speed; consider block-diagonal if available
- Early stopping on small gradient/norm
- Reuse linearization state between frames where possible

6) Algorithmic Levers (If quality holds)
- Reduce features per grid further; increase Huber delta for robustness
- Downscale internal processing (build a 256–384 px pyramid level for tracking) while keeping 512px for geometry if required by loader

## Validation Plan
- Throughput: target ≥10 fps processed at stride=2 over 500+ frames
- Quality: ATE/RPE comparable to current realtime profile; verify orientation error trend
- Long-run stability: run 1000+ frames with no divergence

## Known Follow-ups
- RPE prints 0.0: inspect calculation inputs and sampling logic.
- Document “realtime” definitions: processed fps ≥ input fps after any decimation.

## Next Steps
- Pick target output rate (e.g., 10 Hz minimum). If 6–7 Hz is acceptable short-term, use stride=3 now.
- I can start implementing Rayon-based parallel loops in feature tracking and residual evaluation, then re-benchmark.
