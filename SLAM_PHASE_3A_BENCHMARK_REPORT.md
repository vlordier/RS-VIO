# SLAM Phase 3A Benchmark Report

This report tracks full-frame benchmarks on TUM VI using the shared trajectory evaluation module (ATE/RPE) and ground truth from `mav0/state_groundtruth_estimate0/data.tum`.

## How to run
- Set dataset root: `export RS_VIO_TUMVI_PATH=/absolute/path/to/tum_vi_sequence`
- Run full benchmark (all frames): `cargo test --tests -- --nocapture test_slam_vs_vio_benchmarking`
- Outputs are printed in-console; copy results back into the table below.

## Results (fill after run)
| Sequence | Frames | VIO ATE RMSE (m) | SLAM ATE RMSE (m) | ATE Δ (%) | VIO RPE Trans RMSE (m) | SLAM RPE Trans RMSE (m) | VIO RPE Rot RMSE (deg) | SLAM RPE Rot RMSE (deg) | Loop Closures | Notes |
|---|---|---|---|---|---|---|---|---|---|---|
| <name> | <n> | <v> | <s> | <impr> | <v> | <s> | <v> | <s> | <count> | <observations> |

## Methodology
- Uses `tests/slam_phase2c_benchmarking.rs` with the shared evaluation module (`calculate_ate`, `calculate_rpe`).
- Processes every frame in the sequence (no truncation) for both VIO (sliding window) and SLAM (global optimizer with periodic triggering).
- Ground truth: parsed from TUM VI `data.tum` (timestamp tx ty tz qx qy qz qw).
- RPE delta time inferred from dataset timestamps (first spacing, default 33 ms).

## Next steps
- Run on target sequences and record metrics above.
- Optionally add plot exports (ATE/RPE curves) after the first run if needed.
