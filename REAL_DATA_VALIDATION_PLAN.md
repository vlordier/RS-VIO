# Real-World VIO-SLAM Validation Plan

Goal: Prove each stage works on real datasets using small slices. Keep artifacts (logs, CSVs, plots) for evidence.

## Datasets & Slices
- Select 3 short clips (10–30s) from existing datasets: indoor, outdoor, fast motion.
- For each clip: note sensor rates, known baseline/scale, and any ground truth available.

## Stage-by-Stage Checks (per clip)
1) Sensor & Timing
- Log frame/IMU timestamps, Δt histograms, IMU-to-frame lag; assert bounds.

2) Preprocess
- Reprojection error after undistort/rectify on known calibration; store residual histogram.

3) Features
- Track survival histogram (frames survived), inlier ratio per frame, flow residuals.

4) Visual Pose (PnP/2D-2D)
- Reprojection error per frame; pose jump detection; count RANSAC inliers vs total.

5) IMU Preintegration
- Gravity magnitude near 9.81 ± tol; bias drift bounded; delta pose drift over fixed Δt.

6) Fusion / Optimizer
- NEES/consistency on synthetic-plus-real blend (or zero-motion segments); residual mean≈0, χ² within bounds.

7) Marginalization
- Track state dimension after marg; Hessian condition number trend; BA residual drop after marg step.

8) Loop Closure (if enabled)
- Precision/recall on labeled loop pairs; pose graph cost before/after closure.

9) Trajectory Quality
- ATE/RPE vs ground truth (if available) or VO scale consistency check; landmark reprojection histogram.

## Evidence to Collect
- CSV/JSON per run: timestamps, inliers, residuals, NEES, cond. numbers, ATE/RPE.
- Plots: Δt histogram, reprojection hist, track survival, residual hist, ATE plot.
- Logs: summary stats per clip; store flamegraph if doing perf run.

## Commands (per clip)
- EuRoC: `cargo run --release --bin run_euroc -- <config.yaml> <dataset_root> --stats-out <out_dir>/euroc_stats.txt`
- TUM-VI: `cargo run --release --bin run_tum -- <config.yaml> <dataset_root> --stats-out <out_dir>/tum_stats.txt`
- 4Seasons: `cargo run --release --bin run_4seasons -- <config.yaml> <dataset_root> --stats-out <out_dir>/4seasons_stats.txt`
- Each run writes a human summary and `<name>_frames.csv` with per-frame processing times.
- Tests (sanity): `cargo test --lib`
- Coverage (optional): `cargo tarpaulin`

## Success Criteria (per clip)
- Timing: no dropped timestamps; IMU→frame lag within budget.
- Features: inlier ratio above threshold; median track length >= target.
- PnP/2D-2D: reprojection error < threshold; no pose spikes.
- IMU: gravity within tol; bias drift bounded.
- Fusion: residuals centered; NEES within CI.
- Marginalization: stable state size; Hessian cond. number bounded.
- Trajectory: ATE/RPE within target or scale stable.

## Next Actions
1) Pick 3 clips and record their args/ground truth availability.
2) Add logging/CSV outputs for the metrics above.
3) Run per-clip validation, archive CSVs/plots.
4) Summarize results in a short report (pass/fail vs criteria).
