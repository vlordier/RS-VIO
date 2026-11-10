#!/bin/bash
# Commit all measurement infrastructure
#
# Usage: ./scripts/commit_measurements.sh

set -e

cd /Users/vincent/Work/RS-VIO

echo "========================================================================"
echo "Committing Performance Measurement Infrastructure"
echo "========================================================================"
echo ""

# Show what will be committed
echo "Files to commit:"
git status --short
echo ""

# Commit
git add -A
git commit -m "feat: Add complete performance measurement infrastructure

Benchmark Suite:
- benches/motion_prior.rs: Criterion benchmarks for motion prior overhead
  * process_noise_scale() - 200 calls/sec (IMU rate)
  * gate_measurement() - 30 calls/sec (camera rate)
  * score_prediction() - 20 calls/sec (BA rate)
  * adaptive_huber - computed per BA iteration
  * total_prior_overhead - combined measurement

Accuracy Measurement:
- scripts/measure_prior_accuracy.py: Automated accuracy comparison
  * Runs VIO with/without priors
  * Compares ATE/RPE against ground truth
  * Reports improvement percentage
  * JSON output for scripting

All-in-One Script:
- scripts/run_all_measurements.sh: Complete measurement workflow
  * Runs all benchmarks (~8 min)
  * Runs accuracy comparison (~15 min/dataset)
  * Saves results to performance_results/
  * Generates summary report

Documentation:
- PERFORMANCE_MEASUREMENT_STATUS.md: What's measured vs estimated
- MOTION_PRIOR_OVERHEAD.md: Measurement plan and expected results
- BENCHMARK.md: Updated with measurement status table

Usage:
  # Run all measurements
  ./scripts/run_all_measurements.sh /path/to/dataset
  
  # Just motion prior overhead
  cargo bench --bench motion_prior
  
  # Just accuracy comparison
  python3 scripts/measure_prior_accuracy.py /path/to/dataset --json out.json

Expected Results:
- Motion prior overhead: <0.1% at 30 FPS
- Corridor1 improvement: 3-6× (0.61m → 0.1-0.2m)
- Corridor2 improvement: 2-4× (0.93m → 0.2-0.4m)
- Slides1: Eliminate resets (8.87m → <1m)"

echo ""
echo "✓ Commit complete"
echo ""
echo "To run measurements:"
echo "  ./scripts/run_all_measurements.sh /Users/vincent/data/tum-vi/dataset-corridor1_512_16"
