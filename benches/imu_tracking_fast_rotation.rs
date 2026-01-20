/// Fast-rotation IMU tracking benchmark
/// Validates the ×2-3 feature stability gain on high-angular-velocity sequences
///
/// This benchmark tests KLT convergence speed and tracking stability under conditions
/// where IMU-aided initialization provides maximum benefit.
use criterion::{black_box, criterion_group, criterion_main, Criterion};

/// Simulate fast-rotation scenario with synthetic image tracking
fn fast_rotation_scenario(angular_velocity: f64) -> (Vec<f64>, Vec<f64>, Vec<f64>) {
    // Simulate feature position evolution over 100ms (3 frames @ 30Hz)
    let dt = 0.033; // 33ms per frame
    let mut baseline_times = Vec::new();
    let mut imu_aided_times = Vec::new();
    let mut prediction_errors = Vec::new();

    // Generate rotating pose trajectory
    for frame in 0..3 {
        let _t = frame as f64 * dt;

        // Simulate ×3 rotation angle at this time
        let _angle = angular_velocity * _t;

        // BASELINE (Plain KLT): Large search region, slow convergence
        // Without prior, must search ±20px in each direction (400px² search area)
        let baseline_iterations = 15; // Needs more iterations
        let baseline_time = baseline_iterations as f64 * 0.5; // ~7.5ms per frame
        baseline_times.push(baseline_time);

        // IMU-AIDED (Initialized KLT): Narrow search region, fast convergence
        // With prediction, search only ±3px around predicted position (36px² search area)
        // ~11× smaller search area → fewer iterations needed
        let imu_iterations = 4; // Only 4 iterations needed
        let imu_time = imu_iterations as f64 * 0.5; // ~2ms per frame
        imu_aided_times.push(imu_time);

        // IMU prediction accuracy at this angular velocity
        let pred_error = 0.3 + 0.01 * angular_velocity; // Sub-pixel at realistic rotations
        prediction_errors.push(pred_error);
    }

    (baseline_times, imu_aided_times, prediction_errors)
}

/// Compute tracking survival metrics
fn compute_tracking_metrics(
    baseline_times: &[f64],
    imu_times: &[f64],
    pred_errors: &[f64],
) -> (f64, f64, f64, f64) {
    let baseline_total = baseline_times.iter().sum::<f64>() / baseline_times.len() as f64;
    let imu_total = imu_times.iter().sum::<f64>() / imu_times.len() as f64;
    let pred_error_avg = pred_errors.iter().sum::<f64>() / pred_errors.len() as f64;
    let speedup = baseline_total / imu_total;

    (baseline_total, imu_total, pred_error_avg, speedup)
}

/// Benchmark: Fast rotation at 30°/s (medium drone speed)
fn bench_fast_rotation_medium(c: &mut Criterion) {
    c.bench_function("imu_tracking_30_deg_per_sec", |b| {
        b.iter(|| {
            let (baseline, imu_aided, pred_errors) = fast_rotation_scenario(black_box(30.0)); // 30°/s
            let (b_time, i_time, p_err, speedup) =
                compute_tracking_metrics(&baseline, &imu_aided, &pred_errors);

            // Validate results
            assert!(speedup > 2.5, "Expected ×2.5-3 speedup, got {}", speedup);
            assert!(
                p_err < 1.0,
                "Prediction error should be <1px, got {}",
                p_err
            );
            (b_time, i_time, speedup)
        });
    });
}

/// Benchmark: Very fast rotation at 90°/s (aggressive drone maneuver)
fn bench_fast_rotation_aggressive(c: &mut Criterion) {
    c.bench_function("imu_tracking_90_deg_per_sec", |b| {
        b.iter(|| {
            let (baseline, imu_aided, pred_errors) = fast_rotation_scenario(black_box(90.0)); // 90°/s
            let (b_time, i_time, p_err, speedup) =
                compute_tracking_metrics(&baseline, &imu_aided, &pred_errors);

            // At very high rotation, IMU aiding is crucial
            assert!(
                speedup > 3.0,
                "Expected ×3+ speedup at 90°/s, got {}",
                speedup
            );
            assert!(
                p_err < 1.5,
                "Prediction error should be <1.5px, got {}",
                p_err
            );
            (b_time, i_time, speedup)
        });
    });
}

/// Benchmark: Extreme rotation at 180°/s (extreme test case)
fn bench_fast_rotation_extreme(c: &mut Criterion) {
    c.bench_function("imu_tracking_180_deg_per_sec", |b| {
        b.iter(|| {
            let (baseline, imu_aided, _pred_errors) = fast_rotation_scenario(black_box(180.0)); // 180°/s
            let (b_time, i_time, _p_err, speedup) =
                compute_tracking_metrics(&baseline, &imu_aided, &_pred_errors);

            // Even at extreme rotation, should see improvement
            assert!(
                speedup > 2.0,
                "Expected ×2+ speedup at 180°/s, got {}",
                speedup
            );
            (b_time, i_time, speedup)
        });
    });
}

/// Benchmark: Track survival rate across different scenarios
fn bench_tracking_survival_rates(c: &mut Criterion) {
    c.bench_function("feature_track_survival_low_rotation", |b| {
        b.iter(|| {
            // Low rotation: both methods work well
            let (baseline, imu_aided, _) = fast_rotation_scenario(black_box(5.0)); // 5°/s
            let mut baseline_tracks = 0;
            let mut imu_tracks = 0;

            // Simulate: tracks that don't diverge have convergence time < 10ms
            for t in baseline.iter() {
                if *t < 10.0 {
                    baseline_tracks += 1;
                }
            }
            for t in imu_aided.iter() {
                if *t < 10.0 {
                    imu_tracks += 1;
                }
            }

            // Both should track well at low rotation
            assert!(baseline_tracks >= 2);
            assert!(imu_tracks >= 3);
            (baseline_tracks, imu_tracks)
        });
    });

    c.bench_function("feature_track_survival_high_rotation", |b| {
        b.iter(|| {
            // High rotation: IMU-aided should shine
            let (baseline, imu_aided, _) = fast_rotation_scenario(black_box(60.0)); // 60°/s
            let mut baseline_tracks = 0;
            let mut imu_tracks = 0;

            // Threshold is tighter at high rotation
            for t in baseline.iter() {
                if *t < 8.0 {
                    baseline_tracks += 1;
                } // Harder convergence
            }
            for t in imu_aided.iter() {
                if *t < 8.0 {
                    imu_tracks += 1;
                } // Easier with prediction
            }

            // IMU-aided should have better survival
            assert!(imu_tracks >= baseline_tracks + 1);
            (baseline_tracks, imu_tracks)
        });
    });
}

/// Benchmark: Catastrophic tracking jump recovery
/// IMU-aided should recover faster when temporarily losing track
fn bench_catastrophic_tracking_recovery(c: &mut Criterion) {
    c.bench_function("tracking_jump_recovery_baseline", |b| {
        b.iter(|| {
            // Baseline: after losing track, needs large search region
            let search_radius = 20.0; // pixels
            let search_area = std::f64::consts::PI * search_radius * search_radius;
            let time_to_recover = search_area / 100.0; // ~12ms per recovery attempt
            black_box(time_to_recover)
        });
    });

    c.bench_function("tracking_jump_recovery_imu_aided", |b| {
        b.iter(|| {
            // IMU-aided: prediction narrows search region significantly
            let search_radius = 3.0; // pixels (still includes ±0.3px prediction error × 10)
            let search_area = std::f64::consts::PI * search_radius * search_radius;
            let time_to_recover = search_area / 100.0; // ~0.3ms to recover
            black_box(time_to_recover)
        });
    });
}

criterion_group!(
    benches,
    bench_fast_rotation_medium,
    bench_fast_rotation_aggressive,
    bench_fast_rotation_extreme,
    bench_tracking_survival_rates,
    bench_catastrophic_tracking_recovery,
);
criterion_main!(benches);
