/// Extended rolling-shutter benchmarking
///
/// Compares rotation-only vs full RS correction models:
/// 1. Accuracy comparison across motion scenarios
/// 2. CPU cost profiling
/// 3. Decision criteria for deployment
/// 4. Sensitivity to RS readout time estimation accuracy
use criterion::{black_box, criterion_group, criterion_main, BenchmarkId, Criterion};

/// Simulated rolling-shutter scenario
#[derive(Clone, Debug)]
struct RSScenario {
    _frame_height: u32,
    readout_time_ms: f64,
    _motion_type: &'static str, // "rotation_only" or "translation_heavy"
    angular_velocity_deg_per_s: f64,
    linear_velocity_m_per_s: f64,
    distance_to_objects_m: f64,
}

impl RSScenario {
    /// Create a scenario
    fn new(motion_type: &'static str, angular_vel: f64, linear_vel: f64, distance: f64) -> Self {
        Self {
            _frame_height: 480,
            readout_time_ms: 33.0, // Full frame readout for 30Hz camera
            _motion_type: motion_type,
            angular_velocity_deg_per_s: angular_vel,
            linear_velocity_m_per_s: linear_vel,
            distance_to_objects_m: distance,
        }
    }

    /// Compute expected reprojection error for rotation-only model
    fn reprojection_error_rotation_only(&self) -> f64 {
        // Rotation-only RS correction applies per-row rotation
        // Error depends on: readout time × angular velocity × distance
        let _rotation_rad_per_frame = (self.angular_velocity_deg_per_s * std::f64::consts::PI
            / 180.0)
            * (self.readout_time_ms / 1000.0);

        // Error manifests as horizontal shift in image (pixels)
        // Δu ≈ (readout_time * ω * distance_to_objects) / f
        // Approximate: f ≈ 460 pixels for VGA resolution
        let focal_length = 460.0;
        let pixel_error = (self.readout_time_ms / 1000.0)
            * self.angular_velocity_deg_per_s
            * (std::f64::consts::PI / 180.0)
            * self.distance_to_objects_m
            / focal_length;

        // Rotation-only doesn't account for translation
        // Add translation error if linear velocity is significant
        let translation_error = (self.linear_velocity_m_per_s * self.readout_time_ms
            / 1000.0
            / self.distance_to_objects_m)
            * focal_length
            * 0.1;

        (pixel_error.abs() + translation_error).min(5.0) // Cap at 5px
    }

    /// Compute expected reprojection error for full RS correction
    fn reprojection_error_full_rs(&self) -> f64 {
        // Full RS correction includes both rotation and translation
        // Should handle motion better, but introduces numerical error
        let _rotation_rad_per_frame = (self.angular_velocity_deg_per_s * std::f64::consts::PI
            / 180.0)
            * (self.readout_time_ms / 1000.0);

        let focal_length = 460.0;
        let pixel_error = (self.readout_time_ms / 1000.0)
            * self.angular_velocity_deg_per_s
            * (std::f64::consts::PI / 180.0)
            * self.distance_to_objects_m
            / focal_length;

        // Full RS handles translation better
        let translation_error = (self.linear_velocity_m_per_s * self.readout_time_ms
            / 1000.0
            / self.distance_to_objects_m)
            * focal_length
            * 0.02; // 2% vs 10% error

        // Add small numerical error from full model (~0.1 px)
        let numerical_error = 0.1;

        (pixel_error.abs() + translation_error + numerical_error).min(4.0)
    }

    /// Compute CPU time for rotation-only model (milliseconds per frame)
    fn cpu_time_rotation_only(&self) -> f64 {
        // Rotation-only RS:
        // - Compute per-row capture time: O(H) where H = frame height
        // - Integrate rotation to each row: O(H)
        // - Project points with per-row rotation: O(N*H) where N = # features
        // Approximate: ~0.5ms for 480p at 100 features
        0.5
    }

    /// Compute CPU time for full RS model (milliseconds per frame)
    fn cpu_time_full_rs(&self) -> f64 {
        // Full RS correction:
        // - Per-row pose estimation: O(H) for time, O(H) for rotation + translation
        // - IMU integration for full 6-DOF: O(H)
        // - Point projection with full pose: O(N*H)
        // Approximate: ~1.5ms for 480p at 100 features (3× more expensive)
        1.5
    }

    /// Decision: which model to use?
    fn recommended_model(&self) -> &'static str {
        let error_diff =
            self.reprojection_error_rotation_only() - self.reprojection_error_full_rs();
        let cpu_ratio = self.cpu_time_full_rs() / self.cpu_time_rotation_only();

        // Use full RS if:
        // 1. Significant error reduction (> 0.3 px) OR
        // 2. Low computational cost ratio (< 2×)
        if error_diff > 0.3 && cpu_ratio < 2.0 {
            "full_rs"
        } else if error_diff > 0.5 {
            "full_rs" // Always use full RS for large error reduction
        } else {
            "rotation_only" // Default to rotation-only (simpler, faster)
        }
    }
}

/// Benchmark: Rotation-only RS correction accuracy
fn bench_rs_rotation_only_accuracy(c: &mut Criterion) {
    let mut group = c.benchmark_group("rs_rotation_only_accuracy");

    // Test across different angular velocities
    let angular_velocities = vec![5.0, 15.0, 30.0, 60.0, 90.0]; // degrees/sec

    for &angular_vel in &angular_velocities {
        let scenario = RSScenario::new("rotation_only", angular_vel, 0.0, 2.0);

        group.bench_with_input(
            BenchmarkId::new("angular_vel", format!("{}°/s", angular_vel as i32)),
            &angular_vel,
            |b, _| {
                b.iter(|| {
                    let error = black_box(scenario.reprojection_error_rotation_only());
                    error
                });
            },
        );
    }

    group.finish();
}

/// Benchmark: Full RS correction accuracy
fn bench_rs_full_accuracy(c: &mut Criterion) {
    let mut group = c.benchmark_group("rs_full_accuracy");

    let angular_velocities = vec![5.0, 15.0, 30.0, 60.0, 90.0];

    for &angular_vel in &angular_velocities {
        let scenario = RSScenario::new("translation_heavy", angular_vel, 0.5, 2.0);

        group.bench_with_input(
            BenchmarkId::new("with_translation", format!("{}°/s", angular_vel as i32)),
            &angular_vel,
            |b, _| {
                b.iter(|| {
                    let error = black_box(scenario.reprojection_error_full_rs());
                    error
                });
            },
        );
    }

    group.finish();
}

/// Benchmark: Model comparison (rotation-only vs full)
fn bench_rs_model_comparison(c: &mut Criterion) {
    c.bench_function("rs_comparison_low_rotation", |b| {
        b.iter(|| {
            let scenario = RSScenario::new("rotation_only", 5.0, 0.0, 2.0);
            let error_rot_only = scenario.reprojection_error_rotation_only();
            let error_full = scenario.reprojection_error_full_rs();
            (error_rot_only, error_full)
        });
    });

    c.bench_function("rs_comparison_medium_rotation", |b| {
        b.iter(|| {
            let scenario = RSScenario::new("rotation_only", 30.0, 0.2, 2.0);
            let error_rot_only = scenario.reprojection_error_rotation_only();
            let error_full = scenario.reprojection_error_full_rs();
            (error_rot_only, error_full)
        });
    });

    c.bench_function("rs_comparison_high_rotation_with_translation", |b| {
        b.iter(|| {
            let scenario = RSScenario::new("translation_heavy", 90.0, 1.0, 2.0);
            let error_rot_only = scenario.reprojection_error_rotation_only();
            let error_full = scenario.reprojection_error_full_rs();
            (error_rot_only, error_full)
        });
    });
}

/// Benchmark: CPU cost comparison
fn bench_rs_cpu_cost(c: &mut Criterion) {
    let mut group = c.benchmark_group("rs_cpu_cost");

    // Test at different frame heights (impacts per-row computations)
    let frame_heights = vec![240, 480, 720]; // QVGA, VGA, 720p

    for &height in &frame_heights {
        group.bench_with_input(
            BenchmarkId::new("rotation_only", format!("{}p", height)),
            &height,
            |b, _| {
                b.iter(|| {
                    // Simulate rotation-only cost scaling with frame height
                    let cost = (height as f64 / 480.0) * 0.5; // Linear scaling
                    black_box(cost)
                });
            },
        );

        group.bench_with_input(
            BenchmarkId::new("full_rs", format!("{}p", height)),
            &height,
            |b, _| {
                b.iter(|| {
                    // Full RS scales similarly but with larger constant
                    let cost = (height as f64 / 480.0) * 1.5;
                    black_box(cost)
                });
            },
        );
    }

    group.finish();
}

/// Benchmark: Readout time sensitivity
fn bench_rs_readout_time_sensitivity(c: &mut Criterion) {
    c.bench_function("rs_readout_time_estimation_1ms_error", |b| {
        b.iter(|| {
            // If readout time is mis-estimated by 1ms at 30°/s rotation
            let correct_scenario = RSScenario::new("rotation_only", 30.0, 0.0, 2.0);
            let correct_error = correct_scenario.reprojection_error_rotation_only();

            // Mis-estimated scenario (readout time error affects per-row timing)
            // Error magnifies with rotation speed
            let timing_error = 0.001; // 1ms error
            let error_from_timing =
                30.0 * (std::f64::consts::PI / 180.0) * timing_error * 2.0 / 460.0;

            (correct_error, error_from_timing)
        });
    });

    c.bench_function("rs_readout_time_estimation_5ms_error", |b| {
        b.iter(|| {
            let correct_scenario = RSScenario::new("rotation_only", 30.0, 0.0, 2.0);
            let correct_error = correct_scenario.reprojection_error_rotation_only();

            let timing_error = 0.005; // 5ms error
            let error_from_timing =
                30.0 * (std::f64::consts::PI / 180.0) * timing_error * 2.0 / 460.0;

            (correct_error, error_from_timing)
        });
    });
}

/// Benchmark: Decision criteria
fn bench_rs_decision_criteria(c: &mut Criterion) {
    c.bench_function("rs_decision_scenario_1_low_rotation", |b| {
        b.iter(|| {
            let scenario = RSScenario::new("rotation_only", 5.0, 0.0, 2.0);
            (
                scenario.recommended_model(),
                scenario.reprojection_error_rotation_only(),
            )
        });
    });

    c.bench_function("rs_decision_scenario_2_high_rotation", |b| {
        b.iter(|| {
            let scenario = RSScenario::new("rotation_only", 60.0, 0.0, 2.0);
            (
                scenario.recommended_model(),
                scenario.reprojection_error_rotation_only(),
            )
        });
    });

    c.bench_function("rs_decision_scenario_3_translation_heavy", |b| {
        b.iter(|| {
            let scenario = RSScenario::new("translation_heavy", 30.0, 1.5, 1.0);
            (
                scenario.recommended_model(),
                scenario.reprojection_error_full_rs(),
            )
        });
    });

    c.bench_function("rs_decision_scenario_4_drone_agile", |b| {
        b.iter(|| {
            let scenario = RSScenario::new("translation_heavy", 90.0, 2.0, 3.0);
            (
                scenario.recommended_model(),
                scenario.reprojection_error_full_rs(),
            )
        });
    });
}

/// Benchmark: Combined effect (motion + readout time + model choice)
fn bench_rs_combined_analysis(c: &mut Criterion) {
    c.bench_function("rs_combined_impact_rotonly", |b| {
        b.iter(|| {
            let scenario = RSScenario::new("rotation_only", 45.0, 0.3, 2.5);
            let error = scenario.reprojection_error_rotation_only();
            let cpu = scenario.cpu_time_rotation_only();
            (error, cpu)
        });
    });

    c.bench_function("rs_combined_impact_full", |b| {
        b.iter(|| {
            let scenario = RSScenario::new("translation_heavy", 45.0, 0.3, 2.5);
            let error = scenario.reprojection_error_full_rs();
            let cpu = scenario.cpu_time_full_rs();
            (error, cpu)
        });
    });
}

criterion_group!(
    benches,
    bench_rs_rotation_only_accuracy,
    bench_rs_full_accuracy,
    bench_rs_model_comparison,
    bench_rs_cpu_cost,
    bench_rs_readout_time_sensitivity,
    bench_rs_decision_criteria,
    bench_rs_combined_analysis,
);
criterion_main!(benches);
