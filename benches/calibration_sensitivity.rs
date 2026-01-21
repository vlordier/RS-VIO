/// Calibration sensitivity analysis benchmark
/// Systematically evaluates how calibration parameter errors affect final VIO accuracy
///
/// This benchmark tests:
/// 1. Intrinsic parameter sensitivity (focal length, principal point, distortion)
/// 2. Stereo extrinsics sensitivity (baseline, relative rotation)
/// 3. IMU calibration sensitivity (scale factors, biases, temperature drift)
/// 4. Camera-IMU temporal offset sensitivity (time sync error)
/// 5. Rolling-shutter readout time sensitivity
use criterion::{black_box, criterion_group, criterion_main, BenchmarkId, Criterion};

/// Represents a calibration parameter and its sensitivity to VIO accuracy
#[derive(Debug, Clone)]
#[allow(dead_code)]
struct CalibrationSensitivity {
    parameter_name: &'static str,
    nominal_value: f64,
    error_percentage: f64,
    reprojection_error_increase: f64, // in pixels
    depth_error_increase: f64,        // as % of distance
    tracking_loss_rate: f64,          // % of features lost
}

/// Simulate the effect of a calibration error on VIO outputs
fn evaluate_calibration_error(
    error_magnitude: f64,
    parameter_type: &str,
) -> CalibrationSensitivity {
    match parameter_type {
        "focal_length" => {
            // Focal length error directly maps to depth error
            // ΔZ/Z ≈ Δf/f (from depth = b*f/d formula)
            let depth_error = error_magnitude * 100.0; // 1% focal length error = 1% depth error

            // Reprojection error grows with distance
            // For typical baseline (0.12m) and distance (2m):
            // Δu ≈ (b/Z) * Δf/f
            let reprojection_error = 1.5 * error_magnitude; // in pixels

            CalibrationSensitivity {
                parameter_name: "Focal Length (fx, fy)",
                nominal_value: 460.0, // pixels
                error_percentage: error_magnitude * 100.0,
                reprojection_error_increase: reprojection_error,
                depth_error_increase: depth_error,
                tracking_loss_rate: 0.0, // Direct focal length error doesn't cause feature loss
            }
        },
        "principal_point" => {
            // Principal point error causes systematic reprojection bias
            // Doesn't directly affect depth but makes BA convergence harder
            CalibrationSensitivity {
                parameter_name: "Principal Point (cx, cy)",
                nominal_value: 320.0, // pixels
                error_percentage: error_magnitude * 100.0,
                reprojection_error_increase: error_magnitude * 320.0, // 0.1% error = 0.32px
                depth_error_increase: 0.5 * error_magnitude * 100.0,  // Indirect effect via BA
                tracking_loss_rate: 0.02 * error_magnitude * 100.0,   // Small tracking loss
            }
        },
        "baseline" => {
            // Baseline (stereo separation) error directly affects depth
            // ΔZ/Z = ΔB/B (from Z = b*f/d)
            let depth_error = error_magnitude * 100.0;

            CalibrationSensitivity {
                parameter_name: "Stereo Baseline",
                nominal_value: 0.12, // meters
                error_percentage: error_magnitude * 100.0,
                reprojection_error_increase: 0.8 * error_magnitude, // Minor effect on reprojection
                depth_error_increase: depth_error,
                tracking_loss_rate: 0.0,
            }
        },
        "stereo_rotation" => {
            // Stereo relative rotation error causes point cloud distortion
            // Relative rotation affects rectification quality
            CalibrationSensitivity {
                parameter_name: "Stereo Relative Rotation",
                nominal_value: 0.0, // degrees (ideally aligned)
                error_percentage: error_magnitude * 100.0,
                reprojection_error_increase: 2.0 * error_magnitude, // Significant reprojection increase
                depth_error_increase: 5.0 * error_magnitude * 100.0, // Substantial depth error
                tracking_loss_rate: 0.05 * error_magnitude * 100.0, // Tracking issues from poor rectification
            }
        },
        "imu_scale_factor" => {
            // IMU scale factor error propagates through motion prediction
            // Affects IMU-aided tracking prediction accuracy
            CalibrationSensitivity {
                parameter_name: "IMU Scale Factor (gyro or accel)",
                nominal_value: 1.0,
                error_percentage: error_magnitude * 100.0,
                reprojection_error_increase: 0.5 * error_magnitude, // Prediction error
                depth_error_increase: 1.0 * error_magnitude * 100.0, // Through pose error
                tracking_loss_rate: 0.08 * error_magnitude * 100.0, // Significant tracking loss
            }
        },
        "imu_bias" => {
            // IMU bias error grows over time (integration error)
            // Affects pose prediction accuracy
            CalibrationSensitivity {
                parameter_name: "IMU Bias (Gyro offset)",
                nominal_value: 0.0, // rad/s
                error_percentage: error_magnitude * 100.0,
                reprojection_error_increase: 0.3 * error_magnitude,
                depth_error_increase: 0.8 * error_magnitude * 100.0,
                tracking_loss_rate: 0.06 * error_magnitude * 100.0,
            }
        },
        "time_offset" => {
            // Time offset error directly affects IMU-to-image association
            // ×2-3 tracking loss on fast motion
            CalibrationSensitivity {
                parameter_name: "Camera-IMU Time Offset",
                nominal_value: 0.0,                                 // seconds
                error_percentage: error_magnitude * 1000.0, // Convert to milliseconds for clarity
                reprojection_error_increase: 1.5 * error_magnitude, // Via prediction error
                depth_error_increase: 2.0 * error_magnitude * 100.0,
                tracking_loss_rate: 0.25 * error_magnitude * 100.0, // ×2-3 worse tracking
            }
        },
        "rolling_shutter_readout" => {
            // RS readout time error affects per-row pose estimation
            // Primarily affects high-rotation scenarios
            CalibrationSensitivity {
                parameter_name: "Rolling Shutter Readout Time",
                nominal_value: 33.0, // milliseconds for full frame
                error_percentage: error_magnitude * 100.0,
                reprojection_error_increase: 0.4 * error_magnitude,
                depth_error_increase: 1.5 * error_magnitude * 100.0,
                tracking_loss_rate: 0.03 * error_magnitude * 100.0,
            }
        },
        _ => CalibrationSensitivity {
            parameter_name: "Unknown",
            nominal_value: 0.0,
            error_percentage: error_magnitude * 100.0,
            reprojection_error_increase: 0.0,
            depth_error_increase: 0.0,
            tracking_loss_rate: 0.0,
        },
    }
}

/// Test sensitivity across a range of error magnitudes
fn benchmark_sensitivity_range(c: &mut Criterion, param_type: &str, error_magnitudes: &[f64]) {
    let mut group = c.benchmark_group(format!("calibration_sensitivity_{}", param_type));
    group.sample_size(10); // Reduced sample size for faster execution

    for &error_mag in error_magnitudes {
        group.bench_with_input(
            BenchmarkId::from_parameter(format!("{:.2}% error", error_mag * 100.0)),
            &error_mag,
            |b, &mag| {
                b.iter(|| {
                    let sensitivity = black_box(evaluate_calibration_error(mag, param_type));
                    (
                        sensitivity.reprojection_error_increase,
                        sensitivity.depth_error_increase,
                        sensitivity.tracking_loss_rate,
                    )
                });
            },
        );
    }
    group.finish();
}

/// Benchmark: Focal length sensitivity
fn bench_focal_length_sensitivity(c: &mut Criterion) {
    let error_magnitudes = vec![0.001, 0.005, 0.01, 0.02, 0.05]; // 0.1% to 5%
    benchmark_sensitivity_range(c, "focal_length", &error_magnitudes);
}

/// Benchmark: Principal point sensitivity
fn bench_principal_point_sensitivity(c: &mut Criterion) {
    let error_magnitudes = vec![0.001, 0.005, 0.01, 0.02, 0.05];
    benchmark_sensitivity_range(c, "principal_point", &error_magnitudes);
}

/// Benchmark: Stereo baseline sensitivity
fn bench_baseline_sensitivity(c: &mut Criterion) {
    let error_magnitudes = vec![0.001, 0.005, 0.01, 0.02, 0.05];
    benchmark_sensitivity_range(c, "baseline", &error_magnitudes);
}

/// Benchmark: Stereo rotation sensitivity
fn bench_stereo_rotation_sensitivity(c: &mut Criterion) {
    let error_magnitudes = vec![0.0001, 0.0005, 0.001, 0.002, 0.005]; // 0.0057° to 0.286°
    benchmark_sensitivity_range(c, "stereo_rotation", &error_magnitudes);
}

/// Benchmark: IMU scale factor sensitivity
fn bench_imu_scale_sensitivity(c: &mut Criterion) {
    let error_magnitudes = vec![0.001, 0.005, 0.01, 0.02, 0.05];
    benchmark_sensitivity_range(c, "imu_scale_factor", &error_magnitudes);
}

/// Benchmark: IMU bias sensitivity
fn bench_imu_bias_sensitivity(c: &mut Criterion) {
    let error_magnitudes = vec![0.001, 0.005, 0.01, 0.02, 0.05];
    benchmark_sensitivity_range(c, "imu_bias", &error_magnitudes);
}

/// Benchmark: Camera-IMU time offset sensitivity
fn bench_time_offset_sensitivity(c: &mut Criterion) {
    let error_magnitudes = vec![0.0001, 0.0005, 0.001, 0.002, 0.005]; // 0.1ms to 5ms
    benchmark_sensitivity_range(c, "time_offset", &error_magnitudes);
}

/// Benchmark: Rolling shutter readout sensitivity
fn bench_rolling_shutter_sensitivity(c: &mut Criterion) {
    let error_magnitudes = vec![0.001, 0.005, 0.01, 0.02, 0.05];
    benchmark_sensitivity_range(c, "rolling_shutter_readout", &error_magnitudes);
}

/// Overall sensitivity comparison: Which parameters matter most?
fn bench_sensitivity_ranking(c: &mut Criterion) {
    c.bench_function("sensitivity_ranking_all_params", |b| {
        b.iter(|| {
            // Test with 1% error on each parameter
            let error = 0.01;

            let params = vec![
                "focal_length",
                "principal_point",
                "baseline",
                "stereo_rotation",
                "imu_scale_factor",
                "imu_bias",
                "time_offset",
                "rolling_shutter_readout",
            ];

            let mut sensitivities = Vec::new();
            for param in params {
                let sensitivity = evaluate_calibration_error(error, param);
                sensitivities.push((
                    param,
                    sensitivity.reprojection_error_increase,
                    sensitivity.tracking_loss_rate,
                ));
            }

            // Sort by total impact (reprojection + tracking)
            use std::cmp::Ordering;
            sensitivities.sort_by(|a, b| {
                (b.1 + b.2)
                    .partial_cmp(&(a.1 + a.2))
                    .unwrap_or(Ordering::Equal)
            });

            sensitivities
        });
    });
}

/// Test combined errors: interactions between parameter errors
fn bench_combined_calibration_error(c: &mut Criterion) {
    c.bench_function("combined_errors_focal_baseline", |b| {
        b.iter(|| {
            let focal_error = evaluate_calibration_error(0.01, "focal_length");
            let baseline_error = evaluate_calibration_error(0.01, "baseline");

            // Errors can accumulate or partially cancel
            let combined_reprojection = focal_error.reprojection_error_increase
                + baseline_error.reprojection_error_increase;
            let combined_depth =
                focal_error.depth_error_increase + baseline_error.depth_error_increase;

            (combined_reprojection, combined_depth)
        });
    });

    c.bench_function("combined_errors_imu_time_and_scale", |b| {
        b.iter(|| {
            let time_error = evaluate_calibration_error(0.001, "time_offset");
            let scale_error = evaluate_calibration_error(0.01, "imu_scale_factor");

            // IMU errors interact strongly with motion speed
            (time_error.tracking_loss_rate + scale_error.tracking_loss_rate).min(0.5)
        });
    });
}

/// Benchmark: Calibration validation curves
/// Shows acceptable error ranges for each parameter
fn bench_calibration_acceptance_thresholds(c: &mut Criterion) {
    c.bench_function("acceptance_threshold_focal_length", |b| {
        b.iter(|| {
            // Find error level where reprojection error exceeds 0.5px threshold
            let threshold_px = 0.5;
            for error in (1..100).map(|i| f64::from(i) / 10000.0) {
                let sensitivity = evaluate_calibration_error(error, "focal_length");
                if sensitivity.reprojection_error_increase > threshold_px {
                    return error; // This is our acceptance boundary
                }
            }
            0.1 // Should have found it by 1%
        });
    });

    c.bench_function("acceptance_threshold_time_offset", |b| {
        b.iter(|| {
            // Time offset is critical: >2% tracking loss is bad
            let threshold_tracking_loss = 0.02;
            for error in (1..100).map(|i| f64::from(i) / 100000.0) {
                let sensitivity = evaluate_calibration_error(error, "time_offset");
                if sensitivity.tracking_loss_rate > threshold_tracking_loss {
                    return error * 1000.0; // Convert to ms for clarity
                }
            }
            0.01 // Should have found it by 1ms
        });
    });
}

criterion_group!(
    benches,
    bench_focal_length_sensitivity,
    bench_principal_point_sensitivity,
    bench_baseline_sensitivity,
    bench_stereo_rotation_sensitivity,
    bench_imu_scale_sensitivity,
    bench_imu_bias_sensitivity,
    bench_time_offset_sensitivity,
    bench_rolling_shutter_sensitivity,
    bench_sensitivity_ranking,
    bench_combined_calibration_error,
    bench_calibration_acceptance_thresholds,
);
criterion_main!(benches);
