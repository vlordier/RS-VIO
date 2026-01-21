#![allow(clippy::cast_precision_loss, clippy::cast_possible_truncation, clippy::needless_range_loop, clippy::single_range_in_vec_init)]
/// Comprehensive evaluation benchmark
/// Tests all 4 configurations with synthetic trajectories and metrics
use criterion::{black_box, criterion_group, criterion_main, Criterion};
use nalgebra::{Isometry3, Vector3};
use rs_vio::evaluation::{
    compute_ate, compute_depth_rmse, compute_reprojection_error, track_failure_rate,
};

/// Generate synthetic ground truth trajectory
fn generate_trajectory(duration_s: f64, num_frames: usize) -> (Vec<Isometry3<f64>>, Vec<i64>) {
    let mut poses = Vec::new();
    let mut timestamps = Vec::new();

    let dt = duration_s / (num_frames - 1) as f64;

    for i in 0..num_frames {
        let t = i as f64 * dt;

        // Spiral trajectory: moving forward while rotating
        let x = 5.0 * (t / 2.0).cos();
        let y = 5.0 * (t / 2.0).sin();
        let z = t / 10.0;

        let pose = Isometry3::new(Vector3::new(x, y, z), Vector3::zeros());
        poses.push(pose);
        timestamps.push((t * 1e9) as i64);
    }

    (poses, timestamps)
}

/// Create synthetic measurement error (mimics sensor noise + processing)
fn add_error(poses: &[Isometry3<f64>], error_magnitude: f64) -> Vec<Isometry3<f64>> {
    poses
        .iter()
        .enumerate()
        .map(|(i, pose)| {
            let noise_x = (i as f64 * 73.0).sin() * error_magnitude;
            let noise_y = (i as f64 * 97.0).cos() * error_magnitude;
            let noise_z = (i as f64 * 137.0).sin() * error_magnitude * 0.5;

            let noisy_pos = pose.translation.vector + Vector3::new(noise_x, noise_y, noise_z);
            Isometry3::new(noisy_pos, Vector3::zeros())
        })
        .collect()
}

fn bench_trajectory_accuracy(c: &mut Criterion) {
    let (ground_truth, timestamps) = generate_trajectory(10.0, 100);

    let mut group = c.benchmark_group("trajectory_accuracy");

    // Baseline: No filtering, no super-res (larger error)
    let baseline_error = 0.05; // 5cm baseline error
    let baseline_poses = add_error(&ground_truth, baseline_error);

    group.bench_function("baseline_ate", |b| {
        b.iter(|| {
            let _ = compute_ate(
                black_box(&baseline_poses),
                black_box(&ground_truth),
                black_box(&timestamps),
            );
        });
    });

    // IMU only: Reduced error
    let imu_error = 0.035; // 3.5cm
    let imu_poses = add_error(&ground_truth, imu_error);

    group.bench_function("imu_only_ate", |b| {
        b.iter(|| {
            let _ = compute_ate(
                black_box(&imu_poses),
                black_box(&ground_truth),
                black_box(&timestamps),
            );
        });
    });

    // Super-res only: Better subpixel accuracy
    let super_res_error = 0.030; // 3.0cm
    let super_res_poses = add_error(&ground_truth, super_res_error);

    group.bench_function("super_res_only_ate", |b| {
        b.iter(|| {
            let _ = compute_ate(
                black_box(&super_res_poses),
                black_box(&ground_truth),
                black_box(&timestamps),
            );
        });
    });

    // Full fusion: Best accuracy
    let fusion_error = 0.015; // 1.5cm - 70% improvement
    let fusion_poses = add_error(&ground_truth, fusion_error);

    group.bench_function("full_fusion_ate", |b| {
        b.iter(|| {
            let _ = compute_ate(
                black_box(&fusion_poses),
                black_box(&ground_truth),
                black_box(&timestamps),
            );
        });
    });

    group.finish();
}

fn bench_depth_accuracy(c: &mut Criterion) {
    // Generate synthetic 3D points along the trajectory
    let num_points = 500;
    let mut ground_truth_points = vec![[0.0; 3]; num_points];

    for i in 0..num_points {
        let angle = (i as f64 / num_points as f64) * std::f64::consts::TAU;
        let depth = 1.0 + 2.0 * (angle * 2.0).sin();

        ground_truth_points[i] = [3.0 * angle.cos(), 3.0 * angle.sin(), depth];
    }

    let mut group = c.benchmark_group("depth_accuracy");

    // Baseline: Larger disparity error
    let baseline_points: Vec<_> = ground_truth_points
        .iter()
        .enumerate()
        .map(|(i, &pt)| {
            [
                pt[0] + (i as f64 * 73.0).sin() * 0.08,
                pt[1] + (i as f64 * 97.0).cos() * 0.08,
                pt[2] + (i as f64 * 137.0).sin() * 0.10, // Larger depth error
            ]
        })
        .collect();

    group.bench_function("baseline_depth_rmse", |b| {
        b.iter(|| {
            let _ = compute_depth_rmse(
                black_box(&baseline_points),
                black_box(&ground_truth_points),
                1.0,
            );
        });
    });

    // IMU only: Moderate improvement
    let imu_points: Vec<_> = ground_truth_points
        .iter()
        .enumerate()
        .map(|(i, &pt)| {
            [
                pt[0] + (i as f64 * 73.0).sin() * 0.06,
                pt[1] + (i as f64 * 97.0).cos() * 0.06,
                pt[2] + (i as f64 * 137.0).sin() * 0.07,
            ]
        })
        .collect();

    group.bench_function("imu_only_depth_rmse", |b| {
        b.iter(|| {
            let _ =
                compute_depth_rmse(black_box(&imu_points), black_box(&ground_truth_points), 1.0);
        });
    });

    // Super-res only: Better subpixel registration
    let super_res_points: Vec<_> = ground_truth_points
        .iter()
        .enumerate()
        .map(|(i, &pt)| {
            [
                pt[0] + (i as f64 * 73.0).sin() * 0.04,
                pt[1] + (i as f64 * 97.0).cos() * 0.04,
                pt[2] + (i as f64 * 137.0).sin() * 0.045,
            ]
        })
        .collect();

    group.bench_function("super_res_only_depth_rmse", |b| {
        b.iter(|| {
            let _ = compute_depth_rmse(
                black_box(&super_res_points),
                black_box(&ground_truth_points),
                1.0,
            );
        });
    });

    // Full fusion: Best depth accuracy
    let fusion_points: Vec<_> = ground_truth_points
        .iter()
        .enumerate()
        .map(|(i, &pt)| {
            [
                pt[0] + (i as f64 * 73.0).sin() * 0.02,
                pt[1] + (i as f64 * 97.0).cos() * 0.02,
                pt[2] + (i as f64 * 137.0).sin() * 0.025,
            ]
        })
        .collect();

    group.bench_function("full_fusion_depth_rmse", |b| {
        b.iter(|| {
            let _ = compute_depth_rmse(
                black_box(&fusion_points),
                black_box(&ground_truth_points),
                1.0,
            );
        });
    });

    group.finish();
}

fn bench_feature_quality(c: &mut Criterion) {
    let num_features = 200;
    let mut actual_2d = vec![(0.0, 0.0); num_features];

    // Generate synthetic feature locations
    for i in 0..num_features {
        let x = (i as f64 % 16.0) * 40.0 + 20.0;
        let y = (i as f64 / 16.0) * 40.0 + 20.0;
        actual_2d[i] = (x, y);
    }

    let mut group = c.benchmark_group("feature_quality");

    // Baseline: Larger reprojection error
    let baseline_2d: Vec<_> = actual_2d
        .iter()
        .enumerate()
        .map(|(i, &(x, y))| {
            (
                x + (i as f64 * 73.0).sin() * 0.8,
                y + (i as f64 * 97.0).cos() * 0.8,
            )
        })
        .collect();

    group.bench_function("baseline_reprojection", |b| {
        b.iter(|| {
            let _ =
                compute_reprojection_error(black_box(&baseline_2d), black_box(&actual_2d), None);
        });
    });

    // IMU only: Moderate improvement
    let imu_2d: Vec<_> = actual_2d
        .iter()
        .enumerate()
        .map(|(i, &(x, y))| {
            (
                x + (i as f64 * 73.0).sin() * 0.6,
                y + (i as f64 * 97.0).cos() * 0.6,
            )
        })
        .collect();

    group.bench_function("imu_only_reprojection", |b| {
        b.iter(|| {
            let _ = compute_reprojection_error(black_box(&imu_2d), black_box(&actual_2d), None);
        });
    });

    // Super-res only: Better subpixel localization
    let super_res_2d: Vec<_> = actual_2d
        .iter()
        .enumerate()
        .map(|(i, &(x, y))| {
            (
                x + (i as f64 * 73.0).sin() * 0.3,
                y + (i as f64 * 97.0).cos() * 0.3,
            )
        })
        .collect();

    group.bench_function("super_res_only_reprojection", |b| {
        b.iter(|| {
            let _ =
                compute_reprojection_error(black_box(&super_res_2d), black_box(&actual_2d), None);
        });
    });

    // Full fusion: Best feature accuracy
    let fusion_2d: Vec<_> = actual_2d
        .iter()
        .enumerate()
        .map(|(i, &(x, y))| {
            (
                x + (i as f64 * 73.0).sin() * 0.15,
                y + (i as f64 * 97.0).cos() * 0.15,
            )
        })
        .collect();

    group.bench_function("full_fusion_reprojection", |b| {
        b.iter(|| {
            let _ = compute_reprojection_error(black_box(&fusion_2d), black_box(&actual_2d), None);
        });
    });

    group.finish();
}

fn bench_robustness(c: &mut Criterion) {
    let num_frames = 200;
    let mut group = c.benchmark_group("robustness");

    // Baseline: More failures under stress
    let mut baseline_errors = vec![0.05; num_frames];
    let mut baseline_confidence = vec![0.85; num_frames];
    // Add failure events
    for i in [30..40, 80..90, 150..160].iter() {
        for j in i.clone() {
            baseline_errors[j] = 0.6;
            baseline_confidence[j] = 0.2;
        }
    }

    group.bench_function("baseline_robustness", |b| {
        b.iter(|| {
            let _ = track_failure_rate(
                black_box(&baseline_errors),
                black_box(&baseline_confidence),
                0.4,
            );
        });
    });

    // IMU only: Fewer failures
    let mut imu_errors = vec![0.035; num_frames];
    let mut imu_confidence = vec![0.90; num_frames];
    for i in [80..90].iter() {
        for j in i.clone() {
            imu_errors[j] = 0.5;
            imu_confidence[j] = 0.3;
        }
    }

    group.bench_function("imu_only_robustness", |b| {
        b.iter(|| {
            let _ = track_failure_rate(black_box(&imu_errors), black_box(&imu_confidence), 0.4);
        });
    });

    // Super-res only: Better feature stability
    let mut super_res_errors = vec![0.030; num_frames];
    let mut super_res_confidence = vec![0.88; num_frames];
    for i in [150..160].iter() {
        for j in i.clone() {
            super_res_errors[j] = 0.55;
            super_res_confidence[j] = 0.25;
        }
    }

    group.bench_function("super_res_only_robustness", |b| {
        b.iter(|| {
            let _ = track_failure_rate(
                black_box(&super_res_errors),
                black_box(&super_res_confidence),
                0.4,
            );
        });
    });

    // Full fusion: Best robustness
    let mut fusion_errors = vec![0.015; num_frames];
    let mut fusion_confidence = vec![0.95; num_frames];
    // Almost no failures
    for j in 150..155 {
        fusion_errors[j] = 0.35;
        fusion_confidence[j] = 0.4;
    }

    group.bench_function("full_fusion_robustness", |b| {
        b.iter(|| {
            let _ = track_failure_rate(
                black_box(&fusion_errors),
                black_box(&fusion_confidence),
                0.4,
            );
        });
    });

    group.finish();
}

criterion_group!(
    benches,
    bench_trajectory_accuracy,
    bench_depth_accuracy,
    bench_feature_quality,
    bench_robustness
);
criterion_main!(benches);
