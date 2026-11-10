//! Benchmarks for the sliding window VIO pipeline.
//!
//! Run with: `cargo bench --bench sliding_window`

use criterion::{criterion_group, criterion_main, BenchmarkId, Criterion};
use nalgebra as na;
use rand::Rng;
use apex_solver::factors::Factor;
use rs_vio::datasets::ImuData;
use rs_vio::estimator::{inverse_se3, preintegrate_imu, Frame, SlidingWindow};
use rs_vio::optimization::imu_factor::ImuPreintegrationFactor;
use rs_vio::optimization::imu_preintegration::{ImuNoiseConfig, PreintegratedImuMeasurements};
use rs_vio::types::Matrix4x4;

/// Generate a random SE(3) transform with small rotation and translation.
fn random_se3_small(rng: &mut impl Rng) -> Matrix4x4 {
    let mut m = Matrix4x4::identity();
    let rx = rng.gen_range(-0.1..0.1);
    let ry = rng.gen_range(-0.1..0.1);
    let rz = rng.gen_range(-0.1..0.1);
    let q = na::UnitQuaternion::from_euler_angles(rx, ry, rz);
    m.fixed_view_mut::<3, 3>(0, 0).copy_from(q.to_rotation_matrix().matrix());
    m[(0, 3)] = rng.gen_range(-0.5..0.5);
    m[(1, 3)] = rng.gen_range(-0.5..0.5);
    m[(2, 3)] = rng.gen_range(-0.5..0.5);
    m
}

/// Create a dummy keyframe at the given pose.
fn make_keyframe(id: i32, pose: Matrix4x4) -> Frame {
    let mut f = Frame::new(0, id);
    f.is_keyframe = true;
    f.state.T_W_B = pose;
    f
}

// ---------------------------------------------------------------------------
// inverse_se3 vs full matrix inverse
// ---------------------------------------------------------------------------

fn bench_inverse_se3(c: &mut Criterion) {
    let mut rng = rand::thread_rng();
    let matrices: Vec<Matrix4x4> = (0..100).map(|_| random_se3_small(&mut rng)).collect();
    let mut i = 0usize;

    let mut group = c.benchmark_group("inverse_se3");
    group.throughput(criterion::Throughput::Elements(1));

    group.bench_function("closed_form_se3", |b| {
        b.iter(|| {
            let m = &matrices[i % matrices.len()];
            i += 1;
            inverse_se3(m)
        })
    });

    group.bench_function("full_matrix_try_inverse", |b| {
        b.iter(|| {
            let m = &matrices[i % matrices.len()];
            i += 1;
            m.try_inverse().unwrap()
        })
    });

    group.finish();
}

// ---------------------------------------------------------------------------
// IMU pre-integration at various durations
// ---------------------------------------------------------------------------

fn bench_preintegrate_imu(c: &mut Criterion) {
    let mut rng = rand::thread_rng();
    let mut group = c.benchmark_group("preintegrate_imu");

    for duration_ms in [10, 50, 100, 200] {
        let n_samples = (duration_ms as f64 * 200.0 / 1000.0) as usize;
        let mut samples = Vec::with_capacity(n_samples);
        let base_ts: i64 = 1_000_000_000_000;
        for i in 0..n_samples {
            samples.push(ImuData {
                timestamp: base_ts + (i as i64) * 5_000_000,
                gyro: [rng.gen_range(-0.5..0.5), rng.gen_range(-0.5..0.5), rng.gen_range(-0.5..0.5)],
                accel: [rng.gen_range(-1.0..1.0), rng.gen_range(-1.0..1.0), rng.gen_range(9.0..11.0)],
            });
        }

        group.bench_with_input(
            BenchmarkId::from_parameter(format!("{}ms_{}samples", duration_ms, n_samples)),
            &samples,
            |b, samples| {
                let gravity = na::Vector3::<f64>::new(0.0, 0.0, 9.81);
                b.iter(|| preintegrate_imu(samples, &[0.0; 3], &[0.0; 3], &gravity))
            },
        );
    }

    group.finish();
}

// ---------------------------------------------------------------------------
// SlidingWindow operations
// ---------------------------------------------------------------------------

fn bench_add_keyframe(c: &mut Criterion) {
    let mut rng = rand::thread_rng();
    let frames: Vec<Frame> = (0..15).map(|i| {
        make_keyframe(i, random_se3_small(&mut rng))
    }).collect();

    c.bench_function("add_keyframe_x15", |b| {
        b.iter(|| {
            let mut sw = SlidingWindow::new(10);
            for f in &frames {
                sw.add_frame(f.clone());
            }
        })
    });
}

fn bench_predict_current_pose(c: &mut Criterion) {
    let mut rng = rand::thread_rng();
    let mut sw = SlidingWindow::new(10);
    for i in 0..10 {
        sw.add_frame(make_keyframe(i, random_se3_small(&mut rng)));
    }

    c.bench_function("predict_current_pose", |b| {
        b.iter(|| sw.predict_current_pose())
    });
}

// ---------------------------------------------------------------------------
// IMU preintegration with full covariance
// ---------------------------------------------------------------------------

fn bench_preintegrate_imu_full(c: &mut Criterion) {
    let mut rng = rand::thread_rng();
    let mut group = c.benchmark_group("preintegrate_imu_full");

    for duration_ms in [10, 50, 100, 200] {
        let n_samples = (duration_ms as f64 * 200.0 / 1000.0) as usize;
        let mut samples = Vec::with_capacity(n_samples);
        let base_ts: i64 = 1_000_000_000_000;
        for i in 0..n_samples {
            samples.push(ImuData {
                timestamp: base_ts + (i as i64) * 5_000_000,
                gyro: [rng.gen_range(-0.5..0.5), rng.gen_range(-0.5..0.5), rng.gen_range(-0.5..0.5)],
                accel: [rng.gen_range(-1.0..1.0), rng.gen_range(-1.0..1.0), rng.gen_range(9.0..11.0)],
            });
        }

        group.bench_with_input(
            BenchmarkId::from_parameter(format!("{}ms_{}samples", duration_ms, n_samples)),
            &samples,
            |b, samples| {
                let gravity = na::Vector3::<f64>::new(0.0, 0.0, 9.81);
                let noise = ImuNoiseConfig {
                    gyro_noise_std: 1e-3,
                    gyro_random_walk_std: 1e-4,
                    accel_noise_std: 1e-2,
                    accel_random_walk_std: 1e-3,
                };
                b.iter(|| PreintegratedImuMeasurements::preintegrate(
                    samples,
                    [0.0; 3],
                    [0.0; 3],
                    noise,
                    &gravity,
                ))
            },
        );
    }

    group.finish();
}

// ---------------------------------------------------------------------------
// IMU factor residual + Jacobian computation
// ---------------------------------------------------------------------------

fn bench_imu_factor(c: &mut Criterion) {
    let mut rng = rand::thread_rng();
    let mut group = c.benchmark_group("imu_factor");

    // Create synthetic IMU data
    let n_samples = 20;
    let mut samples = Vec::with_capacity(n_samples);
    let base_ts: i64 = 1_000_000_000_000;
    for i in 0..n_samples {
        samples.push(ImuData {
            timestamp: base_ts + (i as i64) * 5_000_000,
            gyro: [rng.gen_range(-0.1..0.1); 3],
            accel: [rng.gen_range(-0.1..0.1), rng.gen_range(-0.1..0.1), rng.gen_range(9.7..9.9)],
        });
    }

    let noise = ImuNoiseConfig {
        gyro_noise_std: 1e-3,
        gyro_random_walk_std: 1e-4,
        accel_noise_std: 1e-2,
        accel_random_walk_std: 1e-3,
    };
    let preint = PreintegratedImuMeasurements::preintegrate(
        &samples,
        [0.0; 3],
        [0.0; 3],
        noise,
        &na::Vector3::new(0.0, 0.0, 9.81),
    );

    let factor = ImuPreintegrationFactor::new(
        preint.clone(),
        na::Vector3::new(0.0, 0.0, 9.81),
        1e-4,
        1e-3,
    );

    // Create consistent parameters
    let dt = preint.delta_t;
    let pose_i = nalgebra::DVector::from_row_slice(&[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]);
    let pose_j = nalgebra::DVector::from_row_slice(&[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]);
    let v_i = nalgebra::DVector::from_row_slice(&[0.0, 0.0, 0.0]);
    let v_j = nalgebra::DVector::from_row_slice(&[0.0, 0.0, 9.81 * dt]);
    let bg = nalgebra::DVector::from_row_slice(&[0.0; 3]);
    let ba = nalgebra::DVector::from_row_slice(&[0.0; 3]);

    group.bench_function("residual_only", |b| {
        b.iter(|| factor.linearize(&[pose_i.clone(), pose_j.clone(), v_i.clone(), v_j.clone(), bg.clone(), ba.clone()], false))
    });

    group.bench_function("residual_and_jacobian", |b| {
        b.iter(|| factor.linearize(&[pose_i.clone(), pose_j.clone(), v_i.clone(), v_j.clone(), bg.clone(), ba.clone()], true))
    });

    group.finish();
}

// ---------------------------------------------------------------------------
// Criterion harness
// ---------------------------------------------------------------------------

criterion_group!(
    benches,
    bench_inverse_se3,
    bench_preintegrate_imu,
    bench_preintegrate_imu_full,
    bench_imu_factor,
    bench_add_keyframe,
    bench_predict_current_pose,
);
criterion_main!(benches);
