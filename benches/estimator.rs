//! # Estimator Benchmarks
//!
//! Performance benchmarks for the frame estimator subsystem.
//!
//! ## Benchmarks
//!
//! - **Single Frame Processing** - Baseline latency per frame
//! - **Sequential Frame Processing** - Multi-frame pipeline behavior (5, 10, 20 frames)
//! - **Camera Model Creation** - Initialization overhead
//! - **Estimator Initialization** - Setup cost
//! - **Resolution Scaling** - Impact of image resolution (320×240, 640×480, 1280×960)
//! - **IMU Processing** - IMU preintegration, motion prediction, velocity estimation
//!
//! ## Expected Results (baseline: 640×480)
//!
//! Single frame latency: 15-25ms | Camera creation: 1-2ms | 1280×960: 3-4× slower
//! IMU processing overhead: <1ms per 200Hz batch
//!
//! See [BENCHMARKING.md](../BENCHMARKING.md#estimator-benchmarks) for detailed analysis.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::cast_precision_loss,
    clippy::cast_lossless
)]

use criterion::{black_box, criterion_group, criterion_main, BenchmarkId, Criterion};
use rs_vio::datasets::config::Config;
use rs_vio::estimator::Estimator;

fn create_bench_estimator() -> Estimator {
    let yaml = r#"
camera:
  image_width: 640
  image_height: 480
  left_intrinsics: [500.0, 500.0, 320.0, 240.0]
  left_distortion: [0.0, 0.0, 0.0, 0.0]
  right_intrinsics: [500.0, 500.0, 320.0, 240.0]
  right_distortion: [0.0, 0.0, 0.0, 0.0]
  left_model: pinhole-radtan
  T_B_Cl: [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
  T_B_Cr: [1.0, 0.0, 0.0, 0.1, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
keyframe_management:
  keyframe_window_size: 5
  translation_threshold: 0.1
  rotation_threshold: 0.1
feature_detection:
  grid_cols: 10
  optical_flow_max_iterations: 30
  optical_flow_convergence_threshold: 0.01
optimization:
  max_iterations: 10
  tolerance: 1e-6
"#;
    let config: Config = serde_yaml::from_str(yaml).unwrap();
    Estimator::new(config, None)
}

fn create_test_image(width: usize, height: usize) -> Vec<u8> {
    let mut image = vec![0u8; width * height];
    let square_size = 32;
    for y in 0..height {
        for x in 0..width {
            let square_x = x / square_size;
            let square_y = y / square_size;
            image[y * width + x] = if (square_x + square_y) % 2 == 0 {
                255
            } else {
                100
            };
        }
    }
    image
}

fn bench_frame_processing(c: &mut Criterion) {
    let mut estimator = create_bench_estimator();
    let left_image = create_test_image(640, 480);
    let right_image = create_test_image(640, 480);
    let timestamp_ns = 1000000000;

    c.bench_function("process_single_frame_baseline", |b| {
        b.iter(|| {
            let _result = estimator.process_frame(
                black_box(&left_image),
                black_box(&right_image),
                black_box(timestamp_ns),
                black_box(None),
            );
        });
    });
}

fn bench_sequential_frame_processing(c: &mut Criterion) {
    let mut group = c.benchmark_group("sequential_frames");
    group.sample_size(10);

    for num_frames in &[5, 10, 20] {
        group.bench_with_input(
            BenchmarkId::new("frame_count", num_frames),
            num_frames,
            |b, &num_frames| {
                let mut estimator = create_bench_estimator();
                let left_image = black_box(create_test_image(640, 480));
                let right_image = black_box(create_test_image(640, 480));

                b.iter(|| {
                    for frame_id in 0..num_frames {
                        let timestamp_ns = (frame_id as i64) * 100000000;
                        let _result =
                            estimator.process_frame(&left_image, &right_image, timestamp_ns, None);
                    }
                });
            },
        );
    }
    group.finish();
}

fn bench_camera_model_creation(c: &mut Criterion) {
    let yaml = r#"
camera:
  image_width: 640
  image_height: 480
  left_intrinsics: [500.0, 500.0, 320.0, 240.0]
  left_distortion: [0.0, 0.0, 0.0, 0.0]
  right_intrinsics: [500.0, 500.0, 320.0, 240.0]
  right_distortion: [0.0, 0.0, 0.0, 0.0]
  left_model: pinhole-radtan
  T_B_Cl: [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
  T_B_Cr: [1.0, 0.0, 0.0, 0.1, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
keyframe_management:
  keyframe_window_size: 5
  translation_threshold: 0.1
  rotation_threshold: 0.1
feature_detection:
  grid_cols: 10
  optical_flow_max_iterations: 30
  optical_flow_convergence_threshold: 0.01
optimization:
  max_iterations: 10
  tolerance: 1e-6
"#;
    let config: Config = serde_yaml::from_str(yaml).unwrap();

    c.bench_function("create_camera_models", |b| {
        b.iter(|| {
            let _cameras = rs_vio::datasets::create_camera_models_from_config(black_box(&config));
        });
    });
}

fn bench_estimator_initialization(c: &mut Criterion) {
    let yaml = r#"
camera:
  image_width: 640
  image_height: 480
  left_intrinsics: [500.0, 500.0, 320.0, 240.0]
  left_distortion: [0.0, 0.0, 0.0, 0.0]
  right_intrinsics: [500.0, 500.0, 320.0, 240.0]
  right_distortion: [0.0, 0.0, 0.0, 0.0]
  left_model: pinhole-radtan
  T_B_Cl: [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
  T_B_Cr: [1.0, 0.0, 0.0, 0.1, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
keyframe_management:
  keyframe_window_size: 5
  translation_threshold: 0.1
  rotation_threshold: 0.1
feature_detection:
  grid_cols: 10
  optical_flow_max_iterations: 30
  optical_flow_convergence_threshold: 0.01
optimization:
  max_iterations: 10
  tolerance: 1e-6
"#;
    let config: Config = serde_yaml::from_str(yaml).unwrap();

    c.bench_function("estimator_initialization", |b| {
        b.iter(|| {
            let _estimator = Estimator::new(black_box(config.clone()), None);
        });
    });
}

fn bench_image_resolution_scaling(c: &mut Criterion) {
    let mut group = c.benchmark_group("resolution_scaling");
    group.sample_size(10);

    for resolution in &[(320, 240), (640, 480), (1280, 960)] {
        let (width, height) = resolution;
        group.bench_with_input(
            BenchmarkId::new("resolution", format!("{}x{}", width, height)),
            resolution,
            |b, &(width, height)| {
                let yaml = format!(
                    r#"
camera:
  image_width: {}
  image_height: {}
  left_intrinsics: [500.0, 500.0, {}, {}]
  left_distortion: [0.0, 0.0, 0.0, 0.0]
  right_intrinsics: [500.0, 500.0, {}, {}]
  right_distortion: [0.0, 0.0, 0.0, 0.0]
  left_model: pinhole-radtan
  T_B_Cl: [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
  T_B_Cr: [1.0, 0.0, 0.0, 0.1, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
keyframe_management:
  keyframe_window_size: 5
  translation_threshold: 0.1
  rotation_threshold: 0.1
feature_detection:
  grid_cols: 10
  optical_flow_max_iterations: 30
  optical_flow_convergence_threshold: 0.01
optimization:
  max_iterations: 10
  tolerance: 1e-6
"#,
                    width,
                    height,
                    width as f64 / 2.0,
                    height as f64 / 2.0,
                    width as f64 / 2.0,
                    height as f64 / 2.0
                );
                let config: Config = serde_yaml::from_str(&yaml).unwrap();
                let mut estimator = Estimator::new(config, None);

                let left_image = black_box(create_test_image(width, height));
                let right_image = black_box(create_test_image(width, height));

                b.iter(|| {
                    let _result =
                        estimator.process_frame(&left_image, &right_image, 1000000000, None);
                });
            },
        );
    }
    group.finish();
}

fn create_test_imu_data(num_samples: usize, base_timestamp: i64) -> Vec<rs_vio::datasets::ImuData> {
    let dt_ns = 5_000_000; // 5ms between samples (200Hz)
    (0..num_samples)
        .map(|i| {
            let t = (i as f64) * 0.005;
            rs_vio::datasets::ImuData {
                timestamp: base_timestamp + (i as i64) * dt_ns,
                gyro: [(t * 0.1).sin() * 0.05, (t * 0.1).cos() * 0.05, 0.02],
                accel: [(t * 0.5).sin() * 0.5, (t * 0.5).cos() * 0.5, -9.81],
            }
        })
        .collect()
}

fn bench_imu_processing(c: &mut Criterion) {
    let mut group = c.benchmark_group("imu_processing");
    group.sample_size(10);

    for num_samples in &[10, 50, 200] {
        group.bench_with_input(
            BenchmarkId::new("imu_samples", num_samples),
            num_samples,
            |b, &num_samples| {
                let mut estimator = create_bench_estimator();
                let left_image = black_box(create_test_image(640, 480));
                let right_image = black_box(create_test_image(640, 480));
                let base_timestamp = 1000000000;

                b.iter(|| {
                    let imu_data = create_test_imu_data(num_samples, base_timestamp);
                    let timestamp_ns = base_timestamp + (num_samples as i64) * 5_000_000;
                    let _result = estimator.process_frame(
                        black_box(&left_image),
                        black_box(&right_image),
                        black_box(timestamp_ns),
                        black_box(Some(&imu_data)),
                    );
                });
            },
        );
    }
    group.finish();
}

fn bench_imu_high_frequency(c: &mut Criterion) {
    let mut group = c.benchmark_group("imu_high_frequency");
    group.sample_size(10);

    // 1000Hz IMU with 10Hz frames (100 samples per frame)
    let num_samples = 100;
    let mut estimator = create_bench_estimator();
    let left_image = black_box(create_test_image(640, 480));
    let right_image = black_box(create_test_image(640, 480));
    let base_timestamp = 1000000000;

    group.bench_function("1000hz_imu_per_frame", |b| {
        b.iter(|| {
            let imu_data = create_test_imu_data(num_samples, base_timestamp);
            let timestamp_ns = base_timestamp + 100_000_000;
            let _result = estimator.process_frame(
                black_box(&left_image),
                black_box(&right_image),
                black_box(timestamp_ns),
                black_box(Some(&imu_data)),
            );
        });
    });
    group.finish();
}

fn bench_imu_motion_predictor(c: &mut Criterion) {
    use rs_vio::imu::{ImuConfig, ImuMotionPredictor};

    let config = ImuConfig::default();
    let predictor = ImuMotionPredictor::new(config);
    let imu_data = create_test_imu_data(20, 1000000000);
    let focal_length = 500.0;
    let pixel_coord = (320.0, 240.0);

    c.bench_function("imu_motion_prediction", |b| {
        b.iter(|| {
            let _result = predictor.predict_feature_displacement(
                black_box(&imu_data),
                black_box(pixel_coord),
                black_box(focal_length),
            );
        });
    });
}

fn bench_imu_preintegration(c: &mut Criterion) {
    use rs_vio::imu::{ImuConfig, ImuPreintegrator};

    let config = ImuConfig::default();
    let mut preintegrator = ImuPreintegrator::new(config);
    let imu_data = create_test_imu_data(20, 1000000000);

    c.bench_function("imu_preintegration", |b| {
        b.iter(|| {
            for (i, imu) in imu_data.iter().enumerate() {
                let dt = if i > 0 {
                    (imu.timestamp - imu_data[i - 1].timestamp) as f64 / 1e9
                } else {
                    0.005
                };
                if dt > 0.0 {
                    preintegrator.propagate(imu, dt);
                }
            }
        });
    });
}

criterion_group!(
    benches,
    bench_frame_processing,
    bench_sequential_frame_processing,
    bench_camera_model_creation,
    bench_estimator_initialization,
    bench_image_resolution_scaling,
    bench_imu_processing,
    bench_imu_high_frequency,
    bench_imu_motion_predictor,
    bench_imu_preintegration
);
criterion_main!(benches);
