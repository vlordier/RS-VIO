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
//!
//! ## Expected Results (baseline: 640×480)
//!
//! Single frame latency: 15-25ms | Camera creation: 1-2ms | 1280×960: 3-4× slower
//!
//! See [BENCHMARKING.md](../BENCHMARKING.md#estimator-benchmarks) for detailed analysis.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use criterion::{black_box, criterion_group, criterion_main, BenchmarkId, Criterion};
use rs_vio::datasets::config::Config;
use rs_vio::estimator::Estimator;

fn create_bench_estimator() -> Estimator<'static> {
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
                        let timestamp_ns = (frame_id as u64) * 100000000;
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

criterion_group!(
    benches,
    bench_frame_processing,
    bench_sequential_frame_processing,
    bench_camera_model_creation,
    bench_estimator_initialization,
    bench_image_resolution_scaling
);
criterion_main!(benches);
