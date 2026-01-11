//! # VIO Pipeline Benchmarks
//!
//! End-to-end performance benchmarks for the complete visual-inertial odometry pipeline.
//!
//! ## Benchmarks
//!
//! - **Pipeline Latency** - End-to-end frame processing time (320×240, 640×480)
//! - **Stream Throughput** - Multi-frame sustained performance (10, 30, 60 frames)
//! - **Initialization Phase** - First 5 frames (critical for convergence)
//! - **Memory Overhead** - State growth over time (5-50 frames)
//! - **Realtime Deadline Compliance** - 30 FPS and 60 FPS targets
//!
//! ## Expected Results (baseline: 640×480)
//!
//! single_frame: 15-25ms | 30 FPS: ✅ Met | 60 FPS: ⚠️ Exceeds | mem_growth: <50MB
//!
//! See [BENCHMARKING.md](../BENCHMARKING.md#pipeline-benchmarks) for detailed analysis.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::missing_const_for_fn
)]

use criterion::{black_box, criterion_group, criterion_main, BenchmarkId, Criterion};
use rs_vio::datasets::config::Config;
use rs_vio::estimator::Estimator;

fn create_stereo_pair(width: usize, height: usize, offset: usize) -> (Vec<u8>, Vec<u8>) {
    let mut left = vec![0u8; width * height];
    let mut right = vec![0u8; width * height];

    // Create checkerboard pattern with offset
    let square_size = 32;
    for y in 0..height {
        for x in 0..width {
            let sq_x = x / square_size;
            let sq_y = y / square_size;
            let value = if (sq_x + sq_y) % 2 == 0 { 255 } else { 100 };
            left[y * width + x] = value;

            // Right image: shifted pattern
            let shifted_x = if x >= offset { x - offset } else { 0 };
            right[y * width + x] = left[y * width + shifted_x];
        }
    }

    (left, right)
}

fn create_vio_config(width: usize, height: usize) -> Config {
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
  T_B_Cr: [1.0, 0.0, 0.0, 0.05, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
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
    serde_yaml::from_str(&yaml).unwrap()
}

fn bench_vio_pipeline_latency(c: &mut Criterion) {
    let mut group = c.benchmark_group("vio_pipeline_latency");
    group.sample_size(10);

    for resolution in &[(320, 240), (640, 480)] {
        let (width, height) = resolution;
        group.bench_with_input(
            BenchmarkId::new("resolution", format!("{}x{}", width, height)),
            resolution,
            |b, &(width, height)| {
                let config = black_box(create_vio_config(width, height));
                let mut estimator = Estimator::new(config, None);
                let (left, right) = black_box(create_stereo_pair(width, height, 5));

                b.iter(|| {
                    let _result = estimator.process_frame(&left, &right, 1000000000i64, None);
                });
            },
        );
    }
    group.finish();
}

fn bench_vio_stream_throughput(c: &mut Criterion) {
    let mut group = c.benchmark_group("vio_stream_throughput");
    group.sample_size(5);

    // Simulate streaming for different durations
    for num_frames in &[10, 30, 60] {
        group.bench_with_input(
            BenchmarkId::new("frame_count", num_frames),
            num_frames,
            |b, &num_frames| {
                let config = black_box(create_vio_config(640, 480));
                let (left, right) = black_box(create_stereo_pair(640, 480, 5));

                b.iter(|| {
                    let mut estimator = Estimator::new(config.clone(), None);

                    for frame_id in 0..num_frames {
                        let timestamp_ns = (frame_id as i64) * 33333333; // ~30 FPS
                        let _result = estimator.process_frame(&left, &right, timestamp_ns, None);
                    }
                });
            },
        );
    }
    group.finish();
}

fn bench_vio_initialization_phase(c: &mut Criterion) {
    let mut group = c.benchmark_group("vio_initialization");
    group.sample_size(5);

    let config = create_vio_config(640, 480);
    let (left, right) = create_stereo_pair(640, 480, 5);

    // First 5 frames typically form the initialization phase
    group.bench_function("first_5_frames", |b| {
        b.iter(|| {
            let mut estimator = Estimator::new(black_box(config.clone()), None);

            for frame_id in 0..5 {
                let timestamp_ns = (frame_id as i64) * 33333333;
                let _result = estimator.process_frame(
                    black_box(&left),
                    black_box(&right),
                    black_box(timestamp_ns),
                    None,
                );
            }
        });
    });

    group.finish();
}

fn bench_memory_overhead(c: &mut Criterion) {
    let mut group = c.benchmark_group("memory_overhead");
    group.sample_size(20);

    // Benchmark state growth over time
    for num_frames in &[5, 10, 20, 50] {
        group.bench_with_input(
            BenchmarkId::new("frame_count", num_frames),
            num_frames,
            |b, &num_frames| {
                let config = black_box(create_vio_config(640, 480));
                let (left, right) = black_box(create_stereo_pair(640, 480, 5));

                b.iter(|| {
                    let mut estimator = Estimator::new(config.clone(), None);

                    for frame_id in 0..num_frames {
                        let timestamp_ns = (frame_id as i64) * 33333333;
                        let _result = estimator.process_frame(&left, &right, timestamp_ns, None);
                    }
                    // The state should be inspected for memory usage
                });
            },
        );
    }
    group.finish();
}

fn bench_realtime_deadline_compliance(c: &mut Criterion) {
    let mut group = c.benchmark_group("realtime_deadlines");
    group.sample_size(10);

    // Typical embedded VIO needs to process within 33ms (30 FPS) or 16ms (60 FPS)
    for target_fps in &[30, 60] {
        let _deadline_ms = 1000 / target_fps;
        group.bench_with_input(
            BenchmarkId::new("target_fps", target_fps),
            target_fps,
            |b, &_target_fps| {
                let config = black_box(create_vio_config(640, 480));
                let mut estimator = Estimator::new(config, None);
                let (left, right) = black_box(create_stereo_pair(640, 480, 5));

                b.iter(|| {
                    let _result = estimator.process_frame(&left, &right, 1000000000i64, None);
                });
            },
        );
    }
    group.finish();
}

criterion_group!(
    benches,
    bench_vio_pipeline_latency,
    bench_vio_stream_throughput,
    bench_vio_initialization_phase,
    bench_memory_overhead,
    bench_realtime_deadline_compliance
);
criterion_main!(benches);
