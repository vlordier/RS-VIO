use criterion::{black_box, criterion_group, criterion_main, Criterion};
use rs_vio::datasets::config::Config;
use rs_vio::estimator::Estimator;
use serde_yaml;

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

fn bench_frame_processing(c: &mut Criterion) {
    let mut estimator = create_bench_estimator();

    // Create dummy image data
    let left_image = vec![128u8; 640 * 480];
    let right_image = vec![128u8; 640 * 480];
    let timestamp_ns = 1000000000;

    c.bench_function("process_single_frame", |b| {
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

criterion_group!(benches, bench_frame_processing, bench_camera_model_creation);
criterion_main!(benches);
