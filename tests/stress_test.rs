use rs_vio::datasets::config::Config;
use rs_vio::estimator::Estimator;
use std::time::Duration;

fn small_test_config() -> Config {
    let yaml = r#"
    camera:
      image_width: 64
      image_height: 48
      left_intrinsics: [400.0, 400.0, 32.0, 24.0]
      left_distortion: [0.0, 0.0, 0.0, 0.0]
      right_intrinsics: [400.0, 400.0, 32.0, 24.0]
      right_distortion: [0.0, 0.0, 0.0, 0.0]
      left_model: pinhole-radtan
      right_model: pinhole-radtan
      T_B_Cl: [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
      T_B_Cr: [1.0, 0.0, 0.0, 0.05, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
    keyframe_management:
      keyframe_window_size: 5
      translation_threshold: 0.05
      rotation_threshold: 0.05
    feature_detection:
      grid_size: 8
      max_features_per_grid: 32
      optical_flow_max_iterations: 20
      optical_flow_convergence_threshold: 0.01
    optimization:
      bundle_adjustment_max_iterations: 5
      pnp_max_iterations: 3
    "#;
    serde_yaml::from_str(yaml).expect("valid test config")
}

#[test]
fn stress_process_frames_stays_within_bounds() {
    let config = small_test_config();
    let mut estimator = Estimator::new(config, None);
    estimator.set_max_map_points(100);
    estimator.set_max_frame_processing_time(Duration::from_millis(500));

    let img_w = 64;
    let img_h = 48;
    let mut left = vec![0u8; img_w * img_h];
    let mut right = vec![0u8; img_w * img_h];

    // Run a moderate-length soak to mimic a longer mission without making CI slow.
    for frame_id in 0..200 {
        // Simple checker pattern with slight horizontal offset to ensure features move.
        for y in 0..img_h {
            for x in 0..img_w {
                let idx = y * img_w + x;
                left[idx] = if (x / 8 + y / 8) % 2 == 0 { 255 } else { 0 };
                right[idx] = if ((x + 2) / 8 + y / 8) % 2 == 0 { 255 } else { 0 };
            }
        }

        let ts = (frame_id as i64) * 10_000_000; // 10ms increments
        estimator
            .process_frame(&left, &right, ts, None)
          .unwrap_or_else(|e| panic!("Frame {} failed: {}", frame_id, e));
    }

    // Ensure bounded map size and progress.
    assert!(estimator.map_points_len() <= 100);
    assert!(estimator.frame_count() >= 200);
}
