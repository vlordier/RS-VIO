#![allow(clippy::cast_possible_truncation, clippy::cast_sign_loss, clippy::cast_lossless)]

use rs_vio::datasets::config::Config;
use rs_vio::estimator::Estimator;
use std::time::Instant;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("RS-VIO Profiling Example");
    println!("========================");

    // Enable pprof profiling if requested
    let enable_profiling =
        std::env::var("PROFILE").unwrap_or_else(|_| "false".to_string()) == "true";

    if enable_profiling {
        println!("Profiling enabled. Run with pprof to analyze performance.");
        println!("Example: cargo run --example profiling --features pprof -- PROFILE=true");
        println!("Then: pprof -http=:8080 target/release/examples/profiling");
    }

    // Create test configuration
    let yaml = r#"
camera:
  image_width: 1280
  image_height: 720
  left_intrinsics: [800.0, 800.0, 640.0, 360.0]
  left_distortion: [0.0, 0.0, 0.0, 0.0]
  right_intrinsics: [800.0, 800.0, 640.0, 360.0]
  right_distortion: [0.0, 0.0, 0.0, 0.0]
  left_model: pinhole-radtan
  T_B_Cl: [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
  T_B_Cr: [1.0, 0.0, 0.0, 0.1, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
keyframe_management:
  keyframe_window_size: 10
  translation_threshold: 0.1
  rotation_threshold: 0.1
feature_detection:
  grid_cols: 20
  optical_flow_max_iterations: 50
  optical_flow_convergence_threshold: 0.001
optimization:
  max_iterations: 20
  tolerance: 1e-6
"#;

    let config: Config = serde_yaml::from_str(yaml)?;
    let mut estimator = Estimator::new(config, None);

    // Generate test images (synthetic checkerboard)
    let image_width = 1280;
    let image_height = 720;
    let mut left_image = vec![0u8; image_width * image_height];
    let mut right_image = vec![0u8; image_width * image_height];

    for y in 0..image_height {
        for x in 0..image_width {
            let idx = y * image_width + x;
            // Create a checkerboard pattern
            let square_size = 64;
            let pattern = ((x / square_size) + (y / square_size)) % 2;
            left_image[idx] = if pattern == 0 { 255 } else { 0 };
            // Slight offset for right image to simulate stereo
            let right_x = (x as i32 - 10).max(0) as usize;
            let right_pattern = ((right_x / square_size) + (y / square_size)) % 2;
            right_image[idx] = if right_pattern == 0 { 255 } else { 0 };
        }
    }

    println!("Processing 100 frames for profiling...");

    let start_time = Instant::now();
    let num_frames = 100;

    for frame_id in 0..num_frames {
        let timestamp_ns = frame_id as i64 * 33000000; // ~30fps

        // Process frame
        estimator.process_frame(&left_image, &right_image, timestamp_ns, None)?;
    }

    let total_time = start_time.elapsed();
    let avg_time_per_frame = total_time / num_frames as u32;

    println!("Profiling Results:");
    println!("==================");
    println!("Total frames processed: {}", num_frames);
    println!("Total time: {:.2}s", total_time.as_secs_f64());
    println!(
        "Average time per frame: {:.2}ms",
        avg_time_per_frame.as_millis()
    );
    println!(
        "Frames per second: {:.1}",
        1.0 / avg_time_per_frame.as_secs_f64()
    );

    if enable_profiling {
        println!("\nProfiling data collected. Use pprof to analyze.");
    }

    Ok(())
}
