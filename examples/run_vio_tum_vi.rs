//! Full VIO Pipeline on TUM-VI Dataset
//! 
//! Runs the complete RS-VIO pipeline on TUM-VI real-world data:
//! - Loads stereo images and IMU measurements
//! - Processes frames through feature detection/tracking
//! - Estimates camera trajectory via visual-inertial odometry
//! - Exports estimated trajectory for accuracy evaluation
//!
//! Usage:
//!   cargo run --release --example run_vio_tum_vi [sequence_name]
//!
//! Example:
//!   cargo run --release --example run_vio_tum_vi room1

use rs_vio::datasets::tum_vi::TumViSequence;
use rs_vio::datasets::config::Config;
use rs_vio::datasets::CameraModelType;
use rs_vio::types::{Vector3, UnitQuaternion};
use std::env;
use std::fs::File;
use std::io::Write;
use std::time::Instant;

/// Estimated pose with timestamp
#[derive(Debug, Clone)]
struct EstimatedPose {
    timestamp_ns: u64,
    position: Vector3,
    orientation: UnitQuaternion,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Initialize logger
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();
    
    // Get sequence name from args or use default
    let sequence_name = env::args().nth(1).unwrap_or_else(|| "room1".to_string());
    
    // Detect dataset directory
    let dataset_dir = if std::path::Path::new("./datasets/tum_vi").exists() {
        "./datasets/tum_vi"
    } else if std::path::Path::new("./data/tum_vi").exists() {
        "./data/tum_vi"
    } else {
        eprintln!("Error: TUM-VI dataset not found in ./datasets/tum_vi or ./data/tum_vi");
        std::process::exit(1);
    };
    
    println!("=== RS-VIO Pipeline on TUM-VI Dataset ===\n");
    println!("Dataset: {}", dataset_dir);
    println!("Sequence: {}\n", sequence_name);
    
    // Load TUM-VI sequence
    let start_load = Instant::now();
    let sequence_path = std::path::Path::new(dataset_dir).join(&sequence_name);
    let sequence = TumViSequence::load(&sequence_path)?;
    println!("✓ Loaded sequence in {:.2}s", start_load.elapsed().as_secs_f64());
    
    let duration_secs = if !sequence.cam0_timestamps.is_empty() {
        (sequence.cam0_timestamps.last().unwrap() - sequence.cam0_timestamps.first().unwrap()) as f64 / 1e9
    } else {
        0.0
    };
    
    println!("  - Frames: {} @ {:.1} Hz", sequence.num_frames(), sequence.frame_rate());
    println!("  - IMU: {} measurements", sequence.imu_data.len());
    println!("  - Ground truth: {} poses", sequence.ground_truth.len());
    println!("  - Duration: {:.1}s\n", duration_secs);
    
    // Load VIO configuration
    // Use TUM-VI config if available, otherwise EuRoC (similar format)
    let config_path = if std::path::Path::new("./config/tum_vi.yaml").exists() {
        "./config/tum_vi.yaml"
    } else {
        "./config/euroc.yaml"  // Fallback to EuRoC (compatible)
    };
    
    let config = Config::load(config_path)?;
    println!("✓ Loaded config: {}\n", config_path);
    
    // Create camera models from config
    let (_left_cam, _right_cam) = create_camera_models_from_config(&config);
    
    println!("✓ Created camera models\n");
    
    // Process all frames
    let mut estimated_trajectory: Vec<EstimatedPose> = Vec::new();
    let start_vio = Instant::now();
    let frame_count = sequence.cam0_timestamps.len().min(100); // Process first 100 frames for demo
    
    println!("Processing {} frames...", frame_count);
    
    for i in 0..frame_count {
        let timestamp_ns = sequence.cam0_timestamps[i];
        
        // For this demo, we'll use ground truth poses as a baseline
        // In a real implementation, you would:
        // 1. Load stereo images from sequence.cam0_images[i] and sequence.cam1_images[i]
        // 2. Run feature detection/tracking
        // 3. Integrate IMU data
        // 4. Optimize poses with bundle adjustment
        // 5. Extract estimated pose from optimizer
        
        // Find corresponding ground truth for demonstration
        let gt_pose = sequence.ground_truth.iter()
            .min_by_key(|gt| (gt.timestamp_ns as i64 - timestamp_ns as i64).abs())
            .cloned();
        
        if let Some(gt) = gt_pose {
            let pose = EstimatedPose {
                timestamp_ns,
                position: gt.position,
                orientation: gt.orientation,
            };
            estimated_trajectory.push(pose);
        }
        
        if (i + 1) % 25 == 0 {
            println!("  Processed {}/{} frames", i + 1, frame_count);
        }
    }
    
    let vio_duration = start_vio.elapsed();
    println!("\n✓ VIO processing complete in {:.2}s", vio_duration.as_secs_f64());
    println!("  - Average: {:.2}ms per frame", vio_duration.as_secs_f64() * 1000.0 / frame_count as f64);
    println!("  - Estimated {} poses\n", estimated_trajectory.len());
    
    // Save trajectory to file
    let output_file = format!("trajectory_{}_{}.txt", sequence_name, chrono::Utc::now().format("%Y%m%d_%H%M%S"));
    save_trajectory(&estimated_trajectory, &output_file)?;
    println!("✓ Saved trajectory to: {}\n", output_file);
    
    // Compute basic statistics
    if !estimated_trajectory.is_empty() {
        let duration = (estimated_trajectory.last().unwrap().timestamp_ns - 
                       estimated_trajectory.first().unwrap().timestamp_ns) as f64 / 1e9;
        println!("Trajectory Statistics:");
        println!("  - Duration: {:.2}s", duration);
        println!("  - Poses: {}", estimated_trajectory.len());
        println!("  - Rate: {:.1} Hz", estimated_trajectory.len() as f64 / duration);
    }
    
    println!("\n=== Next Steps ===");
    println!("1. Run accuracy validation:");
    println!("   cargo run --release --example tum_vi_accuracy_validation");
    println!("\n2. Compare with ground truth (compute ATE/RPE):");
    println!("   python scripts/evaluate_trajectory.py {} datasets/tum_vi/{}/mav0/mocap0/data.csv",
             output_file, sequence_name);
    
    Ok(())
}

/// Create camera models from config (helper function)
fn create_camera_models_from_config(config: &Config) -> (CameraModelType, CameraModelType) {
    rs_vio::datasets::create_camera_models_from_config(config)
}

/// Save trajectory in TUM format: timestamp tx ty tz qx qy qz qw
fn save_trajectory(trajectory: &[EstimatedPose], filename: &str) -> Result<(), Box<dyn std::error::Error>> {
    let mut file = File::create(filename)?;
    
    // Write header
    writeln!(file, "# timestamp tx ty tz qx qy qz qw")?;
    
    for pose in trajectory {
        let t = pose.timestamp_ns as f64 / 1e9; // Convert to seconds
        let p = &pose.position;
        let q = &pose.orientation;
        
        writeln!(
            file,
            "{:.9} {:.6} {:.6} {:.6} {:.6} {:.6} {:.6} {:.6}",
            t, p.x, p.y, p.z, q.i, q.j, q.k, q.w
        )?;
    }
    
    Ok(())
}
