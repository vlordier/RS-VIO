//! Full VIO Pipeline Example
//!
//! Complete end-to-end VIO pipeline that processes TUM-VI stereo images
//! with the actual estimator, producing real trajectory estimates.

use nalgebra as na;
use rs_vio::datasets::config::Config;
use rs_vio::datasets::tum_vi::TumViSequence;
use rs_vio::datasets::ImuData;
use rs_vio::estimator::Estimator;
use std::env;
use std::fs::File;
use std::io::Write;
use std::time::Instant;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::builder()
        .filter_level(log::LevelFilter::Info)
        .init();

    println!("\n=== Full VIO Pipeline on TUM-VI Dataset ===\n");

    // Parse command line arguments
    let args: Vec<String> = env::args().collect();
    let sequence_name = args.get(1).map(|s| s.as_str()).unwrap_or("room1");

    let frames_to_process = args
        .get(2)
        .and_then(|s| s.parse::<usize>().ok())
        .unwrap_or(200); // Process 200 frames by default

    // Load dataset
    let dataset_dir = if std::path::Path::new("./datasets/tum_vi").exists() {
        "./datasets/tum_vi"
    } else {
        "./data/tum_vi"
    };

    println!("Dataset: {}", dataset_dir);
    println!("Sequence: {}", sequence_name);

    let sequence_path = std::path::Path::new(dataset_dir).join(sequence_name);

    let load_start = Instant::now();
    let sequence = TumViSequence::load(&sequence_path)?;
    let load_time = load_start.elapsed();

    println!("\n✓ Loaded sequence in {:.2}s", load_time.as_secs_f64());
    println!(
        "  - Frames: {} @ {:.1} Hz",
        sequence.num_frames(),
        sequence.frame_rate()
    );
    println!("  - IMU: {} measurements", sequence.imu_data.len());
    println!("  - Ground truth: {} poses", sequence.ground_truth.len());

    let start_timestamp = sequence.cam0_timestamps[0];
    let duration_ns = if sequence.num_frames() > 1 {
        sequence.cam0_timestamps[sequence.num_frames() - 1] - start_timestamp
    } else {
        0
    };
    println!("  - Duration: {:.1}s\n", duration_ns as f64 / 1e9);

    // Load configuration
    let config_path = if std::path::Path::new("./config/tum_vi.yaml").exists() {
        "./config/tum_vi.yaml"
    } else {
        "./config/euroc.yaml" // Fallback to EuRoC (compatible)
    };

    let config = Config::load(config_path)?;
    println!("✓ Loaded config: {}\n", config_path);

    // Create estimator (no viewer for batch processing)
    let mut estimator = Estimator::new(config.clone(), None);
    println!("✓ Created VIO estimator\n");

    // Process frames
    println!(
        "Processing {} frames...",
        frames_to_process.min(sequence.num_frames())
    );

    let process_start = Instant::now();
    let mut processed_count = 0;
    let mut imu_index = 0;

    for i in 0..frames_to_process.min(sequence.num_frames()) {
        let left_timestamp = sequence.cam0_timestamps[i];
        let left_path = &sequence.cam0_images[i];
        let right_path = &sequence.cam1_images[i];

        // Load images
        let left_img = image::open(left_path)
            .map_err(|e| format!("Failed to load left image: {}", e))?
            .to_luma8();
        let right_img = image::open(right_path)
            .map_err(|e| format!("Failed to load right image: {}", e))?
            .to_luma8();

        // Get IMU data for this frame window
        // Find all IMU measurements between last frame and current frame
        let prev_timestamp = if i > 0 {
            sequence.cam0_timestamps[i - 1]
        } else {
            0
        };

        let mut imu_window = Vec::new();
        while imu_index < sequence.imu_data.len() {
            let imu = &sequence.imu_data[imu_index];
            if imu.timestamp_ns > left_timestamp {
                break;
            }
            if imu.timestamp_ns >= prev_timestamp {
                imu_window.push(ImuData {
                    timestamp: imu.timestamp_ns as i64,
                    gyro: [imu.gyro_x, imu.gyro_y, imu.gyro_z],
                    accel: [imu.accel_x, imu.accel_y, imu.accel_z],
                });
            }
            imu_index += 1;
        }

        // Process frame with estimator
        let imu_data = if !imu_window.is_empty() {
            Some(imu_window.as_slice())
        } else {
            None
        };

        estimator.process_frame(
            left_img.as_raw(),
            right_img.as_raw(),
            left_timestamp as i64,
            imu_data,
        )?;

        processed_count += 1;

        // Progress updates
        if (i + 1) % 50 == 0 {
            println!(
                "  Processed {}/{} frames...",
                i + 1,
                frames_to_process.min(sequence.num_frames())
            );
        }
    }

    let process_time = process_start.elapsed();

    println!(
        "\n✓ VIO processing complete in {:.2}s",
        process_time.as_secs_f64()
    );
    println!(
        "  - Average: {:.2}ms per frame",
        process_time.as_secs_f64() * 1000.0 / processed_count as f64
    );

    // Extract trajectory
    let trajectory = estimator.get_trajectory();
    println!("  - Estimated {} poses", trajectory.len());

    // Save trajectory in TUM format
    let output_filename = format!(
        "trajectory_{}_full_{}.txt",
        sequence_name,
        chrono::Local::now().format("%Y%m%d_%H%M%S")
    );

    save_trajectory_tum(&output_filename, trajectory)?;
    println!("\n✓ Saved trajectory to: {}", output_filename);

    // Trajectory statistics
    if trajectory.len() >= 2 {
        let first_timestamp = trajectory[0].0;
        let last_timestamp = trajectory[trajectory.len() - 1].0;
        let traj_duration = (last_timestamp - first_timestamp) as f64 / 1e9;
        let traj_rate = trajectory.len() as f64 / traj_duration;

        println!("\nTrajectory Statistics:");
        println!("  - Duration: {:.2}s", traj_duration);
        println!("  - Poses: {}", trajectory.len());
        println!("  - Rate: {:.1} Hz", traj_rate);

        // Compute total distance traveled
        let mut total_distance = 0.0;
        for i in 1..trajectory.len() {
            let pos1 = trajectory[i - 1].1.fixed_view::<3, 1>(0, 3);
            let pos2 = trajectory[i].1.fixed_view::<3, 1>(0, 3);
            total_distance += (pos2 - pos1).norm();
        }
        println!("  - Distance: {:.2}m", total_distance);
    }

    println!("\nPipeline complete! Ready for accuracy evaluation:");
    println!("  cargo run --release --example evaluate_trajectory \\");
    println!("    {} \\", output_filename);
    println!(
        "    ./datasets/tum_vi/{}/mav0/mocap0/data.csv",
        sequence_name
    );

    Ok(())
}

/// Save trajectory in TUM format: timestamp tx ty tz qx qy qz qw
fn save_trajectory_tum(
    filename: &str,
    trajectory: &[(i64, na::Matrix4<f64>)],
) -> Result<(), Box<dyn std::error::Error>> {
    let mut file = File::create(filename)?;

    // Write header
    writeln!(file, "# timestamp tx ty tz qx qy qz qw")?;

    for (timestamp_ns, T_W_B) in trajectory {
        // Convert timestamp from nanoseconds to seconds
        let timestamp_s = *timestamp_ns as f64 / 1e9;

        // Extract translation
        let t = T_W_B.fixed_view::<3, 1>(0, 3);

        // Extract rotation and convert to quaternion
        let R = T_W_B.fixed_view::<3, 3>(0, 0).into_owned();
        let q = na::UnitQuaternion::from_matrix(&R);

        // TUM format: timestamp tx ty tz qx qy qz qw
        writeln!(
            file,
            "{:.9} {:.9} {:.9} {:.9} {:.9} {:.9} {:.9} {:.9}",
            timestamp_s, t.x, t.y, t.z, q.i, q.j, q.k, q.w
        )?;
    }

    Ok(())
}
