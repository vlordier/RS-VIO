//! TUM-VI Accuracy Validation (Phase 10)
//!
//! Runs the real VIO estimator on a TUM-VI sequence and reports ATE/RPE
//! against mocap ground truth. Defaults to room1 with a limited frame budget
//! for quick validation.

use nalgebra as na;
use rs_vio::datasets::config::Config;
use rs_vio::datasets::trajectory_eval::{AbsoluteTrajectoryError, RelativePoseError};
use rs_vio::datasets::tum_vi::TumViSequence;
use rs_vio::datasets::ImuData;
use rs_vio::estimator::Estimator;
use std::env;
use std::path::Path;
use std::time::Instant;

#[derive(Clone)]
struct Pose {
    timestamp_s: f64,
    position: na::Vector3<f64>,
    orientation: na::UnitQuaternion<f64>,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::builder()
        .filter_level(log::LevelFilter::Info)
        .init();

    let args: Vec<String> = env::args().collect();
    let sequence_name = args.get(1).map(|s| s.as_str()).unwrap_or("room1");
    let max_frames: usize = args
        .get(2)
        .and_then(|s| s.parse::<usize>().ok())
        .unwrap_or(1200);
    let skip_frames: usize = args
        .get(3)
        .and_then(|s| s.parse::<usize>().ok())
        .unwrap_or(0);

    let dataset_dir = env::var("TUM_VI_DIR").unwrap_or_else(|_| {
        if Path::new("./datasets/tum_vi").exists() {
            "./datasets/tum_vi".to_string()
        } else {
            "./data/tum_vi".to_string()
        }
    });

    let sequence_path = Path::new(&dataset_dir).join(sequence_name);
    println!("\n=== Phase 10: TUM-VI Accuracy Validation ===\n");
    println!("Dataset: {}", dataset_dir);
    println!("Sequence: {}", sequence_name);

    let load_start = Instant::now();
    let sequence = TumViSequence::load(&sequence_path)?;
    println!(
        "✓ Loaded sequence in {:.2}s",
        load_start.elapsed().as_secs_f64()
    );
    println!(
        "  - Frames: {} @ {:.1} Hz",
        sequence.num_frames(),
        sequence.frame_rate()
    );
    println!("  - IMU: {} measurements @ 200 Hz", sequence.imu_data.len());
    println!("  - Ground truth: {} poses\n", sequence.ground_truth.len());

    let config_path = if Path::new("./config/tum_vi.yaml").exists() {
        "./config/tum_vi.yaml"
    } else {
        "./config/euroc.yaml"
    };
    let config = Config::load(config_path)?;
    println!("✓ Loaded config: {}\n", config_path);

    let mut estimator = Estimator::new(config, None);
    println!("✓ Estimator created\n");

    // Process frames with IMU windows
    let frames_to_process = max_frames.min(sequence.num_frames() - skip_frames);
    println!(
        "Skipping {} frames, processing {} frames...\n",
        skip_frames, frames_to_process
    );

    let mut imu_index = 0usize;
    let process_start = Instant::now();

    for i in skip_frames..(skip_frames + frames_to_process) {
        let left_ts = sequence.cam0_timestamps[i];
        let left_path = &sequence.cam0_images[i];
        let right_path = &sequence.cam1_images[i];

        let left_img = image::open(left_path)?.to_luma8();
        let right_img = image::open(right_path)?.to_luma8();

        let prev_ts = if i > 0 {
            sequence.cam0_timestamps[i - 1]
        } else {
            0
        };
        let mut imu_window = Vec::new();
        while imu_index < sequence.imu_data.len() {
            let imu = &sequence.imu_data[imu_index];
            if imu.timestamp_ns > left_ts {
                break;
            }
            if imu.timestamp_ns >= prev_ts {
                imu_window.push(ImuData {
                    timestamp: imu.timestamp_ns as i64,
                    gyro: [imu.gyro_x, imu.gyro_y, imu.gyro_z],
                    accel: [imu.accel_x, imu.accel_y, imu.accel_z],
                });
            }
            imu_index += 1;
        }

        let imu_data = if imu_window.is_empty() {
            None
        } else {
            Some(imu_window.as_slice())
        };

        estimator.process_frame(
            left_img.as_raw(),
            right_img.as_raw(),
            left_ts as i64,
            imu_data,
        )?;

        if (i + 1) % 50 == 0 {
            println!("  Processed {}/{} frames", i + 1, frames_to_process);
        }
    }

    let process_time = process_start.elapsed();
    println!(
        "\n✓ VIO processing complete in {:.2}s",
        process_time.as_secs_f64()
    );
    println!(
        "  - Avg: {:.2}ms per frame",
        process_time.as_secs_f64() * 1000.0 / frames_to_process as f64
    );

    // Convert estimator trajectory to Pose list
    let trajectory = estimator.get_trajectory();
    let est_poses: Vec<Pose> = trajectory
        .iter()
        .map(|(ts_ns, T_W_B)| Pose {
            timestamp_s: *ts_ns as f64 / 1e9,
            position: T_W_B.fixed_view::<3, 1>(0, 3).into(),
            orientation: na::UnitQuaternion::from_matrix(
                &T_W_B.fixed_view::<3, 3>(0, 0).into_owned(),
            ),
        })
        .collect();

    println!("  - Estimated {} poses\n", est_poses.len());

    // Prepare ground truth poses
    let gt_poses: Vec<Pose> = sequence
        .ground_truth
        .iter()
        .map(|gt| Pose {
            timestamp_s: gt.timestamp_ns as f64 / 1e9,
            position: gt.position,
            orientation: gt.orientation,
        })
        .collect();

    // Align by nearest timestamp pairs (20ms threshold)
    let (est_aligned, gt_aligned) = align_trajectories(&est_poses, &gt_poses, 0.02);
    println!("Aligned {} pose pairs for evaluation\n", est_aligned.len());

    if est_aligned.len() < 2 {
        eprintln!("Not enough aligned poses to compute metrics");
        return Ok(());
    }

    // Extract for metrics
    let est_positions: Vec<na::Vector3<f64>> = est_aligned.iter().map(|p| p.position).collect();
    let gt_positions: Vec<na::Vector3<f64>> = gt_aligned.iter().map(|p| p.position).collect();

    let est_pose_pairs: Vec<(na::Vector3<f64>, na::UnitQuaternion<f64>)> = est_aligned
        .iter()
        .map(|p| (p.position, p.orientation))
        .collect();
    let gt_pose_pairs: Vec<(na::Vector3<f64>, na::UnitQuaternion<f64>)> = gt_aligned
        .iter()
        .map(|p| (p.position, p.orientation))
        .collect();

    println!("=== Absolute Trajectory Error (ATE) ===");
    if let Some(ate) = AbsoluteTrajectoryError::calculate(&est_positions, &gt_positions) {
        println!("  RMSE:   {:.4} m", ate.rmse);
        println!("  Mean:   {:.4} m", ate.mean);
        println!("  Median: {:.4} m", ate.median);
        println!("  Std:    {:.4} m", ate.std);
        println!("  Min:    {:.4} m", ate.min);
        println!("  Max:    {:.4} m", ate.max);
    } else {
        eprintln!("Failed to compute ATE");
    }

    println!("\n=== Relative Pose Error (RPE) ===");
    for delta in [1_usize, 5, 10, 20] {
        if delta >= est_pose_pairs.len() {
            continue;
        }

        if let Some(rpe) = RelativePoseError::calculate(&est_pose_pairs, &gt_pose_pairs, delta) {
            let interval_sec = delta as f64 / sequence.frame_rate();
            println!("\nInterval: {} frames ({:.2}s)", delta, interval_sec);
            println!("  Translation RMSE: {:.4} m", rpe.trans_rmse);
            println!("  Translation Mean: {:.4} m", rpe.trans_mean);
            println!("  Rotation RMSE:    {:.4}°", rpe.rot_rmse);
            println!("  Rotation Mean:    {:.4}°", rpe.rot_mean);
        }
    }

    println!("\n=== Summary ===");
    let duration_s =
        est_aligned.last().unwrap().timestamp_s - est_aligned.first().unwrap().timestamp_s;
    println!("✓ Evaluation complete");
    println!("  - Frames processed: {}", frames_to_process);
    println!("  - Pose pairs: {}", est_aligned.len());
    println!("  - Trajectory span: {:.2}s", duration_s.max(0.0));

    Ok(())
}

/// Align trajectories by nearest timestamp under a threshold (seconds)
fn align_trajectories(est: &[Pose], gt: &[Pose], max_dt: f64) -> (Vec<Pose>, Vec<Pose>) {
    let mut est_out = Vec::new();
    let mut gt_out = Vec::new();

    for est_pose in est {
        let mut best: Option<&Pose> = None;
        let mut best_diff = f64::MAX;

        for gt_pose in gt {
            let diff = (est_pose.timestamp_s - gt_pose.timestamp_s).abs();
            if diff < best_diff {
                best_diff = diff;
                best = Some(gt_pose);
            }
        }

        if let Some(gt_pose) = best {
            if best_diff <= max_dt {
                est_out.push(est_pose.clone());
                gt_out.push(gt_pose.clone());
            }
        }
    }

    (est_out, gt_out)
}
