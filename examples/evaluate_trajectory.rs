//! Trajectory Accuracy Evaluation
//!
//! Loads estimated trajectory and ground truth, computes ATE/RPE metrics.
//! 
//! Usage:
//!   cargo run --release --example evaluate_trajectory <estimated> <ground_truth>
//!
//! Example:
//!   cargo run --release --example evaluate_trajectory \
//!     trajectory_room1_20260123_203511.txt \
//!     ./datasets/tum_vi/room1/mav0/mocap0/data.csv

use rs_vio::datasets::trajectory_eval::{AbsoluteTrajectoryError, RelativePoseError};
use nalgebra as na;
use std::env;
use std::fs::File;
use std::io::{BufRead, BufReader};

/// Trajectory pose (timestamp, position, quaternion)
#[derive(Debug, Clone)]
struct Pose {
    timestamp: f64,
    position: na::Vector3<f64>,
    orientation: na::UnitQuaternion<f64>,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = env::args().collect();
    
    if args.len() < 3 {
        eprintln!("Usage: {} <estimated_trajectory.txt> <ground_truth.csv>", args[0]);
        eprintln!("\nExample:");
        eprintln!("  {} trajectory_room1.txt ./datasets/tum_vi/room1/mav0/mocap0/data.csv", args[0]);
        std::process::exit(1);
    }
    
    let estimated_file = &args[1];
    let ground_truth_file = &args[2];
    
    println!("=== Trajectory Accuracy Evaluation ===\n");
    println!("Estimated: {}", estimated_file);
    println!("Ground truth: {}\n", ground_truth_file);
    
    // Load trajectories
    let estimated = load_tum_trajectory(estimated_file)?;
    let ground_truth = load_mocap_trajectory(ground_truth_file)?;
    
    println!("Loaded {} estimated poses", estimated.len());
    println!("Loaded {} ground truth poses\n", ground_truth.len());
    
    if estimated.is_empty() || ground_truth.is_empty() {
        eprintln!("Error: Empty trajectory file(s)");
        std::process::exit(1);
    }
    
    // Align trajectories by timestamp
    let (est_aligned, gt_aligned) = align_trajectories(&estimated, &ground_truth);
    
    println!("Aligned {} pose pairs\n", est_aligned.len());
    
    if est_aligned.len() < 2 {
        eprintln!("Error: Not enough aligned poses for evaluation");
        std::process::exit(1);
    }
    
    // Extract positions for ATE
    let est_positions: Vec<na::Vector3<f64>> = est_aligned.iter()
        .map(|p| p.position)
        .collect();
    let gt_positions: Vec<na::Vector3<f64>> = gt_aligned.iter()
        .map(|p| p.position)
        .collect();
    
    // Extract poses (position + orientation) for RPE
    let est_poses: Vec<(na::Vector3<f64>, na::UnitQuaternion<f64>)> = est_aligned.iter()
        .map(|p| (p.position, p.orientation))
        .collect();
    let gt_poses: Vec<(na::Vector3<f64>, na::UnitQuaternion<f64>)> = gt_aligned.iter()
        .map(|p| (p.position, p.orientation))
        .collect();
    
    // Compute ATE
    println!("=== Absolute Trajectory Error (ATE) ===");
    if let Some(ate) = AbsoluteTrajectoryError::calculate(&est_positions, &gt_positions) {
        println!("  RMSE:   {:.6} m", ate.rmse);
        println!("  Mean:   {:.6} m", ate.mean);
        println!("  Median: {:.6} m", ate.median);
        println!("  Std:    {:.6} m", ate.std);
        println!("  Min:    {:.6} m", ate.min);
        println!("  Max:    {:.6} m", ate.max);
    } else {
        eprintln!("Failed to compute ATE");
    }
    
    // Compute RPE at different intervals
    println!("\n=== Relative Pose Error (RPE) ===");
    for delta in [1, 5, 10, 20] {
        if delta >= est_poses.len() {
            continue;
        }
        
        if let Some(rpe) = RelativePoseError::calculate(&est_poses, &gt_poses, delta) {
            let interval_sec = delta as f64 / 20.0; // Assuming 20 Hz
            println!("\nInterval: {} frames ({:.2}s)", delta, interval_sec);
            println!("  Translation RMSE: {:.6} m", rpe.trans_rmse);
            println!("  Translation Mean: {:.6} m", rpe.trans_mean);
            println!("  Rotation RMSE:    {:.6}°", rpe.rot_rmse);
            println!("  Rotation Mean:    {:.6}°", rpe.rot_mean);
        }
    }
    
    println!("\n=== Summary ===");
    println!("✓ Evaluation complete");
    println!("  - Trajectory length: {:.1}s", 
             est_aligned.last().unwrap().timestamp - est_aligned.first().unwrap().timestamp);
    println!("  - Pose pairs: {}", est_aligned.len());
    println!("  - ATE RMSE: {:.6} m", 
             AbsoluteTrajectoryError::calculate(&est_positions, &gt_positions)
                 .map(|ate| ate.rmse).unwrap_or(0.0));
    
    Ok(())
}

/// Load TUM format trajectory: timestamp tx ty tz qx qy qz qw
fn load_tum_trajectory(filename: &str) -> Result<Vec<Pose>, Box<dyn std::error::Error>> {
    let file = File::open(filename)?;
    let reader = BufReader::new(file);
    let mut poses = Vec::new();
    
    for line in reader.lines() {
        let line = line?;
        let line = line.trim();
        
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        
        let parts: Vec<&str> = line.split_whitespace().collect();
        if parts.len() >= 8 {
            let timestamp: f64 = parts[0].parse()?;
            let tx: f64 = parts[1].parse()?;
            let ty: f64 = parts[2].parse()?;
            let tz: f64 = parts[3].parse()?;
            let qx: f64 = parts[4].parse()?;
            let qy: f64 = parts[5].parse()?;
            let qz: f64 = parts[6].parse()?;
            let qw: f64 = parts[7].parse()?;
            
            poses.push(Pose {
                timestamp,
                position: na::Vector3::new(tx, ty, tz),
                orientation: na::UnitQuaternion::from_quaternion(
                    na::Quaternion::new(qw, qx, qy, qz)
                ),
            });
        }
    }
    
    Ok(poses)
}

/// Load mocap ground truth: timestamp,px,py,pz,qw,qx,qy,qz,vx,vy,vz,bwx,bwy,bwz,bax,bay,baz
fn load_mocap_trajectory(filename: &str) -> Result<Vec<Pose>, Box<dyn std::error::Error>> {
    let file = File::open(filename)?;
    let reader = BufReader::new(file);
    let mut poses = Vec::new();
    
    for (line_num, line) in reader.lines().enumerate() {
        let line = line?;
        let line = line.trim();
        
        // Skip header
        if line_num == 0 || line.is_empty() || line.starts_with('#') {
            continue;
        }
        
        let parts: Vec<&str> = line.split(',').collect();
        if parts.len() >= 8 {
            let timestamp_ns: u64 = parts[0].parse()?;
            let timestamp = timestamp_ns as f64 / 1e9;
            
            let px: f64 = parts[1].parse()?;
            let py: f64 = parts[2].parse()?;
            let pz: f64 = parts[3].parse()?;
            let qw: f64 = parts[4].parse()?;
            let qx: f64 = parts[5].parse()?;
            let qy: f64 = parts[6].parse()?;
            let qz: f64 = parts[7].parse()?;
            
            poses.push(Pose {
                timestamp,
                position: na::Vector3::new(px, py, pz),
                orientation: na::UnitQuaternion::from_quaternion(
                    na::Quaternion::new(qw, qx, qy, qz)
                ),
            });
        }
    }
    
    Ok(poses)
}

/// Align trajectories by timestamp (find closest matches)
fn align_trajectories(estimated: &[Pose], ground_truth: &[Pose]) -> (Vec<Pose>, Vec<Pose>) {
    let mut est_aligned = Vec::new();
    let mut gt_aligned = Vec::new();
    
    let max_time_diff = 0.02; // 20ms threshold
    
    for est_pose in estimated {
        // Find closest ground truth pose
        let mut best_match: Option<&Pose> = None;
        let mut best_diff = f64::MAX;
        
        for gt_pose in ground_truth {
            let diff = (est_pose.timestamp - gt_pose.timestamp).abs();
            if diff < best_diff {
                best_diff = diff;
                best_match = Some(gt_pose);
            }
        }
        
        if let Some(gt_pose) = best_match {
            if best_diff < max_time_diff {
                est_aligned.push(est_pose.clone());
                gt_aligned.push(gt_pose.clone());
            }
        }
    }
    
    (est_aligned, gt_aligned)
}
