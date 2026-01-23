//! TUM-VI Accuracy Validation Example
//! 
//! Loads TUM-VI dataset and computes trajectory accuracy metrics (ATE/RPE)
//! against ground truth data.

use rs_vio::datasets::tum_vi::load_all_sequences;
use rs_vio::datasets::trajectory_eval::{AbsoluteTrajectoryError, RelativePoseError};
use nalgebra as na;
use std::env;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Get dataset directory
    let dataset_dir = env::var("TUM_VI_DIR")
        .unwrap_or_else(|_| {
            if std::path::Path::new("./datasets/tum_vi").exists() {
                "./datasets/tum_vi".to_string()
            } else {
                "./data/tum_vi".to_string()
            }
        });
    
    println!("Loading TUM-VI dataset from: {}", dataset_dir);
    
    // Load all sequences
    let sequences = load_all_sequences(&dataset_dir)?;
    
    println!("\n=== TUM-VI Dataset Summary ===");
    println!("Found {} sequences\n", sequences.len());
    
    for seq in &sequences {
        println!("Sequence: {}", seq.name);
        println!("  Frames: {} @ {:.1} Hz", seq.num_frames(), seq.frame_rate());
        println!("  IMU measurements: {}", seq.imu_data.len());
        println!("  Ground truth poses: {}", seq.ground_truth.len());
        
        if seq.ground_truth.len() > 2 {
            // For demonstration, compute ATE/RPE assuming estimated trajectory = ground truth
            // In real VIO, you'd run the pipeline and get estimated poses
            
            // Convert ground truth to format expected by trajectory_eval
            let positions: Vec<na::Vector3<f64>> = seq.ground_truth.iter()
                .map(|gt| gt.position)
                .collect();
            
            let poses: Vec<(na::Vector3<f64>, na::UnitQuaternion<f64>)> = seq.ground_truth.iter()
                .map(|gt| (gt.position, gt.orientation))
                .collect();
            
            // Compute ATE (should be ~0 since estimated = ground truth)
            if let Some(ate) = AbsoluteTrajectoryError::calculate(&positions, &positions) {
                println!("\n  Absolute Trajectory Error (ATE):");
                println!("    RMSE: {:.6} m", ate.rmse);
                println!("    Mean: {:.6} m", ate.mean);
                println!("    Median: {:.6} m", ate.median);
                println!("    Std: {:.6} m", ate.std);
                println!("    Min: {:.6} m", ate.min);
                println!("    Max: {:.6} m", ate.max);
            } else {
                eprintln!("  Failed to compute ATE");
            }
            
            // Compute RPE at 1 frame interval
            if let Some(rpe) = RelativePoseError::calculate(&poses, &poses, 1) {
                println!("\n  Relative Pose Error (RPE @ 1 frame):");
                println!("    Translation RMSE: {:.6} m", rpe.trans_rmse);
                println!("    Translation Mean: {:.6} m", rpe.trans_mean);
                println!("    Rotation RMSE: {:.6}°", rpe.rot_rmse);
                println!("    Rotation Mean: {:.6}°", rpe.rot_mean);
            } else {
                eprintln!("  Failed to compute RPE");
            }
        }
        
        println!("\n{}", "=".repeat(60));
    }
    
    Ok(())
}
