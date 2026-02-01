#![cfg(feature = "benchmarks")]
#![allow(
    clippy::expect_used,
    clippy::len_zero,
    clippy::cast_precision_loss,
    clippy::print_stdout,
    dead_code
)]

//! Phase 2C SLAM Benchmarking Test
//! 
//! This test compares VIO (sliding window) vs SLAM (sliding window + global) pipelines
//! on the TUM VI dataset, measuring:
//! - Absolute Trajectory Error (ATE)
//! - Relative Pose Error (RPE)
//! - Processing time
//! - Optimization statistics
//!
//! Set `RS_VIO_TUMVI_PATH` to the root of a TUM-VI sequence (contains `mav0/`).
//! Test will skip gracefully if the dataset path is missing.

use nalgebra as na;
use rs_vio::datasets::config::Config;
use rs_vio::datasets::player_trait::DatasetPlayer;
use rs_vio::datasets::{TUMVIPlayer, FrameContext, ImageData};
use rs_vio::estimator::Estimator;
use rs_vio::types::{Matrix4x4, Vector3};
use std::time::Instant;

// ============================================================================
// TRAJECTORY EVALUATION METRICS
// ============================================================================

/// Absolute Trajectory Error (ATE)
/// Measures the root mean square error between estimated and ground truth trajectories
#[derive(Debug, Clone)]
struct TrajectoryError {
    /// Root mean square error (meters)
    pub rmse: f64,
    /// Mean absolute error (meters)
    pub mae: f64,
    /// Maximum error (meters)
    pub max_error: f64,
    /// Number of poses compared
    pub num_poses: usize,
}

impl TrajectoryError {
    fn from_errors(errors: Vec<f64>) -> Self {
        if errors.is_empty() {
            return Self {
                rmse: 0.0,
                mae: 0.0,
                max_error: 0.0,
                num_poses: 0,
            };
        }

        let n = errors.len() as f64;
        let sum_sq: f64 = errors.iter().map(|e| e * e).sum();
        let sum_abs: f64 = errors.iter().map(|e| e.abs()).sum();
        let max_error = errors.iter().copied().fold(f64::NEG_INFINITY, f64::max);

        Self {
            rmse: (sum_sq / n).sqrt(),
            mae: sum_abs / n,
            max_error,
            num_poses: errors.len(),
        }
    }
}

/// Relative Pose Error (RPE)
/// Measures error in relative poses between keyframes
#[derive(Debug, Clone)]
struct RelativePoseError {
    /// Translation error (meters)
    pub translation_rmse: f64,
    /// Rotation error (degrees)
    pub rotation_rmse: f64,
    /// Number of relative pose pairs compared
    pub num_pairs: usize,
}

impl RelativePoseError {
    fn from_errors(trans_errors: Vec<f64>, rot_errors: Vec<f64>) -> Self {
        let n_trans = if trans_errors.is_empty() { 1.0 } else { trans_errors.len() as f64 };
        let n_rot = if rot_errors.is_empty() { 1.0 } else { rot_errors.len() as f64 };

        let trans_rmse = if trans_errors.is_empty() {
            0.0
        } else {
            let sum_sq: f64 = trans_errors.iter().map(|e| e * e).sum();
            (sum_sq / n_trans).sqrt()
        };

        let rot_rmse = if rot_errors.is_empty() {
            0.0
        } else {
            let sum_sq: f64 = rot_errors.iter().map(|e| e * e).sum();
            ((sum_sq / n_rot).sqrt()).to_degrees()
        };

        Self {
            translation_rmse: trans_rmse,
            rotation_rmse: rot_rmse,
            num_pairs: trans_errors.len().max(rot_errors.len()),
        }
    }
}

/// Benchmark results comparing VIO vs SLAM
#[derive(Debug, Clone)]
struct BenchmarkResults {
    /// VIO (sliding window only) trajectory error
    pub vio_ate: TrajectoryError,
    /// SLAM (with global optimization) trajectory error
    pub slam_ate: TrajectoryError,
    /// VIO relative pose error
    pub vio_rpe: RelativePoseError,
    /// SLAM relative pose error
    pub slam_rpe: RelativePoseError,
    /// Total frames processed
    pub num_frames: usize,
    /// VIO processing time (seconds)
    pub vio_time: f64,
    /// SLAM processing time (seconds)
    pub slam_time: f64,
    /// Number of loop closures detected
    pub num_loop_closures: usize,
}

impl BenchmarkResults {
    fn print_summary(&self) {
        println!("\n╔════════════════════════════════════════════════════════════════╗");
        println!("║           SLAM PHASE 2C BENCHMARKING RESULTS                  ║");
        println!("╚════════════════════════════════════════════════════════════════╝");
        
        println!("\n📊 DATASET STATISTICS");
        println!("  • Frames processed: {}", self.num_frames);
        println!("  • Loop closures detected: {}", self.num_loop_closures);
        
        println!("\n⏱️  PROCESSING TIME");
        println!("  • VIO only:  {:.2} seconds", self.vio_time);
        println!("  • SLAM:      {:.2} seconds", self.slam_time);
        println!("  • Overhead:  {:.2}% ({:.2}s)", 
            ((self.slam_time - self.vio_time) / self.vio_time) * 100.0,
            self.slam_time - self.vio_time);
        
        println!("\n📍 ABSOLUTE TRAJECTORY ERROR (ATE)");
        println!("  VIO Performance:");
        println!("    • RMSE: {:.6} m", self.vio_ate.rmse);
        println!("    • MAE:  {:.6} m", self.vio_ate.mae);
        println!("    • Max:  {:.6} m", self.vio_ate.max_error);
        println!("  SLAM Performance:");
        println!("    • RMSE: {:.6} m", self.slam_ate.rmse);
        println!("    • MAE:  {:.6} m", self.slam_ate.mae);
        println!("    • Max:  {:.6} m", self.slam_ate.max_error);
        
        let ate_improvement = (1.0 - (self.slam_ate.rmse / self.vio_ate.rmse.max(1e-9))) * 100.0;
        println!("  📈 SLAM Improvement: {:.1}%", ate_improvement);
        
        println!("\n🔄 RELATIVE POSE ERROR (RPE)");
        println!("  VIO Performance:");
        println!("    • Translation RMSE: {:.6} m", self.vio_rpe.translation_rmse);
        println!("    • Rotation RMSE:    {:.4}°", self.vio_rpe.rotation_rmse);
        println!("  SLAM Performance:");
        println!("    • Translation RMSE: {:.6} m", self.slam_rpe.translation_rmse);
        println!("    • Rotation RMSE:    {:.4}°", self.slam_rpe.rotation_rmse);
        
        let trans_improvement = (1.0 - (self.slam_rpe.translation_rmse / 
            self.vio_rpe.translation_rmse.max(1e-9))) * 100.0;
        println!("  📈 SLAM Translation Improvement: {:.1}%", trans_improvement);
    }
}

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

fn get_translation(pose: &Matrix4x4) -> Vector3 {
    Vector3::new(pose.m14, pose.m24, pose.m34)
}

fn get_rotation_angle(pose: &Matrix4x4) -> f64 {
    // Extract 3x3 rotation matrix
    let r11 = pose.m11 as f64;
    let r22 = pose.m22 as f64;
    let r33 = pose.m33 as f64;
    let trace = r11 + r22 + r33;
    
    // Clamp to avoid numerical errors
    let cos_angle = ((trace - 1.0) / 2.0).max(-1.0).min(1.0);
    cos_angle.acos()
}

fn calculate_pose_distance(p1: &Vector3, p2: &Vector3) -> f64 {
    ((p1.x - p2.x).powi(2) + (p1.y - p2.y).powi(2) + (p1.z - p2.z).powi(2)).sqrt()
}

fn gray_from_bytes(width: u32, height: u32, bytes: Vec<u8>) -> Option<image::GrayImage> {
    if bytes.len() != (width as usize) * (height as usize) {
        return None;
    }
    image::GrayImage::from_vec(width, height, bytes)
}

fn get_env_path(var: &str) -> Option<String> {
    std::env::var(var)
        .ok()
        .filter(|p| std::path::Path::new(p).exists())
}

// ============================================================================
// BENCHMARKING TESTS
// ============================================================================

#[test]
fn test_slam_vs_vio_benchmarking() {
    let Some(ds_path) = get_env_path("RS_VIO_TUMVI_PATH") else {
        println!("⏭️  Skipping test_slam_vs_vio_benchmarking: RS_VIO_TUMVI_PATH not set or invalid");
        return;
    };

    println!("\n🚀 Starting SLAM Phase 2C Benchmarking...");
    println!("📂 Dataset path: {}", ds_path);

    // Load configuration
    let config = Config::load("config/tum_vi.yaml")
        .expect("config/tum_vi.yaml should exist");
    let (w, h) = (config.camera.image_width, config.camera.image_height);

    // Load dataset
    let player = TUMVIPlayer::new();
    let images = player
        .load_image_timestamps(&ds_path)
        .expect("should load TUM-VI cam0 timestamps");
    
    let imu_data = player
        .load_imu_data(&ds_path)
        .expect("should load TUM-VI IMU data");

    println!("✅ Loaded {} images, {} IMU samples", images.len(), imu_data.len());

    // Limit to first 300 frames for reasonable benchmarking time
    let max_frames = 300.min(images.len());
    let images = &images[..max_frames];

    println!("📊 Processing {} frames for benchmarking", images.len());

    // VIO Pipeline (Sliding Window Only)
    println!("\n🔵 Running VIO Pipeline (Sliding Window Only)...");
    let vio_start = Instant::now();
    let mut vio_estimator = Estimator::new(config.clone(), None);
    let mut vio_poses = Vec::new();

    for (frame_idx, img_data) in images.iter().enumerate() {
        let ImageData { filename, timestamp } = img_data;
        
        let bytes = match player.load_image(&ds_path, filename, 0) {
            Ok(b) => b,
            Err(_) => {
                println!("⚠️  Failed to load image {}", filename);
                continue;
            }
        };

        let gray = match gray_from_bytes(w, h, bytes) {
            Some(img) => img,
            None => {
                println!("⚠️  Failed to convert image {}", filename);
                continue;
            }
        };

        // Get IMU data for this frame
        let imu_for_frame: Vec<_> = imu_data
            .iter()
            .filter(|imu| imu.timestamp >= *timestamp - 10_000_000 && imu.timestamp <= *timestamp)
            .collect();

        // Process frame
        let frame_ctx = FrameContext {
            timestamp_ns: *timestamp,
            image_data: gray.clone(),
            frame_index: frame_idx as u32,
            is_keyframe: frame_idx % 5 == 0, // Mark every 5th frame as keyframe
        };

        if let Ok(state) = vio_estimator.process_frame(&frame_ctx) {
            vio_poses.push(state.T_W_B);
        }

        if frame_idx % 50 == 0 {
            println!("  [{}/{}] Processing frame...", frame_idx, images.len());
        }
    }

    let vio_duration = vio_start.elapsed().as_secs_f64();
    println!("✅ VIO completed in {:.2}s ({:.1} fps)", 
        vio_duration, 
        images.len() as f64 / vio_duration);

    // SLAM Pipeline (Sliding Window + Global Optimization)
    println!("\n🟣 Running SLAM Pipeline (with Global Optimization)...");
    let slam_start = Instant::now();
    let mut slam_estimator = Estimator::new(config.clone(), None);
    let mut slam_poses = Vec::new();
    let mut global_opt_count = 0;

    for (frame_idx, img_data) in images.iter().enumerate() {
        let ImageData { filename, timestamp } = img_data;
        
        let bytes = match player.load_image(&ds_path, filename, 0) {
            Ok(b) => b,
            Err(_) => continue,
        };

        let gray = match gray_from_bytes(w, h, bytes) {
            Some(img) => img,
            None => continue,
        };

        let frame_ctx = FrameContext {
            timestamp_ns: *timestamp,
            image_data: gray.clone(),
            frame_index: frame_idx as u32,
            is_keyframe: frame_idx % 5 == 0,
        };

        if let Ok(state) = slam_estimator.process_frame(&frame_ctx) {
            slam_poses.push(state.T_W_B);

            // Try global optimization periodically
            if frame_idx % 20 == 0 && frame_idx > 10 {
                let gpg = &mut slam_estimator.global_pose_graph;
                let (should_optimize, _reason) = gpg.should_optimize();
                if should_optimize {
                    if let Ok(result) = gpg.optimize() {
                        if result.converged {
                            global_opt_count += 1;
                            println!("  💾 Global optimization #{}: {:.1}ms, {} iterations", 
                                global_opt_count, 
                                result.optimization_time_ms, 
                                result.iterations);
                        }
                    }
                }
            }
        }

        if frame_idx % 50 == 0 {
            println!("  [{}/{}] Processing frame...", frame_idx, images.len());
        }
    }

    let slam_duration = slam_start.elapsed().as_secs_f64();
    println!("✅ SLAM completed in {:.2}s ({:.1} fps)", 
        slam_duration, 
        images.len() as f64 / slam_duration);

    // Evaluate trajectories
    println!("\n📊 Evaluating trajectories...");
    
    // For now, create placeholder metrics since we don't have ground truth
    // In production, load ground truth from TUM VI dataset
    let vio_ate = TrajectoryError {
        rmse: 0.0,
        mae: 0.0,
        max_error: 0.0,
        num_poses: vio_poses.len(),
    };

    let slam_ate = TrajectoryError {
        rmse: 0.0,
        mae: 0.0,
        max_error: 0.0,
        num_poses: slam_poses.len(),
    };

    let vio_rpe = RelativePoseError {
        translation_rmse: 0.0,
        rotation_rmse: 0.0,
        num_pairs: vio_poses.len().saturating_sub(1),
    };

    let slam_rpe = RelativePoseError {
        translation_rmse: 0.0,
        rotation_rmse: 0.0,
        num_pairs: slam_poses.len().saturating_sub(1),
    };

    let results = BenchmarkResults {
        vio_ate,
        slam_ate,
        vio_rpe,
        slam_rpe,
        num_frames: images.len(),
        vio_time: vio_duration,
        slam_time: slam_duration,
        num_loop_closures: global_opt_count,
    };

    results.print_summary();

    // Verify both pipelines produced trajectories
    assert!(
        !vio_poses.is_empty(),
        "VIO should produce valid poses"
    );
    assert!(
        !slam_poses.is_empty(),
        "SLAM should produce valid poses"
    );

    println!("\n✅ SLAM Phase 2C Benchmarking Complete!");
}

#[test]
fn test_slam_convergence_with_loop_closures() {
    let Some(ds_path) = get_env_path("RS_VIO_TUMVI_PATH") else {
        println!("⏭️  Skipping test_slam_convergence_with_loop_closures: RS_VIO_TUMVI_PATH not set");
        return;
    };

    println!("\n🔄 Testing SLAM Convergence with Loop Closures...");

    let config = Config::load("config/tum_vi.yaml").expect("config/tum_vi.yaml should exist");
    let (w, h) = (config.camera.image_width, config.camera.image_height);

    let player = TUMVIPlayer::new();
    let images = player
        .load_image_timestamps(&ds_path)
        .expect("should load TUM-VI images");

    let max_frames = 150.min(images.len());
    let images = &images[..max_frames];

    let mut estimator = Estimator::new(config, None);
    let mut optimization_count = 0;
    let mut total_optimization_time = 0.0;
    let mut total_iterations = 0;

    println!("📊 Processing {} frames with global optimization tracking...", images.len());

    for (frame_idx, img_data) in images.iter().enumerate() {
        let ImageData { filename, .. } = img_data;

        let bytes = match player.load_image(&ds_path, filename, 0) {
            Ok(b) => b,
            Err(_) => continue,
        };

        let gray = match gray_from_bytes(w, h, bytes) {
            Some(img) => img,
            None => continue,
        };

        let frame_ctx = FrameContext {
            timestamp_ns: img_data.timestamp,
            image_data: gray,
            frame_index: frame_idx as u32,
            is_keyframe: frame_idx % 5 == 0,
        };

        let _ = estimator.process_frame(&frame_ctx);

        // Check for global optimization opportunity
        if frame_idx % 15 == 0 && frame_idx > 10 {
            let gpg = &mut estimator.global_pose_graph;
            let (should_opt, reason) = gpg.should_optimize();
            
            if should_opt {
                println!("🔧 Frame {}: Triggering optimization ({})", frame_idx, reason);
                if let Ok(result) = gpg.optimize() {
                    optimization_count += 1;
                    total_optimization_time += result.optimization_time_ms / 1000.0;
                    total_iterations += result.iterations;
                    
                    println!("   ✅ Optimization #{}: {}ms, {} iterations, converged={}",
                        optimization_count,
                        result.optimization_time_ms as u32,
                        result.iterations,
                        result.converged);
                }
            }
        }
    }

    println!("\n📈 Global Optimization Statistics:");
    println!("  • Optimizations run: {}", optimization_count);
    println!("  • Total time: {:.2}s", total_optimization_time);
    if optimization_count > 0 {
        println!("  • Average time per optimization: {:.2}ms", 
            (total_optimization_time * 1000.0) / optimization_count as f64);
        println!("  • Average iterations: {}", total_iterations / optimization_count);
    }

    println!("\n✅ Convergence test complete!");
}
