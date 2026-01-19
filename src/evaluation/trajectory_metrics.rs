/// Trajectory evaluation metrics
/// 
/// Computes Absolute Trajectory Error (ATE) and Relative Pose Error (RPE)
/// following TUM RGB-D benchmark evaluation protocol

use nalgebra::{Isometry3};

/// Trajectory metrics results
#[derive(Clone, Debug)]
pub struct TrajectoryMetrics {
    /// Absolute Trajectory Error in meters
    pub ate_mean: f64,
    pub ate_std: f64,
    pub ate_rmse: f64,  // Root Mean Square Error
    
    /// Relative Pose Error (drift over fixed distance)
    pub rpe_translation_mean: f64,
    pub rpe_translation_std: f64,
    pub rpe_rotation_mean: f64,  // in degrees
    pub rpe_rotation_std: f64,
    
    /// Max absolute error
    pub ate_max: f64,
    pub ate_min: f64,
    
    /// Number of frames evaluated
    pub frame_count: usize,
}

/// Compute Absolute Trajectory Error
/// Measures how far estimated trajectory deviates from ground truth
/// 
/// # Arguments
/// * `estimated_poses` - Estimated SE(3) poses from VIO
/// * `ground_truth_poses` - Ground truth SE(3) poses
/// * `timestamps` - Frame timestamps for alignment
/// 
/// Returns ATE statistics in meters
pub fn compute_ate(
    estimated_poses: &[Isometry3<f64>],
    ground_truth_poses: &[Isometry3<f64>],
    _timestamps: &[i64],
) -> Result<TrajectoryMetrics, String> {
    if estimated_poses.len() != ground_truth_poses.len() {
        return Err("Pose arrays must have equal length".to_string());
    }
    
    if estimated_poses.is_empty() {
        return Err("No poses to evaluate".to_string());
    }
    
    // Compute pairwise differences
    let mut errors = Vec::new();
    
    for (est, gt) in estimated_poses.iter().zip(ground_truth_poses.iter()) {
        // Translation error
        let est_trans = est.translation.vector;
        let gt_trans = gt.translation.vector;
        let error = (est_trans - gt_trans).norm();
        errors.push(error);
    }
    
    // Compute statistics
    let mean = errors.iter().sum::<f64>() / errors.len() as f64;
    let variance = errors.iter()
        .map(|&e| (e - mean).powi(2))
        .sum::<f64>() / errors.len() as f64;
    let std = variance.sqrt();
    
    let rmse = (errors.iter()
        .map(|&e| e.powi(2))
        .sum::<f64>() / errors.len() as f64).sqrt();
    
    let max = errors.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    let min = errors.iter().copied().fold(f64::INFINITY, f64::min);
    
    Ok(TrajectoryMetrics {
        ate_mean: mean,
        ate_std: std,
        ate_rmse: rmse,
        rpe_translation_mean: 0.0,  // Computed separately
        rpe_translation_std: 0.0,
        rpe_rotation_mean: 0.0,
        rpe_rotation_std: 0.0,
        ate_max: max,
        ate_min: min,
        frame_count: estimated_poses.len(),
    })
}

/// Compute Relative Pose Error
/// Measures drift over fixed distance (e.g., every 0.1m or 1 second)
pub fn compute_rpe(
    estimated_poses: &[Isometry3<f64>],
    ground_truth_poses: &[Isometry3<f64>],
    segment_length: f64,  // in meters
) -> Result<TrajectoryMetrics, String> {
    if estimated_poses.len() != ground_truth_poses.len() {
        return Err("Pose arrays must have equal length".to_string());
    }
    
    // Compute ground truth distances
    let mut distances = vec![0.0];
    for i in 1..ground_truth_poses.len() {
        let prev_trans = ground_truth_poses[i - 1].translation.vector;
        let curr_trans = ground_truth_poses[i].translation.vector;
        let dist = (curr_trans - prev_trans).norm();
        distances.push(distances[i - 1] + dist);
    }
    
    // Find segment pairs
    let mut translation_errors = Vec::new();
    let mut rotation_errors = Vec::new();
    
    for i in 0..ground_truth_poses.len() {
        let current_dist = distances[i];
        let target_dist = current_dist + segment_length;
        
        // Find closest frame to target distance
        let mut best_j = i;
        let mut best_dist_diff = f64::INFINITY;
        
        for j in (i + 1)..ground_truth_poses.len() {
            let diff = (distances[j] - target_dist).abs();
            if diff < best_dist_diff {
                best_dist_diff = diff;
                best_j = j;
            }
        }
        
        if best_j <= i {
            continue;
        }
        
        // Compute relative pose error
        let est_i = &estimated_poses[i];
        let est_j = &estimated_poses[best_j];
        let gt_i = &ground_truth_poses[i];
        let gt_j = &ground_truth_poses[best_j];
        
        // Relative transformation error
        let est_rel = est_i.inverse() * est_j;
        let gt_rel = gt_i.inverse() * gt_j;
        
        // Translation error
        let trans_error = (est_rel.translation.vector - gt_rel.translation.vector).norm();
        translation_errors.push(trans_error);
        
        // Rotation error (angle in degrees)
        let est_rel_rot = est_rel.rotation;
        let gt_rel_rot = gt_rel.rotation;
        
        // Compute angle difference using axis-angle representation
        let diff_rot = est_rel_rot.inverse() * gt_rel_rot;
        let angle = diff_rot.angle().to_degrees();
        rotation_errors.push(angle);
    }
    
    if translation_errors.is_empty() {
        return Err("Not enough frames for RPE computation".to_string());
    }
    
    let trans_mean = translation_errors.iter().sum::<f64>() / translation_errors.len() as f64;
    let trans_variance = translation_errors.iter()
        .map(|&e| (e - trans_mean).powi(2))
        .sum::<f64>() / translation_errors.len() as f64;
    let trans_std = trans_variance.sqrt();
    
    let rot_mean = rotation_errors.iter().sum::<f64>() / rotation_errors.len() as f64;
    let rot_variance = rotation_errors.iter()
        .map(|&e| (e - rot_mean).powi(2))
        .sum::<f64>() / rotation_errors.len() as f64;
    let rot_std = rot_variance.sqrt();
    
    Ok(TrajectoryMetrics {
        ate_mean: 0.0,
        ate_std: 0.0,
        ate_rmse: 0.0,
        rpe_translation_mean: trans_mean,
        rpe_translation_std: trans_std,
        rpe_rotation_mean: rot_mean,
        rpe_rotation_std: rot_std,
        ate_max: translation_errors.iter().copied().fold(f64::NEG_INFINITY, f64::max),
        ate_min: translation_errors.iter().copied().fold(f64::INFINITY, f64::min),
        frame_count: estimated_poses.len(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Vector3;
    
    #[test]
    fn test_ate_perfect_alignment() {
        // Create identical poses
        let poses = vec![
            Isometry3::new(Vector3::new(0.0, 0.0, 0.0), Vector3::zeros()),
            Isometry3::new(Vector3::new(1.0, 0.0, 0.0), Vector3::zeros()),
            Isometry3::new(Vector3::new(2.0, 0.0, 0.0), Vector3::zeros()),
        ];
        
        let timestamps = vec![0, 1, 2];
        let metrics = compute_ate(&poses, &poses, &timestamps).unwrap();
        
        assert!(metrics.ate_mean < 1e-10);
        assert!(metrics.ate_rmse < 1e-10);
    }
    
    #[test]
    fn test_ate_constant_offset() {
        let pose1 = Isometry3::new(Vector3::new(0.0, 0.0, 0.0), Vector3::zeros());
        let pose2 = Isometry3::new(Vector3::new(1.0, 0.0, 0.0), Vector3::zeros());
        
        let estimated = vec![pose1, pose2];
        let ground_truth = vec![
            Isometry3::new(Vector3::new(0.1, 0.0, 0.0), Vector3::zeros()),
            Isometry3::new(Vector3::new(1.1, 0.0, 0.0), Vector3::zeros()),
        ];
        
        let timestamps = vec![0, 1];
        let metrics = compute_ate(&estimated, &ground_truth, &timestamps).unwrap();
        
        // Should have 0.1m constant error
        assert!((metrics.ate_mean - 0.1).abs() < 1e-10);
    }
}
