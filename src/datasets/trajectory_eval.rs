//! Trajectory Evaluation Metrics
//! 
//! Implements standard VIO/SLAM accuracy metrics:
//! - ATE (Absolute Trajectory Error)
//! - RPE (Relative Pose Error)
//! 
//! Based on TUM RGB-D benchmark evaluation tools

use nalgebra as na;
use std::f64;

/// Absolute Trajectory Error (ATE)
/// 
/// Measures global consistency of the estimated trajectory.
/// Compares estimated poses to ground truth after Sim(3) alignment.
#[derive(Debug, Clone, Copy)]
pub struct AbsoluteTrajectoryError {
    /// Root mean square error (meters)
    pub rmse: f64,
    /// Mean error (meters)
    pub mean: f64,
    /// Median error (meters)
    pub median: f64,
    /// Standard deviation (meters)
    pub std: f64,
    /// Minimum error (meters)
    pub min: f64,
    /// Maximum error (meters)
    pub max: f64,
}

impl AbsoluteTrajectoryError {
    /// Calculate ATE from estimated and ground truth trajectories
    /// 
    /// # Arguments
    /// * `estimated` - Estimated 3D positions (N×3)
    /// * `ground_truth` - Ground truth 3D positions (N×3)
    /// 
    /// Assumes trajectories are already timestamp-aligned
    pub fn calculate(
        estimated: &[na::Vector3<f64>],
        ground_truth: &[na::Vector3<f64>],
    ) -> Option<Self> {
        if estimated.is_empty() || estimated.len() != ground_truth.len() {
            return None;
        }
        
        // Calculate position errors (Euclidean distance)
        let mut errors: Vec<f64> = estimated
            .iter()
            .zip(ground_truth.iter())
            .map(|(est, gt)| (est - gt).norm())
            .collect();
        
        // Calculate statistics
        let mean = errors.iter().sum::<f64>() / errors.len() as f64;
        let rmse = (errors.iter().map(|e| e * e).sum::<f64>() / errors.len() as f64).sqrt();
        
        errors.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let median = if errors.len() % 2 == 0 {
            (errors[errors.len() / 2 - 1] + errors[errors.len() / 2]) / 2.0
        } else {
            errors[errors.len() / 2]
        };
        
        let variance = errors.iter()
            .map(|e| (e - mean).powi(2))
            .sum::<f64>() / errors.len() as f64;
        let std = variance.sqrt();
        
        let min = errors.iter().cloned().fold(f64::INFINITY, f64::min);
        let max = errors.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        
        Some(Self {
            rmse,
            mean,
            median,
            std,
            min,
            max,
        })
    }
}

impl std::fmt::Display for AbsoluteTrajectoryError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "ATE: RMSE={:.4}m, Mean={:.4}m, Median={:.4}m, Std={:.4}m, Min={:.4}m, Max={:.4}m",
            self.rmse, self.mean, self.median, self.std, self.min, self.max
        )
    }
}

/// Relative Pose Error (RPE)
/// 
/// Measures local accuracy of the trajectory (drift over fixed distances).
/// Evaluates pose differences between pairs of frames at fixed intervals.
#[derive(Debug, Clone, Copy)]
pub struct RelativePoseError {
    /// Translation RMSE (meters)
    pub trans_rmse: f64,
    /// Translation mean (meters)
    pub trans_mean: f64,
    /// Rotation RMSE (degrees)
    pub rot_rmse: f64,
    /// Rotation mean (degrees)
    pub rot_mean: f64,
}

impl RelativePoseError {
    /// Calculate RPE for pose differences at fixed intervals
    /// 
    /// # Arguments
    /// * `estimated` - Estimated poses (position + orientation)
    /// * `ground_truth` - Ground truth poses
    /// * `delta` - Frame interval for pose differences (e.g., 1 for consecutive frames)
    pub fn calculate(
        estimated: &[(na::Vector3<f64>, na::UnitQuaternion<f64>)],
        ground_truth: &[(na::Vector3<f64>, na::UnitQuaternion<f64>)],
        delta: usize,
    ) -> Option<Self> {
        if estimated.len() < delta + 1 || estimated.len() != ground_truth.len() {
            return None;
        }
        
        let mut trans_errors = Vec::new();
        let mut rot_errors = Vec::new();
        
        for i in 0..(estimated.len() - delta) {
            let j = i + delta;
            
            // Estimated relative pose
            let est_delta_pos = estimated[j].0 - estimated[i].0;
            let est_delta_rot = estimated[i].1.inverse() * estimated[j].1;
            
            // Ground truth relative pose
            let gt_delta_pos = ground_truth[j].0 - ground_truth[i].0;
            let gt_delta_rot = ground_truth[i].1.inverse() * ground_truth[j].1;
            
            // Translation error
            let trans_error = (est_delta_pos - gt_delta_pos).norm();
            trans_errors.push(trans_error);
            
            // Rotation error (angle difference in degrees)
            let rot_diff = est_delta_rot.inverse() * gt_delta_rot;
            let rot_error = rot_diff.angle().to_degrees();
            rot_errors.push(rot_error);
        }
        
        // Calculate statistics
        let trans_mean = trans_errors.iter().sum::<f64>() / trans_errors.len() as f64;
        let trans_rmse = (trans_errors.iter().map(|e| e * e).sum::<f64>() / trans_errors.len() as f64).sqrt();
        
        let rot_mean = rot_errors.iter().sum::<f64>() / rot_errors.len() as f64;
        let rot_rmse = (rot_errors.iter().map(|e| e * e).sum::<f64>() / rot_errors.len() as f64).sqrt();
        
        Some(Self {
            trans_rmse,
            trans_mean,
            rot_rmse,
            rot_mean,
        })
    }
}

impl std::fmt::Display for RelativePoseError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "RPE: Trans RMSE={:.4}m, Mean={:.4}m | Rot RMSE={:.4}°, Mean={:.4}°",
            self.trans_rmse, self.trans_mean, self.rot_rmse, self.rot_mean
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_ate_perfect_match() {
        let trajectory = vec![
            na::Vector3::new(0.0, 0.0, 0.0),
            na::Vector3::new(1.0, 0.0, 0.0),
            na::Vector3::new(2.0, 0.0, 0.0),
        ];
        
        let ate = AbsoluteTrajectoryError::calculate(&trajectory, &trajectory).unwrap();
        
        assert!(ate.rmse < 1e-10);
        assert!(ate.mean < 1e-10);
        assert!(ate.median < 1e-10);
    }
    
    #[test]
    fn test_ate_with_error() {
        let estimated = vec![
            na::Vector3::new(0.0, 0.0, 0.0),
            na::Vector3::new(1.0, 0.1, 0.0),  // 0.1m error
            na::Vector3::new(2.0, 0.2, 0.0),  // 0.2m error
        ];
        
        let ground_truth = vec![
            na::Vector3::new(0.0, 0.0, 0.0),
            na::Vector3::new(1.0, 0.0, 0.0),
            na::Vector3::new(2.0, 0.0, 0.0),
        ];
        
        let ate = AbsoluteTrajectoryError::calculate(&estimated, &ground_truth).unwrap();
        
        // Error should be approximately sqrt((0² + 0.1² + 0.2²) / 3) ≈ 0.129
        assert!((ate.rmse - 0.129).abs() < 0.01);
        assert!((ate.mean - 0.1).abs() < 0.01);
    }
    
    #[test]
    fn test_rpe_perfect_match() {
        let poses = vec![
            (na::Vector3::new(0.0, 0.0, 0.0), na::UnitQuaternion::identity()),
            (na::Vector3::new(1.0, 0.0, 0.0), na::UnitQuaternion::identity()),
            (na::Vector3::new(2.0, 0.0, 0.0), na::UnitQuaternion::identity()),
        ];
        
        let rpe = RelativePoseError::calculate(&poses, &poses, 1).unwrap();
        
        assert!(rpe.trans_rmse < 1e-10);
        assert!(rpe.rot_rmse < 1e-10);
    }
    
    #[test]
    fn test_rpe_with_drift() {
        let estimated = vec![
            (na::Vector3::new(0.0, 0.0, 0.0), na::UnitQuaternion::identity()),
            (na::Vector3::new(1.0, 0.05, 0.0), na::UnitQuaternion::identity()),
            (na::Vector3::new(2.0, 0.10, 0.0), na::UnitQuaternion::identity()),
        ];
        
        let ground_truth = vec![
            (na::Vector3::new(0.0, 0.0, 0.0), na::UnitQuaternion::identity()),
            (na::Vector3::new(1.0, 0.0, 0.0), na::UnitQuaternion::identity()),
            (na::Vector3::new(2.0, 0.0, 0.0), na::UnitQuaternion::identity()),
        ];
        
        let rpe = RelativePoseError::calculate(&estimated, &ground_truth, 1).unwrap();
        
        // Drift should be visible in translation error
        assert!(rpe.trans_rmse > 0.04);
        assert!(rpe.trans_mean > 0.04);
    }
}
