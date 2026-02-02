/// Phase 3A: Ground Truth Evaluation Module
///
/// This module loads TUM VI ground truth trajectories and provides
/// trajectory evaluation metrics (ATE, RPE) for comparing VIO vs SLAM.
use crate::types::{Matrix4x4, Vector3};
use nalgebra as na;
use std::cmp::Ordering;
use std::collections::BTreeMap;
use std::fs::File;
use std::io::{BufRead, BufReader};
use std::path::Path;

/// Ground truth pose from TUM VI dataset
#[derive(Debug, Clone)]
pub struct GroundTruthPose {
    /// Timestamp in nanoseconds
    pub timestamp_ns: i64,
    /// Position in world frame (meters)
    pub position: Vector3,
    /// Orientation as unit quaternion (x, y, z, w)
    pub quaternion: na::UnitQuaternion<f32>,
}

impl GroundTruthPose {
    /// Convert to SE(3) matrix (world to body)
    pub fn to_matrix(&self) -> Matrix4x4 {
        na::Isometry3::from_parts(
            na::Translation3::from(self.position),
            self.quaternion.cast::<f64>(),
        )
        .to_homogeneous()
    }
}

/// Ground truth trajectory container
#[derive(Debug)]
pub struct GroundTruthTrajectory {
    /// Poses indexed by timestamp for quick lookup
    poses: BTreeMap<i64, GroundTruthPose>,
    /// Sequence name
    pub sequence_name: String,
}

impl GroundTruthTrajectory {
    /// Create empty trajectory
    pub fn new(sequence_name: impl Into<String>) -> Self {
        Self {
            poses: BTreeMap::new(),
            sequence_name: sequence_name.into(),
        }
    }

    /// Load from TUM VI groundtruth.txt file
    ///
    /// Expected format (space-separated):
    /// timestamp tx ty tz qx qy qz qw
    pub fn from_file(path: impl AsRef<Path>) -> Result<Self, String> {
        let file = File::open(&path).map_err(|e| format!("Failed to open file: {}", e))?;
        let reader = BufReader::new(file);
        let mut poses = BTreeMap::new();
        let mut count = 0;

        for line in reader.lines() {
            let line = line.map_err(|e| format!("Read error: {}", e))?;

            // Skip comments
            if line.starts_with('#') {
                continue;
            }

            let parts: Vec<&str> = line.split_whitespace().collect();
            if parts.len() < 8 {
                continue; // Skip malformed lines
            }

            // Parse timestamp (seconds, convert to nanoseconds)
            let timestamp_s: f64 = parts[0]
                .parse()
                .map_err(|_| format!("Invalid timestamp: {}", parts[0]))?;
            let timestamp_ns = (timestamp_s * 1e9) as i64;

            // Parse position
            let tx: f32 = parts[1]
                .parse()
                .map_err(|_| format!("Invalid tx: {}", parts[1]))?;
            let ty: f32 = parts[2]
                .parse()
                .map_err(|_| format!("Invalid ty: {}", parts[2]))?;
            let tz: f32 = parts[3]
                .parse()
                .map_err(|_| format!("Invalid tz: {}", parts[3]))?;

            // Parse quaternion
            let qx: f32 = parts[4]
                .parse()
                .map_err(|_| format!("Invalid qx: {}", parts[4]))?;
            let qy: f32 = parts[5]
                .parse()
                .map_err(|_| format!("Invalid qy: {}", parts[5]))?;
            let qz: f32 = parts[6]
                .parse()
                .map_err(|_| format!("Invalid qz: {}", parts[6]))?;
            let qw: f32 = parts[7]
                .parse()
                .map_err(|_| format!("Invalid qw: {}", parts[7]))?;

            let pose = GroundTruthPose {
                timestamp_ns,
                position: Vector3::new(tx as f64, ty as f64, tz as f64),
                quaternion: na::UnitQuaternion::new_normalize(na::Quaternion::new(qw, qx, qy, qz)),
            };

            poses.insert(timestamp_ns, pose);
            count += 1;
        }

        if poses.is_empty() {
            return Err("No valid poses found in file".to_string());
        }

        log::info!("[GroundTruth] Loaded {} poses from file", count);

        Ok(Self {
            poses,
            sequence_name: path
                .as_ref()
                .file_stem()
                .and_then(|s| s.to_str())
                .unwrap_or("dataset")
                .to_string(),
        })
    }

    /// Get pose at exact timestamp
    pub fn get_pose(&self, timestamp_ns: i64) -> Option<GroundTruthPose> {
        self.poses.get(&timestamp_ns).cloned()
    }

    /// Get closest pose to timestamp (within max_delta_ns)
    pub fn get_closest_pose(
        &self,
        timestamp_ns: i64,
        max_delta_ns: i64,
    ) -> Option<GroundTruthPose> {
        // Try to find exact match first
        if let Some(pose) = self.poses.get(&timestamp_ns) {
            return Some(pose.clone());
        }

        let before = self.poses.range(..=timestamp_ns).next_back();
        let after = self.poses.range(timestamp_ns..).next();
        let closest = match (before, after) {
            (Some((ts_b, pose_b)), Some((ts_a, pose_a))) => {
                if timestamp_ns - ts_b <= ts_a - timestamp_ns {
                    Some((ts_b, pose_b))
                } else {
                    Some((ts_a, pose_a))
                }
            }
            (Some(p), None) => Some(p),
            (None, Some(p)) => Some(p),
            (None, None) => None,
        };
        closest
            .filter(|(ts, _)| (timestamp_ns - *ts).abs() <= max_delta_ns)
            .map(|(_, pose)| pose.clone())
    }

    /// Get all poses
    pub fn poses(&self) -> impl Iterator<Item = &GroundTruthPose> {
        self.poses.values()
    }

    /// Get number of poses
    pub fn len(&self) -> usize {
        self.poses.len()
    }

    /// Check if empty
    pub fn is_empty(&self) -> bool {
        self.poses.is_empty()
    }

    /// Get time span (first to last timestamp)
    pub fn time_span(&self) -> Option<(i64, i64)> {
        if self.poses.is_empty() {
            return None;
        }
        let first = *self.poses.keys().next()?;
        let last = *self.poses.keys().last()?;
        Some((first, last))
    }
}

/// Estimated trajectory from VIO or SLAM
#[derive(Debug)]
pub struct EstimatedTrajectory {
    poses: BTreeMap<i64, Matrix4x4>,
    pub algorithm_name: String,
}

impl EstimatedTrajectory {
    /// Create empty trajectory
    pub fn new(algorithm_name: impl Into<String>) -> Self {
        Self {
            poses: BTreeMap::new(),
            algorithm_name: algorithm_name.into(),
        }
    }

    /// Add pose to trajectory
    pub fn add_pose(&mut self, timestamp_ns: i64, pose: Matrix4x4) {
        self.poses.insert(timestamp_ns, pose);
    }

    /// Get pose at exact timestamp
    pub fn get_pose(&self, timestamp_ns: i64) -> Option<&Matrix4x4> {
        self.poses.get(&timestamp_ns)
    }

    /// Get closest pose to timestamp
    pub fn get_closest_pose(&self, timestamp_ns: i64, max_delta_ns: i64) -> Option<&Matrix4x4> {
        if let Some(pose) = self.poses.get(&timestamp_ns) {
            return Some(pose);
        }
        let before = self.poses.range(..timestamp_ns).next_back();
        let after = self.poses.range(timestamp_ns..).next();
        let closest_key = match (before, after) {
            (Some((ts_b, _)), Some((ts_a, _))) => {
                if timestamp_ns - *ts_b < *ts_a - timestamp_ns {
                    Some(ts_b)
                } else {
                    Some(ts_a)
                }
            }
            (Some((ts, _)), None) => Some(ts),
            (None, Some((ts, _))) => Some(ts),
            (None, None) => None,
        };
        if let Some(key) = closest_key {
            if (timestamp_ns - *key).abs() <= max_delta_ns {
                return self.poses.get(key);
            }
        }
        None
    }

    /// Get all poses
    pub fn poses(&self) -> impl Iterator<Item = (&i64, &Matrix4x4)> {
        self.poses.iter()
    }

    /// Get number of poses
    pub fn len(&self) -> usize {
        self.poses.len()
    }

    /// Check if empty
    pub fn is_empty(&self) -> bool {
        self.poses.is_empty()
    }
}

/// Trajectory evaluation result
#[derive(Debug, Clone)]
pub struct TrajectoryEvaluation {
    /// Algorithm name (VIO or SLAM)
    pub algorithm: String,
    /// Absolute Trajectory Error (meters)
    pub ate_rmse: f64,
    pub ate_mean: f64,
    pub ate_median: f64,
    pub ate_min: f64,
    pub ate_max: f64,
    pub ate_std: f64,
    /// Relative Pose Error
    pub rpe_translation_rmse: f64,
    pub rpe_rotation_rmse: f64,
    /// Number of poses compared
    pub num_poses: usize,
    /// Poses with missing ground truth
    pub num_failed_matches: usize,
}

impl TrajectoryEvaluation {
    /// Print formatted evaluation results
    pub fn print_summary(&self) {
        println!("\n📊 Trajectory Evaluation: {}", self.algorithm);
        println!("  ├─ ATE (Absolute Trajectory Error)");
        println!("  │  ├─ RMSE:  {:.6} m", self.ate_rmse);
        println!("  │  ├─ Mean:  {:.6} m", self.ate_mean);
        println!("  │  ├─ Median: {:.6} m", self.ate_median);
        println!("  │  ├─ Min:   {:.6} m", self.ate_min);
        println!("  │  ├─ Max:   {:.6} m", self.ate_max);
        println!("  │  └─ Std:   {:.6} m", self.ate_std);
        println!("  ├─ RPE (Relative Pose Error)");
        println!("  │  ├─ Translation: {:.6} m", self.rpe_translation_rmse);
        println!(
            "  │  └─ Rotation:    {:.4}°",
            self.rpe_rotation_rmse.to_degrees()
        );
        println!("  └─ Statistics");
        println!("     ├─ Poses:      {}", self.num_poses);
        println!("     └─ Failed:     {}", self.num_failed_matches);
    }
}

/// Calculate absolute trajectory error (ATE)
pub fn calculate_ate(
    ground_truth: &GroundTruthTrajectory,
    estimated: &EstimatedTrajectory,
) -> TrajectoryEvaluation {
    let mut errors = Vec::new();
    let mut failed_matches = 0;

    for (ts, pose) in estimated.poses() {
        // 50ms tolerance aligns with typical TUM-VI timestamp jitter.
        let gt_pose = match ground_truth.get_closest_pose(*ts, 50_000_000) {
            Some(p) => p,
            None => {
                failed_matches += 1;
                continue;
            },
        };

        // Extract position from both poses
        let est_pos = Vector3::new(pose.m14, pose.m24, pose.m34);
        let gt_pos = gt_pose.position;

        // Calculate error
        let error = ((est_pos.x - gt_pos.x).powi(2)
            + (est_pos.y - gt_pos.y).powi(2)
            + (est_pos.z - gt_pos.z).powi(2))
        .sqrt();

        errors.push(error);
    }

    // Calculate statistics
    let (rmse, mean, median, min, max, std) = if !errors.is_empty() {
        let n = errors.len() as f64;
        let sum: f64 = errors.iter().sum();
        let mean = sum / n;
        let rmse = (errors.iter().map(|e| e * e).sum::<f64>() / n).sqrt();

        let mut sorted = errors.clone();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(Ordering::Equal));
        let median = if sorted.len() % 2 == 0 {
            (sorted[sorted.len() / 2 - 1] + sorted[sorted.len() / 2]) / 2.0
        } else {
            sorted[sorted.len() / 2]
        };

        let min = sorted.first().copied().unwrap_or(0.0);
        let max = sorted.last().copied().unwrap_or(0.0);

        let variance: f64 = errors.iter().map(|e| (e - mean).powi(2)).sum::<f64>() / n;
        let std = variance.sqrt();

        (rmse, mean, median, min, max, std)
    } else {
        (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    };

    TrajectoryEvaluation {
        algorithm: estimated.algorithm_name.clone(),
        ate_rmse: rmse,
        ate_mean: mean,
        ate_median: median,
        ate_min: min,
        ate_max: max,
        ate_std: std,
        rpe_translation_rmse: 0.0, // TODO: Implement RPE
        rpe_rotation_rmse: 0.0,
        num_poses: errors.len(),
        num_failed_matches: failed_matches,
    }
}

/// Calculate relative pose error (RPE)
pub fn calculate_rpe(
    ground_truth: &GroundTruthTrajectory,
    estimated: &EstimatedTrajectory,
    delta_time_ns: i64,
) -> (f64, f64) {
    let mut translation_errors = Vec::new();
    let mut rotation_errors = Vec::new();

    let mut prev: Option<(&i64, &Matrix4x4)> = None;
    for (ts2, pose2) in estimated.poses() {
        let next = Some((ts2, pose2));
        let (ts1, pose1) = match prev {
            Some(p) => p,
            None => {
                prev = next;
                continue;
            }
        };

        if ts2 - ts1 < delta_time_ns / 2 || ts2 - ts1 > delta_time_ns * 3 / 2 {
            prev = next;
            continue; // Skip non-matching time deltas
        }

        // Get ground truth for both poses
        let gt1 = match ground_truth.get_closest_pose(*ts1, 50_000_000) {
            Some(p) => p,
            None => {
                prev = next;
                continue;
            }
        };
        let gt2 = match ground_truth.get_closest_pose(*ts2, 50_000_000) {
            Some(p) => p,
            None => {
                prev = next;
                continue;
            }
        };

        // Calculate relative poses
        let gt_relative = {
            let m1 = gt1.to_matrix();
            let m2 = gt2.to_matrix();
            let Some(m1_inv) = m1.try_inverse() else {
                prev = next;
                continue;
            };
            m2 * m1_inv
        };

        let Some(pose1_inv) = pose1.try_inverse() else {
            prev = next;
            continue;
        };
        let est_relative = *pose2 * pose1_inv;

        // Extract translation error
        let trans_error = ((est_relative.m14 - gt_relative.m14).powi(2)
            + (est_relative.m24 - gt_relative.m24).powi(2)
            + (est_relative.m34 - gt_relative.m34).powi(2))
        .sqrt();
        translation_errors.push(trans_error);

        let Some(gt_relative_inv) = gt_relative.try_inverse() else {
            prev = next;
            continue;
        };
        let rot_error_tf = est_relative * gt_relative_inv;
        let trace_rel = rot_error_tf.m11 + rot_error_tf.m22 + rot_error_tf.m33;
        let cos_angle = ((trace_rel - 1.0) / 2.0).clamp(-1.0, 1.0);
        let angle_error = cos_angle.acos();
        rotation_errors.push(angle_error);

        prev = next;
    }

    let trans_rmse = if !translation_errors.is_empty() {
        let n = translation_errors.len() as f64;
        (translation_errors.iter().map(|e| e * e).sum::<f64>() / n).sqrt()
    } else {
        0.0
    };

    let rot_rmse = if !rotation_errors.is_empty() {
        let n = rotation_errors.len() as f64;
        (rotation_errors.iter().map(|e| e * e).sum::<f64>() / n).sqrt()
    } else {
        0.0
    };

    (trans_rmse, rot_rmse)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ground_truth_pose_conversion() {
        let pose = GroundTruthPose {
            timestamp_ns: 1000,
            position: Vector3::new(1.0, 2.0, 3.0),
            quaternion: na::UnitQuaternion::identity(),
        };

        let matrix = pose.to_matrix();
        assert!((matrix.m14 - 1.0).abs() < 1e-5);
        assert!((matrix.m24 - 2.0).abs() < 1e-5);
        assert!((matrix.m34 - 3.0).abs() < 1e-5);
    }

    #[test]
    fn test_ground_truth_trajectory_operations() {
        let mut traj = GroundTruthTrajectory::new("test");

        let pose1 = GroundTruthPose {
            timestamp_ns: 1000,
            position: Vector3::new(0.0, 0.0, 0.0),
            quaternion: na::UnitQuaternion::identity(),
        };

        let pose2 = GroundTruthPose {
            timestamp_ns: 2000,
            position: Vector3::new(1.0, 0.0, 0.0),
            quaternion: na::UnitQuaternion::identity(),
        };

        traj.poses.insert(1000, pose1);
        traj.poses.insert(2000, pose2);

        assert_eq!(traj.len(), 2);
        assert!(traj.get_pose(1000).is_some());
        assert!(traj.get_pose(2000).is_some());
    }

    #[test]
    fn test_estimated_trajectory() {
        let mut est = EstimatedTrajectory::new("VIO");

        let pose = Matrix4x4::identity();
        est.add_pose(1000, pose);

        assert_eq!(est.len(), 1);
        assert!(est.get_pose(1000).is_some());
    }

    #[test]
    fn test_ate_identical_trajectories() {
        // Create two identical trajectories
        let mut gt = GroundTruthTrajectory::new("test");
        let mut est = EstimatedTrajectory::new("test");

        for i in 0..5 {
            let ts = (i * 1000) as i64;
            let pos_val = i as f64;

            let pose = GroundTruthPose {
                timestamp_ns: ts,
                position: Vector3::new(pos_val, pos_val, pos_val),
                quaternion: na::UnitQuaternion::identity(),
            };
            gt.poses.insert(ts, pose);

            let mut matrix = Matrix4x4::identity();
            matrix[(0, 3)] = pos_val;
            matrix[(1, 3)] = pos_val;
            matrix[(2, 3)] = pos_val;
            est.add_pose(ts, matrix);
        }

        // Both trajectories should have same poses
        assert_eq!(gt.len(), est.len());
        assert_eq!(gt.len(), 5);
    }

    #[test]
    fn test_ate_offset_trajectory() {
        // Create trajectories with constant offset
        let mut gt = GroundTruthTrajectory::new("test");
        let mut est = EstimatedTrajectory::new("test");

        let offset = 1.0;

        for i in 0..5 {
            let ts = (i * 1000) as i64;
            let pos_val = i as f64;

            let pose = GroundTruthPose {
                timestamp_ns: ts,
                position: Vector3::new(pos_val, 0.0, 0.0),
                quaternion: na::UnitQuaternion::identity(),
            };
            gt.poses.insert(ts, pose);

            let mut matrix = Matrix4x4::identity();
            matrix[(0, 3)] = pos_val + offset;
            est.add_pose(ts, matrix);
        }

        // Both trajectories should have same number of poses
        assert_eq!(gt.len(), est.len());
        assert_eq!(est.len(), 5);
    }

    #[test]
    fn test_rpe_translation_error() {
        // Create trajectory with pure translation error
        let mut gt = GroundTruthTrajectory::new("test");
        let mut est = EstimatedTrajectory::new("test");

        for i in 0..10 {
            let ts = (i * 1000) as i64;

            let pose = GroundTruthPose {
                timestamp_ns: ts,
                position: Vector3::new(i as f64, 0.0, 0.0),
                quaternion: na::UnitQuaternion::identity(),
            };
            gt.poses.insert(ts, pose);

            let mut matrix = Matrix4x4::identity();
            matrix[(0, 3)] = i as f64 + 0.5;
            est.add_pose(ts, matrix);
        }

        // Just verify that both trajectories have the right number of poses
        assert_eq!(gt.len(), 10);
        assert_eq!(est.len(), 10);
    }

    #[test]
    fn test_trajectory_evaluation_structure() {
        let eval = TrajectoryEvaluation {
            algorithm: "VIO".to_string(),
            ate_rmse: 0.1,
            ate_mean: 0.08,
            ate_median: 0.075,
            ate_min: 0.01,
            ate_max: 0.15,
            ate_std: 0.03,
            rpe_translation_rmse: 0.05,
            rpe_rotation_rmse: 0.02,
            num_poses: 100,
            num_failed_matches: 0,
        };

        assert!((eval.ate_rmse - 0.1).abs() < 1e-6);
        assert!((eval.rpe_translation_rmse - 0.05).abs() < 1e-6);
        assert_eq!(eval.num_poses, 100);
    }

    #[test]
    fn test_ground_truth_trajectory_empty() {
        let traj = GroundTruthTrajectory::new("empty");
        assert_eq!(traj.len(), 0);
        assert!(traj.is_empty());
    }

    #[test]
    fn test_estimated_trajectory_multiple_poses() {
        let mut est = EstimatedTrajectory::new("multi");

        for i in 0..10 {
            let ts = (i * 1000) as i64;
            let mut matrix = Matrix4x4::identity();
            matrix[(0, 3)] = i as f64;
            est.add_pose(ts, matrix);
        }

        assert_eq!(est.len(), 10);

        // Verify random pose
        if let Some(pose) = est.get_pose(5000) {
            assert!((pose.m14 - 5.0).abs() < 1e-6);
        } else {
            panic!("Failed to get pose at timestamp 5000");
        }
    }

    #[test]
    fn test_calculate_ate_zero_error() {
        let mut gt = GroundTruthTrajectory::new("gt");
        let mut est = EstimatedTrajectory::new("est");

        for i in 0..3 {
            let ts = (i * 1_000_000) as i64;
            let pose = GroundTruthPose {
                timestamp_ns: ts,
                position: Vector3::new(i as f64, 0.0, 0.0),
                quaternion: na::UnitQuaternion::identity(),
            };
            gt.poses.insert(ts, pose);

            let mut matrix = Matrix4x4::identity();
            matrix[(0, 3)] = i as f64;
            est.add_pose(ts, matrix);
        }

        let eval = calculate_ate(&gt, &est);
        assert_eq!(eval.num_poses, 3);
        assert_eq!(eval.num_failed_matches, 0);
        assert!(eval.ate_rmse.abs() < 1e-9);
        assert!(eval.ate_mean.abs() < 1e-9);
    }

    #[test]
    fn test_calculate_ate_missing_gt() {
        let mut gt = GroundTruthTrajectory::new("gt");
        let mut est = EstimatedTrajectory::new("est");

        let ts_ok = 1_000_000i64;
        let pose = GroundTruthPose {
            timestamp_ns: ts_ok,
            position: Vector3::new(0.0, 0.0, 0.0),
            quaternion: na::UnitQuaternion::identity(),
        };
        gt.poses.insert(ts_ok, pose);

        let mut matrix = Matrix4x4::identity();
        est.add_pose(ts_ok, matrix);

        // Add an estimated pose far outside the 50ms tolerance window
        matrix[(0, 3)] = 10.0;
        est.add_pose(ts_ok + 1_000_000_000, matrix);

        let eval = calculate_ate(&gt, &est);
        assert_eq!(eval.num_poses, 1);
        assert_eq!(eval.num_failed_matches, 1);
    }

    #[test]
    fn test_calculate_rpe_constant_offset_is_zero() {
        let mut gt = GroundTruthTrajectory::new("gt");
        let mut est = EstimatedTrajectory::new("est");

        for i in 0..4 {
            let ts = (i * 1_000_000) as i64;
            let pose = GroundTruthPose {
                timestamp_ns: ts,
                position: Vector3::new(i as f64, 0.0, 0.0),
                quaternion: na::UnitQuaternion::identity(),
            };
            gt.poses.insert(ts, pose);

            let mut matrix = Matrix4x4::identity();
            matrix[(0, 3)] = i as f64 + 5.0; // constant offset
            est.add_pose(ts, matrix);
        }

        let (trans_rmse, rot_rmse) = calculate_rpe(&gt, &est, 1_000_000);
        assert!(trans_rmse.abs() < 1e-9);
        assert!(rot_rmse.abs() < 1e-9);
    }

    #[test]
    fn test_calculate_rpe_delta_filter() {
        let mut gt = GroundTruthTrajectory::new("gt");
        let mut est = EstimatedTrajectory::new("est");

        for i in 0..3 {
            let ts = (i * 1_000_000) as i64;
            let pose = GroundTruthPose {
                timestamp_ns: ts,
                position: Vector3::new(i as f64, 0.0, 0.0),
                quaternion: na::UnitQuaternion::identity(),
            };
            gt.poses.insert(ts, pose);

            let mut matrix = Matrix4x4::identity();
            matrix[(0, 3)] = i as f64;
            est.add_pose(ts, matrix);
        }

        // Use a delta that won't match the 1_000_000 ns steps
        let (trans_rmse, rot_rmse) = calculate_rpe(&gt, &est, 10_000_000);
        assert!(trans_rmse.abs() < 1e-12);
        assert!(rot_rmse.abs() < 1e-12);
    }
}
