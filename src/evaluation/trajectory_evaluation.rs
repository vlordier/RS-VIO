//! Ground truth trajectory evaluation for VIO accuracy assessment.
//!
//! Loads TUM VI ground truth trajectories and provides trajectory
//! evaluation metrics (ATE, RPE) for comparing VIO vs SLAM.
use crate::types::{Matrix4x4, Vector3};
use nalgebra as na;
use rayon::prelude::*;
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
    pub quaternion: na::UnitQuaternion<f64>,
}

impl GroundTruthPose {
    /// Convert to SE(3) matrix (world to body)
    pub fn to_matrix(&self) -> Matrix4x4 {
        na::Isometry3::from_parts(na::Translation3::from(self.position), self.quaternion)
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
            let tx: f64 = parts[1]
                .parse()
                .map_err(|_| format!("Invalid tx: {}", parts[1]))?;
            let ty: f64 = parts[2]
                .parse()
                .map_err(|_| format!("Invalid ty: {}", parts[2]))?;
            let tz: f64 = parts[3]
                .parse()
                .map_err(|_| format!("Invalid tz: {}", parts[3]))?;

            // Parse quaternion
            let qx: f64 = parts[4]
                .parse()
                .map_err(|_| format!("Invalid qx: {}", parts[4]))?;
            let qy: f64 = parts[5]
                .parse()
                .map_err(|_| format!("Invalid qy: {}", parts[5]))?;
            let qz: f64 = parts[6]
                .parse()
                .map_err(|_| format!("Invalid qz: {}", parts[6]))?;
            let qw: f64 = parts[7]
                .parse()
                .map_err(|_| format!("Invalid qw: {}", parts[7]))?;

            let pose = GroundTruthPose {
                timestamp_ns,
                position: Vector3::new(tx, ty, tz),
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
            },
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
            },
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
        log::info!("Trajectory Evaluation: {}", self.algorithm);
        log::info!("  ATE (Absolute Trajectory Error)");
        log::info!("    RMSE:   {:.6} m", self.ate_rmse);
        log::info!("    Mean:   {:.6} m", self.ate_mean);
        log::info!("    Median: {:.6} m", self.ate_median);
        log::info!("    Min:    {:.6} m", self.ate_min);
        log::info!("    Max:    {:.6} m", self.ate_max);
        log::info!("    Std:    {:.6} m", self.ate_std);
        log::info!("  RPE (Relative Pose Error)");
        log::info!("    Translation: {:.6} m", self.rpe_translation_rmse);
        log::info!(
            "    Rotation:    {:.4} deg",
            self.rpe_rotation_rmse.to_degrees()
        );
        log::info!("  Statistics");
        log::info!("    Poses:  {}", self.num_poses);
        log::info!("    Failed: {}", self.num_failed_matches);
    }
}

/// Calculate absolute trajectory error (ATE)
pub fn calculate_ate(
    ground_truth: &GroundTruthTrajectory,
    estimated: &EstimatedTrajectory,
) -> TrajectoryEvaluation {
    // Collect estimated poses into a vector for parallel iteration
    let estimated_poses: Vec<_> = estimated.poses().collect();

    let (mut errors, failed_matches) = estimated_poses
        .par_iter()
        .fold(
            || (Vec::new(), 0),
            |(mut errs, mut fails), (ts, pose)| {
                // 50ms tolerance aligns with typical TUM-VI timestamp jitter.
                if let Some(gt_pose) = ground_truth.get_closest_pose(**ts, 50_000_000) {
                    // Extract position from both poses
                    let est_pos = Vector3::new(pose.m14, pose.m24, pose.m34);
                    let gt_pos = gt_pose.position;

                    // Calculate error
                    let error = ((est_pos.x - gt_pos.x).powi(2)
                        + (est_pos.y - gt_pos.y).powi(2)
                        + (est_pos.z - gt_pos.z).powi(2))
                    .sqrt();

                    errs.push(error);
                } else {
                    fails += 1;
                }
                (errs, fails)
            },
        )
        .reduce(
            || (Vec::new(), 0),
            |(mut errs1, fails1), (errs2, fails2)| {
                errs1.extend(errs2);
                (errs1, fails1 + fails2)
            },
        );

    // Calculate statistics
    let (rmse, mean, median, min, max, std) = if !errors.is_empty() {
        let n = errors.len() as f64;
        let sum: f64 = errors.par_iter().sum();
        let mean = sum / n;
        let rmse = (errors.par_iter().map(|e| e * e).sum::<f64>() / n).sqrt();

        errors.sort_by(|a, b| a.partial_cmp(b).unwrap_or(Ordering::Equal));
        let sorted = &errors;
        let median = if sorted.len() % 2 == 0 {
            (sorted[sorted.len() / 2 - 1] + sorted[sorted.len() / 2]) / 2.0
        } else {
            sorted[sorted.len() / 2]
        };

        let min = sorted.first().copied().unwrap_or(0.0);
        let max = sorted.last().copied().unwrap_or(0.0);

        let variance: f64 = errors.par_iter().map(|e| (e - mean).powi(2)).sum::<f64>() / n;
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
    let estimated_poses: Vec<_> = estimated.poses().collect();

    let (translation_errors, rotation_errors): (Vec<f64>, Vec<f64>) = estimated_poses
        .par_windows(2)
        .filter_map(|window| {
            let (ts1, pose1) = window[0];
            let (ts2, pose2) = window[1];

            if *ts2 - *ts1 < delta_time_ns / 2 || *ts2 - *ts1 > delta_time_ns * 3 / 2 {
                return None; // Skip non-matching time deltas
            }

            // Get ground truth for both poses
            let gt1 = ground_truth.get_closest_pose(*ts1, 50_000_000)?;
            let gt2 = ground_truth.get_closest_pose(*ts2, 50_000_000)?;

            // Calculate relative poses
            let gt_relative = {
                let m1 = gt1.to_matrix();
                let m2 = gt2.to_matrix();
                let m1_inv = m1.try_inverse()?;
                m2 * m1_inv
            };

            let pose1_inv = pose1.try_inverse()?;
            let est_relative = *pose2 * pose1_inv;

            // Extract translation error
            let trans_error = ((est_relative.m14 - gt_relative.m14).powi(2)
                + (est_relative.m24 - gt_relative.m24).powi(2)
                + (est_relative.m34 - gt_relative.m34).powi(2))
            .sqrt();

            let gt_relative_inv = gt_relative.try_inverse()?;
            let rot_error_tf = est_relative * gt_relative_inv;
            let trace_rel = rot_error_tf.m11 + rot_error_tf.m22 + rot_error_tf.m33;
            let cos_angle = ((trace_rel - 1.0) / 2.0).clamp(-1.0, 1.0);
            let angle_error = cos_angle.acos();

            Some((trans_error, angle_error))
        })
        .unzip();

    let trans_rmse = if !translation_errors.is_empty() {
        let n = translation_errors.len() as f64;
        (translation_errors
            .par_iter()
            .map(|e: &f64| e * e)
            .sum::<f64>()
            / n)
            .sqrt()
    } else {
        0.0
    };

    let rot_rmse = if !rotation_errors.is_empty() {
        let n = rotation_errors.len() as f64;
        (rotation_errors.par_iter().map(|e: &f64| e * e).sum::<f64>() / n).sqrt()
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

        // Identical trajectories should have zero ATE
        let eval = super::calculate_ate(&gt, &est);
        assert!(
            eval.ate_rmse < 1e-9,
            "Identical trajectories should have zero ATE, got {}",
            eval.ate_rmse
        );
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

        // Constant 1.0m offset in X should produce ATE RMSE of 1.0
        let eval = super::calculate_ate(&gt, &est);
        assert!(
            (eval.ate_rmse - 1.0).abs() < 0.01,
            "ATE RMSE should be ~1.0, got {}",
            eval.ate_rmse
        );
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

        // Constant 0.5m offset should have zero RPE (relative motion is correct)
        let (trans_rmse, _rot_rmse) = super::calculate_rpe(&gt, &est, 1000);
        assert!(
            trans_rmse < 1e-9,
            "Constant offset should give zero RPE, got {}",
            trans_rmse
        );
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

#[cfg(test)]
mod benchmarks {
    use super::*;
    use std::time::Instant;

    #[test]
    #[ignore] // Ad-hoc benchmark — run with `cargo test -- --ignored`
    fn benchmark_ate_calculation() {
        let num_poses = 20_000;
        let mut gt = GroundTruthTrajectory::new("bench_gt");
        let mut est = EstimatedTrajectory::new("bench_est");

        for i in 0..num_poses {
            let ts = i as i64 * 10_000_000;
            let t = i as f64 * 0.01;

            // Ground Truth
            let gt_pose = GroundTruthPose {
                timestamp_ns: ts,
                position: Vector3::new(t, t.sin(), t.cos()),
                quaternion: na::UnitQuaternion::identity(),
            };
            gt.poses.insert(ts, gt_pose);

            // Estimated (with some error)
            let mut matrix = Matrix4x4::identity();
            matrix[(0, 3)] = t + 0.1;
            matrix[(1, 3)] = t.sin() + 0.05;
            matrix[(2, 3)] = t.cos() - 0.05;
            est.add_pose(ts, matrix);
        }

        let start = Instant::now();
        let _result = calculate_ate(&gt, &est);
        let duration = start.elapsed();

        println!("calculate_ate ({} poses): {:?}", num_poses, duration);
    }

    #[test]
    #[ignore] // Ad-hoc benchmark — run with `cargo test -- --ignored`
    fn benchmark_rpe_calculation() {
        let num_poses = 50_000; // Increase to 50k
        let mut gt = GroundTruthTrajectory::new("bench_gt");
        let mut est = EstimatedTrajectory::new("bench_est");

        for i in 0..num_poses {
            let ts = i as i64 * 10_000_000;
            let t = i as f64 * 0.01;

            let gt_pose = GroundTruthPose {
                timestamp_ns: ts,
                position: Vector3::new(t, 0.0, 0.0),
                quaternion: na::UnitQuaternion::from_euler_angles(0.0, 0.0, 0.01 * t),
            };
            gt.poses.insert(ts, gt_pose);

            let rot = na::UnitQuaternion::from_euler_angles(0.0, 0.0, 0.01 * t + 0.01);
            let trans = na::Translation3::new(t + 0.1, 0.0, 0.0);
            let iso = na::Isometry3::from_parts(trans, rot);
            est.add_pose(ts, iso.to_homogeneous());
        }

        let delta = 10_000_000; // 10ms (matching generation step)
        let start = Instant::now();
        let _result = calculate_rpe(&gt, &est, delta);
        let duration = start.elapsed();

        println!("calculate_rpe ({} poses): {:?}", num_poses, duration);
    }
}
