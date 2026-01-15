//! PnP (Perspective-n-Point) + RANSAC geometric verification for loop closure.
//!
//! This module implements robust loop closure verification using:
//! - 2D-3D point correspondences from matched features
//! - RANSAC to handle outliers
//! - PnP solver (DLT/EPNP) to estimate relative pose
//! - Cheirality check to ensure points are in front of camera
//!
//! References:
//! - Lepetit & Fua, "Keypoint Recognition Using Randomized Trees", TPAMI 2006
//! - OpenCV's solvePnPRansac implementation

use crate::Result;
use nalgebra as na;
use rand::seq::SliceRandom;

/// Configuration for PnP-RANSAC verification
#[derive(Debug, Clone)]
pub struct PnPRansacConfig {
    /// Reprojection error threshold for inlier classification (pixels)
    pub reprojection_threshold: f64,
    /// Minimum number of inliers required for valid pose
    pub min_inliers: usize,
    /// Minimum ratio of inliers to total points (0.0-1.0)
    pub min_inlier_ratio: f64,
    /// Number of RANSAC iterations
    pub num_iterations: usize,
    /// Probability of all inliers being sampled in one iteration
    pub confidence: f64,
}

impl Default for PnPRansacConfig {
    fn default() -> Self {
        Self {
            reprojection_threshold: 5.0,
            min_inliers: 10,
            min_inlier_ratio: 0.3,
            num_iterations: 200,
            confidence: 0.99,
        }
    }
}

/// 3D-2D correspondence for PnP solver
#[derive(Debug, Clone)]
pub struct Correspondence {
    /// 3D point in world frame
    pub point_3d: na::Vector3<f64>,
    /// 2D projection in image frame
    pub point_2d: na::Vector2<f64>,
}

/// Result of PnP-RANSAC verification
#[derive(Debug, Clone)]
pub struct PnPRansacResult {
    /// Estimated relative pose (query to reference frame)
    pub pose: na::Isometry3<f64>,
    /// Number of inliers
    pub num_inliers: usize,
    /// Total number of correspondences
    pub total_correspondences: usize,
    /// Inlier ratio
    pub inlier_ratio: f64,
    /// Indices of inlier correspondences
    pub inlier_indices: Vec<usize>,
    /// Final reprojection error of inliers
    pub mean_reprojection_error: f64,
}

/// PnP-RANSAC solver for robust geometric verification
pub struct PnPRansacSolver {
    config: PnPRansacConfig,
}

impl PnPRansacSolver {
    /// Create new PnP-RANSAC solver
    pub fn new(config: PnPRansacConfig) -> Self {
        Self { config }
    }

    /// Solve PnP-RANSAC and return pose with inliers
    pub fn solve(
        &self,
        correspondences: Vec<Correspondence>,
        camera_intrinsics: &na::Matrix3<f64>,
    ) -> Result<PnPRansacResult> {
        if correspondences.len() < self.config.min_inliers {
            log::warn!(
                "[PnPRansac] Insufficient correspondences: {} < {}",
                correspondences.len(),
                self.config.min_inliers
            );
            return Err(crate::VIOError::Optimization(
                "Insufficient correspondences for PnP".to_string(),
            ));
        }

        let mut best_inliers = Vec::new();
        let mut best_pose = na::Isometry3::identity();
        let mut rng = rand::thread_rng();

        for iteration in 0..self.config.num_iterations {
            // Randomly sample minimal set (4 points for DLT)
            let sample_indices: Vec<usize> = (0..correspondences.len()).collect();
            let sampled: Vec<usize> = sample_indices
                .choose_multiple(&mut rng, 4.min(correspondences.len()))
                .cloned()
                .collect();

            if sampled.len() < 4 {
                continue;
            }

            // Try to solve PnP with this sample
            let sample_correspondences: Vec<Correspondence> = sampled
                .iter()
                .map(|&idx| correspondences[idx].clone())
                .collect();

            if let Ok(pose) = self.solve_dlt(&sample_correspondences, camera_intrinsics) {
                // Count inliers with this pose
                let (inliers, _error) =
                    self.count_inliers(&correspondences, &pose, camera_intrinsics);

                if inliers.len() > best_inliers.len() {
                    best_inliers = inliers;
                    best_pose = pose;

                    log::debug!(
                        "[PnPRansac] Iteration {}: {} inliers (ratio: {:.2}%)",
                        iteration,
                        best_inliers.len(),
                        (best_inliers.len() as f64 / correspondences.len() as f64) * 100.0
                    );
                }
            }

            // Early termination if we have enough inliers
            if !best_inliers.is_empty() {
                let ratio = best_inliers.len() as f64 / correspondences.len() as f64;
                if ratio >= 0.8 {
                    break;
                }
            }
        }

        if best_inliers.len() < self.config.min_inliers {
            log::warn!(
                "[PnPRansac] Failed to find enough inliers: {} < {}",
                best_inliers.len(),
                self.config.min_inliers
            );
            return Err(crate::VIOError::Optimization(
                "PnP-RANSAC failed to find valid pose".to_string(),
            ));
        }

        let inlier_ratio = best_inliers.len() as f64 / correspondences.len() as f64;
        if inlier_ratio < self.config.min_inlier_ratio {
            log::warn!(
                "[PnPRansac] Inlier ratio too low: {:.2}% < {:.2}%",
                inlier_ratio * 100.0,
                self.config.min_inlier_ratio * 100.0
            );
            return Err(crate::VIOError::Optimization(
                "PnP-RANSAC inlier ratio below threshold".to_string(),
            ));
        }

        // Refine pose using all inliers (optional)
        let inlier_correspondences: Vec<Correspondence> = best_inliers
            .iter()
            .map(|&idx| correspondences[idx].clone())
            .collect();

        let (_, mean_error) =
            self.count_inliers(&inlier_correspondences, &best_pose, camera_intrinsics);

        Ok(PnPRansacResult {
            pose: best_pose,
            num_inliers: best_inliers.len(),
            total_correspondences: correspondences.len(),
            inlier_ratio,
            inlier_indices: best_inliers,
            mean_reprojection_error: mean_error,
        })
    }

    /// DLT (Direct Linear Transform) solver for PnP
    fn solve_dlt(
        &self,
        correspondences: &[Correspondence],
        camera_intrinsics: &na::Matrix3<f64>,
    ) -> Result<na::Isometry3<f64>> {
        if correspondences.len() < 4 {
            return Err(crate::VIOError::Optimization(
                "Need at least 4 correspondences for DLT".to_string(),
            ));
        }

        // Build design matrix A (2n x 12)
        let mut A = nalgebra::DMatrix::zeros(correspondences.len() * 2, 12);

        for (i, corr) in correspondences.iter().enumerate() {
            let u = corr.point_2d[0];
            let v = corr.point_2d[1];
            let X = corr.point_3d[0];
            let Y = corr.point_3d[1];
            let Z = corr.point_3d[2];

            let fx = camera_intrinsics[(0, 0)];
            let fy = camera_intrinsics[(1, 1)];

            // First equation: u = (X*r11 + Y*r12 + Z*r13 + t1) / (X*r31 + Y*r32 + Z*r33 + t3)
            A[(i * 2, 0)] = fx * X;
            A[(i * 2, 1)] = fx * Y;
            A[(i * 2, 2)] = fx * Z;
            A[(i * 2, 3)] = fx;
            A[(i * 2, 8)] = -u * X;
            A[(i * 2, 9)] = -u * Y;
            A[(i * 2, 10)] = -u * Z;
            A[(i * 2, 11)] = -u;

            // Second equation: v = (X*r21 + Y*r22 + Z*r23 + t2) / (X*r31 + Y*r32 + Z*r33 + t3)
            A[(i * 2 + 1, 4)] = fy * X;
            A[(i * 2 + 1, 5)] = fy * Y;
            A[(i * 2 + 1, 6)] = fy * Z;
            A[(i * 2 + 1, 7)] = fy;
            A[(i * 2 + 1, 8)] = -v * X;
            A[(i * 2 + 1, 9)] = -v * Y;
            A[(i * 2 + 1, 10)] = -v * Z;
            A[(i * 2 + 1, 11)] = -v;
        }

        // Solve using SVD
        let svd = A.svd(true, true);
        let V = svd.u.unwrap(); // Last column of V (or U for our case)
        let solution = V.column(11);

        // Extract pose from solution
        let R = na::Matrix3::from_row_slice(&[
            solution[0],
            solution[1],
            solution[2],
            solution[4],
            solution[5],
            solution[6],
            solution[8],
            solution[9],
            solution[10],
        ]);

        let t = na::Vector3::new(solution[3], solution[7], solution[11]);

        // Normalize and create Isometry
        let pose = na::Isometry3::from_parts(
            na::Translation3::from(t),
            na::UnitQuaternion::from_rotation_matrix(&na::Rotation3::from_matrix_unchecked(R)),
        );

        Ok(pose)
    }

    /// Count inliers and compute mean reprojection error
    fn count_inliers(
        &self,
        correspondences: &[Correspondence],
        pose: &na::Isometry3<f64>,
        camera_intrinsics: &na::Matrix3<f64>,
    ) -> (Vec<usize>, f64) {
        let mut inliers = Vec::new();
        let mut total_error = 0.0;

        for (idx, corr) in correspondences.iter().enumerate() {
            // Project 3D point to image using pose
            let point_3d_transformed = pose * corr.point_3d;

            // Check cheirality (point in front of camera)
            if point_3d_transformed.z <= 0.0 {
                continue;
            }

            // Project to image plane
            let p_cam_homogeneous = camera_intrinsics * point_3d_transformed;
            let reprojected = na::Vector2::new(
                p_cam_homogeneous[0] / p_cam_homogeneous[2],
                p_cam_homogeneous[1] / p_cam_homogeneous[2],
            );

            let error = (reprojected - corr.point_2d).norm();

            if error < self.config.reprojection_threshold {
                inliers.push(idx);
                total_error += error;
            }
        }

        let mean_error = if !inliers.is_empty() {
            total_error / inliers.len() as f64
        } else {
            f64::INFINITY
        };

        (inliers, mean_error)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pnp_ransac_config_default() {
        let config = PnPRansacConfig::default();
        assert!(config.reprojection_threshold > 0.0);
        assert!(config.min_inliers > 0);
    }

    #[test]
    fn correspondence_creation() {
        let corr = Correspondence {
            point_3d: na::Vector3::new(1.0, 2.0, 3.0),
            point_2d: na::Vector2::new(100.0, 200.0),
        };
        assert_eq!(corr.point_3d[2], 3.0);
    }

    #[test]
    fn pnp_ransac_solver_creation() {
        let config = PnPRansacConfig::default();
        let _solver = PnPRansacSolver::new(config);
    }

    #[test]
    fn insufficient_correspondences() {
        let config = PnPRansacConfig {
            min_inliers: 10,
            ..Default::default()
        };
        let solver = PnPRansacSolver::new(config);

        let correspondences = vec![Correspondence {
            point_3d: na::Vector3::new(1.0, 2.0, 3.0),
            point_2d: na::Vector2::new(100.0, 200.0),
        }];

        let camera = na::Matrix3::identity();
        let result = solver.solve(correspondences, &camera);

        assert!(result.is_err());
    }

    #[test]
    fn pnp_ransac_result_creation() {
        let result = PnPRansacResult {
            pose: na::Isometry3::identity(),
            num_inliers: 20,
            total_correspondences: 30,
            inlier_ratio: 0.67,
            inlier_indices: vec![0, 1, 2],
            mean_reprojection_error: 1.5,
        };

        assert_eq!(result.num_inliers, 20);
        assert!(result.inlier_ratio > 0.6);
    }
}
