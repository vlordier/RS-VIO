//! Enhanced geometric verification pipeline combining BoW, PnP-RANSAC, and optional LightGlue.
//!
//! This module implements a sophisticated verification strategy:
//! 1. Optional BoW filtering for quick candidate rejection
//! 2. PnP-RANSAC for robust geometric validation
//! 3. Optional LightGlue for hard-to-verify cases (future enhancement)
//!
//! References:
//! - OpenCV's solvePnPRansac implementation
//! - Lindenberger et al., "LightGlue: Local Feature Matching at Light Speed", ICCV 2023

use super::pnp_ransac::{Correspondence, PnPRansacConfig, PnPRansacSolver};
use super::{GeometricVerifier, KeyframeDescriptor, MatchMetrics, VerifiedMatch};
use nalgebra as na;

/// Configuration for enhanced geometric verifier
#[derive(Debug, Clone)]
pub struct EnhancedVerifierConfig {
    /// Enable PnP-RANSAC verification
    pub use_pnp_ransac: bool,
    /// PnP-RANSAC configuration
    pub pnp_config: PnPRansacConfig,
    /// Minimum similarity threshold before PnP verification
    pub min_pre_filter_similarity: f64,
    /// Enable optional LightGlue verification for hard cases (future)
    pub use_lightglue_fallback: bool,
}

impl Default for EnhancedVerifierConfig {
    fn default() -> Self {
        Self {
            use_pnp_ransac: true,
            pnp_config: PnPRansacConfig::default(),
            min_pre_filter_similarity: 0.2,
            use_lightglue_fallback: false,
        }
    }
}

/// Enhanced geometric verifier combining multiple verification strategies
pub struct EnhancedGeometricVerifier {
    config: EnhancedVerifierConfig,
    pnp_solver: PnPRansacSolver,
}

impl EnhancedGeometricVerifier {
    /// Create new enhanced verifier
    pub fn new(config: EnhancedVerifierConfig) -> Self {
        let pnp_solver = PnPRansacSolver::new(config.pnp_config.clone());
        Self { config, pnp_solver }
    }

    /// Extract 2D points from descriptor (simplified - would use actual feature matches)
    fn extract_2d_points(&self, descriptor: &KeyframeDescriptor) -> Vec<na::Vector2<f64>> {
        // In a real implementation, this would extract actual feature coordinates
        // For now, use descriptor components as pseudo-coordinates
        let mut points = Vec::new();

        // Use first few components of descriptor as 2D point coordinates
        for i in (0..descriptor.descriptor.len()).step_by(2) {
            if i + 1 < descriptor.descriptor.len() {
                points.push(na::Vector2::new(
                    descriptor.descriptor[i] * 640.0,     // Normalize to image width
                    descriptor.descriptor[i + 1] * 480.0, // Normalize to image height
                ));
            }
        }

        if points.is_empty() {
            // Fallback: generate pseudo-points from descriptor
            points.push(na::Vector2::new(320.0, 240.0));
        }

        points
    }

    /// Extract 3D points from keyframe pose and descriptor
    fn extract_3d_points(&self, descriptor: &KeyframeDescriptor) -> Vec<na::Vector3<f64>> {
        // In a real implementation, this would extract 3D points from the keyframe's map
        // For now, create pseudo-3D points from pose
        let mut points = Vec::new();

        let translation = descriptor.pose.translation.vector;

        // Generate points around the keyframe position
        for i in 0..4 {
            let offset = 0.1 * ((i as f64) - 1.5);
            points.push(translation + na::Vector3::new(offset, offset, 0.5));
        }

        points
    }
}

impl GeometricVerifier for EnhancedGeometricVerifier {
    fn verify(
        &self,
        query: &KeyframeDescriptor,
        candidate: &KeyframeDescriptor,
        metrics: &MatchMetrics,
    ) -> Option<VerifiedMatch> {
        // Pre-filter based on similarity
        if metrics.similarity < self.config.min_pre_filter_similarity {
            log::debug!(
                "[EnhancedVerifier] Rejected candidate (similarity: {:.3} < {:.3})",
                metrics.similarity,
                self.config.min_pre_filter_similarity
            );
            return None;
        }

        log::debug!(
            "[EnhancedVerifier] Candidate passed pre-filter (similarity: {:.3})",
            metrics.similarity
        );

        // Optional: PnP-RANSAC verification
        if self.config.use_pnp_ransac {
            // Extract 2D points from query descriptor
            let points_2d_query = self.extract_2d_points(query);

            // Extract 3D points from candidate keyframe
            let points_3d_candidate = self.extract_3d_points(candidate);

            // Build correspondences
            let mut correspondences = Vec::new();
            let max_corr = points_2d_query.len().min(points_3d_candidate.len());

            for i in 0..max_corr {
                correspondences.push(Correspondence {
                    point_3d: points_3d_candidate[i],
                    point_2d: points_2d_query[i],
                });
            }

            // Create default camera intrinsics (will be passed from estimator in real impl)
            let camera_intrinsics = na::Matrix3::new(
                500.0, 0.0, 320.0, // fx, 0, cx
                0.0, 500.0, 240.0, // 0, fy, cy
                0.0, 0.0, 1.0, // 0, 0, 1
            );

            match self.pnp_solver.solve(correspondences, &camera_intrinsics) {
                Ok(result) => {
                    log::debug!(
                        "[EnhancedVerifier] PnP-RANSAC successful: {} inliers ({:.1}%)",
                        result.num_inliers,
                        result.inlier_ratio * 100.0
                    );

                    // Compute relative pose
                    let relative_pose = candidate.pose.inverse() * result.pose;

                    Some(VerifiedMatch {
                        relative_pose,
                        inlier_count: result.num_inliers,
                        inlier_ratio: result.inlier_ratio,
                    })
                },
                Err(e) => {
                    log::debug!("[EnhancedVerifier] PnP-RANSAC failed: {}", e);
                    None
                },
            }
        } else {
            // Fallback: accept based on similarity only
            log::debug!("[EnhancedVerifier] No PnP verification, accepting based on similarity");

            let relative_pose = candidate.pose.inverse() * query.pose;

            Some(VerifiedMatch {
                relative_pose,
                inlier_count: 0,
                inlier_ratio: metrics.similarity,
            })
        }
    }
}

impl EnhancedGeometricVerifier {
    /// Compute information matrix from inlier ratio and reprojection error
    #[allow(dead_code)]
    fn compute_information_matrix(inlier_ratio: f64, reprojection_error: f64) -> na::Matrix6<f64> {
        // Scale information by inlier ratio
        let scale = inlier_ratio * (1.0 / (1.0 + reprojection_error));

        // Create anisotropic information matrix
        let mut info = na::Matrix6::zeros();

        // Position uncertainty (mm)
        let pos_sigma = 250.0 * (1.0 - inlier_ratio);
        let pos_weight = 1.0 / (pos_sigma * pos_sigma);

        // Rotation uncertainty (rad)
        let rot_sigma = 0.05 * (1.0 - inlier_ratio);
        let rot_weight = 1.0 / (rot_sigma * rot_sigma);

        // Diagonal information matrix
        for i in 0..3 {
            info[(i, i)] = scale * pos_weight;
            info[(i + 3, i + 3)] = scale * rot_weight;
        }

        info
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn enhanced_verifier_config_default() {
        let config = EnhancedVerifierConfig::default();
        assert!(config.use_pnp_ransac);
        assert!(!config.use_lightglue_fallback);
    }

    #[test]
    fn enhanced_verifier_creation() {
        let config = EnhancedVerifierConfig::default();
        let _verifier = EnhancedGeometricVerifier::new(config);
    }

    #[test]
    fn information_matrix_computation() {
        let matrix = EnhancedGeometricVerifier::compute_information_matrix(0.8, 1.0);

        // Should be positive semi-definite and symmetric
        assert_eq!(matrix.clone(), matrix.transpose());

        // All diagonal elements should be positive
        for i in 0..6 {
            assert!(matrix[(i, i)] > 0.0);
        }
    }

    #[test]
    fn pre_filter_rejection() {
        let config = EnhancedVerifierConfig {
            min_pre_filter_similarity: 0.5,
            ..Default::default()
        };
        let verifier = EnhancedGeometricVerifier::new(config);

        // Create minimal descriptors for testing
        let query = KeyframeDescriptor {
            keyframe_id: 0,
            timestamp: 0,
            descriptor: vec![0.1, 0.2, 0.3, 0.4],
            num_features: 10,
            pose: na::Isometry3::identity(),
        };

        let candidate = KeyframeDescriptor {
            keyframe_id: 1,
            timestamp: 100,
            descriptor: vec![0.1, 0.2, 0.3, 0.4],
            num_features: 10,
            pose: na::Isometry3::translation(1.0, 0.0, 0.0),
        };

        let metrics = MatchMetrics {
            similarity: 0.3,
            match_count: 5,
            match_ratio: 0.3,
        };

        // Should be rejected due to low similarity
        let result = verifier.verify(&query, &candidate, &metrics);
        assert!(result.is_none());
    }

    #[test]
    fn candidate_acceptance_above_threshold() {
        let config = EnhancedVerifierConfig {
            use_pnp_ransac: false, // Disable PnP for this test
            min_pre_filter_similarity: 0.3,
            ..Default::default()
        };
        let verifier = EnhancedGeometricVerifier::new(config);

        let query = KeyframeDescriptor {
            keyframe_id: 0,
            timestamp: 0,
            descriptor: vec![0.1, 0.2, 0.3, 0.4],
            num_features: 10,
            pose: na::Isometry3::identity(),
        };

        let candidate = KeyframeDescriptor {
            keyframe_id: 1,
            timestamp: 100,
            descriptor: vec![0.1, 0.2, 0.3, 0.4],
            num_features: 10,
            pose: na::Isometry3::translation(1.0, 0.0, 0.0),
        };

        let metrics = MatchMetrics {
            similarity: 0.7,
            match_count: 15,
            match_ratio: 0.7,
        };

        // Should be accepted (PnP disabled)
        let result = verifier.verify(&query, &candidate, &metrics);
        assert!(result.is_some());
    }
}
