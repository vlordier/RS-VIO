//! Tests for stereo camera calibration module

use apex_solver::factors::Factor;
use nalgebra as na;
use rs_vio::calibration::{
    config::CalibrationConfig,
    factors::{EpipolarFactor, StereoReprojectionFactor},
    stereo_calibrator::{StereoCalibrator, StereoPair},
};

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_stereo_reprojection_factor_creation() {
        let left_obs = na::Vector2::new(100.0, 200.0);
        let right_obs = na::Vector2::new(120.0, 205.0);

        let factor = StereoReprojectionFactor::new(left_obs, right_obs);

        assert_eq!(factor.left_observation, left_obs);
        assert_eq!(factor.right_observation, right_obs);
        assert_eq!(factor.get_dimension(), 4);
    }

    #[test]
    fn test_epipolar_factor_creation() {
        let left_pt = na::Vector2::new(150.0, 180.0);
        let right_pt = na::Vector2::new(170.0, 185.0);

        let factor = EpipolarFactor::new(left_pt, right_pt);

        assert_eq!(factor.left_point, left_pt);
        assert_eq!(factor.right_point, right_pt);
        assert_eq!(factor.get_dimension(), 1);
    }

    #[test]
    fn test_stereo_calibrator_creation() {
        let config = CalibrationConfig::default();
        let calibrator = StereoCalibrator::new(config);

        assert_eq!(
            calibrator.status(),
            rs_vio::calibration::stereo_calibrator::CalibrationStatus::NotStarted
        );
        assert_eq!(calibrator.num_stereo_pairs(), 0);
    }

    #[test]
    fn test_calibration_config_defaults() {
        let config = CalibrationConfig::default();

        assert_eq!(config.max_stereo_pairs, 50);
        assert_eq!(config.min_feature_matches, 20);
        assert!((config.max_reprojection_error - 2.0).abs() < f64::EPSILON);
        assert!((config.huber_delta - 1.0).abs() < f64::EPSILON);
        assert_eq!(config.max_iterations, 100);
        assert!(config.optimize_distortion);
        assert!(config.optimize_principal_point);
    }

    #[test]
    fn test_stereo_pair_creation() {
        let left_features = vec![
            na::Vector2::new(100.0, 200.0),
            na::Vector2::new(150.0, 250.0),
            na::Vector2::new(200.0, 180.0),
        ];
        let right_features = vec![
            na::Vector2::new(120.0, 205.0),
            na::Vector2::new(170.0, 255.0),
            na::Vector2::new(220.0, 185.0),
        ];
        let correspondences = vec![(0, 0), (1, 1), (2, 2)];

        let stereo_pair = StereoPair {
            left_features,
            right_features,
            correspondences,
            timestamp: 1.0,
            angular_velocity: Some(na::Vector3::new(0.1, 0.0, 0.0)),
            linear_velocity: Some(na::Vector3::new(0.0, 0.0, 0.0)),
            feature_qualities: vec![0.9, 0.8, 0.7],
        };

        assert_eq!(stereo_pair.left_features.len(), 3);
        assert_eq!(stereo_pair.right_features.len(), 3);
        assert_eq!(stereo_pair.correspondences.len(), 3);
    }

    #[test]
    fn test_factor_linearize_dimensions() {
        // Test that linearize returns correct dimensions
        let left_obs = na::Vector2::new(100.0, 200.0);
        let right_obs = na::Vector2::new(120.0, 205.0);

        let factor = StereoReprojectionFactor::new(left_obs, right_obs);

        // Create dummy parameters (9 left intrinsics, 9 right intrinsics, 6 extrinsics, 3 point)
        let params = vec![
            na::DVector::from_vec(vec![500.0, 500.0, 320.0, 240.0, 0.0, 0.0, 0.0, 0.0, 0.0]), // left intrinsics
            na::DVector::from_vec(vec![500.0, 500.0, 320.0, 240.0, 0.0, 0.0, 0.0, 0.0, 0.0]), // right intrinsics
            na::DVector::from_vec(vec![0.0, 0.0, 0.0, 0.1, 0.0, 0.0]), // extrinsics
            na::DVector::from_vec(vec![0.0, 0.0, 1.0]),                // 3D point
        ];

        let (residual, jacobian): (na::DVector<f64>, Option<na::DMatrix<f64>>) =
            factor.linearize(&params, true);

        // Should return 4D residual (left_u, left_v, right_u, right_v)
        assert_eq!(residual.len(), 4);

        // Jacobian should exist when requested
        assert!(jacobian.is_some());

        // Jacobian should have 4 rows (residual dimension) and 27 columns (9+9+6+3 parameters)
        if let Some(jac) = jacobian {
            assert_eq!(jac.nrows(), 4);
            assert_eq!(jac.ncols(), 27);
        }
    }

    #[test]
    fn test_epipolar_factor_linearize() {
        let left_pt = na::Vector2::new(100.0, 200.0);
        let right_pt = na::Vector2::new(120.0, 205.0);

        let factor = EpipolarFactor::new(left_pt, right_pt);

        // Create dummy parameters (9 left intrinsics, 9 right intrinsics, 6 extrinsics)
        let params = vec![
            na::DVector::from_vec(vec![500.0, 500.0, 320.0, 240.0, 0.0, 0.0, 0.0, 0.0, 0.0]), // left intrinsics
            na::DVector::from_vec(vec![500.0, 500.0, 320.0, 240.0, 0.0, 0.0, 0.0, 0.0, 0.0]), // right intrinsics
            na::DVector::from_vec(vec![0.0, 0.0, 0.0, 0.1, 0.0, 0.0]), // extrinsics
        ];

        let (residual, jacobian): (na::DVector<f64>, Option<na::DMatrix<f64>>) =
            factor.linearize(&params, true);

        // Should return 1D residual (epipolar error)
        assert_eq!(residual.len(), 1);

        // Jacobian should exist when requested
        assert!(jacobian.is_some());

        // Jacobian should have 1 row and 24 columns (9+9+6 parameters)
        if let Some(jac) = jacobian {
            assert_eq!(jac.nrows(), 1);
            assert_eq!(jac.ncols(), 24);
        }
    }
}
