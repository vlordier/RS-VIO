//! Stereo camera calibrator implementation

use crate::calibration::config::CalibrationConfig;
use crate::calibration::factors::{StereoReprojectionFactor, EpipolarFactor};
use apex_solver::core::problem::Problem;
use apex_solver::core::loss_functions::HuberLoss;
use apex_solver::manifold::ManifoldType;
use apex_solver::optimizer::levenberg_marquardt::{LevenbergMarquardt, LevenbergMarquardtConfig};
use nalgebra as na;

/// Status of calibration process
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CalibrationStatus {
    /// Calibration not started
    NotStarted,
    /// Collecting stereo pairs
    CollectingData,
    /// Running optimization
    Optimizing,
    /// Calibration completed successfully
    Success,
    /// Calibration failed
    Failed,
}

/// Result of stereo calibration
#[derive(Debug, Clone)]
pub struct CalibrationResult {
    /// Calibrated left camera intrinsics [fx, fy, cx, cy, k1, k2, p1, p2, k3]
    pub left_intrinsics: Vec<f64>,
    /// Calibrated right camera intrinsics [fx, fy, cx, cy, k1, k2, p1, p2, k3]
    pub right_intrinsics: Vec<f64>,
    /// Calibrated stereo extrinsics (right camera pose relative to left)
    pub stereo_extrinsics: na::Isometry3<f64>,
    /// Final reprojection error (pixels)
    pub final_reprojection_error: f64,
    /// Number of stereo pairs used
    pub num_stereo_pairs: usize,
    /// Number of feature matches used
    pub num_feature_matches: usize,
}

/// Stereo pair data for calibration
#[derive(Debug, Clone)]
pub struct StereoPair {
    /// Left image features (pixel coordinates)
    pub left_features: Vec<na::Vector2<f64>>,
    /// Right image features (pixel coordinates)
    pub right_features: Vec<na::Vector2<f64>>,
    /// Feature correspondences (indices into left/right feature vectors)
    pub correspondences: Vec<(usize, usize)>,
}

/// Stereo camera auto-calibrator
pub struct StereoCalibrator {
    config: CalibrationConfig,
    stereo_pairs: Vec<StereoPair>,
    status: CalibrationStatus,
}

impl StereoCalibrator {
    /// Create a new stereo calibrator with given configuration
    pub fn new(config: CalibrationConfig) -> Self {
        Self {
            config,
            stereo_pairs: Vec::new(),
            status: CalibrationStatus::NotStarted,
        }
    }

    /// Add a stereo pair for calibration
    ///
    /// Features should be detected and matched between the left and right images.
    /// Correspondences should be established between feature indices.
    pub fn add_stereo_pair(&mut self, stereo_pair: StereoPair) {
        if self.stereo_pairs.len() < self.config.max_stereo_pairs {
            // Validate that we have enough matches
            if stereo_pair.correspondences.len() >= self.config.min_feature_matches {
                self.stereo_pairs.push(stereo_pair);
                self.status = CalibrationStatus::CollectingData;
            } else {
                log::warn!(
                    "Skipping stereo pair with {} matches (minimum required: {})",
                    stereo_pair.correspondences.len(),
                    self.config.min_feature_matches
                );
            }
        } else {
            log::warn!("Maximum number of stereo pairs ({}) reached", self.config.max_stereo_pairs);
        }
    }

    /// Run the stereo calibration optimization
    pub fn calibrate(&mut self) -> Result<CalibrationResult, Box<dyn std::error::Error>> {
        if self.stereo_pairs.is_empty() {
            return Err("No stereo pairs available for calibration".into());
        }

        self.status = CalibrationStatus::Optimizing;
        log::info!("Starting stereo calibration with {} stereo pairs", self.stereo_pairs.len());

        // Initialize parameters
        let initial_params = self.initialize_parameters();

        // Create optimization problem and initial values
        let mut problem = Problem::new();
        let mut initial_values = std::collections::HashMap::new();

        // Add variables
        let left_intrinsics_var = "left_intrinsics".to_string();
        let right_intrinsics_var = "right_intrinsics".to_string();
        let extrinsics_var = "stereo_extrinsics".to_string();

        initial_values.insert(
            left_intrinsics_var.clone(),
            (ManifoldType::RN, na::DVector::from_vec(initial_params.left_intrinsics.clone())),
        );
        initial_values.insert(
            right_intrinsics_var.clone(),
            (ManifoldType::RN, na::DVector::from_vec(initial_params.right_intrinsics.clone())),
        );
        initial_values.insert(
            extrinsics_var.clone(),
            (ManifoldType::RN, na::DVector::from_vec(initial_params.extrinsics.clone())),
        );

        // Add 3D point variables for each correspondence
        let mut point_vars = Vec::new();
        for (pair_idx, stereo_pair) in self.stereo_pairs.iter().enumerate() {
            for (corr_idx, &(left_idx, right_idx)) in stereo_pair.correspondences.iter().enumerate() {
                let point_var = format!("point_{}_{}", pair_idx, corr_idx);
                let initial_point = self.triangulate_initial_point(
                    &stereo_pair.left_features[left_idx],
                    &stereo_pair.right_features[right_idx],
                    &initial_params,
                );
                initial_values.insert(
                    point_var.clone(),
                    (ManifoldType::RN, na::DVector::from_vec(initial_point)),
                );
                point_vars.push(point_var);
            }
        }

        // Add factors
        let mut point_var_iter = point_vars.iter();
        for stereo_pair in &self.stereo_pairs {
            for &(left_idx, right_idx) in &stereo_pair.correspondences {
                let left_obs = stereo_pair.left_features[left_idx];
                let right_obs = stereo_pair.right_features[right_idx];

                let point_var = point_var_iter.next().unwrap();

                // Add reprojection factor
                let factor = StereoReprojectionFactor::new(left_obs, right_obs);
                let huber_loss = HuberLoss::new(self.config.huber_delta).ok();
                problem.add_residual_block(
                    &[&left_intrinsics_var, &right_intrinsics_var, &extrinsics_var, point_var],
                    Box::new(factor),
                    huber_loss.map(|l| Box::new(l) as Box<dyn apex_solver::core::loss_functions::LossFunction + Send>),
                );

                // Optionally add epipolar constraint
                let epipolar_factor = EpipolarFactor::new(left_obs, right_obs);
                problem.add_residual_block(
                    &[&left_intrinsics_var, &right_intrinsics_var, &extrinsics_var],
                    Box::new(epipolar_factor),
                    None,
                );
            }
        }

        // Initialize variables in the problem
        problem.initialize_variables(&initial_values);

        // Configure optimizer
        let optimizer_config = LevenbergMarquardtConfig {
            max_iterations: self.config.max_iterations,
            parameter_tolerance: self.config.parameter_tolerance,
            cost_tolerance: self.config.cost_tolerance,
            ..Default::default()
        };

        let mut optimizer = LevenbergMarquardt::with_config(optimizer_config);

        // Run optimization
        let opt_result = optimizer.optimize(&problem, &initial_values)?;

        // Check if optimization was successful
        let is_successful = matches!(
            &opt_result.status,
            apex_solver::optimizer::OptimizationStatus::Converged
                | apex_solver::optimizer::OptimizationStatus::CostToleranceReached
                | apex_solver::optimizer::OptimizationStatus::ParameterToleranceReached
                | apex_solver::optimizer::OptimizationStatus::GradientToleranceReached
                | apex_solver::optimizer::OptimizationStatus::TrustRegionRadiusTooSmall
                | apex_solver::optimizer::OptimizationStatus::MinCostThresholdReached
                | apex_solver::optimizer::OptimizationStatus::MaxIterationsReached
        );

        if !is_successful {
            return Err(format!("Calibration optimization failed with status: {:?}", opt_result.status).into());
        }

        // Extract results
        let final_left_intrinsics = opt_result.parameters.get(&left_intrinsics_var)
            .ok_or("Missing left intrinsics in result")?
            .to_vector();
        let final_right_intrinsics = opt_result.parameters.get(&right_intrinsics_var)
            .ok_or("Missing right intrinsics in result")?
            .to_vector();
        let final_extrinsics = opt_result.parameters.get(&extrinsics_var)
            .ok_or("Missing extrinsics in result")?
            .to_vector();

        // Convert extrinsics back to SE(3)
        let final_extrinsics_se3 = Self::vector_to_isometry(final_extrinsics.as_slice());

        // Compute final reprojection error
        let final_error = self.compute_reprojection_error(
            final_left_intrinsics.as_slice(),
            final_right_intrinsics.as_slice(),
            &final_extrinsics_se3
        );

        let calibration_result = CalibrationResult {
            left_intrinsics: final_left_intrinsics.as_slice().to_vec(),
            right_intrinsics: final_right_intrinsics.as_slice().to_vec(),
            stereo_extrinsics: final_extrinsics_se3,
            final_reprojection_error: final_error,
            num_stereo_pairs: self.stereo_pairs.len(),
            num_feature_matches: self.stereo_pairs.iter().map(|p| p.correspondences.len()).sum(),
        };

        self.status = CalibrationStatus::Success;
        log::info!("Stereo calibration completed successfully!");
        log::info!("Final reprojection error: {:.2} pixels", final_error);
        log::info!("Used {} stereo pairs with {} feature matches",
                  calibration_result.num_stereo_pairs, calibration_result.num_feature_matches);

        Ok(calibration_result)
    }

    /// Get current calibration status
    pub fn status(&self) -> CalibrationStatus {
        self.status
    }

    /// Get number of collected stereo pairs
    pub fn num_stereo_pairs(&self) -> usize {
        self.stereo_pairs.len()
    }

    /// Initialize optimization parameters
    fn initialize_parameters(&self) -> InitialParameters {
        // Use config defaults or estimate from data
        let left_intrinsics = vec![
            self.config.initial_focal_length,  // fx
            self.config.initial_focal_length,  // fy
            self.config.initial_principal_point.0,  // cx
            self.config.initial_principal_point.1,  // cy
            0.0, 0.0, 0.0, 0.0, 0.0,  // distortion (k1, k2, p1, p2, k3)
        ];

        let right_intrinsics = left_intrinsics.clone();

        // Initialize extrinsics with small baseline
        let extrinsics = vec![0.0, 0.0, 0.0, 0.1, 0.0, 0.0];  // rx, ry, rz, tx, ty, tz

        InitialParameters {
            left_intrinsics,
            right_intrinsics,
            extrinsics,
        }
    }

    /// Triangulate initial 3D point for optimization
    fn triangulate_initial_point(
        &self,
        left_point: &na::Vector2<f64>,
        right_point: &na::Vector2<f64>,
        params: &InitialParameters,
    ) -> Vec<f64> {
        // Simple triangulation assuming known intrinsics and small baseline
        // This is a rough initialization - optimization will refine it

        let fx = params.left_intrinsics[0];
        let fy = params.left_intrinsics[1];
        let cx = params.left_intrinsics[2];
        let cy = params.left_intrinsics[3];

        let baseline = params.extrinsics[3];  // tx

        // Convert to normalized coordinates
        let xl = (left_point.x - cx) / fx;
        let yl = (left_point.y - cy) / fy;
        let xr = (right_point.x - cx) / fx;
        let _yr = (right_point.y - cy) / fy;

        // Disparity
        let disparity = xl - xr;
        if disparity.abs() < 1e-6 {
            // Points too close, use default depth
            return vec![0.0, 0.0, 1.0];
        }

        // Triangulate
        let z = baseline / disparity;
        let x = xl * z;
        let y = yl * z;

        vec![x, y, z]
    }

    /// Convert parameter vector to SE(3) isometry
    fn vector_to_isometry(params: &[f64]) -> na::Isometry3<f64> {
        let rx = params[0];
        let ry = params[1];
        let rz = params[2];
        let tx = params[3];
        let ty = params[4];
        let tz = params[5];

        let rotation = na::UnitQuaternion::from_euler_angles(rx, ry, rz);
        let translation = na::Vector3::new(tx, ty, tz);

        na::Isometry3::from_parts(translation.into(), rotation)
    }

    /// Compute final reprojection error
    fn compute_reprojection_error(
        &self,
        left_intrinsics: &[f64],
        right_intrinsics: &[f64],
        extrinsics: &na::Isometry3<f64>,
    ) -> f64 {
        let mut total_error = 0.0;
        let mut total_points = 0;

        for stereo_pair in &self.stereo_pairs {
            for &(left_idx, right_idx) in &stereo_pair.correspondences {
                let left_obs = stereo_pair.left_features[left_idx];
                let right_obs = stereo_pair.right_features[right_idx];

                // Triangulate point
                let point_3d = self.triangulate_point(&left_obs, &right_obs, left_intrinsics, extrinsics);

                // Project back to cameras
                let left_proj = self.project_point(&point_3d, left_intrinsics);
                let right_proj = self.project_point(&(extrinsics.inverse() * point_3d), right_intrinsics);

                // Compute errors
                let left_error = (left_proj - left_obs).norm();
                let right_error = (right_proj - right_obs).norm();

                total_error += left_error + right_error;
                total_points += 2;
            }
        }

        if total_points > 0 {
            total_error / total_points as f64
        } else {
            0.0
        }
    }

    /// Triangulate 3D point from stereo observations
    fn triangulate_point(
        &self,
        left_point: &na::Vector2<f64>,
        right_point: &na::Vector2<f64>,
        left_intrinsics: &[f64],
        extrinsics: &na::Isometry3<f64>,
    ) -> na::Vector3<f64> {
        // Simplified triangulation - in practice, you'd use proper stereo triangulation
        let fx = left_intrinsics[0];
        let fy = left_intrinsics[1];
        let cx = left_intrinsics[2];
        let cy = left_intrinsics[3];

        let baseline = extrinsics.translation.x;  // Assume horizontal baseline

        let xl = (left_point.x - cx) / fx;
        let yl = (left_point.y - cy) / fy;
        let xr = (right_point.x - cx) / fx;
        let _yr = (right_point.y - cy) / fy;

        let disparity = xl - xr;
        if disparity.abs() < 1e-6 {
            return na::Vector3::new(0.0, 0.0, 1.0);
        }

        let z = baseline / disparity;
        let x = xl * z;
        let y = yl * z;

        na::Vector3::new(x, y, z)
    }

    /// Project 3D point to camera
    fn project_point(&self, point: &na::Vector3<f64>, intrinsics: &[f64]) -> na::Vector2<f64> {
        let fx = intrinsics[0];
        let fy = intrinsics[1];
        let cx = intrinsics[2];
        let cy = intrinsics[3];

        let u = fx * point.x / point.z + cx;
        let v = fy * point.y / point.z + cy;

        na::Vector2::new(u, v)
    }
}

/// Helper struct for initial parameter estimation
struct InitialParameters {
    left_intrinsics: Vec<f64>,
    right_intrinsics: Vec<f64>,
    extrinsics: Vec<f64>,
}