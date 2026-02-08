//! Multi-camera calibration for heterogeneous camera rigs
//!
//! This module provides calibration of camera networks with different intrinsics,
//! enabling systems with 3, 4, or N cameras for improved accuracy and robustness.

use crate::calibration::camera_models::{CameraConfig, CameraModel};
use crate::calibration::factors::{CameraGraphFactor, MultiCameraReprojectionFactor};
use crate::calibration::quality::CalibrationQualityMetrics;
use apex_solver::core::problem::Problem;
use apex_solver::manifold::ManifoldType;
use apex_solver::optimizer::levenberg_marquardt::{LevenbergMarquardt, LevenbergMarquardtConfig};
use nalgebra as na;
use std::collections::{HashMap, HashSet};
use std::time::Instant;

/// Status of multi-camera calibration
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MultiCameraCalibrationStatus {
    /// Calibration not started
    NotStarted,
    /// Collecting multi-view observations
    CollectingData,
    /// Initializing camera poses
    Initializing,
    /// Running bundle adjustment
    Optimizing,
    /// Calibration completed successfully
    Success,
    /// Calibration failed
    Failed,
}

/// Multi-view observation (point seen by multiple cameras)
#[derive(Debug, Clone)]
pub struct MultiViewObservation {
    /// Camera observations of the same 3D point
    pub camera_observations: HashMap<String, na::Vector2<f64>>, // camera_id -> image_point
    /// Timestamp when observations were made
    pub timestamp: f64,
    /// Feature quality scores per camera
    pub quality_scores: HashMap<String, f64>,
}

/// Camera pose in the rig coordinate system
#[derive(Debug, Clone)]
pub struct CameraPose {
    /// Transform from camera to rig coordinates
    pub transform: na::Isometry3<f64>,
    /// Whether this pose is fixed (known) or to be optimized
    pub is_fixed: bool,
    /// Confidence in this pose estimate
    pub confidence: f64,
}

/// Multi-camera calibration result
#[derive(Debug, Clone)]
pub struct MultiCameraCalibrationResult {
    /// Calibrated camera intrinsics (camera_id -> intrinsics)
    pub camera_intrinsics: HashMap<String, Vec<f64>>,
    /// Calibrated camera poses relative to reference camera
    pub camera_poses: HashMap<String, CameraPose>,
    /// Final reprojection error (pixels)
    pub final_reprojection_error: f64,
    /// Number of multi-view observations used
    pub num_observations: usize,
    /// Number of cameras calibrated
    pub num_cameras: usize,
    /// Calibration quality metrics
    pub quality_metrics: CalibrationQualityMetrics,
}

/// Camera graph representation for multi-camera systems
#[derive(Debug, Clone)]
pub struct CameraGraph {
    /// Cameras in the graph
    pub cameras: HashMap<String, CameraConfig>,
    /// Edges between cameras (overlapping views)
    pub edges: HashMap<(String, String), CameraEdge>,
    /// Reference camera ID (coordinate system origin)
    pub reference_camera: String,
}

#[derive(Debug, Clone)]
pub struct CameraEdge {
    /// Relative transform between cameras
    pub relative_pose: na::Isometry3<f64>,
    /// Number of common observations
    pub num_observations: usize,
    /// Quality of the relative pose estimate
    pub quality_score: f64,
}

/// Multi-camera auto-calibrator
pub struct MultiCameraCalibrator {
    config: MultiCameraCalibrationConfig,
    camera_graph: CameraGraph,
    observations: Vec<MultiViewObservation>,
    status: MultiCameraCalibrationStatus,
    start_time: Option<Instant>,
}

#[derive(Debug, Clone)]
pub struct MultiCameraCalibrationConfig {
    /// Maximum number of multi-view observations to use
    pub max_observations: usize,
    /// Minimum number of cameras that must observe a point
    pub min_views_per_point: usize,
    /// Maximum reprojection error threshold (pixels)
    pub max_reprojection_error: f64,
    /// Robust loss function parameter
    pub huber_delta: f64,
    /// Maximum iterations for bundle adjustment
    pub max_iterations: usize,
    /// Convergence tolerance
    pub parameter_tolerance: f64,
    pub cost_tolerance: f64,
    /// Whether to optimize intrinsics
    pub optimize_intrinsics: bool,
    /// Whether to optimize extrinsics
    pub optimize_extrinsics: bool,
    /// Logging enabled
    pub log_calibration_metrics: bool,
}

impl Default for MultiCameraCalibrationConfig {
    fn default() -> Self {
        Self {
            max_observations: 200,
            min_views_per_point: 2,
            max_reprojection_error: 2.0,
            huber_delta: 1.0,
            max_iterations: 200,
            parameter_tolerance: 1e-6,
            cost_tolerance: 1e-6,
            optimize_intrinsics: true,
            optimize_extrinsics: true,
            log_calibration_metrics: true,
        }
    }
}

impl MultiCameraCalibrator {
    /// Create a new multi-camera calibrator
    pub const fn new(config: MultiCameraCalibrationConfig, camera_graph: CameraGraph) -> Self {
        Self {
            config,
            camera_graph,
            observations: Vec::new(),
            status: MultiCameraCalibrationStatus::NotStarted,
            start_time: None,
        }
    }

    /// Add a multi-view observation
    pub fn add_multi_view_observation(&mut self, observation: MultiViewObservation) {
        if self.observations.len() < self.config.max_observations {
            // Validate observation has minimum views
            let num_cameras = observation.camera_observations.len();
            if num_cameras >= self.config.min_views_per_point {
                self.observations.push(observation);
                self.status = MultiCameraCalibrationStatus::CollectingData;

                let progress = self.progress_percentage();
                self.log_progress(&format!(
                    "📸 Added multi-view observation {}/{} ({} cameras) - Progress: {:.1}%",
                    self.observations.len(),
                    self.config.max_observations,
                    num_cameras,
                    progress
                ));

                // Check if we have enough data
                if self.observations.len() >= 10 {
                    // Minimum for reasonable calibration
                    self.log_progress("🎯 Ready for multi-camera calibration!");
                }
            } else {
                self.log_progress(&format!(
                    "⚠️  Skipped observation with {} views (minimum required: {})",
                    num_cameras, self.config.min_views_per_point
                ));
            }
        } else {
            self.log_progress(&format!(
                "🛑 Maximum observations ({}) reached",
                self.config.max_observations
            ));
        }
    }

    /// Initialize camera poses using pairwise calibration
    fn initialize_camera_poses(
        &mut self,
    ) -> Result<HashMap<String, CameraPose>, Box<dyn std::error::Error>> {
        self.status = MultiCameraCalibrationStatus::Initializing;
        self.log_progress("🔧 Initializing camera poses from pairwise relationships...");

        let mut camera_poses = HashMap::new();

        // Start with reference camera at origin
        camera_poses.insert(
            self.camera_graph.reference_camera.clone(),
            CameraPose {
                transform: na::Isometry3::identity(),
                is_fixed: true, // Reference camera is fixed
                confidence: 1.0,
            },
        );

        // Build poses incrementally using graph edges
        let mut visited = HashSet::new();
        visited.insert(self.camera_graph.reference_camera.clone());

        let mut queue = vec![self.camera_graph.reference_camera.clone()];

        while let Some(current_camera) = queue.pop() {
            for ((cam1, cam2), edge) in &self.camera_graph.edges {
                let next_camera = if cam1 == &current_camera {
                    cam2.clone()
                } else if cam2 == &current_camera {
                    cam1.clone()
                } else {
                    continue;
                };

                if visited.contains(&next_camera) {
                    continue;
                }

                // Compute pose relative to reference camera
                let current_pose = &camera_poses[&current_camera];
                let relative_transform = if cam1 == &current_camera {
                    edge.relative_pose
                } else {
                    edge.relative_pose.inverse()
                };

                let new_pose = CameraPose {
                    transform: current_pose.transform * relative_transform,
                    is_fixed: false,
                    confidence: edge.quality_score,
                };

                camera_poses.insert(next_camera.clone(), new_pose);
                visited.insert(next_camera.clone());
                queue.push(next_camera);
            }
        }

        if camera_poses.len() != self.camera_graph.cameras.len() {
            return Err(format!(
                "Could not initialize poses for all cameras. Initialized: {}, Total: {}",
                camera_poses.len(),
                self.camera_graph.cameras.len()
            )
            .into());
        }

        self.log_progress(&format!(
            "✅ Initialized poses for {} cameras",
            camera_poses.len()
        ));

        Ok(camera_poses)
    }

    /// Run multi-camera bundle adjustment
    pub fn calibrate(
        &mut self,
    ) -> Result<MultiCameraCalibrationResult, Box<dyn std::error::Error>> {
        if self.observations.is_empty() {
            return Err("No multi-view observations available for calibration".into());
        }

        self.status = MultiCameraCalibrationStatus::Optimizing;
        self.start_time = Some(Instant::now());

        self.log_progress(&format!(
            "🚀 Starting multi-camera calibration with {} cameras and {} observations",
            self.camera_graph.cameras.len(),
            self.observations.len()
        ));

        // Initialize camera poses
        let mut camera_poses = self.initialize_camera_poses()?;

        // Initialize intrinsics
        let mut camera_intrinsics = HashMap::new();
        for (camera_id, camera_config) in &self.camera_graph.cameras {
            let intrinsics = camera_config.initial_intrinsics.clone().unwrap_or_else(|| {
                camera_config
                    .model
                    .default_intrinsics(camera_config.image_width, camera_config.image_height)
            });
            camera_intrinsics.insert(camera_id.clone(), intrinsics);
        }

        // Create optimization problem and initial values
        let mut problem = Problem::new();
        let mut initial_values = std::collections::HashMap::new();

        // Add intrinsic parameters
        for (camera_id, intrinsics) in &camera_intrinsics {
            if self.config.optimize_intrinsics {
                let var_name = format!("intrinsics_{}", camera_id);
                initial_values.insert(
                    var_name,
                    (ManifoldType::RN, na::DVector::from_vec(intrinsics.clone())),
                );
            }
        }

        // Add extrinsic parameters (poses)
        for (camera_id, pose) in &camera_poses {
            if !pose.is_fixed && self.config.optimize_extrinsics {
                // Convert pose to 6D vector (rx, ry, rz, tx, ty, tz)
                let rotation = pose.transform.rotation.scaled_axis();
                let translation = pose.transform.translation.vector;

                let pose_params = vec![
                    rotation.x,
                    rotation.y,
                    rotation.z,
                    translation.x,
                    translation.y,
                    translation.z,
                ];

                let var_name = format!("pose_{}", camera_id);
                initial_values.insert(
                    var_name,
                    (ManifoldType::RN, na::DVector::from_vec(pose_params)),
                );
            }
        }

        // Add 3D point variables (one per observation)
        let mut point_vars = Vec::new();
        for (i, _) in self.observations.iter().enumerate() {
            // Initialize 3D point (will be estimated)
            let point_var = format!("point_{}", i);
            let initial_point = vec![0.0, 0.0, 1.0]; // Unit depth
            initial_values.insert(
                point_var.clone(),
                (ManifoldType::RN, na::DVector::from_vec(initial_point)),
            );
            point_vars.push(point_var);
        }

        // Add factors for each observation
        for (obs_idx, observation) in self.observations.iter().enumerate() {
            let point_var = &point_vars[obs_idx];

            // Create factors for each camera that observed this point
            for (camera_id, image_point) in &observation.camera_observations {
                let camera_config = &self.camera_graph.cameras[camera_id];

                // Get variable names for this camera
                let intrinsics_var = format!("intrinsics_{}", camera_id);
                let pose_var = format!("pose_{}", camera_id);

                // Create multi-camera reprojection factor
                let factor = MultiCameraReprojectionFactor::new(
                    *image_point,
                    &camera_config.model,
                    camera_id.clone(),
                );

                // Add factor with appropriate variables
                let mut variables = vec![point_var.as_str()];
                if initial_values.contains_key(&intrinsics_var) {
                    variables.push(&intrinsics_var);
                }
                if initial_values.contains_key(&pose_var) {
                    variables.push(&pose_var);
                }

                problem.add_residual_block(
                    &variables,
                    Box::new(factor),
                    None, // No loss function for now
                );
            }
        }

        // Add camera graph consistency factors
        for ((cam1, cam2), edge) in &self.camera_graph.edges {
            let pose1_var = format!("pose_{}", cam1);
            let pose2_var = format!("pose_{}", cam2);

            if initial_values.contains_key(&pose1_var) && initial_values.contains_key(&pose2_var) {
                let factor = CameraGraphFactor::new(edge.relative_pose);
                problem.add_residual_block(
                    &[&pose1_var, &pose2_var],
                    Box::new(factor),
                    None, // No loss function for now
                );
            }
        }

        // Initialize variables in the problem
        problem.initialize_variables(&initial_values);

        // Solve the optimization problem
        let optimizer_config = LevenbergMarquardtConfig {
            max_iterations: self.config.max_iterations,
            parameter_tolerance: self.config.parameter_tolerance,
            cost_tolerance: self.config.cost_tolerance,
            ..Default::default()
        };

        let mut optimizer = LevenbergMarquardt::with_config(optimizer_config);
        let opt_result = optimizer.optimize(&problem, &initial_values)?;

        // Check if optimization was successful
        let is_successful = crate::optimization::optimization_acceptable(&opt_result.status);

        if !is_successful {
            return Err(format!(
                "Multi-camera bundle adjustment failed with status: {:?}",
                opt_result.status
            )
            .into());
        }

        // Extract results
        let final_cost = opt_result.final_cost;
        let num_iterations = opt_result.iterations;

        // Extract optimized intrinsics
        for camera_id in self.camera_graph.cameras.keys() {
            let var_name = format!("intrinsics_{}", camera_id);
            if let Some(optimized) = opt_result.parameters.get(&var_name) {
                camera_intrinsics
                    .insert(camera_id.clone(), optimized.to_vector().as_slice().to_vec());
            }
        }

        // Extract optimized poses
        let camera_ids: Vec<String> = camera_poses.keys().cloned().collect();
        for camera_id in camera_ids {
            let var_name = format!("pose_{}", camera_id);
            if let Some(optimized) = opt_result.parameters.get(&var_name) {
                let params_vec = optimized.to_vector();
                let params = params_vec.as_slice();

                let rotation = na::UnitQuaternion::from_scaled_axis(na::Vector3::new(
                    params[0], params[1], params[2],
                ));
                let translation = na::Vector3::new(params[3], params[4], params[5]);

                camera_poses.insert(
                    camera_id.clone(),
                    CameraPose {
                        transform: na::Isometry3::from_parts(translation.into(), rotation),
                        is_fixed: camera_id == self.camera_graph.reference_camera,
                        confidence: 0.9, // High confidence after optimization
                    },
                );
            }
        }

        // Compute final reprojection error
        let final_reprojection_error = (final_cost / self.observations.len() as f64).sqrt();

        // Create quality metrics
        let quality_metrics = CalibrationQualityMetrics::from_reprojection_errors(vec![
                final_reprojection_error;
                self.observations.len()
            ]);

        self.status = MultiCameraCalibrationStatus::Success;
        let duration = self
            .start_time
            .unwrap_or(std::time::Instant::now())
            .elapsed();

        self.log_progress(&format!(
            "✅ Multi-camera calibration completed in {:.2}s ({} iterations)",
            duration.as_secs_f64(),
            num_iterations
        ));
        self.log_progress(&format!(
            "📊 Final reprojection error: {:.2}px, Quality: {:.1}%",
            final_reprojection_error, quality_metrics.accuracy_percentage_1px
        ));

        Ok(MultiCameraCalibrationResult {
            camera_intrinsics,
            camera_poses,
            final_reprojection_error,
            num_observations: self.observations.len(),
            num_cameras: self.camera_graph.cameras.len(),
            quality_metrics,
        })
    }

    /// Get current calibration status
    pub const fn status(&self) -> MultiCameraCalibrationStatus {
        self.status
    }

    /// Get progress percentage (0-100)
    pub fn progress_percentage(&self) -> f64 {
        if self.observations.is_empty() {
            0.0
        } else {
            (self.observations.len() as f64 / self.config.max_observations as f64 * 100.0)
                .min(100.0)
        }
    }

    /// Log a progress message
    fn log_progress(&self, message: &str) {
        log::info!("{}", message);
    }
}

/// Helper function to create a camera graph from camera configurations
pub fn create_camera_graph(cameras: Vec<CameraConfig>, reference_camera: String) -> CameraGraph {
    let camera_map: HashMap<String, CameraConfig> = cameras
        .into_iter()
        .map(|config| (config.id.clone(), config))
        .collect();

    CameraGraph {
        cameras: camera_map,
        edges: HashMap::new(), // Edges will be added during calibration
        reference_camera,
    }
}
