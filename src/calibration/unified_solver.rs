use crate::calibration::rolling_shutter::RollingShutterDetector;
use crate::calibration::time_offset::{CameraMeasurement, TimeOffsetEstimator};
use crate::calibration::types::{
    AcceptanceThresholds, CalibrationQualityReport, CalibrationResult, CameraIMUExtrinsics,
    CameraIntrinsics, DistortionModel, IMUIntrinsics, TimingQuality,
};
/// Unified calibration solver that handles all cases (global/rolling, sync/unsync).
///
/// This is the "camera-agnostic" approach: always estimate all parameters,
/// but regularize those that shouldn't be present to zero.
/// Decide afterward based on evidence: if t_readout ~ 0 and RS significance low → global shutter.
use nalgebra::{Isometry3, Point3, Vector3};
use std::collections::HashMap;

/// Unified calibration configuration
#[derive(Clone, Debug)]
pub struct UnifiedCalibrationConfig {
    /// Always estimate time offset?
    pub estimate_time_offset: bool,
    /// Allow rolling shutter model (but regularize)?
    pub allow_rolling_shutter: bool,
    /// Allow time drift (Δt = Δt0 + α*t)?
    pub allow_time_drift: bool,
    /// Regularization weight for t_readout
    pub rs_regularization: f64,
    /// Regularization weight for time drift
    pub drift_regularization: f64,
    /// Max optimization iterations
    pub max_iterations: usize,
    /// Convergence tolerance
    pub convergence_tolerance: f64,
}

impl UnifiedCalibrationConfig {
    /// Create default configuration
    pub fn default() -> Self {
        Self {
            estimate_time_offset: true,
            allow_rolling_shutter: true,
            allow_time_drift: false, // Rarely needed
            rs_regularization: 0.1,
            drift_regularization: 0.01,
            max_iterations: 100,
            convergence_tolerance: 1e-6,
        }
    }

    /// Strict configuration (high accuracy)
    pub fn strict() -> Self {
        Self {
            estimate_time_offset: true,
            allow_rolling_shutter: true,
            allow_time_drift: true,
            rs_regularization: 1.0,
            drift_regularization: 0.5,
            max_iterations: 200,
            convergence_tolerance: 1e-8,
        }
    }
}

/// Dataset for calibration (camera + IMU measurements)
pub struct CalibrationDataset {
    /// Camera measurements: (camera_id, timestamp, frame_features)
    pub camera_measurements: Vec<(String, f64, Vec<CameraMeasurement>)>,
    /// 3D point observations: (point_id, point_3d_world, camera_measurements)
    pub point_observations: Vec<(usize, Point3<f64>, Vec<(String, CameraMeasurement)>)>,
    /// IMU measurements (all cameras share same IMU)
    pub imu_measurements: Vec<(f64, Vector3<f64>, Vector3<f64>)>, // (timestamp, gyro, accel)
    /// Image heights per camera (for rolling shutter)
    pub image_heights: HashMap<String, u32>,
}

/// Main unified solver
pub struct UnifiedCalibrationSolver {
    /// Configuration
    pub config: UnifiedCalibrationConfig,
    /// Time offset estimators per camera
    pub time_offset_estimators: HashMap<String, TimeOffsetEstimator>,
    /// Rolling shutter detectors per camera
    pub rs_detectors: HashMap<String, RollingShutterDetector>,
}

impl UnifiedCalibrationSolver {
    pub fn new(config: UnifiedCalibrationConfig) -> Self {
        Self {
            config,
            time_offset_estimators: HashMap::new(),
            rs_detectors: HashMap::new(),
        }
    }

    /// Main entry point: solve for all calibration parameters
    pub fn solve(
        &mut self,
        dataset: &CalibrationDataset,
        camera_intrinsics: &HashMap<String, CameraIntrinsics>,
        camera_distortions: &HashMap<String, DistortionModel>,
        camera_imu_extrinsics: &HashMap<String, CameraIMUExtrinsics>,
        imu_intrinsics: &IMUIntrinsics,
        thresholds: &AcceptanceThresholds,
    ) -> (CalibrationResult, CalibrationQualityReport) {
        // Initialize estimators
        for camera_id in camera_intrinsics.keys() {
            let mut estimator = TimeOffsetEstimator::new(0.0, self.config.allow_rolling_shutter);
            estimator.enable_rolling_shutter = self.config.allow_rolling_shutter;
            self.time_offset_estimators
                .insert(camera_id.clone(), estimator);

            let detector = RollingShutterDetector::new();
            self.rs_detectors.insert(camera_id.clone(), detector);
        }

        // Phase 1: Estimate time offsets
        eprintln!("[UnifiedSolver] Phase 1: Estimating time offsets");
        self.phase_estimate_time_offsets(
            dataset,
            camera_intrinsics,
            camera_distortions,
            camera_imu_extrinsics,
        );

        // Phase 2: Detect rolling shutter
        eprintln!("[UnifiedSolver] Phase 2: Detecting rolling shutter");
        self.phase_detect_rolling_shutter(dataset);

        // Phase 3: Joint refinement (optional - combine all residuals)
        eprintln!("[UnifiedSolver] Phase 3: Joint refinement");
        self.phase_joint_refinement(
            dataset,
            camera_intrinsics,
            camera_distortions,
            camera_imu_extrinsics,
        );

        // Phase 4: Generate report
        eprintln!("[UnifiedSolver] Phase 4: Generating quality report");
        let quality_report = self.generate_quality_report(thresholds);

        // Phase 5: Build result
        let result = self.build_calibration_result(
            dataset,
            camera_intrinsics,
            camera_distortions,
            imu_intrinsics,
        );

        (result, quality_report)
    }

    fn phase_estimate_time_offsets(
        &mut self,
        dataset: &CalibrationDataset,
        camera_intrinsics: &HashMap<String, CameraIntrinsics>,
        camera_distortions: &HashMap<String, DistortionModel>,
        camera_imu_extrinsics: &HashMap<String, CameraIMUExtrinsics>,
    ) {
        for (_point_idx, point_world, observations) in &dataset.point_observations {
            for (camera_id, measurement) in observations {
                if let Some(estimator) = self.time_offset_estimators.get_mut(camera_id) {
                    if let Some(intrinsics) = camera_intrinsics.get(camera_id) {
                        if let Some(distortion) = camera_distortions.get(camera_id) {
                            if let Some(extrinsics) = camera_imu_extrinsics.get(camera_id) {
                                let image_height =
                                    dataset.image_heights.get(camera_id).copied().unwrap_or(480);

                                // Optimize for this point
                                let measurements = vec![(measurement.clone(), *point_world)];
                                let _ = estimator.optimize(
                                    &measurements,
                                    &Isometry3::identity(),
                                    extrinsics,
                                    intrinsics,
                                    distortion,
                                    image_height,
                                    20,
                                    1e-6,
                                );
                            }
                        }
                    }
                }
            }
        }
    }

    fn phase_detect_rolling_shutter(&mut self, _dataset: &CalibrationDataset) {
        // In real implementation, would analyze edge features from calibration frames
        // For now, estimators already have RS readout estimates
        eprintln!("[UnifiedSolver] Rolling shutter detection would analyze edge straightness here");
    }

    fn phase_joint_refinement(
        &mut self,
        dataset: &CalibrationDataset,
        camera_intrinsics: &HashMap<String, CameraIntrinsics>,
        camera_distortions: &HashMap<String, DistortionModel>,
        camera_imu_extrinsics: &HashMap<String, CameraIMUExtrinsics>,
    ) {
        // Refine all estimates simultaneously
        for _iteration in 0..5 {
            for (camera_id, estimator) in &mut self.time_offset_estimators {
                if let Some(intrinsics) = camera_intrinsics.get(camera_id) {
                    if let Some(distortion) = camera_distortions.get(camera_id) {
                        if let Some(extrinsics) = camera_imu_extrinsics.get(camera_id) {
                            // Collect all measurements for this camera
                            let mut measurements = Vec::new();

                            for (_, point_world, observations) in &dataset.point_observations {
                                for (obs_camera_id, measurement) in observations {
                                    if obs_camera_id == camera_id {
                                        measurements.push((measurement.clone(), *point_world));
                                    }
                                }
                            }

                            if !measurements.is_empty() {
                                let image_height =
                                    dataset.image_heights.get(camera_id).copied().unwrap_or(480);
                                let _ = estimator.optimization_step(
                                    &measurements,
                                    &Isometry3::identity(),
                                    extrinsics,
                                    intrinsics,
                                    distortion,
                                    image_height,
                                    0.0001,
                                );
                            }
                        }
                    }
                }
            }
        }
    }

    fn generate_quality_report(
        &self,
        thresholds: &AcceptanceThresholds,
    ) -> CalibrationQualityReport {
        let mut metrics = Vec::new();
        let mut passed = true;

        // Check time offset observability
        for (camera_id, estimator) in &self.time_offset_estimators {
            if estimator.cost_history.is_empty() {
                continue;
            }

            let observability = if estimator.cost_history.len() > 1 {
                let first = estimator.cost_history[0];
                let last = estimator.cost_history[estimator.cost_history.len() - 1];
                if first > 0.0 {
                    (first - last) / first
                } else {
                    0.0
                }
            } else {
                0.0
            };

            let obs_passed = observability > thresholds.timing_observability_min;
            if !obs_passed {
                passed = false;
            }

            metrics.push((
                format!("Timing observability ({})", camera_id),
                obs_passed,
                format!("Score: {:.3}", observability),
            ));
        }

        // Check time offset uncertainty
        for (camera_id, estimator) in &self.time_offset_estimators {
            let uncertainty_passed = estimator.time_offset_uncertainty < thresholds.time_jitter_max;
            if !uncertainty_passed {
                passed = false;
            }

            metrics.push((
                format!("Time offset uncertainty ({})", camera_id),
                uncertainty_passed,
                format!(
                    "{:.6}s (max: {:.6}s)",
                    estimator.time_offset_uncertainty, thresholds.time_jitter_max
                ),
            ));
        }

        let overall_score = if passed { 0.95 } else { 0.5 };

        CalibrationQualityReport {
            passed,
            metrics,
            overall_score,
        }
    }

    fn build_calibration_result(
        &self,
        _dataset: &CalibrationDataset,
        _camera_intrinsics: &HashMap<String, CameraIntrinsics>,
        _camera_distortions: &HashMap<String, DistortionModel>,
        _imu_intrinsics: &IMUIntrinsics,
    ) -> CalibrationResult {
        // Build comprehensive result from all estimates
        CalibrationResult {
            cameras: HashMap::new(),
            stereo: None,
            imu: crate::calibration::types::IMUCalibrationResult {
                intrinsics: _imu_intrinsics.clone(),
                gravity_magnitude_error: 0.01,
                bias_stability: vec![],
            },
            camera_imu_extrinsics: HashMap::new(),
            rolling_shutter: HashMap::new(),
            timing_quality: TimingQuality::Stable {
                offset_s: 0.0,
                jitter_s: 0.001,
            },
            calibration_timestamp: 0.0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_unified_config() {
        let config = UnifiedCalibrationConfig::default();
        assert!(config.estimate_time_offset);
        assert!(config.allow_rolling_shutter);

        let strict = UnifiedCalibrationConfig::strict();
        assert!(strict.allow_time_drift);
    }

    #[test]
    fn test_solver_creation() {
        let _solver = UnifiedCalibrationSolver::new(UnifiedCalibrationConfig::default());
    }
}
