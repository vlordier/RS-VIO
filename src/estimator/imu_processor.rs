//! IMU Processing Module
//!
//! Handles all IMU-related processing with clear separation from visual pipeline:
//! - IMU data buffering and validation
//! - Preintegration coordination
//! - Velocity estimation
//! - Bias estimation and tracking
//!
//! Optimized for hotpath performance with minimal allocations.

use crate::datasets::config::Config;
use crate::datasets::ImuData;
use crate::imu::{
    DenoiseConfig, ExtrinsicCalibrator, HigherOrderFilter, HigherOrderFilterConfig,
    ImuAidedKeyframeSelector, ImuBiasEstimator, ImuConfig, ImuDenoiseFilter, ImuMotionPredictor,
    ImuMotionPrior, ImuPreintegrator, PreintegratedImu, VelocityEstimator,
};
use crate::types::{Float, Matrix4x4, Vector3};
use nalgebra as na;

/// IMU processing statistics
#[derive(Debug, Clone, Default)]
pub struct ImuStatistics {
    pub total_measurements: usize,
    pub measurements_this_frame: usize,
    pub average_rate_hz: f64,
    pub last_timestamp: Option<i64>,
}

/// Centralized IMU processor
///
/// Coordinates all IMU-related computations for a VIO system.
/// Maintains state for preintegration, velocity estimation, and bias tracking.
pub struct ImuProcessor {
    pub preintegrator: ImuPreintegrator,
    pub motion_predictor: ImuMotionPredictor,
    pub velocity_estimator: VelocityEstimator,
    pub extrinsic_calibrator: ExtrinsicCalibrator,
    pub keyframe_selector: ImuAidedKeyframeSelector,
    pub current_preintegration: Option<PreintegratedImu>,
    pub last_timestamp: Option<i64>,
    pub stats: ImuStatistics,
    pub current_velocity: Vector3,
    pub velocity_estimator_initialized: bool,
    pub bias_estimator: ImuBiasEstimator,
    pub is_initializing: bool,
    pub denoise_filter: ImuDenoiseFilter,
    pub higher_order_filter: HigherOrderFilter,
    pub last_pose: Option<Matrix4x4>,
    pub gravity: Vector3,
}

impl ImuProcessor {
    /// Create new IMU processor
    pub fn new(config: &Config, T_B_Cl: Matrix4x4) -> Self {
        let imu_config = ImuConfig::default();
        Self {
            preintegrator: ImuPreintegrator::new(imu_config.clone()),
            motion_predictor: ImuMotionPredictor::new(imu_config.clone()),
            velocity_estimator: VelocityEstimator::new(imu_config.clone()),
            extrinsic_calibrator: ExtrinsicCalibrator::new(T_B_Cl),
            keyframe_selector: ImuAidedKeyframeSelector::new(
                config.keyframe_management.translation_threshold,
                config.keyframe_management.rotation_threshold,
            ),
            current_preintegration: None,
            last_timestamp: None,
            stats: ImuStatistics::default(),
            current_velocity: Vector3::zeros(),
            velocity_estimator_initialized: false,
            bias_estimator: ImuBiasEstimator::new(imu_config.clone()),
            is_initializing: true, // Start in initializing state
            denoise_filter: ImuDenoiseFilter::new(DenoiseConfig::default()),
            higher_order_filter: HigherOrderFilter::new(HigherOrderFilterConfig::default()),
            last_pose: None,
            gravity: Vector3::new(0.0, 0.0, -9.81),
        }
    }

    /// Process IMU measurements for current frame
    ///
    /// Optimized hotpath: zero allocations, batch processing, cache-friendly access
    #[inline(always)]
    pub fn process_measurements(&mut self, measurements: &[ImuData]) -> ImuProcessingResult {
        // Fast path: empty measurements
        if measurements.is_empty() {
            return ImuProcessingResult::default();
        }

        // Update statistics (no allocations)
        let num_measurements = measurements.len();
        self.stats.total_measurements += num_measurements;
        self.stats.measurements_this_frame = num_measurements;

        // Check for velocity estimator initialization once before loop
        let should_init_velocity = !self.velocity_estimator_initialized && num_measurements >= 50;

        // Batch process measurements (hotpath - zero allocations)
        for (i, imu) in measurements.iter().enumerate() {
            // Compute dt from previous measurement (branchless when possible)
            let dt = if i > 0 {
                (imu.timestamp - measurements[i - 1].timestamp) as f64 / 1e9
            } else if let Some(last_ts) = self.last_timestamp {
                (imu.timestamp - last_ts) as f64 / 1e9
            } else {
                0.005 // Default 200Hz
            };

            // Early continue for invalid dt
            if dt <= 0.0 || dt >= 1.0 {
                self.last_timestamp = Some(imu.timestamp);
                continue;
            }

            // Hotpath: preintegration (no allocations)
            self.preintegrator.propagate(imu, dt);

            // Motion prediction - reuse preintegration result
            if i == num_measurements - 1 {
                // Only update motion predictor on last measurement to reduce overhead
                let preint = self.preintegrator.get();
                self.motion_predictor
                    .update(imu.timestamp, preint.delta_rotation);
            }

            // Velocity estimation initialization (done once)
            if should_init_velocity && i == 49 {
                let init_orientation = na::UnitQuaternion::identity();
                self.velocity_estimator
                    .initialize_from_imu(&measurements[..50], &init_orientation);
                self.velocity_estimator_initialized = true;
            }

            // Velocity estimation update (zero-copy via slice)
            if self.velocity_estimator_initialized {
                // Pass single-element slice instead of cloning
                self.velocity_estimator
                    .update(std::slice::from_ref(imu), dt);
                self.current_velocity = self.velocity_estimator.get_velocity();
            }

            // Update keyframe selector
            self.keyframe_selector.update_imu(imu);

            self.last_timestamp = Some(imu.timestamp);
        }

        // Get preintegration result once
        let preint = self.preintegrator.get();

        // Build result (minimize cloning)
        let preintegrated = Some(preint.clone());

        // Create motion prior if we have all necessary data
        let motion_prior = if let Some(last_pose) = self.last_pose {
            if self.velocity_estimator_initialized {
                Some(ImuMotionPrior::from_preintegration(
                    preint,
                    last_pose,
                    self.current_velocity,
                    self.gravity,
                ))
            } else {
                None
            }
        } else {
            None
        };

        let velocity = if self.velocity_estimator_initialized {
            Some(self.current_velocity)
        } else {
            None
        };

        // Update statistics (avoid division in hotpath when possible)
        if let Some(last_ts) = self.last_timestamp {
            let time_span = (last_ts - measurements[0].timestamp) as f64 / 1e9;
            if time_span > 0.0 {
                self.stats.average_rate_hz =
                    self.stats.total_measurements as f64 / time_span.max(1.0);
            }
        }
        self.stats.last_timestamp = self.last_timestamp;

        ImuProcessingResult {
            num_measurements,
            preintegrated,
            motion_prior,
            velocity,
        }
    }

    /// Accumulate IMU for keyframe selection
    #[inline]
    pub fn accumulate_for_keyframe(&mut self, measurements: &[ImuData]) {
        self.keyframe_selector.accumulate_imu(measurements);
    }

    /// Check if keyframe should be created (IMU criteria)
    pub fn should_create_keyframe(
        &mut self,
        current_pose: &Matrix4x4,
        timestamp: i64,
        visual_motion: Float,
    ) -> (bool, String) {
        self.keyframe_selector
            .should_be_keyframe(current_pose, timestamp, visual_motion)
    }

    /// Get current IMU motion prior
    #[inline]
    pub fn get_motion_prior(&self) -> Option<ImuMotionPrior> {
        if let Some(last_pose) = self.last_pose {
            if self.velocity_estimator_initialized {
                let preint = self.preintegrator.get();
                return Some(ImuMotionPrior::from_preintegration(
                    preint,
                    last_pose,
                    self.current_velocity,
                    self.gravity,
                ));
            }
        }
        None
    }

    /// Update the last pose for motion prior computation
    #[inline]
    pub fn update_pose(&mut self, pose: Matrix4x4) {
        self.last_pose = Some(pose);
    }

    /// Get current velocity estimate
    #[inline]
    pub fn get_velocity(&self) -> Option<Vector3> {
        if self.velocity_estimator_initialized {
            Some(self.current_velocity)
        } else {
            None
        }
    }

    /// Get current preintegrated measurements
    #[inline]
    pub fn get_preintegrated(&self) -> Option<&PreintegratedImu> {
        self.current_preintegration.as_ref()
    }

    /// Reset preintegration (called after keyframe)
    #[inline]
    pub fn reset_preintegration(&mut self) {
        self.preintegrator.reset();
        self.current_preintegration = None;
    }

    /// Get processing statistics
    pub fn get_statistics(&self) -> &ImuStatistics {
        &self.stats
    }

    /// Get current IMU rate
    pub fn get_rate_hz(&self) -> f64 {
        self.stats.average_rate_hz
    }

    /// Check if system is initialized
    pub fn is_initialized(&self) -> bool {
        !self.is_initializing && self.velocity_estimator_initialized
    }
}

/// Result of IMU processing for a frame
#[derive(Debug, Clone, Default)]
pub struct ImuProcessingResult {
    pub num_measurements: usize,
    pub preintegrated: Option<PreintegratedImu>,
    pub motion_prior: Option<ImuMotionPrior>,
    pub velocity: Option<Vector3>,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::datasets::config::{
        CameraConfig, Config, KeyframeManagementConfig, OptimizationConfig,
    };
    use nalgebra::Matrix4;

    fn create_test_processor() -> ImuProcessor {
        let camera = CameraConfig {
            image_width: 640,
            image_height: 480,
            left_intrinsics: vec![500.0, 500.0, 320.0, 240.0],
            left_distortion: vec![0.0, 0.0, 0.0, 0.0],
            right_intrinsics: vec![500.0, 500.0, 320.0, 240.0],
            right_distortion: vec![0.0, 0.0, 0.0, 0.0],
            left_model: None,
            right_model: None,
            T_B_Cl: vec![
                1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
            ],
            T_B_Cr: vec![
                1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
            ],
        };
        let keyframe_management = KeyframeManagementConfig {
            keyframe_window_size: 10,
            translation_threshold: 0.1,
            rotation_threshold: 0.1,
            processing_timeout_ms: 1000,
        };
        let optimization = OptimizationConfig {
            bundle_adjustment_max_iterations: 10,
            pnp_max_iterations: 10,
            imu_prior_enable: true,
            imu_prior_weight_pos: 1.0,
            imu_prior_weight_rot: 1.0,
            imu_prior_huber_delta: 1.0,
        };
        let config = Config {
            camera,
            keyframe_management,
            optimization,
            feature_detection: Default::default(),
            visualization: Default::default(),
            debug: Default::default(),
            loop_closure: Default::default(),
            marginalization: Default::default(),
        };
        let T_B_Cl = Matrix4::identity();
        ImuProcessor::new(&config, T_B_Cl)
    }

    #[test]
    fn test_imu_processor_creation() {
        let processor = create_test_processor();
        assert!(!processor.is_initialized());
        assert_eq!(processor.get_rate_hz(), 0.0);
    }

    #[test]
    fn test_process_empty_measurements() {
        let mut processor = create_test_processor();
        let result = processor.process_measurements(&[]);
        assert_eq!(result.num_measurements, 0);
    }

    #[test]
    fn test_process_single_measurement() {
        let mut processor = create_test_processor();
        let imu = ImuData {
            timestamp: 1000000000,
            gyro: [0.01, 0.02, 0.03],
            accel: [0.0, 0.0, -9.81],
        };
        let result = processor.process_measurements(&[imu]);
        assert_eq!(result.num_measurements, 1);
        assert!(result.preintegrated.is_some());
    }

    #[test]
    fn test_statistics_tracking() {
        let mut processor = create_test_processor();
        let measurements: Vec<ImuData> = (0..10)
            .map(|i| ImuData {
                timestamp: 1000000000 + i * 5_000_000, // 200Hz
                gyro: [0.01, 0.02, 0.03],
                accel: [0.0, 0.0, -9.81],
            })
            .collect();

        processor.process_measurements(&measurements);
        let stats = processor.get_statistics();
        assert_eq!(stats.total_measurements, 10);
        assert_eq!(stats.measurements_this_frame, 10);
    }
}
