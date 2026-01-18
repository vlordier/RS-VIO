//! IMU Processing Module
//!
//! Handles all IMU-related processing with clear separation from visual pipeline:
//! - IMU data buffering and validation
//! - Preintegration coordination
//! - Velocity estimation
//! - Bias estimation and tracking
//!
//! Optimized for hotpath performance with minimal allocations.

use crate::datasets::ImuData;
use crate::imu::{
    ImuAidedKeyframeSelector, ImuBiasEstimator, ImuMotionPredictor, ImuMotionPrior,
    ImuPreintegrator, PreintegratedImu, VelocityEstimator,
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
    preintegrator: ImuPreintegrator,
    motion_predictor: ImuMotionPredictor,
    velocity_estimator: VelocityEstimator,
    #[allow(dead_code)]
    bias_estimator: ImuBiasEstimator,
    keyframe_selector: ImuAidedKeyframeSelector,

    // State
    current_preintegration: Option<PreintegratedImu>,
    last_timestamp: Option<i64>,
    stats: ImuStatistics,
    #[allow(dead_code)]
    current_velocity: Vector3,
    current_rotation: na::UnitQuaternion<f64>,
    is_initialized: bool,
}

impl ImuProcessor {
    /// Create new IMU processor
    pub fn new(
        preintegrator: ImuPreintegrator,
        motion_predictor: ImuMotionPredictor,
        velocity_estimator: VelocityEstimator,
        bias_estimator: ImuBiasEstimator,
        keyframe_selector: ImuAidedKeyframeSelector,
    ) -> Self {
        Self {
            preintegrator,
            motion_predictor,
            velocity_estimator,
            bias_estimator,
            keyframe_selector,
            current_preintegration: None,
            last_timestamp: None,
            stats: ImuStatistics::default(),
            current_velocity: Vector3::zeros(),
            current_rotation: na::UnitQuaternion::identity(),
            is_initialized: false,
        }
    }

    /// Process IMU measurements for current frame
    ///
    /// Optimized hotpath: minimal allocations, early returns, batch processing
    #[inline]
    pub fn process_measurements(&mut self, measurements: &[ImuData]) -> ImuProcessingResult {
        if measurements.is_empty() {
            return ImuProcessingResult::default();
        }

        let mut result = ImuProcessingResult {
            num_measurements: measurements.len(),
            preintegrated: None,
            motion_prior: None,
            velocity: None,
        };

        // Update statistics
        self.stats.total_measurements += measurements.len();
        self.stats.measurements_this_frame = measurements.len();

        // Batch process measurements (hotpath optimization)
        for (i, imu) in measurements.iter().enumerate() {
            // Compute dt from previous measurement
            let dt = if i > 0 {
                (imu.timestamp - measurements[i - 1].timestamp) as f64 / 1e9
            } else if let Some(last_ts) = self.last_timestamp {
                (imu.timestamp - last_ts) as f64 / 1e9
            } else {
                0.005 // Default 200Hz
            };

            if dt > 0.0 && dt < 1.0 {
                // Reasonable dt (0-1 second)
                // Preintegration
                self.preintegrator.propagate(imu, dt);

                // Update rotation estimate for motion predictor
                // Use preintegrated rotation
                let preint = self.preintegrator.get();
                self.current_rotation = preint.delta_rotation;

                // Motion prediction for feature tracking
                self.motion_predictor
                    .update(imu.timestamp, self.current_rotation);

                // Velocity estimation
                if !self.is_initialized && measurements.len() >= 50 {
                    // Initialize velocity estimator with first N measurements
                    let init_orientation = na::UnitQuaternion::identity();
                    self.velocity_estimator
                        .initialize_from_imu(&measurements[..50], &init_orientation);
                    self.is_initialized = true;
                }

                if self.is_initialized {
                    let imu_copy = [imu.clone()];
                    self.velocity_estimator.update(&imu_copy, dt);
                }

                // Update keyframe selector
                self.keyframe_selector.update_imu(imu);
            }

            self.last_timestamp = Some(imu.timestamp);
        }

        // Get results
        result.preintegrated = Some(self.preintegrator.get().clone());
        result.motion_prior = None; // Motion prior not used in current implementation
        result.velocity = if self.is_initialized {
            Some(self.velocity_estimator.get_velocity())
        } else {
            None
        };

        // Update statistics
        if let Some(last_ts) = self.last_timestamp {
            let time_span = (last_ts - measurements[0].timestamp) as f64 / 1e9;
            if time_span > 0.0 {
                self.stats.average_rate_hz =
                    self.stats.total_measurements as f64 / time_span.max(1.0);
            }
        }
        self.stats.last_timestamp = self.last_timestamp;

        result
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
        None // Not implemented yet
    }

    /// Get current velocity estimate
    #[inline]
    pub fn get_velocity(&self) -> Option<Vector3> {
        if self.is_initialized {
            Some(self.velocity_estimator.get_velocity())
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
        self.is_initialized
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
    use crate::imu::ImuConfig;

    fn create_test_processor() -> ImuProcessor {
        let config = ImuConfig::default();
        ImuProcessor::new(
            ImuPreintegrator::new(config.clone()),
            ImuMotionPredictor::new(config.clone()),
            VelocityEstimator::new(config.clone()),
            ImuBiasEstimator::new(config.clone()),
            ImuAidedKeyframeSelector::new(0.5, 0.1),
        )
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
