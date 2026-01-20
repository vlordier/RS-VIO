use super::state::Estimator;
use crate::imu::ImuMotionPrior;
use crate::types::{Float, Matrix4x4, Vector3};

impl Estimator {
    /// Test hook: set maximum map points for bounding memory during tests.
    pub fn set_max_map_points(&mut self, max_map_points: usize) {
        self.sliding_window.set_max_map_points(max_map_points);
    }

    /// Test hook: inspect current map point count.
    pub fn map_points_len(&self) -> usize {
        self.sliding_window.map_points_len()
    }

    /// Test hook: adjust frame processing deadline for timing-sensitive tests.
    pub fn set_max_frame_processing_time(&mut self, duration: std::time::Duration) {
        self.max_frame_processing_time = duration;
    }

    /// Test hook: number of frames processed.
    pub fn frame_count(&self) -> u64 {
        self.frame_count
    }

    /// Get the current trajectory (list of keyframe poses with timestamps)
    pub fn get_trajectory(&self) -> &Vec<(i64, Matrix4x4)> {
        &self.trajectory
    }

    /// Get preintegrated IMU measurements between last two keyframes
    pub fn get_imu_preintegration(&self) -> Option<&crate::imu::PreintegratedImu> {
        self.current_imu_preintegration.as_ref()
    }

    /// Get current velocity estimate
    pub fn get_velocity(&self) -> Option<Vector3> {
        if self.velocity_estimator.is_initialized() {
            let v = self.velocity_estimator.get_velocity();
            Some(Vector3::new(v.x as Float, v.y as Float, v.z as Float))
        } else {
            None
        }
    }

    /// Get current IMU-camera extrinsic calibration
    pub fn get_extrinsic_calibration(&self) -> Matrix4x4 {
        self.extrinsic_calibrator.get_extrinsics()
    }

    /// Get number of IMU measurements processed
    pub fn imu_measurement_count(&self) -> usize {
        self.imu_measurement_count
    }

    /// Get IMU motion prior for optimization
    ///
    /// Returns the preintegrated IMU measurements between the last two keyframes,
    /// useful for adding IMU constraints to bundle adjustment.
    pub fn get_imu_motion_prior(&self) -> Option<ImuMotionPrior> {
        // Check if we have valid preintegrated measurements
        if !self.imu_preintegrator.is_valid() {
            log::trace!("[Estimator] No valid IMU preintegration for prior");
            return None;
        }

        // Get last keyframe pose from sliding window
        let keyframe_poses = self.sliding_window.get_keyframe_poses();
        let last_keyframe_pose = keyframe_poses.last()?;

        // Get current velocity
        let velocity = if self.velocity_estimator_initialized {
            self.velocity_estimator.get_velocity()
        } else {
            self.current_velocity
        };

        // Use the preintegrator's method to create the prior with proper config
        self.imu_preintegrator
            .create_motion_prior(*last_keyframe_pose, velocity)
    }

    /// Reset IMU-aided keyframe selector (e.g., after loop closure)
    pub fn reset_keyframe_selector(&mut self) {
        self.keyframe_selector.reset();
    }

    /// Get IMU measurement rate (for monitoring)
    pub fn get_imu_rate(&self) -> f64 {
        if self.imu_measurement_count > 0 && self.frame_id_counter > 0 {
            self.imu_measurement_count as f64 / self.frame_id_counter as f64
        } else {
            0.0
        }
    }
}
