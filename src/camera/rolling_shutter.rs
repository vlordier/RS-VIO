//! # Rolling Shutter Compensation
//!
//! Compensates for rolling shutter distortion in camera images.
//! Rolling shutter cameras expose rows sequentially, causing geometric
//! distortion during fast motion, especially rotation.
//!
//! ## Algorithm Overview
//!
//! 1. **Row Timing Model**: Each image row has a different exposure timestamp
//! 2. **Motion Interpolation**: Use IMU data to interpolate camera pose at each row time
//! 3. **Feature Correction**: Adjust feature positions based on relative motion during exposure
//! 4. **Temporal undistortion**: Transform features to appear as if captured instantaneously
//!
//! ## Key Features
//!
//! - IMU-based motion compensation during exposure
//! - Per-row timestamp modeling
//! - Real-time feature position correction
//! - Integration with existing VIO pipeline
//!
//! ## Usage
//!
//! ```rust,ignore
//! let compensator = RollingShutterCompensator::new(readout_time, image_height);
//! let corrected_features = compensator.compensate_features(
//!     &raw_features,
//!     &imu_buffer,
//!     image_timestamp,
//!     &current_pose
//! );
//! ```

use crate::datasets::ImuData;
use crate::types::Float;
use nalgebra as na;

/// Configuration for rolling shutter compensation
#[derive(Debug, Clone)]
pub struct RollingShutterConfig {
    /// Total readout time for full frame [seconds]
    pub readout_time: Float,
    /// Image height in pixels
    pub image_height: usize,
    /// Readout direction (true = top-to-bottom, false = bottom-to-top)
    pub top_to_bottom: bool,
}

impl Default for RollingShutterConfig {
    fn default() -> Self {
        Self {
            readout_time: 0.02, // 20ms typical for global shutter cameras
            image_height: 480,
            top_to_bottom: true,
        }
    }
}

/// Rolling shutter compensator for feature position correction
pub struct RollingShutterCompensator {
    config: RollingShutterConfig,
}

impl RollingShutterCompensator {
    /// Create new rolling shutter compensator
    pub fn new(readout_time: Float, image_height: usize) -> Self {
        Self {
            config: RollingShutterConfig {
                readout_time,
                image_height,
                top_to_bottom: true,
            },
        }
    }

    /// Compensate feature positions for rolling shutter distortion
    ///
    /// # Arguments
    /// * `features` - Raw feature positions [(u, v), ...]
    /// * `imu_buffer` - IMU measurements during the exposure period
    /// * `image_timestamp` - Timestamp when first row begins exposure
    /// * `current_pose` - Current camera pose estimate T_W_C
    ///
    /// # Returns
    /// Compensated feature positions
    pub fn compensate_features(
        &self,
        features: &[(Float, Float)],
        imu_buffer: &[ImuData],
        image_timestamp: i64,
        current_pose: &na::Matrix4<Float>,
    ) -> Vec<(Float, Float)> {
        if imu_buffer.is_empty() || features.is_empty() {
            return features.to_vec();
        }

        features
            .iter()
            .map(|&(u, v)| {
                self.compensate_single_feature(u, v, imu_buffer, image_timestamp, current_pose)
            })
            .collect()
    }

    /// Compensate a single feature position
    fn compensate_single_feature(
        &self,
        u: Float,
        v: Float,
        imu_buffer: &[ImuData],
        image_timestamp: i64,
        current_pose: &na::Matrix4<Float>,
    ) -> (Float, Float) {
        // Calculate row timestamp based on vertical position
        let row_time = self.row_timestamp(v, image_timestamp);

        // Interpolate camera pose at row timestamp
        let row_pose = self.interpolate_pose(imu_buffer, row_time, current_pose);

        // Calculate relative transformation from current pose to row pose
        let relative_transform = if let Some(inv_current) = current_pose.try_inverse() {
            inv_current * row_pose
        } else {
            na::Matrix4::identity()
        };

        // Apply transformation to feature position
        self.transform_feature(u, v, &relative_transform)
    }

    /// Calculate timestamp for a given image row
    fn row_timestamp(&self, v: Float, image_timestamp: i64) -> i64 {
        let row_fraction = if self.config.top_to_bottom {
            v / self.config.image_height as Float
        } else {
            (self.config.image_height as Float - v) / self.config.image_height as Float
        };

        let row_delay_ns = (row_fraction * self.config.readout_time * 1e9) as i64;
        image_timestamp + row_delay_ns
    }

    /// Interpolate camera pose at a specific timestamp using IMU data
    fn interpolate_pose(
        &self,
        imu_buffer: &[ImuData],
        target_timestamp: i64,
        current_pose: &na::Matrix4<Float>,
    ) -> na::Matrix4<Float> {
        if imu_buffer.is_empty() {
            return *current_pose;
        }

        // Find IMU measurements bracketing the target timestamp
        let mut before_idx = None;
        let mut after_idx = None;

        for (i, imu) in imu_buffer.iter().enumerate() {
            if imu.timestamp <= target_timestamp {
                before_idx = Some(i);
            }
            if imu.timestamp >= target_timestamp && after_idx.is_none() {
                after_idx = Some(i);
                break;
            }
        }

        match (before_idx, after_idx) {
            (Some(b), Some(a)) if b == a => {
                // Exact match
                self.pose_from_imu(&imu_buffer[b], current_pose)
            },
            (Some(b), Some(a)) => {
                // Interpolate between measurements
                let imu_before = &imu_buffer[b];
                let imu_after = &imu_buffer[a];

                let dt_total = (imu_after.timestamp - imu_before.timestamp) as Float;
                let dt_target = (target_timestamp - imu_before.timestamp) as Float;
                let alpha = if dt_total > 0.0 {
                    dt_target / dt_total
                } else {
                    0.0
                };

                self.interpolate_between_imu(imu_before, imu_after, alpha, current_pose)
            },
            _ => {
                // Extrapolate using the most recent measurement if available
                if let Some(latest_imu) = imu_buffer.last() {
                    self.pose_from_imu(latest_imu, current_pose)
                } else {
                    *current_pose
                }
            },
        }
    }

    /// Estimate pose from single IMU measurement (simplified)
    fn pose_from_imu(
        &self,
        _imu: &ImuData,
        reference_pose: &na::Matrix4<Float>,
    ) -> na::Matrix4<Float> {
        // This is a simplified implementation
        // In practice, this would integrate IMU measurements properly
        *reference_pose
    }

    /// Interpolate pose between two IMU measurements
    fn interpolate_between_imu(
        &self,
        imu_before: &ImuData,
        imu_after: &ImuData,
        alpha: Float,
        reference_pose: &na::Matrix4<Float>,
    ) -> na::Matrix4<Float> {
        // Linear interpolation of angular velocity
        let gyro_before = na::Vector3::new(
            imu_before.gyro[0] as Float,
            imu_before.gyro[1] as Float,
            imu_before.gyro[2] as Float,
        );
        let gyro_after = na::Vector3::new(
            imu_after.gyro[0] as Float,
            imu_after.gyro[1] as Float,
            imu_after.gyro[2] as Float,
        );

        let gyro_interp = gyro_before.lerp(&gyro_after, alpha);

        // Estimate small rotation
        let angle = gyro_interp.norm();
        let axis = if angle > 1e-6 {
            gyro_interp / angle
        } else {
            na::Vector3::z()
        };

        let delta_rot = na::UnitQuaternion::new(axis * angle * 0.01); // Approximate dt

        // Apply rotation to reference pose
        let rot_mat = reference_pose.fixed_view::<3, 3>(0, 0);
        let current_rot = na::UnitQuaternion::from_rotation_matrix(
            &na::Rotation3::from_matrix_unchecked(rot_mat.into_owned()),
        );
        let new_rot = delta_rot * current_rot;

        let mut new_pose = *reference_pose;
        let new_rot_mat = new_rot.to_rotation_matrix().into_inner();
        new_pose
            .fixed_view_mut::<3, 3>(0, 0)
            .copy_from(&new_rot_mat);

        new_pose
    }

    /// Transform feature position using relative pose transformation
    fn transform_feature(
        &self,
        u: Float,
        v: Float,
        transform: &na::Matrix4<Float>,
    ) -> (Float, Float) {
        // Convert pixel coordinates to normalized coordinates (assuming pinhole model)
        // This is a simplified implementation - in practice would use actual camera intrinsics
        let fx = 500.0; // focal length approximation
        let fy = 500.0;
        let cx = 320.0; // principal point approximation
        let cy = 240.0;

        let x_norm = (u - cx) / fx;
        let y_norm = (v - cy) / fy;

        // Assume unit depth for distant features
        let point_3d = na::Vector4::new(x_norm, y_norm, 1.0, 1.0);

        // Apply transformation
        let transformed = transform * point_3d;

        // Project back to pixel coordinates
        let u_new = fx * (transformed[0] / transformed[2]) + cx;
        let v_new = fy * (transformed[1] / transformed[2]) + cy;

        (u_new, v_new)
    }

    /// Get compensation configuration
    pub fn config(&self) -> &RollingShutterConfig {
        &self.config
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_rolling_shutter_compensator_creation() {
        let compensator = RollingShutterCompensator::new(0.02, 480);
        assert_eq!(compensator.config().readout_time, 0.02);
        assert_eq!(compensator.config().image_height, 480);
    }

    #[test]
    fn test_row_timestamp_calculation() {
        let compensator = RollingShutterCompensator::new(0.02, 480); // 20ms readout
        let image_timestamp = 1000000000; // 1 second

        // Top row (v=0) should have timestamp close to image_timestamp
        let top_time = compensator.row_timestamp(0.0, image_timestamp);
        assert!((top_time - image_timestamp).abs() < 1000); // Within 1us

        // Bottom row (v=479) should be delayed by readout time
        let bottom_time = compensator.row_timestamp(479.0, image_timestamp);
        let expected_delay = (479.0 / 480.0 * 0.02 * 1e9) as i64;
        assert!((bottom_time - image_timestamp - expected_delay).abs() < 1000);
    }

    #[test]
    fn test_feature_compensation_empty_imu() {
        let compensator = RollingShutterCompensator::new(0.02, 480);
        let features = vec![(100.0, 100.0), (200.0, 200.0)];
        let imu_buffer = vec![];
        let pose = na::Matrix4::identity();

        let compensated =
            compensator.compensate_features(&features, &imu_buffer, 1000000000, &pose);

        // Should return original features when no IMU data
        assert_eq!(compensated.len(), features.len());
        for (orig, comp) in features.iter().zip(compensated.iter()) {
            assert!((orig.0 - comp.0).abs() < 1e-6);
            assert!((orig.1 - comp.1).abs() < 1e-6);
        }
    }
}
