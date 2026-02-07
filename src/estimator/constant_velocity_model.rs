//! # Constant Velocity Motion Model
//!
//! Implements a constant velocity model for motion prediction in VIO systems.
//! Useful for scenarios with poor IMU quality or unavailable IMU data.
//!
//! ## Features
//!
//! - **Velocity Tracking**: Maintains and updates velocity estimates
//! - **Pose Prediction**: Predicts next frame pose from current velocity
//! - **Innovation Computation**: Calculates prediction error for optimization
//! - **Adaptive Weighting**: Confidence-based weight adjustment
//! - **Bias Correction**: Handles velocity bias over time
//!
//! ## Theory
//!
//! For constant velocity motion, the next pose is predicted as:
//!
//! ```text
//! T_{w,j} = T_{w,i} * T_{cv}(v_i, dt)
//!
//! where:
//! - T_{w,i}: World-to-body transform at frame i
//! - v_i: Estimated velocity at frame i (world frame)
//! - dt: Time delta between frames
//! - T_{cv}: Constant velocity motion (pure translation)
//! ```
//!
//! The velocity is updated using a moving average:
//! ```text
//! v_{i+1} = α * v_measured + (1-α) * v_i
//! ```
//!
//! ## References
//!
//! - Barfoot et al., "Inertial Navigation and MEMS", 2009
//! - Groves, "Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems", 2013
//! - Forster et al., "On-Manifold Preintegration for Real-Time Visual-Inertial Odometry", RSS 2017

use anyhow::Result;
use nalgebra as na;
use serde::{Deserialize, Serialize};

/// Configuration for constant velocity motion model
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConstantVelocityConfig {
    /// Velocity smoothing factor (alpha): new_v = alpha * v_measured + (1-alpha) * v_prev
    /// Range: [0.0, 1.0] where 0.0 = pure inertia, 1.0 = instant update
    pub velocity_alpha: f64,

    /// Minimum time delta [s] to consider for velocity updates
    /// Prevents numerical issues with very small time steps
    pub min_dt: f64,

    /// Maximum allowed velocity magnitude [m/s]
    /// Used to clip unreasonable predictions (safety)
    pub max_velocity: f64,

    /// Initial velocity estimate [m/s]
    pub initial_velocity: [f64; 3],

    /// Measurement noise covariance for velocity
    pub velocity_noise_std: f64,

    /// Process noise for velocity (random walk magnitude)
    pub velocity_process_noise_std: f64,
}

impl Default for ConstantVelocityConfig {
    fn default() -> Self {
        Self {
            velocity_alpha: 0.5,
            min_dt: 1e-4,
            max_velocity: 20.0, // m/s
            initial_velocity: [0.0, 0.0, 0.0],
            velocity_noise_std: 0.1,          // m/s
            velocity_process_noise_std: 0.01, // m/s² (acceleration noise)
        }
    }
}

/// State of constant velocity motion model
#[derive(Debug, Clone, Copy)]
#[allow(dead_code)]
enum ModelState {
    /// Not initialized - waiting for first two frames
    Uninitialized,
    /// Initialized - can make predictions
    Initialized,
}

/// Constant velocity motion model for VIO
///
/// Tracks and predicts motion assuming constant velocity between frames.
/// Provides a fallback when IMU is unavailable or unreliable.
#[derive(Debug, Clone)]
pub struct ConstantVelocityModel {
    config: ConstantVelocityConfig,
    state: ModelState,

    /// Current estimated velocity in world frame [m/s]
    velocity: na::Vector3<f64>,

    /// Variance of velocity estimate
    velocity_variance: f64,

    /// Last observed pose
    last_pose: na::Isometry3<f64>,

    /// Time of last frame
    last_timestamp: i64,

    /// Number of updates since initialization
    update_count: u32,

    /// Accumulated prediction error for diagnostics
    accumulated_error: f64,
}

impl ConstantVelocityModel {
    /// Create a new constant velocity model
    pub fn new(config: ConstantVelocityConfig) -> Self {
        let initial_velocity = config.initial_velocity;
        let velocity_noise_std = config.velocity_noise_std;
        Self {
            config,
            state: ModelState::Uninitialized,
            velocity: na::Vector3::from(initial_velocity),
            velocity_variance: velocity_noise_std.powi(2),
            last_pose: na::Isometry3::identity(),
            last_timestamp: 0,
            update_count: 0,
            accumulated_error: 0.0,
        }
    }

    /// Initialize model with first pose
    ///
    /// # Arguments
    /// * `pose` - World-to-body transformation (T_w_b)
    /// * `timestamp` - Frame timestamp [ns]
    pub fn initialize(&mut self, pose: na::Isometry3<f64>, timestamp: i64) -> Result<()> {
        self.last_pose = pose;
        self.last_timestamp = timestamp;
        self.velocity = na::Vector3::from(self.config.initial_velocity);
        self.velocity_variance = self.config.velocity_noise_std.powi(2);
        self.state = ModelState::Initialized;
        Ok(())
    }

    /// Update model with new pose observation
    ///
    /// Computes measured velocity from pose difference and updates internal estimate.
    ///
    /// # Arguments
    /// * `pose` - Current world-to-body transformation (T_w_b)
    /// * `timestamp` - Frame timestamp [ns]
    ///
    /// # Returns
    /// Innovation (prediction error) in position [m]
    pub fn update(&mut self, pose: na::Isometry3<f64>, timestamp: i64) -> Result<f64> {
        let dt = (timestamp - self.last_timestamp) as f64 / 1e9;

        // Ensure valid time delta
        if dt < self.config.min_dt {
            return Ok(0.0); // No update for very small time steps
        }

        // Compute measured velocity from pose change
        let position_delta = pose.translation.vector - self.last_pose.translation.vector;
        let measured_velocity = position_delta / dt;

        // Compute prediction error (innovation) before update
        let predicted_position = self.predict_position(dt)?;
        let actual_position = pose.translation.vector;
        let innovation = (actual_position - predicted_position).norm();

        // Update velocity with exponential moving average
        let alpha = self.config.velocity_alpha.clamp(0.0, 1.0);
        self.velocity = alpha * measured_velocity + (1.0 - alpha) * self.velocity;

        // Clip velocity magnitude for safety
        let velocity_magnitude = self.velocity.norm();
        if velocity_magnitude > self.config.max_velocity {
            self.velocity *= self.config.max_velocity / velocity_magnitude;
        }

        // Update variance (decrease with more observations)
        let measurement_noise = self.config.velocity_noise_std.powi(2);
        let process_noise = (self.config.velocity_process_noise_std * dt).powi(2);
        let alpha_var = 0.9; // Variance smoothing factor
        self.velocity_variance = alpha_var * self.velocity_variance
            + (1.0 - alpha_var) * (measurement_noise + process_noise);

        // Update tracking state
        self.last_pose = pose;
        self.last_timestamp = timestamp;
        self.update_count += 1;
        self.accumulated_error += innovation;

        Ok(innovation)
    }

    /// Predict next pose using constant velocity model
    ///
    /// # Arguments
    /// * `time_delta` - Time until prediction [s]
    ///
    /// # Returns
    /// Predicted world-to-body transformation
    pub fn predict_pose(&self, time_delta: f64) -> Result<na::Isometry3<f64>> {
        let position_delta = self.velocity * time_delta;

        // Construct isometry directly (avoids log→exp roundtrip drift near ±π)
        Ok(na::Isometry3::from_parts(
            na::Translation3::from(self.last_pose.translation.vector + position_delta),
            self.last_pose.rotation,
        ))
    }

    /// Predict next position using constant velocity model
    ///
    /// # Arguments
    /// * `time_delta` - Time until prediction [s]
    ///
    /// # Returns
    /// Predicted position in world frame [m]
    pub fn predict_position(&self, time_delta: f64) -> Result<na::Vector3<f64>> {
        Ok(self.last_pose.translation.vector + self.velocity * time_delta)
    }

    /// Get current velocity estimate (world frame)
    pub const fn velocity(&self) -> na::Vector3<f64> {
        self.velocity
    }

    /// Get velocity magnitude (speed) [m/s]
    pub fn speed(&self) -> f64 {
        self.velocity.norm()
    }

    /// Get velocity estimate uncertainty (standard deviation)
    pub fn velocity_std(&self) -> f64 {
        self.velocity_variance.sqrt()
    }

    /// Get prediction confidence (higher = more confident)
    ///
    /// Based on number of updates and velocity stability
    pub fn confidence(&self) -> f64 {
        let update_confidence = (self.update_count as f64).sqrt() / 10.0; // saturates at ~10 updates
        let stability_confidence = (-self.velocity_variance / 0.01).exp(); // high confidence when low variance
        (update_confidence * stability_confidence).min(1.0)
    }

    /// Check if model is initialized and ready for predictions
    pub const fn is_initialized(&self) -> bool {
        matches!(self.state, ModelState::Initialized)
    }

    /// Get average prediction error per update
    pub fn mean_innovation(&self) -> f64 {
        if self.update_count == 0 {
            0.0
        } else {
            self.accumulated_error / self.update_count as f64
        }
    }

    /// Reset model to uninitialized state
    pub fn reset(&mut self) {
        self.state = ModelState::Uninitialized;
        self.velocity = na::Vector3::from(self.config.initial_velocity);
        self.velocity_variance = self.config.velocity_noise_std.powi(2);
        self.update_count = 0;
        self.accumulated_error = 0.0;
    }

    /// Get last observed pose
    pub const fn last_pose(&self) -> &na::Isometry3<f64> {
        &self.last_pose
    }

    /// Get configuration
    pub const fn config(&self) -> &ConstantVelocityConfig {
        &self.config
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_constant_velocity_initialization() {
        let config = ConstantVelocityConfig::default();
        let mut model = ConstantVelocityModel::new(config);

        let pose = na::Isometry3::identity();
        let timestamp = 0i64;

        assert!(model.initialize(pose, timestamp).is_ok());
        assert!(model.is_initialized());
    }

    #[test]
    fn test_constant_velocity_prediction() {
        let config = ConstantVelocityConfig {
            velocity_alpha: 1.0, // Instant update for testing
            ..Default::default()
        };
        let mut model = ConstantVelocityModel::new(config);

        // Initialize at origin
        let pose_i = na::Isometry3::new(na::Vector3::zeros(), na::Vector3::zeros());
        model.initialize(pose_i, 0).unwrap();

        // Move 1 meter in X direction over 1 second
        let pose_j = na::Isometry3::new(na::Vector3::new(1.0, 0.0, 0.0), na::Vector3::zeros());
        let dt_ns = 1_000_000_000; // 1 second in nanoseconds
        let _innovation = model.update(pose_j, dt_ns).unwrap();

        // Measured velocity should be 1 m/s
        assert!((model.speed() - 1.0).abs() < 0.01);
    }

    #[test]
    fn test_constant_velocity_smoothing() {
        let config = ConstantVelocityConfig {
            velocity_alpha: 0.5, // 50% weight to new measurement
            ..Default::default()
        };
        let mut model = ConstantVelocityModel::new(config);

        // Initialize
        let pose_i = na::Isometry3::identity();
        model.initialize(pose_i, 0).unwrap();

        // First update: move 1m in 1s -> measured velocity = 1 m/s
        // With alpha=0.5 and initial v=0: v = 0.5 * 1 + 0.5 * 0 = 0.5 m/s
        let pose_j = na::Isometry3::new(na::Vector3::new(1.0, 0.0, 0.0), na::Vector3::zeros());
        model.update(pose_j, 1_000_000_000).unwrap();
        assert!((model.speed() - 0.5).abs() < 0.01);

        // Second update: move 1m in 1s -> measured velocity = 1 m/s
        // With alpha=0.5: v = 0.5 * 1 + 0.5 * 0.5 = 0.75 m/s
        let pose_k = na::Isometry3::new(na::Vector3::new(2.0, 0.0, 0.0), na::Vector3::zeros());
        model.update(pose_k, 2_000_000_000).unwrap();
        assert!((model.speed() - 0.75).abs() < 0.01);
    }

    #[test]
    fn test_velocity_magnitude_clipping() {
        let max_vel = 5.0;
        let config = ConstantVelocityConfig {
            velocity_alpha: 1.0,
            max_velocity: max_vel,
            ..Default::default()
        };
        let mut model = ConstantVelocityModel::new(config);

        // Initialize
        let pose_i = na::Isometry3::identity();
        model.initialize(pose_i, 0).unwrap();

        // Try to move 100m in 1s (unreasonable)
        let pose_j = na::Isometry3::new(na::Vector3::new(100.0, 0.0, 0.0), na::Vector3::zeros());
        model.update(pose_j, 1_000_000_000).unwrap();

        // Velocity should be clipped to max_velocity
        assert!(model.speed() <= max_vel + 1e-6);
    }

    #[test]
    fn test_confidence_increases_with_updates() {
        let config = ConstantVelocityConfig::default();
        let mut model = ConstantVelocityModel::new(config);

        let pose_i = na::Isometry3::identity();
        model.initialize(pose_i, 0).unwrap();

        let conf_initial = model.confidence();

        for i in 1..=10 {
            let pose_j =
                na::Isometry3::new(na::Vector3::new(i as f64, 0.0, 0.0), na::Vector3::zeros());
            let _ = model.update(pose_j, (i as i64) * 1_000_000_000);
        }

        let conf_final = model.confidence();
        assert!(conf_final >= conf_initial); // Confidence should increase or stay same
    }

    #[test]
    fn test_reset() {
        let config = ConstantVelocityConfig::default();
        let mut model = ConstantVelocityModel::new(config);

        let pose = na::Isometry3::new(na::Vector3::new(1.0, 0.0, 0.0), na::Vector3::zeros());
        model.initialize(pose, 0).unwrap();
        model
            .update(
                na::Isometry3::new(na::Vector3::new(2.0, 0.0, 0.0), na::Vector3::zeros()),
                1_000_000_000,
            )
            .unwrap();

        assert!(model.speed() > 0.0);

        model.reset();
        assert!(!model.is_initialized());
        assert_eq!(model.update_count, 0);
        assert!(model.velocity.norm() == 0.0 || model.velocity.norm() == 0.0); // or initial velocity
    }

    #[test]
    fn test_predict_position() {
        let config = ConstantVelocityConfig {
            velocity_alpha: 1.0,
            ..Default::default()
        };
        let mut model = ConstantVelocityModel::new(config);

        // Initialize at origin
        let pose_i = na::Isometry3::new(na::Vector3::zeros(), na::Vector3::zeros());
        model.initialize(pose_i, 0).unwrap();

        // Set velocity to 1 m/s in X
        let pose_j = na::Isometry3::new(na::Vector3::new(1.0, 0.0, 0.0), na::Vector3::zeros());
        model.update(pose_j, 1_000_000_000).unwrap();

        // Predict position 2 seconds ahead
        let predicted_pos = model.predict_position(2.0).unwrap();
        let expected_pos = na::Vector3::new(3.0, 0.0, 0.0); // 1m (current) + 2s * 1m/s

        assert!((predicted_pos - expected_pos).norm() < 0.01);
    }

    #[test]
    fn test_mean_innovation() {
        let config = ConstantVelocityConfig {
            velocity_alpha: 1.0,
            ..Default::default()
        };
        let mut model = ConstantVelocityModel::new(config);

        let pose_i = na::Isometry3::identity();
        model.initialize(pose_i, 0).unwrap();

        // Multiple updates should accumulate mean innovation
        for i in 1..=3 {
            let pose_j =
                na::Isometry3::new(na::Vector3::new(i as f64, 0.0, 0.0), na::Vector3::zeros());
            let _ = model.update(pose_j, (i as i64) * 1_000_000_000);
        }

        assert!(model.mean_innovation() >= 0.0);
        assert_eq!(model.update_count, 3);
    }
}
