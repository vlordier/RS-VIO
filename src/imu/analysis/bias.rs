//! Bias estimation and gravity direction estimation
//!
//! Estimates accelerometer and gyroscope bias with adaptive filtering
//! based on motor state. Also estimates gravity direction from accelerometer.

use super::config::{MotorState, MIN_GRAVITY_FOR_ESTIMATION, GRAVITY_MAG, BIAS_FILTER_COEFF_MOTORS_RUNNING, BIAS_UPDATE_RATE_MOTORS_RUNNING};
use crate::types::Vector3;

/// Update gravity direction estimate from accelerometer mean
pub fn update_gravity_estimate(gravity_estimate: &mut Vector3, accel_mean: &Vector3) {
    let gravity_mag = accel_mean.norm();

    if gravity_mag > MIN_GRAVITY_FOR_ESTIMATION {
        *gravity_estimate = (accel_mean / gravity_mag) * GRAVITY_MAG;
    }
}

/// Update accelerometer bias estimate (motor-state adaptive)
pub fn update_accel_bias_estimate(
    accel_bias_estimate: &mut Vector3,
    accel_mean: &Vector3,
    gravity_estimate: &Vector3,
    motor_state: MotorState,
) {
    let bias_candidate = accel_mean - gravity_estimate;

    *accel_bias_estimate = match motor_state {
        MotorState::Off => {
            // Fast update when stationary
            bias_candidate
        },
        MotorState::Running | MotorState::Transitioning => {
            // Slow filtered update during flight
            *accel_bias_estimate * BIAS_FILTER_COEFF_MOTORS_RUNNING
                + bias_candidate * BIAS_UPDATE_RATE_MOTORS_RUNNING
        },
    };
}

/// Compute mean of vector samples
pub fn compute_mean(samples: &[Vector3]) -> Vector3 {
    if samples.is_empty() {
        return Vector3::zeros();
    }
    let sum: Vector3 = samples.iter().sum();
    sum / samples.len() as f64
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_bias_estimation_convergence() {
        let true_bias = Vector3::new(0.2, -0.15, 0.1);
        let true_gravity = -9.81;

        let mut accel_bias_estimate = Vector3::zeros();
        let mut gravity_estimate = Vector3::new(0.0, 0.0, true_gravity);

        // Feed biased measurements
        for i in 0..150 {
            let accel_mean = Vector3::new(
                true_bias.x + 0.01 * ((i % 5) as f64 - 2.5) / 5.0,
                true_bias.y + 0.01 * ((i % 7) as f64 - 3.5) / 7.0,
                true_gravity + true_bias.z + 0.01 * ((i % 3) as f64 - 1.5) / 3.0,
            );

            update_gravity_estimate(&mut gravity_estimate, &accel_mean);
            update_accel_bias_estimate(
                &mut accel_bias_estimate,
                &accel_mean,
                &gravity_estimate,
                MotorState::Off,
            );
        }

        // Bias should converge close to true bias
        assert!(
            accel_bias_estimate.x.abs() < 1.0,
            "X bias should be small: {}",
            accel_bias_estimate.x
        );
        assert!(
            accel_bias_estimate.y.abs() < 1.0,
            "Y bias should be small: {}",
            accel_bias_estimate.y
        );
    }

    #[test]
    fn test_gravity_direction_estimation() {
        let mut gravity_estimate = Vector3::new(0.0, 0.0, -9.81);

        // Tilted orientation (gravity not aligned with Z)
        // Simulate sensor tilted 45 degrees
        let angle = std::f64::consts::PI / 4.0;
        let g = 9.81;

        for _ in 0..100 {
            let accel_mean = Vector3::new(0.0, g * angle.sin(), -g * angle.cos());
            update_gravity_estimate(&mut gravity_estimate, &accel_mean);
        }

        // Gravity magnitude should still be ~9.81
        let gravity_mag = gravity_estimate.norm();
        assert!(
            (gravity_mag - 9.81).abs() < 0.5,
            "Gravity magnitude should be ~9.81: {}",
            gravity_mag
        );
    }

    #[test]
    fn test_accel_bias_motor_state_adaptation() {
        let mut accel_bias = Vector3::zeros();
        let gravity = Vector3::new(0.0, 0.0, -9.81);
        let bias_candidate = Vector3::new(0.2, -0.15, 0.1);

        // Fast update when motors off
        update_accel_bias_estimate(&mut accel_bias, &bias_candidate, &gravity, MotorState::Off);
        assert!(accel_bias.norm() > 0.1);

        // Slower update when motors running
        let initial_bias = accel_bias;
        update_accel_bias_estimate(&mut accel_bias, &bias_candidate, &gravity, MotorState::Running);
        let change = (accel_bias - initial_bias).norm();
        assert!(change < (bias_candidate - initial_bias).norm());
    }

    #[test]
    fn test_gravity_magnitude_preservation() {
        let mut gravity_estimate = Vector3::new(1.0, 2.0, 3.0);

        let accel_mean = Vector3::new(1.0, 2.0, 3.0);
        update_gravity_estimate(&mut gravity_estimate, &accel_mean);

        // Magnitude should be ~9.81
        let mag = gravity_estimate.norm();
        assert!((mag - 9.81).abs() < 0.1);
    }
}
