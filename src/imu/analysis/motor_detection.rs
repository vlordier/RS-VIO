//! Motor/rotor state detection and transitions
//!
//! Detects motor on/off states based on vibration magnitude with hysteresis
//! to avoid chatter during state transitions.

use super::config::{MotorState, MOTOR_OFF_HYSTERESIS};
use crate::types::Vector3;
use std::collections::VecDeque;

/// Compute RMS vibration magnitude
pub fn compute_vibration_rms(accel_history: &VecDeque<Vector3>) -> f32 {
    if accel_history.is_empty() {
        return 0.0;
    }
    let samples: Vec<Vector3> = accel_history.iter().copied().collect();
    let mean = compute_mean(&samples);

    let vibration_sum: f64 = accel_history
        .iter()
        .map(|accel| (accel - mean).norm())
        .sum();

    (vibration_sum / accel_history.len() as f64) as f32
}

/// Detect motor state based on vibration magnitude with hysteresis
pub fn detect_motor_state(
    current_state: MotorState,
    vibration_rms: f32,
    motor_on_threshold: f32,
) -> MotorState {
    let motor_off_threshold = motor_on_threshold * MOTOR_OFF_HYSTERESIS;

    match current_state {
        MotorState::Off if vibration_rms > motor_on_threshold => MotorState::Transitioning,
        MotorState::Running if vibration_rms < motor_off_threshold => MotorState::Transitioning,
        MotorState::Transitioning if vibration_rms > motor_on_threshold => MotorState::Running,
        MotorState::Transitioning if vibration_rms < motor_off_threshold => MotorState::Off,
        state => state, // No change
    }
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
    fn test_motor_state_detection() {
        let mut accel_history = VecDeque::new();

        // Low vibration (motors off)
        for _ in 0..50 {
            accel_history.push_back(Vector3::new(0.01, -0.01, -9.81));
        }

        let vibration_rms = compute_vibration_rms(&accel_history);
        assert!(vibration_rms < 0.5);

        let state = detect_motor_state(MotorState::Off, vibration_rms, 0.5);
        assert_eq!(state, MotorState::Off);

        // High vibration (motors on)
        accel_history.clear();
        for i in 0..100 {
            let vibration_x = 1.5 * ((i % 7) as f64 / 7.0 - 0.5);
            let vibration_y = 1.2 * ((i % 5) as f64 / 5.0 - 0.5);
            accel_history.push_back(Vector3::new(vibration_x, vibration_y, -9.81));
        }

        let vibration_rms = compute_vibration_rms(&accel_history);
        let state = detect_motor_state(MotorState::Off, vibration_rms, 0.5);
        assert_eq!(state, MotorState::Transitioning);
    }

    #[test]
    fn test_motor_state_hysteresis() {
        let motor_on_threshold = 0.5;
        let motor_off_threshold = motor_on_threshold * MOTOR_OFF_HYSTERESIS;

        // Vibration between thresholds
        let mid_vib = (motor_on_threshold + motor_off_threshold) / 2.0;

        // From Off state with mid vibration - should transition
        let state = detect_motor_state(MotorState::Off, motor_on_threshold + 0.1, 0.5);
        assert_eq!(state, MotorState::Transitioning);

        // From Running state with mid vibration - should remain Running
        let state = detect_motor_state(MotorState::Running, mid_vib, 0.5);
        assert_eq!(state, MotorState::Running);

        // From Running state below threshold - should transition
        let state = detect_motor_state(MotorState::Running, motor_off_threshold - 0.01, 0.5);
        assert_eq!(state, MotorState::Transitioning);
    }

    #[test]
    fn test_compute_vibration_rms() {
        let mut accel_history = VecDeque::new();

        // Add varying vibration (mean-centered)
        for i in 0..10 {
            let vibration = 1.0 * (i as f64 / 5.0 - 1.0); // Varies from -1 to 1
            accel_history.push_back(Vector3::new(vibration, 0.0, 0.0));
        }

        let rms = compute_vibration_rms(&accel_history);
        // RMS of varying signal should be non-zero
        assert!(rms > 0.0, "RMS should be non-zero for varying signal: {}", rms);
    }
}
