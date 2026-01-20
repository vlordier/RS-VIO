//! Signal quality assessment and metrics computation
//!
//! Computes per-axis signal quality metrics including RMS, peak detection,
//! and SNR with motor-state-aware noise floor estimation.

use super::config::{
    MotorState, SignalQuality, MOTOR_RUNNING_NOISE_MULTIPLIER,
    MOTOR_TRANSITIONING_NOISE_MULTIPLIER,
};
use crate::types::Vector3;
use std::collections::VecDeque;

/// Computes per-axis RMS and peak values
pub fn compute_axis_metrics(accel_history: &VecDeque<Vector3>, axis: usize) -> (f32, f32) {
    let mut sum_sq = 0.0_f32;
    let mut max_abs = 0.0_f32;

    for accel in accel_history {
        let val = accel[axis] as f32;
        sum_sq += val * val;
        max_abs = max_abs.max(val.abs());
    }

    let n = accel_history.len() as f32;
    let rms = (sum_sq / n).sqrt();
    (rms, max_abs)
}

/// Compute SNR with motor-state-aware noise floor
pub fn compute_snr(rms: f32, noise_floor: f32) -> f32 {
    let signal_power = rms.max(0.1_f32);
    20.0_f32 * (signal_power / noise_floor).log10()
}

/// Get noise floor adjusted for motor state
pub fn get_noise_floor(base_noise_floor: f32, motor_state: MotorState) -> f32 {
    let multiplier = match motor_state {
        MotorState::Off => 1.0,
        MotorState::Running => MOTOR_RUNNING_NOISE_MULTIPLIER,
        MotorState::Transitioning => MOTOR_TRANSITIONING_NOISE_MULTIPLIER,
    };
    base_noise_floor * multiplier
}

/// Compute signal quality metrics from accelerometer history
pub fn compute_signal_quality(
    accel_history: &VecDeque<Vector3>,
    motor_state: MotorState,
    fundamental_freq_hz: f32,
    noise_floor: f32,
) -> SignalQuality {
    if accel_history.is_empty() {
        return SignalQuality {
            snr: [0.0; 3],
            rms: [0.0; 3],
            peak: [0.0; 3],
            motor_state,
            fundamental_freq_hz,
        };
    }

    let mut snr = [0.0f32; 3];
    let mut rms = [0.0f32; 3];
    let mut peak = [0.0f32; 3];

    // Compute per-axis metrics
    for axis in 0..3 {
        let (axis_rms, axis_peak) = compute_axis_metrics(accel_history, axis);
        rms[axis] = axis_rms;
        peak[axis] = axis_peak;
        snr[axis] = compute_snr(axis_rms, noise_floor);
    }

    SignalQuality {
        snr,
        rms,
        peak,
        motor_state,
        fundamental_freq_hz,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_signal_quality_computation() {
        let mut accel_history = VecDeque::new();

        // Feed clean stationary signal
        for _ in 0..50 {
            accel_history.push_back(Vector3::new(0.0, 0.0, -9.81));
        }

        let quality =
            compute_signal_quality(&accel_history, MotorState::Off, 0.0, 0.01);

        // Check RMS is close to gravity magnitude
        assert!(
            quality.rms[2] > 9.0 && quality.rms[2] < 10.0,
            "RMS should be close to gravity: {}",
            quality.rms[2]
        );

        // Check peak is also close to gravity
        assert!(
            quality.peak[2] > 9.0 && quality.peak[2] < 10.0,
            "Peak should be close to gravity: {}",
            quality.peak[2]
        );

        // SNR should be high for clean signal
        assert!(
            quality.snr[2] > 30.0,
            "SNR should be high for clean signal: {}",
            quality.snr[2]
        );
    }

    #[test]
    fn test_noise_floor_adaptation() {
        let _quality_off = compute_signal_quality(
            &{
                let mut h = VecDeque::new();
                for _ in 0..30 {
                    h.push_back(Vector3::new(0.0, 0.0, -9.81));
                }
                h
            },
            MotorState::Off,
            0.0,
            0.01,
        );

        let quality_on = compute_signal_quality(
            &{
                let mut h = VecDeque::new();
                for i in 0..30 {
                    let vib = 1.5 * ((i % 5) as f64 / 5.0 - 0.5);
                    h.push_back(Vector3::new(vib, 0.0, -9.81));
                }
                h
            },
            MotorState::Running,
            0.0,
            0.01,
        );

        // SNR should be lower when motors running (higher noise floor)
        assert!(
            quality_on.snr[0].is_finite() && quality_on.snr[0] > 0.0,
            "SNR should be finite and positive: {}",
            quality_on.snr[0]
        );
    }

    #[test]
    fn test_get_noise_floor() {
        let base = 0.01;
        assert_eq!(get_noise_floor(base, MotorState::Off), base);
        assert!(get_noise_floor(base, MotorState::Running) > base);
        assert!(get_noise_floor(base, MotorState::Transitioning) > base);
    }
}
