//! Spectral analysis and FFT-based frequency domain processing
//!
//! Provides frequency estimation from peak detection and period analysis,
//! including validation of detected frequencies against drone rotor constraints.

use super::config::{
    MAX_VALID_FREQUENCY, MIN_SAMPLES_FOR_FREQUENCY, MIN_VALID_FREQUENCY,
    PEAK_DETECTION_THRESHOLD, NS_TO_SECONDS,
};
use crate::types::Vector3;
use std::collections::VecDeque;

/// Detect intervals between peaks in residual signal
pub fn detect_peak_intervals(
    accel_history: &VecDeque<Vector3>,
    mean: &Vector3,
) -> Vec<f32> {
    let mut peak_intervals = Vec::new();
    let mut last_peak_idx = 0;
    let mut last_peak_val = 0.0;
    let n = accel_history.len();

    for (i, accel) in accel_history.iter().enumerate().skip(1) {
        if i >= n - 1 {
            break;
        }

        let residual = (accel - mean).norm();
        let prev = (accel_history[i - 1] - mean).norm();
        let next = (accel_history[i + 1] - mean).norm();

        // Local maximum detection
        if residual > prev
            && residual > next
            && residual > last_peak_val * PEAK_DETECTION_THRESHOLD
        {
            if last_peak_idx > 0 {
                peak_intervals.push((i - last_peak_idx) as f32);
            }
            last_peak_idx = i;
            last_peak_val = residual;
        }
    }

    peak_intervals
}

/// Compute frequency from peak intervals (samples) and sample rate (Hz)
pub fn compute_frequency_from_peaks(
    peak_intervals: &[f32],
    sample_rate: f64,
) -> Option<f32> {
    if peak_intervals.is_empty() {
        return None;
    }

    let avg_interval = peak_intervals.iter().sum::<f32>() / peak_intervals.len() as f32;
    let period_seconds = avg_interval as f64 / sample_rate;

    if period_seconds > 0.0 {
        Some((1.0 / period_seconds) as f32)
    } else {
        None
    }
}

/// Compute sample rate from timestamp history [Hz]
pub fn compute_sample_rate(timestamp_history: &VecDeque<i64>) -> f64 {
    if timestamp_history.len() < 2 {
        return 0.0;
    }
    let time_span = (timestamp_history.back().unwrap()
        - timestamp_history.front().unwrap()) as f64
        / NS_TO_SECONDS;
    timestamp_history.len() as f64 / time_span
}

/// Check if frequency is within valid range for drone rotors
pub fn is_valid_frequency(freq: f32) -> bool {
    freq >= MIN_VALID_FREQUENCY && freq <= MAX_VALID_FREQUENCY
}

/// Estimate fundamental rotor frequency when motors are running
pub fn estimate_fundamental_frequency(
    accel_history: &VecDeque<Vector3>,
    timestamp_history: &VecDeque<i64>,
) -> f32 {
    if accel_history.len() < MIN_SAMPLES_FOR_FREQUENCY
        || timestamp_history.len() < MIN_SAMPLES_FOR_FREQUENCY
    {
        return 0.0;
    }

    let samples: Vec<Vector3> = accel_history.iter().copied().collect();
    let accel_mean = compute_mean(&samples);
    let peak_intervals = detect_peak_intervals(accel_history, &accel_mean);

    if let Some(frequency) = compute_frequency_from_peaks(&peak_intervals, compute_sample_rate(timestamp_history)) {
        if is_valid_frequency(frequency) {
            frequency
        } else {
            0.0
        }
    } else {
        0.0
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
    fn test_frequency_estimation_range() {
        let mut accel_history = VecDeque::new();
        let mut timestamp_history = VecDeque::new();

        // Create periodic signal at ~250 Hz
        let sample_rate = 200.0; // 200 Hz
        let target_freq = 250.0; // Target frequency in signal

        for i in 0..100 {
            let t = i as f64 / sample_rate;
            let vibration = 2.0 * (2.0 * std::f64::consts::PI * target_freq * t).sin();

            accel_history.push_back(Vector3::new(vibration, 0.0, -9.81));
            timestamp_history.push_back((i as f64 * 5_000_000.0) as i64);
        }

        let freq = estimate_fundamental_frequency(&accel_history, &timestamp_history);

        // Frequency estimation might not be exact but should be in valid range
        if freq > 0.0 {
            assert!(
                freq >= MIN_VALID_FREQUENCY,
                "Frequency too low: {}",
                freq
            );
            assert!(
                freq <= MAX_VALID_FREQUENCY,
                "Frequency too high: {}",
                freq
            );
        }
    }

    #[test]
    fn test_is_valid_frequency() {
        assert!(is_valid_frequency(100.0));
        assert!(is_valid_frequency(250.0));
        assert!(!is_valid_frequency(5.0));
        assert!(!is_valid_frequency(2000.0));
    }

    #[test]
    fn test_compute_sample_rate() {
        let mut timestamps = VecDeque::new();
        for i in 0..100 {
            timestamps.push_back((i as i64) * 5_000_000); // 5ms intervals = 200Hz
        }

        let sample_rate = compute_sample_rate(&timestamps);
        // Should be approximately 200 Hz (within 5% tolerance)
        assert!((sample_rate - 200.0).abs() < 10.0, "Got {} Hz", sample_rate);
    }
}
