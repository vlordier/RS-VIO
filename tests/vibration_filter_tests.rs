//! Comprehensive tests for FFT-based vibration filtering
//! Tests happy paths, edge cases, error conditions, and performance

use rs_vio::datasets::ImuData;
use rs_vio::imu::vibration_filter::{VibrationFilterConfig, VibrationNotchFilter, VibrationPeak};

/// Test basic vibration filter creation and configuration
#[test]
fn test_vibration_filter_creation() {
    let config = VibrationFilterConfig {
        sampling_rate: 200.0,
        fft_size: 512,
        min_freq: 20.0,
        max_freq: 500.0,
        notch_q: 10.0,
        peak_threshold: 0.1,
        max_peaks: 3,
    };

    let filter = VibrationNotchFilter::new(config.sampling_rate, config.fft_size);
    assert_eq!(filter.config().sampling_rate, 200.0);
    assert_eq!(filter.config().fft_size, 512);
}

/// Test vibration filter with normal IMU data (happy path)
#[test]
fn test_vibration_filter_normal_operation() {
    let mut filter = VibrationNotchFilter::new(200.0, 512);

    // Generate normal IMU data with some vibration
    let imu_data = generate_test_imu_data(100, 0.01); // 100 samples, 0.01 rad/s vibration

    for imu in imu_data {
        let filtered = filter.process_measurement(&imu);
        // Filtered data should be similar to input (no extreme changes)
        assert!((filtered.gyro[0] - imu.gyro[0]).abs() < 0.1);
        assert!((filtered.accel[0] - imu.accel[0]).abs() < 1.0);
    }

    // Check that peaks are detected
    let peaks = filter.get_detected_peaks();
    assert!(!peaks.is_empty()); // Should detect some peaks
}

/// Test vibration filter with empty/noisy data
#[test]
fn test_vibration_filter_edge_cases() {
    let mut filter = VibrationNotchFilter::new(200.0, 512);

    // Test with constant zero data
    let zero_imu = ImuData {
        timestamp: 0,
        gyro: [0.0; 3],
        accel: [0.0; 3],
    };

    for _ in 0..100 {
        let filtered = filter.process_measurement(&zero_imu);
        assert_eq!(filtered.gyro, [0.0; 3]);
        assert_eq!(filtered.accel, [0.0; 3]);
    }

    // Test with extreme noise
    let noisy_imu = ImuData {
        timestamp: 1000000,
        gyro: [10.0, -5.0, 20.0], // Very noisy
        accel: [50.0, -30.0, 100.0],
    };

    let filtered = filter.process_measurement(&noisy_imu);
    // Should still produce finite outputs
    for &g in &filtered.gyro {
        assert!(g.is_finite());
    }
    for &a in &filtered.accel {
        assert!(a.is_finite());
    }
}

/// Test vibration filter reset functionality
#[test]
fn test_vibration_filter_reset() {
    let mut filter = VibrationNotchFilter::new(200.0, 512);

    // Process some data
    let imu_data = generate_test_imu_data(50, 0.1);
    for imu in imu_data {
        let _ = filter.process_measurement(&imu);
    }

    // Should have detected peaks
    assert!(!filter.get_detected_peaks().is_empty());

    // Reset
    filter.reset();

    // Should be empty after reset
    assert!(filter.get_detected_peaks().is_empty());

    // Process data again - should work normally
    let imu_data2 = generate_test_imu_data(50, 0.05);
    for imu in imu_data2 {
        let filtered = filter.process_measurement(&imu);
        assert!(filtered.gyro[0].is_finite());
        assert!(filtered.accel[0].is_finite());
    }
}

/// Test vibration filter with insufficient data
#[test]
fn test_vibration_filter_insufficient_data() {
    let mut filter = VibrationNotchFilter::new(200.0, 512);

    // Process only a few samples (less than FFT window)
    let imu_data = generate_test_imu_data(10, 0.1);
    for imu in imu_data {
        let _ = filter.process_measurement(&imu);
    }

    // Should still work but with no peaks detected yet
    let peaks = filter.get_detected_peaks();
    // Might have peaks or not depending on implementation
    assert!(peaks.len() <= filter.config().max_peaks);
}

/// Test vibration filter with high vibration scenarios
#[test]
fn test_vibration_filter_high_vibration() {
    let mut filter = VibrationNotchFilter::new(200.0, 512);

    // Generate high vibration data
    let imu_data = generate_test_imu_data(200, 1.0); // High amplitude

    for imu in imu_data {
        let _ = filter.process_measurement(&imu);
    }

    let peaks = filter.get_detected_peaks();
    // Should detect significant peaks
    assert!(!peaks.is_empty());

    // Check that peaks are in valid frequency range
    for peak in peaks {
        assert!(peak.frequency >= filter.config().min_freq);
        assert!(peak.frequency <= filter.config().max_freq);
        assert!(peak.magnitude >= 0.0);
        assert!(peak.snr >= 0.0);
    }
}

/// Test vibration filter with multiple vibration frequencies
#[test]
fn test_vibration_filter_multiple_frequencies() {
    let mut filter = VibrationNotchFilter::new(200.0, 512);

    // Generate data with multiple frequency components
    let imu_data = generate_multitone_imu_data(300, &[25.0, 50.0, 75.0], &[0.1, 0.15, 0.2]);

    for imu in imu_data {
        let _ = filter.process_measurement(&imu);
    }

    let peaks = filter.get_detected_peaks();
    // Should detect multiple peaks
    assert!(peaks.len() >= 2);

    // Peaks should be at or near the input frequencies
    let detected_freqs: Vec<f64> = peaks.iter().map(|p| p.frequency).collect();
    assert!(detected_freqs.iter().any(|&f| (f - 25.0).abs() < 5.0));
    assert!(detected_freqs.iter().any(|&f| (f - 50.0).abs() < 5.0));
}

/// Test vibration filter performance under stress
#[test]
fn test_vibration_filter_performance() {
    let mut filter = VibrationNotchFilter::new(200.0, 512);

    let imu_data = generate_test_imu_data(1000, 0.1);

    let start = std::time::Instant::now();
    for imu in imu_data {
        let _ = filter.process_measurement(&imu);
    }
    let duration = start.elapsed();

    // Should process 1000 samples in reasonable time
    // At 200Hz, this is 5 seconds of data
    assert!(duration.as_millis() < 500); // Should be much faster than real-time
}

/// Test vibration filter with different sampling rates
#[test]
fn test_vibration_filter_different_rates() {
    let rates = [100.0, 200.0, 400.0, 1000.0];

    for &rate in &rates {
        let mut filter = VibrationNotchFilter::new(rate, 256);
        let imu_data = generate_test_imu_data_at_rate(100, 0.1, rate);

        for imu in imu_data {
            let filtered = filter.process_measurement(&imu);
            assert!(filtered.gyro[0].is_finite());
            assert!(filtered.accel[0].is_finite());
        }

        // Check that frequency ranges are adjusted for sampling rate
        let nyquist = rate / 2.0;
        assert!(filter.config().max_freq <= nyquist);
    }
}

// Helper functions for test data generation

fn generate_test_imu_data(num_samples: usize, vibration_amplitude: f64) -> Vec<ImuData> {
    (0..num_samples)
        .map(|i| {
            let t = i as f64 * 0.005; // 200Hz sampling
            let vibration = vibration_amplitude * (t * 50.0 * 2.0 * std::f64::consts::PI).sin();

            ImuData {
                timestamp: (i as i64) * 5000, // 5ms intervals
                gyro: [vibration, vibration * 0.5, vibration * 0.3],
                accel: [vibration * 10.0, vibration * 5.0, 9.81 + vibration * 2.0],
            }
        })
        .collect()
}

fn generate_test_imu_data_at_rate(
    num_samples: usize,
    vibration_amplitude: f64,
    rate: f64,
) -> Vec<ImuData> {
    let dt = 1.0 / rate;
    (0..num_samples)
        .map(|i| {
            let t = i as f64 * dt;
            let vibration = vibration_amplitude * (t * 50.0 * 2.0 * std::f64::consts::PI).sin();

            ImuData {
                timestamp: (i as i64) * (dt * 1e9) as i64,
                gyro: [vibration, vibration * 0.5, vibration * 0.3],
                accel: [vibration * 10.0, vibration * 5.0, 9.81 + vibration * 2.0],
            }
        })
        .collect()
}

fn generate_multitone_imu_data(
    num_samples: usize,
    frequencies: &[f64],
    amplitudes: &[f64],
) -> Vec<ImuData> {
    assert_eq!(frequencies.len(), amplitudes.len());

    (0..num_samples)
        .map(|i| {
            let t = i as f64 * 0.005; // 200Hz sampling
            let mut vibration = 0.0;

            for (&freq, &amp) in frequencies.iter().zip(amplitudes.iter()) {
                vibration += amp * (t * freq * 2.0 * std::f64::consts::PI).sin();
            }

            ImuData {
                timestamp: (i as i64) * 5000,
                gyro: [vibration, vibration * 0.5, vibration * 0.3],
                accel: [vibration * 10.0, vibration * 5.0, 9.81 + vibration * 2.0],
            }
        })
        .collect()
}

/// Test VibrationPeak structure
#[test]
fn test_vibration_peak_structure() {
    let peak = VibrationPeak {
        frequency: 100.0,
        magnitude: 2.5,
        snr: 15.0,
    };

    assert_eq!(peak.frequency, 100.0);
    assert_eq!(peak.magnitude, 2.5);
    assert_eq!(peak.snr, 15.0);
}

/// Test vibration filter configuration validation
#[test]
fn test_vibration_filter_config_validation() {
    // Test valid config
    let config = VibrationFilterConfig::default();
    assert!(config.sampling_rate > 0.0);
    assert!(config.fft_size > 0);
    assert!(config.min_freq < config.max_freq);
    assert!(config.notch_q > 0.0);
    assert!(config.peak_threshold >= 0.0);

    // Test FFT size is power of 2
    let mut fft_size = config.fft_size;
    assert!(fft_size > 0);
    while fft_size > 1 {
        assert_eq!(fft_size % 2, 0);
        fft_size /= 2;
    }
}
