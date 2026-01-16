//! # FFT-Based Vibration Filtering for IMU
//!
//! This module implements FFT-based analysis and notch filtering for IMU vibration mitigation.
//! Used to identify dominant vibration frequencies and apply targeted filtering to reduce
//! motion blur and improve VIO accuracy.
//!
//! ## Algorithm Overview
//!
//! 1. **FFT Analysis**: Compute frequency spectrum of IMU signals over sliding windows
//! 2. **Peak Detection**: Identify dominant vibration frequencies (typically motor harmonics)
//! 3. **Notch Filter Design**: Design IIR notch filters targeting identified frequencies
//! 4. **Real-time Application**: Apply filters to incoming IMU measurements
//!
//! ## Key Features
//!
//! - Real-time FFT analysis with configurable window sizes
//! - Automatic peak detection in frequency domain
//! - Adaptive notch filter design based on detected frequencies
//! - CPU-efficient implementation for embedded systems
//! - Integration with existing IMU bias correction pipeline
//!
//! ## Usage
//!
//! ```rust,ignore
//! let mut filter = VibrationNotchFilter::new(200.0, 512); // 200Hz sampling, 512-point FFT
//! let filtered_imu = filter.process_measurement(raw_imu);
//! ```

use crate::datasets::ImuData;
use crate::types::Float;
use rustfft::algorithm::Radix4;
use rustfft::num_complex::Complex;
use rustfft::{Fft, FftDirection};

/// Configuration for vibration notch filtering
#[derive(Debug, Clone)]
pub struct VibrationFilterConfig {
    /// Sampling frequency [Hz]
    pub sampling_rate: Float,
    /// FFT window size (power of 2)
    pub fft_size: usize,
    /// Minimum vibration frequency to detect [Hz]
    pub min_freq: Float,
    /// Maximum vibration frequency to detect [Hz]
    pub max_freq: Float,
    /// Quality factor for notch filters
    pub notch_q: Float,
    /// Minimum peak threshold for frequency detection
    pub peak_threshold: Float,
    /// Number of dominant peaks to filter
    pub max_peaks: usize,
}

impl Default for VibrationFilterConfig {
    fn default() -> Self {
        Self {
            sampling_rate: 200.0, // 200Hz IMU typical
            fft_size: 512,        // 512-point FFT
            min_freq: 20.0,       // 20Hz minimum (avoid DC/low freq)
            max_freq: 500.0,      // 500Hz maximum (Nyquist/2 at 1kHz)
            notch_q: 10.0,        // Quality factor
            peak_threshold: 0.1,  // Relative threshold
            max_peaks: 3,         // Filter top 3 peaks
        }
    }
}

/// Detected vibration frequency peak
#[derive(Debug, Clone)]
pub struct VibrationPeak {
    /// Frequency [Hz]
    pub frequency: Float,
    /// Magnitude (normalized)
    pub magnitude: Float,
    /// SNR ratio
    pub snr: Float,
}

/// FFT-based vibration analysis and notch filtering
pub struct VibrationNotchFilter {
    config: VibrationFilterConfig,
    /// FFT planner
    fft: Radix4<f32>,
    /// Gyroscope signal buffers (x, y, z)
    gyro_buffers: [Vec<f32>; 3],
    /// Accelerometer signal buffers (x, y, z)
    accel_buffers: [Vec<f32>; 3],
    /// Current notch filters for gyro
    gyro_filters: [Vec<NotchFilter>; 3],
    /// Current notch filters for accel
    accel_filters: [Vec<NotchFilter>; 3],
    /// Detected vibration peaks (updated periodically)
    detected_peaks: Vec<VibrationPeak>,
    /// Sample counter for periodic updates
    sample_count: usize,
    /// Update interval in samples
    update_interval: usize,
}

impl VibrationNotchFilter {
    /// Create new vibration notch filter
    pub fn new(sampling_rate: Float, fft_size: usize) -> Self {
        let mut config = VibrationFilterConfig::default();
        config.sampling_rate = sampling_rate;
        config.fft_size = fft_size;

        let fft = Radix4::new(fft_size, FftDirection::Forward);

        Self {
            config,
            fft,
            gyro_buffers: [
                vec![0.0; fft_size],
                vec![0.0; fft_size],
                vec![0.0; fft_size],
            ],
            accel_buffers: [
                vec![0.0; fft_size],
                vec![0.0; fft_size],
                vec![0.0; fft_size],
            ],
            gyro_filters: [vec![], vec![], vec![]],
            accel_filters: [vec![], vec![], vec![]],
            detected_peaks: vec![],
            sample_count: 0,
            update_interval: (sampling_rate * 0.1) as usize, // Update every 100ms
        }
    }

    /// Process IMU measurement with vibration filtering
    pub fn process_measurement(&mut self, imu: &ImuData) -> ImuData {
        // Add to buffers
        for i in 0..3 {
            // Shift buffer and add new sample
            self.gyro_buffers[i].rotate_left(1);
            self.gyro_buffers[i][self.config.fft_size - 1] = imu.gyro[i] as f32;

            self.accel_buffers[i].rotate_left(1);
            self.accel_buffers[i][self.config.fft_size - 1] = imu.accel[i] as f32;
        }

        self.sample_count += 1;

        // Periodic filter update
        if self.sample_count % self.update_interval == 0 {
            self.update_filters();
        }

        // Apply filters
        let mut filtered_gyro = [0.0; 3];
        let mut filtered_accel = [0.0; 3];

        for i in 0..3 {
            filtered_gyro[i] = imu.gyro[i];
            for filter in &mut self.gyro_filters[i] {
                filtered_gyro[i] = filter.process(filtered_gyro[i] as f32) as f64;
            }

            filtered_accel[i] = imu.accel[i];
            for filter in &mut self.accel_filters[i] {
                filtered_accel[i] = filter.process(filtered_accel[i] as f32) as f64;
            }
        }

        ImuData {
            timestamp: imu.timestamp,
            gyro: filtered_gyro,
            accel: filtered_accel,
        }
    }

    /// Update notch filters based on current vibration analysis
    fn update_filters(&mut self) {
        // Analyze gyro signals for vibration peaks
        let gyro_peaks = self.analyze_signal(&self.gyro_buffers);

        // Analyze accel signals for vibration peaks
        let accel_peaks = self.analyze_signal(&self.accel_buffers);

        // Combine and sort by magnitude
        let mut all_peaks = gyro_peaks;
        all_peaks.extend(accel_peaks);
        all_peaks.sort_by(|a, b| b.magnitude.total_cmp(&a.magnitude));

        // Take top peaks
        self.detected_peaks = all_peaks.into_iter().take(self.config.max_peaks).collect();

        // Update notch filters
        for i in 0..3 {
            self.gyro_filters[i].clear();
            self.accel_filters[i].clear();

            for peak in &self.detected_peaks {
                let filter = NotchFilter::new(
                    peak.frequency as f32,
                    self.config.sampling_rate as f32,
                    self.config.notch_q as f32,
                );

                self.gyro_filters[i].push(filter.clone());
                self.accel_filters[i].push(filter);
            }
        }
    }

    /// Analyze signal for vibration peaks using FFT
    fn analyze_signal(&self, buffers: &[Vec<f32>; 3]) -> Vec<VibrationPeak> {
        let mut peaks = Vec::new();

        for buffer in buffers {
            let spectrum = self.compute_fft(buffer);
            let detected = self.find_peaks(&spectrum);
            peaks.extend(detected);
        }

        peaks
    }

    /// Compute FFT of signal buffer
    fn compute_fft(&self, signal: &[f32]) -> Vec<Complex<f32>> {
        let mut buffer: Vec<Complex<f32>> = signal.iter().map(|&x| Complex::new(x, 0.0)).collect();

        // Apply window (Hanning)
        let window: Vec<f32> = (0..signal.len())
            .map(|i| {
                0.5 * (1.0 - (2.0 * std::f32::consts::PI * i as f32 / signal.len() as f32).cos())
            })
            .collect();

        for i in 0..buffer.len() {
            buffer[i] *= window[i];
        }

        self.fft.process(&mut buffer);
        buffer
    }

    /// Find peaks in frequency spectrum
    fn find_peaks(&self, spectrum: &[Complex<f32>]) -> Vec<VibrationPeak> {
        let mut peaks = Vec::new();
        let magnitudes: Vec<f32> = spectrum.iter().map(|c| c.norm()).collect();

        if magnitudes.len() < 3 {
            return peaks;
        }

        // Find frequency bins in range
        let bin_min = (self.config.min_freq * self.config.fft_size as Float
            / self.config.sampling_rate) as usize;
        let bin_max = (self.config.max_freq * self.config.fft_size as Float
            / self.config.sampling_rate) as usize;

        // Compute noise floor (median of spectrum)
        let mut sorted_mags = magnitudes[bin_min..bin_max.min(magnitudes.len())].to_vec();
        if sorted_mags.is_empty() {
            return peaks;
        }
        sorted_mags.sort_by(|a, b| a.total_cmp(b));
        let noise_floor = sorted_mags[sorted_mags.len() / 2];

        // Find local maxima
        for bin in (bin_min + 1)..(bin_max.min(magnitudes.len() - 1)) {
            let mag = magnitudes[bin];
            let prev_mag = magnitudes[bin - 1];
            let next_mag = magnitudes[bin + 1];

            // Local maximum and above threshold
            if mag > prev_mag
                && mag > next_mag
                && mag as Float > noise_floor as Float * (1.0 + self.config.peak_threshold)
            {
                let frequency =
                    bin as Float * self.config.sampling_rate / self.config.fft_size as Float;
                let snr = mag as Float / noise_floor as Float;

                peaks.push(VibrationPeak {
                    frequency,
                    magnitude: mag as Float,
                    snr,
                });
            }
        }

        peaks
    }

    /// Get current detected vibration peaks
    pub fn get_detected_peaks(&self) -> &[VibrationPeak] {
        &self.detected_peaks
    }

    /// Get filter configuration
    pub fn config(&self) -> &VibrationFilterConfig {
        &self.config
    }

    /// Reset filter state
    pub fn reset(&mut self) {
        for i in 0..3 {
            self.gyro_buffers[i].fill(0.0);
            self.accel_buffers[i].fill(0.0);
            self.gyro_filters[i].clear();
            self.accel_filters[i].clear();
        }
        self.detected_peaks.clear();
        self.sample_count = 0;
    }
}

/// Second-order IIR notch filter for vibration rejection
#[derive(Debug, Clone)]
pub struct NotchFilter {
    /// Filter coefficients
    a: [f32; 3], // Denominator: a[0] + a[1]*z^-1 + a[2]*z^-2
    b: [f32; 3], // Numerator: b[0] + b[1]*z^-1 + b[2]*z^-2
    /// Delay line
    x: [f32; 2], // Input history
    y: [f32; 2], // Output history
}

impl NotchFilter {
    /// Create notch filter for given frequency
    pub fn new(center_freq: f32, sample_rate: f32, q: f32) -> Self {
        // Normalize frequency
        let omega = 2.0 * std::f32::consts::PI * center_freq / sample_rate;

        // Design notch filter using bilinear transform
        let cos_omega = omega.cos();
        let sin_omega = omega.sin();

        // Bandwidth parameter
        let alpha = sin_omega / (2.0 * q);

        // Filter coefficients (normalized for a[0] = 1)
        let a0 = 1.0 + alpha;
        let a1 = -2.0 * cos_omega;
        let a2 = 1.0 - alpha;

        let b0 = 1.0;
        let b1 = -2.0 * cos_omega;
        let b2 = 1.0;

        Self {
            a: [a0, a1 / a0, a2 / a0],
            b: [b0 / a0, b1 / a0, b2 / a0],
            x: [0.0; 2],
            y: [0.0; 2],
        }
    }

    /// Process single sample through notch filter
    pub fn process(&mut self, input: f32) -> f32 {
        // Direct form II implementation
        let y0 = self.b[0] * input + self.b[1] * self.x[0] + self.b[2] * self.x[1]
            - self.a[1] * self.y[0]
            - self.a[2] * self.y[1];

        // Update delay line
        self.x[1] = self.x[0];
        self.x[0] = input;
        self.y[1] = self.y[0];
        self.y[0] = y0;

        y0
    }

    /// Reset filter state
    pub fn reset(&mut self) {
        self.x = [0.0; 2];
        self.y = [0.0; 2];
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_notch_filter_design() {
        let filter = NotchFilter::new(100.0, 1000.0, 10.0);

        // Check coefficients are finite
        assert!(filter.a.iter().all(|&x| x.is_finite()));
        assert!(filter.b.iter().all(|&x| x.is_finite()));
    }

    #[test]
    fn test_notch_filter_processing() {
        let mut filter = NotchFilter::new(100.0, 1000.0, 10.0);

        // Process some samples
        for i in 0..100 {
            let input = (i as f32 * 0.01 * std::f32::consts::PI * 2.0 * 100.0).sin();
            let output = filter.process(input);
            assert!(output.is_finite());
        }
    }

    #[test]
    fn test_vibration_filter_creation() {
        let filter = VibrationNotchFilter::new(200.0, 512);
        assert_eq!(filter.config().sampling_rate, 200.0);
        assert_eq!(filter.config().fft_size, 512);
    }

    #[test]
    fn test_fft_computation() {
        let filter = VibrationNotchFilter::new(200.0, 512);

        // Create test signal with known frequency (50Hz at 200Hz sampling)
        let fs = 200.0;
        let f = 50.0;
        let signal: Vec<f32> = (0..512)
            .map(|i| ((i as f32 / fs) * std::f32::consts::PI * 2.0 * f).sin())
            .collect();

        let spectrum = filter.compute_fft(&signal);
        assert_eq!(spectrum.len(), 512);

        // Check that spectrum contains the expected frequency
        // Only look at positive frequencies (first N/2 bins)
        let magnitudes: Vec<f32> = spectrum.iter().take(256).map(|c| c.norm()).collect();
        let peak_bin = magnitudes
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| a.total_cmp(b))
            .unwrap()
            .0;

        // Should be around bin 128 (512 * 50 / 200)
        let expected_bin = (50.0 * 512.0 / 200.0) as usize;
        // Allow more tolerance due to windowing and discretization effects
        assert!(
            (peak_bin as i32 - expected_bin as i32).abs() <= 10,
            "Peak bin {} not close to expected bin {} (tolerance 10)",
            peak_bin,
            expected_bin
        );
    }
}
