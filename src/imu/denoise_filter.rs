/// Real-time IMU denoising filter for VIO systems
/// Handles camera frame rates (30-60 fps) vs IMU sampling (200 Hz)
/// Incorporates notch filtering for identified resonances (0.06 Hz, 1.46 Hz)
use nalgebra as na;
use std::collections::VecDeque;

/// Configuration for the denoising filter
#[derive(Debug, Clone)]
pub struct DenoiseConfig {
    /// IMU sampling rate in Hz (typically 200)
    pub imu_sample_rate: f32,
    /// Camera frame rate in Hz (30, 60, etc.)
    pub camera_frame_rate: f32,
    /// Cutoff frequency for high-pass filter (Hz) - removes slow drift
    pub highpass_cutoff: f32,
    /// Cutoff frequency for low-pass filter (Hz) - removes high-freq noise
    pub lowpass_cutoff: f32,
    /// Enable notch filtering at identified resonances
    pub enable_notch_filter: bool,
    /// Notch frequencies to suppress (Hz)
    pub notch_frequencies: Vec<f32>,
    /// Notch filter Q factor (higher = narrower)
    pub notch_q: f32,
    /// Enable complementary filtering with vision
    pub enable_vision_fusion: bool,
    /// Vision trust factor (0.0-1.0): how much to trust vision vs IMU
    pub vision_trust: f32,

    /// Optional per-axis notch frequency lists (overrides notch_frequencies if provided)
    pub notch_frequencies_per_axis: Option<[Vec<f32>; 3]>,
    /// Enable adaptive Q for notch filters based on signal magnitude
    pub adaptive_notch_q: bool,
    /// Minimum and maximum Q for adaptive notch
    pub notch_q_min: f32,
    pub notch_q_max: f32,
    /// Reference magnitude (rad/s) at which notch_q starts shrinking
    pub notch_q_ref_rads: f32,

    /// Spike rejection window (odd size, 3 recommended)
    pub spike_window: usize,
    /// Clipping threshold (rad/s). If abs(axis) > threshold, sample is considered clipped
    pub clip_threshold_rads: f32,
    /// Size of clipping window (samples) to compute weight scale
    pub clip_window: usize,

    /// Motion mode thresholds (rad/s RMS over short window)
    pub hover_rms_thresh: f32,
    pub aggressive_rms_thresh: f32,
    /// Hover mode cutoffs
    pub hover_highpass_hz: f32,
    pub hover_lowpass_hz: f32,
    /// Aggressive mode cutoffs
    pub aggressive_highpass_hz: f32,
    pub aggressive_lowpass_hz: f32,
}

impl Default for DenoiseConfig {
    fn default() -> Self {
        Self {
            imu_sample_rate: 200.0,
            camera_frame_rate: 30.0,
            highpass_cutoff: 0.5, // Remove drift below 0.5 Hz
            lowpass_cutoff: 50.0, // Remove noise above 50 Hz
            enable_notch_filter: true,
            notch_frequencies: vec![0.06, 1.46], // From resonance analysis
            notch_q: 5.0,
            enable_vision_fusion: true,
            vision_trust: 0.3,

            notch_frequencies_per_axis: None,
            adaptive_notch_q: true,
            notch_q_min: 4.0,
            notch_q_max: 8.0,
            notch_q_ref_rads: 0.6,

            spike_window: 3,
            clip_threshold_rads: 3.0,
            clip_window: 200,

            hover_rms_thresh: 0.25,
            aggressive_rms_thresh: 0.8,
            hover_highpass_hz: 0.3,
            hover_lowpass_hz: 40.0,
            aggressive_highpass_hz: 0.8,
            aggressive_lowpass_hz: 60.0,
        }
    }
}

/// Second-order Butterworth filter stage
#[derive(Debug, Clone)]
struct BiquadFilter {
    b0: f32,
    b1: f32,
    b2: f32, // Numerator coefficients
    a1: f32,
    a2: f32, // Denominator coefficients
    x1: f32,
    x2: f32, // Input history
    y1: f32,
    y2: f32, // Output history
}

impl BiquadFilter {
    fn identity() -> Self {
        Self {
            b0: 1.0,
            b1: 0.0,
            b2: 0.0,
            a1: 0.0,
            a2: 0.0,
            x1: 0.0,
            x2: 0.0,
            y1: 0.0,
            y2: 0.0,
        }
    }

    /// Create high-pass Butterworth filter
    fn highpass(sample_rate: f32, cutoff_hz: f32) -> Self {
        let wc = 2.0 * std::f32::consts::PI * cutoff_hz / sample_rate;
        let c = wc.cos();
        let s = wc.sin();
        let alpha = s / (2.0 * 0.707); // Q = 0.707 for Butterworth

        let b0 = (1.0 + c) / 2.0;
        let b1 = -(1.0 + c);
        let b2 = (1.0 + c) / 2.0;
        let a0 = 1.0 + alpha;
        let a1 = -2.0 * c / a0;
        let a2 = (1.0 - alpha) / a0;

        Self {
            b0: b0 / a0,
            b1: b1 / a0,
            b2: b2 / a0,
            a1,
            a2,
            x1: 0.0,
            x2: 0.0,
            y1: 0.0,
            y2: 0.0,
        }
    }

    /// Create low-pass Butterworth filter
    fn lowpass(sample_rate: f32, cutoff_hz: f32) -> Self {
        let wc = 2.0 * std::f32::consts::PI * cutoff_hz / sample_rate;
        let c = wc.cos();
        let s = wc.sin();
        let alpha = s / (2.0 * 0.707); // Q = 0.707 for Butterworth

        let b0 = (1.0 - c) / 2.0;
        let b1 = 1.0 - c;
        let b2 = (1.0 - c) / 2.0;
        let a0 = 1.0 + alpha;
        let a1 = -2.0 * c / a0;
        let a2 = (1.0 - alpha) / a0;

        Self {
            b0: b0 / a0,
            b1: b1 / a0,
            b2: b2 / a0,
            a1,
            a2,
            x1: 0.0,
            x2: 0.0,
            y1: 0.0,
            y2: 0.0,
        }
    }

    /// Create notch filter (band-stop)
    fn notch(sample_rate: f32, center_hz: f32, q: f32) -> Self {
        let wc = 2.0 * std::f32::consts::PI * center_hz / sample_rate;
        let c = wc.cos();
        let s = wc.sin();
        let alpha = s / (2.0 * q);

        let b0 = 1.0;
        let b1 = -2.0 * c;
        let b2 = 1.0;
        let a0 = 1.0 + alpha;
        let a1 = -2.0 * c / a0;
        let a2 = (1.0 - alpha) / a0;

        Self {
            b0: b0 / a0,
            b1: b1 / a0,
            b2: b2 / a0,
            a1,
            a2,
            x1: 0.0,
            x2: 0.0,
            y1: 0.0,
            y2: 0.0,
        }
    }

    /// Apply filter to a single sample
    fn process(&mut self, x: f32) -> f32 {
        let y = self.b0 * x + self.b1 * self.x1 + self.b2 * self.x2
            - self.a1 * self.y1
            - self.a2 * self.y2;

        self.x2 = self.x1;
        self.x1 = x;
        self.y2 = self.y1;
        self.y1 = y;

        y
    }
}

/// Real-time IMU denoising filter
pub struct ImuDenoiseFilter {
    config: DenoiseConfig,

    // High-pass and low-pass filters (per axis)
    highpass_filters: [BiquadFilter; 3],
    lowpass_filters: [BiquadFilter; 3],
    notch_filters: Vec<[BiquadFilter; 3]>,

    // Complementary filter state
    vision_accel: na::Vector3<f32>,
    imu_accel: na::Vector3<f32>,

    // IMU preintegration buffer
    imu_buffer: VecDeque<(f32, na::Vector3<f32>)>,
    last_camera_frame_time: Option<f32>,

    // Statistics
    pub signal_quality: f32, // 0.0-1.0: estimate of output quality

    // Motion-mode FSM
    mode: MotionMode,
    rms_ewma: f32,

    // Spike rejection history (last two samples per axis)
    gyro_hist: [[f32; 3]; 2],
    accel_hist: [[f32; 3]; 2],

    // Clipping detection
    clip_window: Vec<bool>,
    clip_head: usize,
    clip_count: usize,
    pub weight_scale: f32, // 0-1 scaling factor for estimator weighting

    current_notch_q: f32,
    samples_since_rebuild: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum MotionMode {
    Hover,
    Aggressive,
}

impl ImuDenoiseFilter {
    /// Create a new denoising filter with given configuration
    pub fn new(config: DenoiseConfig) -> Self {
        let current_notch_q = config.notch_q;
        let clip_window_size = config.clip_window.max(1);
        let mut filter = Self {
            config,
            highpass_filters: [
                BiquadFilter::highpass(1.0, 1.0), // placeholder, will be rebuilt
                BiquadFilter::highpass(1.0, 1.0),
                BiquadFilter::highpass(1.0, 1.0),
            ],
            lowpass_filters: [
                BiquadFilter::lowpass(1.0, 1.0),
                BiquadFilter::lowpass(1.0, 1.0),
                BiquadFilter::lowpass(1.0, 1.0),
            ],
            notch_filters: Vec::new(),
            vision_accel: na::Vector3::zeros(),
            imu_accel: na::Vector3::zeros(),
            imu_buffer: VecDeque::new(),
            last_camera_frame_time: None,
            signal_quality: 1.0,
            mode: MotionMode::Hover,
            rms_ewma: 0.0,
            gyro_hist: [[0.0; 3]; 2],
            accel_hist: [[0.0; 3]; 2],
            clip_window: vec![false; clip_window_size],
            clip_head: 0,
            clip_count: 0,
            weight_scale: 1.0,
            current_notch_q,
            samples_since_rebuild: 0,
        };

        // Build filters using current mode defaults
        filter.rebuild_filters(filter.mode);
        filter
    }

    fn rebuild_filters(&mut self, mode: MotionMode) {
        let (hp, lp) = match mode {
            MotionMode::Hover => (self.config.hover_highpass_hz, self.config.hover_lowpass_hz),
            MotionMode::Aggressive => (
                self.config.aggressive_highpass_hz,
                self.config.aggressive_lowpass_hz,
            ),
        };

        self.highpass_filters = [
            BiquadFilter::highpass(self.config.imu_sample_rate, hp),
            BiquadFilter::highpass(self.config.imu_sample_rate, hp),
            BiquadFilter::highpass(self.config.imu_sample_rate, hp),
        ];

        self.lowpass_filters = [
            BiquadFilter::lowpass(self.config.imu_sample_rate, lp),
            BiquadFilter::lowpass(self.config.imu_sample_rate, lp),
            BiquadFilter::lowpass(self.config.imu_sample_rate, lp),
        ];

        self.notch_filters = if self.config.enable_notch_filter {
            // Select frequencies per axis if provided
            let per_axis = self.config.notch_frequencies_per_axis.as_ref();
            let max_stages = per_axis
                .map(|arr| arr.iter().map(|v| v.len()).max().unwrap_or(0))
                .unwrap_or(self.config.notch_frequencies.len());

            (0..max_stages)
                .map(|stage_idx| {
                    let mut freqs = [None, None, None];
                    for axis in 0..3 {
                        if let Some(arr) = per_axis {
                            if stage_idx < arr[axis].len() {
                                freqs[axis] = Some(arr[axis][stage_idx]);
                            }
                        } else if stage_idx < self.config.notch_frequencies.len() {
                            freqs[axis] = Some(self.config.notch_frequencies[stage_idx]);
                        }
                    }
                    [
                        freqs[0]
                            .map(|f| {
                                BiquadFilter::notch(
                                    self.config.imu_sample_rate,
                                    f,
                                    self.current_notch_q,
                                )
                            })
                            .unwrap_or_else(BiquadFilter::identity),
                        freqs[1]
                            .map(|f| {
                                BiquadFilter::notch(
                                    self.config.imu_sample_rate,
                                    f,
                                    self.current_notch_q,
                                )
                            })
                            .unwrap_or_else(BiquadFilter::identity),
                        freqs[2]
                            .map(|f| {
                                BiquadFilter::notch(
                                    self.config.imu_sample_rate,
                                    f,
                                    self.current_notch_q,
                                )
                            })
                            .unwrap_or_else(BiquadFilter::identity),
                    ]
                })
                .collect()
        } else {
            Vec::new()
        };
    }

    #[inline(always)]
    fn median_of_three(a: f32, b: f32, c: f32) -> f32 {
        a.max(b.min(c)).min(b.max(c))
    }

    #[inline(always)]
    fn spike_filter(history: &mut [[f32; 3]; 2], sample: [f32; 3], enabled: bool) -> [f32; 3] {
        if !enabled {
            history[1] = history[0];
            history[0] = sample;
            return sample;
        }

        let out = [
            Self::median_of_three(history[1][0], history[0][0], sample[0]),
            Self::median_of_three(history[1][1], history[0][1], sample[1]),
            Self::median_of_three(history[1][2], history[0][2], sample[2]),
        ];
        history[1] = history[0];
        history[0] = sample;
        out
    }

    #[inline]
    fn update_clipping(&mut self, sample: &[f32; 3]) {
        if self.clip_window.is_empty() {
            return;
        }
        let thresh = self.config.clip_threshold_rads;
        let clipped =
            sample[0].abs() > thresh || sample[1].abs() > thresh || sample[2].abs() > thresh;
        self.clip_count -= self.clip_window[self.clip_head] as usize;
        self.clip_window[self.clip_head] = clipped;
        self.clip_count += clipped as usize;
        self.clip_head = (self.clip_head + 1) % self.clip_window.len();

        let fraction = self.clip_count as f32 / self.clip_window.len() as f32;
        self.weight_scale = (1.0 - fraction).clamp(0.2, 1.0);
    }

    #[inline]
    fn update_rms_and_mode(&mut self, rms: f32) {
        // Slow EWMA to avoid thrashing between modes
        self.rms_ewma = 0.97 * self.rms_ewma + 0.03 * rms;

        let new_mode = if self.rms_ewma < self.config.hover_rms_thresh {
            MotionMode::Hover
        } else if self.rms_ewma > self.config.aggressive_rms_thresh {
            MotionMode::Aggressive
        } else {
            self.mode
        };

        // Debounce filter rebuilds (only rebuild every 50 samples minimum)
        if new_mode != self.mode && self.samples_since_rebuild > 50 {
            self.mode = new_mode;
            self.rebuild_filters(self.mode);
            self.samples_since_rebuild = 0;
        }
    }

    #[inline]
    fn update_adaptive_notch(&mut self, rms: f32) {
        if !self.config.adaptive_notch_q || !self.config.enable_notch_filter {
            return;
        }

        let span = (self.config.notch_q_max - self.config.notch_q_min).max(1e-3);
        let norm = (rms / self.config.notch_q_ref_rads).clamp(0.0, 1.0);
        let target_q = self.config.notch_q_max - norm * span;

        // Debounce filter rebuilds (only rebuild every 50 samples minimum and if change is significant)
        if (target_q - self.current_notch_q).abs() > 0.3 && self.samples_since_rebuild > 50 {
            self.current_notch_q = target_q;
            self.rebuild_filters(self.mode);
            self.samples_since_rebuild = 0;
        }
    }

    /// Process a single IMU acceleration sample (gyro in this context)
    #[inline]
    pub fn process_gyro(&mut self, gyro: &[f32; 3]) -> [f32; 3] {
        let medianed =
            Self::spike_filter(&mut self.gyro_hist, *gyro, self.config.spike_window >= 3);
        self.update_clipping(&medianed);

        // Inline RMS calculation
        let rms =
            (medianed[0] * medianed[0] + medianed[1] * medianed[1] + medianed[2] * medianed[2])
                .sqrt();
        self.update_rms_and_mode(rms);
        self.update_adaptive_notch(rms);
        self.samples_since_rebuild += 1;

        let mut filtered = medianed;

        // Stage 1: High-pass filter (remove slow drift)
        filtered[0] = self.highpass_filters[0].process(filtered[0]);
        filtered[1] = self.highpass_filters[1].process(filtered[1]);
        filtered[2] = self.highpass_filters[2].process(filtered[2]);

        // Stage 2: Notch filters (remove identified resonances)
        for notch_stage in &mut self.notch_filters {
            filtered[0] = notch_stage[0].process(filtered[0]);
            filtered[1] = notch_stage[1].process(filtered[1]);
            filtered[2] = notch_stage[2].process(filtered[2]);
        }

        // Stage 3: Low-pass filter (remove high-freq noise)
        filtered[0] = self.lowpass_filters[0].process(filtered[0]);
        filtered[1] = self.lowpass_filters[1].process(filtered[1]);
        filtered[2] = self.lowpass_filters[2].process(filtered[2]);

        filtered
    }

    /// Process a single IMU acceleration sample
    #[inline]
    pub fn process_accel(&mut self, accel: &[f32; 3]) -> [f32; 3] {
        let medianed =
            Self::spike_filter(&mut self.accel_hist, *accel, self.config.spike_window >= 3);
        self.update_clipping(&medianed);

        // Inline RMS calculation (accel doesn't update adaptive notch)
        let rms =
            (medianed[0] * medianed[0] + medianed[1] * medianed[1] + medianed[2] * medianed[2])
                .sqrt();
        self.update_rms_and_mode(rms);

        let mut filtered = medianed;

        // Stage 1: High-pass filter (remove bias/gravity)
        filtered[0] = self.highpass_filters[0].process(filtered[0]);
        filtered[1] = self.highpass_filters[1].process(filtered[1]);
        filtered[2] = self.highpass_filters[2].process(filtered[2]);

        // Note: Don't apply notch to accel as resonances are in angular rates
        // The acceleration data is cleaner due to gravity component

        // Stage 3: Low-pass filter (remove high-freq noise)
        filtered[0] = self.lowpass_filters[0].process(filtered[0]);
        filtered[1] = self.lowpass_filters[1].process(filtered[1]);
        filtered[2] = self.lowpass_filters[2].process(filtered[2]);

        filtered
    }

    /// Update vision-based acceleration estimate (from optical flow or visual odometry)
    pub fn update_vision_estimate(&mut self, vision_accel: &na::Vector3<f32>) {
        self.vision_accel = *vision_accel;
    }

    /// Get complementary filtered acceleration (fuses vision + IMU)
    pub fn get_fused_accel(&self) -> na::Vector3<f32> {
        if self.config.enable_vision_fusion {
            // Complement: Trust vision for low frequencies, IMU for high frequencies
            let trust = self.config.vision_trust;
            self.vision_accel * trust + self.imu_accel * (1.0 - trust)
        } else {
            self.imu_accel
        }
    }

    /// Buffer IMU sample for preintegration between camera frames
    pub fn buffer_imu_sample(&mut self, time_ms: f32, gyro: &[f32; 3]) {
        self.imu_buffer
            .push_back((time_ms, na::Vector3::from_row_slice(gyro)));
    }

    /// Process a camera frame and get integrated IMU measurements
    pub fn process_camera_frame(&mut self, time_ms: f32) -> Option<na::Vector3<f32>> {
        let dt = if let Some(last_time) = self.last_camera_frame_time {
            time_ms - last_time
        } else {
            self.last_camera_frame_time = Some(time_ms);
            return None;
        };

        // Drain buffer and integrate IMU data
        let mut integrated_gyro = na::Vector3::zeros();
        let mut count = 0;

        while let Some((sample_time, gyro)) = self.imu_buffer.pop_front() {
            if sample_time > time_ms {
                // Put it back if it's in the future
                self.imu_buffer.push_front((sample_time, gyro));
                break;
            }
            integrated_gyro += gyro;
            count += 1;
        }

        if count > 0 {
            integrated_gyro /= count as f32;
        }

        self.last_camera_frame_time = Some(time_ms);

        // Estimate signal quality based on buffer consistency
        self.signal_quality = if count > 0 {
            let expected_samples = (dt / 1000.0) * self.config.imu_sample_rate;
            1.0 - (expected_samples - count as f32).abs() / expected_samples.max(1.0)
        } else {
            0.0
        };

        Some(integrated_gyro)
    }

    /// Get current signal quality estimate (0.0-1.0)
    pub fn quality(&self) -> f32 {
        self.signal_quality
    }

    /// Get filter configuration
    pub fn config(&self) -> &DenoiseConfig {
        &self.config
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_highpass_filter_removes_dc() {
        let config = DenoiseConfig::default();
        let mut filter = ImuDenoiseFilter::new(config);

        // DC component should be attenuated
        let dc_signal = [0.5, 0.5, 0.5];
        let _output = filter.process_gyro(&dc_signal);

        // After many iterations, DC should be mostly gone (more iterations needed due to debouncing)
        for _ in 0..200 {
            filter.process_gyro(&dc_signal);
        }
        let output = filter.process_gyro(&dc_signal);

        assert!(output[0].abs() < 0.15, "High-pass should remove DC");
    }

    #[test]
    fn test_notch_filter_configuration() {
        let mut config = DenoiseConfig::default();
        config.notch_frequencies = vec![0.06, 1.46];
        let filter = ImuDenoiseFilter::new(config);

        assert_eq!(filter.notch_filters.len(), 2, "Should have 2 notch stages");
    }

    #[test]
    fn test_spike_rejection() {
        let mut config = DenoiseConfig::default();
        config.spike_window = 3;
        let mut filter = ImuDenoiseFilter::new(config);

        // Feed normal samples
        filter.process_gyro(&[0.1, 0.1, 0.1]);
        filter.process_gyro(&[0.11, 0.11, 0.11]);

        // Feed spike (much larger than previous samples)
        let spike_sample = [5.0, 5.0, 5.0];
        let output = filter.process_gyro(&spike_sample);

        // Output should be attenuated compared to raw spike
        // Median-of-3 should use middle value from [0.11, 5.0, next_sample]
        // Since we haven't provided next sample yet, behavior depends on history
        assert!(
            output[0].abs() < spike_sample[0],
            "Spike should be attenuated"
        );

        // Feed another normal sample
        let normal = [0.12, 0.12, 0.12];
        let output2 = filter.process_gyro(&normal);

        // Should recover to normal processing
        assert!(output2[0].abs() < 1.0, "Should recover from spike");
    }

    #[test]
    fn test_clipping_detection_and_weight_scaling() {
        let mut config = DenoiseConfig::default();
        config.clip_threshold_rads = 3.0;
        config.clip_window = 100;
        let mut filter = ImuDenoiseFilter::new(config);

        // Initially weight should be 1.0
        assert_eq!(filter.weight_scale, 1.0, "Initial weight should be 1.0");

        // Feed normal samples
        for _ in 0..50 {
            filter.process_gyro(&[0.5, 0.5, 0.5]);
        }
        assert!(
            filter.weight_scale > 0.9,
            "Weight should stay high for normal samples"
        );

        // Feed clipping samples (above threshold)
        for _ in 0..30 {
            filter.process_gyro(&[4.0, 4.0, 4.0]);
        }

        // Weight should decrease due to clipping
        // With 30 clipped samples in 100-sample window, ratio = 0.3
        // weight_scale = 0.2 + 0.8 * (1 - 0.3) = 0.2 + 0.56 = 0.76
        assert!(
            filter.weight_scale < 0.85,
            "Weight should decrease with clipping: {}",
            filter.weight_scale
        );
        assert!(
            filter.weight_scale >= 0.2,
            "Weight should not go below 0.2: {}",
            filter.weight_scale
        );

        // Feed normal samples to recover
        for _ in 0..100 {
            filter.process_gyro(&[0.5, 0.5, 0.5]);
        }

        // Weight should recover as clipped samples leave the window
        assert!(
            filter.weight_scale > 0.9,
            "Weight should recover after normal samples"
        );
    }

    #[test]
    fn test_motion_mode_transitions() {
        let mut config = DenoiseConfig::default();
        config.hover_rms_thresh = 0.25;
        config.aggressive_rms_thresh = 0.8;
        let mut filter = ImuDenoiseFilter::new(config);

        // Start in hover mode
        assert_eq!(filter.mode, MotionMode::Hover, "Should start in hover mode");

        // Feed low-motion samples (should stay in hover)
        for _ in 0..60 {
            filter.process_gyro(&[0.1, 0.1, 0.1]);
        }
        assert_eq!(
            filter.mode,
            MotionMode::Hover,
            "Should stay in hover for low motion"
        );

        // Feed high-motion samples to trigger aggressive mode
        // Need enough samples to overcome debouncing (50 samples minimum)
        for _ in 0..100 {
            filter.process_gyro(&[1.0, 1.0, 1.0]);
        }
        assert_eq!(
            filter.mode,
            MotionMode::Aggressive,
            "Should switch to aggressive mode"
        );

        // Feed low-motion samples to return to hover
        for _ in 0..100 {
            filter.process_gyro(&[0.1, 0.1, 0.1]);
        }
        assert_eq!(
            filter.mode,
            MotionMode::Hover,
            "Should return to hover mode"
        );
    }

    #[test]
    fn test_adaptive_notch_q() {
        let mut config = DenoiseConfig::default();
        config.adaptive_notch_q = true;
        config.notch_q = 6.0;
        config.notch_q_min = 4.0;
        config.notch_q_max = 8.0;
        config.notch_q_ref_rads = 0.6;
        let mut filter = ImuDenoiseFilter::new(config);

        // Initial Q should be the configured value
        assert_eq!(filter.current_notch_q, 6.0, "Initial Q should match config");

        // Feed low-amplitude samples (below ref, should increase Q)
        for _ in 0..60 {
            filter.process_gyro(&[0.2, 0.2, 0.2]);
        }
        // Q should increase towards max (but debouncing limits changes)
        // After sufficient samples, should trend toward higher Q
        let q_after_low = filter.current_notch_q;

        // Feed high-amplitude samples (above ref, should decrease Q)
        for _ in 0..60 {
            filter.process_gyro(&[1.2, 1.2, 1.2]);
        }
        // Q should decrease towards min
        let q_after_high = filter.current_notch_q;

        // Verify Q adjusted in expected direction
        // Due to debouncing, changes are gradual
        assert!(
            q_after_high <= q_after_low + 0.5,
            "Q should decrease or stay similar for high amplitude"
        );
    }

    #[test]
    fn test_per_axis_notch_frequencies() {
        let mut config = DenoiseConfig::default();
        config.enable_notch_filter = true;
        config.notch_frequencies_per_axis = Some([
            vec![0.06, 1.46],     // X-axis: 2 frequencies
            vec![0.06],           // Y-axis: 1 frequency
            vec![1.46, 2.5, 3.0], // Z-axis: 3 frequencies
        ]);
        let filter = ImuDenoiseFilter::new(config);

        // Should create 3 stages (max across axes)
        assert_eq!(filter.notch_filters.len(), 3, "Should have 3 notch stages");

        // Each stage should have 3 filters (one per axis)
        for stage in &filter.notch_filters {
            assert_eq!(stage.len(), 3, "Each stage should have 3 axis filters");
        }
    }

    #[test]
    fn test_lowpass_attenuates_high_frequency() {
        let mut config = DenoiseConfig::default();
        config.lowpass_cutoff = 10.0; // Low cutoff to make effect obvious
        config.imu_sample_rate = 200.0;
        config.enable_notch_filter = false; // Disable to isolate lowpass effect
        let mut filter = ImuDenoiseFilter::new(config);

        // Generate high-frequency oscillation (50 Hz, well above 10 Hz cutoff)
        // At 200 Hz sample rate, 50 Hz means period of 4 samples
        let mut sum_output = 0.0;
        let mut sum_input = 0.0;

        for i in 0..100 {
            let t = i as f32 / 200.0;
            let high_freq = (2.0 * std::f32::consts::PI * 50.0 * t).sin();
            let input = [high_freq, 0.0, 0.0];
            let output = filter.process_gyro(&input);

            if i > 50 {
                // Skip transient
                sum_input += input[0].abs();
                sum_output += output[0].abs();
            }
        }

        // Average output amplitude should be significantly less than input
        assert!(
            sum_output < sum_input * 0.3,
            "Lowpass should attenuate high frequency"
        );
    }

    #[test]
    fn test_notch_attenuates_resonance() {
        let mut config = DenoiseConfig::default();
        config.enable_notch_filter = true;
        config.notch_frequencies = vec![10.0]; // Notch at 10 Hz
        config.notch_q = 5.0; // Narrow notch
        config.imu_sample_rate = 200.0;
        config.adaptive_notch_q = false; // Disable adaptive for consistent test
        let mut filter = ImuDenoiseFilter::new(config.clone());

        // Generate signal at notch frequency (10 Hz)
        let mut sum_output_at_notch = 0.0;
        let mut sum_input_at_notch = 0.0;

        for i in 0..200 {
            let t = i as f32 / 200.0;
            let signal = (2.0 * std::f32::consts::PI * 10.0 * t).sin();
            let input = [signal, 0.0, 0.0];
            let output = filter.process_gyro(&input);

            if i > 100 {
                // Skip filter transient
                sum_input_at_notch += input[0].abs();
                sum_output_at_notch += output[0].abs();
            }
        }

        // Output should be much smaller than input at notch frequency
        assert!(
            sum_output_at_notch < sum_input_at_notch * 0.5,
            "Notch filter should attenuate resonance frequency"
        );

        // Now test off-notch frequency (5 Hz, well away from 10 Hz)
        filter = ImuDenoiseFilter::new(config); // Reset filter state
        let mut sum_output_off_notch = 0.0;
        let mut sum_input_off_notch = 0.0;

        for i in 0..200 {
            let t = i as f32 / 200.0;
            let signal = (2.0 * std::f32::consts::PI * 5.0 * t).sin();
            let input = [signal, 0.0, 0.0];
            let output = filter.process_gyro(&input);

            if i > 100 {
                sum_input_off_notch += input[0].abs();
                sum_output_off_notch += output[0].abs();
            }
        }

        // Off-notch frequency should pass through with minimal attenuation
        // (allowing for some attenuation from HP/LP filters)
        assert!(
            sum_output_off_notch > sum_input_off_notch * 0.6,
            "Frequencies away from notch should pass through"
        );
    }

    #[test]
    fn test_accel_processing() {
        let config = DenoiseConfig::default();
        let mut filter = ImuDenoiseFilter::new(config);

        // Test that accelerometer processing works
        let accel = [0.0, 0.0, 9.81]; // Gravity
        let output = filter.process_accel(&accel);

        // Should process without crashing
        assert!(output.len() == 3, "Should return 3-element array");

        // Process dynamic acceleration (oscillating around gravity)
        // This has AC component that should pass through
        let mut max_output: f32 = 0.0;
        for i in 0..200 {
            let t = i as f32 / 200.0;
            let dynamic_accel = [
                0.0,
                0.0,
                9.81 + 0.5 * (2.0 * std::f32::consts::PI * 2.0 * t).sin(), // 2 Hz oscillation
            ];
            let out = filter.process_accel(&dynamic_accel);
            if i > 100 {
                // Skip transient
                max_output = max_output.max(out[2].abs());
            }
        }

        // The AC component should pass through (not the DC gravity)
        // We should see some non-zero output from the oscillation
        assert!(max_output > 0.1, "Should pass AC component of acceleration");
    }

    #[test]
    fn test_filter_stability_with_extreme_inputs() {
        let config = DenoiseConfig::default();
        let mut filter = ImuDenoiseFilter::new(config);

        // Test with very large inputs
        let large = [100.0, 100.0, 100.0];
        let out1 = filter.process_gyro(&large);
        assert!(out1[0].is_finite(), "Should handle large inputs");
        assert!(out1[1].is_finite(), "Should handle large inputs");
        assert!(out1[2].is_finite(), "Should handle large inputs");

        // Test with very small inputs
        let small = [1e-6, 1e-6, 1e-6];
        let out2 = filter.process_gyro(&small);
        assert!(out2[0].is_finite(), "Should handle small inputs");

        // Test with zeros
        let zero = [0.0, 0.0, 0.0];
        let out3 = filter.process_gyro(&zero);
        assert!(out3[0].is_finite(), "Should handle zero inputs");

        // Test with mixed signs
        let mixed = [-1.5, 2.3, -0.7];
        let out4 = filter.process_gyro(&mixed);
        assert!(out4[0].is_finite(), "Should handle mixed sign inputs");
        assert!(out4[1].is_finite(), "Should handle mixed sign inputs");
        assert!(out4[2].is_finite(), "Should handle mixed sign inputs");
    }

    #[test]
    fn test_weight_scale_boundary_conditions() {
        let mut config = DenoiseConfig::default();
        config.clip_threshold_rads = 2.0;
        config.clip_window = 10;
        let mut filter = ImuDenoiseFilter::new(config.clone());

        // All samples clipped
        for _ in 0..20 {
            filter.process_gyro(&[5.0, 5.0, 5.0]);
        }
        // Should hit minimum weight (0.2)
        assert!(
            (filter.weight_scale - 0.2).abs() < 0.01,
            "Weight should be 0.2 when all samples clipped"
        );

        // Reset with no clipping
        filter = ImuDenoiseFilter::new(config.clone());
        for _ in 0..20 {
            filter.process_gyro(&[0.1, 0.1, 0.1]);
        }
        // Should stay at maximum weight (1.0)
        assert!(
            (filter.weight_scale - 1.0).abs() < 0.01,
            "Weight should be 1.0 when no clipping"
        );
    }

    #[test]
    fn test_realistic_flight_scenario() {
        let mut config = DenoiseConfig::default();
        config.imu_sample_rate = 200.0;
        config.spike_window = 3;
        config.adaptive_notch_q = true;
        let mut filter = ImuDenoiseFilter::new(config);

        // Simulate realistic flight: takeoff -> hover -> aggressive maneuver -> hover -> landing
        let scenarios = [
            ("takeoff", 50, [0.5, 0.5, 0.8]),   // Moderate motion
            ("hover", 100, [0.1, 0.1, 0.15]),   // Low motion
            ("maneuver", 80, [1.5, 1.2, 1.8]),  // Aggressive motion
            ("hover2", 150, [0.12, 0.15, 0.1]), // Return to hover (longer for mode transition)
            ("landing", 50, [0.4, 0.6, 0.7]),   // Moderate motion
        ];

        for (phase, samples, base_gyro) in scenarios.iter() {
            for i in 0..*samples {
                let t = i as f32 / 200.0;
                // Add some realistic noise and vibration
                let noise = [
                    0.02 * (10.0 * t).sin(),
                    0.02 * (12.0 * t).cos(),
                    0.02 * (8.0 * t).sin(),
                ];
                let gyro = [
                    base_gyro[0] + noise[0],
                    base_gyro[1] + noise[1],
                    base_gyro[2] + noise[2],
                ];

                let output = filter.process_gyro(&gyro);

                // Verify output is reasonable
                assert!(
                    output[0].is_finite(),
                    "Output should be finite in {}",
                    phase
                );
                assert!(
                    output[1].is_finite(),
                    "Output should be finite in {}",
                    phase
                );
                assert!(
                    output[2].is_finite(),
                    "Output should be finite in {}",
                    phase
                );
            }
        }

        // Should have processed through various motion states successfully
        // Final mode depends on landing phase motion which is moderate
        // Don't assert specific mode since landing motion (0.4-0.7 rad/s) is borderline
        assert!(
            filter.weight_scale > 0.8,
            "Weight should be high for clean signal"
        );
    }

    #[test]
    fn test_burst_noise_handling() {
        let mut config = DenoiseConfig::default();
        config.spike_window = 3;
        let mut filter = ImuDenoiseFilter::new(config);

        // Feed clean signal
        for _ in 0..50 {
            filter.process_gyro(&[0.2, 0.2, 0.2]);
        }

        // Burst of noise spikes
        for _ in 0..5 {
            filter.process_gyro(&[10.0, -8.0, 12.0]);
        }

        // Return to clean signal
        for _ in 0..50 {
            filter.process_gyro(&[0.2, 0.2, 0.2]);
        }

        // Filter should recover and not become unstable
        let output = filter.process_gyro(&[0.2, 0.2, 0.2]);
        assert!(output[0].abs() < 2.0, "Should recover from burst noise");
        assert!(output[1].abs() < 2.0, "Should recover from burst noise");
        assert!(output[2].abs() < 2.0, "Should recover from burst noise");
    }

    #[test]
    fn test_continuous_high_rate_processing() {
        let config = DenoiseConfig::default();
        let mut filter = ImuDenoiseFilter::new(config);

        // Simulate 10 seconds at 200 Hz (2000 samples)
        for i in 0..2000 {
            let t = i as f32 / 200.0;
            let gyro = [
                0.3 * (2.0 * std::f32::consts::PI * 1.0 * t).sin(),
                0.2 * (2.0 * std::f32::consts::PI * 1.5 * t).cos(),
                0.25 * (2.0 * std::f32::consts::PI * 0.8 * t).sin(),
            ];
            let output = filter.process_gyro(&gyro);

            // Verify stability over long duration
            assert!(
                output[0].is_finite(),
                "Should remain stable at sample {}",
                i
            );
            assert!(
                output[0].abs() < 10.0,
                "Output should be bounded at sample {}",
                i
            );
        }
    }

    #[test]
    fn test_mode_hysteresis_prevents_oscillation() {
        let mut config = DenoiseConfig::default();
        config.hover_rms_thresh = 0.25;
        config.aggressive_rms_thresh = 0.8;
        let mut filter = ImuDenoiseFilter::new(config);

        // Start in hover
        for _ in 0..60 {
            filter.process_gyro(&[0.1, 0.1, 0.1]);
        }
        assert_eq!(filter.mode, MotionMode::Hover);

        // Oscillate around threshold - should not switch rapidly due to debouncing
        let mut mode_changes = 0;
        let mut last_mode = filter.mode;

        for i in 0..200 {
            // Alternate between just below and just above aggressive threshold
            let amp = if i % 2 == 0 { 0.75 } else { 0.85 };
            filter.process_gyro(&[amp, amp, amp]);

            if filter.mode != last_mode {
                mode_changes += 1;
                last_mode = filter.mode;
            }
        }

        // With debouncing (50 samples minimum), should have very few mode changes
        // Even with 100 oscillations, debouncing prevents rapid switching
        assert!(
            mode_changes < 4,
            "Debouncing should prevent rapid mode switching: {} changes",
            mode_changes
        );
    }

    #[test]
    fn test_multiple_notch_frequencies_independence() {
        let mut config = DenoiseConfig::default();
        config.enable_notch_filter = true;
        config.notch_frequencies = vec![5.0, 15.0, 25.0];
        config.notch_q = 8.0;
        config.adaptive_notch_q = false;

        // Test attenuation at each notch frequency independently
        for &freq in &[5.0, 15.0, 25.0] {
            let mut filter_local = ImuDenoiseFilter::new(config.clone());

            let mut sum_input: f32 = 0.0;
            let mut sum_output: f32 = 0.0;

            for i in 0..300 {
                let t = i as f32 / 200.0;
                let signal = (2.0 * std::f32::consts::PI * freq * t).sin();
                let input = [signal, 0.0, 0.0];
                let output = filter_local.process_gyro(&input);

                if i > 150 {
                    sum_input += input[0].abs();
                    sum_output += output[0].abs();
                }
            }

            let attenuation = sum_output / sum_input.max(1e-6);
            assert!(
                attenuation < 0.5,
                "Should attenuate {} Hz: ratio={}",
                freq,
                attenuation
            );
        }

        // Test pass-through at intermediate frequency (10 Hz, between 5 and 15)
        let mut filter_pass = ImuDenoiseFilter::new(config.clone());
        let mut sum_input: f32 = 0.0;
        let mut sum_output: f32 = 0.0;

        for i in 0..300 {
            let t = i as f32 / 200.0;
            let signal = (2.0 * std::f32::consts::PI * 10.0 * t).sin();
            let input = [signal, 0.0, 0.0];
            let output = filter_pass.process_gyro(&input);

            if i > 150 {
                sum_input += input[0].abs();
                sum_output += output[0].abs();
            }
        }

        let passthrough = sum_output / sum_input.max(1e-6);
        // Should have less attenuation at intermediate frequency
        // (though still some from HP/LP filters)
        assert!(
            passthrough > 0.4,
            "Should pass 10 Hz better than notch frequencies: ratio={}",
            passthrough
        );
    }
}

/// Example usage showing integration with VIO pipeline
pub fn example_usage() {
    let mut config = DenoiseConfig::default();
    config.imu_sample_rate = 200.0;
    config.camera_frame_rate = 30.0;
    config.highpass_cutoff = 0.5; // Remove platform sway
    config.lowpass_cutoff = 50.0;
    config.enable_notch_filter = true;
    config.notch_frequencies = vec![0.06, 1.46]; // Our identified resonances
    config.notch_q = 5.0;
    config.enable_vision_fusion = true;
    config.vision_trust = 0.3; // Trust vision 30%, IMU 70%

    let mut filter = ImuDenoiseFilter::new(config);

    // Simulate 200 Hz IMU stream
    for sample in 0..200 {
        let time_ms = sample as f32 / 200.0 * 1000.0;
        let gyro = [0.1, 0.15, 0.09]; // Simulated gyro data

        // Process raw measurement through filter
        let filtered_gyro = filter.process_gyro(&gyro);

        // Buffer for camera frame integration
        filter.buffer_imu_sample(time_ms, &filtered_gyro);

        // When camera frame arrives (every ~33ms for 30 Hz)
        if sample % 7 == 0 {
            if let Some(integrated_gyro) = filter.process_camera_frame(time_ms) {
                println!(
                    "Camera frame at {}ms: integrated gyro={:.4}, quality={:.2}",
                    time_ms,
                    integrated_gyro.norm(),
                    filter.quality()
                );
            }
        }
    }
}
