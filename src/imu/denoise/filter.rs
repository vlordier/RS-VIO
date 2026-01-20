use nalgebra as na;
use std::collections::VecDeque;

use super::biquad::BiquadFilter;
use super::config::DenoiseConfig;
use super::motion_mode::MotionMode;

/// Real-time IMU denoising filter
pub struct ImuDenoiseFilter {
    config: DenoiseConfig,

    // High-pass and low-pass filters (per axis)
    highpass_filters: [BiquadFilter; 3],
    lowpass_filters: [BiquadFilter; 3],
    pub(crate) notch_filters: Vec<[BiquadFilter; 3]>,

    // Complementary filter state
    vision_accel: na::Vector3<f32>,
    imu_accel: na::Vector3<f32>,

    // IMU preintegration buffer
    imu_buffer: VecDeque<(f32, na::Vector3<f32>)>,
    last_camera_frame_time: Option<f32>,

    // Statistics
    pub signal_quality: f32, // 0.0-1.0: estimate of output quality

    // Motion-mode FSM
    pub(crate) mode: MotionMode,
    rms_ewma: f32,

    // Spike rejection history (last two samples per axis)
    gyro_hist: [[f32; 3]; 2],
    accel_hist: [[f32; 3]; 2],

    // Clipping detection
    clip_window: Vec<bool>,
    clip_head: usize,
    clip_count: usize,
    pub weight_scale: f32, // 0-1 scaling factor for estimator weighting

    pub(crate) current_notch_q: f32,
    samples_since_rebuild: u32,
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
