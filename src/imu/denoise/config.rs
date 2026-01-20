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
