//! Hardware profiling and automatic timeout tuning
//!
//! Provides hardware profile detection and automatic timeout adjustment based on
//! observed pipeline latencies. Supports predefined profiles for common embedded systems.

use std::fmt;

/// Hardware profile for timeout tuning
#[derive(Debug, Clone)]
pub struct HardwareProfile {
    /// Profile name (e.g., "Jetson Nano", "Desktop CPU")
    pub name: String,
    /// Number of CPU cores
    pub cpu_cores: u32,
    /// Available memory in MB
    pub memory_mb: u32,
    /// Estimated base detection latency in microseconds
    pub base_detection_us: u64,
    /// Estimated base optimization latency in microseconds
    pub base_optimization_us: u64,
    /// Default detection timeout multiplier (e.g., 2x base latency)
    pub detection_timeout_multiplier: f64,
    /// Default optimization timeout multiplier (e.g., 3x base latency)
    pub optimization_timeout_multiplier: f64,
    /// Whether to enable auto-tuning
    pub enable_autotuning: bool,
    /// How many samples before auto-tuning adjusts
    pub tuning_sample_count: u32,
}

impl HardwareProfile {
    /// Create a custom hardware profile
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            cpu_cores: 4,
            memory_mb: 2048,
            base_detection_us: 50,
            base_optimization_us: 100,
            detection_timeout_multiplier: 2.0,
            optimization_timeout_multiplier: 3.0,
            enable_autotuning: true,
            tuning_sample_count: 100,
        }
    }

    /// Get detection timeout in milliseconds
    pub fn detection_timeout_ms(&self) -> u64 {
        (self.base_detection_us as f64 * self.detection_timeout_multiplier / 1000.0).ceil() as u64
    }

    /// Get optimization timeout in milliseconds
    pub fn optimization_timeout_ms(&self) -> u64 {
        (self.base_optimization_us as f64 * self.optimization_timeout_multiplier / 1000.0).ceil()
            as u64
    }

    /// Jetson Nano profile (2 ARM cores, 4GB RAM)
    pub fn jetson_nano() -> Self {
        Self {
            name: "Jetson Nano".to_string(),
            cpu_cores: 4,
            memory_mb: 4096,
            base_detection_us: 100,
            base_optimization_us: 200,
            detection_timeout_multiplier: 3.0,
            optimization_timeout_multiplier: 4.0,
            enable_autotuning: true,
            tuning_sample_count: 100,
        }
    }

    /// Jetson Xavier profile (8 ARM cores, 8GB RAM)
    pub fn jetson_xavier() -> Self {
        Self {
            name: "Jetson Xavier".to_string(),
            cpu_cores: 8,
            memory_mb: 8192,
            base_detection_us: 1000,
            base_optimization_us: 2000,
            detection_timeout_multiplier: 2.5,
            optimization_timeout_multiplier: 3.5,
            enable_autotuning: true,
            tuning_sample_count: 100,
        }
    }

    /// Jetson Orin profile (12 ARM cores, 12GB RAM)
    pub fn jetson_orin() -> Self {
        Self {
            name: "Jetson Orin".to_string(),
            cpu_cores: 12,
            memory_mb: 12288,
            base_detection_us: 30,
            base_optimization_us: 60,
            detection_timeout_multiplier: 2.0,
            optimization_timeout_multiplier: 3.0,
            enable_autotuning: true,
            tuning_sample_count: 100,
        }
    }

    /// Desktop CPU profile (16+ cores, 16GB+ RAM)
    pub fn desktop_cpu() -> Self {
        Self {
            name: "Desktop CPU".to_string(),
            cpu_cores: 16,
            memory_mb: 16384,
            base_detection_us: 20,
            base_optimization_us: 50,
            detection_timeout_multiplier: 1.5,
            optimization_timeout_multiplier: 2.5,
            enable_autotuning: true,
            tuning_sample_count: 50,
        }
    }

    /// Robot board (Raspberry Pi 4, 4 cores, 4GB RAM)
    pub fn robot_board() -> Self {
        Self {
            name: "Robot Board (RPi4)".to_string(),
            cpu_cores: 4,
            memory_mb: 4096,
            base_detection_us: 150,
            base_optimization_us: 300,
            detection_timeout_multiplier: 4.0,
            optimization_timeout_multiplier: 5.0,
            enable_autotuning: true,
            tuning_sample_count: 100,
        }
    }

    /// Get profile by name
    pub fn by_name(name: &str) -> Option<Self> {
        match name.to_lowercase().as_str() {
            "jetson nano" | "nano" => Some(Self::jetson_nano()),
            "jetson xavier" | "xavier" => Some(Self::jetson_xavier()),
            "jetson orin" | "orin" => Some(Self::jetson_orin()),
            "desktop" | "desktop cpu" => Some(Self::desktop_cpu()),
            "robot board" | "raspberrypi" | "rpi4" => Some(Self::robot_board()),
            _ => None,
        }
    }
}

impl fmt::Display for HardwareProfile {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{} ({} cores, {}MB RAM, det_timeout={}ms, opt_timeout={}ms)",
            self.name,
            self.cpu_cores,
            self.memory_mb,
            self.detection_timeout_ms(),
            self.optimization_timeout_ms()
        )
    }
}

/// Automatic timeout tuner based on observed latencies
pub struct TimeoutAutoTuner {
    /// Hardware profile
    profile: HardwareProfile,
    /// Exponential moving average of detection latencies
    detection_ema: f64,
    /// Exponential moving average of optimization latencies
    optimization_ema: f64,
    /// Sample count
    sample_count: u32,
    /// EMA smoothing factor (alpha)
    ema_alpha: f64,
}

impl TimeoutAutoTuner {
    /// Create new auto-tuner for profile
    pub fn new(profile: HardwareProfile) -> Self {
        let detection_ema = profile.base_detection_us as f64;
        let optimization_ema = profile.base_optimization_us as f64;

        Self {
            profile,
            detection_ema,
            optimization_ema,
            sample_count: 0,
            ema_alpha: 0.1, // 10% new sample, 90% historical
        }
    }

    /// Record observed detection latency
    pub fn observe_detection(&mut self, latency_us: u64) {
        let latency = latency_us as f64;
        self.detection_ema = self.ema_alpha * latency + (1.0 - self.ema_alpha) * self.detection_ema;
        self.sample_count += 1;
    }

    /// Record observed optimization latency
    pub fn observe_optimization(&mut self, latency_us: u64) {
        let latency = latency_us as f64;
        self.optimization_ema =
            self.ema_alpha * latency + (1.0 - self.ema_alpha) * self.optimization_ema;
        self.sample_count += 1;
    }

    /// Get current recommended detection timeout in milliseconds
    pub fn recommended_detection_timeout_ms(&self) -> u64 {
        (self.detection_ema * self.profile.detection_timeout_multiplier / 1000.0).ceil() as u64
    }

    /// Get current recommended optimization timeout in milliseconds
    pub fn recommended_optimization_timeout_ms(&self) -> u64 {
        (self.optimization_ema * self.profile.optimization_timeout_multiplier / 1000.0).ceil()
            as u64
    }

    /// Get default timeout (before any tuning)
    pub fn default_detection_timeout_ms(&self) -> u64 {
        self.profile.detection_timeout_ms()
    }

    /// Get default timeout (before any tuning)
    pub fn default_optimization_timeout_ms(&self) -> u64 {
        self.profile.optimization_timeout_ms()
    }

    /// Check if tuning should happen (enough samples collected)
    pub fn should_tune(&self) -> bool {
        self.profile.enable_autotuning && self.sample_count >= self.profile.tuning_sample_count
    }

    /// Get current profile
    pub fn profile(&self) -> &HardwareProfile {
        &self.profile
    }

    /// Get sample count
    pub fn sample_count(&self) -> u32 {
        self.sample_count
    }

    /// Reset tuning data but keep profile
    pub fn reset(&mut self) {
        self.detection_ema = self.profile.base_detection_us as f64;
        self.optimization_ema = self.profile.base_optimization_us as f64;
        self.sample_count = 0;
    }
}

impl fmt::Display for TimeoutAutoTuner {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{} - Samples: {}, Det_EMA: {:.1}us (timeout={}ms), Opt_EMA: {:.1}us (timeout={}ms)",
            self.profile.name,
            self.sample_count,
            self.detection_ema,
            self.recommended_detection_timeout_ms(),
            self.optimization_ema,
            self.recommended_optimization_timeout_ms()
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_hardware_profile_creation() {
        let profile = HardwareProfile::new("Test Profile");
        assert_eq!(profile.name, "Test Profile");
        assert_eq!(profile.cpu_cores, 4);
    }

    #[test]
    fn test_jetson_nano_profile() {
        let profile = HardwareProfile::jetson_nano();
        assert_eq!(profile.name, "Jetson Nano");
        assert_eq!(profile.cpu_cores, 4);
        assert!(profile.detection_timeout_ms() > 0);
    }

    #[test]
    fn test_jetson_xavier_profile() {
        let profile = HardwareProfile::jetson_xavier();
        assert_eq!(profile.cpu_cores, 8);
        assert!(profile.optimization_timeout_ms() > profile.detection_timeout_ms());
    }

    #[test]
    fn test_jetson_orin_profile() {
        let profile = HardwareProfile::jetson_orin();
        assert_eq!(profile.cpu_cores, 12);
        assert!(profile.detection_timeout_ms() > 0);
    }

    #[test]
    fn test_desktop_profile() {
        let profile = HardwareProfile::desktop_cpu();
        assert!(profile.cpu_cores >= 16);
        assert!(profile.memory_mb >= 16384);
    }

    #[test]
    fn test_robot_board_profile() {
        let profile = HardwareProfile::robot_board();
        assert_eq!(profile.cpu_cores, 4);
        // Robot board should have longer timeouts due to lower performance
        assert!(
            profile.detection_timeout_ms() >= HardwareProfile::jetson_nano().detection_timeout_ms()
        );
    }

    #[test]
    fn test_profile_by_name() {
        assert!(HardwareProfile::by_name("jetson nano").is_some());
        assert!(HardwareProfile::by_name("Xavier").is_some());
        assert!(HardwareProfile::by_name("orin").is_some());
        assert!(HardwareProfile::by_name("desktop").is_some());
        assert!(HardwareProfile::by_name("robot board").is_some());
        assert!(HardwareProfile::by_name("unknown").is_none());
    }

    #[test]
    fn test_timeout_auto_tuner_creation() {
        let profile = HardwareProfile::jetson_nano();
        let tuner = TimeoutAutoTuner::new(profile);

        assert_eq!(tuner.sample_count(), 0);
        assert!(!tuner.should_tune());
    }

    #[test]
    fn test_auto_tuner_ema_detection() {
        let profile = HardwareProfile::jetson_nano();
        let mut tuner = TimeoutAutoTuner::new(profile);

        let default_timeout = tuner.recommended_detection_timeout_ms();

        // Observe a higher latency
        tuner.observe_detection(200);
        let adjusted_timeout = tuner.recommended_detection_timeout_ms();

        // EMA should move towards higher value
        assert!(adjusted_timeout >= default_timeout);
    }

    #[test]
    fn test_auto_tuner_ema_optimization() {
        let profile = HardwareProfile::jetson_nano();
        let mut tuner = TimeoutAutoTuner::new(profile);

        tuner.observe_optimization(300);
        let timeout = tuner.recommended_optimization_timeout_ms();

        assert!(timeout > 0);
    }

    #[test]
    fn test_auto_tuner_should_tune() {
        let mut profile = HardwareProfile::jetson_nano();
        profile.tuning_sample_count = 5;

        let mut tuner = TimeoutAutoTuner::new(profile);

        for _ in 0..4 {
            tuner.observe_detection(100);
        }
        assert!(!tuner.should_tune());

        tuner.observe_detection(100);
        assert!(tuner.should_tune());
    }

    #[test]
    fn test_auto_tuner_reset() {
        let profile = HardwareProfile::jetson_nano();
        let mut tuner = TimeoutAutoTuner::new(profile);

        tuner.observe_detection(200);
        tuner.observe_optimization(300);
        assert_eq!(tuner.sample_count(), 2);

        tuner.reset();
        assert_eq!(tuner.sample_count(), 0);
    }

    #[test]
    fn test_profile_display() {
        let profile = HardwareProfile::jetson_nano();
        let display = format!("{}", profile);
        assert!(display.contains("Jetson Nano"));
        assert!(display.contains("4 cores"));
    }

    #[test]
    fn test_tuner_display() {
        let profile = HardwareProfile::desktop_cpu();
        let tuner = TimeoutAutoTuner::new(profile);
        let display = format!("{}", tuner);
        assert!(display.contains("Desktop CPU"));
        assert!(display.contains("Samples: 0"));
    }
}
