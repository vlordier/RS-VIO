/// Speed/velocity-aware accuracy metrics
///
/// Bin metrics by motion speed (static, slow, normal, fast, very fast).
use super::types::BinMetrics;

/// Accuracy metrics binned by speed
#[derive(Clone, Debug)]
pub struct SpeedBinnedMetrics {
    /// Static or near-static (< 0.1 m/s)
    pub static_scene: BinMetrics,

    /// Slow motion (0.1 - 0.5 m/s)
    pub slow_motion: BinMetrics,

    /// Normal motion (0.5 - 2.0 m/s)
    pub normal_motion: BinMetrics,

    /// Fast motion (2.0 - 5.0 m/s)
    pub fast_motion: BinMetrics,

    /// Very fast motion (5.0+ m/s)
    pub very_fast_motion: BinMetrics,
}

impl Default for SpeedBinnedMetrics {
    fn default() -> Self {
        Self {
            static_scene: BinMetrics::new(),
            slow_motion: BinMetrics::new(),
            normal_motion: BinMetrics::new(),
            fast_motion: BinMetrics::new(),
            very_fast_motion: BinMetrics::new(),
        }
    }
}

/// Per-speed-bin improvement metrics
#[derive(Clone, Debug)]
pub struct SpeedBinnedImprovement {
    /// Static (<0.1 m/s): (weighted, unweighted, improvement %)
    pub static_scene: (f64, f64, f32),

    /// Slow (0.1-0.5 m/s)
    pub slow_motion: (f64, f64, f32),

    /// Normal (0.5-2.0 m/s)
    pub normal_motion: (f64, f64, f32),

    /// Fast (2.0-5.0 m/s)
    pub fast_motion: (f64, f64, f32),

    /// Very fast (5.0+ m/s)
    pub very_fast_motion: (f64, f64, f32),
}

impl SpeedBinnedImprovement {
    pub fn new() -> Self {
        Self {
            static_scene: (0.0, 0.0, 0.0),
            slow_motion: (0.0, 0.0, 0.0),
            normal_motion: (0.0, 0.0, 0.0),
            fast_motion: (0.0, 0.0, 0.0),
            very_fast_motion: (0.0, 0.0, 0.0),
        }
    }
}

impl Default for SpeedBinnedImprovement {
    fn default() -> Self {
        Self::new()
    }
}

/// Bin sample by speed
pub fn bin_by_speed(speed: f32) -> usize {
    match speed {
        s if s < 0.1 => 0,
        s if s < 0.5 => 1,
        s if s < 2.0 => 2,
        s if s < 5.0 => 3,
        _ => 4,
    }
}
