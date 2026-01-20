/// Accuracy and error metrics
///
/// Tracks residual errors and feature tracking quality.

/// Weighted vs unweighted residual statistics
#[derive(Clone, Debug)]
pub struct WeightedResidualStats {
    /// RMS error for visual residuals
    pub visual_rms: f64,

    /// RMS error for IMU residuals
    pub imu_rms: f64,

    /// Combined RMS
    pub total_rms: f64,

    /// Number of samples
    pub sample_count: usize,

    /// Outlier rate (% of residuals > 3×median)
    pub outlier_rate: f32,

    /// Mean residual magnitude
    pub mean_residual: f64,

    /// Median residual magnitude
    pub median_residual: f64,

    /// Std dev of residuals
    pub std_residual: f64,
}

impl WeightedResidualStats {
    pub fn new() -> Self {
        Self {
            visual_rms: 0.0,
            imu_rms: 0.0,
            total_rms: 0.0,
            sample_count: 0,
            outlier_rate: 0.0,
            mean_residual: 0.0,
            median_residual: 0.0,
            std_residual: 0.0,
        }
    }

    /// Compute improvement percentage vs baseline
    pub fn improvement_vs(&self, baseline: &WeightedResidualStats) -> f32 {
        if baseline.total_rms == 0.0 {
            0.0
        } else {
            ((baseline.total_rms - self.total_rms) / baseline.total_rms * 100.0) as f32
        }
    }
}

impl Default for WeightedResidualStats {
    fn default() -> Self {
        Self::new()
    }
}

/// Feature tracking survival statistics
#[derive(Clone, Debug)]
pub struct TrackSurvivalStats {
    /// Median track length (number of frames a feature survives)
    pub median_track_length: f32,

    /// Mean track length
    pub mean_track_length: f32,

    /// Max track length
    pub max_track_length: usize,

    /// Percentage of features lasting > 5 frames
    pub survival_rate_5: f32,

    /// Percentage of features lasting > 10 frames
    pub survival_rate_10: f32,

    /// Total features tracked
    pub total_features: usize,
}

impl TrackSurvivalStats {
    pub fn new() -> Self {
        Self {
            median_track_length: 0.0,
            mean_track_length: 0.0,
            max_track_length: 0,
            survival_rate_5: 0.0,
            survival_rate_10: 0.0,
            total_features: 0,
        }
    }
}

impl Default for TrackSurvivalStats {
    fn default() -> Self {
        Self::new()
    }
}

/// Compute residual statistics from samples
pub fn compute_residual_stats(
    visual_samples: &[(f64, f32, f32)],
    imu_samples: &[(f64, f32, f32)],
) -> WeightedResidualStats {
    let visual_rms = if !visual_samples.is_empty() {
        let sum_sq: f32 = visual_samples.iter().map(|(_, _, r)| r * r).sum();
        (sum_sq as f64 / visual_samples.len() as f64).sqrt()
    } else {
        0.0
    };

    let imu_rms = if !imu_samples.is_empty() {
        let sum_sq: f32 = imu_samples.iter().map(|(_, _, r)| r * r).sum();
        (sum_sq as f64 / imu_samples.len() as f64).sqrt()
    } else {
        0.0
    };

    let total_count = visual_samples.len() + imu_samples.len();
    let total_rms = if total_count > 0 {
        let visual_sum_sq: f32 = visual_samples.iter().map(|(_, _, r)| r * r).sum();
        let imu_sum_sq: f32 = imu_samples.iter().map(|(_, _, r)| r * r).sum();
        ((visual_sum_sq + imu_sum_sq) as f64 / total_count as f64).sqrt()
    } else {
        0.0
    };

    // Compute outlier rate
    let mut all_residuals: Vec<f32> = visual_samples
        .iter()
        .chain(imu_samples.iter())
        .map(|(_, _, r)| *r)
        .collect();
    all_residuals.sort_by(|a, b| a.partial_cmp(b).unwrap());

    let median = if !all_residuals.is_empty() {
        all_residuals[all_residuals.len() / 2]
    } else {
        0.0
    };

    let outlier_threshold = 3.0 * median;
    let outlier_count = all_residuals
        .iter()
        .filter(|&&r| r > outlier_threshold)
        .count();
    let outlier_rate = if !all_residuals.is_empty() {
        (outlier_count as f32 / all_residuals.len() as f32) * 100.0
    } else {
        0.0
    };

    let mean_residual = if !all_residuals.is_empty() {
        all_residuals.iter().map(|&r| r as f64).sum::<f64>() / all_residuals.len() as f64
    } else {
        0.0
    };

    let std_residual = if !all_residuals.is_empty() {
        let variance = all_residuals
            .iter()
            .map(|&r| {
                let diff = r as f64 - mean_residual;
                diff * diff
            })
            .sum::<f64>()
            / all_residuals.len() as f64;
        variance.sqrt()
    } else {
        0.0
    };

    WeightedResidualStats {
        visual_rms,
        imu_rms,
        total_rms,
        sample_count: total_count,
        outlier_rate,
        mean_residual,
        median_residual: median as f64,
        std_residual,
    }
}

/// Compute track survival statistics
pub fn compute_track_survival(track_lengths: &[usize]) -> TrackSurvivalStats {
    if track_lengths.is_empty() {
        return TrackSurvivalStats::default();
    }

    let mut sorted = track_lengths.to_vec();
    sorted.sort();

    let median_track_length = sorted[sorted.len() / 2] as f32;
    let mean_track_length = track_lengths.iter().sum::<usize>() as f32 / track_lengths.len() as f32;
    let max_track_length = *sorted.last().unwrap_or(&0);

    let survival_rate_5 = track_lengths.iter().filter(|&&len| len > 5).count() as f32
        / track_lengths.len() as f32
        * 100.0;

    let survival_rate_10 = track_lengths.iter().filter(|&&len| len > 10).count() as f32
        / track_lengths.len() as f32
        * 100.0;

    TrackSurvivalStats {
        median_track_length,
        mean_track_length,
        max_track_length,
        survival_rate_5,
        survival_rate_10,
        total_features: track_lengths.len(),
    }
}
