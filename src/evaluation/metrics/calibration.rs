/// Calibration-aware performance metrics
///
/// Tracks how calibration quality affects tracking accuracy across distance/speed bins.

use super::accuracy::{compute_residual_stats, compute_track_survival, TrackSurvivalStats, WeightedResidualStats};
use super::distance::DistanceBinnedImprovement;
use super::speed::SpeedBinnedImprovement;

/// Calibration-aware performance metrics
#[derive(Clone, Debug)]
pub struct CalibrationAwareMetrics {
    /// Weighted residuals (with calibration confidence)
    pub weighted_residuals: WeightedResidualStats,

    /// Unweighted residuals (baseline)
    pub unweighted_residuals: WeightedResidualStats,

    /// Track survival: how many features last N frames
    pub track_survival: TrackSurvivalStats,

    /// Per-bin performance comparison (distance)
    pub distance_bin_improvements: DistanceBinnedImprovement,

    /// Per-bin performance comparison (speed)
    pub speed_bin_improvements: SpeedBinnedImprovement,
}

/// Analyzer for calibration-aware performance
pub struct CalibrationAwareAnalyzer {
    /// Weighted visual residuals: (distance, speed, residual)
    weighted_visual_samples: Vec<(f64, f32, f32)>,

    /// Unweighted visual residuals: (distance, speed, residual)
    unweighted_visual_samples: Vec<(f64, f32, f32)>,

    /// Weighted IMU residuals: (distance, speed, residual)
    weighted_imu_samples: Vec<(f64, f32, f32)>,

    /// Unweighted IMU residuals: (distance, speed, residual)
    unweighted_imu_samples: Vec<(f64, f32, f32)>,

    /// Feature track lengths
    track_lengths: Vec<usize>,
}

impl CalibrationAwareAnalyzer {
    pub fn new() -> Self {
        Self {
            weighted_visual_samples: Vec::new(),
            unweighted_visual_samples: Vec::new(),
            weighted_imu_samples: Vec::new(),
            unweighted_imu_samples: Vec::new(),
            track_lengths: Vec::new(),
        }
    }

    /// Record a weighted visual residual at distance/speed
    pub fn record_weighted_visual_residual(&mut self, distance: f64, speed: f32, residual: f32) {
        self.weighted_visual_samples
            .push((distance, speed, residual));
    }

    /// Record an unweighted visual residual at distance/speed
    pub fn record_unweighted_visual_residual(&mut self, distance: f64, speed: f32, residual: f32) {
        self.unweighted_visual_samples
            .push((distance, speed, residual));
    }

    /// Record a weighted IMU residual at distance/speed
    pub fn record_weighted_imu_residual(&mut self, distance: f64, speed: f32, residual: f32) {
        self.weighted_imu_samples.push((distance, speed, residual));
    }

    /// Record an unweighted IMU residual at distance/speed
    pub fn record_unweighted_imu_residual(&mut self, distance: f64, speed: f32, residual: f32) {
        self.unweighted_imu_samples
            .push((distance, speed, residual));
    }

    /// Record feature track length
    pub fn record_track_length(&mut self, length: usize) {
        self.track_lengths.push(length);
    }

    /// Compute comprehensive calibration-aware metrics
    pub fn analyze(&self) -> CalibrationAwareMetrics {
        let weighted_residuals =
            compute_residual_stats(&self.weighted_visual_samples, &self.weighted_imu_samples);
        let unweighted_residuals = compute_residual_stats(
            &self.unweighted_visual_samples,
            &self.unweighted_imu_samples,
        );
        let track_survival = compute_track_survival(&self.track_lengths);
        let distance_bin_improvements = self.compute_distance_improvements();
        let speed_bin_improvements = self.compute_speed_improvements();

        CalibrationAwareMetrics {
            weighted_residuals,
            unweighted_residuals,
            track_survival,
            distance_bin_improvements,
            speed_bin_improvements,
        }
    }

    /// Compute per-distance-bin improvements
    fn compute_distance_improvements(&self) -> DistanceBinnedImprovement {
        let mut improvement = DistanceBinnedImprovement::new();

        // Process weighted samples by distance
        let mut weighted_by_distance: Vec<Vec<f32>> = vec![Vec::new(); 4];
        for (distance, _, residual) in &self.weighted_visual_samples {
            let idx = match distance {
                d if d < &1.0 => 0,
                d if d < &3.0 => 1,
                d if d < &10.0 => 2,
                _ => 3,
            };
            weighted_by_distance[idx].push(*residual);
        }

        // Process unweighted samples by distance
        let mut unweighted_by_distance: Vec<Vec<f32>> = vec![Vec::new(); 4];
        for (distance, _, residual) in &self.unweighted_visual_samples {
            let idx = match distance {
                d if d < &1.0 => 0,
                d if d < &3.0 => 1,
                d if d < &10.0 => 2,
                _ => 3,
            };
            unweighted_by_distance[idx].push(*residual);
        }

        // Compute RMS and improvement for each bin
        let compute_bin_improvement = |weighted: &[f32], unweighted: &[f32]| -> (f64, f64, f32) {
            let weighted_rms = if !weighted.is_empty() {
                let sum_sq: f32 = weighted.iter().map(|r| r * r).sum();
                (sum_sq as f64 / weighted.len() as f64).sqrt()
            } else {
                0.0
            };

            let unweighted_rms = if !unweighted.is_empty() {
                let sum_sq: f32 = unweighted.iter().map(|r| r * r).sum();
                (sum_sq as f64 / unweighted.len() as f64).sqrt()
            } else {
                0.0
            };

            let improvement = if unweighted_rms > 0.0 {
                ((unweighted_rms - weighted_rms) / unweighted_rms * 100.0) as f32
            } else {
                0.0
            };

            (weighted_rms, unweighted_rms, improvement)
        };

        improvement.near_field =
            compute_bin_improvement(&weighted_by_distance[0], &unweighted_by_distance[0]);
        improvement.mid_field =
            compute_bin_improvement(&weighted_by_distance[1], &unweighted_by_distance[1]);
        improvement.far_field =
            compute_bin_improvement(&weighted_by_distance[2], &unweighted_by_distance[2]);
        improvement.very_far_field =
            compute_bin_improvement(&weighted_by_distance[3], &unweighted_by_distance[3]);

        improvement
    }

    /// Compute per-speed-bin improvements
    fn compute_speed_improvements(&self) -> SpeedBinnedImprovement {
        let mut improvement = SpeedBinnedImprovement::new();

        // Process weighted samples by speed
        let mut weighted_by_speed: Vec<Vec<f32>> = vec![Vec::new(); 5];
        for (_, speed, residual) in &self.weighted_visual_samples {
            let idx = match speed {
                s if s < &0.1 => 0,
                s if s < &0.5 => 1,
                s if s < &2.0 => 2,
                s if s < &5.0 => 3,
                _ => 4,
            };
            weighted_by_speed[idx].push(*residual);
        }

        // Process unweighted samples by speed
        let mut unweighted_by_speed: Vec<Vec<f32>> = vec![Vec::new(); 5];
        for (_, speed, residual) in &self.unweighted_visual_samples {
            let idx = match speed {
                s if s < &0.1 => 0,
                s if s < &0.5 => 1,
                s if s < &2.0 => 2,
                s if s < &5.0 => 3,
                _ => 4,
            };
            unweighted_by_speed[idx].push(*residual);
        }

        // Compute RMS and improvement for each bin
        let compute_bin_improvement = |weighted: &[f32], unweighted: &[f32]| -> (f64, f64, f32) {
            let weighted_rms = if !weighted.is_empty() {
                let sum_sq: f32 = weighted.iter().map(|r| r * r).sum();
                (sum_sq as f64 / weighted.len() as f64).sqrt()
            } else {
                0.0
            };

            let unweighted_rms = if !unweighted.is_empty() {
                let sum_sq: f32 = unweighted.iter().map(|r| r * r).sum();
                (sum_sq as f64 / unweighted.len() as f64).sqrt()
            } else {
                0.0
            };

            let improvement = if unweighted_rms > 0.0 {
                ((unweighted_rms - weighted_rms) / unweighted_rms * 100.0) as f32
            } else {
                0.0
            };

            (weighted_rms, unweighted_rms, improvement)
        };

        improvement.static_scene =
            compute_bin_improvement(&weighted_by_speed[0], &unweighted_by_speed[0]);
        improvement.slow_motion =
            compute_bin_improvement(&weighted_by_speed[1], &unweighted_by_speed[1]);
        improvement.normal_motion =
            compute_bin_improvement(&weighted_by_speed[2], &unweighted_by_speed[2]);
        improvement.fast_motion =
            compute_bin_improvement(&weighted_by_speed[3], &unweighted_by_speed[3]);
        improvement.very_fast_motion =
            compute_bin_improvement(&weighted_by_speed[4], &unweighted_by_speed[4]);

        improvement
    }
}

impl Default for CalibrationAwareAnalyzer {
    fn default() -> Self {
        Self::new()
    }
}
