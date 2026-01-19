/// Distance-aware and speed-aware evaluation metrics
/// 
/// Measure how accuracy varies with:
/// - Distance from camera (near vs far field)
/// - Speed of motion (static, slow, normal, fast)
/// - Combined distance + speed scenarios

/// Calibration-aware performance metrics
/// Tracks how calibration quality affects tracking accuracy across distance/speed bins
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

/// Per-distance-bin improvement metrics
#[derive(Clone, Debug)]
pub struct DistanceBinnedImprovement {
    /// Near field (0-1m): (weighted, unweighted, improvement %)
    pub near_field: (f64, f64, f32),
    
    /// Mid field (1-3m)
    pub mid_field: (f64, f64, f32),
    
    /// Far field (3-10m)
    pub far_field: (f64, f64, f32),
    
    /// Very far (10m+)
    pub very_far_field: (f64, f64, f32),
}

impl DistanceBinnedImprovement {
    pub fn new() -> Self {
        Self {
            near_field: (0.0, 0.0, 0.0),
            mid_field: (0.0, 0.0, 0.0),
            far_field: (0.0, 0.0, 0.0),
            very_far_field: (0.0, 0.0, 0.0),
        }
    }
}

impl Default for DistanceBinnedImprovement {
    fn default() -> Self {
        Self::new()
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

/// Accuracy metrics binned by distance
#[derive(Clone, Debug)]
pub struct DistanceBinnedMetrics {
    /// Near field (0-1m)
    pub near_field: BinMetrics,
    
    /// Mid field (1-3m)
    pub mid_field: BinMetrics,
    
    /// Far field (3-10m)
    pub far_field: BinMetrics,
    
    /// Very far (10m+)
    pub very_far_field: BinMetrics,
}

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

/// Per-bin accuracy metrics
#[derive(Clone, Debug)]
pub struct BinMetrics {
    /// Mean accuracy/error
    pub mean: f32,
    
    /// Standard deviation
    pub std: f32,
    
    /// Min value in bin
    pub min: f32,
    
    /// Max value in bin
    pub max: f32,
    
    /// Median value
    pub median: f32,
    
    /// Sample count
    pub count: usize,
    
    /// Percentile 95
    pub percentile_95: f32,
}

impl BinMetrics {
    pub fn new() -> Self {
        Self {
            mean: 0.0,
            std: 0.0,
            min: f32::INFINITY,
            max: f32::NEG_INFINITY,
            median: 0.0,
            count: 0,
            percentile_95: 0.0,
        }
    }
    
    /// Update metrics with new sample
    pub fn add_sample(&mut self, value: f32) {
        self.count += 1;
        self.min = self.min.min(value);
        self.max = self.max.max(value);
    }
    
    /// Finalize statistics from all samples
    pub fn finalize(&mut self, samples: &[f32]) {
        if samples.is_empty() {
            return;
        }
        
        self.count = samples.len();
        self.mean = samples.iter().sum::<f32>() / samples.len() as f32;
        self.std = (samples.iter()
            .map(|&x| (x - self.mean).powi(2))
            .sum::<f32>() / samples.len() as f32)
            .sqrt();
        
        let mut sorted = samples.to_vec();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
        
        self.median = sorted[sorted.len() / 2];
        self.percentile_95 = sorted[(sorted.len() as f32 * 0.95) as usize];
    }
}

impl Default for BinMetrics {
    fn default() -> Self {
        Self::new()
    }
}

/// Combined distance+speed analysis
#[derive(Clone, Debug)]
pub struct DistanceSpeedMatrix {
    /// 4x5 matrix: 4 distance bins x 5 speed bins
    pub accuracy_matrix: [[BinMetrics; 5]; 4],
}

impl DistanceSpeedMatrix {
    pub fn new() -> Self {
        Self {
            accuracy_matrix: [
                [BinMetrics::new(), BinMetrics::new(), BinMetrics::new(), BinMetrics::new(), BinMetrics::new()],
                [BinMetrics::new(), BinMetrics::new(), BinMetrics::new(), BinMetrics::new(), BinMetrics::new()],
                [BinMetrics::new(), BinMetrics::new(), BinMetrics::new(), BinMetrics::new(), BinMetrics::new()],
                [BinMetrics::new(), BinMetrics::new(), BinMetrics::new(), BinMetrics::new(), BinMetrics::new()],
            ],
        }
    }
    
    /// Add sample to appropriate distance+speed bin
    pub fn add_sample(&mut self, distance: f64, speed: f32, error: f32) {
        let distance_idx = match distance {
            d if d < 1.0 => 0,
            d if d < 3.0 => 1,
            d if d < 10.0 => 2,
            _ => 3,
        };
        
        let speed_idx = match speed {
            s if s < 0.1 => 0,
            s if s < 0.5 => 1,
            s if s < 2.0 => 2,
            s if s < 5.0 => 3,
            _ => 4,
        };
        
        self.accuracy_matrix[distance_idx][speed_idx].add_sample(error);
    }
}

impl Default for DistanceSpeedMatrix {
    fn default() -> Self {
        Self::new()
    }
}

/// Comprehensive distance/speed analysis
#[derive(Clone, Debug)]
pub struct DistanceSpeedAnalysis {
    /// 3D point accuracy by distance
    pub depth_by_distance: DistanceBinnedMetrics,
    
    /// 3D point accuracy by speed
    pub depth_by_speed: SpeedBinnedMetrics,
    
    /// 2D reprojection error by distance
    pub reprojection_by_distance: DistanceBinnedMetrics,
    
    /// 2D reprojection error by speed
    pub reprojection_by_speed: SpeedBinnedMetrics,
    
    /// Joint distance-speed matrix
    pub distance_speed_matrix: DistanceSpeedMatrix,
    
    /// Worst-case combinations (distance + speed)
    pub worst_cases: Vec<WorstCase>,
}

#[derive(Clone, Debug)]
pub struct WorstCase {
    pub distance: f64,
    pub speed: f32,
    pub error: f32,
    pub combination: String,
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
    pub fn record_weighted_visual_residual(
        &mut self,
        distance: f64,
        speed: f32,
        residual: f32,
    ) {
        self.weighted_visual_samples.push((distance, speed, residual));
    }
    
    /// Record an unweighted visual residual at distance/speed
    pub fn record_unweighted_visual_residual(
        &mut self,
        distance: f64,
        speed: f32,
        residual: f32,
    ) {
        self.unweighted_visual_samples.push((distance, speed, residual));
    }
    
    /// Record a weighted IMU residual at distance/speed
    pub fn record_weighted_imu_residual(
        &mut self,
        distance: f64,
        speed: f32,
        residual: f32,
    ) {
        self.weighted_imu_samples.push((distance, speed, residual));
    }
    
    /// Record an unweighted IMU residual at distance/speed
    pub fn record_unweighted_imu_residual(
        &mut self,
        distance: f64,
        speed: f32,
        residual: f32,
    ) {
        self.unweighted_imu_samples.push((distance, speed, residual));
    }
    
    /// Record feature track length
    pub fn record_track_length(&mut self, length: usize) {
        self.track_lengths.push(length);
    }
    
    /// Compute comprehensive calibration-aware metrics
    pub fn analyze(&self) -> CalibrationAwareMetrics {
        let weighted_residuals = self.compute_residual_stats(
            &self.weighted_visual_samples,
            &self.weighted_imu_samples,
        );
        let unweighted_residuals = self.compute_residual_stats(
            &self.unweighted_visual_samples,
            &self.unweighted_imu_samples,
        );
        let track_survival = self.compute_track_survival();
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
    
    /// Compute residual statistics from samples
    fn compute_residual_stats(
        &self,
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
    fn compute_track_survival(&self) -> TrackSurvivalStats {
        if self.track_lengths.is_empty() {
            return TrackSurvivalStats::default();
        }
        
        let mut sorted = self.track_lengths.clone();
        sorted.sort();
        
        let median_track_length = sorted[sorted.len() / 2] as f32;
        let mean_track_length =
            self.track_lengths.iter().sum::<usize>() as f32 / self.track_lengths.len() as f32;
        let max_track_length = *sorted.last().unwrap_or(&0);
        
        let survival_rate_5 = self
            .track_lengths
            .iter()
            .filter(|&&len| len > 5)
            .count() as f32
            / self.track_lengths.len() as f32
            * 100.0;
        
        let survival_rate_10 = self
            .track_lengths
            .iter()
            .filter(|&&len| len > 10)
            .count() as f32
            / self.track_lengths.len() as f32
            * 100.0;
        
        TrackSurvivalStats {
            median_track_length,
            mean_track_length,
            max_track_length,
            survival_rate_5,
            survival_rate_10,
            total_features: self.track_lengths.len(),
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
        
        improvement.near_field = compute_bin_improvement(&weighted_by_distance[0], &unweighted_by_distance[0]);
        improvement.mid_field = compute_bin_improvement(&weighted_by_distance[1], &unweighted_by_distance[1]);
        improvement.far_field = compute_bin_improvement(&weighted_by_distance[2], &unweighted_by_distance[2]);
        improvement.very_far_field = compute_bin_improvement(&weighted_by_distance[3], &unweighted_by_distance[3]);
        
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
        
        improvement.static_scene = compute_bin_improvement(&weighted_by_speed[0], &unweighted_by_speed[0]);
        improvement.slow_motion = compute_bin_improvement(&weighted_by_speed[1], &unweighted_by_speed[1]);
        improvement.normal_motion = compute_bin_improvement(&weighted_by_speed[2], &unweighted_by_speed[2]);
        improvement.fast_motion = compute_bin_improvement(&weighted_by_speed[3], &unweighted_by_speed[3]);
        improvement.very_fast_motion = compute_bin_improvement(&weighted_by_speed[4], &unweighted_by_speed[4]);
        
        improvement
    }
}

impl Default for CalibrationAwareAnalyzer {
    fn default() -> Self {
        Self::new()
    }
}

/// Analyzer for distance and speed effects
pub struct DistanceSpeedAnalyzer {
    depth_distance_samples: Vec<(f64, f32)>,
    depth_speed_samples: Vec<(f32, f32)>,
    reprojection_distance_samples: Vec<(f64, f32)>,
    reprojection_speed_samples: Vec<(f32, f32)>,
}

impl DistanceSpeedAnalyzer {
    pub fn new() -> Self {
        Self {
            depth_distance_samples: Vec::new(),
            depth_speed_samples: Vec::new(),
            reprojection_distance_samples: Vec::new(),
            reprojection_speed_samples: Vec::new(),
        }
    }
    
    /// Record depth accuracy at specific distance
    pub fn record_depth_at_distance(&mut self, distance: f64, error: f32) {
        self.depth_distance_samples.push((distance, error));
    }
    
    /// Record depth accuracy at specific speed
    pub fn record_depth_at_speed(&mut self, speed: f32, error: f32) {
        self.depth_speed_samples.push((speed, error));
    }
    
    /// Record reprojection accuracy at specific distance
    pub fn record_reprojection_at_distance(&mut self, distance: f64, error: f32) {
        self.reprojection_distance_samples.push((distance, error));
    }
    
    /// Record reprojection accuracy at specific speed
    pub fn record_reprojection_at_speed(&mut self, speed: f32, error: f32) {
        self.reprojection_speed_samples.push((speed, error));
    }
    
    /// Compute comprehensive analysis
    pub fn analyze(&self) -> DistanceSpeedAnalysis {
        let mut depth_by_distance = DistanceBinnedMetrics {
            near_field: BinMetrics::new(),
            mid_field: BinMetrics::new(),
            far_field: BinMetrics::new(),
            very_far_field: BinMetrics::new(),
        };
        
        // Bin depth samples by distance
        let mut near_field_depth = Vec::new();
        let mut mid_field_depth = Vec::new();
        let mut far_field_depth = Vec::new();
        let mut very_far_depth = Vec::new();
        
        for (distance, error) in &self.depth_distance_samples {
            match distance {
                d if d < &1.0 => near_field_depth.push(*error),
                d if d < &3.0 => mid_field_depth.push(*error),
                d if d < &10.0 => far_field_depth.push(*error),
                _ => very_far_depth.push(*error),
            }
        }
        
        depth_by_distance.near_field.finalize(&near_field_depth);
        depth_by_distance.mid_field.finalize(&mid_field_depth);
        depth_by_distance.far_field.finalize(&far_field_depth);
        depth_by_distance.very_far_field.finalize(&very_far_depth);
        
        // Similar binning for speed
        let mut depth_by_speed = SpeedBinnedMetrics {
            static_scene: BinMetrics::new(),
            slow_motion: BinMetrics::new(),
            normal_motion: BinMetrics::new(),
            fast_motion: BinMetrics::new(),
            very_fast_motion: BinMetrics::new(),
        };
        
        let mut static_depth = Vec::new();
        let mut slow_depth = Vec::new();
        let mut normal_depth = Vec::new();
        let mut fast_depth = Vec::new();
        let mut very_fast_depth = Vec::new();
        
        for (speed, error) in &self.depth_speed_samples {
            match speed {
                s if s < &0.1 => static_depth.push(*error),
                s if s < &0.5 => slow_depth.push(*error),
                s if s < &2.0 => normal_depth.push(*error),
                s if s < &5.0 => fast_depth.push(*error),
                _ => very_fast_depth.push(*error),
            }
        }
        
        depth_by_speed.static_scene.finalize(&static_depth);
        depth_by_speed.slow_motion.finalize(&slow_depth);
        depth_by_speed.normal_motion.finalize(&normal_depth);
        depth_by_speed.fast_motion.finalize(&fast_depth);
        depth_by_speed.very_fast_motion.finalize(&very_fast_depth);
        
        // Reprojection by distance
        let mut reprojection_by_distance = DistanceBinnedMetrics {
            near_field: BinMetrics::new(),
            mid_field: BinMetrics::new(),
            far_field: BinMetrics::new(),
            very_far_field: BinMetrics::new(),
        };
        
        let mut near_repr = Vec::new();
        let mut mid_repr = Vec::new();
        let mut far_repr = Vec::new();
        let mut very_far_repr = Vec::new();
        
        for (distance, error) in &self.reprojection_distance_samples {
            match distance {
                d if d < &1.0 => near_repr.push(*error),
                d if d < &3.0 => mid_repr.push(*error),
                d if d < &10.0 => far_repr.push(*error),
                _ => very_far_repr.push(*error),
            }
        }
        
        reprojection_by_distance.near_field.finalize(&near_repr);
        reprojection_by_distance.mid_field.finalize(&mid_repr);
        reprojection_by_distance.far_field.finalize(&far_repr);
        reprojection_by_distance.very_far_field.finalize(&very_far_repr);
        
        // Reprojection by speed
        let mut reprojection_by_speed = SpeedBinnedMetrics {
            static_scene: BinMetrics::new(),
            slow_motion: BinMetrics::new(),
            normal_motion: BinMetrics::new(),
            fast_motion: BinMetrics::new(),
            very_fast_motion: BinMetrics::new(),
        };
        
        let mut static_repr = Vec::new();
        let mut slow_repr = Vec::new();
        let mut normal_repr = Vec::new();
        let mut fast_repr = Vec::new();
        let mut very_fast_repr = Vec::new();
        
        for (speed, error) in &self.reprojection_speed_samples {
            match speed {
                s if s < &0.1 => static_repr.push(*error),
                s if s < &0.5 => slow_repr.push(*error),
                s if s < &2.0 => normal_repr.push(*error),
                s if s < &5.0 => fast_repr.push(*error),
                _ => very_fast_repr.push(*error),
            }
        }
        
        reprojection_by_speed.static_scene.finalize(&static_repr);
        reprojection_by_speed.slow_motion.finalize(&slow_repr);
        reprojection_by_speed.normal_motion.finalize(&normal_repr);
        reprojection_by_speed.fast_motion.finalize(&fast_repr);
        reprojection_by_speed.very_fast_motion.finalize(&very_fast_repr);
        
        DistanceSpeedAnalysis {
            depth_by_distance,
            depth_by_speed,
            reprojection_by_distance,
            reprojection_by_speed,
            distance_speed_matrix: DistanceSpeedMatrix::new(),
            worst_cases: Vec::new(),
        }
    }
}

impl Default for DistanceSpeedAnalyzer {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_calibration_aware_metrics() {
        let mut analyzer = CalibrationAwareAnalyzer::new();
        
        // Simulate good calibration: weighted residuals significantly lower
        // Near field, static scene
        analyzer.record_weighted_visual_residual(0.5, 0.05, 0.1);
        analyzer.record_unweighted_visual_residual(0.5, 0.05, 0.25);
        
        // Far field, fast motion
        analyzer.record_weighted_visual_residual(8.0, 3.0, 0.4);
        analyzer.record_unweighted_visual_residual(8.0, 3.0, 0.8);
        
        let metrics = analyzer.analyze();
        
        // Weighted RMS should be significantly lower than unweighted
        assert!(metrics.weighted_residuals.visual_rms < metrics.unweighted_residuals.visual_rms);
        assert!(metrics.unweighted_residuals.visual_rms > 0.0);
    }
    
    #[test]
    fn test_track_survival_computation() {
        let mut analyzer = CalibrationAwareAnalyzer::new();
        
        // Record various track lengths
        for _ in 0..3 {
            analyzer.record_track_length(3);  // 3 frames
        }
        for _ in 0..5 {
            analyzer.record_track_length(8);  // 8 frames
        }
        for _ in 0..2 {
            analyzer.record_track_length(15); // 15 frames
        }
        
        let metrics = analyzer.analyze();
        let survival = &metrics.track_survival;
        
        assert_eq!(survival.total_features, 10);
        assert!(survival.median_track_length > 0.0);
        assert!(survival.survival_rate_5 > 0.0);  // Some features last > 5 frames
        assert!(survival.survival_rate_10 > 0.0); // Some features last > 10 frames
    }
    
    #[test]
    fn test_distance_bin_improvements() {
        let mut analyzer = CalibrationAwareAnalyzer::new();
        
        // Near field: good improvement
        analyzer.record_weighted_visual_residual(0.5, 0.5, 0.1);
        analyzer.record_unweighted_visual_residual(0.5, 0.5, 0.3);
        
        // Far field: less improvement
        analyzer.record_weighted_visual_residual(12.0, 0.5, 0.5);
        analyzer.record_unweighted_visual_residual(12.0, 0.5, 0.6);
        
        let metrics = analyzer.analyze();
        let improvements = &metrics.distance_bin_improvements;
        
        // Near field should show improvement
        assert!(improvements.near_field.2 > 0.0); // Improvement %
        
        // Far field should show less improvement (or none)
        assert!(improvements.very_far_field.2 >= 0.0);
        
        // Weighted RMS should be less than unweighted in both bins
        assert!(improvements.near_field.0 < improvements.near_field.1);
    }
    
    #[test]
    fn test_speed_bin_improvements() {
        let mut analyzer = CalibrationAwareAnalyzer::new();
        
        // Static scene: good residuals
        analyzer.record_weighted_visual_residual(2.0, 0.05, 0.2);
        analyzer.record_unweighted_visual_residual(2.0, 0.05, 0.4);
        
        // Fast motion: worse residuals
        analyzer.record_weighted_visual_residual(2.0, 4.0, 0.4);
        analyzer.record_unweighted_visual_residual(2.0, 4.0, 0.7);
        
        let metrics = analyzer.analyze();
        let improvements = &metrics.speed_bin_improvements;
        
        // Both should show improvement
        assert!(improvements.static_scene.2 > 0.0);
        assert!(improvements.fast_motion.2 > 0.0);
        
        // Weighted RMS should be better than unweighted
        assert!(improvements.static_scene.0 < improvements.static_scene.1);
        assert!(improvements.fast_motion.0 < improvements.fast_motion.1);
    }
    
    #[test]
    fn test_outlier_detection() {
        let mut analyzer = CalibrationAwareAnalyzer::new();
        
        // Most good residuals (value 0.1)
        for _ in 0..9 {
            analyzer.record_weighted_visual_residual(2.0, 1.0, 0.1);
        }
        
        // One much larger outlier (10x the typical value)
        analyzer.record_weighted_visual_residual(2.0, 1.0, 1.0);
        
        let metrics = analyzer.analyze();
        
        // Should detect outlier rate (should be > 0% when there's a clear outlier)
        // With 10 samples, median is 0.1, 3x median threshold = 0.3
        // The 1.0 value exceeds this, so outlier rate should be 10%
        assert!(metrics.weighted_residuals.outlier_rate >= 5.0, 
            "Expected outlier rate >= 5%, got {}", metrics.weighted_residuals.outlier_rate);
    }
    
    #[test]
    fn test_distance_binning() {
        let mut analyzer = DistanceSpeedAnalyzer::new();
        
        analyzer.record_depth_at_distance(0.5, 0.02);
        analyzer.record_depth_at_distance(2.0, 0.04);
        analyzer.record_depth_at_distance(5.0, 0.08);
        analyzer.record_depth_at_distance(15.0, 0.20);
        
        let analysis = analyzer.analyze();
        assert!(analysis.depth_by_distance.near_field.mean < analysis.depth_by_distance.very_far_field.mean);
    }
    
    #[test]
    fn test_speed_binning() {
        let mut analyzer = DistanceSpeedAnalyzer::new();
        
        analyzer.record_depth_at_speed(0.05, 0.01);
        analyzer.record_depth_at_speed(0.3, 0.02);
        analyzer.record_depth_at_speed(1.0, 0.04);
        analyzer.record_depth_at_speed(3.0, 0.08);
        analyzer.record_depth_at_speed(6.0, 0.15);
        
        let analysis = analyzer.analyze();
        assert!(analysis.depth_by_speed.static_scene.mean < analysis.depth_by_speed.very_fast_motion.mean);
    }
}
