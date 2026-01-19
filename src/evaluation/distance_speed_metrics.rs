/// Distance-aware and speed-aware evaluation metrics
/// 
/// Measure how accuracy varies with:
/// - Distance from camera (near vs far field)
/// - Speed of motion (static, slow, normal, fast)
/// - Combined distance + speed scenarios

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
