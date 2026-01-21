/// Distance-speed combined analysis
///
/// Analyzer for measuring accuracy effects of distance and speed independently and together.
use super::distance::DistanceBinnedMetrics;
use super::speed::SpeedBinnedMetrics;
use super::trajectory::DistanceSpeedAnalysis;
use super::types::BinMetrics;

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
        reprojection_by_distance
            .very_far_field
            .finalize(&very_far_repr);

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
        reprojection_by_speed
            .very_fast_motion
            .finalize(&very_fast_repr);

        DistanceSpeedAnalysis {
            depth_by_distance,
            depth_by_speed,
            reprojection_by_distance,
            reprojection_by_speed,
            distance_speed_matrix: Default::default(),
            worst_cases: Vec::new(),
        }
    }
}

impl Default for DistanceSpeedAnalyzer {
    fn default() -> Self {
        Self::new()
    }
}
