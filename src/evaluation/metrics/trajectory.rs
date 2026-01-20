/// Trajectory analysis metrics
///
/// Analyzes accuracy across combined distance and speed scenarios.

use super::distance::DistanceBinnedMetrics;
use super::speed::SpeedBinnedMetrics;
use super::types::BinMetrics;

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
                [
                    BinMetrics::new(),
                    BinMetrics::new(),
                    BinMetrics::new(),
                    BinMetrics::new(),
                    BinMetrics::new(),
                ],
                [
                    BinMetrics::new(),
                    BinMetrics::new(),
                    BinMetrics::new(),
                    BinMetrics::new(),
                    BinMetrics::new(),
                ],
                [
                    BinMetrics::new(),
                    BinMetrics::new(),
                    BinMetrics::new(),
                    BinMetrics::new(),
                    BinMetrics::new(),
                ],
                [
                    BinMetrics::new(),
                    BinMetrics::new(),
                    BinMetrics::new(),
                    BinMetrics::new(),
                    BinMetrics::new(),
                ],
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

/// Worst-case distance+speed combination
#[derive(Clone, Debug)]
pub struct WorstCase {
    pub distance: f64,
    pub speed: f32,
    pub error: f32,
    pub combination: String,
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
