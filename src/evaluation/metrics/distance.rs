/// Distance-aware accuracy metrics
///
/// Bin metrics by distance from camera (near, mid, far, very far fields).

use super::types::BinMetrics;

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

impl Default for DistanceBinnedMetrics {
    fn default() -> Self {
        Self {
            near_field: BinMetrics::new(),
            mid_field: BinMetrics::new(),
            far_field: BinMetrics::new(),
            very_far_field: BinMetrics::new(),
        }
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

/// Bin sample by distance
pub fn bin_by_distance(distance: f64) -> usize {
    match distance {
        d if d < 1.0 => 0,
        d if d < 3.0 => 1,
        d if d < 10.0 => 2,
        _ => 3,
    }
}
