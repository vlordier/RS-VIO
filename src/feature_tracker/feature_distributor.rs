//! Adaptive feature distribution controller
//!
//! Maintains uniform spatial distribution of features across the image
//! using grid-based occupancy tracking and pyramid-aware density targets.

use nalgebra::Point2;
use serde::{Deserialize, Serialize};

/// Configuration for adaptive feature distribution
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DistributionConfig {
    /// Grid cell size (pixels)
    pub grid_cell_size: u32,
    /// Target features per grid cell
    pub target_per_cell: u32,
    /// Minimum features per cell (hard constraint)
    pub min_per_cell: u32,
    /// Maximum features per cell (soft constraint)
    pub max_per_cell: u32,
    /// Pyramid levels to consider
    pub num_pyramid_levels: u32,
    /// Weighting for different levels (0=top, higher=bottom)
    pub level_weight: Vec<f32>,
}

impl Default for DistributionConfig {
    fn default() -> Self {
        Self {
            grid_cell_size: 32,
            target_per_cell: 2,
            min_per_cell: 1,
            max_per_cell: 4,
            num_pyramid_levels: 4,
            level_weight: vec![0.1, 0.3, 0.4, 0.2], // Prefer middle levels
        }
    }
}

/// Grid cell occupancy status
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CellStatus {
    Underoccupied,  // Below target density
    Balanced,       // At target density
    Overoccupied,   // Above target density
    Critical,       // Empty or nearly empty
}

/// Feature distribution analyzer and controller
pub struct FeatureDistributor {
    config: DistributionConfig,
    /// Grid occupancy map (width × height grid of cell counts)
    grid: Vec<Vec<u32>>,
    /// Cell status map
    status_map: Vec<Vec<CellStatus>>,
    /// Width and height of grid
    grid_width: u32,
    grid_height: u32,
    /// Image dimensions
    image_width: u32,
    image_height: u32,
    /// Statistics
    total_features: u32,
    /// Frame tracking for temporal change detection (reserved for future enhancement)
    #[allow(dead_code)] // Will be used for adaptive grid resizing based on scene dynamics
    last_update_frame: u32,
}

impl FeatureDistributor {
    /// Create new feature distributor
    pub fn new(config: DistributionConfig) -> Self {
        Self {
            config,
            grid: Vec::new(),
            status_map: Vec::new(),
            grid_width: 0,
            grid_height: 0,
            image_width: 640,
            image_height: 480,
            total_features: 0,
            last_update_frame: 0,
        }
    }

    /// Initialize grid for given image dimensions
    pub fn initialize(&mut self, width: u32, height: u32) {
        self.image_width = width;
        self.image_height = height;

        self.grid_width = (width + self.config.grid_cell_size - 1) / self.config.grid_cell_size;
        self.grid_height = (height + self.config.grid_cell_size - 1) / self.config.grid_cell_size;

        self.grid = vec![vec![0; self.grid_width as usize]; self.grid_height as usize];
        self.status_map = vec![
            vec![CellStatus::Critical; self.grid_width as usize];
            self.grid_height as usize
        ];
    }

    /// Get grid width
    pub fn grid_width(&self) -> u32 {
        self.grid_width
    }

    /// Get grid height
    pub fn grid_height(&self) -> u32 {
        self.grid_height
    }

    /// Update occupancy based on current feature positions
    pub fn update_occupancy(
        &mut self,
        feature_positions: &[Point2<f64>],
        pyramid_levels: &[u32],
    ) {
        // Reset grid
        for row in &mut self.grid {
            for cell in row {
                *cell = 0;
            }
        }

        self.total_features = feature_positions.len() as u32;

        // Count features in each cell
        for (pos, &level) in feature_positions.iter().zip(pyramid_levels.iter()) {
            let gx = (pos.x as u32 / self.config.grid_cell_size).min(self.grid_width - 1);
            let gy = (pos.y as u32 / self.config.grid_cell_size).min(self.grid_height - 1);

            // Weight by pyramid level
            let weight = if (level as usize) < self.config.level_weight.len() {
                self.config.level_weight[level as usize]
            } else {
                0.1
            };

            self.grid[gy as usize][gx as usize] += (weight * 10.0) as u32; // Scale for integer counting
        }

        // Update status map
        for y in 0..self.grid_height {
            for x in 0..self.grid_width {
                let count = self.grid[y as usize][x as usize];
                self.status_map[y as usize][x as usize] = match count {
                    0 => CellStatus::Critical,
                    c if c < self.config.min_per_cell => CellStatus::Underoccupied,
                    c if c >= self.config.target_per_cell && c <= self.config.max_per_cell => {
                        CellStatus::Balanced
                    }
                    _ => CellStatus::Overoccupied,
                };
            }
        }
    }

    /// Get cells that need feature detection
    pub fn get_detection_regions(&self) -> Vec<(u32, u32)> {
        let mut regions = Vec::new();

        for y in 0..self.grid_height {
            for x in 0..self.grid_width {
                if matches!(
                    self.status_map[y as usize][x as usize],
                    CellStatus::Critical | CellStatus::Underoccupied
                ) {
                    regions.push((x, y));
                }
            }
        }

        // Sort by distance from center (prefer central regions)
        let center_x = self.grid_width as f32 / 2.0;
        let center_y = self.grid_height as f32 / 2.0;

        regions.sort_by(|a, b| {
            let dist_a = (a.0 as f32 - center_x).powi(2) + (a.1 as f32 - center_y).powi(2);
            let dist_b = (b.0 as f32 - center_x).powi(2) + (b.1 as f32 - center_y).powi(2);
            dist_a.partial_cmp(&dist_b).unwrap_or(std::cmp::Ordering::Equal)
        });

        regions
    }

    /// Get cells with overoccupied status (candidates for pruning)
    pub fn get_overcrowded_regions(&self) -> Vec<(u32, u32)> {
        let mut regions = Vec::new();

        for y in 0..self.grid_height {
            for x in 0..self.grid_width {
                if self.status_map[y as usize][x as usize] == CellStatus::Overoccupied {
                    regions.push((x, y));
                }
            }
        }

        regions
    }

    /// Compute adaptive quality threshold based on local density
    pub fn get_adaptive_quality_threshold(&self, cell_x: u32, cell_y: u32) -> f32 {
        if cell_x >= self.grid_width || cell_y >= self.grid_height {
            return 0.01; // Default
        }

        let status = self.status_map[cell_y as usize][cell_x as usize];
        match status {
            CellStatus::Critical => 0.005, // Very permissive in empty regions
            CellStatus::Underoccupied => 0.008,
            CellStatus::Balanced => 0.01,
            CellStatus::Overoccupied => 0.02, // Stricter in crowded regions
        }
    }

    /// Get coverage percentage
    pub fn coverage_percentage(&self) -> f32 {
        let total_cells = self.grid_width * self.grid_height;
        let covered = self
            .status_map
            .iter()
            .flat_map(|row| row.iter())
            .filter(|&&cell| cell != CellStatus::Critical)
            .count() as u32;

        if total_cells == 0 {
            0.0
        } else {
            (covered as f32 / total_cells as f32) * 100.0
        }
    }

    /// Get distribution uniformity metric [0, 1] (1 = perfect)
    pub fn uniformity_metric(&self) -> f32 {
        let mean = self.total_features as f32 / ((self.grid_width * self.grid_height) as f32);

        if mean == 0.0 {
            return 0.0;
        }

        let variance: f32 = self
            .grid
            .iter()
            .flat_map(|row| row.iter())
            .map(|&count| {
                let diff = count as f32 - mean;
                diff * diff
            })
            .sum::<f32>()
            / (self.grid_width * self.grid_height) as f32;

        let std_dev = variance.sqrt();
        let cv = std_dev / mean; // Coefficient of variation

        // Convert to uniformity: 1.0 = perfect, 0.0 = highly non-uniform
        1.0 / (1.0 + cv)
    }

    /// Get statistics about distribution
    pub fn stats(&self) -> DistributionStats {
        let critical_cells = self
            .status_map
            .iter()
            .flat_map(|row| row.iter())
            .filter(|&&cell| cell == CellStatus::Critical)
            .count() as u32;

        let underoccupied = self
            .status_map
            .iter()
            .flat_map(|row| row.iter())
            .filter(|&&cell| cell == CellStatus::Underoccupied)
            .count() as u32;

        let balanced = self
            .status_map
            .iter()
            .flat_map(|row| row.iter())
            .filter(|&&cell| cell == CellStatus::Balanced)
            .count() as u32;

        let overoccupied = self
            .status_map
            .iter()
            .flat_map(|row| row.iter())
            .filter(|&&cell| cell == CellStatus::Overoccupied)
            .count() as u32;

        DistributionStats {
            total_features: self.total_features,
            critical_cells,
            underoccupied_cells: underoccupied,
            balanced_cells: balanced,
            overoccupied_cells: overoccupied,
            coverage_percentage: self.coverage_percentage(),
            uniformity: self.uniformity_metric(),
        }
    }
}

impl Default for FeatureDistributor {
    fn default() -> Self {
        Self::new(DistributionConfig::default())
    }
}

/// Statistics from distribution analysis
#[derive(Debug, Clone, Copy)]
pub struct DistributionStats {
    pub total_features: u32,
    pub critical_cells: u32,
    pub underoccupied_cells: u32,
    pub balanced_cells: u32,
    pub overoccupied_cells: u32,
    pub coverage_percentage: f32,
    pub uniformity: f32,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_distributor_creation() {
        let distributor = FeatureDistributor::new(DistributionConfig::default());
        assert_eq!(distributor.grid.len(), 0);
    }

    #[test]
    fn test_grid_initialization() {
        let mut distributor = FeatureDistributor::new(DistributionConfig::default());
        distributor.initialize(640, 480);

        assert_eq!(distributor.grid_width, 20); // 640 / 32 = 20
        assert_eq!(distributor.grid_height, 15); // 480 / 32 = 15
        assert_eq!(distributor.grid.len(), 15);
    }

    #[test]
    fn test_occupancy_update() {
        let mut config = DistributionConfig::default();
        config.num_pyramid_levels = 1;
        config.level_weight = vec![1.0];

        let mut distributor = FeatureDistributor::new(config);
        distributor.initialize(640, 480);

        let positions = vec![
            Point2::new(50.0, 50.0),
            Point2::new(100.0, 100.0),
        ];
        let levels = vec![0, 0];

        distributor.update_occupancy(&positions, &levels);

        assert_eq!(distributor.total_features, 2);
    }

    #[test]
    fn test_detection_regions() {
        let mut distributor = FeatureDistributor::new(DistributionConfig::default());
        distributor.initialize(640, 480);

        // Add features only at one corner
        let positions = vec![Point2::new(50.0, 50.0)];
        let levels = vec![0];

        distributor.update_occupancy(&positions, &levels);

        let regions = distributor.get_detection_regions();
        // Should identify many empty regions needing detection
        assert!(regions.len() > 0);
    }

    #[test]
    fn test_coverage_percentage() {
        let mut distributor = FeatureDistributor::new(DistributionConfig::default());
        distributor.initialize(640, 480);

        let positions = vec![
            Point2::new(32.0, 32.0),
            Point2::new(96.0, 96.0),
        ];
        let levels = vec![0, 0];

        distributor.update_occupancy(&positions, &levels);

        let coverage = distributor.coverage_percentage();
        assert!(coverage >= 0.0 && coverage <= 100.0);
    }

    #[test]
    fn test_uniformity_metric() {
        let mut distributor = FeatureDistributor::new(DistributionConfig::default());
        distributor.initialize(640, 480);

        let positions = vec![
            Point2::new(32.0, 32.0),
            Point2::new(96.0, 96.0),
        ];
        let levels = vec![0, 0];

        distributor.update_occupancy(&positions, &levels);

        let uniformity = distributor.uniformity_metric();
        assert!(uniformity >= 0.0 && uniformity <= 1.0);
    }

    #[test]
    fn test_adaptive_quality_threshold() {
        let mut distributor = FeatureDistributor::new(DistributionConfig::default());
        distributor.initialize(640, 480);

        let positions = vec![];
        let levels = vec![];

        distributor.update_occupancy(&positions, &levels);

        // Empty cell should have lower threshold
        let empty_threshold = distributor.get_adaptive_quality_threshold(5, 5);
        
        // Add features to another cell
        let positions = vec![Point2::new(200.0, 200.0), Point2::new(205.0, 205.0)];
        let levels = vec![0, 0];

        distributor.update_occupancy(&positions, &levels);

        let crowded_threshold = distributor.get_adaptive_quality_threshold(6, 6);

        // Empty cells should have more permissive threshold
        assert!(empty_threshold < crowded_threshold);
    }

    #[test]
    fn test_distribution_stats() {
        let mut distributor = FeatureDistributor::new(DistributionConfig::default());
        distributor.initialize(640, 480);

        let positions = vec![
            Point2::new(32.0, 32.0),
            Point2::new(96.0, 96.0),
            Point2::new(160.0, 160.0),
        ];
        let levels = vec![0, 0, 0];

        distributor.update_occupancy(&positions, &levels);

        let stats = distributor.stats();
        assert_eq!(stats.total_features, 3);
        assert!(stats.coverage_percentage >= 0.0);
        assert!(stats.uniformity >= 0.0);
    }
}
