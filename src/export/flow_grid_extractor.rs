/// Sparse optical flow grid extraction for teacher export
///
/// Extracts a sparse 8×6 grid of optical flow points from tracked features.
use crate::export::OpticalFlowPoint;

/// Simple feature point representation for flow extraction
#[derive(Clone, Debug)]
pub struct FlowFeaturePoint {
    pub x: f32,
    pub y: f32,
}

/// Extracts sparse optical flow grid from tracked features
pub struct FlowGridExtractor {
    grid_rows: usize,
    grid_cols: usize,
}

impl FlowGridExtractor {
    /// Create new flow grid extractor
    ///
    /// # Arguments
    /// * `grid_rows` - Number of rows in the flow grid (typically 8)
    /// * `grid_cols` - Number of columns in the flow grid (typically 6)
    pub fn new(grid_rows: usize, grid_cols: usize) -> Self {
        Self {
            grid_rows,
            grid_cols,
        }
    }

    /// Extract sparse flow grid from current and previous features
    ///
    /// Divides the image into grid cells and selects one flow point per cell
    /// (or leaves empty if no features in that cell).
    pub fn extract_flow_grid(
        &self,
        image_width: f32,
        image_height: f32,
        current_features: &[(f32, f32)],
        previous_features: &[(f32, f32)],
    ) -> Vec<OpticalFlowPoint> {
        let cell_width = image_width / self.grid_cols as f32;
        let cell_height = image_height / self.grid_rows as f32;

        let mut grid: Vec<Vec<Option<OpticalFlowPoint>>> =
            vec![vec![None; self.grid_cols]; self.grid_rows];

        // Match current features to previous positions and populate grid
        let min_len = current_features.len().min(previous_features.len());
        for idx in 0..min_len {
            let (cx, cy) = current_features[idx];
            let (px, py) = previous_features[idx];

            // Calculate grid cell
            let col = ((cx / cell_width).floor() as usize).min(self.grid_cols - 1);
            let row = ((cy / cell_height).floor() as usize).min(self.grid_rows - 1);

            // Only keep if no point in this cell yet (first one wins)
            if grid[row][col].is_none() {
                grid[row][col] = Some(OpticalFlowPoint {
                    prev: (px, py),
                    curr: (cx, cy),
                });
            }
        }

        // Flatten grid, filtering out None values
        let mut flow_points = Vec::new();
        for row in grid.iter() {
            for point in row.iter() {
                if let Some(p) = point {
                    flow_points.push(p.clone());
                }
            }
        }

        flow_points
    }

    /// Compute flow grid inlier ratio (0-1)
    ///
    /// Ratio of cells with valid flow points to total grid cells.
    pub fn compute_inlier_ratio(flow_points: &[OpticalFlowPoint], total_cells: usize) -> f64 {
        flow_points.len() as f64 / total_cells as f64
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_flow_grid_extractor() {
        let extractor = FlowGridExtractor::new(8, 6);

        // Create dummy features
        let curr_features = vec![(50.0, 50.0), (150.0, 150.0)];

        let prev_features = vec![(48.0, 48.0), (148.0, 148.0)];

        let grid = extractor.extract_flow_grid(512.0, 512.0, &curr_features, &prev_features);
        assert_eq!(grid.len(), 2);

        let inlier_ratio = FlowGridExtractor::compute_inlier_ratio(&grid, 8 * 6);
        assert!(inlier_ratio > 0.0 && inlier_ratio <= 1.0);
    }
}
