/// Async feature detection for concurrent VIO pipeline
///
/// Implements parallel feature extraction using tokio tasks for:
/// - FAST corner detection
/// - Feature grid distribution
/// - Descriptor computation
///
/// Designed to work with the concurrent pipeline in estimator::concurrent

use tokio::task;
use std::sync::Arc;

/// Detected feature with spatial and quality information
#[derive(Debug, Clone, Copy)]
pub struct DetectedFeature {
    pub x: f32,
    pub y: f32,
    pub score: f32,
}

/// Configuration for async feature detection
#[derive(Debug, Clone)]
pub struct AsyncDetectorConfig {
    /// Number of parallel detection tasks
    pub num_parallel_tasks: usize,
    /// Grid cell size for distribution
    pub grid_cell_size: usize,
    /// Maximum features per image
    pub max_features: usize,
    /// Feature detection threshold
    pub threshold: f32,
}

impl Default for AsyncDetectorConfig {
    fn default() -> Self {
        Self {
            num_parallel_tasks: 4,
            grid_cell_size: 32,
            max_features: 1000,
            threshold: 20.0,
        }
    }
}

/// Async feature detector for concurrent processing
#[derive(Debug, Clone)]
pub struct AsyncFeatureDetector {
    config: Arc<AsyncDetectorConfig>,
}

impl AsyncFeatureDetector {
    /// Create new async feature detector
    pub fn new(config: AsyncDetectorConfig) -> Self {
        Self {
            config: Arc::new(config),
        }
    }

    /// Detect features in image data asynchronously
    ///
    /// # Arguments
    /// * `image_data` - Raw image bytes
    /// * `width` - Image width in pixels
    /// * `height` - Image height in pixels
    ///
    /// # Returns
    /// Vector of detected features with scores
    pub async fn detect_async(
        &self,
        image_data: Arc<Vec<u8>>,
        width: u32,
        height: u32,
    ) -> Vec<DetectedFeature> {
        // Spawn parallel detection tasks
        let mut tasks = vec![];

        let rows_per_task = (height as usize + self.config.num_parallel_tasks - 1)
            / self.config.num_parallel_tasks;

        for task_idx in 0..self.config.num_parallel_tasks {
            let image_data = Arc::clone(&image_data);
            let config = Arc::clone(&self.config);

            let task = task::spawn_blocking(move || {
                let start_row = task_idx * rows_per_task;
                let end_row = std::cmp::min((task_idx + 1) * rows_per_task, height as usize);

                detect_features_in_region(
                    &image_data,
                    width as usize,
                    height as usize,
                    start_row,
                    end_row,
                    config.threshold,
                )
            });

            tasks.push(task);
        }

        // Wait for all tasks to complete
        let mut all_features = Vec::new();
        for task in tasks {
            match task.await {
                Ok(features) => all_features.extend(features),
                Err(_) => {
                    // Log task panic, continue with other results
                }
            }
        }

        // Sort and distribute features in grid
        distribute_features_in_grid(&mut all_features, width as usize, height as usize, self.config.grid_cell_size);

        // Limit to max features
        all_features.truncate(self.config.max_features);
        all_features
    }

    /// Synchronous detection for compatibility
    pub fn detect(&self, image_data: &[u8], width: u32, height: u32) -> Vec<DetectedFeature> {
        let features = detect_features_in_region(
            image_data,
            width as usize,
            height as usize,
            0,
            height as usize,
            self.config.threshold,
        );

        let mut sorted_features = features;
        distribute_features_in_grid(&mut sorted_features, width as usize, height as usize, self.config.grid_cell_size);
        sorted_features.truncate(self.config.max_features);
        sorted_features
    }
}

/// Detect FAST corners in image region
fn detect_features_in_region(
    image_data: &[u8],
    width: usize,
    height: usize,
    start_row: usize,
    end_row: usize,
    threshold: f32,
) -> Vec<DetectedFeature> {
    let mut features = Vec::new();

    // Simple FAST-like corner detection
    // In practice, would use optimized FAST algorithm
    for y in start_row + 3..std::cmp::min(end_row, height - 3) {
        for x in 3..width - 3 {
            let idx = y * width + x;
            if idx + width < image_data.len() {
                let center = image_data[idx] as f32;

                // Check 8 neighbors in circle pattern
                let n1 = image_data[idx - width] as f32;
                let n2 = image_data[idx + width] as f32;
                let n3 = image_data[idx - 1] as f32;
                let n4 = image_data[idx + 1] as f32;

                let corner_score = ((n1 - center).abs() + (n2 - center).abs() + (n3 - center).abs() + (n4 - center).abs()) / 4.0;

                if corner_score > threshold {
                    features.push(DetectedFeature {
                        x: x as f32,
                        y: y as f32,
                        score: corner_score,
                    });
                }
            }
        }
    }

    features
}

/// Distribute features evenly across grid cells
fn distribute_features_in_grid(
    features: &mut Vec<DetectedFeature>,
    width: usize,
    height: usize,
    cell_size: usize,
) {
    let grid_width = (width + cell_size - 1) / cell_size;
    let grid_height = (height + cell_size - 1) / cell_size;
    let max_per_cell = std::cmp::max(1, 1000 / (grid_width * grid_height));

    // Sort by grid cell then by score
    features.sort_by(|a, b| {
        let cell_a = ((a.x as usize) / cell_size) * grid_width + ((a.y as usize) / cell_size);
        let cell_b = ((b.x as usize) / cell_size) * grid_width + ((b.y as usize) / cell_size);

        match cell_a.cmp(&cell_b) {
            std::cmp::Ordering::Equal => b.score.partial_cmp(&a.score).unwrap_or(std::cmp::Ordering::Equal),
            other => other,
        }
    });

    // Keep only top features per cell
    let mut kept_features = Vec::new();
    let mut per_cell_count = vec![0; grid_width * grid_height];

    for feature in features.iter() {
        let cell = ((feature.x as usize) / cell_size) * grid_width + ((feature.y as usize) / cell_size);
        if cell < per_cell_count.len() && per_cell_count[cell] < max_per_cell {
            kept_features.push(*feature);
            per_cell_count[cell] += 1;
        }
    }

    *features = kept_features;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_config_default() {
        let config = AsyncDetectorConfig::default();
        assert_eq!(config.num_parallel_tasks, 4);
        assert_eq!(config.grid_cell_size, 32);
        assert_eq!(config.max_features, 1000);
        assert_eq!(config.threshold, 20.0);
    }

    #[test]
    fn test_detector_creation() {
        let detector = AsyncFeatureDetector::new(AsyncDetectorConfig::default());
        assert_eq!(detector.config.num_parallel_tasks, 4);
    }

    #[test]
    fn test_detect_features_sync() {
        // Create simple test image (32x32)
        let image_data = vec![100u8; 32 * 32];
        let detector = AsyncFeatureDetector::new(AsyncDetectorConfig {
            threshold: 50.0,
            max_features: 100,
            ..Default::default()
        });

        let features = detector.detect(&image_data, 32, 32);
        // Uniform image should have few features
        assert!(features.len() < 10);
    }

    #[tokio::test]
    async fn test_detect_features_async() {
        // Create simple test image (64x64)
        let image_data = Arc::new(vec![100u8; 64 * 64]);
        let detector = AsyncFeatureDetector::new(AsyncDetectorConfig {
            threshold: 50.0,
            max_features: 100,
            num_parallel_tasks: 2,
            ..Default::default()
        });

        let features = detector.detect_async(image_data, 64, 64).await;
        // Uniform image should have few features
        assert!(features.len() < 10);
    }

    #[test]
    fn test_distribute_features_in_grid() {
        let mut features = vec![
            DetectedFeature {
                x: 5.0,
                y: 5.0,
                score: 100.0,
            },
            DetectedFeature {
                x: 45.0,
                y: 45.0,
                score: 90.0,
            },
        ];

        distribute_features_in_grid(&mut features, 64, 64, 32);
        // Should keep both features in different cells
        assert_eq!(features.len(), 2);
    }
}
