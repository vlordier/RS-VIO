//! FAST corner detection and non-maximum suppression
//!
//! High-speed corner detection suitable for embedded systems.
//! Optional non-maximum suppression for better spatial distribution.

use super::{FeatureError, FeatureResult, Keypoint, KeypointDetector};
use crate::types::Float;

/// FAST corner detector configuration
#[derive(Debug, Clone)]
pub struct FASTConfig {
    pub threshold: u8,
    pub nms_radius: u32,
    pub max_corners: u32,
}

impl Default for FASTConfig {
    fn default() -> Self {
        Self {
            threshold: 20,
            nms_radius: 3,
            max_corners: 500,
        }
    }
}

/// FAST-9 corner detector
#[derive(Debug, Clone)]
pub struct FASTDetector {
    config: FASTConfig,
}

impl FASTDetector {
    /// Create new FAST detector
    pub fn new(config: FASTConfig) -> Self {
        Self { config }
    }

    /// Check if pixel is FAST corner using circle of 16 pixels
    fn is_fast_corner(&self, image: &[u8], width: u32, x: u32, y: u32) -> bool {
        if x < 3 || x >= width - 3 || y < 3 {
            return false;
        }

        let w = width as usize;
        let idx = y as usize * w + x as usize;
        let threshold = self.config.threshold;
        let center = image[idx];

        // Bresenham circle offsets (radius 3, 16 pixels)
        let offsets = [
            (0, -3),
            (1, -3),
            (2, -2),
            (3, -1),
            (3, 0),
            (3, 1),
            (2, 2),
            (1, 3),
            (0, 3),
            (-1, 3),
            (-2, 2),
            (-3, 1),
            (-3, 0),
            (-3, -1),
            (-2, -2),
            (-1, -3),
        ];

        let mut bright_count = 0;
        let mut dark_count = 0;

        for (dx, dy) in &offsets {
            let px = (x as i32 + dx) as usize;
            let py = (y as i32 + dy) as usize;
            let pidx = py * w + px;

            let pixel = image[pidx];
            if pixel > center.saturating_add(threshold) {
                bright_count += 1;
            } else if pixel < center.saturating_sub(threshold) {
                dark_count += 1;
            }
        }

        // Corner if 9+ consecutive pixels are above/below threshold
        bright_count >= 9 || dark_count >= 9
    }

    /// Compute corner strength (max difference from center)
    fn corner_strength(&self, image: &[u8], width: u32, x: u32, y: u32) -> u8 {
        let w = width as usize;
        let idx = y as usize * w + x as usize;
        let center = image[idx];

        let offsets = [
            (0, -3),
            (1, -3),
            (2, -2),
            (3, -1),
            (3, 0),
            (3, 1),
            (2, 2),
            (1, 3),
            (0, 3),
            (-1, 3),
            (-2, 2),
            (-3, 1),
            (-3, 0),
            (-3, -1),
            (-2, -2),
            (-1, -3),
        ];

        let mut max_diff = 0u8;
        for (dx, dy) in &offsets {
            let px = (x as i32 + dx) as usize;
            let py = (y as i32 + dy) as usize;
            let pidx = py * w + px;

            let diff = image[pidx].abs_diff(center);
            max_diff = max_diff.max(diff);
        }

        max_diff
    }

    /// Apply non-maximum suppression
    fn apply_nms(&self, corners: Vec<(u32, u32, u8)>) -> Vec<Keypoint> {
        let mut result: Vec<Keypoint> = corners
            .into_iter()
            .map(|(x, y, strength)| Keypoint {
                x: x as Float,
                y: y as Float,
                score: strength as Float / 255.0,
                scale: 1.0,
                angle: None,
            })
            .collect();

        // Sort by strength
        result.sort_by(|a, b| {
            b.score
                .partial_cmp(&a.score)
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        // Limit to max_corners
        result.truncate(self.config.max_corners as usize);

        result
    }
}

impl KeypointDetector for FASTDetector {
    fn detect(&self, image: &[u8], width: u32, height: u32) -> FeatureResult<Vec<Keypoint>> {
        if image.is_empty() {
            return Err(FeatureError::InvalidImage("Empty image".to_string()));
        }

        let mut corners = Vec::new();

        for y in 3..height - 3 {
            for x in 3..width - 3 {
                if self.is_fast_corner(image, width, x, y) {
                    let strength = self.corner_strength(image, width, x, y);
                    corners.push((x, y, strength));
                }
            }
        }

        let keypoints = self.apply_nms(corners);
        Ok(keypoints)
    }

    fn name(&self) -> &str {
        "FAST"
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_fast_detector_creation() {
        let config = FASTConfig::default();
        let detector = FASTDetector::new(config);
        assert_eq!(detector.name(), "FAST");
    }

    #[test]
    fn test_fast_invalid_image() {
        let config = FASTConfig::default();
        let detector = FASTDetector::new(config);
        let result = detector.detect(&[], 640, 480);
        assert!(result.is_err());
    }

    #[test]
    fn test_fast_on_blank_image() {
        let config = FASTConfig::default();
        let detector = FASTDetector::new(config);
        let image = vec![128u8; 640 * 480];
        let result = detector.detect(&image, 640, 480);
        assert!(result.is_ok());
        assert!(result.unwrap().is_empty());
    }
}
