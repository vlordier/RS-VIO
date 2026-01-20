//! Common types and structures for ORB feature extraction

use na::Vector2;
use nalgebra as na;
use std::f64::consts::PI;

/// ORB descriptor configuration
#[derive(Debug, Clone)]
pub struct OrbConfig {
    /// Number of features to extract
    pub num_features: usize,
    /// Scale factor between pyramid levels (typically 1.2)
    pub scale_factor: f64,
    /// Number of pyramid levels
    pub num_levels: usize,
    /// Patch size for orientation computation
    pub patch_size: usize,
    /// Whether to use multi-scale extraction
    pub use_pyramid: bool,
}

impl Default for OrbConfig {
    fn default() -> Self {
        Self {
            num_features: 500,
            scale_factor: 1.2,
            num_levels: 8,
            patch_size: 31,
            use_pyramid: true,
        }
    }
}

/// A single ORB feature with position, orientation, and descriptor
#[derive(Debug, Clone)]
pub struct OrbFeature {
    /// 2D position in image (pixel coordinates)
    pub position: Vector2<f64>,
    /// Orientation in radians [0, 2π)
    pub orientation: f64,
    /// ORB descriptor: 256 bits as 32 bytes (dynamic allocation, poolable)
    pub descriptor: Vec<u8>,
    /// Pyramid level where feature was detected
    pub level: usize,
    /// Feature strength (FAST score)
    pub strength: f64,
}

impl OrbFeature {
    /// Compute Hamming distance to another descriptor
    pub fn hamming_distance(&self, other: &[u8]) -> u32 {
        self.descriptor
            .iter()
            .zip(other.iter())
            .map(|(a, b)| (a ^ b).count_ones())
            .sum()
    }

    /// Check if orientation is consistent (for rotation-invariant matching)
    pub fn orientation_match(&self, other_orientation: f64, threshold: f64) -> bool {
        let diff = (self.orientation - other_orientation).abs();
        let normalized_diff = if diff > PI { 2.0 * PI - diff } else { diff };
        normalized_diff < threshold
    }
}
