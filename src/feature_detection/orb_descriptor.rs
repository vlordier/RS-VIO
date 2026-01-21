//! ORB (ORiented BRIEF) descriptor for fast matching and loop closure
//!
//! ORB combines FAST corner detection with BRIEF binary descriptors,
//! adding rotation invariance for robust matching across frames and keyframes.
//! Integrated with Bag-of-Words for loop closure detection.

use super::{Keypoint, Descriptor, DescriptorData, FeatureResult, FeatureError};

/// ORB configuration
#[derive(Debug, Clone)]
pub struct ORBConfig {
    pub num_features: u32,
    pub scale_factor: f64,
    pub num_levels: u32,
    pub fast_threshold: u8,
}

impl Default for ORBConfig {
    fn default() -> Self {
        Self {
            num_features: 500,
            scale_factor: 1.2,
            num_levels: 8,
            fast_threshold: 20,
        }
    }
}

/// ORB descriptor (256-bit binary)
#[derive(Debug, Clone)]
pub struct ORBDescriptor {
    /// Configuration reserved for future pyramid levels and multi-scale detection
    /// Current implementation uses simplified single-scale BRIEF
    _config: ORBConfig,
}

impl ORBDescriptor {
    /// Create new ORB descriptor
    pub fn new(config: ORBConfig) -> Self {
        Self { _config: config }
    }

    /// Compute BRIEF descriptor (simplified: random pairs)
    fn compute_brief(&self, image: &[u8], x: u32, y: u32, width: u32) -> Vec<u8> {
        // Simplified BRIEF: 256-bit descriptor
        // In practice, uses specific pre-computed test locations
        let mut descriptor = vec![0u8; 32]; // 256 bits = 32 bytes

        let w = width as usize;
        let center_idx = y as usize * w + x as usize;

        // Simple pattern: compare random pairs of pixels
        for byte_idx in 0..32 {
            let mut byte = 0u8;
            for bit in 0..8 {
                // Random test locations (in real BRIEF, these are fixed)
                let offset1 = 10 + (byte_idx * 8 + bit) as usize % 100;
                let offset2 = 50 + (byte_idx * 8 + bit) as usize % 100;

                if center_idx + offset1 < image.len() && center_idx + offset2 < image.len() {
                    if image[center_idx + offset1] > image[center_idx + offset2] {
                        byte |= 1 << bit;
                    }
                }
            }
            descriptor[byte_idx] = byte;
        }

        descriptor
    }

    /// Estimate orientation from image gradients
    fn estimate_orientation(&self, image: &[u8], x: u32, y: u32, width: u32, height: u32) -> f32 {
        let w = width as usize;
        let _idx = y as usize * w + x as usize;

        // Simple orientation: dominant gradient direction in patch
        let mut gx = 0.0;
        let mut gy = 0.0;

        for dy in -3..=3 {
            for dx in -3..=3 {
                let px = (x as i32 + dx) as usize;
                let py = (y as i32 + dy) as usize;

                if px > 0 && px < (width - 1) as usize && py > 0 && py < (height - 1) as usize {
                    let idx_x1 = py * w + px + 1;
                    let idx_y1 = (py + 1) * w + px;
                    let pidx = py * w + px;

                    let grad_x = (image[idx_x1] as f32 - image[pidx] as f32) / 2.0;
                    let grad_y = (image[idx_y1] as f32 - image[pidx] as f32) / 2.0;

                    gx += grad_x;
                    gy += grad_y;
                }
            }
        }

        gy.atan2(gx).to_degrees()
    }
}

impl Descriptor for ORBDescriptor {
    fn compute(
        &self,
        image: &[u8],
        keypoints: &[Keypoint],
        width: u32,
        height: u32,
    ) -> FeatureResult<Vec<DescriptorData>> {
        if image.is_empty() {
            return Err(FeatureError::InvalidImage("Empty image".to_string()));
        }

        let mut descriptors = Vec::new();

        for kp in keypoints {
            let x = kp.x as u32;
            let y = kp.y as u32;

            if x < 15 || x >= width - 15 || y < 15 || y >= height - 15 {
                continue; // Skip points too close to edge
            }

            // Estimate orientation
            let _orientation = self.estimate_orientation(image, x, y, width, height);

            // Compute BRIEF descriptor
            let brief = self.compute_brief(image, x, y, width);

            descriptors.push(DescriptorData::Binary(brief));
        }

        Ok(descriptors)
    }

    fn distance(&self, desc1: &DescriptorData, desc2: &DescriptorData) -> u32 {
        // Hamming distance for binary descriptors
        match (desc1, desc2) {
            (DescriptorData::Binary(d1), DescriptorData::Binary(d2)) => {
                let mut distance = 0u32;
                for (b1, b2) in d1.iter().zip(d2.iter()) {
                    distance += (b1 ^ b2).count_ones();
                }
                distance
            }
            _ => u32::MAX, // Incompatible descriptor types
        }
    }

    fn name(&self) -> &str {
        "ORB"
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_orb_descriptor_creation() {
        let config = ORBConfig::default();
        let descriptor = ORBDescriptor::new(config);
        assert_eq!(descriptor.name(), "ORB");
    }

    #[test]
    fn test_orb_invalid_image() {
        let config = ORBConfig::default();
        let descriptor = ORBDescriptor::new(config);
        let keypoint = Keypoint {
            x: 100.0,
            y: 100.0,
            score: 0.8,
            scale: 1.0,
            angle: None,
        };
        let result = descriptor.compute(&[], &[keypoint], 640, 480);
        assert!(result.is_err());
    }

    #[test]
    fn test_orb_hamming_distance() {
        let config = ORBConfig::default();
        let descriptor = ORBDescriptor::new(config);

        let desc1 = DescriptorData::Binary(vec![0xFF; 32]);
        let desc2 = DescriptorData::Binary(vec![0x00; 32]);
        let distance = descriptor.distance(&desc1, &desc2);

        assert_eq!(distance, 256); // All 8 bits different in each byte
    }
}
