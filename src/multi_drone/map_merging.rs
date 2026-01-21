//! Map merging and alignment for multi-drone SLAM
//!
//! Enables distributed mapping by merging local maps from multiple drones
//! into a globally consistent representation.

use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// Visual descriptor for a frame
#[derive(Debug, Clone)]
pub struct FrameDescriptor {
    pub frame_id: u32,
    pub features: Vec<f32>,
}

impl FrameDescriptor {
    pub fn new(frame_id: u32, features: Vec<f32>) -> Self {
        Self { frame_id, features }
    }

    /// Compute similarity with another descriptor (normalized dot product)
    pub fn similarity(&self, other: &FrameDescriptor) -> f64 {
        if self.features.len() != other.features.len() {
            return 0.0;
        }

        let dot: f32 = self
            .features
            .iter()
            .zip(&other.features)
            .map(|(a, b)| a * b)
            .sum();

        let norm_a: f32 = self.features.iter().map(|x| x * x).sum::<f32>().sqrt();
        let norm_b: f32 = other.features.iter().map(|x| x * x).sum::<f32>().sqrt();

        if norm_a > 0.0 && norm_b > 0.0 {
            (dot / (norm_a * norm_b)) as f64
        } else {
            0.0
        }
    }
}

/// 3D point in world coordinates
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Point3D {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Point3D {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    pub fn distance_to(&self, other: &Point3D) -> f64 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        let dz = self.z - other.z;
        (dx * dx + dy * dy + dz * dz).sqrt()
    }
}

/// SE(3) transformation (rotation + translation)
#[derive(Debug, Clone, Copy)]
pub struct Transform {
    /// Rotation matrix (3x3, row-major)
    pub rotation: [[f64; 3]; 3],
    /// Translation vector
    pub translation: [f64; 3],
}

impl Transform {
    /// Create identity transform
    pub fn identity() -> Self {
        Self {
            rotation: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            translation: [0.0, 0.0, 0.0],
        }
    }

    /// Transform a point
    pub fn transform_point(&self, p: &Point3D) -> Point3D {
        let x = self.rotation[0][0] * p.x
            + self.rotation[0][1] * p.y
            + self.rotation[0][2] * p.z
            + self.translation[0];
        let y = self.rotation[1][0] * p.x
            + self.rotation[1][1] * p.y
            + self.rotation[1][2] * p.z
            + self.translation[1];
        let z = self.rotation[2][0] * p.x
            + self.rotation[2][1] * p.y
            + self.rotation[2][2] * p.z
            + self.translation[2];
        Point3D::new(x, y, z)
    }

    /// Inverse transformation
    pub fn inverse(&self) -> Self {
        // R^T
        let r_t = [
            [self.rotation[0][0], self.rotation[1][0], self.rotation[2][0]],
            [self.rotation[0][1], self.rotation[1][1], self.rotation[2][1]],
            [self.rotation[0][2], self.rotation[1][2], self.rotation[2][2]],
        ];

        // t' = -R^T * t
        let t_prime = [
            -(r_t[0][0] * self.translation[0]
                + r_t[0][1] * self.translation[1]
                + r_t[0][2] * self.translation[2]),
            -(r_t[1][0] * self.translation[0]
                + r_t[1][1] * self.translation[1]
                + r_t[1][2] * self.translation[2]),
            -(r_t[2][0] * self.translation[0]
                + r_t[2][1] * self.translation[1]
                + r_t[2][2] * self.translation[2]),
        ];

        Self {
            rotation: r_t,
            translation: t_prime,
        }
    }

    /// Compose transformations: self * other
    pub fn compose(&self, other: &Transform) -> Self {
        // R = R1 * R2
        let mut r = [[0.0; 3]; 3];
        for i in 0..3 {
            for j in 0..3 {
                r[i][j] = self.rotation[i][0] * other.rotation[0][j]
                    + self.rotation[i][1] * other.rotation[1][j]
                    + self.rotation[i][2] * other.rotation[2][j];
            }
        }

        // t = R1 * t2 + t1
        let t = [
            self.rotation[0][0] * other.translation[0]
                + self.rotation[0][1] * other.translation[1]
                + self.rotation[0][2] * other.translation[2]
                + self.translation[0],
            self.rotation[1][0] * other.translation[0]
                + self.rotation[1][1] * other.translation[1]
                + self.rotation[1][2] * other.translation[2]
                + self.translation[1],
            self.rotation[2][0] * other.translation[0]
                + self.rotation[2][1] * other.translation[1]
                + self.rotation[2][2] * other.translation[2]
                + self.translation[2],
        ];

        Self {
            rotation: r,
            translation: t,
        }
    }
}

/// Local map from a single drone
#[derive(Debug, Clone)]
pub struct DroneMap {
    /// Drone ID
    pub drone_id: usize,
    /// 3D landmarks in drone's local frame
    pub landmarks: Vec<Point3D>,
    /// Visual descriptors for place recognition
    pub descriptors: Vec<FrameDescriptor>,
    /// Timestamp of map creation
    pub timestamp: f64,
}

impl DroneMap {
    /// Create new drone map
    pub fn new(drone_id: usize, timestamp: f64) -> Self {
        Self {
            drone_id,
            landmarks: Vec::new(),
            descriptors: Vec::new(),
            timestamp,
        }
    }

    /// Add landmark to map
    pub fn add_landmark(&mut self, point: Point3D) {
        self.landmarks.push(point);
    }

    /// Add descriptor to map
    pub fn add_descriptor(&mut self, descriptor: FrameDescriptor) {
        self.descriptors.push(descriptor);
    }

    /// Number of landmarks
    pub fn num_landmarks(&self) -> usize {
        self.landmarks.len()
    }
}

/// Configuration for map merging
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct MapMergingConfig {
    /// Minimum overlap score to consider maps for merging
    pub min_overlap_score: f64,
    /// Maximum distance threshold for point correspondence
    pub max_correspondence_distance: f64,
    /// Minimum number of correspondences
    pub min_correspondences: usize,
    /// RANSAC iterations for robust estimation
    pub ransac_iterations: usize,
    /// RANSAC inlier threshold
    pub ransac_threshold: f64,
}

impl Default for MapMergingConfig {
    fn default() -> Self {
        Self {
            min_overlap_score: 0.3,
            max_correspondence_distance: 1.0,
            min_correspondences: 10,
            ransac_iterations: 100,
            ransac_threshold: 0.1,
        }
    }
}

/// Map merging manager
pub struct MapMerger {
    config: MapMergingConfig,
    /// Maps from all drones
    drone_maps: HashMap<usize, DroneMap>,
    /// Relative transformations between drones
    relative_transforms: HashMap<(usize, usize), Transform>,
}

impl MapMerger {
    /// Create new map merger
    pub fn new(config: MapMergingConfig) -> Self {
        Self {
            config,
            drone_maps: HashMap::new(),
            relative_transforms: HashMap::new(),
        }
    }

    /// Add or update drone map
    pub fn add_drone_map(&mut self, map: DroneMap) {
        self.drone_maps.insert(map.drone_id, map);
    }

    /// Find potential map overlaps between drones
    pub fn find_overlaps(&self, drone_id: usize) -> Vec<(usize, f64)> {
        let mut overlaps = Vec::new();

        let Some(query_map) = self.drone_maps.get(&drone_id) else {
            return overlaps;
        };

        // Compare with all other drone maps
        for (other_id, other_map) in &self.drone_maps {
            if *other_id == drone_id {
                continue;
            }

            // Compute similarity between descriptor sets
            let mut total_score = 0.0;
            let mut count = 0;

            for query_desc in &query_map.descriptors {
                for other_desc in &other_map.descriptors {
                    let sim = query_desc.similarity(other_desc);
                    if sim > 0.5 {
                        // Threshold for potential match
                        total_score += sim;
                        count += 1;
                    }
                }
            }

            if count > 0 {
                let avg_score = total_score / count as f64;
                if avg_score >= self.config.min_overlap_score {
                    overlaps.push((*other_id, avg_score));
                }
            }
        }

        overlaps
    }

    /// Estimate relative transformation between two drone maps
    pub fn estimate_transform(
        &self,
        source_id: usize,
        target_id: usize,
    ) -> Option<Transform> {
        let source_map = self.drone_maps.get(&source_id)?;
        let target_map = self.drone_maps.get(&target_id)?;

        if source_map.landmarks.len() < self.config.min_correspondences
            || target_map.landmarks.len() < self.config.min_correspondences
        {
            return None;
        }

        // For simplicity, use centroid alignment (ICP would be better)
        let source_centroid = self.compute_centroid(&source_map.landmarks);
        let target_centroid = self.compute_centroid(&target_map.landmarks);

        // Translation to align centroids
        let translation = [
            target_centroid.x - source_centroid.x,
            target_centroid.y - source_centroid.y,
            target_centroid.z - source_centroid.z,
        ];

        Some(Transform {
            rotation: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            translation,
        })
    }

    /// Compute centroid of points
    fn compute_centroid(&self, points: &[Point3D]) -> Point3D {
        if points.is_empty() {
            return Point3D::new(0.0, 0.0, 0.0);
        }

        let sum_x: f64 = points.iter().map(|p| p.x).sum();
        let sum_y: f64 = points.iter().map(|p| p.y).sum();
        let sum_z: f64 = points.iter().map(|p| p.z).sum();
        let n = points.len() as f64;

        Point3D::new(sum_x / n, sum_y / n, sum_z / n)
    }

    /// Merge two drone maps using estimated transformation
    pub fn merge_maps(&mut self, source_id: usize, target_id: usize) -> bool {
        let Some(transform) = self.estimate_transform(source_id, target_id) else {
            return false;
        };

        // Store transformation
        self.relative_transforms
            .insert((source_id, target_id), transform);
        self.relative_transforms
            .insert((target_id, source_id), transform.inverse());

        true
    }

    /// Get merged global map (transform all to reference frame)
    pub fn get_global_map(&self, reference_drone: usize) -> Vec<Point3D> {
        let mut global_landmarks = Vec::new();

        for (drone_id, map) in &self.drone_maps {
            if *drone_id == reference_drone {
                // Reference frame - no transformation
                global_landmarks.extend(map.landmarks.iter().copied());
            } else if let Some(transform) = self.relative_transforms.get(&(*drone_id, reference_drone)) {
                // Transform to reference frame
                for landmark in &map.landmarks {
                    global_landmarks.push(transform.transform_point(landmark));
                }
            }
        }

        global_landmarks
    }

    /// Get number of registered drones
    pub fn num_drones(&self) -> usize {
        self.drone_maps.len()
    }

    /// Check if transformation exists between drones
    pub fn has_transform(&self, source: usize, target: usize) -> bool {
        self.relative_transforms.contains_key(&(source, target))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_point3d_creation() {
        let p = Point3D::new(1.0, 2.0, 3.0);
        assert_eq!(p.x, 1.0);
        assert_eq!(p.y, 2.0);
        assert_eq!(p.z, 3.0);
    }

    #[test]
    fn test_point3d_distance() {
        let p1 = Point3D::new(0.0, 0.0, 0.0);
        let p2 = Point3D::new(3.0, 4.0, 0.0);
        assert!((p1.distance_to(&p2) - 5.0).abs() < 0.001);
    }

    #[test]
    fn test_transform_identity() {
        let t = Transform::identity();
        let p = Point3D::new(1.0, 2.0, 3.0);
        let transformed = t.transform_point(&p);
        assert!((transformed.x - p.x).abs() < 0.001);
        assert!((transformed.y - p.y).abs() < 0.001);
        assert!((transformed.z - p.z).abs() < 0.001);
    }

    #[test]
    fn test_transform_translation() {
        let t = Transform {
            rotation: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            translation: [1.0, 2.0, 3.0],
        };
        let p = Point3D::new(0.0, 0.0, 0.0);
        let transformed = t.transform_point(&p);
        assert!((transformed.x - 1.0).abs() < 0.001);
        assert!((transformed.y - 2.0).abs() < 0.001);
        assert!((transformed.z - 3.0).abs() < 0.001);
    }

    #[test]
    fn test_transform_inverse() {
        let t = Transform {
            rotation: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            translation: [1.0, 2.0, 3.0],
        };
        let t_inv = t.inverse();
        let p = Point3D::new(5.0, 6.0, 7.0);
        
        let transformed = t.transform_point(&p);
        let back = t_inv.transform_point(&transformed);
        
        assert!((back.x - p.x).abs() < 0.001);
        assert!((back.y - p.y).abs() < 0.001);
        assert!((back.z - p.z).abs() < 0.001);
    }

    #[test]
    fn test_transform_compose() {
        let t1 = Transform {
            rotation: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            translation: [1.0, 0.0, 0.0],
        };
        let t2 = Transform {
            rotation: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            translation: [0.0, 1.0, 0.0],
        };
        
        let composed = t1.compose(&t2);
        let p = Point3D::new(0.0, 0.0, 0.0);
        let result = composed.transform_point(&p);
        
        assert!((result.x - 1.0).abs() < 0.001);
        assert!((result.y - 1.0).abs() < 0.001);
    }

    #[test]
    fn test_drone_map_creation() {
        let map = DroneMap::new(0, 100.0);
        assert_eq!(map.drone_id, 0);
        assert_eq!(map.timestamp, 100.0);
        assert_eq!(map.num_landmarks(), 0);
    }

    #[test]
    fn test_drone_map_add_landmark() {
        let mut map = DroneMap::new(0, 100.0);
        map.add_landmark(Point3D::new(1.0, 2.0, 3.0));
        assert_eq!(map.num_landmarks(), 1);
    }

    #[test]
    fn test_map_merging_config_defaults() {
        let config = MapMergingConfig::default();
        assert!(config.min_overlap_score > 0.0);
        assert!(config.max_correspondence_distance > 0.0);
        assert!(config.min_correspondences > 0);
    }

    #[test]
    fn test_map_merger_creation() {
        let merger = MapMerger::new(MapMergingConfig::default());
        assert_eq!(merger.num_drones(), 0);
    }

    #[test]
    fn test_map_merger_add_map() {
        let mut merger = MapMerger::new(MapMergingConfig::default());
        let map = DroneMap::new(0, 100.0);
        merger.add_drone_map(map);
        assert_eq!(merger.num_drones(), 1);
    }

    #[test]
    fn test_map_merger_estimate_transform() {
        let mut merger = MapMerger::new(MapMergingConfig::default());
        
        let mut map1 = DroneMap::new(0, 100.0);
        for i in 0..15 {
            map1.add_landmark(Point3D::new(i as f64, 0.0, 0.0));
        }
        
        let mut map2 = DroneMap::new(1, 101.0);
        for i in 0..15 {
            map2.add_landmark(Point3D::new(i as f64 + 10.0, 5.0, 0.0));
        }
        
        merger.add_drone_map(map1);
        merger.add_drone_map(map2);
        
        let transform = merger.estimate_transform(0, 1);
        assert!(transform.is_some());
    }

    #[test]
    fn test_map_merger_global_map() {
        let mut merger = MapMerger::new(MapMergingConfig::default());
        
        let mut map1 = DroneMap::new(0, 100.0);
        map1.add_landmark(Point3D::new(0.0, 0.0, 0.0));
        map1.add_landmark(Point3D::new(1.0, 0.0, 0.0));
        
        let mut map2 = DroneMap::new(1, 101.0);
        map2.add_landmark(Point3D::new(10.0, 0.0, 0.0));
        map2.add_landmark(Point3D::new(11.0, 0.0, 0.0));
        
        merger.add_drone_map(map1);
        merger.add_drone_map(map2);
        
        let global = merger.get_global_map(0);
        assert_eq!(global.len(), 2); // Only reference drone's points (no transform)
    }

    #[test]
    fn test_map_merger_has_transform() {
        let mut merger = MapMerger::new(MapMergingConfig::default());
        
        let mut map1 = DroneMap::new(0, 100.0);
        for i in 0..15 {
            map1.add_landmark(Point3D::new(i as f64, 0.0, 0.0));
        }
        
        let mut map2 = DroneMap::new(1, 101.0);
        for i in 0..15 {
            map2.add_landmark(Point3D::new(i as f64, 0.0, 0.0));
        }
        
        merger.add_drone_map(map1);
        merger.add_drone_map(map2);
        
        assert!(!merger.has_transform(0, 1));
        merger.merge_maps(0, 1);
        assert!(merger.has_transform(0, 1));
    }
}
