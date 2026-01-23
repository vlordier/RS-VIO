//! TSDF (Truncated Signed Distance Field) volume for 3D reconstruction
//!
//! Implements volumetric fusion of depth maps into a consistent 3D representation.

use serde::{Deserialize, Serialize};

/// Configuration for TSDF volume
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct TSDFConfig {
    /// Number of voxels in X dimension
    pub resolution_x: usize,
    /// Number of voxels in Y dimension
    pub resolution_y: usize,
    /// Number of voxels in Z dimension
    pub resolution_z: usize,
    /// Size of each voxel in meters
    pub voxel_size: f32,
    /// Truncation distance in meters
    pub truncation_distance: f32,
    /// Origin of the volume in world coordinates
    pub origin: [f32; 3],
}

impl Default for TSDFConfig {
    fn default() -> Self {
        Self {
            resolution_x: 256,
            resolution_y: 256,
            resolution_z: 256,
            voxel_size: 0.01,              // 1cm voxels
            truncation_distance: 0.05,     // 5cm truncation
            origin: [-1.28, -1.28, -1.28], // Center volume at origin
        }
    }
}

/// Single voxel in the TSDF volume
#[derive(Debug, Clone, Copy)]
pub struct Voxel {
    /// Truncated signed distance value
    pub tsdf: f32,
    /// Weight (number of observations)
    pub weight: f32,
}

impl Voxel {
    /// Create empty voxel
    pub fn new() -> Self {
        Self {
            tsdf: 0.0,
            weight: 0.0,
        }
    }

    /// Check if voxel has been observed
    pub fn is_observed(&self) -> bool {
        self.weight > 0.0
    }
}

impl Default for Voxel {
    fn default() -> Self {
        Self::new()
    }
}

/// 3D point in world coordinates
#[derive(Debug, Clone, Copy)]
pub struct Point3D {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Point3D {
    /// Create new 3D point
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    /// Compute distance to another point
    pub fn distance_to(&self, other: &Point3D) -> f32 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        let dz = self.z - other.z;
        (dx * dx + dy * dy + dz * dz).sqrt()
    }
}

/// Simple camera pose (position + orientation)
#[derive(Debug, Clone, Copy)]
pub struct CameraPose {
    /// Position in world coordinates
    pub position: [f32; 3],
    /// Rotation matrix (3x3)
    pub rotation: [[f32; 3]; 3],
}

impl CameraPose {
    /// Create identity pose (at origin, no rotation)
    pub fn identity() -> Self {
        Self {
            position: [0.0, 0.0, 0.0],
            rotation: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        }
    }

    /// Transform a point from world to camera coordinates
    pub fn world_to_camera(&self, point: &Point3D) -> Point3D {
        // Translate to camera origin
        let translated = [
            point.x - self.position[0],
            point.y - self.position[1],
            point.z - self.position[2],
        ];

        // Rotate (R^T * p)
        let x = self.rotation[0][0] * translated[0]
            + self.rotation[1][0] * translated[1]
            + self.rotation[2][0] * translated[2];
        let y = self.rotation[0][1] * translated[0]
            + self.rotation[1][1] * translated[1]
            + self.rotation[2][1] * translated[2];
        let z = self.rotation[0][2] * translated[0]
            + self.rotation[1][2] * translated[1]
            + self.rotation[2][2] * translated[2];

        Point3D::new(x, y, z)
    }
}

/// Depth map from a single viewpoint
#[derive(Debug, Clone)]
pub struct DepthMap {
    pub width: usize,
    pub height: usize,
    /// Depth values in meters (row-major)
    pub depths: Vec<f32>,
    /// Camera intrinsics (fx, fy, cx, cy)
    pub intrinsics: [f32; 4],
}

impl DepthMap {
    /// Create new depth map
    pub fn new(width: usize, height: usize, intrinsics: [f32; 4]) -> Self {
        Self {
            width,
            height,
            depths: vec![0.0; width * height],
            intrinsics,
        }
    }

    /// Get depth at pixel (u, v)
    pub fn get_depth(&self, u: usize, v: usize) -> Option<f32> {
        if u >= self.width || v >= self.height {
            return None;
        }
        let depth = self.depths[v * self.width + u];
        if depth > 0.0 {
            Some(depth)
        } else {
            None
        }
    }

    /// Set depth at pixel (u, v)
    pub fn set_depth(&mut self, u: usize, v: usize, depth: f32) {
        if u < self.width && v < self.height {
            self.depths[v * self.width + u] = depth;
        }
    }

    /// Project 3D point to pixel coordinates
    pub fn project(&self, point: &Point3D) -> Option<(usize, usize)> {
        if point.z <= 0.0 {
            return None; // Behind camera
        }

        let fx = self.intrinsics[0];
        let fy = self.intrinsics[1];
        let cx = self.intrinsics[2];
        let cy = self.intrinsics[3];

        let u = (fx * point.x / point.z + cx) as isize;
        let v = (fy * point.y / point.z + cy) as isize;

        if u >= 0 && v >= 0 && (u as usize) < self.width && (v as usize) < self.height {
            Some((u as usize, v as usize))
        } else {
            None
        }
    }
}

/// TSDF volume for volumetric fusion
pub struct TSDFVolume {
    config: TSDFConfig,
    voxels: Vec<Voxel>,
}

impl TSDFVolume {
    /// Create new TSDF volume
    pub fn new(config: TSDFConfig) -> Self {
        let num_voxels = config.resolution_x * config.resolution_y * config.resolution_z;
        Self {
            config,
            voxels: vec![Voxel::new(); num_voxels],
        }
    }

    /// Get linear index from 3D voxel coordinates
    fn voxel_index(&self, x: usize, y: usize, z: usize) -> Option<usize> {
        if x >= self.config.resolution_x
            || y >= self.config.resolution_y
            || z >= self.config.resolution_z
        {
            return None;
        }
        Some(
            z * (self.config.resolution_x * self.config.resolution_y)
                + y * self.config.resolution_x
                + x,
        )
    }

    /// Get voxel at 3D coordinates
    pub fn get_voxel(&self, x: usize, y: usize, z: usize) -> Option<&Voxel> {
        self.voxel_index(x, y, z).map(|idx| &self.voxels[idx])
    }

    /// Get mutable voxel at 3D coordinates
    fn get_voxel_mut(&mut self, x: usize, y: usize, z: usize) -> Option<&mut Voxel> {
        self.voxel_index(x, y, z)
            .map(move |idx| &mut self.voxels[idx])
    }

    /// Convert voxel indices to world coordinates (center of voxel)
    pub fn voxel_to_world(&self, x: usize, y: usize, z: usize) -> Point3D {
        Point3D::new(
            self.config.origin[0] + (x as f32 + 0.5) * self.config.voxel_size,
            self.config.origin[1] + (y as f32 + 0.5) * self.config.voxel_size,
            self.config.origin[2] + (z as f32 + 0.5) * self.config.voxel_size,
        )
    }

    /// Integrate a depth map into the volume
    pub fn integrate(&mut self, depth_map: &DepthMap, pose: &CameraPose) {
        // Iterate through all voxels
        for z in 0..self.config.resolution_z {
            for y in 0..self.config.resolution_y {
                for x in 0..self.config.resolution_x {
                    // Get voxel center in world coordinates
                    let world_point = self.voxel_to_world(x, y, z);

                    // Transform to camera coordinates
                    let camera_point = pose.world_to_camera(&world_point);

                    // Skip if behind camera
                    if camera_point.z <= 0.0 {
                        continue;
                    }

                    // Project to image
                    if let Some((u, v)) = depth_map.project(&camera_point) {
                        if let Some(measured_depth) = depth_map.get_depth(u, v) {
                            // Compute signed distance
                            let sdf = measured_depth - camera_point.z;

                            // Truncate
                            let tsdf = if sdf < -self.config.truncation_distance {
                                -1.0
                            } else if sdf > self.config.truncation_distance {
                                1.0
                            } else {
                                sdf / self.config.truncation_distance
                            };

                            // Update voxel with weighted average
                            if let Some(voxel) = self.get_voxel_mut(x, y, z) {
                                let new_weight = 1.0; // Uniform weight for now
                                let total_weight = voxel.weight + new_weight;

                                voxel.tsdf =
                                    (voxel.tsdf * voxel.weight + tsdf * new_weight) / total_weight;
                                voxel.weight = total_weight;
                            }
                        }
                    }
                }
            }
        }
    }

    /// Extract points near surface (TSDF ≈ 0)
    pub fn extract_points(&self) -> Vec<Point3D> {
        let mut points = Vec::new();
        let threshold = 0.1; // Extract points with |TSDF| < threshold

        for z in 0..self.config.resolution_z {
            for y in 0..self.config.resolution_y {
                for x in 0..self.config.resolution_x {
                    if let Some(voxel) = self.get_voxel(x, y, z) {
                        if voxel.is_observed() && voxel.tsdf.abs() < threshold {
                            points.push(self.voxel_to_world(x, y, z));
                        }
                    }
                }
            }
        }

        points
    }

    /// Get number of observed voxels
    pub fn num_observed(&self) -> usize {
        self.voxels.iter().filter(|v| v.is_observed()).count()
    }

    /// Get configuration
    pub fn config(&self) -> &TSDFConfig {
        &self.config
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_config_defaults() {
        let config = TSDFConfig::default();
        assert_eq!(config.resolution_x, 256);
        assert!(config.voxel_size > 0.0);
        assert!(config.truncation_distance > 0.0);
    }

    #[test]
    fn test_voxel_creation() {
        let voxel = Voxel::new();
        assert_eq!(voxel.tsdf, 0.0);
        assert_eq!(voxel.weight, 0.0);
        assert!(!voxel.is_observed());
    }

    #[test]
    fn test_point3d_distance() {
        let p1 = Point3D::new(0.0, 0.0, 0.0);
        let p2 = Point3D::new(3.0, 4.0, 0.0);
        assert!((p1.distance_to(&p2) - 5.0).abs() < 0.01);
    }

    #[test]
    fn test_camera_pose_identity() {
        let pose = CameraPose::identity();
        assert_eq!(pose.position, [0.0, 0.0, 0.0]);
        assert_eq!(pose.rotation[0][0], 1.0);
    }

    #[test]
    fn test_world_to_camera_transform() {
        let pose = CameraPose::identity();
        let world_point = Point3D::new(1.0, 2.0, 3.0);
        let camera_point = pose.world_to_camera(&world_point);

        // Identity should preserve coordinates
        assert!((camera_point.x - 1.0).abs() < 0.001);
        assert!((camera_point.y - 2.0).abs() < 0.001);
        assert!((camera_point.z - 3.0).abs() < 0.001);
    }

    #[test]
    fn test_depth_map_creation() {
        let intrinsics = [500.0, 500.0, 320.0, 240.0]; // fx, fy, cx, cy
        let depth_map = DepthMap::new(640, 480, intrinsics);

        assert_eq!(depth_map.width, 640);
        assert_eq!(depth_map.height, 480);
        assert_eq!(depth_map.depths.len(), 640 * 480);
    }

    #[test]
    fn test_depth_map_get_set() {
        let intrinsics = [500.0, 500.0, 320.0, 240.0];
        let mut depth_map = DepthMap::new(640, 480, intrinsics);

        depth_map.set_depth(100, 100, 2.5);
        assert_eq!(depth_map.get_depth(100, 100), Some(2.5));

        // Out of bounds
        assert_eq!(depth_map.get_depth(1000, 1000), None);
    }

    #[test]
    fn test_depth_map_projection() {
        let intrinsics = [500.0, 500.0, 320.0, 240.0];
        let depth_map = DepthMap::new(640, 480, intrinsics);

        // Point in front of camera at (0, 0, 1)
        let point = Point3D::new(0.0, 0.0, 1.0);
        let projected = depth_map.project(&point);

        // Should project to principal point (cx, cy)
        assert_eq!(projected, Some((320, 240)));
    }

    #[test]
    fn test_tsdf_volume_creation() {
        let config = TSDFConfig::default();
        let volume = TSDFVolume::new(config);

        assert_eq!(volume.num_observed(), 0);
    }

    #[test]
    fn test_voxel_indexing() {
        let config = TSDFConfig {
            resolution_x: 10,
            resolution_y: 10,
            resolution_z: 10,
            ..Default::default()
        };
        let volume = TSDFVolume::new(config);

        // Valid indices
        assert!(volume.get_voxel(0, 0, 0).is_some());
        assert!(volume.get_voxel(9, 9, 9).is_some());

        // Invalid indices
        assert!(volume.get_voxel(10, 0, 0).is_none());
        assert!(volume.get_voxel(0, 10, 0).is_none());
    }

    #[test]
    fn test_voxel_to_world() {
        let config = TSDFConfig {
            resolution_x: 10,
            resolution_y: 10,
            resolution_z: 10,
            voxel_size: 1.0,
            origin: [0.0, 0.0, 0.0],
            ..Default::default()
        };
        let volume = TSDFVolume::new(config);

        let point = volume.voxel_to_world(0, 0, 0);
        // Center of first voxel should be at (0.5, 0.5, 0.5)
        assert!((point.x - 0.5).abs() < 0.01);
        assert!((point.y - 0.5).abs() < 0.01);
        assert!((point.z - 0.5).abs() < 0.01);
    }

    #[test]
    fn test_integration_empty_depth() {
        let config = TSDFConfig {
            resolution_x: 10,
            resolution_y: 10,
            resolution_z: 10,
            ..Default::default()
        };
        let mut volume = TSDFVolume::new(config);

        let intrinsics = [500.0, 500.0, 320.0, 240.0];
        let depth_map = DepthMap::new(640, 480, intrinsics);
        let pose = CameraPose::identity();

        volume.integrate(&depth_map, &pose);

        // No depths, so no voxels should be updated
        assert_eq!(volume.num_observed(), 0);
    }

    #[test]
    fn test_extract_points_empty() {
        let config = TSDFConfig::default();
        let volume = TSDFVolume::new(config);

        let points = volume.extract_points();
        assert_eq!(points.len(), 0);
    }
}
