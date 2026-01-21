//! Point cloud generation from depth maps
//!
//! Converts depth maps to 3D point clouds with color information.

use crate::dense_reconstruction::tsdf_volume::{CameraPose, DepthMap, Point3D};
use serde::{Deserialize, Serialize};

/// Configuration for point cloud generation
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct PointCloudConfig {
    /// Minimum depth in meters
    pub min_depth: f32,
    /// Maximum depth in meters
    pub max_depth: f32,
    /// Downsample factor (1 = no downsampling, 2 = half resolution, etc.)
    pub downsample_factor: usize,
    /// Enable outlier filtering
    pub filter_outliers: bool,
    /// Outlier filter: minimum number of neighbors within radius
    pub outlier_min_neighbors: usize,
    /// Outlier filter: search radius in meters
    pub outlier_radius: f32,
}

impl Default for PointCloudConfig {
    fn default() -> Self {
        Self {
            min_depth: 0.1,
            max_depth: 10.0,
            downsample_factor: 1,
            filter_outliers: false,
            outlier_min_neighbors: 5,
            outlier_radius: 0.05,
        }
    }
}

/// RGB color
#[derive(Debug, Clone, Copy)]
pub struct Color {
    pub r: u8,
    pub g: u8,
    pub b: u8,
}

impl Color {
    /// Create new color
    pub fn new(r: u8, g: u8, b: u8) -> Self {
        Self { r, g, b }
    }

    /// Create white color
    pub fn white() -> Self {
        Self::new(255, 255, 255)
    }

    /// Create black color
    pub fn black() -> Self {
        Self::new(0, 0, 0)
    }
}

impl Default for Color {
    fn default() -> Self {
        Self::white()
    }
}

/// Grayscale image
#[derive(Debug, Clone)]
pub struct GrayImage {
    pub width: usize,
    pub height: usize,
    pub pixels: Vec<u8>,
}

impl GrayImage {
    /// Create new grayscale image
    pub fn new(width: usize, height: usize) -> Self {
        Self {
            width,
            height,
            pixels: vec![0; width * height],
        }
    }

    /// Get pixel value
    pub fn get_pixel(&self, u: usize, v: usize) -> Option<u8> {
        if u >= self.width || v >= self.height {
            return None;
        }
        Some(self.pixels[v * self.width + u])
    }

    /// Set pixel value
    pub fn set_pixel(&mut self, u: usize, v: usize, value: u8) {
        if u < self.width && v < self.height {
            self.pixels[v * self.width + u] = value;
        }
    }
}

/// Point cloud with colors
#[derive(Debug, Clone)]
pub struct PointCloud {
    pub points: Vec<Point3D>,
    pub colors: Vec<Color>,
}

impl PointCloud {
    /// Create empty point cloud
    pub fn new() -> Self {
        Self {
            points: Vec::new(),
            colors: Vec::new(),
        }
    }

    /// Create with capacity
    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            points: Vec::with_capacity(capacity),
            colors: Vec::with_capacity(capacity),
        }
    }

    /// Add a point with color
    pub fn add_point(&mut self, point: Point3D, color: Color) {
        self.points.push(point);
        self.colors.push(color);
    }

    /// Get number of points
    pub fn len(&self) -> usize {
        self.points.len()
    }

    /// Check if empty
    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    /// Compute bounding box (min, max)
    pub fn bounding_box(&self) -> Option<(Point3D, Point3D)> {
        if self.points.is_empty() {
            return None;
        }

        let mut min_x = f32::MAX;
        let mut min_y = f32::MAX;
        let mut min_z = f32::MAX;
        let mut max_x = f32::MIN;
        let mut max_y = f32::MIN;
        let mut max_z = f32::MIN;

        for p in &self.points {
            min_x = min_x.min(p.x);
            min_y = min_y.min(p.y);
            min_z = min_z.min(p.z);
            max_x = max_x.max(p.x);
            max_y = max_y.max(p.y);
            max_z = max_z.max(p.z);
        }

        Some((
            Point3D::new(min_x, min_y, min_z),
            Point3D::new(max_x, max_y, max_z),
        ))
    }
}

impl Default for PointCloud {
    fn default() -> Self {
        Self::new()
    }
}

/// Point cloud generator
pub struct PointCloudGenerator {
    config: PointCloudConfig,
}

impl PointCloudGenerator {
    /// Create new generator
    pub fn new(config: PointCloudConfig) -> Self {
        Self { config }
    }

    /// Generate point cloud from depth map
    pub fn from_depth_map(
        &self,
        depth_map: &DepthMap,
        pose: &CameraPose,
        image: Option<&GrayImage>,
    ) -> PointCloud {
        let step = self.config.downsample_factor;
        let estimated_points = (depth_map.width / step) * (depth_map.height / step);
        let mut cloud = PointCloud::with_capacity(estimated_points);

        let fx = depth_map.intrinsics[0];
        let fy = depth_map.intrinsics[1];
        let cx = depth_map.intrinsics[2];
        let cy = depth_map.intrinsics[3];

        for v in (0..depth_map.height).step_by(step) {
            for u in (0..depth_map.width).step_by(step) {
                if let Some(depth) = depth_map.get_depth(u, v) {
                    // Depth filtering
                    if depth < self.config.min_depth || depth > self.config.max_depth {
                        continue;
                    }

                    // Backproject to camera coordinates
                    let x_cam = (u as f32 - cx) * depth / fx;
                    let y_cam = (v as f32 - cy) * depth / fy;
                    let z_cam = depth;

                    // Transform to world coordinates
                    let world_point = self.camera_to_world(
                        Point3D::new(x_cam, y_cam, z_cam),
                        pose,
                    );

                    // Get color from image
                    let color = if let Some(img) = image {
                        if let Some(intensity) = img.get_pixel(u, v) {
                            Color::new(intensity, intensity, intensity)
                        } else {
                            Color::white()
                        }
                    } else {
                        Color::white()
                    };

                    cloud.add_point(world_point, color);
                }
            }
        }

        if self.config.filter_outliers {
            self.filter_outliers_internal(&mut cloud);
        }

        cloud
    }

    /// Transform point from camera to world coordinates
    fn camera_to_world(&self, camera_point: Point3D, pose: &CameraPose) -> Point3D {
        // Apply rotation: R * p
        let rotated = [
            pose.rotation[0][0] * camera_point.x
                + pose.rotation[0][1] * camera_point.y
                + pose.rotation[0][2] * camera_point.z,
            pose.rotation[1][0] * camera_point.x
                + pose.rotation[1][1] * camera_point.y
                + pose.rotation[1][2] * camera_point.z,
            pose.rotation[2][0] * camera_point.x
                + pose.rotation[2][1] * camera_point.y
                + pose.rotation[2][2] * camera_point.z,
        ];

        // Add translation: R * p + t
        Point3D::new(
            rotated[0] + pose.position[0],
            rotated[1] + pose.position[1],
            rotated[2] + pose.position[2],
        )
    }

    /// Filter outliers using radius search
    fn filter_outliers_internal(&self, cloud: &mut PointCloud) {
        if cloud.points.len() < self.config.outlier_min_neighbors {
            return;
        }

        let mut keep_mask = vec![false; cloud.points.len()];

        // For each point, count neighbors within radius
        for i in 0..cloud.points.len() {
            let mut neighbor_count = 0;

            for j in 0..cloud.points.len() {
                if i == j {
                    continue;
                }

                let dist = cloud.points[i].distance_to(&cloud.points[j]);
                if dist <= self.config.outlier_radius {
                    neighbor_count += 1;
                }
            }

            keep_mask[i] = neighbor_count >= self.config.outlier_min_neighbors;
        }

        // Filter points and colors
        let mut filtered_points = Vec::new();
        let mut filtered_colors = Vec::new();

        for (i, keep) in keep_mask.iter().enumerate() {
            if *keep {
                filtered_points.push(cloud.points[i]);
                filtered_colors.push(cloud.colors[i]);
            }
        }

        cloud.points = filtered_points;
        cloud.colors = filtered_colors;
    }

    /// Downsample point cloud using voxel grid
    pub fn downsample_voxel_grid(&self, cloud: &PointCloud, voxel_size: f32) -> PointCloud {
        if cloud.is_empty() || voxel_size <= 0.0 {
            return cloud.clone();
        }

        use std::collections::HashMap;

        // Hash points into voxel grid
        let mut voxel_map: HashMap<(i32, i32, i32), Vec<usize>> = HashMap::new();

        for (i, point) in cloud.points.iter().enumerate() {
            let vx = (point.x / voxel_size).floor() as i32;
            let vy = (point.y / voxel_size).floor() as i32;
            let vz = (point.z / voxel_size).floor() as i32;

            voxel_map.entry((vx, vy, vz)).or_default().push(i);
        }

        // Average points within each voxel
        let mut downsampled = PointCloud::with_capacity(voxel_map.len());

        for indices in voxel_map.values() {
            let mut avg_x = 0.0f32;
            let mut avg_y = 0.0f32;
            let mut avg_z = 0.0f32;
            let mut avg_r = 0.0f32;
            let mut avg_g = 0.0f32;
            let mut avg_b = 0.0f32;

            for &idx in indices {
                avg_x += cloud.points[idx].x;
                avg_y += cloud.points[idx].y;
                avg_z += cloud.points[idx].z;
                avg_r += cloud.colors[idx].r as f32;
                avg_g += cloud.colors[idx].g as f32;
                avg_b += cloud.colors[idx].b as f32;
            }

            let count = indices.len() as f32;
            let avg_point = Point3D::new(avg_x / count, avg_y / count, avg_z / count);
            let avg_color = Color::new(
                (avg_r / count) as u8,
                (avg_g / count) as u8,
                (avg_b / count) as u8,
            );

            downsampled.add_point(avg_point, avg_color);
        }

        downsampled
    }
}

impl Default for PointCloudGenerator {
    fn default() -> Self {
        Self::new(PointCloudConfig::default())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_config_defaults() {
        let config = PointCloudConfig::default();
        assert_eq!(config.downsample_factor, 1);
        assert!(config.min_depth > 0.0);
        assert!(config.max_depth > config.min_depth);
    }

    #[test]
    fn test_color_creation() {
        let color = Color::new(100, 150, 200);
        assert_eq!(color.r, 100);
        assert_eq!(color.g, 150);
        assert_eq!(color.b, 200);
    }

    #[test]
    fn test_gray_image() {
        let mut img = GrayImage::new(640, 480);
        img.set_pixel(100, 100, 128);

        assert_eq!(img.get_pixel(100, 100), Some(128));
        assert_eq!(img.get_pixel(1000, 1000), None);
    }

    #[test]
    fn test_point_cloud_creation() {
        let mut cloud = PointCloud::new();
        assert!(cloud.is_empty());

        cloud.add_point(Point3D::new(1.0, 2.0, 3.0), Color::white());
        assert_eq!(cloud.len(), 1);
    }

    #[test]
    fn test_point_cloud_bounding_box() {
        let mut cloud = PointCloud::new();
        cloud.add_point(Point3D::new(0.0, 0.0, 0.0), Color::white());
        cloud.add_point(Point3D::new(1.0, 1.0, 1.0), Color::white());

        let (min, max) = cloud.bounding_box().unwrap();
        assert_eq!(min.x, 0.0);
        assert_eq!(max.x, 1.0);
    }

    #[test]
    fn test_generator_creation() {
        let generator = PointCloudGenerator::new(PointCloudConfig::default());
        assert_eq!(generator.config.downsample_factor, 1);
    }

    #[test]
    fn test_from_depth_map_empty() {
        let generator = PointCloudGenerator::new(PointCloudConfig::default());
        let depth_map = DepthMap::new(640, 480, [500.0, 500.0, 320.0, 240.0]);
        let pose = CameraPose::identity();

        let cloud = generator.from_depth_map(&depth_map, &pose, None);
        assert_eq!(cloud.len(), 0); // No valid depths
    }

    #[test]
    fn test_from_depth_map_with_data() {
        let generator = PointCloudGenerator::new(PointCloudConfig::default());
        let mut depth_map = DepthMap::new(10, 10, [100.0, 100.0, 5.0, 5.0]);

        // Add some depths
        for v in 0..10 {
            for u in 0..10 {
                depth_map.set_depth(u, v, 1.0);
            }
        }

        let pose = CameraPose::identity();
        let cloud = generator.from_depth_map(&depth_map, &pose, None);

        assert!(cloud.len() > 0);
        assert_eq!(cloud.points.len(), cloud.colors.len());
    }

    #[test]
    fn test_depth_filtering() {
        let config = PointCloudConfig {
            min_depth: 0.5,
            max_depth: 2.0,
            ..Default::default()
        };
        let generator = PointCloudGenerator::new(config);
        let mut depth_map = DepthMap::new(10, 10, [100.0, 100.0, 5.0, 5.0]);

        // Add depths: some in range, some out
        depth_map.set_depth(0, 0, 0.1); // Too close
        depth_map.set_depth(1, 0, 1.0); // In range
        depth_map.set_depth(2, 0, 5.0); // Too far

        let pose = CameraPose::identity();
        let cloud = generator.from_depth_map(&depth_map, &pose, None);

        // Only middle point should be kept
        assert_eq!(cloud.len(), 1);
    }

    #[test]
    fn test_downsampling() {
        let config = PointCloudConfig {
            downsample_factor: 2,
            ..Default::default()
        };
        let generator = PointCloudGenerator::new(config);
        let mut depth_map = DepthMap::new(10, 10, [100.0, 100.0, 5.0, 5.0]);

        for v in 0..10 {
            for u in 0..10 {
                depth_map.set_depth(u, v, 1.0);
            }
        }

        let pose = CameraPose::identity();
        let cloud = generator.from_depth_map(&depth_map, &pose, None);

        // With downsampling factor 2, we expect roughly 1/4 the points
        assert!(cloud.len() < 100);
        assert!(cloud.len() >= 20); // At least some points
    }

    #[test]
    fn test_voxel_grid_downsample() {
        let generator = PointCloudGenerator::default();
        let mut cloud = PointCloud::new();

        // Add points in same voxel
        cloud.add_point(Point3D::new(0.01, 0.01, 0.01), Color::white());
        cloud.add_point(Point3D::new(0.02, 0.02, 0.02), Color::white());
        cloud.add_point(Point3D::new(1.0, 1.0, 1.0), Color::white());

        let downsampled = generator.downsample_voxel_grid(&cloud, 0.1);

        // First two points should merge into one voxel
        assert!(downsampled.len() < cloud.len());
    }

    #[test]
    fn test_camera_to_world() {
        let generator = PointCloudGenerator::default();
        let pose = CameraPose::identity();
        let camera_point = Point3D::new(1.0, 2.0, 3.0);

        let world_point = generator.camera_to_world(camera_point, &pose);

        // Identity should preserve coordinates
        assert!((world_point.x - 1.0).abs() < 0.001);
        assert!((world_point.y - 2.0).abs() < 0.001);
        assert!((world_point.z - 3.0).abs() < 0.001);
    }
}
