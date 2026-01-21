//! Mesh extraction from TSDF volume using Marching Cubes
//!
//! Extracts triangle meshes from volumetric TSDF data.

use crate::dense_reconstruction::tsdf_volume::{Point3D, TSDFVolume};
use serde::{Deserialize, Serialize};

/// Configuration for mesh extraction
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct MeshExtractionConfig {
    /// ISO-surface value (typically 0.0 for TSDF)
    pub iso_value: f32,
    /// Minimum weight threshold for valid voxels
    pub min_weight: f32,
    /// Enable normal computation
    pub compute_normals: bool,
}

impl Default for MeshExtractionConfig {
    fn default() -> Self {
        Self {
            iso_value: 0.0,
            min_weight: 1.0,
            compute_normals: true,
        }
    }
}

/// 3D vertex in a mesh
#[derive(Debug, Clone, Copy)]
pub struct Vertex {
    pub position: Point3D,
}

impl Vertex {
    pub fn new(position: Point3D) -> Self {
        Self { position }
    }
}

/// 3D normal vector
#[derive(Debug, Clone, Copy)]
pub struct Normal {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Normal {
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    /// Normalize the vector
    pub fn normalize(&mut self) {
        let len = (self.x * self.x + self.y * self.y + self.z * self.z).sqrt();
        if len > 1e-6 {
            self.x /= len;
            self.y /= len;
            self.z /= len;
        }
    }

    /// Get normalized copy
    pub fn normalized(&self) -> Self {
        let mut n = *self;
        n.normalize();
        n
    }
}

impl Default for Normal {
    fn default() -> Self {
        Self::new(0.0, 0.0, 1.0)
    }
}

/// Triangle defined by 3 vertex indices
#[derive(Debug, Clone, Copy)]
pub struct Triangle {
    pub v0: usize,
    pub v1: usize,
    pub v2: usize,
}

impl Triangle {
    pub fn new(v0: usize, v1: usize, v2: usize) -> Self {
        Self { v0, v1, v2 }
    }
}

/// Triangle mesh
#[derive(Debug, Clone)]
pub struct Mesh {
    pub vertices: Vec<Vertex>,
    pub triangles: Vec<Triangle>,
    pub normals: Vec<Normal>,
}

impl Mesh {
    /// Create empty mesh
    pub fn new() -> Self {
        Self {
            vertices: Vec::new(),
            triangles: Vec::new(),
            normals: Vec::new(),
        }
    }

    /// Create with capacity
    pub fn with_capacity(vertex_capacity: usize, triangle_capacity: usize) -> Self {
        Self {
            vertices: Vec::with_capacity(vertex_capacity),
            triangles: Vec::with_capacity(triangle_capacity),
            normals: Vec::with_capacity(vertex_capacity),
        }
    }

    /// Add a vertex
    pub fn add_vertex(&mut self, vertex: Vertex) -> usize {
        let idx = self.vertices.len();
        self.vertices.push(vertex);
        idx
    }

    /// Add a triangle
    pub fn add_triangle(&mut self, triangle: Triangle) {
        self.triangles.push(triangle);
    }

    /// Get number of vertices
    pub fn num_vertices(&self) -> usize {
        self.vertices.len()
    }

    /// Get number of triangles
    pub fn num_triangles(&self) -> usize {
        self.triangles.len()
    }

    /// Check if mesh is valid
    pub fn is_valid(&self) -> bool {
        // Check all triangle indices are within bounds
        for tri in &self.triangles {
            if tri.v0 >= self.vertices.len()
                || tri.v1 >= self.vertices.len()
                || tri.v2 >= self.vertices.len()
            {
                return false;
            }
        }
        true
    }

    /// Compute normals for all vertices
    pub fn compute_normals(&mut self) {
        self.normals.clear();
        self.normals.resize(self.vertices.len(), Normal::default());

        // Compute face normals and accumulate to vertices
        for tri in &self.triangles {
            let v0 = &self.vertices[tri.v0].position;
            let v1 = &self.vertices[tri.v1].position;
            let v2 = &self.vertices[tri.v2].position;

            // Compute edge vectors
            let e1 = Point3D::new(v1.x - v0.x, v1.y - v0.y, v1.z - v0.z);
            let e2 = Point3D::new(v2.x - v0.x, v2.y - v0.y, v2.z - v0.z);

            // Cross product for face normal
            let nx = e1.y * e2.z - e1.z * e2.y;
            let ny = e1.z * e2.x - e1.x * e2.z;
            let nz = e1.x * e2.y - e1.y * e2.x;

            let face_normal = Normal::new(nx, ny, nz).normalized();

            // Accumulate to vertex normals
            self.normals[tri.v0].x += face_normal.x;
            self.normals[tri.v0].y += face_normal.y;
            self.normals[tri.v0].z += face_normal.z;

            self.normals[tri.v1].x += face_normal.x;
            self.normals[tri.v1].y += face_normal.y;
            self.normals[tri.v1].z += face_normal.z;

            self.normals[tri.v2].x += face_normal.x;
            self.normals[tri.v2].y += face_normal.y;
            self.normals[tri.v2].z += face_normal.z;
        }

        // Normalize all vertex normals
        for normal in &mut self.normals {
            normal.normalize();
        }
    }
}

impl Default for Mesh {
    fn default() -> Self {
        Self::new()
    }
}

/// Marching cubes mesh extractor
pub struct MarchingCubes {
    config: MeshExtractionConfig,
}

impl MarchingCubes {
    /// Create new marching cubes extractor
    pub fn new(config: MeshExtractionConfig) -> Self {
        Self { config }
    }

    /// Extract mesh from TSDF volume
    pub fn extract_mesh(&self, volume: &TSDFVolume) -> Mesh {
        let mut mesh = Mesh::new();
        let config = volume.config();

        // Simplified marching cubes: extract triangles where TSDF crosses zero
        // This is a simplified version - full marching cubes uses lookup tables

        for z in 0..(config.resolution_z - 1) {
            for y in 0..(config.resolution_y - 1) {
                for x in 0..(config.resolution_x - 1) {
                    self.process_voxel(volume, x, y, z, &mut mesh);
                }
            }
        }

        if self.config.compute_normals && !mesh.vertices.is_empty() {
            mesh.compute_normals();
        }

        mesh
    }

    /// Process a single voxel cube
    fn process_voxel(&self, volume: &TSDFVolume, x: usize, y: usize, z: usize, mesh: &mut Mesh) {
        // Get 8 corner values
        let corners = [
            self.get_voxel_value(volume, x, y, z),
            self.get_voxel_value(volume, x + 1, y, z),
            self.get_voxel_value(volume, x + 1, y + 1, z),
            self.get_voxel_value(volume, x, y + 1, z),
            self.get_voxel_value(volume, x, y, z + 1),
            self.get_voxel_value(volume, x + 1, y, z + 1),
            self.get_voxel_value(volume, x + 1, y + 1, z + 1),
            self.get_voxel_value(volume, x, y + 1, z + 1),
        ];

        // Check if any corners are invalid
        if corners.iter().any(|c| c.is_none()) {
            return;
        }

        let corner_values: Vec<f32> = corners.iter().map(|c| c.unwrap()).collect();

        // Simplified: if there's a sign change, add a triangle at the center
        let has_negative = corner_values.iter().any(|&v| v < self.config.iso_value);
        let has_positive = corner_values.iter().any(|&v| v > self.config.iso_value);

        if has_negative && has_positive {
            // Get voxel center position
            let center = volume.voxel_to_world(x, y, z);
            
            // Simple triangulation: create 2 triangles forming a quad
            let v0 = mesh.add_vertex(Vertex::new(center));
            let v1 = mesh.add_vertex(Vertex::new(Point3D::new(
                center.x + volume.config().voxel_size,
                center.y,
                center.z,
            )));
            let v2 = mesh.add_vertex(Vertex::new(Point3D::new(
                center.x,
                center.y + volume.config().voxel_size,
                center.z,
            )));

            mesh.add_triangle(Triangle::new(v0, v1, v2));
        }
    }

    /// Get TSDF value at voxel, returning None if invalid
    fn get_voxel_value(&self, volume: &TSDFVolume, x: usize, y: usize, z: usize) -> Option<f32> {
        volume.get_voxel(x, y, z).and_then(|voxel| {
            if voxel.weight >= self.config.min_weight {
                Some(voxel.tsdf)
            } else {
                None
            }
        })
    }
}

impl Default for MarchingCubes {
    fn default() -> Self {
        Self::new(MeshExtractionConfig::default())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dense_reconstruction::tsdf_volume::TSDFConfig;

    #[test]
    fn test_config_defaults() {
        let config = MeshExtractionConfig::default();
        assert_eq!(config.iso_value, 0.0);
        assert!(config.min_weight > 0.0);
        assert!(config.compute_normals);
    }

    #[test]
    fn test_vertex_creation() {
        let v = Vertex::new(Point3D::new(1.0, 2.0, 3.0));
        assert_eq!(v.position.x, 1.0);
    }

    #[test]
    fn test_normal_creation() {
        let n = Normal::new(1.0, 0.0, 0.0);
        assert_eq!(n.x, 1.0);
    }

    #[test]
    fn test_normal_normalization() {
        let mut n = Normal::new(3.0, 4.0, 0.0);
        n.normalize();
        
        let len = (n.x * n.x + n.y * n.y + n.z * n.z).sqrt();
        assert!((len - 1.0).abs() < 0.001);
    }

    #[test]
    fn test_triangle_creation() {
        let tri = Triangle::new(0, 1, 2);
        assert_eq!(tri.v0, 0);
        assert_eq!(tri.v1, 1);
        assert_eq!(tri.v2, 2);
    }

    #[test]
    fn test_mesh_creation() {
        let mesh = Mesh::new();
        assert_eq!(mesh.num_vertices(), 0);
        assert_eq!(mesh.num_triangles(), 0);
    }

    #[test]
    fn test_mesh_add_vertex() {
        let mut mesh = Mesh::new();
        let v = Vertex::new(Point3D::new(1.0, 2.0, 3.0));
        let idx = mesh.add_vertex(v);
        
        assert_eq!(idx, 0);
        assert_eq!(mesh.num_vertices(), 1);
    }

    #[test]
    fn test_mesh_add_triangle() {
        let mut mesh = Mesh::new();
        mesh.add_vertex(Vertex::new(Point3D::new(0.0, 0.0, 0.0)));
        mesh.add_vertex(Vertex::new(Point3D::new(1.0, 0.0, 0.0)));
        mesh.add_vertex(Vertex::new(Point3D::new(0.0, 1.0, 0.0)));
        
        mesh.add_triangle(Triangle::new(0, 1, 2));
        assert_eq!(mesh.num_triangles(), 1);
    }

    #[test]
    fn test_mesh_validation() {
        let mut mesh = Mesh::new();
        mesh.add_vertex(Vertex::new(Point3D::new(0.0, 0.0, 0.0)));
        mesh.add_vertex(Vertex::new(Point3D::new(1.0, 0.0, 0.0)));
        mesh.add_vertex(Vertex::new(Point3D::new(0.0, 1.0, 0.0)));
        
        // Valid triangle
        mesh.add_triangle(Triangle::new(0, 1, 2));
        assert!(mesh.is_valid());
        
        // Invalid triangle (index out of bounds)
        mesh.add_triangle(Triangle::new(0, 1, 10));
        assert!(!mesh.is_valid());
    }

    #[test]
    fn test_normal_computation() {
        let mut mesh = Mesh::new();
        mesh.add_vertex(Vertex::new(Point3D::new(0.0, 0.0, 0.0)));
        mesh.add_vertex(Vertex::new(Point3D::new(1.0, 0.0, 0.0)));
        mesh.add_vertex(Vertex::new(Point3D::new(0.0, 1.0, 0.0)));
        mesh.add_triangle(Triangle::new(0, 1, 2));
        
        mesh.compute_normals();
        
        assert_eq!(mesh.normals.len(), 3);
        
        // Check normals are normalized
        for normal in &mesh.normals {
            let len = (normal.x * normal.x + normal.y * normal.y + normal.z * normal.z).sqrt();
            assert!((len - 1.0).abs() < 0.01);
        }
    }

    #[test]
    fn test_marching_cubes_creation() {
        let mc = MarchingCubes::new(MeshExtractionConfig::default());
        assert_eq!(mc.config.iso_value, 0.0);
    }

    #[test]
    fn test_extract_mesh_empty_volume() {
        let mc = MarchingCubes::new(MeshExtractionConfig::default());
        let volume = TSDFVolume::new(TSDFConfig {
            resolution_x: 10,
            resolution_y: 10,
            resolution_z: 10,
            ..Default::default()
        });
        
        let mesh = mc.extract_mesh(&volume);
        
        // Empty volume should produce empty mesh
        assert_eq!(mesh.num_triangles(), 0);
    }

    #[test]
    fn test_mesh_with_capacity() {
        let mesh = Mesh::with_capacity(100, 200);
        assert_eq!(mesh.vertices.capacity(), 100);
        assert_eq!(mesh.triangles.capacity(), 200);
    }
}
