//! # Dense 3D Reconstruction Module
//!
//! Implements dense reconstruction using TSDF volumetric fusion.
//!
//! ## Overview
//!
//! This module builds dense 3D models from stereo depth maps and corrected
//! poses from loop closure detection.
//!
//! ## Components
//!
//! - **TSDF Volume** (8.2) - Volumetric fusion of depth maps
//! - **Point Cloud Generation** (8.4) - Dense point cloud extraction
//! - **Surface Extraction** (8.3) - Mesh generation via marching cubes
//!
//! ## Usage
//!
//! ```rust,ignore
//! use rs_vio::dense_reconstruction::{
//!     TSDFVolume, TSDFConfig, PointCloudGenerator, PointCloudConfig,
//!     MarchingCubes, MeshExtractionConfig
//! };
//!
//! // Volumetric fusion
//! let mut volume = TSDFVolume::new(TSDFConfig::default());
//! volume.integrate(&depth_map, &pose);
//!
//! // Point cloud generation
//! let generator = PointCloudGenerator::new(PointCloudConfig::default());
//! let cloud = generator.from_depth_map(&depth_map, &pose, Some(&image));
//!
//! // Mesh extraction
//! let mc = MarchingCubes::new(MeshExtractionConfig::default());
//! let mesh = mc.extract_mesh(&volume);
//! ```

pub mod mesh_extraction;
pub mod point_cloud;
pub mod tsdf_volume;

pub use mesh_extraction::{
    MarchingCubes, Mesh, MeshExtractionConfig, Normal, Triangle, Vertex,
};
pub use point_cloud::{
    Color, GrayImage, PointCloud, PointCloudConfig, PointCloudGenerator,
};
pub use tsdf_volume::{
    CameraPose, DepthMap, Point3D, TSDFConfig, TSDFVolume, Voxel,
};
