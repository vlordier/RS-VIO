# Phase 8: Dense Reconstruction - Planning

## Overview

Phase 8 implements **Dense 3D Reconstruction** using the corrected trajectory from Phase 7 loop closure detection. This creates dense point clouds or meshes from stereo imagery.

**Status**: 🔄 IN PROGRESS
**Prerequisites**: ✅ Phase 7 complete (corrected poses available)

---

## Architecture

```
Phase 8: Dense Reconstruction Pipeline
├─ 8.1: Depth Refinement (COMPLETE Phase 1-5)
│  ├─ Stereo matching (already implemented)
│  ├─ Disparity to depth conversion
│  └─ Depth map validation
│
├─ 8.2: Multi-View Fusion (NEW)
│  ├─ TSDF (Truncated Signed Distance Field) voxel grid
│  ├─ Volumetric integration from multiple views
│  └─ Voxel weight updates
│
├─ 8.3: Surface Extraction (NEW)
│  ├─ Marching cubes algorithm
│  ├─ Normal estimation
│  └─ Mesh simplification
│
└─ 8.4: Point Cloud Generation (NEW)
   ├─ Depth map to 3D points
   ├─ Color mapping
   └─ Outlier filtering

OUTPUTS:
├─ Dense Point Cloud (.ply format)
├─ Mesh (.obj or .ply format)
└─ Normal maps
```

---

## Implementation Plan

### 8.1: Depth Refinement ✅ (Already Complete)
- Existing stereo matching from Phase 1-5
- Disparity computation
- Depth validation

### 8.2: Multi-View Fusion (NEW - Priority 1)
**Purpose**: Fuse depth maps from multiple viewpoints into a consistent 3D volume

**Components**:
- TSDF voxel grid representation
- Volumetric integration (weight accumulation)
- View frustum culling
- Voxel traversal

**API Design**:
```rust
pub struct TSDFVolume {
    voxels: Vec<Voxel>,
    resolution: [usize; 3],
    voxel_size: f32,
    truncation_distance: f32,
}

impl TSDFVolume {
    pub fn new(config: TSDFConfig) -> Self;
    pub fn integrate(&mut self, depth_map: &DepthMap, pose: &Pose);
    pub fn extract_points(&self) -> Vec<Point3D>;
    pub fn extract_mesh(&self) -> Mesh;
}
```

**Tests**:
- Empty volume initialization
- Single depth map integration
- Multi-view fusion
- Truncation distance effects
- Weight accumulation
- Edge cases (empty regions, occluded areas)

**Estimated**: 350-400 lines, 8-10 tests

### 8.3: Surface Extraction (NEW - Priority 2)
**Purpose**: Extract mesh from TSDF volume using marching cubes

**Components**:
- Marching cubes lookup table
- Triangle generation
- Normal computation
- Mesh smoothing (optional)

**API Design**:
```rust
pub struct MarchingCubes {
    lookup_table: &'static [TriangleConfig],
}

impl MarchingCubes {
    pub fn extract_mesh(&self, volume: &TSDFVolume) -> Mesh;
    pub fn compute_normals(&self, mesh: &mut Mesh);
}

pub struct Mesh {
    pub vertices: Vec<Vertex>,
    pub triangles: Vec<Triangle>,
    pub normals: Vec<Normal>,
}
```

**Tests**:
- Marching cubes basic cases
- Normal computation
- Edge cases (flat surfaces, sharp edges)
- Mesh validation

**Estimated**: 300-350 lines, 6-8 tests

### 8.4: Point Cloud Generation (NEW - Priority 3)
**Purpose**: Generate dense point clouds from depth maps

**Components**:
- Depth map to 3D projection
- Color mapping from images
- Outlier filtering (statistical, radius)
- Downsampling (voxel grid)

**API Design**:
```rust
pub struct PointCloudGenerator {
    config: PointCloudConfig,
}

impl PointCloudGenerator {
    pub fn from_depth_map(&self, depth: &DepthMap, pose: &Pose, image: &Image) -> PointCloud;
    pub fn filter_outliers(&self, cloud: &mut PointCloud);
    pub fn downsample(&self, cloud: &PointCloud) -> PointCloud;
}

pub struct PointCloud {
    pub points: Vec<Point3D>,
    pub colors: Vec<Color>,
}
```

**Tests**:
- Depth to 3D conversion
- Color mapping
- Outlier filtering
- Downsampling
- Edge cases

**Estimated**: 250-300 lines, 6-8 tests

---

## Total Estimates

| Component | Lines | Tests | Priority |
|-----------|-------|-------|----------|
| 8.2 Multi-View Fusion | 350-400 | 8-10 | 1 |
| 8.3 Surface Extraction | 300-350 | 6-8 | 2 |
| 8.4 Point Cloud | 250-300 | 6-8 | 3 |
| **Total** | **900-1050** | **20-26** | - |

---

## Integration Points

### From Phase 7
- Corrected poses → Accurate depth map alignment
- Loop-free trajectory → No duplicate surfaces

### From Phase 1-5
- Stereo matching → Depth maps
- Camera calibration → Projection matrices
- Feature tracks → Sparse structure

### To External Tools
- Export `.ply` files for MeshLab/CloudCompare
- Export `.obj` files for Blender
- Export `.pcd` files for PCL

---

## Performance Targets

| Operation | Target | Notes |
|-----------|--------|-------|
| TSDF integration | <100ms/frame | Real-time capable |
| Marching cubes | <500ms | One-time extraction |
| Point cloud gen | <50ms/frame | Real-time capable |
| Memory usage | <1GB | For 512³ voxel grid |

---

## Implementation Order

1. **Phase 8.2: Multi-View Fusion** (Start here)
   - Core TSDF volume
   - Integration algorithm
   - Basic testing

2. **Phase 8.4: Point Cloud Generation**
   - Simpler than mesh extraction
   - Immediate visualization
   - Good for debugging

3. **Phase 8.3: Surface Extraction**
   - More complex (marching cubes)
   - Depends on TSDF volume
   - Final polish

---

## Next Steps

✅ Create TSDF volume structure
✅ Implement volumetric integration
✅ Test with synthetic depth maps
✅ Integrate with corrected poses from Phase 7

---

*Planning document for Phase 8*
*Ready to begin implementation*
