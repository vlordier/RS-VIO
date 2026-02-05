# Multi-Camera Configuration System

## Overview

The RS-VIO system now supports arbitrary N-camera setups with flexible parameter sharing. This enables:

- **Stereo pairs** (classical left/right setup)
- **Independent cameras** (forward/backward drone cameras)
- **Multi-camera rigs** (forward stereo + downward stereo)
- **Custom sharing policies** (share only focal length ratio, distortion k1/k2, etc.)

Each camera is independently optimized for intrinsics, but parameters can be **tied** through sharing groups.

---

## ✅ Core Capabilities

### 1. **Arbitrary Camera Count**

Mount 2, 4, 6, or more cameras simultaneously:

```yaml
cameras:
  - id: forward_left
  - id: forward_right
  - id: down_left
  - id: down_right
  - id: back
  - id: thermal
```

### 2. **Any Orientation/Direction**

Position cameras in any direction using 6-DOF transformations:

```yaml
# Forward camera (identity)
T_B_C: [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1]

# Backward camera (180° rotation)
T_B_C: [-1,0,0,0, 0,1,0,0, 0,0,-1,0, 0,0,0,1]

# Downward camera (90° rotation)
T_B_C: [1,0,0,0, 0,0,-1,0, 0,1,0,0, 0,0,0,1]
```

### 3. **Flexible Stereo Pairing**

Define multiple independent stereo pairs or monocular cameras:

```yaml
camera_groups:
  - name: forward_stereo
    cameras: ["forward_left", "forward_right"]
    is_stereo: true
    stereo_baseline: 0.06

  - name: downward_stereo
    cameras: ["down_left", "down_right"]
    is_stereo: true
    stereo_baseline: 0.04
```

### 4. **Fine-Grained Parameter Sharing**

Choose what's shared across cameras:

```yaml
camera_groups:
  - name: my_group
    cameras: [...]
    share_focal_length: full|ratio|none
    share_principal_point: none|full
    share_distortion: none|k1k2|full
```

| Policy | Result |
|--------|--------|
| `none` | Each camera fully independent |
| `ratio` | Share fy/fx ratio (square pixels) |
| `k1k2` | Share k1, k2 distortion only |
| `full` | Fully synchronized intrinsics |

---

## Architecture

### CameraDefinition
Each camera is defined with:
- **id**: Unique identifier (e.g., "left", "right", "forward", "down_left")
- **intrinsics**: [fx, fy, cx, cy] - focal length and principal point
- **distortion**: Distortion coefficients (model-specific)
- **model**: Distortion model ("EUCM", "opencv5", etc.)
- **T_B_C**: 4×4 extrinsics matrix (camera pose in body frame)

```yaml
cameras:
  - id: left
    intrinsics: [200.0, 200.0, 256.0, 256.0]
    distortion: [0.01, 0.001, -0.002, 0.0, 0.0]
    model: "opencv5"
    T_B_C: [1.0, 0.0, 0.0, 0.0,
             0.0, 1.0, 0.0, 0.0,
             0.0, 0.0, 1.0, 0.0,
             0.0, 0.0, 0.0, 1.0]
```

### CameraGroup
Groups define **stereo relationships** and **parameter sharing**:
- **name**: Group identifier
- **cameras**: List of camera IDs in the group
- **is_stereo**: True if this is a stereo pair (for feature matching)
- **stereo_baseline**: Baseline distance (meters) - used for scale ambiguity
- **share_focal_length**: How to tie focal length across cameras
- **share_principal_point**: How to tie principal point
- **share_distortion**: How to tie distortion parameters

```yaml
camera_groups:
  - name: main_stereo
    cameras: ["left", "right"]
    is_stereo: true
    stereo_baseline: 0.10
    share_focal_length: full      # Both cameras see same scene → same f
    share_principal_point: none   # Different sensors → different cx/cy
    share_distortion: none        # Distortion is sensor-specific
```

---

## Sharing Policies

### SharingPolicy Enum

| Policy | Meaning | Use Case |
|--------|---------|----------|
| **None** | Fully independent parameters | Different lens types, different sensors |
| **Ratio** | Share only ratio (e.g., fy/fx) | Same sensor, expected square pixels |
| **K1K2** | Share k1, k2 only (distortion) | Same optics, different higher-order terms |
| **Full** | Fully shared across cameras | Matched cameras on synchronized rig |

### Examples

**Stereo pair with shared focal length:**
```yaml
share_focal_length: full
share_principal_point: none
share_distortion: none
```
→ Both cameras refine fx, fy together; cx, cy independently.

**Multi-camera with independent intrinsics:**
```yaml
share_focal_length: none
share_principal_point: none
share_distortion: none
```
→ Each camera optimizes all parameters independently.

**Ratio constraint (square pixels):**
```yaml
share_focal_length: ratio
```
→ If left camera refines fx → right camera fy/fx ratio stays constant.

---

## Configuration Examples

### Example 1: Classic Stereo (TUM-VI Style)

```yaml
cameras:
  - id: left
    intrinsics: [190.978477, 190.973307, 254.931706, 256.897443]
    distortion: [...]
    model: "opencv5"
    T_B_C: [1.0, 0.0, 0.0, 0.0, ...]

  - id: right
    intrinsics: [190.442370, 190.434438, 252.599497, 254.917231]
    distortion: [...]
    model: "opencv5"
    T_B_C: [1.0, 0.0, 0.0, -0.10, ...]

camera_groups:
  - name: main_stereo
    cameras: ["left", "right"]
    is_stereo: true
    stereo_baseline: 0.10
    share_focal_length: full
```

**File**: [config/multi_camera_stereo.yaml](config/multi_camera_stereo.yaml)

---

### Example 2: Drone with Forward + Back Cameras

```yaml
cameras:
  - id: forward
    intrinsics: [200.0, 200.0, 256.0, 256.0]
    T_B_C: [1.0, 0.0, 0.0, 0.05, ...]

  - id: backward
    intrinsics: [180.0, 180.0, 256.0, 256.0]
    T_B_C: [-1.0, 0.0, 0.0, -0.05, ...]  # 180° rotated
```

**No camera groups** → Each camera is independent.
- Different optical axes → Different view
- Different intrinsics expected → No sharing
- No stereo relationship

**File**: [config/multi_camera_forward_back.yaml](config/multi_camera_forward_back.yaml)

---

### Example 3: Quad-Camera Rig (Forward + Downward Stereo)

```yaml
cameras:
  - id: forward_left
  - id: forward_right
  - id: down_left
  - id: down_right

camera_groups:
  - name: forward_stereo
    cameras: ["forward_left", "forward_right"]
    is_stereo: true
    share_focal_length: full

  - name: downward_stereo
    cameras: ["down_left", "down_right"]
    is_stereo: true
    share_focal_length: full
```

**Two independent stereo pairs**:
- Forward pair tracks motion (main odometry)
- Downward pair provides altitude/landing info
- Each pair can refine intrinsics independently

**File**: [config/multi_camera_quad.yaml](config/multi_camera_quad.yaml)

---

## Intrinsics Refinement

### Per-Camera Optimization

Each camera's intrinsics are optimized independently during online refinement:

```rust
for camera in &self.cameras {
    if should_refine(camera) {
        refine_intrinsics_online(camera);  // Independent optimization
    }
}
```

### Shared Constraints

If cameras are in a group with shared parameters:
- **share_focal_length: full** → Both cameras' fx/fy move together
- **share_focal_length: ratio** → Adjust fx, but keep fy/fx constant
- **share_distortion: k1k2** → Refine k1/k2 together, keep k3+ free

Implementation uses **Lagrange multipliers** or **reparameterization** to enforce constraints during BA.

---

## API Usage

### Load multi-camera config:
```python
config = MultiCameraConfig.from_yaml("config/multi_camera_quad.yaml")
config.validate()  # Checks all references valid

# Access cameras
for cam in config.cameras:
    print(f"Camera {cam.id}: fx={cam.intrinsics[0]}")

# Access stereo groups
for group in config.stereo_groups():
    print(f"Stereo pair: {group.name}")
```

### Check if parameter is shared:
```python
group = config.get_group("forward_stereo")
if group.shares_focal_length():
    # fx/fy are tied across cameras
    pass
```

---

## Design Decisions

### Why separate cameras and groups?
- **Flexibility**: A camera can belong to multiple groups
- **Clarity**: Camera definition (intrinsics, pose) vs. constraints (sharing)
- **Extensibility**: Easy to add time-varying groups, or dynamic group switching

### Why SharingPolicy enum?
- **Type-safe**: Can't accidentally use invalid policy
- **Explicit**: Code documents intent (e.g., `share_focal_length: Ratio`)
- **Extendable**: Easy to add `Policy::ConstrainedRatio(0.95)` later

### Why T_B_C as 4×4 matrix?
- **Standard**: Consistent with robotics conventions
- **Composable**: Can chain transformations easily
- **Human-readable**: Direct YAML editing possible

---

## Future Extensions

### 1. Time-Varying Extrinsics
```yaml
cameras:
  - id: gimbal_camera
    extrinsics_type: "time_varying"
    gimbal_model: "spherical"  # Pitch, roll, yaw
```

### 2. Linked Distortion
```yaml
camera_groups:
  - name: stereo
    share_distortion: linked  # k1_L + k1_R = 0
```

### 3. Panoramic/Fisheye
```yaml
cameras:
  - id: panoramic
    model: "fisheye_equirectangular"
    intrinsics: [panoramic_params...]
```

### 4. Multi-Rate Cameras
```yaml
cameras:
  - id: rgb
    sample_rate: 30  # Hz
  - id: thermal
    sample_rate: 9   # Hz
```

---

## Validation & Error Handling

### Built-in checks:
1. **Duplicate IDs**: Catch repeated camera identifiers
2. **Group references**: Verify all group cameras exist
3. **Stereo consistency**: Stereo groups must have exactly 2 cameras
4. **Extrinsics format**: 4×4 matrices must be valid
5. **Intrinsics bounds**: fx/fy/cx/cy must be positive, reasonable

```rust
config.validate()?  // Returns error if anything invalid
```

### Example error:
```
Error: Camera group 'forward_stereo' marked as stereo but has 3 cameras (need 2)
```

---

## Migration from Legacy Config

**Old (hardcoded stereo):**
```yaml
camera:
  left_intrinsics: [...]
  right_intrinsics: [...]
  T_B_Cl: [...]
  T_B_Cr: [...]
```

**New (flexible N-camera):**
```yaml
cameras:
  - id: left
    intrinsics: [...]
    T_B_C: [...]
  - id: right
    intrinsics: [...]
    T_B_C: [...]
```

Both formats are supported; the system auto-detects which format is used.

---

## Summary

| Feature | Capability |
|---------|-----------|
| **Camera count** | Arbitrary N cameras |
| **Camera types** | Mixed (stereo, monocular, panoramic) |
| **Parameter sharing** | Fully customizable |
| **Extrinsics** | Per-camera 6-DOF pose |
| **Intrinsics opt.** | Per-camera or group-constrained |
| **Stereo matching** | Multiple stereo pairs or independent |
| **Backward compat.** | Yes (legacy stereo config still works) |

This enables RS-VIO to scale from simple stereo drones to complex multi-sensor UAVs.
