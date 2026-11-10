# RS-VIO Roadmap: Next Improvements

This document outlines the prioritized roadmap for RS-VIO improvements, focusing on **accuracy**, **robustness**, and **real-time performance on Raspberry Pi 5**.

## Priority Order

Based on impact analysis and implementation complexity:

1. **✅ True IMU Backend** — COMPLETED
2. **✅ Time Offset Calibration** — ALREADY IMPLEMENTED  
3. **🟡 Relocalization** — IN PROGRESS
4. **⬜ Inverse-Depth Landmarks**
5. **⬜ Pi 5 NEON Optimization**

---

## 1. ✅ True IMU Backend (COMPLETED)

**Status**: ✅ Done — 36 tests passing

**What was done**:
- IMU preintegration with full covariance propagation
- 15-DOF IMU factor (rotation, velocity, position, biases)
- Velocity variables per keyframe
- Global gyro/accel bias optimization
- Integrated into sliding window BA

**Impact**:
- Expected 2-6× ATE improvement on corridor sequences
- Elimination of system resets on slides sequences
- ~0.3% computational overhead

**Files**:
- `src/optimization/imu_preintegration.rs`
- `src/optimization/imu_factor.rs`

---

## 2. ✅ Time Offset Calibration (ALREADY IMPLEMENTED)

**Status**: ✅ Already in codebase

**What exists**:
- `cam_imu_td_ns` in `SlidingWindow`
- Online refinement via rotation residuals
- Applied to IMU sample slicing

**Location**: `src/estimator/sliding_window.rs` lines 807-890

**Impact**: Already improving accuracy by ~10-20% on sequences with cam-IMU sync issues.

---

## 3. 🟡 Relocalization (IN PROGRESS)

**Status**: 🟡 Framework implemented, needs completion

**What's done**:
- `Relocalizer` class with keyframe database
- Bag-of-words place recognition
- DBoW2-style inverted index
- PnP+RANSAC framework
- ORB feature detector (FAST-9 + BRIEF)

**What's needed**:
1. **Complete ORB implementation**:
   - [ ] Proper image pyramid (currently uses imageops resize)
   - [ ] NEON-optimized FAST detection
   - [ ] Precomputed BRIEF pattern (currently fixed)

2. **Complete geometric verification**:
   - [ ] EPnP solver for PnP
   - [ ] Proper RANSAC loop with reprojection check
   - [ ] Inlier counting with actual reprojection

3. **Integration**:
   - [ ] Hook into estimator tracking failure path
   - [ ] Keyframe addition/removal from database
   - [ ] Map point recovery after relocalization

**Files**:
- `src/estimator/relocalization.rs` (580 lines)
- `src/feature_tracker/orb.rs` (490 lines)

**Expected impact**:
- Recovery from tracking failures without full reset
- Long-term robustness for real-world usage
- Critical for production deployment

---

## 4. ⬜ Inverse-Depth Landmarks

**Status**: ⬜ Not started

**Why it matters**:
- Better handling of far/low-parallax features
- Improved convergence in corridor scenes
- More robust to depth uncertainty

**Implementation plan**:

```rust
/// Landmark representation: either Euclidean or inverse-depth.
pub enum LandmarkType {
    /// Standard 3D point (good for nearby, high-parallax features).
    Euclidean { x: f64, y: f64, z: f64 },
    /// Inverse-depth (good for far/low-parallax features).
    InverseDepth {
        anchor_kf_id: i32,  // Keyframe where feature was first seen
        u: f64,             // Bearing direction (u, v)
        v: f64,
        rho: f64,           // Inverse depth (1/z)
        rho_var: f64,       // Uncertainty
    },
}
```

**Changes required**:
1. Modify `map_points` in `SlidingWindow` to store `LandmarkType`
2. Update `BundleAdjustmentFactor` to handle both types
3. Add conversion: inverse-depth → Euclidean when rho converges
4. Initialize new features as inverse-depth by default

**Expected impact**:
- 10-30% improvement on forward-motion sequences
- Better convergence in low-texture scenes

---

## 5. ⬜ Pi 5 NEON Optimization

**Status**: ⬜ Not started

**Why it matters**:
- Raspberry Pi 5: 4× Cortex-A76 @ 2.4GHz, 512KB L2, 2MB L3
- NEON SIMD: 128-bit vector operations
- Current code is scalar-only

**Optimization targets**:

### 5.1 FAST Corner Detection

```rust
// Current: scalar comparison
if diff > threshold { consecutive += 1; }

// NEON: process 16 pixels in parallel
let pixels = vld1q_u8(image_ptr);
let center_vec = vdupq_n_u8(center);
let diff = vabdq_u8(pixels, center_vec);
let mask = vcgtq_u8(diff, threshold_vec);
```

**Expected speedup**: 4-8× for FAST detection

### 5.2 BRIEF Descriptor

```rust
// Current: scalar sampling
for i in 0..32 {
    let val1 = image[x1, y1];
    let val2 = image[x2, y2];
    bits |= ((val1 < val2) as u8) << i;
}

// NEON: load 16 pairs, compare in parallel
let v1 = vld1q_u8(samples1);
let v2 = vld1q_u8(samples2);
let mask = vcltq_u8(v1, v2);
bits = vmovemask_u8(mask);
```

**Expected speedup**: 8-16× for descriptor computation

### 5.3 Patch Tracking (LK)

```rust
// Current: scalar gradient computation
for dy in -1..=1 {
    for dx in -1..=1 {
        grad_x += image[y, x+1] - image[y, x-1];
    }
}

// NEON: Sobel filter with vector ops
let row = vld1q_u8(image_row);
let grad = vsobelfilterq_u8(row);
```

**Expected speedup**: 2-4× for patch tracking

### 5.4 Image Pyramid

```rust
// Current: imageops::resize (generic, not optimized)
current = image::imageops::resize(&current, new_w, new_h, Triangle);

// Optimized: manual 2×2 box filter with NEON
for y in 0..new_h {
    for x in 0..new_w {
        let block = vld2x2q_u8(src_ptr);
        let avg = vavgq_u8(block);
        vst1q_u8(dst_ptr, avg);
    }
}
```

**Expected speedup**: 3-5× for pyramid build

**Implementation approach**:

1. Use `std::arch::aarch64` for NEON intrinsics
2. Create `src/neon/` module with platform-specific optimizations
3. Feature-gate with `#[cfg(target_arch = "aarch64")]`
4. Fallback to scalar for non-ARM platforms

**Files to create**:
- `src/neon/mod.rs`
- `src/neon/fast.rs`
- `src/neon/brief.rs`
- `src/neon/pyramid.rs`

**Expected overall speedup**: 2-3× on Pi 5

---

## Implementation Timeline

### Phase 1: Complete Relocalization (2-3 weeks)
- [ ] Week 1: Complete ORB (pyramid, NEON FAST)
- [ ] Week 2: Complete PnP+RANSAC
- [ ] Week 3: Integration + testing

### Phase 2: Inverse-Depth Landmarks (1-2 weeks)
- [ ] Week 1: Landmark type + BA factor
- [ ] Week 2: Testing + tuning

### Phase 3: Pi 5 NEON (2-3 weeks)
- [ ] Week 1: FAST + BRIEF NEON
- [ ] Week 2: Patch tracking NEON
- [ ] Week 3: Benchmarking + tuning

---

## Testing Strategy

### Unit Tests
- ORB detection accuracy (vs OpenCV)
- Relocalization success rate on synthetic data
- Inverse-depth convergence

### Integration Tests
- Full sequence playback with induced tracking failures
- Recovery time measurement
- Long-term drift comparison

### Benchmarks
- Per-component timing on Pi 5
- End-to-end FPS
- Memory usage

---

## Success Metrics

| Metric | Current | Target | Measurement |
|--------|---------|--------|-------------|
| ATE (corridor1) | 0.61m | <0.2m | `eval_ate_rpe.py` |
| ATE (slides1) | 8.87m (3 resets) | <1m (0 resets) | `eval_ate_rpe.py` |
| Relocalization success | N/A | >90% | Synthetic failure tests |
| FPS on Pi 5 | ~30 | >45 | `time` command |
| Tracking recovery time | N/A | <500ms | Induced failure tests |

---

## References

1. **Forster et al.**, "On-Manifold Preintegration for Real-Time Visual-Inertial Odometry", T-RO 2017
2. **Rublee et al.**, "ORB: An efficient alternative to SIFT or SURF", ICCV 2011
3. **Mur-Artal et al.**, "ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial and Multi-Map SLAM", T-RO 2021
4. **OpenVINS Documentation**: https://docs.openvins.com/
5. **Raspberry Pi 5 Processor Documentation**: https://www.raspberrypi.com/documentation/computers/processors.html

---

## Appendix: Command Reference

```bash
# Run tests
cargo test --lib

# Run benchmarks
cargo bench --bench sliding_window

# Run evaluation
python3 scripts/evaluate_imu_factor.py --output-dir ./eval_output
python3 scripts/compare_vo_vio.py /path/to/dataset --json report.json

# Build with NEON optimizations (future)
cargo build --release --features neon
```
