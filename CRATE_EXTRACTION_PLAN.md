# Crate Extraction Plan for RS-VIO

## Overview
The RS-VIO codebase contains **2.4 MB of Rust source** across **213 files** organized into 21 top-level modules. This analysis identifies opportunities to extract reusable, self-contained modules into separate crates to reduce monolith complexity while maintaining clean API boundaries.

---

## Module Inventory by Size

| Module | Size | LOC | Category | Extractability |
|--------|------|-----|----------|-----------------|
| src/optimization/ | 472K | ~4000 | Core | Keep |
| src/feature_tracker/ | 368K | ~3200 | Core | Keep |
| src/estimator/ | 356K | ~3100 | Core | Keep |
| src/imu/ | 256K | ~2200 | Core | Keep |
| src/evaluation/ | 132K | ~1200 | Analysis | 🟢 Extract |
| src/vision/ | 160K | ~1400 | Algorithms | 🟢 Extract |
| src/datasets/ | 104K | ~800 | I/O | 🟡 Optional |
| src/calibration/ | 220K | ~2000 | Core | Keep |
| src/feature_detection/ | 52K | ~450 | Algorithms | 🟡 Optional |
| src/dense_reconstruction/ | 52K | ~500 | Research | 🟢 Extract |
| src/viewers/ | 84K | ~700 | UI | 🟡 Optional |
| src/multi_drone/ | 76K | ~800 | Research | 🟢 Extract |
| src/camera/ | 16K | ~150 | Models | 🟢 Extract |
| src/swarm/ | 48K | ~400 | Research | 🟢 Extract |
| src/gpu/ | 16K | ~200 | Acceleration | 🟡 Optional |
| src/common/ | 88K | ~850 | Utilities | 🟢 Extract |
| src/traits.rs | 326 LOC | Core | 🟢 Extract |
| src/types.rs | 256 LOC | Core | 🟢 Extract |
| src/lib.rs | 290 LOC | Core | Keep |

**Legend**: 
- 🟢 Extract: Pure, reusable, zero VIO-specific coupling
- 🟡 Optional: Useful for some use cases, light coupling
- Keep: Core algorithm; too tightly coupled

---

## Recommended Extraction Strategy

### Phase 1: Foundation Crates (Must Extract First)

These enable all subsequent extractions:

#### 1. `rs-vio-types` (New Public Crate)
**Current**: `src/types.rs` (256 LOC)
**Dependencies**: `nalgebra`
**Usage**: Imported by ~80% of codebase
**Reason**: Pure type aliases; foundational; enables other crates
**Action**: 
```bash
# Create new crate with Cargo.toml
cargo new --lib crates/rs-vio-types
mv src/types.rs crates/rs-vio-types/src/lib.rs
# Update RS-VIO Cargo.toml: add [dependencies] rs-vio-types = { path = "crates/rs-vio-types" }
```
**Benefits**: 
- Types can be used by external projects
- Separates type concerns from algorithm logic
- No impact on main crate's build time

---

#### 2. `rs-vio-traits` (New Public Crate)
**Current**: `src/traits.rs` (326 LOC) + `src/common/config.rs` trait section (~120 LOC)
**Dependencies**: `rs-vio-types`
**Usage**: Implemented by estimator, fusion, vision modules
**Reason**: Pure interfaces; define architecture boundaries
**Action**:
```bash
cargo new --lib crates/rs-vio-traits
# Move traits.rs + consolidate common/config traits
# Keep: Convert, Strategy, StateView, StateTransform, Viewer, FrameProcessor
#       Clampable, ConfigBuilder, Mergeable, Validatable
```
**Benefits**:
- Traits become stable public API
- External implementations possible (alternative estimators, fusion algorithms)
- Clearer architectural contracts

---

#### 3. `rs-vio-common` (New Public Crate)
**Current**: `src/common/` (88K)
  - `config.rs` (config builders, validation) - 240 LOC
  - `error.rs` (error types, traits) - 180 LOC
  - `macros.rs` (debug/perf macros) - 80 LOC
  - `math.rs` (matrix utilities) - 160 LOC
  - `validation.rs` (range checkers) - 41 LOC
  - `safe_convert.rs` (type conversion) - 60 LOC
  - `perf.rs` (performance monitoring) - 120 LOC
  - `realtime_monitor.rs` (system monitoring) - 200 LOC

**Dependencies**: `rs-vio-types`, `rs-vio-traits`
**Usage**: Used by estimator, imu, optimization
**Reason**: Pure utilities; zero algorithm logic
**Action**:
```bash
cargo new --lib crates/rs-vio-common
# Move entire src/common/ directory
```
**Benefits**:
- Reusable math/error/config utilities
- Can be used independently for other VIO projects
- Reduces monolith by 88K

---

### Phase 2: Algorithm Extraction (Extract Next)

#### 4. `rs-vio-vision` (Public Crate)
**Current**: `src/vision/` (160K, 1400 LOC)
  - `imu_aided_tracking.rs` - IMU-informed feature prediction
  - `rolling_shutter_correction.rs` - Camera shutter model
  - `stereo_super_resolution.rs` - Subpixel refinement
  - `motion_aware_depth_optimization.rs` - Geometric constraints
  - `temporal_super_resolution.rs` - Multi-frame fusion
  - `adaptive_fusion_algorithm.rs` - Confidence weighting
  - `subpixel_disparity.rs` - Patch matching
  - `visualization.rs` - Analysis plots

**Dependencies**: `rs-vio-types`, `rs-vio-common`, `nalgebra`
**Usage**: Called by feature_tracker for post-processing
**Reason**: 
- No dependency on estimator, optimization, or state machine
- Uses only types and math utilities
- Provides alternative algorithm implementations
- Complete encapsulation: input (features, depths) → output (refined features, depths)

**Action**:
```bash
cargo new --lib crates/rs-vio-vision
mv src/vision crates/rs-vio-vision/src/
# Update Cargo.toml deps: rs-vio-types, rs-vio-common
# Update RS-VIO: rs-vio-vision = { path = "crates/rs-vio-vision" }
```

**Benefits**:
- Reusable vision library for other projects
- Can improve independently (optimize super-resolution, add new detectors)
- Separates vision R&D from core VIO

---

#### 5. `rs-vio-evaluation` (Public Crate)
**Current**: `src/evaluation/` (132K, 1200 LOC)
  - `trajectory_metrics.rs` - ATE/RPE computation
  - `depth_metrics.rs` - Depth accuracy
  - `feature_metrics.rs` - Reprojection error
  - `calibration_monitor.rs` - Parameter drift tracking
  - `calibration_quality.rs` - Quality factors
  - `robustness_metrics.rs` - Outlier resilience
  - `results.rs` - Result serialization
  - `metrics/` - Binned metrics, distance-speed analysis

**Dependencies**: `rs-vio-types`, `rs-vio-common`, `nalgebra`, `serde`
**Usage**: 
- Benchmarking binary only (`bin/`)
- NOT used by runtime estimator
- **Completely optional for production**

**Reason**:
- Zero imports by core modules
- Only used for offline evaluation scripts
- Heavy serde dependencies for serialization
- Can be excluded from release binary

**Action**:
```bash
cargo new --lib crates/rs-vio-evaluation
mv src/evaluation crates/rs-vio-evaluation/src/
# Update Cargo.toml to make optional feature
# In RS-VIO Cargo.toml: [features] evaluation = ["rs-vio-evaluation"]
```

**Benefits**:
- Eliminates 132K from production binary
- Shareable evaluation framework
- Independent benchmarking tools

---

#### 6. `rs-vio-camera` (Public Crate)
**Current**: `src/camera/rolling_shutter.rs` (16K)
**Dependencies**: `rs-vio-types`, `nalgebra`
**Usage**: feature_tracker, vision modules
**Reason**:
- Pure camera model library
- Could be used for other vision projects (SLAM, SfM)
- Zero VIO-specific logic

**Action**:
```bash
cargo new --lib crates/rs-vio-camera
mv src/camera crates/rs-vio-camera/src/
```

---

### Phase 3: Research Module Extraction (Optional)

These are currently zero-integrated; move to workspace but keep in repo:

#### 7. `rs-vio-dense` (Private Crate)
**Current**: `src/dense_reconstruction/` (52K)
  - Point cloud generation
  - Mesh extraction
  - TSDF volumetric reconstruction
**Status**: Zero imports from main pipeline; complete feature
**Action**: Move to `crates/rs-vio-dense/` as research module

#### 8. `rs-vio-swarm` (Private Crate)
**Current**: `src/swarm/` (48K)
  - Multi-robot communication
  - Swarm coordination
**Status**: Zero imports from main; isolated
**Action**: Move to `crates/rs-vio-swarm/`

#### 9. `rs-vio-multidrone` (Private Crate)
**Current**: `src/multi_drone/` (76K)
  - Boids flocking
  - Distributed optimization
  - Map merging
**Status**: Zero imports from main
**Action**: Move to `crates/rs-vio-multidrone/`

---

## Extraction Priority Order

**Do This First** (enables other crates):
1. ✅ `rs-vio-types` - Foundation
2. ✅ `rs-vio-traits` - API contracts
3. ✅ `rs-vio-common` - Utilities

**Do Next** (major modularity wins):
4. `rs-vio-vision` - Largest extractable algorithm module (160K)
5. `rs-vio-evaluation` - Heaviest (132K), optional
6. `rs-vio-camera` - Standalone camera models (16K)

**Then** (lower priority research):
7. `rs-vio-dense` - Dense reconstruction
8. `rs-vio-swarm` - Multi-robot
9. `rs-vio-multidrone` - Swarm algorithms

---

## Keep in Main Crate

These are too tightly coupled with state machine logic:

- **src/estimator/** (356K) - State machine, sliding window, IMU processor
- **src/optimization/** (472K) - Factor graphs, marginalization, bundle adjustment, loop closure
- **src/imu/** (256K) - IMU preintegration, filtering, state propagation
- **src/feature_tracker/** (368K) - Feature detection pipeline, frame-to-frame tracking
- **src/calibration/** (220K) - Parameter estimation, intrinsics/extrinsics
- **src/datasets/** (104K, optional) - Data loading; could extract later if needed
- **src/feature_detection/** (52K) - Feature detector implementations
- **src/viewers/** (84K) - Visualization; could extract later
- **src/output/** - Output formatting

---

## Workspace Structure After Extraction

```
RS-VIO/                           (mono-repo root)
├── Cargo.toml                     (workspace)
├── src/                           (main crate: rs-vio)
│   ├── estimator/                 (keep)
│   ├── optimization/              (keep)
│   ├── imu/                       (keep)
│   ├── feature_tracker/           (keep)
│   ├── calibration/               (keep)
│   ├── datasets/                  (keep)
│   ├── feature_detection/         (keep)
│   ├── viewers/                   (keep)
│   ├── output/                    (keep)
│   ├── lib.rs                     (updated to re-export from crates)
│   └── bin/                       (binaries)
│
├── crates/                        (extracted crates)
│   ├── rs-vio-types/
│   │   ├── Cargo.toml
│   │   └── src/lib.rs             (types.rs)
│   │
│   ├── rs-vio-traits/
│   │   ├── Cargo.toml
│   │   └── src/
│   │       ├── lib.rs
│   │       ├── convert.rs
│   │       ├── strategy.rs
│   │       ├── state.rs
│   │       └── config.rs           (consolidated config traits)
│   │
│   ├── rs-vio-common/
│   │   ├── Cargo.toml
│   │   └── src/lib.rs              (union of config, error, macros, math, validation, etc.)
│   │
│   ├── rs-vio-vision/
│   │   ├── Cargo.toml
│   │   └── src/lib.rs
│   │
│   ├── rs-vio-evaluation/
│   │   ├── Cargo.toml
│   │   └── src/lib.rs
│   │
│   ├── rs-vio-camera/
│   │   ├── Cargo.toml
│   │   └── src/lib.rs
│   │
│   ├── rs-vio-dense/               (research)
│   │   ├── Cargo.toml
│   │   └── src/lib.rs
│   │
│   ├── rs-vio-swarm/               (research)
│   │   ├── Cargo.toml
│   │   └── src/lib.rs
│   │
│   └── rs-vio-multidrone/          (research)
│       ├── Cargo.toml
│       └── src/lib.rs
```

---

## Estimated Impact

### Monolith Reduction
- **Before**: 1 crate, 213 files, 2.4 MB
- **After Phase 1** (types, traits, common): Remove 352K → **2.05 MB**
- **After Phase 2** (vision, evaluation, camera): Remove 308K → **1.74 MB** ✅
- **After Phase 3** (research): Remove 176K → **1.57 MB**

### Compilation Time
- Parallel builds of independent crates
- Evaluation crate optional (can skip for release builds)
- Vision algorithms can improve independently

### Testing
- Crate-specific tests run in parallel
- Easier to isolate regressions
- Research modules can have separate CI

### Reusability
- `rs-vio-types`: Generic types for any vision system
- `rs-vio-traits`: Architecture contract for plug-in algorithms
- `rs-vio-common`: Utilities for other Rust VIO projects
- `rs-vio-vision`: Vision library for non-VIO applications
- `rs-vio-camera`: Camera models for SfM, SLAM, VO systems

---

## Migration Checklist

- [ ] Phase 1: Extract types → traits → common
- [ ] Update all imports in main crate
- [ ] Verify tests pass across crates
- [ ] Phase 2: Extract vision → evaluation → camera
- [ ] Update workspace Cargo.toml
- [ ] Phase 3: Move research modules
- [ ] Update CI/CD for multi-crate builds
- [ ] Document public APIs for extracted crates
- [ ] Update README with new project structure

---

## Notes

1. **Types First**: `rs-vio-types` must be created before others
2. **Traits Second**: `rs-vio-traits` depends on types
3. **Common Third**: `rs-vio-common` depends on types and traits
4. **Algorithm Crates**: Can extract in any order after Phase 1
5. **Research Modules**: Can remain in workspace indefinitely; they're isolated
6. **Testing**: Each phase should pass all tests before proceeding
7. **Documentation**: Public crates need robust API docs
8. **Versioning**: Consider semantic versioning for extracted crates

---

## Quick Start (Phase 1 Only)

```bash
# Create workspace crates
cargo new --lib crates/rs-vio-types
cargo new --lib crates/rs-vio-traits
cargo new --lib crates/rs-vio-common

# Move files
mv src/types.rs crates/rs-vio-types/src/lib.rs
mv src/traits.rs crates/rs-vio-traits/src/lib.rs
mv src/common crates/rs-vio-common/src/

# Update main Cargo.toml with workspace and crate dependencies
# Verify: cargo test --all
```

This plan reduces the monolith by **~700 KB** while maintaining backward compatibility through re-exports.
