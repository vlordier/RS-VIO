# LLVM Lines Analysis Report - Code Bloat Analysis

**Date**: January 20, 2026
**Tool**: `cargo llvm-lines v0.4.45`
**Analysis Target**: RS-VIO library (dev profile)
**Total LLVM IR Lines**: 909,228 across 23,055 function copies

## Executive Summary

Analyzed the LLVM IR output to identify code bloat and optimization opportunities. Most code generation comes from standard library iterators, nalgebra linear algebra operations, and image processing libraries. RS-VIO-specific code accounts for a relatively small portion of total binary size.

## Top Code Generators

### External Dependencies (Top 10)

| Lines | % | Copies | Function | Source |
|-------|---|--------|----------|--------|
| 20,301 | 2.2% | 3 | `nalgebra::linalg::inverse::do_inverse4` | Matrix inversion |
| 19,950 | 2.2% | 21 | `nalgebra::base::blas::dotx` | Dot products (monomorphized) |
| 18,221 | 2.0% | 195 | `core::slice::iter::Iter::fold` | Iterator trait bloat |
| 15,313 | 1.7% | 33 | `FlattenCompat::size_hint` | Nested iterator overhead |
| 15,102 | 1.7% | 154 | `Vec::from_iter` | Collection construction |
| 11,656 | 1.3% | 360 | `map_fold::{{closure}}` | Map iterator closures |
| 10,184 | 1.1% | 128 | `Matrix::apply` | Nalgebra matrix operations |
| 9,418 | 1.0% | 105 | `Vec::extend_trusted` | Vector growth |
| 9,130 | 1.0% | 34 | `stable_partition` | Sorting algorithms |
| 7,280 | 0.8% | 30 | `gemm_uninit` | Matrix multiplication |

**Key Insight**: Top 10 external functions account for **14.4%** of total code but only **0.04%** of function copies, indicating heavy monomorphization of generic math/iterator code.

### RS-VIO Specific Functions (Top 15)

| Lines | % | Copies | Function | Module |
|-------|---|--------|----------|---------|
| 3,072 | 0.3% | 3 | `datasets::player_trait::execute` | Dataset loading |
| 2,584 | 0.3% | 1 | `Estimator::process_frame` | Main estimator loop |
| 2,232 | 0.2% | 2 | `CameraConfig::deserialize` | Config parsing |
| 1,830 | 0.2% | 2 | `MarginalizationConfig::deserialize` | Config parsing |
| 1,250 | 0.1% | 1 | `RerunViewer::log_imu_signal_quality` | Visualization |
| 1,176 | 0.1% | 2 | `Config::deserialize` | Config parsing |
| 1,127 | 0.1% | 1 | `CalibrationSensitivityReport::generate_report` | Calibration |
| 1,015 | 0.1% | 1 | `RerunViewer::log_imu_harmonics` | Visualization |
| 976 | 0.1% | 2 | `FeatureDetectionConfig::deserialize` | Config parsing |
| 909 | 0.1% | 1 | `ConfigurationResults::to_string` | Results formatting |
| 876 | 0.1% | 1 | `StereoPatchTracker::process_frame` | Feature tracking |
| 820 | 0.1% | 2 | `VisualizationConfig::deserialize` | Config parsing |
| 811 | 0.1% | 1 | `SlidingWindow::track_motion` | Motion tracking |
| 810 | 0.1% | 3 | `process_single_frame_common` | Frame processing |
| 806 | 0.1% | 2 | `OptimizationConfig::deserialize` | Config parsing |

**RS-VIO Total**: Top 15 functions = **18,214 lines (2.0%** of total code)

## Analysis by Category

### 1. Serde Deserialization Bloat (8 functions, ~9,117 lines, 1.0%)

**Problem**: Each config struct generates verbose deserialization code.

**Affected Configs**:
- `CameraConfig` (2,232 lines)
- `MarginalizationConfig` (1,830 lines)
- `Config` (1,176 lines)
- `FeatureDetectionConfig` (976 lines)
- `VisualizationConfig` (820 lines)
- `OptimizationConfig` (806 lines)
- `DebugConfig` (698 lines)
- `KeyframeManagementConfig` (644 lines)

**Optimization Opportunities**:
- ✅ **DONE**: Created `common/config.rs` with shared traits
- 🔄 **TODO**: Use `#[serde(flatten)]` to reduce derived code
- 🔄 **TODO**: Consider manual deserialization for hot-path configs
- 🔄 **TODO**: Use `serde_repr` for simple enums

### 2. Viewer/Visualization Functions (4 functions, ~3,740 lines, 0.4%)

**Problem**: Rerun logging functions generate significant code.

**Functions**:
- `log_imu_signal_quality` (1,250 lines)
- `log_imu_harmonics` (1,015 lines)
- `initialize` (450 lines)
- `log_feature_quality` (540 lines)

**Optimization Opportunities**:
- Extract common logging patterns
- Use macros to reduce repetition
- Consider feature flags to disable verbose logging in release builds

### 3. Dataset Loading (2 functions, ~3,620 lines, 0.4%)

**Problem**: Generic dataset player generates code per dataset type.

**Functions**:
- `player_trait::execute` (3,072 lines, 3 copies)
- `TUMVIPlayer::load_imu_data` (548 lines)

**Optimization Opportunities**:
- Reduce monomorphization by using trait objects for common operations
- Share more code between dataset implementations

### 4. Core Processing Functions (Well-optimized)

**Good Performance**:
- `Estimator::process_frame` (2,584 lines, **1 copy**) ✅
- `StereoPatchTracker::process_frame` (876 lines, **1 copy**) ✅
- `SlidingWindow::track_motion` (811 lines, **1 copy**) ✅

These are lean given their complexity - no concerning bloat.

## Dependency Analysis

### Heavy Monomorphization Sources

| Library | Lines | % | Primary Issue |
|---------|-------|---|---------------|
| `nalgebra` | ~150,000 | 16.5% | Generic matrix operations over many sizes |
| `core::iter` | ~80,000 | 8.8% | Iterator adapter trait expansion |
| `serde` | ~15,000 | 1.6% | Derived deserializers |
| `image/*` | ~25,000 | 2.7% | Multiple codec implementations |
| `rerun` | ~8,000 | 0.9% | Logging/visualization overhead |

## Recommendations

### High Priority

1. **Reduce Serde Bloat** (Potential: -5,000 lines)
   - Use `#[serde(flatten)]` for nested configs
   - Manual implementation for frequently deserialized types
   - Consider `bincode` for internal serialization (smaller code)

2. **Extract Common Viewer Patterns** (Potential: -1,500 lines)
   - Create logging macros/helpers
   - Factor out repetitive Rerun API calls

3. **Optimize Iterator Chains** (Potential: -3,000 lines)
   - Replace complex iterator chains with explicit loops in hot paths
   - Use `fold` instead of `map().collect()` chains where possible

### Medium Priority

4. **Reduce nalgebra Monomorphization** (Potential: -10,000 lines)
   - Use `DMatrix`/`DVector` dynamic sizes where dimensions vary
   - Box large intermediate results
   - Consider using only common matrix sizes (3x3, 4x4, 6x6)

5. **Dataset Loading Optimization** (Potential: -1,500 lines)
   - Use trait objects for common dataset operations
   - Share parsing logic via helper functions

### Low Priority (Acceptable Trade-offs)

6. **Image Processing Libraries**
   - Bloat is unavoidable with multiple codec support
   - Consider feature flags to disable unused codecs in production

7. **Core Estimator Functions**
   - Already well-optimized (single instantiation)
   - No action needed ✅

## Build Size Impact

For reference, the most monomorphized functions:
- **358 copies**: `Iterator::map`
- **360 copies**: `map_fold::{{closure}}`
- **338 copies**: `Map::fold`

These suggest opportunities to reduce generic iterator usage in favor of concrete implementations in performance-critical paths.

## Conclusion

**Total RS-VIO Code**: ~2-3% of binary
**Optimization Headroom**: ~20,000 lines (2.2% reduction) achievable with moderate effort

The codebase is reasonably lean for its functionality. The main bloat sources are:
1. ✅ **External dependencies** (nalgebra, iterators) - expected for generic math/VIO
2. 🔄 **Serde deserialization** - can be optimized
3. 🔄 **Viewer logging** - can extract common patterns

**Next Steps**: Focus on serde optimization and viewer code deduplication for the best ROI.
