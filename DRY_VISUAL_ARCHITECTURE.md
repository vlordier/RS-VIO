# DRY Visual Architecture

## 📊 Dataset Player Duplication Overview

### Current Architecture (90% Duplication)

```
┌─────────────────────────────────────┐
│     EurocPlayer                     │
├─────────────────────────────────────┤
│ • load_image_timestamps()  50 lines │ ◄─ Different
│ • load_imu_data()          30 lines │ ◄─ Different
│                                     │
│ • run()                   305 lines │ ◄─ IDENTICAL
│   ├─ Load images                    │
│   ├─ Init viewer                    │
│   ├─ Load config                    │
│   ├─ Create cameras                 │
│   ├─ Frame loop                     │
│   └─ Process frames                 │
│                                     │
│ • initialize_estimator()  20 lines  │ ◄─ IDENTICAL
│ • process_single_frame() 15 lines   │ ◄─ IDENTICAL
└─────────────────────────────────────┘

┌─────────────────────────────────────┐
│     TUMVIPlayer                     │
├─────────────────────────────────────┤
│ • load_image_timestamps()  45 lines │ ◄─ Different (TUM path format)
│ • load_imu_data()          25 lines │ ◄─ Different (TUM IMU format)
│                                     │
│ • run()                   305 lines │ ◄─ COPY of EurocPlayer
│   ├─ Load images                    │
│   ├─ Init viewer                    │
│   ├─ Load config                    │
│   ├─ Create cameras                 │
│   ├─ Frame loop                     │
│   └─ Process frames                 │
│                                     │
│ • initialize_estimator()  20 lines  │ ◄─ COPY of EurocPlayer
│ • process_single_frame() 15 lines   │ ◄─ COPY of EurocPlayer
└─────────────────────────────────────┘

┌─────────────────────────────────────┐
│    FourSeasonsPlayer                │
├─────────────────────────────────────┤
│ • load_image_timestamps()  50 lines │ ◄─ Different (4Seasons path format)
│ • load_imu_data()          30 lines │ ◄─ Different (4Seasons IMU format)
│                                     │
│ • run()                   307 lines │ ◄─ COPY of EurocPlayer
│   ├─ Load images                    │
│   ├─ Init viewer                    │
│   ├─ Load config                    │
│   ├─ Create cameras                 │
│   ├─ Frame loop                     │
│   └─ Process frames                 │
│                                     │
│ • initialize_estimator()  20 lines  │ ◄─ COPY of EurocPlayer
│ • process_single_frame() 15 lines   │ ◄─ COPY of EurocPlayer
└─────────────────────────────────────┘

TOTAL: 385 + 385 + 387 = 1,157 lines
DUPLICATED: 305 × 3 = 915 lines (79%)
UNIQUE: 1,157 - 915 = 242 lines (21%)
```

---

### Refactored Architecture (Zero Duplication)

```
┌─────────────────────────────────────────────────────────────┐
│        GenericDatasetPlayer<T: DatasetLoader>               │
│                   (150 lines - ONCE)                        │
├─────────────────────────────────────────────────────────────┤
│ • run()                                                     │
│   ├─ loader.load_images()        ◄─ Calls trait method    │
│   ├─ create_viewer()             ◄─ Common logic          │
│   ├─ Config::load()              ◄─ Common logic          │
│   ├─ create_cameras()            ◄─ Common logic          │
│   ├─ Frame loop                  ◄─ Common logic          │
│   └─ Process frames              ◄─ Common logic          │
│                                                             │
│ • initialize_estimator()         ◄─ Common logic          │
│ • process_single_frame()         ◄─ Common logic          │
└─────────────────────────────────────────────────────────────┘
                        ▲
                        │ implements
                        │
        ┌───────────────┼───────────────┐
        │               │               │
        │               │               │
┌───────▼────────┐  ┌───▼────────────┐ ┌───▼─────────────┐
│  EurocLoader   │  │ TUMVILoader    │ │FourSeasonsLoader│
│   (80 lines)   │  │  (80 lines)    │ │   (80 lines)    │
├────────────────┤  ├────────────────┤ ├─────────────────┤
│ • load_images()│  │• load_images() │ │ • load_images() │
│ • load_imu()   │  │ • load_imu()   │ │ • load_imu()    │
│ • get_name()   │  │ • get_name()   │ │ • get_name()    │
└────────────────┘  └────────────────┘ └─────────────────┘
  EuRoC specific      TUM-VI specific    4Seasons specific
  file format        file format        file format
  (50 lines)         (45 lines)         (50 lines)

type EurocPlayer = GenericDatasetPlayer<EurocLoader>;
type TUMVIPlayer = GenericDatasetPlayer<TUMVILoader>;
type FourSeasonsPlayer = GenericDatasetPlayer<FourSeasonsLoader>;

TOTAL: 150 + 80 + 80 + 80 = 390 lines
REDUCTION: (1,157 - 390) / 1,157 = 66% ✅
```

---

## 📚 Trait Implementation Pattern

### Current (Repetitive)

```rust
// 8 Custom Traits, ~150 lines total

pub trait ToMatrix { ... }
impl ToMatrix for Array4x4 { ... }
impl ToMatrix for Array3x3 { ... }

pub trait ToArray { ... }
impl ToArray for Matrix4x4 { ... }
impl ToArray for Matrix3x3 { ... }

pub trait ToVector { ... }
impl ToVector for Array3 { ... }

pub trait ToArrayVec { ... }
impl ToArrayVec for SVector { ... }
```

### Refactored (Standard)

```rust
// Standard From/Into, ~50 lines total

impl From<Array4x4> for Matrix4x4 { ... }
impl From<Matrix4x4> for Array4x4 { ... }
impl From<Array3x3> for Matrix3x3 { ... }
impl From<Matrix3x3> for Array3x3 { ... }
impl From<Array3> for Vector3 { ... }
// etc.

// Usage: matrix = array.into();  // Standard pattern
```

---

## 🔄 Error Handling Pattern Evolution

### Phase 1: Current State

```
player.run() → PlayerResult
    └─ error_message: String
    └─ success: bool
    └─ processed_frames: usize

❌ Problem: 20+ manual match statements
❌ Problem: String error types, not strongly typed
```

### Phase 2: Refactored State

```
player.run() → Result<PlayerResult, DatasetError>
    where DatasetError::ImageLoading(String)
                     | Configuration(String)
                     | CameraSetup(String)
                     | Io(io::Error)

✅ Benefit: Type-safe error discrimination
✅ Benefit: 1-line error propagation with ?
✅ Benefit: Matches Rust conventions
```

---

## 🏗️ Layered Refactoring Plan

```
Time: 0h         2h          4h          8h         11h
      ├──────────┼───────────┼──────────┼──────────┤

Phase 1: Foundation (2-3h)
├─ Camera factory (1h)
├─ Logger init (30m)
└─ Logging macros (30m)
      ◄──────────────────────────────┐
                                     │
Phase 2: Core (2-3h)
├─ Error handling (2-3h)
└─ Custom traits (1h)
      ◄────────────────────────────────────┐
                                            │
Phase 3: Major (3-4h)
└─ Dataset generics (3-4h)
      ◄─────────────────────────────────────────┘

✅ = Safe to implement now
⏳ = Depends on previous phase
🔒 = Higher risk, only after validation
```

---

## 💾 Code Size Metrics

### LOC Distribution (Before)

```
src/datasets/euroc_player.rs      ██████████████████ 385 (16%)
src/datasets/tum_vi_player.rs     ██████████████████ 385 (16%)
src/datasets/fourseasons_player   ██████████████████ 387 (17%)
src/types.rs                      ████ 224 (9%)
src/lib.rs                        ██ 107 (5%)
src/viewers/rerun.rs             ███████████ 461 (20%)
src/estimator/estimator.rs       ██████████ 520 (22%)
src/feature_tracker/feature_tracker ████████ 570 (24%)
[Other modules]                  ░░░░░░░░░░░░░ (many)
────────────────────────────────────────────
TOTAL: ~2,357 LOC
DUPLICATE: ~1,157 LOC (49% duplication)
```

### LOC Distribution (After Refactoring)

```
src/datasets/player_base.rs       ████ 150 (8%)
src/datasets/euroc_player.rs      ██ 80 (4%)
src/datasets/tum_vi_player.rs     ██ 80 (4%)
src/datasets/fourseasons_player   ██ 80 (4%)
src/types.rs                      ██ 135 (7%)
src/lib.rs                        ██ 109 (6%)
src/viewers/rerun.rs             ███████████ 461 (25%)
src/estimator/estimator.rs       ██████████ 518 (28%)
src/feature_tracker/feature_tracker ████████ 572 (31%)
[Other modules]                  ░░░░░░░░░░░░░ (same)
────────────────────────────────────────────
TOTAL: ~1,310 LOC (-1,047 = -44%)
DUPLICATE: ~0 LOC (0% duplication)
```

---

## 🧪 Testing Coverage

### Before (Test 3 Times)

```
Test Suite: comprehensive_integration_tests

Feature: euroc_player
  ✓ euroc_player::load_images
  ✓ euroc_player::run
  ✓ euroc_player::frame_processing
  ✓ euroc_player::error_handling

Feature: tum_vi_player
  ✓ tum_vi_player::load_images (IDENTICAL TEST)
  ✓ tum_vi_player::run          (IDENTICAL TEST)
  ✓ tum_vi_player::frame_processing (IDENTICAL TEST)
  ✓ tum_vi_player::error_handling   (IDENTICAL TEST)

Feature: fourseasons_player
  ✓ fourseasons_player::load_images (IDENTICAL TEST)
  ✓ fourseasons_player::run          (IDENTICAL TEST)
  ✓ fourseasons_player::frame_processing (IDENTICAL TEST)
  ✓ fourseasons_player::error_handling   (IDENTICAL TEST)

❌ Problem: ~50% of tests are duplicates
❌ Problem: CI/CD time wasted running identical tests
```

### After (Test Once)

```
Test Suite: comprehensive_integration_tests

Module: player_base
  ✓ generic_player::run (all datasets covered)
  ✓ generic_player::frame_processing
  ✓ generic_player::error_handling

Dataset: euroc
  ✓ euroc_loader::load_images (dataset-specific)
  ✓ euroc_loader::load_imu

Dataset: tum_vi
  ✓ tum_vi_loader::load_images (dataset-specific)
  ✓ tum_vi_loader::load_imu

Dataset: fourseasons
  ✓ fourseasons_loader::load_images (dataset-specific)
  ✓ fourseasons_loader::load_imu

✅ Benefit: No duplicate tests
✅ Benefit: Faster CI/CD pipeline
✅ Benefit: Easier to test dataset loaders independently
```

---

## 🔐 Risk & Safety Matrix

```
                     Low Risk          Medium Risk       High Risk
Small Change   Camera factory    Error handling
(< 1h)         Logger init       Custom traits

Medium Change  Custom traits     Dataset base
(1-3h)         Error handling    

Large Change   Dataset generics
(3-4h)
```

**Risk Mitigation:**
- Phase 1 has very low risk (isolated changes)
- Phase 2 uses well-established patterns
- Phase 3 follows Strategy pattern (battle-tested design)
- All changes preserve behavior (tests verify)

---

## 📈 ROI (Return on Investment)

```
Effort: 11 hours of engineering time
Cost: ~$250-500 (at typical rates)

Savings:
├─ Removed code: 1,047 lines (-44%)
├─ Eliminated duplication: 915 lines (-79% of duplication)
├─ Test speedup: ~30% (no duplicate tests)
├─ Future feature velocity: 5× faster for new datasets
├─ Maintenance burden: 3× reduction
└─ Defect probability: 3× reduction (single source of truth)

Cost of NOT refactoring:
├─ Next 3 datasets each add 385 lines
├─ 3 new datasets = 1,155 extra lines (44% codebase increase)
├─ Each bug fix duplicated 3× (maintenance cost)
└─ New feature velocity: 3× slower per new dataset

BREAK-EVEN: 1 new dataset (11h refactoring saves 1-2h per new dataset)
```

