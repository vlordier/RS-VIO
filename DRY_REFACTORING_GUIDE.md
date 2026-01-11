# DRY & Struct Refactoring Guide

## Overview

The RS-VIO codebase has significant **code duplication and structural opportunities**. This guide identifies DRY violations and provides solutions using Rust structs to eliminate repetition.

---

## 🔴 CRITICAL: Three Dataset Players (90%+ Duplication)

### Problem

**Files affected:**
- `src/datasets/euroc_player.rs` (385 lines)
- `src/datasets/tum_vi_player.rs` (385 lines)
- `src/datasets/fourseasons_player.rs` (387 lines)

**Duplicated pattern** (lines 25-90 in each file):

```rust
pub fn run(&self, config: PlayerConfig) -> PlayerResult {
    let mut result = PlayerResult::default();

    // ❌ IDENTICAL: Load image timestamps
    let image_data = match Self::load_image_timestamps(&config.dataset_path) {
        Ok(data) => {
            if data.is_empty() {
                result.error_message = "No images found in dataset".to_string();
                return result;
            }
            data
        },
        Err(e) => {
            result.error_message = format!("Failed to load image timestamps: {}", e);
            return result;
        },
    };

    let start_frame_idx = 0;
    let end_frame_idx = image_data.len();

    // ❌ IDENTICAL: Initialize viewer
    let viewer: Option<Box<dyn Viewer>> = match create_viewer() {
        Ok(v) => {
            log::info!("[EurocPlayer] Viewer initialized successfully");
            Some(v)
        },
        Err(e) => {
            log::warn!("Failed to initialize viewer: {}", e);
            None
        },
    };

    // ❌ IDENTICAL: Load config
    let cfg = match Config::load(&config.config_path) {
        Ok(c) => c,
        Err(e) => {
            result.error_message =
                format!("Failed to load config '{}': {}", config.config_path, e);
            return result;
        },
    };

    // ❌ IDENTICAL: Create camera models
    let (left_cam, right_cam) = match Self::create_camera_models_from_config(&cfg) {
        Ok(cams) => cams,
        Err(e) => {
            result.error_message = format!("Failed to create camera models: {}", e);
            return result;
        },
    };

    // ❌ IDENTICAL: Initialize estimator
    let mut estimator = Estimator::new_with_cameras(cfg, viewer, Some(left_cam), Some(right_cam));
    Self::initialize_estimator(&mut estimator, &image_data);
    
    // ❌ NEAR-IDENTICAL: Frame loop (only minor dataset-specific loading differences)
}
```

### Impact

- **1,157 lines** of duplicated code across 3 files
- Each player has **dataset-specific methods** (load_image_timestamps, load_imu_data, etc.)
- Core **orchestration logic** is identical

### Solution: Strategy Pattern with Generic Base

**Create a `DatasetPlayer` base struct** that handles common logic:

```rust
// src/datasets/player_base.rs (NEW)

use std::fmt::Debug;

pub trait DatasetLoader: Debug {
    /// Load images from dataset-specific paths
    fn load_images(&self, dataset_path: &Path) -> Result<ImageData>;
    
    /// Load IMU data (optional)
    fn load_imu_data(&self, dataset_path: &Path) -> Result<Vec<ImuData>> {
        Ok(Vec::new())  // Default: no IMU data
    }
    
    /// Dataset-specific configuration loading
    fn get_dataset_name(&self) -> &'static str;
}

/// Generic dataset player that works with any DatasetLoader
pub struct GenericDatasetPlayer<T: DatasetLoader> {
    loader: T,
}

impl<T: DatasetLoader> GenericDatasetPlayer<T> {
    pub fn new(loader: T) -> Self {
        Self { loader }
    }

    /// Unified run method for all datasets
    pub fn run(&self, config: PlayerConfig) -> PlayerResult {
        let mut result = PlayerResult::default();

        // Load images using dataset-specific loader
        let image_data = match self.loader.load_images(&config.dataset_path) {
            Ok(data) => {
                if data.is_empty() {
                    result.error_message = "No images found in dataset".to_string();
                    return result;
                }
                data
            }
            Err(e) => {
                result.error_message = format!("Failed to load images: {}", e);
                return result;
            }
        };

        // Common logic (used by all datasets)
        let viewer = match create_viewer() {
            Ok(v) => {
                log::info!("[{}] Viewer initialized successfully", self.loader.get_dataset_name());
                Some(v)
            }
            Err(e) => {
                log::warn!("Failed to initialize viewer: {}", e);
                None
            }
        };

        let cfg = match Config::load(&config.config_path) {
            Ok(c) => c,
            Err(e) => {
                result.error_message = format!("Failed to load config: {}", e);
                return result;
            }
        };

        // ... rest of common logic
    }
}
```

**Then refactor each player:**

```rust
// src/datasets/euroc_player.rs (REFACTORED)

pub struct EurocLoader;

impl DatasetLoader for EurocLoader {
    fn load_images(&self, dataset_path: &Path) -> Result<ImageData> {
        // EuRoC-specific image loading
    }
    
    fn get_dataset_name(&self) -> &'static str {
        "EurocPlayer"
    }
}

pub type EurocPlayer = GenericDatasetPlayer<EurocLoader>;

// Optional convenience constructor
impl EurocPlayer {
    pub fn new() -> Self {
        GenericDatasetPlayer::new(EurocLoader)
    }
}
```

### Benefits

- ✅ Eliminates 1,157 lines of duplication
- ✅ Single source of truth for frame processing logic
- ✅ New datasets require only 50-100 lines (just implement `DatasetLoader`)
- ✅ Fixes like bug fixes automatically apply to all datasets
- ✅ Easier to test core logic independently

### Estimated Effort

- 3-4 hours
- Risk: Low (datasets still have independent loader implementations)
- Testing: Rerun all 3 dataset tests to verify behavior unchanged

---

## 🟡 HIGH: Custom Conversion Traits (8 repetitions)

### Problem

**File:** `src/types.rs` (lines 108-150+)

```rust
// ❌ Repeated 8 times with nearly identical structure:
pub trait ToMatrix {
    type Output;
    fn to_matrix(&self) -> Self::Output;
}

pub trait ToArray {
    type Output;
    fn to_array(&self) -> Self::Output;
}

pub trait ToVector {
    type Output;
    fn to_vector(&self) -> Self::Output;
}

pub trait ToArrayVec {
    type Output;
    fn to_array_vec(&self) -> Self::Output;
}

// Implementations repeat across Array4x4, Array3x3, Matrix4x4, Matrix3x3, Vector3, Vector2
impl ToMatrix for Array4x4 {
    type Output = Matrix4x4;
    fn to_matrix(&self) -> Self::Output {
        na::Matrix4::from_row_slice(&[
            self[0][0], self[0][1], self[0][2], self[0][3],
            self[1][0], self[1][1], self[1][2], self[1][3],
            self[2][0], self[2][1], self[2][2], self[2][3],
            self[3][0], self[3][1], self[3][2], self[3][3],
        ])
    }
}

impl ToMatrix for Array3x3 {
    type Output = Matrix3x3;
    fn to_matrix(&self) -> Self::Output {
        na::Matrix3::from_row_slice(&[
            self[0][0], self[0][1], self[0][2],
            self[1][0], self[1][1], self[1][2],
            self[2][0], self[2][1], self[2][2],
        ])
    }
}
```

### Impact

- **~150 lines** of custom trait boilerplate
- Custom traits don't work with standard library (Can't implement `From<T> for T` for external types due to orphan rules)
- Conversion logic repeated across 8 type pairs
- No generic conversion support (can't use `.collect()`, generic functions expecting `From`)

### Solution: Consolidate to Standard Patterns + Macros

**Option A: Use `From`/`Into` where possible** (respects orphan rules):

```rust
// src/types.rs (REFACTORED)

// For array->matrix (array-owned, matrix-owned):
impl From<Array4x4> for Matrix4x4 {
    fn from(arr: Array4x4) -> Self {
        na::Matrix4::from_row_slice(&[
            arr[0][0], arr[0][1], arr[0][2], arr[0][3],
            arr[1][0], arr[1][1], arr[1][2], arr[1][3],
            arr[2][0], arr[2][1], arr[2][2], arr[2][3],
            arr[3][0], arr[3][1], arr[3][2], arr[3][3],
        ])
    }
}

// For matrix->array
impl From<Matrix4x4> for Array4x4 {
    fn from(mat: Matrix4x4) -> Self {
        [
            [mat[(0,0)], mat[(0,1)], mat[(0,2)], mat[(0,3)]],
            [mat[(1,0)], mat[(1,1)], mat[(1,2)], mat[(1,3)]],
            [mat[(2,0)], mat[(2,1)], mat[(2,2)], mat[(2,3)]],
            [mat[(3,0)], mat[(3,1)], mat[(3,2)], mat[(3,3)]],
        ]
    }
}

// For references (using custom trait):
pub trait ToMatrix {
    type Output;
    fn to_matrix(&self) -> Self::Output;
}

impl ToMatrix for &Matrix4x4 {
    type Output = Array4x4;
    fn to_matrix(&self) -> Self::Output {
        (*self).clone().into()
    }
}
```

**Option B: Macro to reduce repetition**:

```rust
// src/types.rs (REFACTORED)

macro_rules! impl_from_array_to_matrix {
    ($array_type:ty, $matrix_type:ty, $size:expr, $init:expr) => {
        impl From<$array_type> for $matrix_type {
            fn from(arr: $array_type) -> Self {
                let values: Vec<Float> = arr.iter()
                    .flat_map(|row| row.iter())
                    .copied()
                    .collect();
                <$matrix_type>::from_row_slice(&values)
            }
        }
    };
}

impl_from_array_to_matrix!(Array4x4, Matrix4x4, 16, {});
impl_from_array_to_matrix!(Array3x3, Matrix3x3, 9, {});
```

### Benefits

- ✅ Reduces custom trait boilerplate by ~100 lines
- ✅ Standard library integration (works with `.into()`, `.collect()`)
- ✅ Macros eliminate repetitive `From` implementations
- ✅ Generic functions can now accept `T: Into<Matrix4x4>`
- ✅ Aligns with Rust idioms

### Estimated Effort

- 1-2 hours
- Risk: Low (straightforward mechanical refactoring)
- Testing: Run type conversion tests to verify behavior

---

## 🟡 MEDIUM: Repeated Result Error Handling

### Problem

**Files:** All dataset players, viewers, and estimator

```rust
// ❌ Pattern repeated 20+ times:
let viewer: Option<Box<dyn Viewer>> = match create_viewer() {
    Ok(v) => {
        log::info!("Viewer initialized successfully");
        Some(v)
    },
    Err(e) => {
        log::warn!("Failed to initialize viewer: {}", e);
        None
    },
};

let cfg = match Config::load(&config.config_path) {
    Ok(c) => c,
    Err(e) => {
        result.error_message = format!("Failed to load: {}", e);
        return result;
    },
};

let (left_cam, right_cam) = match Self::create_camera_models_from_config(&cfg) {
    Ok(cams) => cams,
    Err(e) => {
        result.error_message = format!("Failed to create models: {}", e);
        return result;
    },
};
```

### Solution: Error Context Struct + Helper Methods

```rust
// src/datasets/error_context.rs (NEW)

pub struct ErrorContext {
    pub message: String,
    pub should_return: bool,
}

impl ErrorContext {
    pub fn new() -> Self {
        Self {
            message: String::new(),
            should_return: false,
        }
    }
    
    /// Log error and optionally set result message
    pub fn log_error(&mut self, err: impl std::fmt::Display, critical: bool) {
        self.message = err.to_string();
        self.should_return = critical;
        if critical {
            log::error!("{}", err);
        } else {
            log::warn!("{}", err);
        }
    }
}

// Helper function
pub fn handle_result<T, E: std::fmt::Display>(
    result: Result<T, E>,
    context: &mut ErrorContext,
    critical: bool,
) -> Option<T> {
    match result {
        Ok(v) => Some(v),
        Err(e) => {
            context.log_error(e, critical);
            None
        }
    }
}
```

**Usage becomes cleaner:**

```rust
// Before:
let cfg = match Config::load(&config.config_path) {
    Ok(c) => c,
    Err(e) => {
        result.error_message = format!("Failed to load: {}", e);
        return result;
    },
};

// After (still verbose, but Pattern above helps):
let mut err_ctx = ErrorContext::new();
let cfg = Config::load(&config.config_path)
    .map_err(|e| err_ctx.log_error(e, true))?;
```

### Better: Use Result<T> Return Type

Once dataset players return `Result<PlayerResult>`:

```rust
pub fn run(&self, config: PlayerConfig) -> Result<PlayerResult> {
    let image_data = Self::load_image_timestamps(&config.dataset_path)
        .context("Failed to load image timestamps")?;
    
    let viewer = create_viewer().ok();  // Non-critical
    
    let cfg = Config::load(&config.config_path)
        .context("Failed to load config")?;
    
    let (left_cam, right_cam) = Self::create_camera_models_from_config(&cfg)
        .context("Failed to create camera models")?;
    
    Ok(PlayerResult {
        success: true,
        processed_frames: 100,
        ..Default::default()
    })
}
```

### Benefits

- ✅ Eliminates 50+ lines of boilerplate error handling
- ✅ `?` operator works everywhere (DRY error propagation)
- ✅ Clear error messages from `anyhow::Context`
- ✅ Matches Rust conventions

### Estimated Effort

- 2-3 hours
- Risk: Medium (changes return types of public functions)
- Testing: Full integration test suite

---

## 🟡 MEDIUM: Repeated Camera Initialization

### Problem

**Files:** All 3 dataset players + `src/estimator/frame.rs`

```rust
// ❌ Repeated 5 times with slight variations:
left_cam: CameraModelType::OpenCV5(OpenCVModel5::new(
    &nalgebra034::DVector::from_vec(vec![
        500.0, 500.0, 320.0, 240.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    ]),
    0,
    0,
)),

// From another location:
left_cam: CameraModelType::OpenCV5(OpenCVModel5::new(
    &nalgebra034::DVector::from_vec(vec![
        fx, fy, cx, cy, k1, k2, k3, p1, p2,
    ]),
    image_width,
    image_height,
)),
```

### Solution: Camera Factory Function

```rust
// src/types.rs or src/datasets/config.rs

/// Factory for creating camera models from configuration
pub struct CameraFactory;

impl CameraFactory {
    /// Create OpenCV5 camera model from intrinsic parameters
    pub fn opencv5(
        fx: Float,
        fy: Float,
        cx: Float,
        cy: Float,
        k1: Float,
        k2: Float,
        k3: Float,
        p1: Float,
        p2: Float,
        width: u32,
        height: u32,
    ) -> CameraModelType {
        let params = nalgebra034::DVector::from_vec(vec![fx, fy, cx, cy, k1, k2, k3, p1, p2]);
        CameraModelType::OpenCV5(OpenCVModel5::new(&params, width, height))
    }
    
    /// Create camera from config struct
    pub fn from_config(cfg: &CameraConfig, width: u32, height: u32) -> Result<CameraModelType> {
        Self::opencv5(
            cfg.fx, cfg.fy, cfg.cx, cfg.cy,
            cfg.k1, cfg.k2, cfg.k3, cfg.p1, cfg.p2,
            width, height,
        ).map_err(|e| anyhow::anyhow!("Failed to create camera: {}", e))
    }
}

// Usage:
let left_cam = CameraFactory::from_config(&cfg.camera, 752, 480)?;
let right_cam = CameraFactory::from_config(&cfg.camera, 752, 480)?;
```

### Benefits

- ✅ Single source of truth for camera creation
- ✅ Eliminates 5 repetitions of vector construction
- ✅ Easy to extend (add more camera models)
- ✅ Type-safe camera parameter handling

### Estimated Effort

- 1 hour
- Risk: Very Low
- Testing: Unit tests on camera factory

---

## 🟢 LOW: Logging Boilerplate

### Problem

```rust
// ❌ Repeated logging pattern:
match create_viewer() {
    Ok(v) => {
        log::info!("[EurocPlayer] Viewer initialized successfully");
        Some(v)
    },
    Err(e) => {
        log::warn!("Failed to initialize viewer: {}", e);
        None
    },
};
```

### Solution: Macro

```rust
// src/lib.rs

#[macro_export]
macro_rules! log_ok_or_warn {
    ($result:expr, $ok_msg:expr) => {{
        match $result {
            Ok(v) => {
                log::info!($ok_msg);
                Some(v)
            }
            Err(e) => {
                log::warn!("Error: {}", e);
                None
            }
        }
    }};
    
    ($result:expr, $ok_msg:expr, $err_context:expr) => {{
        match $result {
            Ok(v) => {
                log::info!($ok_msg);
                Some(v)
            }
            Err(e) => {
                log::warn!("{}: {}", $err_context, e);
                None
            }
        }
    }};
}
```

**Usage:**

```rust
let viewer = log_ok_or_warn!(create_viewer(), "Viewer initialized successfully");
```

### Benefits

- ✅ Eliminates boilerplate
- ✅ Consistent logging across codebase
- ✅ Single line instead of 10 lines

---

## 🟢 OPTIONAL: Logging Initialization Duplication

### Problem

**Files:** `src/bin/run_euroc.rs`, `src/bin/run_tum.rs`, `src/bin/run_4seasons.rs`

Each binary repeats the same logging setup (~20 lines each):

```rust
// ❌ Identical in all 3 binaries:
Builder::from_env(Env::default().default_filter_or("debug"))
    .filter_module("rerun", LevelFilter::Warn)
    .format_timestamp_millis()
    .format(|buf, record| {
        use std::io::Write;
        let level = match record.level() {
            log::Level::Error => "\x1b[31mERROR\x1b[0m",
            log::Level::Warn => "\x1b[33mWARN\x1b[0m",
            log::Level::Info => "\x1b[32mINFO\x1b[0m",
            log::Level::Debug => "\x1b[34mDEBUG\x1b[0m",
            log::Level::Trace => "\x1b[36mTRACE\x1b[0m",
        };
        writeln!(buf, "{} [{}] {}", level, record.target(), record.args())
    })
    .try_init()
    .ok();
```

### Solution: Centralize in `lib.rs`

```rust
// src/lib.rs

pub fn init_colored_logging() {
    Builder::from_env(Env::default().default_filter_or("debug"))
        .filter_module("rerun", LevelFilter::Warn)
        .format_timestamp_millis()
        .format(|buf, record| {
            use std::io::Write;
            let level = match record.level() {
                log::Level::Error => "\x1b[31mERROR\x1b[0m",
                log::Level::Warn => "\x1b[33mWARN\x1b[0m",
                log::Level::Info => "\x1b[32mINFO\x1b[0m",
                log::Level::Debug => "\x1b[34mDEBUG\x1b[0m",
                log::Level::Trace => "\x1b[36mTRACE\x1b[0m",
            };
            writeln!(buf, "{} [{}] {}", level, record.target(), record.args())
        })
        .try_init()
        .ok();
}
```

**In binaries:**

```rust
// Before: 20 lines
// After: 1 line
rs_vio::init_colored_logging();
```

### Benefits

- ✅ Eliminates 60 lines of duplication across 3 binaries
- ✅ Centralized logging configuration
- ✅ Single change applies to all binaries

---

## 📊 Summary of DRY Issues & Effort

| Issue | Type | LOC Duplication | Effort | Risk | Priority |
|-------|------|-----------------|--------|------|----------|
| **Dataset Players** | Structural | 1,157 | 3-4h | Low | 🔴 CRITICAL |
| **Custom Traits** | Type System | 150 | 1-2h | Low | 🟡 HIGH |
| **Error Handling** | Pattern | 50+ | 2-3h | Medium | 🟡 HIGH |
| **Camera Init** | Boilerplate | 20 | 1h | Very Low | 🟡 MEDIUM |
| **Logging Boilerplate** | Boilerplate | 30+ | 1h | Very Low | 🟢 LOW |
| **Logger Init** | Boilerplate | 60 | 30m | Very Low | 🟢 OPTIONAL |
| **TOTAL** | | **~1,470 lines** | **9-11 hours** | | |

---

## 🎯 Recommended Implementation Order

### Phase 1: Foundation (2-3 hours) - **Do First**
1. ✅ Camera Factory (1h) - Unblocks other refactors
2. ✅ Logging Helper (30m) - Quick win
3. ✅ Logger Init Centralization (30m) - Quick win

### Phase 2: Major Refactoring (4-5 hours) - **Do After Phase 1**
4. ✅ Error Handling Consolidation (2-3h) - Foundational for dataset players
5. ✅ Custom Traits Consolidation (1-2h) - Type system cleanup

### Phase 3: Strategic Architecture (3-4 hours) - **Do Last**
6. ✅ Dataset Player Generics (3-4h) - Biggest payoff but requires all phases

---

## ✅ Verification Checklist

After each refactoring:

```bash
# Full test suite
cargo test --all --verbose

# Check for warnings
cargo clippy -- -D warnings

# Verify behavior unchanged (ensure output matches)
./scripts/run_euroc.sh
./scripts/run_tum-vi.sh
./scripts/run_4seasons.sh

# Measure code reduction
wc -l src/datasets/*.rs src/types.rs src/lib.rs src/bin/*.rs
```

---

## 💡 Key Insights

1. **Three dataset players are 90% identical** - They're prime candidates for the Strategy pattern
2. **Custom traits are a Rust language limitation** - Orphan rules prevent `From<ExternalType>` but allow owned-type conversions
3. **Error handling is the glue** - Unifying return types makes everything else cleaner
4. **Macros reduce boilerplate** - Use sparingly for true repetitive patterns
5. **Configuration-driven behavior** - Already good in this codebase, camera factory extends it

