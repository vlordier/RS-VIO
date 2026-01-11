# DRY Implementation Roadmap

## Quick Reference: Where the Duplication Lives

### 1. **Dataset Players (CRITICAL - 1,157 lines)**
```
src/datasets/euroc_player.rs    → 385 lines
src/datasets/tum_vi_player.rs   → 385 lines  
src/datasets/fourseasons_player.rs → 387 lines
────────────────────────────────────────────
OVERLAP: ~90% (same orchestration logic)
```

**Key duplicate sections:**
- Lines 25-90: Load images, init viewer, load config, create cameras
- Lines 93-150: Frame processing loop setup
- Lines 150+: Frame-by-frame processing orchestration
- Helper methods: `initialize_estimator()`, `process_single_frame()`

**Dataset-specific:** Only image loading and IMU loading differ (50 lines each)

---

### 2. **Custom Traits (HIGH - 150 lines)**
```
src/types.rs
├─ pub trait ToMatrix { ... }        ← 4 implementations
├─ pub trait ToArray { ... }         ← 4 implementations  
├─ pub trait ToVector { ... }        ← 2 implementations
└─ pub trait ToArrayVec { ... }      ← 2 implementations
────────────────────────────────────────────
TOTAL: ~150 lines of boilerplate
```

**Root cause:** Implementing conversion traits for all combinations of arrays, matrices, vectors

---

### 3. **Error Handling Patterns (MEDIUM - 50+ lines)**

**Repeated pattern (20+ times):**
```rust
match operation() {
    Ok(value) => { log_success(); ... },
    Err(e) => { 
        result.error_message = format!(...);
        return result;
    }
}
```

**Files affected:**
- `src/datasets/euroc_player.rs` (8 occurrences)
- `src/datasets/tum_vi_player.rs` (8 occurrences)
- `src/datasets/fourseasons_player.rs` (8 occurrences)
- `src/viewers/rerun.rs` (3 occurrences)
- `src/estimator/estimator.rs` (2 occurrences)

---

### 4. **Camera Initialization (MEDIUM - 20 lines)**

**Pattern (5 times):**
```rust
CameraModelType::OpenCV5(OpenCVModel5::new(
    &nalgebra034::DVector::from_vec(vec![
        fx, fy, cx, cy, k1, k2, k3, p1, p2
    ]),
    width,
    height,
))
```

---

### 5. **Logging Setup (OPTIONAL - 60 lines)**

**Identical 20-line setup in:**
- `src/bin/run_euroc.rs`
- `src/bin/run_tum.rs`
- `src/bin/run_4seasons.rs`

---

## Implementation Strategy by Priority

### ✅ DO FIRST: Simple, Non-Blocking Changes

#### 1. Logger Initialization Centralization (30 minutes)

**Goal:** Move 20-line logger setup from 3 binaries to 1 library function

**Steps:**

1. In `src/lib.rs`, add:
```rust
/// Initialize colored logging with RS-VIO defaults
pub fn init_colored_logging() {
    use env_logger::Builder;
    use log::LevelFilter;
    use env_logger::Env;
    use std::io::Write;
    
    Builder::from_env(Env::default().default_filter_or("debug"))
        .filter_module("rerun", LevelFilter::Warn)
        .format_timestamp_millis()
        .format(|buf, record| {
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

2. Replace in `src/bin/run_euroc.rs`, `src/bin/run_tum.rs`, `src/bin/run_4seasons.rs`:
```rust
// Old (20 lines)
Builder::from_env(...)...
    .try_init()
    .ok();

// New (1 line)
rs_vio::init_colored_logging();
```

**Test:**
```bash
cargo run --bin run_euroc 2>&1 | head -5
# Should show colored output
```

---

#### 2. Camera Factory Pattern (1 hour)

**Goal:** Consolidate camera creation logic

**Steps:**

1. Add to `src/types.rs` or create `src/utils/camera_factory.rs`:

```rust
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
        let params = nalgebra034::DVector::from_vec(vec![
            fx, fy, cx, cy, k1, k2, k3, p1, p2
        ]);
        CameraModelType::OpenCV5(OpenCVModel5::new(&params, width, height))
    }
    
    /// Create camera from config intrinsics
    pub fn from_camera_config(
        intrinsics: &CameraIntrinsics,
        width: u32,
        height: u32,
    ) -> CameraModelType {
        Self::opencv5(
            intrinsics.fx,
            intrinsics.fy,
            intrinsics.cx,
            intrinsics.cy,
            intrinsics.k1,
            intrinsics.k2,
            intrinsics.k3,
            intrinsics.p1,
            intrinsics.p2,
            width,
            height,
        )
    }
}
```

2. In dataset players, replace:
```rust
// Old (repeated 5 times)
let left_cam = CameraModelType::OpenCV5(OpenCVModel5::new(
    &nalgebra034::DVector::from_vec(vec![
        cfg.camera.fx, cfg.camera.fy, cfg.camera.cx, cfg.camera.cy,
        cfg.camera.k1, cfg.camera.k2, cfg.camera.k3, cfg.camera.p1, cfg.camera.p2,
    ]),
    width,
    height,
));

// New
let left_cam = CameraFactory::from_camera_config(&cfg.camera, width, height);
```

**Test:**
```bash
cargo test --lib types::tests
cargo test --lib datasets::tests
```

---

### 🔧 DO SECOND: Foundational Architecture Changes

#### 3. Unify Error Return Types (2-3 hours)

**Current state:** Dataset players use `PlayerResult` with `error_message` field

**Goal:** Use standard Rust `Result<T, E>`

**Steps:**

1. Create error type in `src/datasets/mod.rs`:
```rust
use thiserror::Error;

#[derive(Error, Debug)]
pub enum DatasetError {
    #[error("Failed to load images: {0}")]
    ImageLoading(String),
    
    #[error("Failed to load IMU data: {0}")]
    ImuLoading(String),
    
    #[error("Configuration error: {0}")]
    Configuration(String),
    
    #[error("Camera setup failed: {0}")]
    CameraSetup(String),
    
    #[error("Processing error: {0}")]
    Processing(String),
    
    #[error("IO error: {0}")]
    Io(#[from] std::io::Error),
}

pub type DatasetResult<T> = Result<T, DatasetError>;
```

2. Update player signatures:
```rust
// Old
pub fn run(&self, config: PlayerConfig) -> PlayerResult

// New
pub fn run(&self, config: PlayerConfig) -> DatasetResult<PlayerResult>
```

3. Simplify error handling with `?` operator:
```rust
// Old (8 lines)
let image_data = match Self::load_image_timestamps(&config.dataset_path) {
    Ok(data) => {
        if data.is_empty() {
            result.error_message = "No images found".to_string();
            return result;
        }
        data
    }
    Err(e) => {
        result.error_message = format!("Failed to load: {}", e);
        return result;
    }
};

// New (2 lines, with `.context()` from anyhow)
let image_data = Self::load_image_timestamps(&config.dataset_path)
    .context("Failed to load image timestamps")?;
```

**Test:**
```bash
cargo test --lib datasets::euroc_player::tests
cargo test --test comprehensive_integration_tests
```

---

### 🏗️ DO LAST: Major Architectural Refactoring

#### 4. Dataset Player Strategy Pattern (3-4 hours)

**Goal:** Extract common logic into generic base, keep dataset-specific loading separate

**Steps:**

1. Create `src/datasets/player_base.rs`:

```rust
use crate::datasets::{Config, ImageData, ImuData, PlayerConfig, PlayerResult, FrameContext};
use crate::estimator::Estimator;
use crate::viewers::Viewer;
use std::path::Path;
use std::time::{Duration, Instant};
use std::thread;

/// Trait for dataset-specific loading operations
pub trait DatasetLoader: std::fmt::Debug {
    /// Load images from dataset
    fn load_images(&self, dataset_path: &Path) -> Result<ImageData, String>;
    
    /// Load IMU data (optional)
    fn load_imu_data(&self, dataset_path: &Path) -> Result<Vec<ImuData>, String> {
        Ok(Vec::new())
    }
    
    /// Get dataset name for logging
    fn get_dataset_name(&self) -> &'static str;
}

/// Generic dataset player that works with any DatasetLoader
pub struct BaseDatasetPlayer<T: DatasetLoader> {
    loader: T,
}

impl<T: DatasetLoader> BaseDatasetPlayer<T> {
    pub fn new(loader: T) -> Self {
        Self { loader }
    }

    /// Run dataset through VIO pipeline
    pub fn run(&self, config: PlayerConfig) -> Result<PlayerResult, String> {
        let mut result = PlayerResult::default();

        // Load images using dataset-specific loader
        let image_data = match self.loader.load_images(&config.dataset_path) {
            Ok(data) => {
                if data.is_empty() {
                    return Err("No images found in dataset".to_string());
                }
                data
            }
            Err(e) => return Err(format!("Failed to load images: {}", e)),
        };

        let start_frame_idx = 0;
        let end_frame_idx = image_data.len();

        // Initialize viewer (common logic)
        let viewer = crate::viewers::create_viewer().ok();
        if let Some(ref v) = viewer {
            log::info!("[{}] Viewer initialized successfully", self.loader.get_dataset_name());
        } else {
            log::warn!("Failed to initialize viewer");
        }

        // Load config
        let cfg = Config::load(&config.config_path)
            .map_err(|e| format!("Failed to load config: {}", e))?;

        // Create cameras
        let (left_cam, right_cam) = self.create_cameras_from_config(&cfg)?;

        // Initialize estimator
        let mut estimator = Estimator::new_with_cameras(cfg, viewer, Some(left_cam), Some(right_cam));
        self.initialize_estimator(&mut estimator, &image_data);

        // Frame processing loop (common for all datasets)
        let mut context = FrameContext::new(config.step_mode);
        context.current_idx = start_frame_idx;

        while context.current_idx < end_frame_idx {
            let should_process = if context.auto_play {
                true
            } else if context.advance_frame {
                context.advance_frame = false;
                true
            } else {
                thread::sleep(Duration::from_millis(30));
                continue;
            };

            if should_process {
                let frame_start = Instant::now();
                
                // Process frame (common logic)
                let img_left = &image_data[context.current_idx].left;
                let img_right = &image_data[context.current_idx].right;
                
                estimator.process_frame(img_left, img_right, image_data[context.current_idx].timestamp_ns);
                
                result.processed_frames += 1;
                let elapsed = frame_start.elapsed();
                log::debug!("Frame {} processed in {:?}", context.current_idx, elapsed);

                context.current_idx += 1;
            }
        }

        result.success = true;
        Ok(result)
    }
    
    fn create_cameras_from_config(&self, cfg: &Config) -> Result<(CameraModelType, CameraModelType), String> {
        let width = cfg.camera.image_width.unwrap_or(752);
        let height = cfg.camera.image_height.unwrap_or(480);
        
        let left = crate::types::CameraFactory::from_camera_config(&cfg.camera, width, height);
        let right = crate::types::CameraFactory::from_camera_config(&cfg.camera, width, height);
        
        Ok((left, right))
    }
    
    fn initialize_estimator(&self, estimator: &mut Estimator, image_data: &[ImageData]) {
        // Common initialization logic
        if !image_data.is_empty() {
            let first = &image_data[0];
            estimator.initialize_with_first_frame(&first.left, &first.right);
        }
    }
}
```

2. Refactor `EurocPlayer`:
```rust
// src/datasets/euroc_player.rs (SIMPLIFIED)

#[derive(Debug)]
pub struct EurocLoader;

impl DatasetLoader for EurocLoader {
    fn load_images(&self, dataset_path: &Path) -> Result<Vec<ImageData>, String> {
        // EuRoC-specific image loading (from old load_image_timestamps)
        // ~50 lines specific to EuRoC file layout
    }
    
    fn load_imu_data(&self, dataset_path: &Path) -> Result<Vec<ImuData>, String> {
        // EuRoC-specific IMU loading
        // ~30 lines specific to EuRoC file format
    }
    
    fn get_dataset_name(&self) -> &'static str {
        "EurocPlayer"
    }
}

/// Type alias for convenience
pub type EurocPlayer = BaseDatasetPlayer<EurocLoader>;

impl EurocPlayer {
    pub fn new() -> Self {
        BaseDatasetPlayer::new(EurocLoader)
    }
}
```

3. Repeat for `TUMVIPlayer` and `FourSeasonsPlayer` (each ~50 lines)

**Test:**
```bash
cargo test --lib datasets::euroc_player::tests
cargo test --test comprehensive_integration_tests euroc
cargo test --test comprehensive_integration_tests tum
cargo test --test comprehensive_integration_tests 4seasons
```

---

### 🎯 Bonus: Replace Custom Traits with `From`/`Into` (1-2 hours)

**Only do if confident with Rust trait system**

Replace in `src/types.rs`:

```rust
// Old (8 custom traits)
pub trait ToMatrix { type Output; fn to_matrix(&self) -> Self::Output; }
pub trait ToArray { type Output; fn to_array(&self) -> Self::Output; }

// New (standard From/Into)
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

// Usage becomes: let matrix: Matrix4x4 = array.into();
```

**Benefit:** Works with `.collect()`, generic functions expecting `Into<T>`

---

## Effort Summary

| Task | Effort | Risk | Payoff |
|------|--------|------|--------|
| Logger centralization | 30m | Very Low | 60 LOC removed |
| Camera factory | 1h | Very Low | 20 LOC removed |
| Error return types | 2-3h | Medium | 50 LOC simplified, better errors |
| Dataset player generics | 3-4h | Low | 1,100 LOC eliminated, extensible |
| Custom traits → From/Into | 1-2h | Low | 100 LOC removed |
| **TOTAL** | **8-11 hours** | | **~1,400 LOC improved** |

---

## Verification After Each Step

```bash
# Always run after changes
cargo test --all
cargo clippy -- -D warnings
cargo fmt --check
```

