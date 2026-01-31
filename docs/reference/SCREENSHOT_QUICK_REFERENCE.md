# Screenshot Testing Quick Reference

## Two Approaches Available

### Approach 1: Per-View Screenshots (Renderer API)

**When to use**: Capturing individual views from Rerun viewer with precise control

**Key Types**:
```rust
use rs_vio::viewers::{
    ViewScreenshotRequest,
    ScreenshotBatch,
    ScreenshotId,
    ScreenshotMetadata,
    ScreenshotResult,
};
```

**Quick Pattern**:
```rust
// 1. Create batch request
let mut batch = ScreenshotBatch::new();
batch.add(ViewScreenshotRequest::new("View Name", 1024, 768));

// 2. Get screenshot ID for tracking
let req = &batch.requests()[0];
let id = req.id();

// 3. With Rerun viewer: schedule capture
viewer_builder.schedule_screenshot(ctx, id);

// 4. Read result when ready
let result = processor.next_readback_result();

// 5. Save PNG
result.save_to_file("artifacts/screenshot.png")?;
```

**API Reference**:
| Type | Method | Returns |
|------|--------|---------|
| `ViewScreenshotRequest` | `new(name, w, h)` | Self |
| | `id()` | `ScreenshotId` |
| | `view_name()` | `&str` |
| | `dimensions()` | `(u32, u32)` |
| | `build_metadata()` | `ScreenshotMetadata` |
| `ScreenshotBatch` | `new()` | Self |
| | `add(request)` | () |
| | `len()` | `usize` |
| | `into_requests()` | `Vec<ViewScreenshotRequest>` |
| `ScreenshotResult` | `to_png()` | `Result<Vec<u8>>` |
| | `save_to_file(path)` | `Result<()>` |

---

### Approach 2: CLI Screenshots (Simpler)

**When to use**: Quick screenshots without API integration, entire scene capture

**Quick Pattern**:
```rust
use std::process::Command;
use rerun::RecordingStreamBuilder;

// 1. Record your data to RRD file
let rec = RecordingStreamBuilder::new("test")
    .save("/tmp/test.rrd")?;
rec.log("world/data", &data)?;

// 2. Use Rerun CLI to convert to PNG
Command::new("rerun")
    .arg("/tmp/test.rrd")
    .arg("--screenshot-to")
    .arg("/tmp/screenshot.png")
    .status()?;

// 3. Store in artifacts
artifacts.save_screenshot("final", fs::read("/tmp/screenshot.png")?)?;
```

**Note**: Rerun CLI may panic after saving (still saves PNG)

---

## Test Artifacts Manager

**When to use**: Organizing and storing screenshots from tests

**Quick Pattern**:
```rust
use test_screenshot_artifacts::ScreenshotArtifacts;

#[test]
fn test_with_screenshots() -> Result<(), Box<dyn std::error::Error>> {
    // 1. Create artifacts directory
    let mut artifacts = ScreenshotArtifacts::new_test("test_name")?;

    // 2a. Option: Save immediately
    artifacts.save_screenshot("frame_1", png_data)?;

    // 2b. Option: Queue for batch
    artifacts.queue_screenshot("frame_2", png_data);
    artifacts.queue_screenshot("frame_3", png_data);

    // 3. Finalize (writes manifest.txt)
    artifacts.finalize()?;

    // 4. Cleanup (optional)
    artifacts.cleanup()?;
    Ok(())
}
```

**Output Structure**:
```
target/test-artifacts/test_name/
├── frame_1.png
├── frame_2.png
├── frame_3.png
└── manifest.txt
```

**API Reference**:
| Method | Returns | Purpose |
|--------|---------|---------|
| `new_test(name)` | `Result<Self>` | Initialize for test |
| `save_screenshot(name, data)` | `Result<PathBuf>` | Save immediately |
| `queue_screenshot(name, data)` | () | Queue for batch |
| `finalize()` | `Result<Vec<PathBuf>>` | Write all + manifest |
| `cleanup()` | `Result<()>` | Remove test dir |
| `artifact_paths()` | `Result<Vec<PathBuf>>` | List artifacts |
| `test_dir()` | `&Path` | Get test directory |

---

## Combined Usage Example

```rust
#[test]
fn test_vio_pipeline_visual() -> Result<(), Box<dyn std::error::Error>> {
    use rs_vio::viewers::*;

    // Setup artifacts
    let mut artifacts = ScreenshotArtifacts::new_test("vio_pipeline")?;

    // Initialize viewer with Rerun
    let mut viewer = create_rerun_viewer()?;
    let mut batch = ScreenshotBatch::new();

    // Schedule multiple views
    batch.add(ViewScreenshotRequest::new("3D Scene", 1920, 1080));
    batch.add(ViewScreenshotRequest::new("Trajectory", 1200, 400));
    batch.add(ViewScreenshotRequest::new("Features", 800, 600));

    // Run pipeline and collect screenshots
    for req in batch.into_requests() {
        // Log data to Rerun
        viewer.log_frame_data(&req.view_name())?;

        // Schedule screenshot
        viewer.schedule_screenshot(req.id())?;

        // Get result and save
        let result = viewer.read_screenshot_blocking(req.id())?;
        artifacts.save_screenshot(req.view_name(), result.to_png()?)?;
    }

    // Save manifest with all artifacts
    artifacts.finalize()?;

    Ok(())
}
```

---

## Test Results Status

**Unit Tests**: ✅ 4/4 PASSING
- ID generation
- Request creation
- Batch operations
- Metadata capture

**Integration Tests**: ✅ 4/4 PASSING
- Directory creation
- Screenshot save
- Queue + finalize
- Artifact listing

**CLI Test**: ✅ 1/1 PASSING
- Rerun screenshot-to flag

---

## Common Patterns

### Pattern 1: Checkpoint Screenshots
```rust
let mut artifacts = ScreenshotArtifacts::new_test("checkpoints")?;

artifacts.save_screenshot("checkpoint_init", initial_png)?;
artifacts.save_screenshot("checkpoint_mid", mid_png)?;
artifacts.save_screenshot("checkpoint_final", final_png)?;

artifacts.finalize()?;
```

### Pattern 2: Batch Queue
```rust
let mut batch = ScreenshotBatch::new();
for view in get_all_views() {
    batch.add(ViewScreenshotRequest::new(view.name, view.w, view.h));
}

let mut artifacts = ScreenshotArtifacts::new_test("batch_capture")?;
for (png_data, req) in collect_screenshots(batch) {
    artifacts.queue_screenshot(req.view_name(), png_data);
}
artifacts.finalize()?;
```

### Pattern 3: Conditional Cleanup
```rust
let artifacts = ScreenshotArtifacts::new_test("test")?;

// Do work...

if test_passed {
    artifacts.cleanup()?;  // Remove on success
} else {
    println!("Debug artifacts at: {}", artifacts.test_dir().display());
    // Keep for debugging
}
```

---

## Files & Locations

| Component | File |
|-----------|------|
| Per-view helper | `src/viewers/screenshot_helper.rs` |
| Artifacts manager | `tests/test_screenshot_artifacts.rs` |
| CLI example | `tests/rerun_screenshot_test.rs` |
| Module exports | `src/viewers/mod.rs` |
| Examples | `examples/screenshot_integration.rs` |
| Full docs | `SCREENSHOT_INTEGRATION_SUMMARY.md` |

---

## Troubleshooting

### Q: Screenshot is black/empty
**A**: Rerun needs time to render. Add delay before capturing or use viewport settings.

### Q: Rerun CLI not found
**A**: Install: `cargo install rerun-cli` or use renderer API instead

### Q: PNG encoding fails
**A**: Ensure image crate is in dependencies (already in Cargo.toml)

### Q: Artifacts not found after test
**A**: Call `finalize()` before accessing files. Cleanup happens in test cleanup.

### Q: Thread panic after screenshot save
**A**: Expected behavior. Rerun panics after save but PNG is already written. Test handles this.
