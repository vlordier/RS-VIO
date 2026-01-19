//! Example: Capturing screenshots in integration tests
//!
//! This example demonstrates how to use the screenshot artifacts utilities
//! to capture and store test screenshots for debugging and validation.

#[cfg(test)]
mod integration_example {
    use std::path::PathBuf;

    /// Mock integration test that captures multiple screenshots
    /// 
    /// In a real scenario, this would run your VIO pipeline and capture
    /// screenshots at key stages for visual validation.
    #[test]
    fn example_vio_pipeline_with_screenshots() {
        // Note: This is a documentation example showing the pattern
        // In actual use, import from tests/test_screenshot_artifacts.rs
        
        // Pattern:
        // 1. Create artifacts manager for your test
        // let mut artifacts = ScreenshotArtifacts::new_test("vio_integration")?;
        //
        // 2. Run your VIO pipeline or viewer setup
        // viewer.initialize()?;
        // 
        // 3. Capture screenshots at key checkpoints
        // - Initial state (empty scene)
        // - After first frame
        // - After feature tracking
        // - Final optimized state
        //
        // 4. For Rerun-based captures, use screenshot helper:
        // let mut batch = ScreenshotBatch::new();
        // batch.add(ViewScreenshotRequest::new("3D View", 1024, 768));
        // batch.add(ViewScreenshotRequest::new("Feature Matches", 800, 600));
        // batch.add(ViewScreenshotRequest::new("Trajectory", 1200, 400));
        //
        // 5. Schedule and collect results from Rerun renderer
        // for req in batch.into_requests() {
        //     viewer.schedule_screenshot(req)?;
        //     let result = viewer.read_screenshot(req.id())?;
        //     artifacts.save_screenshot(req.view_name(), result.to_png()?)?;
        // }
        //
        // 6. Finalize (writes manifest with all artifacts)
        // artifacts.finalize()?;
        // 
        // The test now has organized screenshots in:
        // target/test-artifacts/vio_integration/
        //   ├── 3D View.png
        //   ├── Feature Matches.png
        //   ├── Trajectory.png
        //   └── manifest.txt

        println!("Example: Screenshot integration pattern documented");
    }

    /// CLI-based screenshot approach (simpler, no Rerun integration)
    ///
    /// Useful when you need quick screenshots without API integration
    /// - Record RRD file with your VIO data
    /// - Use Rerun CLI to convert to PNG with offscreen rendering
    /// - Store PNG artifacts
    #[test]
    fn example_cli_based_screenshots() {
        // Pattern:
        // 1. Record your visualization to RRD file
        // let rec = RecordingStreamBuilder::new("my_test").save(&rrd_path)?;
        // rec.log("world/trajectory", &curve_data)?;
        // rec.log("world/points", &point_cloud)?;
        //
        // 2. Convert to PNG using CLI
        // Command::new("rerun")
        //     .arg(&rrd_path)
        //     .arg("--screenshot-to")
        //     .arg(&png_path)
        //     .status()?;
        //
        // 3. Store in artifacts
        // fs::copy(&png_path, artifacts.test_dir().join("final_state.png"))?;
        
        println!("Example: CLI-based screenshot pattern documented");
    }

    /// Per-view screenshot helper usage
    ///
    /// Direct API for scheduling and capturing individual view screenshots
    /// from Rerun's GPU renderer
    #[test]
    fn example_per_view_screenshot_helper() {
        // use rs_vio::viewers::{ScreenshotBatch, ViewScreenshotRequest, ScreenshotId};

        // Pattern:
        // 1. Create batch of screenshot requests
        // let mut batch = ScreenshotBatch::new();
        //
        // 2. Schedule screenshots for specific views
        // batch.add(ViewScreenshotRequest::new("3D Visualization", 1920, 1080));
        // batch.add(ViewScreenshotRequest::new("Statistics Panel", 1200, 600));
        //
        // 3. Each request gets unique ScreenshotId for tracking
        // - Automatic timestamp capture
        // - Metadata (view name, dimensions)
        //
        // 4. Integration with Rerun (pseudo-code):
        // for req in batch.into_requests() {
        //     viewer_builder.schedule_screenshot(ctx, req.id());
        // }
        // let result = processor.next_readback_result();
        // result.save_to_file("artifacts/screenshot.png")?;

        println!("Example: Per-view screenshot helper pattern documented");
    }
}

fn main() {
    println!("Screenshot Integration Examples");
    println!("==============================");
    println!();
    println!("1. Test Artifacts (target/test-artifacts/)");
    println!("   - Organized by test name");
    println!("   - Includes manifest.txt with metadata");
    println!("   - Automatic cleanup on success");
    println!();
    println!("2. Per-View Screenshot Helper");
    println!("   - ScreenshotId: Unique identification");
    println!("   - ScreenshotMetadata: Timestamp, dimensions");
    println!("   - ScreenshotBatch: Batch scheduling");
    println!();
    println!("3. ScreenshotResult");
    println!("   - RGBA8 pixel data (Arc-wrapped)");
    println!("   - PNG encoding support");
    println!("   - Direct file saving");
    println!();
    println!("See tests/ for working examples:");
    println!("  - test_screenshot_artifacts.rs (utilities)");
    println!("  - rerun_screenshot_test.rs (CLI approach)");
}
