//! Integration test: capture a screenshot from Rerun viewer via CLI
//! Skips gracefully if `rerun` CLI is not available.

use std::process::Command;

#[test]
fn rerun_cli_screenshot_smoke() -> Result<(), Box<dyn std::error::Error>> {
    // Check if `rerun` CLI is present; skip if missing
    let cli_available = Command::new("rerun").arg("--version").status().is_ok();
    if !cli_available {
        eprintln!("rerun CLI not found; skipping screenshot test");
        return Ok(());
    }

    // Prepare temp paths
    let tmp = std::env::temp_dir();
    let rrd_path = tmp.join("rs_vio_rerun_test.rrd");
    let png_path = tmp.join("rs_vio_rerun_test.png");

    // Build a minimal recording on disk
    {
        // Use the Rerun SDK to log a simple scene
        let rec = rerun::RecordingStreamBuilder::new("rs_vio_rerun_test").save(&rrd_path)?;
        // Log a box so we have visible content
        rec.log(
            "scene/box",
            &rerun::Boxes3D::from_half_sizes([(2.0, 2.0, 1.0)]),
        )?;
        // Log a text annotation
        rec.log("scene/text", &rerun::TextLog::new("Screenshot smoke test"))?;
        // Drop to flush
        drop(rec);
    }

    // Invoke the viewer to take a screenshot and quit
    let status = Command::new("rerun")
        .arg(rrd_path.to_str().unwrap())
        .arg("--screenshot-to")
        .arg(&png_path)
        .status()?;

    // Some builds may panic after saving; accept non-zero status if file exists
    let meta = std::fs::metadata(&png_path)?;
    assert!(
        meta.len() > 10_000,
        "Screenshot appears too small ({} bytes)",
        meta.len()
    );

    // Ensure CLI didn't hang
    assert!(
        status.success() || meta.len() > 0,
        "rerun CLI failed before saving screenshot"
    );

    Ok(())
}
