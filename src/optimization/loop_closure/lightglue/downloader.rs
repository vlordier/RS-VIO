//! Model downloader helper for LightGlue.

use std::path::{Path, PathBuf};

/// LightGlue model downloader helper
pub struct LightGlueModelDownloader;

impl LightGlueModelDownloader {
    /// Download pre-trained LightGlue ONNX model
    pub fn download_model(output_dir: &Path, feature_type: &str) -> Result<PathBuf, String> {
        let model_name = format!("lightglue_{}.onnx", feature_type);
        let output_path = output_dir.join(&model_name);

        if output_path.exists() {
            log::info!("Model already exists at {:?}", output_path);
            return Ok(output_path);
        }

        let url = format!(
            "https://github.com/fabio-sim/LightGlue-ONNX/releases/download/v1.0/{}",
            model_name
        );

        log::info!("Downloading LightGlue model from {}", url);
        log::warn!("Auto-download not implemented. Please download manually:");
        log::warn!("  1. Visit: https://github.com/fabio-sim/LightGlue-ONNX/releases");
        log::warn!("  2. Download {}", model_name);
        log::warn!("  3. Place at {:?}", output_path);

        Err("Manual download required".to_string())
    }
}
