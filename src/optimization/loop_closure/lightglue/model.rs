//! Model loading and initialization for LightGlue ONNX Runtime.

#[cfg(feature = "lightglue")]
use ort::session::{builder::GraphOptimizationLevel, Session};

use super::config::LightGlueConfig;

#[cfg(feature = "lightglue")]
/// Load ONNX model and create session
pub fn load_model(config: &LightGlueConfig) -> Result<Session, String> {
    if !config.model_path.exists() {
        return Err(format!(
            "LightGlue model not found at {:?}. Download from https://github.com/fabio-sim/LightGlue-ONNX",
            config.model_path
        ));
    }

    let session = Session::builder()
        .map_err(|e| format!("Failed to create session builder: {}", e))?
        .with_optimization_level(GraphOptimizationLevel::Level3)
        .map_err(|e| format!("Failed to set optimization level: {}", e))?
        .with_intra_threads(4)
        .map_err(|e| format!("Failed to set threads: {}", e))?
        .commit_from_file(&config.model_path)
        .map_err(|e| format!("Failed to load model: {}", e))?;

    log::info!("LightGlue model loaded from {:?}", config.model_path);
    Ok(session)
}
