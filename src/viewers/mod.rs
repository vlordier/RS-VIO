//! # Visualization Module
//!
//! Real-time 3D visualization of VIO results.
//!
//! ## Overview
//!
//! Provides visualization tools for monitoring VIO pipeline output:
//! - Camera trajectories
//! - 3D map points
//! - Camera frustums
//! - Feature tracking (in-progress)
//!
//! ## Components
//!
//! - [`Viewer`] - Visualization trait
//! - [`RerunViewer`] - Rerun.io backend (optional, gated by "rerun-viewer" feature)
//! - Color mapping for consistent feature visualization
//!
//! ## Visualization Backend
//!
//! By default (feature "rerun-viewer" disabled), uses a no-op stub that maintains
//! API compatibility but eliminates ~30% of IR bloat from Rerun/Arrow/Parquet stack.
//!
//! When "rerun-viewer" feature is enabled, uses [Rerun.io](https://www.rerun.io/):
//! - Real-time 3D visualization
//! - Low-latency streaming
//! - Rich annotation support
//! - Cloud-native architecture

pub mod screenshot_helper;
pub mod viewer;

pub use rerun::{create_viewer, RerunViewer};
pub use screenshot_helper::{
    ScreenshotBatch, ScreenshotId, ScreenshotMetadata, ScreenshotResult, ViewScreenshotRequest,
};
pub use viewer::Viewer;

#[cfg(feature = "rerun-viewer")]
pub mod rerun;

#[cfg(not(feature = "rerun-viewer"))]
pub mod rerun_stub;

#[cfg(not(feature = "rerun-viewer"))]
pub use rerun_stub as rerun;
use std::collections::HashMap;
use std::sync::Mutex;

/// Global color map for feature IDs
static FEATURE_COLOR_MAP: Mutex<Option<HashMap<usize, [u8; 3]>>> = Mutex::new(None);

/// Get or assign a color for a given feature ID
pub fn get_feature_color(feature_id: usize) -> [u8; 3] {
    let mut map = match FEATURE_COLOR_MAP.lock() {
        Ok(m) => m,
        Err(poisoned) => {
            log::warn!("[Viewer] Mutex was poisoned, recovering");
            poisoned.into_inner()
        },
    };

    if map.is_none() {
        *map = Some(HashMap::new());
    }

    let map = match map.as_mut() {
        Some(m) => m,
        None => {
            log::error!("[Viewer] Color map is unexpectedly None");
            return [128, 128, 128];
        },
    };

    if let Some(&color) = map.get(&feature_id) {
        return color;
    }

    let hash = feature_id as u64;
    let r = ((hash * 2654435761) % 256) as u8;
    let g = ((hash * 2246822507) % 256) as u8;
    let b = ((hash * 3266489917) % 256) as u8;

    let color = [r.max(50), g.max(50), b.max(50)];
    map.insert(feature_id, color);
    color
}
