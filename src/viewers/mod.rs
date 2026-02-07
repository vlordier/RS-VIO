//! Visualization back-ends for VIO output.
//!
//! Defines the [`Viewer`] trait and optional Rerun-based implementation
//! for logging poses, point clouds, and images.

#[cfg(feature = "rerun-viewer")]
pub mod rerun;
pub mod viewer;

#[cfg(feature = "rerun-viewer")]
pub use self::rerun::RerunViewer;
pub use viewer::Viewer;

use anyhow::Result;

/// Create a viewer instance. Returns the Rerun viewer when the `rerun-viewer`
/// feature is enabled, otherwise returns `Ok(None)`.
pub fn create_viewer() -> Result<Option<Box<dyn Viewer>>> {
    #[cfg(feature = "rerun-viewer")]
    {
        self::rerun::create_viewer().map(Some)
    }
    #[cfg(not(feature = "rerun-viewer"))]
    {
        Ok(None)
    }
}

/// Get a deterministic color for a given feature ID.
/// Uses hash-based color assignment — no caching needed since the function is pure.
pub fn get_feature_color(feature_id: usize) -> [u8; 3] {
    let hash = feature_id as u64;
    let r = ((hash.wrapping_mul(2654435761)) % 256) as u8;
    let g = ((hash.wrapping_mul(2246822507)) % 256) as u8;
    let b = ((hash.wrapping_mul(3266489917)) % 256) as u8;
    // Ensure minimum brightness for visibility
    [r.max(50), g.max(50), b.max(50)]
}
