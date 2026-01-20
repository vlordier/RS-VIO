//! Per-view screenshot capture helper using Rerun renderer APIs.
//!
//! This module provides utilities to capture screenshots of individual views
//! via the Rerun viewer's GPU rendering pipeline, using ViewBuilder scheduling
//! and ScreenshotProcessor readback.

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;

/// Unique identifier for screenshot requests
#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash)]
pub struct ScreenshotId(u64);

impl ScreenshotId {
    /// Generate a new unique screenshot ID
    pub fn new() -> Self {
        static COUNTER: AtomicU64 = AtomicU64::new(1);
        ScreenshotId(COUNTER.fetch_add(1, Ordering::SeqCst))
    }
}

impl Default for ScreenshotId {
    fn default() -> Self {
        Self::new()
    }
}

/// Metadata about a captured screenshot
#[derive(Clone, Debug)]
pub struct ScreenshotMetadata {
    /// Unique identifier for this screenshot
    pub id: ScreenshotId,
    /// Name of the view that was captured (e.g., "3D Scene", "Time Series")
    pub view_name: String,
    /// Width in pixels
    pub width: u32,
    /// Height in pixels
    pub height: u32,
    /// Timestamp when the screenshot was taken (nanoseconds since epoch)
    pub timestamp_ns: i64,
}

/// Result of a screenshot capture operation
#[derive(Clone)]
pub struct ScreenshotResult {
    /// Metadata about the capture
    pub metadata: ScreenshotMetadata,
    /// Raw RGBA8 pixel data (uncompressed)
    pub pixel_data: Arc<Vec<u8>>,
}

impl ScreenshotResult {
    /// Encode the screenshot to PNG format
    pub fn to_png(&self) -> Result<Vec<u8>, String> {
        use image::ImageEncoder as _;

        let mut png_bytes = Vec::new();
        image::codecs::png::PngEncoder::new(&mut png_bytes)
            .write_image(
                &self.pixel_data,
                self.metadata.width,
                self.metadata.height,
                image::ExtendedColorType::Rgba8,
            )
            .map_err(|e| format!("PNG encoding error: {}", e))?;

        Ok(png_bytes)
    }

    /// Save the screenshot to a PNG file
    pub fn save_to_file(&self, path: impl AsRef<std::path::Path>) -> Result<(), String> {
        let png_data = self.to_png()?;
        std::fs::write(path, png_data).map_err(|e| format!("Failed to write PNG file: {}", e))
    }
}

/// Builder for scheduling view screenshots
pub struct ViewScreenshotRequest {
    id: ScreenshotId,
    view_name: String,
    width: u32,
    height: u32,
}

impl ViewScreenshotRequest {
    /// Create a new screenshot request for a view
    pub fn new(view_name: impl Into<String>, width: u32, height: u32) -> Self {
        Self {
            id: ScreenshotId::new(),
            view_name: view_name.into(),
            width,
            height,
        }
    }

    /// Get the screenshot ID (for tracking readback results)
    pub fn id(&self) -> ScreenshotId {
        self.id
    }

    /// Get the view name
    pub fn view_name(&self) -> &str {
        &self.view_name
    }

    /// Get dimensions
    pub fn dimensions(&self) -> (u32, u32) {
        (self.width, self.height)
    }

    /// Build metadata for this screenshot
    pub fn build_metadata(&self) -> ScreenshotMetadata {
        ScreenshotMetadata {
            id: self.id,
            view_name: self.view_name.clone(),
            width: self.width,
            height: self.height,
            timestamp_ns: {
                use std::time::{SystemTime, UNIX_EPOCH};
                SystemTime::now()
                    .duration_since(UNIX_EPOCH)
                    .map(|d| d.as_nanos() as i64)
                    .unwrap_or(0)
            },
        }
    }
}

/// Batch screenshot capture manager
pub struct ScreenshotBatch {
    requests: Vec<ViewScreenshotRequest>,
}

impl ScreenshotBatch {
    /// Create a new empty batch
    pub fn new() -> Self {
        Self {
            requests: Vec::new(),
        }
    }

    /// Add a screenshot request to the batch
    pub fn add(&mut self, request: ViewScreenshotRequest) {
        self.requests.push(request);
    }

    /// Get all pending requests
    pub fn requests(&self) -> &[ViewScreenshotRequest] {
        &self.requests
    }

    /// Get the count of pending requests
    pub fn len(&self) -> usize {
        self.requests.len()
    }

    /// Check if batch is empty
    pub fn is_empty(&self) -> bool {
        self.requests.is_empty()
    }

    /// Clear all pending requests
    pub fn clear(&mut self) {
        self.requests.clear();
    }

    /// Consume batch and return requests
    pub fn into_requests(self) -> Vec<ViewScreenshotRequest> {
        self.requests
    }
}

impl Default for ScreenshotBatch {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_screenshot_id_generation() {
        let id1 = ScreenshotId::new();
        let id2 = ScreenshotId::new();
        assert_ne!(id1, id2);
    }

    #[test]
    fn test_screenshot_request_creation() {
        let req = ViewScreenshotRequest::new("3D View", 1024, 768);
        assert_eq!(req.view_name(), "3D View");
        assert_eq!(req.dimensions(), (1024, 768));
    }

    #[test]
    fn test_screenshot_batch() {
        let mut batch = ScreenshotBatch::new();
        assert!(batch.is_empty());

        batch.add(ViewScreenshotRequest::new("View1", 800, 600));
        batch.add(ViewScreenshotRequest::new("View2", 1024, 768));

        assert_eq!(batch.len(), 2);
        assert!(!batch.is_empty());

        let requests = batch.into_requests();
        assert_eq!(requests.len(), 2);
    }

    #[test]
    fn test_screenshot_metadata() {
        let req = ViewScreenshotRequest::new("Test View", 512, 512);
        let meta = req.build_metadata();

        assert_eq!(meta.view_name, "Test View");
        assert_eq!(meta.width, 512);
        assert_eq!(meta.height, 512);
        assert!(meta.timestamp_ns > 0);
    }
}
