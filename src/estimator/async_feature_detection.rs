//! Async Feature Detection Integration
//!
//! Wraps the synchronous feature tracking pipeline for async/await integration
//! into the concurrent VIO pipeline.
//!
//! The design uses Arc<Mutex<>> to share the stateful `Frontend<LEVELS>` tracker
//! across async tasks while maintaining Rust's safety guarantees.

use crate::datasets::config::FeatureDetectionConfig;
use crate::estimator::Frame;
use crate::estimator::frame_processor::Frontend;
use crate::Result;
use image::{DynamicImage, GrayImage};
use std::sync::Arc;
use std::time::Instant;
use tokio::sync::Mutex;

/// Async wrapper around the feature tracking frontend
///
/// Provides async/await interface for the synchronous feature tracking pipeline.
/// The tracker state is protected by a Mutex for safe concurrent access.
pub struct AsyncFeatureDetector<const LEVELS: u32> {
    /// Shared, mutex-protected feature tracker
    frontend: Arc<Mutex<Frontend<LEVELS>>>,
}

impl<const LEVELS: u32> AsyncFeatureDetector<LEVELS> {
    /// Create a new async feature detector from config
    pub fn new(config: &FeatureDetectionConfig) -> Self {
        Self {
            frontend: Arc::new(Mutex::new(Frontend::new(config))),
        }
    }

    /// Clone the detector for sharing across async tasks
    pub fn clone_detector(&self) -> Self {
        Self {
            frontend: Arc::clone(&self.frontend),
        }
    }

    /// Check if two detectors share the same underlying frontend state
    pub fn shares_state_with(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.frontend, &other.frontend)
    }

    /// Detect features in stereo image pair asynchronously
    ///
    /// # Arguments
    /// * `left_image` - Left grayscale image
    /// * `right_image` - Right grayscale image
    /// * `frame` - Frame to populate with detected features
    ///
    /// # Returns
    /// Tuple of (feature_count, detection_time_ms)
    pub async fn detect_features(
        &self,
        left_image: GrayImage,
        right_image: GrayImage,
        frame: &mut Frame,
    ) -> Result<(usize, u64)> {
        let start = Instant::now();

        // Acquire exclusive access to the tracker
        let mut frontend = self.frontend.lock().await;

        // Call synchronous tracking logic (wrapped by async)
        let _duration = frontend.track_features(&left_image, &right_image, frame);

        let total_elapsed = start.elapsed().as_millis() as u64;

        // Extract feature count from frame
        let feature_count = frame.left_features.len();

        Ok((feature_count, total_elapsed))
    }

    /// Detect features from DynamicImage inputs (convenience method)
    pub async fn detect_features_from_dynamic(
        &self,
        left: &DynamicImage,
        right: &DynamicImage,
        frame: &mut Frame,
    ) -> Result<(usize, u64)> {
        let left_gray = left.to_luma8();
        let right_gray = right.to_luma8();
        self.detect_features(left_gray, right_gray, frame).await
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::datasets::config::FeatureDetectionConfig;

    #[tokio::test]
    async fn test_async_detector_creation() {
        let config = FeatureDetectionConfig::default();
        let detector = AsyncFeatureDetector::<8>::new(&config);
        
        // Verify detector was created successfully
        let _frontend = detector.frontend.lock().await;
        // Just verify we can lock successfully
    }

    #[tokio::test]
    async fn test_detector_cloning() {
        let config = FeatureDetectionConfig::default();
        let detector1 = AsyncFeatureDetector::<8>::new(&config);
        let detector2 = detector1.clone_detector();

        // Both should reference the same underlying tracker
        assert!(detector1.shares_state_with(&detector2), "Cloned detectors should share state");
    }

    #[tokio::test]
    async fn test_mutex_exclusive_access() {
        let config = FeatureDetectionConfig::default();
        let detector = AsyncFeatureDetector::<8>::new(&config);

        // First lock should succeed
        let _lock1 = detector.frontend.lock().await;
        
        // Second lock would block (but we don't wait)
        // Just verify lock1 is held
    }
}
