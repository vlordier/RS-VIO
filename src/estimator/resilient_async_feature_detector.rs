//! Enhanced async feature detection with resilience and error recovery
//!
//! Wraps AsyncFeatureDetector with timeout, validation, and graceful degradation.

use crate::datasets::config::FeatureDetectionConfig;
use crate::estimator::{AsyncFeatureDetector, Frame, MetricsTimer, PipelineError, PipelineMetrics};
use image::GrayImage;
use std::time::Duration;
use tokio::time::timeout;

const DEFAULT_DETECTION_TIMEOUT_MS: u64 = 100;
const MIN_FEATURES: usize = 50;

/// Resilient async feature detector with validation and recovery
pub struct ResilientAsyncFeatureDetector<const LEVELS: u32> {
    detector: AsyncFeatureDetector<LEVELS>,
    timeout_ms: u64,
    metrics: PipelineMetrics,
    min_features: usize,
}

impl<const LEVELS: u32> ResilientAsyncFeatureDetector<LEVELS> {
    /// Create new resilient feature detector
    pub fn new(config: &FeatureDetectionConfig, metrics: Option<PipelineMetrics>) -> Self {
        Self {
            detector: AsyncFeatureDetector::new(config),
            timeout_ms: DEFAULT_DETECTION_TIMEOUT_MS,
            metrics: metrics.unwrap_or_default(),
            min_features: MIN_FEATURES,
        }
    }

    /// Set custom timeout for detection
    pub fn with_timeout(mut self, timeout_ms: u64) -> Self {
        self.timeout_ms = timeout_ms;
        self
    }

    /// Set minimum required features (soft requirement)
    pub fn with_min_features(mut self, min: usize) -> Self {
        self.min_features = min;
        self
    }

    /// Clone for task sharing
    pub fn clone_detector(&self) -> Self {
        Self {
            detector: self.detector.clone_detector(),
            timeout_ms: self.timeout_ms,
            metrics: self.metrics.clone(),
            min_features: self.min_features,
        }
    }

    /// Detect features with timeout and validation
    pub async fn detect_features(
        &self,
        left_image: GrayImage,
        right_image: GrayImage,
        frame: &mut Frame,
    ) -> std::result::Result<usize, PipelineError> {
        // Validate input images
        if left_image.width() == 0 || left_image.height() == 0 {
            return Err(PipelineError::FeatureDetectionFailed(
                "Left image has zero dimensions".to_string(),
            ));
        }
        if right_image.width() == 0 || right_image.height() == 0 {
            return Err(PipelineError::FeatureDetectionFailed(
                "Right image has zero dimensions".to_string(),
            ));
        }

        let timer = MetricsTimer::new_detection(&self.metrics);
        let timeout_duration = Duration::from_millis(self.timeout_ms);

        match timeout(
            timeout_duration,
            self.detector
                .detect_features(left_image, right_image, frame),
        )
        .await
        {
            Ok(Ok((feature_count, _))) => {
                // Warn if feature count is below expected
                if feature_count < self.min_features {
                    tracing::warn!(
                        "Low feature count: {} < {} (expected)",
                        feature_count,
                        self.min_features
                    );
                    // Still succeed - might recover in next frame
                }
                timer.stop(false);
                Ok(feature_count)
            },
            Ok(Err(e)) => {
                timer.stop(true);
                self.metrics.record_recovered_error();
                Err(PipelineError::FeatureDetectionFailed(e.to_string()))
            },
            Err(_) => {
                let elapsed_ms = timer.elapsed_us() / 1000;
                timer.stop(true);
                self.metrics.record_recovered_error();
                Err(PipelineError::Timeout {
                    stage: "feature_detection".to_string(),
                    limit_ms: self.timeout_ms,
                    elapsed_ms,
                })
            },
        }
    }

    /// Check if detectors share the same underlying state
    pub fn shares_state_with(&self, other: &Self) -> bool {
        self.detector.shares_state_with(&other.detector)
    }

    pub fn metrics(&self) -> &PipelineMetrics {
        &self.metrics
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::datasets::config::FeatureDetectionConfig;

    #[test]
    fn test_resilient_detector_creation() {
        let config = FeatureDetectionConfig::default();
        let detector = ResilientAsyncFeatureDetector::<3>::new(&config, None);
        assert_eq!(detector.timeout_ms, DEFAULT_DETECTION_TIMEOUT_MS);
        assert_eq!(detector.min_features, MIN_FEATURES);
    }

    #[test]
    fn test_detector_with_custom_timeout() {
        let config = FeatureDetectionConfig::default();
        let detector = ResilientAsyncFeatureDetector::<3>::new(&config, None)
            .with_timeout(200)
            .with_min_features(100);

        assert_eq!(detector.timeout_ms, 200);
        assert_eq!(detector.min_features, 100);
    }

    #[test]
    fn test_detector_clone_shares_state() {
        let config = FeatureDetectionConfig::default();
        let detector1 = ResilientAsyncFeatureDetector::<3>::new(&config, None);
        let detector2 = detector1.clone_detector();

        assert!(detector1.shares_state_with(&detector2));
    }

    #[test]
    fn test_metrics_integration() {
        let config = FeatureDetectionConfig::default();
        let metrics = PipelineMetrics::new();
        let _detector = ResilientAsyncFeatureDetector::<3>::new(&config, Some(metrics.clone()));

        assert_eq!(metrics.total_frames_processed(), 0);
    }
}
