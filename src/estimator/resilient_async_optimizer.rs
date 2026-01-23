//! Enhanced async optimization with resilience and timeout handling
//!
//! Wraps the AsyncOptimizer with timeout, error recovery, and metrics collection.

use crate::datasets::config::Config;
use crate::estimator::{AsyncOptimizer, Frame, MetricsTimer, PipelineError, PipelineMetrics};
use std::time::Duration;
use tokio::time::timeout;

const DEFAULT_OPTIMIZATION_TIMEOUT_MS: u64 = 500;

/// Resilient async optimizer with timeout and error recovery
pub struct ResilientAsyncOptimizer {
    optimizer: AsyncOptimizer,
    timeout_ms: u64,
    metrics: PipelineMetrics,
}

impl ResilientAsyncOptimizer {
    /// Create new resilient async optimizer
    pub fn new(config: &Config, metrics: Option<PipelineMetrics>) -> Self {
        Self {
            optimizer: AsyncOptimizer::new(config),
            timeout_ms: DEFAULT_OPTIMIZATION_TIMEOUT_MS,
            metrics: metrics.unwrap_or_default(),
        }
    }

    /// Set custom timeout for optimization
    pub fn with_timeout(mut self, timeout_ms: u64) -> Self {
        self.timeout_ms = timeout_ms;
        self
    }

    /// Clone for task sharing
    pub fn clone_optimizer(&self) -> Self {
        Self {
            optimizer: self.optimizer.clone_optimizer(),
            timeout_ms: self.timeout_ms,
            metrics: self.metrics.clone(),
        }
    }

    /// Add frame with timeout and recovery
    pub async fn add_frame_and_optimize(
        &self,
        frame: Frame,
        run_ba: bool,
    ) -> std::result::Result<bool, PipelineError> {
        let timer = MetricsTimer::new_optimization(&self.metrics);

        let timeout_duration = Duration::from_millis(self.timeout_ms);

        match timeout(
            timeout_duration,
            self.optimizer.add_frame_and_optimize(frame, run_ba),
        )
        .await
        {
            Ok(Ok((success, _))) => {
                timer.stop(false);
                Ok(success)
            },
            Ok(Err(e)) => {
                timer.stop(true);
                self.metrics.record_recovered_error();
                Err(PipelineError::OptimizationFailed(e.to_string()))
            },
            Err(_) => {
                let elapsed_ms = timer.elapsed_us() / 1000;
                timer.stop(true);
                self.metrics.record_recovered_error();
                Err(PipelineError::Timeout {
                    stage: "optimization".to_string(),
                    limit_ms: self.timeout_ms,
                    elapsed_ms,
                })
            },
        }
    }

    /// Query state (non-blocking)
    pub async fn keyframe_count(&self) -> usize {
        self.optimizer.keyframe_count().await
    }

    pub async fn map_point_count(&self) -> usize {
        self.optimizer.map_point_count().await
    }

    pub fn metrics(&self) -> &PipelineMetrics {
        &self.metrics
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::datasets::config::Config;

    fn create_test_config() -> Config {
        Config::load("config/tum_vi.yaml").expect("Test requires config/tum_vi.yaml")
    }

    #[tokio::test]
    async fn test_resilient_optimizer_creation() {
        let config = create_test_config();
        let optimizer = ResilientAsyncOptimizer::new(&config, None);

        assert_eq!(optimizer.keyframe_count().await, 0);
        assert_eq!(optimizer.map_point_count().await, 0);
    }

    #[tokio::test]
    async fn test_resilient_optimizer_with_custom_timeout() {
        let config = create_test_config();
        let optimizer = ResilientAsyncOptimizer::new(&config, None).with_timeout(1000);

        assert_eq!(optimizer.timeout_ms, 1000);
    }

    #[tokio::test]
    async fn test_optimizer_metrics_tracking() {
        let config = create_test_config();
        let metrics = PipelineMetrics::new();
        let _optimizer = ResilientAsyncOptimizer::new(&config, Some(metrics.clone()));

        let initial_frames = metrics.total_frames_processed();
        assert_eq!(initial_frames, 0);
    }
}
