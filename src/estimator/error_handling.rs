//! Production error handling and recovery for async pipeline
//!
//! Provides structured error types, recovery strategies, and graceful degradation
//! for the concurrent VIO pipeline.

use crate::VIOError;
use std::fmt;
use std::time::Duration;

/// Errors specific to the async VIO pipeline
#[derive(Debug, Clone)]
pub enum PipelineError {
    /// Mutex protecting state was poisoned (indicates panicked task)
    PoisonedState(String),
    /// Channel closed unexpectedly during processing
    ChannelClosed(String),
    /// Processing exceeded timeout threshold
    Timeout {
        stage: String,
        limit_ms: u64,
        elapsed_ms: u64,
    },
    /// Feature detection failed (e.g., invalid image data)
    FeatureDetectionFailed(String),
    /// Optimization/bundle adjustment failed
    OptimizationFailed(String),
    /// Frame buffer exceeded maximum size or memory limit
    BufferOverflow(String),
    /// Configuration error (invalid parameters, missing files)
    ConfigError(String),
    /// Recoverable transient error (should retry)
    Transient(String),
    /// Unrecoverable error
    Fatal(String),
}

impl fmt::Display for PipelineError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::PoisonedState(msg) => write!(f, "Poisoned state: {}", msg),
            Self::ChannelClosed(msg) => write!(f, "Channel closed: {}", msg),
            Self::Timeout {
                stage,
                limit_ms,
                elapsed_ms,
            } => {
                write!(
                    f,
                    "Timeout in {}: {}ms > {}ms limit",
                    stage, elapsed_ms, limit_ms
                )
            },
            Self::FeatureDetectionFailed(msg) => write!(f, "Feature detection failed: {}", msg),
            Self::OptimizationFailed(msg) => write!(f, "Optimization failed: {}", msg),
            Self::BufferOverflow(msg) => write!(f, "Buffer overflow: {}", msg),
            Self::ConfigError(msg) => write!(f, "Configuration error: {}", msg),
            Self::Transient(msg) => write!(f, "Transient error (recoverable): {}", msg),
            Self::Fatal(msg) => write!(f, "Fatal error: {}", msg),
        }
    }
}

impl std::error::Error for PipelineError {}

impl From<PipelineError> for crate::VIOError {
    fn from(err: PipelineError) -> Self {
        match err {
            PipelineError::Transient(msg) => VIOError::Transient(msg),
            PipelineError::ChannelClosed(msg) => VIOError::Transient(msg),
            _ => VIOError::Transient(err.to_string()),
        }
    }
}

/// Recovery strategy for pipeline errors
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RecoveryStrategy {
    /// Retry the operation (up to max_attempts times)
    Retry { max_attempts: u32, backoff_ms: u64 },
    /// Skip this frame and continue with next
    Skip,
    /// Skip this frame and slow down pipeline temporarily
    SlowDown { backoff_ms: u64 },
    /// Gracefully shut down pipeline
    Shutdown,
}

impl Default for RecoveryStrategy {
    fn default() -> Self {
        Self::Retry {
            max_attempts: 3,
            backoff_ms: 10,
        }
    }
}

impl RecoveryStrategy {
    /// Determine recovery strategy based on error type
    pub fn for_error(err: &PipelineError) -> Self {
        match err {
            // Transient errors: retry with backoff
            PipelineError::Transient(_) | PipelineError::ChannelClosed(_) => Self::Retry {
                max_attempts: 3,
                backoff_ms: 50,
            },
            // Timeout: skip frame and slow down
            PipelineError::Timeout { .. } => Self::SlowDown { backoff_ms: 33 },
            // Feature detection failure: try next frame
            PipelineError::FeatureDetectionFailed(_) => Self::Skip,
            // Optimization failure: retry with backoff
            PipelineError::OptimizationFailed(_) => Self::Retry {
                max_attempts: 2,
                backoff_ms: 100,
            },
            // Poisoned state: try once more, then skip
            PipelineError::PoisonedState(_) => Self::Retry {
                max_attempts: 1,
                backoff_ms: 0,
            },
            // Buffer/config errors: shutdown
            PipelineError::BufferOverflow(_) | PipelineError::ConfigError(_) => Self::Shutdown,
            // Fatal: always shutdown
            PipelineError::Fatal(_) => Self::Shutdown,
        }
    }

    /// Compute backoff duration
    pub fn backoff_for_attempt(&self, attempt: u32) -> Duration {
        match self {
            Self::Retry { backoff_ms, .. } => {
                // Exponential backoff: backoff_ms, backoff_ms*2, backoff_ms*4, ...
                let ms = backoff_ms * (2_u64.pow(attempt));
                Duration::from_millis(ms)
            },
            Self::SlowDown { backoff_ms } => Duration::from_millis(*backoff_ms),
            _ => Duration::ZERO,
        }
    }

    /// Check if this strategy supports retries
    pub fn is_retryable(&self) -> bool {
        matches!(self, Self::Retry { .. })
    }
}

/// Error recovery context for tracking retry state
#[derive(Debug, Clone)]
pub struct RecoveryContext {
    pub frame_id: u64,
    pub stage: String,
    pub attempt: u32,
    pub error_count: u32,
}

impl RecoveryContext {
    pub fn new(frame_id: u64, stage: String) -> Self {
        Self {
            frame_id,
            stage,
            attempt: 0,
            error_count: 0,
        }
    }

    pub fn next_attempt(&mut self) {
        self.attempt += 1;
        self.error_count += 1;
    }

    pub fn can_retry(&self, strategy: &RecoveryStrategy) -> bool {
        match strategy {
            RecoveryStrategy::Retry { max_attempts, .. } => self.attempt < *max_attempts,
            _ => false,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_recovery_strategy_for_transient() {
        let err = PipelineError::Transient("test".to_string());
        let strategy = RecoveryStrategy::for_error(&err);
        assert!(strategy.is_retryable());
    }

    #[test]
    fn test_recovery_strategy_for_fatal() {
        let err = PipelineError::Fatal("test".to_string());
        let strategy = RecoveryStrategy::for_error(&err);
        assert_eq!(strategy, RecoveryStrategy::Shutdown);
    }

    #[test]
    fn test_backoff_exponential() {
        let strategy = RecoveryStrategy::Retry {
            max_attempts: 3,
            backoff_ms: 10,
        };
        assert_eq!(strategy.backoff_for_attempt(0).as_millis(), 10);
        assert_eq!(strategy.backoff_for_attempt(1).as_millis(), 20);
        assert_eq!(strategy.backoff_for_attempt(2).as_millis(), 40);
    }

    #[test]
    fn test_recovery_context() {
        let mut ctx = RecoveryContext::new(42, "feature_detection".to_string());
        assert_eq!(ctx.attempt, 0);
        assert_eq!(ctx.error_count, 0);

        ctx.next_attempt();
        assert_eq!(ctx.attempt, 1);
        assert_eq!(ctx.error_count, 1);

        let strategy = RecoveryStrategy::Retry {
            max_attempts: 3,
            backoff_ms: 10,
        };
        assert!(ctx.can_retry(&strategy));

        ctx.next_attempt();
        ctx.next_attempt();
        assert!(!ctx.can_retry(&strategy));
    }
}
