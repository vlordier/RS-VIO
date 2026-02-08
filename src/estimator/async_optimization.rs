//! Async Optimization Integration
//!
//! Demonstrates async/await patterns for optimization pipeline integration.
//!
//! The design uses Arc<Mutex<>> to share state across async tasks while
//! maintaining Rust's safety guarantees.

use crate::estimator::sliding_window::SlidingWindow;
use anyhow::Result;
use std::sync::Arc;
use std::time::Instant;
use tokio::sync::Mutex;

/// Async wrapper around sliding window optimization
///
/// Provides async/await interface for bundle adjustment and
/// sliding window management. State is protected by a Mutex for concurrent access.
pub struct AsyncOptimizer {
    /// Shared, mutex-protected sliding window
    sliding_window: Arc<Mutex<SlidingWindow>>,
}

impl AsyncOptimizer {
    /// Create a new async optimizer
    pub fn new() -> Self {
        Self {
            sliding_window: Arc::new(Mutex::new(SlidingWindow::new(10))),
        }
    }

    /// Clone the optimizer for sharing across async tasks
    pub fn clone_optimizer(&self) -> Self {
        Self {
            sliding_window: Arc::clone(&self.sliding_window),
        }
    }

    /// Check if two optimizers share the same underlying state
    pub fn shares_state_with(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.sliding_window, &other.sliding_window)
    }

    /// Run optimization on the current sliding window
    pub async fn optimize(&self) -> Result<u64> {
        let start = Instant::now();

        let mut window = self.sliding_window.lock().await;
        let _ = window.optimize();

        let elapsed_ms = start.elapsed().as_millis() as u64;
        Ok(elapsed_ms)
    }

    /// Get the current number of keyframes in the sliding window
    pub async fn keyframe_count(&self) -> usize {
        let window = self.sliding_window.lock().await;
        window.len()
    }

    /// Get the current number of map points
    pub async fn map_point_count(&self) -> usize {
        let window = self.sliding_window.lock().await;
        window.map_points.len()
    }

    /// Check if the sliding window is full
    pub async fn is_full(&self) -> bool {
        let window = self.sliding_window.lock().await;
        window.is_full()
    }
}

impl Default for AsyncOptimizer {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn test_async_optimizer_creation() {
        let optimizer = AsyncOptimizer::new();
        let count = optimizer.keyframe_count().await;
        assert_eq!(count, 0, "New optimizer should have 0 keyframes");
    }

    #[tokio::test]
    async fn test_optimizer_cloning() {
        let optimizer1 = AsyncOptimizer::new();
        let optimizer2 = optimizer1.clone_optimizer();

        assert!(
            optimizer1.shares_state_with(&optimizer2),
            "Cloned optimizers should share state"
        );
    }

    #[tokio::test]
    async fn test_optimizer_state_queries() {
        let optimizer = AsyncOptimizer::new();

        let kf_count = optimizer.keyframe_count().await;
        let mp_count = optimizer.map_point_count().await;
        let is_full = optimizer.is_full().await;

        assert_eq!(kf_count, 0);
        assert_eq!(mp_count, 0);
        assert!(!is_full);
    }
}
