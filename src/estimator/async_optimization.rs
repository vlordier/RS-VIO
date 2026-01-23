//! Async Optimization Integration
//!
//! Wraps the synchronous sliding window optimization for async/await integration
//! into the concurrent VIO pipeline.
//!
//! The design uses Arc<Mutex<>> to share the stateful Backend (sliding window + BA)
//! across async tasks while maintaining Rust's safety guarantees.

use crate::datasets::config::Config;
use crate::estimator::sliding_window::Backend;
use crate::estimator::Frame;
use crate::{Result, VIOError};
use std::sync::Arc;
use std::time::Instant;
use tokio::sync::Mutex;

/// Async wrapper around the sliding window optimization backend
///
/// Provides async/await interface for the synchronous bundle adjustment and
/// sliding window management. The backend state is protected by a Mutex for
/// safe concurrent access.
pub struct AsyncOptimizer {
    /// Shared, mutex-protected optimization backend
    backend: Arc<Mutex<Backend>>,
}

impl AsyncOptimizer {
    /// Create a new async optimizer from config
    pub fn new(config: &Config) -> Self {
        Self {
            backend: Arc::new(Mutex::new(Backend::new(config))),
        }
    }

    /// Clone the optimizer for sharing across async tasks
    pub fn clone_optimizer(&self) -> Self {
        Self {
            backend: Arc::clone(&self.backend),
        }
    }

    /// Check if two optimizers share the same underlying backend state
    pub fn shares_state_with(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.backend, &other.backend)
    }

    /// Add a frame to the sliding window and optionally run optimization
    ///
    /// # Arguments
    /// * `frame` - Keyframe to add to the sliding window
    /// * `run_optimization` - Whether to run bundle adjustment after adding
    ///
    /// # Returns
    /// Tuple of (success: bool, optimization_time_ms: u64)
    pub async fn add_frame_and_optimize(
        &self,
        frame: Frame,
        run_optimization: bool,
    ) -> Result<(bool, u64)> {
        let start = Instant::now();

        // Acquire exclusive access to the backend
        let mut backend = self.backend.lock().await;

        // Add frame to sliding window
        let added = backend.sliding_window.add_frame(frame);
        
        if !added {
            return Ok((false, start.elapsed().as_millis() as u64));
        }

        // Optionally run optimization
        if run_optimization {
            match backend.sliding_window.optimize() {
                Ok(optimized) => {
                    let elapsed_ms = start.elapsed().as_millis() as u64;
                    Ok((optimized, elapsed_ms))
                }
                Err(e) => Err(VIOError::Optimization(format!(
                    "Bundle adjustment failed: {}",
                    e
                ))),
            }
        } else {
            let elapsed_ms = start.elapsed().as_millis() as u64;
            Ok((true, elapsed_ms))
        }
    }

    /// Run optimization on the current sliding window
    ///
    /// # Returns
    /// Tuple of (success: bool, optimization_time_ms: u64)
    pub async fn optimize(&self) -> Result<(bool, u64)> {
        let start = Instant::now();

        let mut backend = self.backend.lock().await;

        match backend.sliding_window.optimize() {
            Ok(optimized) => {
                let elapsed_ms = start.elapsed().as_millis() as u64;
                Ok((optimized, elapsed_ms))
            }
            Err(e) => Err(VIOError::Optimization(format!(
                "Bundle adjustment failed: {}",
                e
            ))),
        }
    }

    /// Get the current number of keyframes in the sliding window
    pub async fn keyframe_count(&self) -> usize {
        let backend = self.backend.lock().await;
        backend.sliding_window.len()
    }

    /// Get the current number of map points
    pub async fn map_point_count(&self) -> usize {
        let backend = self.backend.lock().await;
        backend.sliding_window.map_points_len()
    }

    /// Check if the sliding window is full
    pub async fn is_full(&self) -> bool {
        let backend = self.backend.lock().await;
        backend.sliding_window.is_full()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn create_test_config() -> Config {
        Config::load("config/tum_vi.yaml")
            .expect("Test requires config/tum_vi.yaml")
    }

    #[tokio::test]
    async fn test_async_optimizer_creation() {
        let config = create_test_config();
        let optimizer = AsyncOptimizer::new(&config);
        
        // Verify optimizer was created successfully
        let count = optimizer.keyframe_count().await;
        assert_eq!(count, 0, "New optimizer should have 0 keyframes");
    }

    #[tokio::test]
    async fn test_optimizer_cloning() {
        let config = create_test_config();
        let optimizer1 = AsyncOptimizer::new(&config);
        let optimizer2 = optimizer1.clone_optimizer();

        // Both should reference the same underlying backend
        assert!(
            optimizer1.shares_state_with(&optimizer2),
            "Cloned optimizers should share state"
        );
    }

    #[tokio::test]
    async fn test_optimizer_state_queries() {
        let config = create_test_config();
        let optimizer = AsyncOptimizer::new(&config);

        let kf_count = optimizer.keyframe_count().await;
        let mp_count = optimizer.map_point_count().await;
        let is_full = optimizer.is_full().await;

        assert_eq!(kf_count, 0);
        assert_eq!(mp_count, 0);
        assert!(!is_full);
    }
}
