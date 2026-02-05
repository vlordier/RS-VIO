//! Async wrapper for sequential estimator pipeline
//!
//! Provides async/await interface for the synchronous Estimator,
//! enabling non-blocking frame submission and result retrieval.

use anyhow::Result;
use crate::datasets::ImuData;

/// Async wrapper around sequential Estimator
///
/// Spawns blocking Estimator operations on dedicated tokio blocking threads
/// to prevent starving other async tasks.
pub struct AsyncEstimator {
    // Note: Estimator contains non-Send types, so we can't easily spawn_blocking
    // Instead, we provide async methods that use blocking semaphores
    _phantom: std::marker::PhantomData<()>,
}

impl AsyncEstimator {
    /// Create new async estimator (placeholder for future implementation)
    ///
    /// Currently returns a stub. Full async support requires redesigning
    /// Estimator to support Send+Sync or using external process communication.
    pub fn new() -> Self {
        Self {
            _phantom: std::marker::PhantomData,
        }
    }

    /// Process frame asynchronously (placeholder)
    pub async fn process_frame_async(
        &self,
        _left_image: Vec<u8>,
        _right_image: Vec<u8>,
        _timestamp_ns: i64,
        _imu_data: Option<Vec<ImuData>>,
    ) -> Result<()> {
        // TODO: Implement async frame processing
        // Challenge: Estimator contains non-Send types (references to strategy traits, etc.)
        // Options:
        // 1. Redesign Estimator to use Send-safe trait objects
        // 2. Use external process with IPC
        // 3. Spawn multiple Estimator instances, one per async task
        Err(anyhow::anyhow!(
            "Async processing not yet implemented"
        ))
    }

    /// Get current state asynchronously (placeholder)
    pub async fn get_pose(&self) -> Result<nalgebra::Isometry3<f32>> {
        // TODO: Implement pose retrieval
        Err(anyhow::anyhow!(
            "Pose retrieval not yet implemented"
        ))
    }

    /// Shutdown the async estimator gracefully
    pub async fn shutdown(self) {
        // No-op for now
    }
}

#[cfg(test)]
mod tests {
    #[tokio::test]
    async fn test_async_estimator_creation() {
        // This test will compile once Estimator can be imported properly
        // For now, we're testing the API structure
    }
}
