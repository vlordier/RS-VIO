//! Parallel factor batch operations using Rayon for CPU utilization
//!
//! Provides parallelized batch operations for creating and processing
//! optimization factors in bundle adjustment and other optimization tasks.

use rayon::prelude::*;
use super::factors::{BundleAdjustmentFactor, PinholeProjectionFactor};
use nalgebra as na;
use na::{Matrix4, Vector2};

/// Configuration for parallel factor processing
#[derive(Debug, Clone)]
pub struct ParallelFactorConfig {
    /// Minimum number of factors to parallelize
    pub parallelization_threshold: usize,
}

impl Default for ParallelFactorConfig {
    fn default() -> Self {
        Self {
            parallelization_threshold: 32,
        }
    }
}

/// Factory for creating and managing parallel factor batches
pub struct ParallelFactorBatch {
    config: ParallelFactorConfig,
}

impl ParallelFactorBatch {
    /// Create new parallel factor batch processor
    pub fn new(config: ParallelFactorConfig) -> Self {
        Self { config }
    }

    /// Batch create PinholeProjectionFactors in parallel
    ///
    /// # Arguments
    /// * `observations` - Vector of (2D observation, camera transform) pairs
    ///
    /// # Returns
    /// Vector of created factors
    pub fn create_pinhole_factors_parallel(
        &self,
        observations: Vec<(Vector2<f64>, Matrix4<f64>)>,
    ) -> Vec<PinholeProjectionFactor> {
        if observations.len() < self.config.parallelization_threshold {
            // Serial processing for small batches
            observations
                .into_iter()
                .map(|(obs, transform)| PinholeProjectionFactor::new(obs, transform))
                .collect()
        } else {
            // Parallel processing for large batches using Rayon
            observations
                .into_par_iter()
                .map(|(obs, transform)| PinholeProjectionFactor::new(obs, transform))
                .collect()
        }
    }

    /// Batch create BundleAdjustmentFactors in parallel
    ///
    /// # Arguments
    /// * `observations` - Vector of (2D observation, camera-to-body transform) pairs
    ///
    /// # Returns
    /// Vector of created factors
    pub fn create_ba_factors_parallel(
        &self,
        observations: Vec<(Vector2<f64>, Matrix4<f64>)>,
    ) -> Vec<BundleAdjustmentFactor> {
        if observations.len() < self.config.parallelization_threshold {
            // Serial processing for small batches
            observations
                .into_iter()
                .map(|(obs, T_C_B)| BundleAdjustmentFactor::new(obs, T_C_B))
                .collect()
        } else {
            // Parallel processing for large batches
            observations
                .into_par_iter()
                .map(|(obs, T_C_B)| BundleAdjustmentFactor::new(obs, T_C_B))
                .collect()
        }
    }

    /// Process factors in batches with parallelization
    ///
    /// Useful for applying the same operation to a large set of factors
    pub fn process_batch_parallel<T, F, O>(
        &self,
        items: Vec<T>,
        mapper: F,
    ) -> Vec<O>
    where
        T: Send,
        F: Fn(T) -> O + Send + Sync,
        O: Send,
    {
        if items.len() < self.config.parallelization_threshold {
            items.into_iter().map(mapper).collect()
        } else {
            items.into_par_iter().map(mapper).collect()
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parallel_batch_creation() {
        let batch = ParallelFactorBatch::new(ParallelFactorConfig::default());

        let observations = (0..100)
            .map(|i| {
                (
                    Vector2::new(i as f64 / 100.0, i as f64 / 100.0),
                    Matrix4::identity(),
                )
            })
            .collect::<Vec<_>>();

        let factors = batch.create_pinhole_factors_parallel(observations);
        assert_eq!(factors.len(), 100);
    }

    #[test]
    fn test_small_batch_serial() {
        let batch = ParallelFactorBatch::new(ParallelFactorConfig {
            parallelization_threshold: 1000,
        });

        let observations = vec![
            (Vector2::new(0.5, 0.5), Matrix4::identity()),
            (Vector2::new(0.6, 0.6), Matrix4::identity()),
        ];

        let factors = batch.create_pinhole_factors_parallel(observations);
        assert_eq!(factors.len(), 2);
    }

    #[test]
    fn test_ba_factors_parallel() {
        let batch = ParallelFactorBatch::new(ParallelFactorConfig::default());

        let observations = (0..50)
            .map(|i| {
                (
                    Vector2::new(0.5 + i as f64 * 0.01, 0.5),
                    Matrix4::identity(),
                )
            })
            .collect::<Vec<_>>();

        let factors = batch.create_ba_factors_parallel(observations);
        assert_eq!(factors.len(), 50);
    }

    #[test]
    fn test_generic_batch_processing() {
        let batch = ParallelFactorBatch::new(ParallelFactorConfig::default());

        let numbers: Vec<i32> = (0..100).collect();
        let doubled = batch.process_batch_parallel(numbers, |x| x * 2);

        assert_eq!(doubled.len(), 100);
        assert_eq!(doubled[0], 0);
        assert_eq!(doubled[50], 100);
    }
}
