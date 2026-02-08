//! Parallel factor batch operations using Rayon for CPU utilization
//!
//! Provides parallelized batch operations for creating and processing
//! optimization factors in bundle adjustment and other optimization tasks.

use super::factors::{BundleAdjustmentFactor, PinholeProjectionFactor};
use na::{Matrix4, Vector2};
use nalgebra as na;
use rayon::prelude::*;

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
    pub const fn new(config: ParallelFactorConfig) -> Self {
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
        self.process_batch_parallel(observations, |(obs, transform)| {
            PinholeProjectionFactor::new(obs, transform)
        })
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
        self.process_batch_parallel(observations, |(obs, T_C_B)| {
            BundleAdjustmentFactor::new(obs, std::sync::Arc::new(T_C_B))
        })
    }

    /// Process factors in batches with parallelization
    ///
    /// Useful for applying the same operation to a large set of factors
    pub fn process_batch_parallel<T, F, O>(&self, items: Vec<T>, mapper: F) -> Vec<O>
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
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::cloned_ref_to_slice_refs)]
mod tests {
    use super::*;

    use apex_solver::factors::Factor;
    use na::DVector;

    #[test]
    fn test_empty_batch() {
        let batch = ParallelFactorBatch::new(ParallelFactorConfig::default());
        let observations: Vec<(Vector2<f64>, Matrix4<f64>)> = Vec::new();

        let factors = batch.create_pinhole_factors_parallel(observations);
        assert!(factors.is_empty());
    }

    /// Verify parallel and serial paths produce numerically identical factors.
    ///
    /// Creates 200 pinhole factors via both the serial path (threshold=10000)
    /// and the parallel path (threshold=1), then compares every factor's
    /// linearization output (residual + Jacobian) at the same evaluation point.
    #[test]
    fn test_serial_parallel_pinhole_factors_numerically_identical() {
        let observations: Vec<(Vector2<f64>, Matrix4<f64>)> = (0..200)
            .map(|i| {
                let u = (i as f64) * 0.005 - 0.5;
                let v = (i as f64) * 0.003 - 0.3;
                (Vector2::new(u, v), Matrix4::identity())
            })
            .collect();

        let serial_batch = ParallelFactorBatch::new(ParallelFactorConfig {
            parallelization_threshold: 10_000,
        });
        let parallel_batch = ParallelFactorBatch::new(ParallelFactorConfig {
            parallelization_threshold: 1,
        });

        let serial_factors = serial_batch.create_pinhole_factors_parallel(observations.clone());
        let parallel_factors = parallel_batch.create_pinhole_factors_parallel(observations);

        assert_eq!(serial_factors.len(), parallel_factors.len());

        // Evaluate each factor at the same 3D point and compare residual + Jacobian
        let test_point = DVector::from_vec(vec![0.3, -0.2, 2.0]);
        for (i, (sf, pf)) in serial_factors.iter().zip(parallel_factors.iter()).enumerate() {
            let (r_s, j_s) = sf.linearize(&[test_point.clone()], true);
            let (r_p, j_p) = pf.linearize(&[test_point.clone()], true);

            assert_eq!(
                r_s.as_slice(),
                r_p.as_slice(),
                "Residual mismatch at factor {i}"
            );
            let j_s = j_s.expect("serial Jacobian");
            let j_p = j_p.expect("parallel Jacobian");
            assert_eq!(
                j_s.as_slice(),
                j_p.as_slice(),
                "Jacobian mismatch at factor {i}"
            );
        }
    }

    /// Same equivalence check for BundleAdjustmentFactors.
    #[test]
    fn test_serial_parallel_ba_factors_numerically_identical() {
        let observations: Vec<(Vector2<f64>, Matrix4<f64>)> = (0..200)
            .map(|i| {
                let u = (i as f64) * 0.004 - 0.4;
                let v = (i as f64) * 0.002 - 0.2;
                (Vector2::new(u, v), Matrix4::identity())
            })
            .collect();

        let serial_batch = ParallelFactorBatch::new(ParallelFactorConfig {
            parallelization_threshold: 10_000,
        });
        let parallel_batch = ParallelFactorBatch::new(ParallelFactorConfig {
            parallelization_threshold: 1,
        });

        let serial_factors = serial_batch.create_ba_factors_parallel(observations.clone());
        let parallel_factors = parallel_batch.create_ba_factors_parallel(observations);

        assert_eq!(serial_factors.len(), parallel_factors.len());

        // Evaluate at a known 3D landmark + camera pose
        let test_landmark = DVector::from_vec(vec![0.5, -0.1, 3.0]);
        let test_pose = DVector::from_vec(vec![0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]);
        for (i, (sf, pf)) in serial_factors.iter().zip(parallel_factors.iter()).enumerate() {
            let (r_s, j_s) = sf.linearize(&[test_landmark.clone(), test_pose.clone()], true);
            let (r_p, j_p) = pf.linearize(&[test_landmark.clone(), test_pose.clone()], true);

            assert_eq!(
                r_s.as_slice(),
                r_p.as_slice(),
                "BA residual mismatch at factor {i}"
            );
            let j_s = j_s.expect("serial Jacobian");
            let j_p = j_p.expect("parallel Jacobian");
            assert_eq!(
                j_s.as_slice(),
                j_p.as_slice(),
                "BA Jacobian mismatch at factor {i}"
            );
        }
    }

    #[test]
    fn test_generic_batch_processing_values() {
        let batch = ParallelFactorBatch::new(ParallelFactorConfig {
            parallelization_threshold: 1,
        });

        let numbers: Vec<i32> = (0..200).collect();
        let doubled = batch.process_batch_parallel(numbers, |x| x * 2);

        assert_eq!(doubled.len(), 200);
        for (i, &val) in doubled.iter().enumerate() {
            assert_eq!(val, (i as i32) * 2, "Mismatch at index {i}");
        }
    }

    #[cfg(feature = "benchmarks")]
    #[test]
    fn benchmark_parallel_vs_serial_factor_creation() {
        use std::time::Instant;
        let batch_parallel = ParallelFactorBatch::new(ParallelFactorConfig {
            parallelization_threshold: 16,
        });
        let batch_serial = ParallelFactorBatch::new(ParallelFactorConfig {
            parallelization_threshold: 10_000,
        });

        let observations = (0..10_000)
            .map(|i| {
                (
                    Vector2::new(i as f64 / 10_000.0, i as f64 / 10_000.0),
                    Matrix4::identity(),
                )
            })
            .collect::<Vec<_>>();

        let start_serial = Instant::now();
        let serial_factors = batch_serial.create_pinhole_factors_parallel(observations.clone());
        let serial_duration = start_serial.elapsed();

        let start_parallel = Instant::now();
        let parallel_factors = batch_parallel.create_pinhole_factors_parallel(observations);
        let parallel_duration = start_parallel.elapsed();

        assert_eq!(serial_factors.len(), 10_000);
        assert_eq!(parallel_factors.len(), 10_000);

        let serial_ms = serial_duration.as_secs_f64() * 1_000.0;
        let parallel_ms = parallel_duration.as_secs_f64() * 1_000.0;
        let speedup = if parallel_ms > 0.0 {
            serial_ms / parallel_ms
        } else {
            0.0
        };

        println!("serial:   {:?}", serial_duration);
        println!("parallel: {:?}", parallel_duration);
        println!(
            "benchmark_parallel_factors serial_ms={:.3} parallel_ms={:.3} speedup={:.3}",
            serial_ms, parallel_ms, speedup
        );
    }
}
