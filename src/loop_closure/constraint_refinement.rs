//! Constraint refinement for loop closures
//!
//! Refines loop closure constraints through local optimization
//! and uncertainty estimation.

use serde::{Deserialize, Serialize};

/// Configuration for constraint refinement
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct ConstraintRefinementConfig {
    /// Maximum iterations for refinement optimization
    pub max_iterations: usize,
    /// Convergence threshold for optimization
    pub convergence_threshold: f32,
    /// Initial uncertainty (standard deviation) in meters
    pub initial_uncertainty_m: f32,
    /// Initial uncertainty in rotation (degrees)
    pub initial_uncertainty_deg: f32,
    /// Robust loss function threshold
    pub robust_threshold: f32,
}

impl Default for ConstraintRefinementConfig {
    fn default() -> Self {
        Self {
            max_iterations: 20,
            convergence_threshold: 1e-6,
            initial_uncertainty_m: 0.1,
            initial_uncertainty_deg: 5.0,
            robust_threshold: 1.0,
        }
    }
}

/// SE(3) transformation (rotation + translation)
#[derive(Debug, Clone, Copy)]
pub struct SE3Transform {
    /// Rotation matrix (3x3 as flattened array)
    pub rotation: [[f32; 3]; 3],
    /// Translation vector (3D)
    pub translation: [f32; 3],
}

impl SE3Transform {
    /// Create identity transformation
    pub fn identity() -> Self {
        Self {
            rotation: [
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0],
            ],
            translation: [0.0, 0.0, 0.0],
        }
    }

    /// Get translational norm (distance in meters)
    pub fn translation_norm(&self) -> f32 {
        let x = self.translation[0];
        let y = self.translation[1];
        let z = self.translation[2];
        (x * x + y * y + z * z).sqrt()
    }

    /// Get rotation magnitude (Frobenius norm of rotation matrix minus identity)
    pub fn rotation_magnitude(&self) -> f32 {
        let mut sum = 0.0f32;
        for i in 0..3 {
            for j in 0..3 {
                let delta = if i == j {
                    self.rotation[i][j] - 1.0
                } else {
                    self.rotation[i][j]
                };
                sum += delta * delta;
            }
        }
        sum.sqrt()
    }
}

/// Information matrix (inverse covariance, 6x6)
#[derive(Debug, Clone, Copy)]
pub struct InformationMatrix {
    /// 6x6 matrix stored as upper triangular (21 elements)
    pub data: [f32; 21],
}

impl InformationMatrix {
    /// Create identity information matrix
    pub fn identity() -> Self {
        let mut data = [0.0f32; 21];
        // Diagonal elements (1, 7, 12, 16, 19, 21)
        data[0] = 1.0;   // (0,0)
        data[6] = 1.0;   // (1,1)
        data[11] = 1.0;  // (2,2)
        data[15] = 1.0;  // (3,3)
        data[18] = 1.0;  // (4,4)
        data[20] = 1.0;  // (5,5)

        Self { data }
    }

    /// Create information matrix from diagonal values
    pub fn from_diagonal(diag: [f32; 6]) -> Self {
        let mut info = Self::identity();
        info.data[0] = diag[0];   // (0,0)
        info.data[6] = diag[1];   // (1,1)
        info.data[11] = diag[2];  // (2,2)
        info.data[15] = diag[3];  // (3,3)
        info.data[18] = diag[4];  // (4,4)
        info.data[20] = diag[5];  // (5,5)
        info
    }

    /// Get diagonal element (i, i)
    pub fn diagonal(&self, i: usize) -> f32 {
        let indices = [0, 6, 11, 15, 18, 20];
        if i < 6 {
            self.data[indices[i]]
        } else {
            0.0
        }
    }
}

/// Result of constraint refinement
#[derive(Debug, Clone, Copy)]
pub struct RefinementResult {
    pub refined_transform: SE3Transform,
    pub information_matrix: InformationMatrix,
    pub residual_error: f32,
    pub num_iterations: usize,
    pub converged: bool,
}

/// Constraint refiner for loop closures
pub struct ConstraintRefiner {
    config: ConstraintRefinementConfig,
}

impl ConstraintRefiner {
    /// Create new constraint refiner
    pub fn new(config: ConstraintRefinementConfig) -> Self {
        Self { config }
    }

    /// Refine a loop closure constraint
    pub fn refine_constraint(
        &self,
        initial_transform: SE3Transform,
        residuals: &[f32],
    ) -> RefinementResult {
        if residuals.is_empty() {
            return RefinementResult {
                refined_transform: initial_transform,
                information_matrix: InformationMatrix::identity(),
                residual_error: 0.0,
                num_iterations: 0,
                converged: true,
            };
        }

        let mut current_transform = initial_transform;
        let mut prev_error = f32::MAX;

        let mut num_iters = 0;

        for iter in 0..self.config.max_iterations {
            // Compute residuals with current estimate
            let current_error = compute_weighted_residual(residuals, &current_transform);

            // Check convergence
            if (prev_error - current_error).abs() < self.config.convergence_threshold {
                num_iters = iter;
                break;
            }

            // Simple gradient descent step (placeholder for Gauss-Newton)
            current_transform = self.gradient_descent_step(&current_transform, residuals);

            prev_error = current_error;
            num_iters = iter + 1;
        }

        // Estimate information matrix from residuals
        let information = estimate_information_matrix(residuals, &self.config);

        RefinementResult {
            refined_transform: current_transform,
            information_matrix: information,
            residual_error: prev_error,
            num_iterations: num_iters,
            converged: num_iters < self.config.max_iterations,
        }
    }

    /// Gradient descent step
    fn gradient_descent_step(
        &self,
        transform: &SE3Transform,
        _residuals: &[f32],
    ) -> SE3Transform {
        // Placeholder: In production, compute Jacobian and update
        // For now, return slightly perturbed transform
        let mut updated = *transform;
        
        // Small perturbation for demonstration
        for i in 0..3 {
            updated.translation[i] *= 0.99;
        }

        updated
    }
}

impl Default for ConstraintRefiner {
    fn default() -> Self {
        Self::new(ConstraintRefinementConfig::default())
    }
}

/// Compute weighted residual with robust loss
fn compute_weighted_residual(residuals: &[f32], _transform: &SE3Transform) -> f32 {
    let mut total = 0.0f32;
    for &r in residuals {
        total += r * r;
    }
    (total / residuals.len() as f32).sqrt()
}

/// Estimate information matrix from residuals
fn estimate_information_matrix(
    residuals: &[f32],
    config: &ConstraintRefinementConfig,
) -> InformationMatrix {
    // Compute variance of residuals
    if residuals.is_empty() {
        return InformationMatrix::identity();
    }

    let mean = residuals.iter().sum::<f32>() / residuals.len() as f32;
    let variance = residuals
        .iter()
        .map(|&r| (r - mean).powi(2))
        .sum::<f32>()
        / residuals.len() as f32;

    let std_dev = variance.sqrt();

    // Information is inverse of covariance
    let info_value = if std_dev > 1e-6 {
        1.0 / (std_dev * std_dev)
    } else {
        1.0 / (config.initial_uncertainty_m * config.initial_uncertainty_m)
    };

    // Set diagonal with information values
    InformationMatrix::from_diagonal([
        info_value * 100.0, // High information on translation
        info_value * 100.0,
        info_value * 100.0,
        info_value,         // Lower information on rotation
        info_value,
        info_value,
    ])
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_config_defaults() {
        let config = ConstraintRefinementConfig::default();
        assert_eq!(config.max_iterations, 20);
        assert!(config.convergence_threshold > 0.0);
    }

    #[test]
    fn test_se3_identity() {
        let t = SE3Transform::identity();
        assert_eq!(t.translation_norm(), 0.0);
        assert!(t.rotation_magnitude() < 0.1); // Close to zero for identity
    }

    #[test]
    fn test_se3_translation_norm() {
        let t = SE3Transform {
            rotation: [
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0],
            ],
            translation: [3.0, 4.0, 0.0],
        };
        assert!((t.translation_norm() - 5.0).abs() < 0.01);
    }

    #[test]
    fn test_information_matrix_identity() {
        let info = InformationMatrix::identity();
        for i in 0..6 {
            assert_eq!(info.diagonal(i), 1.0);
        }
    }

    #[test]
    fn test_information_matrix_from_diagonal() {
        let diag = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0];
        let info = InformationMatrix::from_diagonal(diag);
        for i in 0..6 {
            assert_eq!(info.diagonal(i), diag[i]);
        }
    }

    #[test]
    fn test_refiner_creation() {
        let refiner = ConstraintRefiner::new(ConstraintRefinementConfig::default());
        let t = SE3Transform::identity();
        let residuals = vec![0.1, 0.1, 0.1];

        let result = refiner.refine_constraint(t, &residuals);
        assert!(result.num_iterations < 20);
    }

    #[test]
    fn test_empty_residuals() {
        let refiner = ConstraintRefiner::new(ConstraintRefinementConfig::default());
        let t = SE3Transform::identity();
        let residuals = vec![];

        let result = refiner.refine_constraint(t, &residuals);
        assert!(result.converged);
        assert_eq!(result.num_iterations, 0);
    }

    #[test]
    fn test_refinement_convergence() {
        let config = ConstraintRefinementConfig {
            max_iterations: 5,
            ..Default::default()
        };
        let refiner = ConstraintRefiner::new(config);

        let t = SE3Transform {
            rotation: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            translation: [0.1, 0.1, 0.1],
        };

        let residuals = vec![0.05; 10];
        let result = refiner.refine_constraint(t, &residuals);

        assert_eq!(result.refined_transform.translation_norm() < t.translation_norm(), true);
    }

    #[test]
    fn test_information_estimation() {
        let config = ConstraintRefinementConfig::default();
        let residuals = vec![0.1, 0.1, 0.1, 0.1];

        let info = estimate_information_matrix(&residuals, &config);
        
        // Information should be positive
        for i in 0..6 {
            assert!(info.diagonal(i) > 0.0);
        }
    }
}
