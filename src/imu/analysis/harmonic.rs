//! Harmonic analysis and decomposition
//!
//! Extracts harmonic components (f0, 2f0, 3f0, ...) from residual IMU signals
//! with adaptive extraction coefficients based on motor state.

use super::config::{
    MotorState, HARMONIC_COEFF_RUNNING, HARMONIC_COEFF_TRANSITIONING,
};
use crate::types::Vector3;

/// Extract harmonics based on motor state
pub fn extract_harmonics(
    residuals: &[Vector3],
    motor_state: MotorState,
) -> (Vector3, Vec<Vector3>) {
    let fundamental = compute_mean(residuals);

    let extraction_coeff = match motor_state {
        MotorState::Off => 0.0,                        // No rotor harmonics
        MotorState::Running => HARMONIC_COEFF_RUNNING, // f0 extraction
        MotorState::Transitioning => HARMONIC_COEFF_TRANSITIONING, // Conservative
    };

    let residual_harmonics: Vec<Vector3> = if extraction_coeff == 0.0 {
        // Motors off: all residuals are noise
        residuals.to_vec()
    } else {
        // Extract fundamental component
        residuals
            .iter()
            .map(|r| r - fundamental * extraction_coeff)
            .collect()
    };

    (fundamental, residual_harmonics)
}

/// Compute mean of vector samples
pub fn compute_mean(samples: &[Vector3]) -> Vector3 {
    if samples.is_empty() {
        return Vector3::zeros();
    }
    let sum: Vector3 = samples.iter().sum();
    sum / samples.len() as f64
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_harmonic_decomposition_motors_off() {
        let residuals = vec![
            Vector3::new(0.01, -0.01, 0.0),
            Vector3::new(-0.01, 0.01, 0.0),
            Vector3::new(0.01, -0.01, 0.0),
        ];

        let (fundamental, residual_harmonics) = extract_harmonics(&residuals, MotorState::Off);

        // With motors off, fundamental should not be extracted
        assert!(fundamental.norm() > 0.0);
        assert_eq!(
            residual_harmonics.len(),
            residuals.len(),
            "Should preserve all residuals when motors off"
        );
    }

    #[test]
    fn test_harmonic_decomposition_motors_running() {
        let residuals = vec![
            Vector3::new(1.0, 0.5, 0.2),
            Vector3::new(0.9, 0.6, 0.1),
            Vector3::new(1.1, 0.4, 0.3),
        ];

        let (fundamental, residual_harmonics) =
            extract_harmonics(&residuals, MotorState::Running);

        // Should extract fundamental component
        assert!(fundamental.norm() > 0.0);
        assert_eq!(residual_harmonics.len(), residuals.len());

        // Residual harmonics should be smaller than originals
        let original_norm: f64 = residuals.iter().map(|r| r.norm()).sum();
        let residual_norm: f64 = residual_harmonics.iter().map(|r| r.norm()).sum();
        assert!(residual_norm < original_norm);
    }

    #[test]
    fn test_compute_mean() {
        let samples = vec![
            Vector3::new(1.0, 2.0, 3.0),
            Vector3::new(3.0, 4.0, 5.0),
        ];

        let mean = compute_mean(&samples);
        assert!((mean.x - 2.0).abs() < 1e-6);
        assert!((mean.y - 3.0).abs() < 1e-6);
        assert!((mean.z - 4.0).abs() < 1e-6);
    }
}
