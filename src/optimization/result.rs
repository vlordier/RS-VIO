/// Optimization result containing refined parameters
///
/// This structure holds the results from bundle adjustment optimization,
/// including refined biases that can be fed back to the ESKF for improved
/// high-rate IMU prediction.
#[derive(Debug, Clone)]
pub struct OptimizationResult {
    /// Refined gyroscope bias [rad/s]
    pub gyro_bias: nalgebra::Vector3<f64>,

    /// Refined accelerometer bias [m/s²]
    pub accel_bias: nalgebra::Vector3<f64>,

    /// Bias uncertainty (standard deviation)
    pub bias_uncertainty: f64,

    /// Number of iterations performed
    pub iterations: usize,

    /// Final cost/objective value
    pub final_cost: f64,

    /// Whether optimization converged
    pub converged: bool,
}

impl OptimizationResult {
    /// Create new optimization result
    pub const fn new(
        gyro_bias: nalgebra::Vector3<f64>,
        accel_bias: nalgebra::Vector3<f64>,
        bias_uncertainty: f64,
    ) -> Self {
        Self {
            gyro_bias,
            accel_bias,
            bias_uncertainty,
            iterations: 0,
            final_cost: 0.0,
            converged: false,
        }
    }

    /// Create with full metadata
    pub const fn with_metadata(
        gyro_bias: nalgebra::Vector3<f64>,
        accel_bias: nalgebra::Vector3<f64>,
        bias_uncertainty: f64,
        iterations: usize,
        final_cost: f64,
        converged: bool,
    ) -> Self {
        Self {
            gyro_bias,
            accel_bias,
            bias_uncertainty,
            iterations,
            final_cost,
            converged,
        }
    }
}

#[cfg(test)]
#[allow(clippy::float_cmp)]
mod tests {
    use super::*;
    use nalgebra::Vector3;

    #[test]
    fn test_optimization_result_creation() {
        let bg = Vector3::new(0.01, -0.02, 0.005);
        let ba = Vector3::new(0.1, 0.05, -0.08);

        let result = OptimizationResult::new(bg, ba, 0.001);

        assert_eq!(result.gyro_bias, bg);
        assert_eq!(result.accel_bias, ba);
        assert_eq!(result.bias_uncertainty, 0.001);
        assert!(!result.converged);
    }

    #[test]
    fn test_optimization_result_with_metadata() {
        let bg = Vector3::new(0.01, -0.02, 0.005);
        let ba = Vector3::new(0.1, 0.05, -0.08);

        let result = OptimizationResult::with_metadata(bg, ba, 0.001, 10, 1.5, true);

        assert_eq!(result.iterations, 10);
        assert_eq!(result.final_cost, 1.5);
        assert!(result.converged);
    }
}
