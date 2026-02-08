//! Optimization result types for bundle adjustment output.

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


