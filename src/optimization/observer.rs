//! Optimization observer for convergence monitoring.

use apex_solver::core::problem::VariableEnum;
use apex_solver::observers::OptObserver;
use std::cell::RefCell;
use std::collections::HashMap;

/// Metrics collected during optimization iterations.
#[derive(Default, Clone)]
pub struct IterationMetrics {
    pub cost: Option<f64>,
    pub gradient_norm: Option<f64>,
    pub damping: Option<f64>,
    pub step_norm: Option<f64>,
    pub step_quality: Option<f64>,
}

/// Terminal observer that prints optimization progress to stdout.
///
/// This observer implements the `OptObserver` trait to monitor optimization
/// iterations and print metrics in a tab-separated format suitable for
/// terminal output or CSV logging.
pub struct TerminalObserver {
    iteration_metrics: RefCell<IterationMetrics>,
}

impl Default for TerminalObserver {
    fn default() -> Self {
        Self::new()
    }
}

impl TerminalObserver {
    /// Create a new terminal observer.
    pub fn new() -> Self {
        Self {
            iteration_metrics: RefCell::new(IterationMetrics::default()),
        }
    }

    /// Print header for the metrics output.
    pub fn print_header() {
        log::debug!("Iter\tCost\t\tGradNorm\tDamping\t\tStepNorm\tStepQuality");
        log::debug!("{}", "-".repeat(80));
    }
}

impl OptObserver for TerminalObserver {
    fn on_step(&self, _values: &HashMap<String, VariableEnum>, iteration: usize) {
        let metrics = self.iteration_metrics.borrow();
        let cost = metrics.cost.unwrap_or(0.0);
        let grad_norm = metrics.gradient_norm.unwrap_or(0.0);
        let damping = metrics.damping.unwrap_or(0.0);
        let step_norm = metrics.step_norm.unwrap_or(0.0);
        let step_quality = metrics.step_quality.unwrap_or(0.0);

        log::debug!(
            "{}\t{:.6e}\t{:.6e}\t{:.6e}\t{:.6e}\t{:.6e}",
            iteration,
            cost,
            grad_norm,
            damping,
            step_norm,
            step_quality
        );
    }

    fn set_iteration_metrics(
        &self,
        cost: f64,
        gradient_norm: f64,
        damping: Option<f64>,
        step_norm: f64,
        step_quality: Option<f64>,
    ) {
        let mut metrics = self.iteration_metrics.borrow_mut();
        metrics.cost = Some(cost);
        metrics.gradient_norm = Some(gradient_norm);
        metrics.damping = damping;
        metrics.step_norm = Some(step_norm);
        metrics.step_quality = step_quality;
    }
}
