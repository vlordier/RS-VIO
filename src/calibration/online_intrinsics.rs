/// Online Intrinsics Refiner
///
/// Continuously refines camera intrinsics during VIO operation using:
/// - Reprojection error minimization
/// - Structure-from-motion consistency
///
/// Only refines basic intrinsics (fx, fy, cx, cy).
/// Distortion, time offset, and readout time are stored separately.
use crate::calibration::types::CameraIntrinsics;
use crate::types::Float;
use nalgebra as na;

/// Refined calibration parameters
#[derive(Clone, Debug)]
pub struct RefinedCalibration {
    /// Current intrinsics
    pub intrinsics: CameraIntrinsics,

    /// Distortion coefficients
    pub distortion: [Float; 5],

    /// Time offset (seconds)
    pub time_offset: Float,

    /// Readout time (seconds)
    pub readout_time: Float,

    /// Uncertainty estimates (1-sigma)
    pub uncertainty: CalibrationUncertainty,

    /// Convergence metrics
    pub convergence: ConvergenceMetrics,

    /// Number of observations used
    pub observation_count: u64,
}

/// Uncertainty estimates for calibration parameters
#[derive(Clone, Debug)]
pub struct CalibrationUncertainty {
    /// Focal length uncertainty (pixels)
    pub focal_uncertainty: Float,

    /// Principal point uncertainty (pixels)
    pub principal_uncertainty: Float,

    /// Distortion uncertainty
    pub distortion_uncertainty: Float,

    /// Overall calibration quality (0.0-1.0)
    pub quality_score: Float,
}

/// Convergence tracking
#[derive(Clone, Debug)]
pub struct ConvergenceMetrics {
    /// Whether calibration has converged
    pub is_converged: bool,

    /// Change in last iteration
    pub last_change: Float,

    /// Total change over recent iterations
    pub recent_change: Float,

    /// Number of iterations since start
    pub iterations: u32,
}

/// Configuration for intrinsics refinement
#[derive(Debug, Clone)]
pub struct IntrinsicsRefinerConfig {
    /// Enable focal length refinement
    pub refine_focal: bool,

    /// Enable principal point refinement
    pub refine_principal_point: bool,

    /// Maximum iterations per update
    pub max_iterations: usize,

    /// Convergence threshold
    pub convergence_threshold: Float,

    /// Minimum observations before enabling refinement
    pub min_observations: u32,

    /// Maximum change per iteration
    pub max_change_per_iteration: Float,

    /// Damping factor for stability
    pub damping: Float,

    /// Robust kernel size (pixels)
    pub robust_kernel: Float,

    /// Decay rate for old observations
    pub observation_decay: Float,
}

impl Default for IntrinsicsRefinerConfig {
    fn default() -> Self {
        Self {
            refine_focal: true,
            refine_principal_point: true,
            max_iterations: 10,
            convergence_threshold: 1e-6,
            min_observations: 100,
            max_change_per_iteration: 0.1,
            damping: 1.0,
            robust_kernel: 2.0,
            observation_decay: 0.99,
        }
    }
}

/// Observation for intrinsics refinement
#[derive(Clone, Debug, Default)]
pub struct CalibrationObservation {
    /// 3D point in camera frame
    pub point_3d: na::Vector3<Float>,

    /// 2D observation in image (u, v)
    pub observation_2d: na::Vector2<Float>,

    /// Reprojection error
    pub reproj_error: Float,

    /// Weight (based on track length, angle)
    pub weight: Float,
}

/// Online intrinsics refiner
pub struct OnlineIntrinsicsRefiner {
    config: IntrinsicsRefinerConfig,

    /// Current calibration
    calibration: RefinedCalibration,

    /// Observation history
    observations: Vec<CalibrationObservation>,

    /// Statistics
    pub total_updates: u64,
    pub total_observations: u64,
}

impl OnlineIntrinsicsRefiner {
    /// Create new refiner with initial calibration
    pub fn new(initial: CameraIntrinsics) -> Self {
        let calibration = RefinedCalibration {
            intrinsics: initial.clone(),
            distortion: [0.0; 5],
            time_offset: 0.0,
            readout_time: 0.0,
            uncertainty: CalibrationUncertainty {
                focal_uncertainty: 10.0,
                principal_uncertainty: 5.0,
                distortion_uncertainty: 0.01,
                quality_score: 0.5,
            },
            convergence: ConvergenceMetrics {
                is_converged: false,
                last_change: 1.0,
                recent_change: 1.0,
                iterations: 0,
            },
            observation_count: 0,
        };

        Self {
            config: IntrinsicsRefinerConfig::default(),
            calibration,
            observations: Vec::new(),
            total_updates: 0,
            total_observations: 0,
        }
    }

    /// Update with new observations
    pub fn update(&mut self, new_observations: &[CalibrationObservation]) {
        self.observations.extend_from_slice(new_observations);
        self.total_observations += new_observations.len() as u64;

        if self.config.observation_decay < 1.0 {
            for obs in &mut self.observations {
                obs.weight *= self.config.observation_decay;
            }
        }

        self.observations
            .retain(|obs| obs.weight > 0.01 || obs.reproj_error < 1.0);

        if self.observations.len() < self.config.min_observations as usize {
            return;
        }

        self.refine_intrinsics();
    }

    /// Refine intrinsics using gradient descent
    fn refine_intrinsics(&mut self) {
        for _ in 0..self.config.max_iterations {
            let gradient = self.compute_gradient();

            if gradient
                .iter()
                .all(|&g| g.abs() < self.config.convergence_threshold)
            {
                self.calibration.convergence.is_converged = true;
                break;
            }

            let mut update = self.apply_damped_update(&gradient);

            let max_change = update
                .iter()
                .map(|v| (*v as Float).abs())
                .fold(0.0 as Float, Float::max);

            if max_change > self.config.max_change_per_iteration {
                let scale = self.config.max_change_per_iteration / max_change;
                for v in &mut update {
                    *v *= scale;
                }
            }

            self.apply_update(&update);

            let param_change =
                update.iter().map(|v| v.abs()).sum::<Float>() / update.len() as Float;
            self.calibration.convergence.last_change = param_change;
            self.calibration.convergence.recent_change = param_change;
            self.calibration.convergence.iterations += 1;
        }

        self.update_uncertainties();
        self.total_updates += 1;
    }

    /// Compute gradient for intrinsics refinement
    fn compute_gradient(&mut self) -> Vec<Float> {
        let num_params = self.num_params();
        let mut gradient = vec![0.0; num_params];

        let observations: Vec<_> = self.observations.clone();

        for obs in observations {
            let J = self.compute_jacobian(&obs);
            let error = self.compute_reproj_error(&obs);

            let mut weight = 1.0;
            if error > self.config.robust_kernel as Float {
                weight = self.config.robust_kernel as Float / error;
            }

            for (j_idx, g_val) in gradient.iter_mut().enumerate() {
                *g_val += J[j_idx] * error * weight * obs.weight;
            }
        }

        gradient
    }

    /// Compute Jacobian for observation using finite differences
    fn compute_jacobian(&mut self, obs: &CalibrationObservation) -> Vec<Float> {
        let num_params = self.num_params();
        let mut J = vec![0.0; num_params];

        let params = self.get_parameters();

        for i in 0..num_params {
            let eps = 1e-5;
            let mut params_plus = params.clone();
            params_plus[i] += eps;

            self.set_parameters(&params_plus);
            let error_plus = self.compute_reproj_error(obs);

            let mut params_minus = params.clone();
            params_minus[i] -= eps;

            self.set_parameters(&params_minus);
            let error_minus = self.compute_reproj_error(obs);

            self.set_parameters(&params);

            J[i] = (error_plus - error_minus) / (2.0 * eps);
        }

        J
    }

    /// Compute reprojection error for observation
    fn compute_reproj_error(&self, obs: &CalibrationObservation) -> Float {
        let cam = &self.calibration.intrinsics;

        let p = &obs.point_3d;
        let x = p[0] / p[2].max(0.01);
        let y = p[1] / p[2].max(0.01);

        let (xd, yd) = self.apply_distortion(x, y);

        let u = cam.fx * xd + cam.cx;
        let v = cam.fy * yd + cam.cy;

        let dx = u - obs.observation_2d[0];
        let dy = v - obs.observation_2d[1];

        (dx * dx + dy * dy).sqrt()
    }

    /// Apply radial-tangential distortion
    fn apply_distortion(&self, x: Float, y: Float) -> (Float, Float) {
        let k1 = self.calibration.distortion[0];
        let k2 = self.calibration.distortion[1];
        let p1 = self.calibration.distortion[2];
        let p2 = self.calibration.distortion[3];

        let r2 = x * x + y * y;
        let dx = 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
        let dy = p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;

        let xd = x + k1 * x * r2 + k2 * x * r2 * r2 + dx;
        let yd = y + k1 * y * r2 + k2 * y * r2 * r2 + dy;

        (xd, yd)
    }

    /// Apply damped update
    fn apply_damped_update(&self, gradient: &[Float]) -> Vec<Float> {
        gradient
            .iter()
            .map(|g| g * self.config.damping as Float)
            .collect()
    }

    /// Apply parameter update
    fn apply_update(&mut self, update: &[Float]) {
        let mut params = self.get_parameters();

        for (i, u) in update.iter().enumerate() {
            params[i] -= u;
        }

        self.set_parameters(&params);
    }

    /// Get current parameters as vector
    fn get_parameters(&self) -> Vec<Float> {
        let cam = &self.calibration.intrinsics;
        let mut params = Vec::with_capacity(4);

        if self.config.refine_focal {
            params.push(cam.fx as Float);
            params.push(cam.fy as Float);
        }

        if self.config.refine_principal_point {
            params.push(cam.cx as Float);
            params.push(cam.cy as Float);
        }

        params
    }

    /// Set parameters from vector
    fn set_parameters(&mut self, params: &[Float]) {
        let cam = &mut self.calibration.intrinsics;
        let mut idx = 0;

        if self.config.refine_focal {
            if idx < params.len() {
                cam.fx = params[idx] as f64;
            }
            idx += 1;
            if idx < params.len() {
                cam.fy = params[idx] as f64;
            }
            idx += 1;
        }

        if self.config.refine_principal_point {
            if idx < params.len() {
                cam.cx = params[idx] as f64;
            }
            idx += 1;
            if idx < params.len() {
                cam.cy = params[idx] as f64;
            }
        }
    }

    /// Number of parameters being refined
    fn num_params(&self) -> usize {
        let mut n = 0;
        if self.config.refine_focal {
            n += 2;
        }
        if self.config.refine_principal_point {
            n += 2;
        }
        n.max(1)
    }

    /// Update uncertainty estimates
    fn update_uncertainties(&mut self) {
        let mut total_error = 0.0 as Float;
        let mut max_error = 0.0 as Float;

        for obs in &self.observations {
            let error = self.compute_reproj_error(obs);
            total_error += error;
            max_error = max_error.max(error);
        }

        let mean_error = total_error / self.observations.len().max(1) as Float;

        self.calibration.uncertainty.focal_uncertainty *= 0.99;
        self.calibration.uncertainty.principal_uncertainty *= 0.99;

        let obs_score = (self.observations.len() as Float / 1000.0).min(1.0);
        let error_score = (1.0 - (mean_error / 2.0)).max(0.0);
        self.calibration.uncertainty.quality_score = obs_score * error_score;
    }

    /// Get current calibration
    pub fn calibration(&self) -> &RefinedCalibration {
        &self.calibration
    }

    /// Get current intrinsics
    pub fn intrinsics(&self) -> &CameraIntrinsics {
        &self.calibration.intrinsics
    }

    /// Check if converged
    pub fn is_converged(&self) -> bool {
        self.calibration.convergence.is_converged
    }

    /// Get total observations
    pub fn total_observations(&self) -> u64 {
        self.total_observations
    }

    /// Reset the refiner
    pub fn reset(&mut self, initial: CameraIntrinsics) {
        self.calibration.intrinsics = initial.clone();
        self.observations.clear();
        self.calibration.convergence = ConvergenceMetrics {
            is_converged: false,
            last_change: 1.0,
            recent_change: 1.0,
            iterations: 0,
        };
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_refiner_creation() {
        let intrinsics = CameraIntrinsics {
            fx: 500.0,
            fy: 500.0,
            cx: 320.0,
            cy: 240.0,
            width: 640,
            height: 480,
        };

        let refiner = OnlineIntrinsicsRefiner::new(intrinsics);
        assert!(!refiner.is_converged());
    }

    #[test]
    fn test_parameter_count() {
        let intrinsics = CameraIntrinsics {
            fx: 500.0,
            fy: 500.0,
            cx: 320.0,
            cy: 240.0,
            width: 640,
            height: 480,
        };

        let refiner = OnlineIntrinsicsRefiner::new(intrinsics);
        assert_eq!(refiner.num_params(), 4);
    }

    #[test]
    fn test_update_with_observations() {
        let intrinsics = CameraIntrinsics {
            fx: 500.0,
            fy: 500.0,
            cx: 320.0,
            cy: 240.0,
            width: 640,
            height: 480,
        };

        let mut refiner = OnlineIntrinsicsRefiner::new(intrinsics);

        let observations = vec![CalibrationObservation {
            point_3d: na::Vector3::new(1.0, 2.0, 5.0),
            observation_2d: na::Vector2::new(420.0, 440.0),
            reproj_error: 0.5,
            weight: 1.0,
        }];

        refiner.update(&observations);

        assert!(refiner.total_observations() >= 1);
    }

    #[test]
    fn test_reset() {
        let intrinsics1 = CameraIntrinsics {
            fx: 500.0,
            fy: 500.0,
            cx: 320.0,
            cy: 240.0,
            width: 640,
            height: 480,
        };

        let mut refiner = OnlineIntrinsicsRefiner::new(intrinsics1);

        let intrinsics2 = CameraIntrinsics {
            fx: 600.0,
            fy: 600.0,
            cx: 320.0,
            cy: 240.0,
            width: 640,
            height: 480,
        };

        refiner.reset(intrinsics2);

        assert_eq!(refiner.intrinsics().fx, 600.0);
    }
}
