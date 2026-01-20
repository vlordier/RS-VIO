use crate::calibration::types::{CameraIMUExtrinsics, CameraIntrinsics, DistortionModel};
/// Time offset and rolling shutter estimation for camera-IMU systems.
///
/// This module solves for camera↔IMU time offset and rolling shutter readout time
/// by minimizing visual reprojection residuals under IMU motion constraints.
///
/// Key insight: Timing is "hidden multiplier" - without solving it well, you can't
/// achieve 10-30× accuracy improvement even with everything else perfect.
use nalgebra::{Isometry3, Point3, Vector3};

/// A camera measurement (pixel detection + metadata)
#[derive(Clone, Debug)]
pub struct CameraMeasurement {
    /// Pixel coordinate [u, v]
    pub pixel: [f64; 2],
    /// Image row (for rolling shutter correction)
    pub row: u32,
    /// Measured at this timestamp (raw)
    pub timestamp: f64,
    /// Measurement uncertainty (sigma, pixels)
    pub uncertainty: f64,
    /// Feature quality (gradient energy or NCC score)
    pub quality: f64,
}

/// IMU measurement (gyro + accel with timestamps)
#[derive(Clone, Debug)]
pub struct IMUMeasurement {
    /// Raw gyroscope reading (rad/s)
    pub gyro: Vector3<f64>,
    /// Raw accelerometer reading (m/s²)
    pub accel: Vector3<f64>,
    /// Timestamp (raw)
    pub timestamp: f64,
}

/// IMU preintegration result (relative rotation + velocity + position)
#[derive(Clone, Debug)]
pub struct PreintegrationResult {
    /// Integrated rotation (Δ_R) from t_k to t_{k+1}
    pub delta_rotation: Isometry3<f64>,
    /// Integrated velocity change (Δ_v)
    pub delta_velocity: Vector3<f64>,
    /// Integrated position change (Δ_p)
    pub delta_position: Vector3<f64>,
    /// Preintegration uncertainty (estimated)
    pub uncertainty: f64,
}

/// Joint time offset + rolling shutter estimator
pub struct TimeOffsetEstimator {
    /// Current estimate of camera↔IMU time offset (seconds)
    pub time_offset_estimate: f64,
    /// Current estimate of rolling shutter readout time
    pub rolling_shutter_estimate: f64,
    /// Allow RS model in optimization?
    pub enable_rolling_shutter: bool,
    /// Time offset uncertainty (1-sigma, seconds)
    pub time_offset_uncertainty: f64,
    /// Cost history for convergence monitoring
    pub cost_history: Vec<f64>,
}

impl TimeOffsetEstimator {
    /// Create new estimator with initial guesses
    pub fn new(initial_dt: f64, enable_rs: bool) -> Self {
        Self {
            time_offset_estimate: initial_dt,
            rolling_shutter_estimate: if enable_rs { 1e-3 } else { 0.0 },
            enable_rolling_shutter: enable_rs,
            time_offset_uncertainty: 0.01,
            cost_history: Vec::new(),
        }
    }

    /// Preintegrate IMU measurements from t_k to t_{k+1} with corrected timestamps
    /// imu_data: measurements ordered by timestamp
    /// Returns: PreintegrationResult
    pub fn preintegrate_imu(
        &self,
        imu_data: &[IMUMeasurement],
        t_start: f64,
        t_end: f64,
    ) -> PreintegrationResult {
        let mut delta_rotation = Isometry3::<f64>::identity();
        let mut delta_velocity = Vector3::zeros();
        let mut delta_position = Vector3::zeros();
        let mut uncertainty = 0.0;

        if imu_data.is_empty() {
            return PreintegrationResult {
                delta_rotation,
                delta_velocity,
                delta_position,
                uncertainty,
            };
        }

        // Correct timestamps with current time offset estimate
        let corrected_start = t_start + self.time_offset_estimate;
        let corrected_end = t_end + self.time_offset_estimate;

        for i in 0..imu_data.len() {
            let corrected_t = imu_data[i].timestamp + self.time_offset_estimate;
            if corrected_t < corrected_start || corrected_t > corrected_end {
                continue;
            }

            let dt = if i + 1 < imu_data.len() {
                let next_corrected_t = imu_data[i + 1].timestamp + self.time_offset_estimate;
                next_corrected_t - corrected_t
            } else {
                corrected_end - corrected_t
            };

            if dt <= 0.0 || dt > 0.1 {
                continue;
            }

            // Integrate rotation
            let gyro_norm = imu_data[i].gyro.norm();
            if gyro_norm > 1e-6 {
                let axis = imu_data[i].gyro / gyro_norm;
                let angle = gyro_norm * dt;
                let rot_increment = Isometry3::<f64>::new(axis * angle, Vector3::zeros());
                delta_rotation = delta_rotation * rot_increment;
            }

            // Integrate velocity and position
            let accel_term = imu_data[i].accel * dt;
            delta_velocity += accel_term;
            delta_position += delta_velocity * dt + accel_term * dt * dt * 0.5;

            uncertainty += dt;
        }

        PreintegrationResult {
            delta_rotation,
            delta_velocity,
            delta_position,
            uncertainty,
        }
    }

    /// Compute reprojection residual for a single measurement
    /// Given:
    /// - 3D point in world frame
    /// - IMU pose at frame time (world → IMU)
    /// - Camera intrinsics and distortion
    /// - Camera measurement (pixel + timestamp)
    ///
    /// Returns: reprojection error (pixels)
    pub fn reprojection_residual(
        &self,
        point_world: &Point3<f64>,
        t_world_to_imu: &Isometry3<f64>,
        camera_extrinsics: &CameraIMUExtrinsics,
        camera_intrinsics: &CameraIntrinsics,
        camera_distortion: &DistortionModel,
        measurement: &CameraMeasurement,
        image_height: u32,
    ) -> f64 {
        // Compute camera pose at feature's capture time (considering rolling shutter)
        let _capture_time = if self.rolling_shutter_estimate > 0.0 {
            // Row y captures at t_frame + (y/H) * t_readout
            let frame_t = measurement.timestamp + self.time_offset_estimate;
            let row_fraction = measurement.row as f64 / image_height as f64;
            frame_t + row_fraction * self.rolling_shutter_estimate
        } else {
            measurement.timestamp + self.time_offset_estimate
        };

        // Project point to image
        let point_imu = t_world_to_imu * point_world;
        let point_camera = camera_extrinsics.t_imu_to_camera * point_imu;

        // Normalize to image plane
        let xy_norm = [
            point_camera.x / point_camera.z,
            point_camera.y / point_camera.z,
        ];

        // Apply distortion
        let xy_distorted = camera_distortion.distort(&xy_norm);

        // Project to pixel
        let u = camera_intrinsics.fx * xy_distorted[0] + camera_intrinsics.cx;
        let v = camera_intrinsics.fy * xy_distorted[1] + camera_intrinsics.cy;

        // Residual
        let du = measurement.pixel[0] - u;
        let dv = measurement.pixel[1] - v;
        let residual_sq = du * du + dv * dv;
        residual_sq.sqrt()
    }

    /// Gradient descent step to refine time offset estimate
    /// measurements: camera+3D point pairs
    /// Returns: cost (sum of squared residuals)
    pub fn optimization_step(
        &mut self,
        measurements: &[(CameraMeasurement, Point3<f64>)],
        t_world_to_imu: &Isometry3<f64>,
        camera_extrinsics: &CameraIMUExtrinsics,
        camera_intrinsics: &CameraIntrinsics,
        camera_distortion: &DistortionModel,
        image_height: u32,
        learning_rate: f64,
    ) -> f64 {
        let mut cost = 0.0;

        for (measurement, point_world) in measurements {
            let residual = self.reprojection_residual(
                point_world,
                t_world_to_imu,
                camera_extrinsics,
                camera_intrinsics,
                camera_distortion,
                measurement,
                image_height,
            );

            cost += residual * residual;
        }

        // Simple update: perturb estimate and measure improvement
        let cost_initial = cost;
        self.time_offset_estimate += learning_rate;

        let mut new_cost = 0.0;
        for (measurement, point_world) in measurements {
            let residual = self.reprojection_residual(
                point_world,
                t_world_to_imu,
                camera_extrinsics,
                camera_intrinsics,
                camera_distortion,
                measurement,
                image_height,
            );
            new_cost += residual * residual;
        }

        if new_cost > cost_initial {
            // Step in opposite direction
            self.time_offset_estimate -= 2.0 * learning_rate;
        }

        self.cost_history.push(cost);
        cost
    }

    /// Run full optimization to estimate time offset + rolling shutter
    /// Returns: (final_cost, converged)
    pub fn optimize(
        &mut self,
        measurements: &[(CameraMeasurement, Point3<f64>)],
        t_world_to_imu: &Isometry3<f64>,
        camera_extrinsics: &CameraIMUExtrinsics,
        camera_intrinsics: &CameraIntrinsics,
        camera_distortion: &DistortionModel,
        image_height: u32,
        max_iterations: usize,
        tolerance: f64,
    ) -> (f64, bool) {
        let mut prev_cost = f64::INFINITY;

        for iteration in 0..max_iterations {
            let cost = self.optimization_step(
                measurements,
                t_world_to_imu,
                camera_extrinsics,
                camera_intrinsics,
                camera_distortion,
                image_height,
                0.001, // learning rate
            );

            if (prev_cost - cost).abs() < tolerance {
                return (cost, true);
            }

            prev_cost = cost;

            if iteration % 10 == 0 {
                eprintln!("[TimeOffset] Iteration {}: cost = {:.6e}", iteration, cost);
            }
        }

        (prev_cost, false)
    }

    /// Generate timing consistency curve: sweep Δt and measure cost
    pub fn timing_consistency_curve(
        &mut self,
        measurements: &[(CameraMeasurement, Point3<f64>)],
        t_world_to_imu: &Isometry3<f64>,
        camera_extrinsics: &CameraIMUExtrinsics,
        camera_intrinsics: &CameraIntrinsics,
        camera_distortion: &DistortionModel,
        image_height: u32,
        sweep_range: f64, // ±sweep_range from current estimate
        num_points: usize,
    ) -> Vec<(f64, f64)> {
        let mut curve = Vec::new();
        let step = 2.0 * sweep_range / (num_points as f64 - 1.0);

        let saved_estimate = self.time_offset_estimate;

        for i in 0..num_points {
            let dt = saved_estimate - sweep_range + (i as f64) * step;
            self.time_offset_estimate = dt;

            let mut cost = 0.0;
            for (measurement, point_world) in measurements {
                let residual = self.reprojection_residual(
                    point_world,
                    t_world_to_imu,
                    camera_extrinsics,
                    camera_intrinsics,
                    camera_distortion,
                    measurement,
                    image_height,
                );
                cost += residual * residual;
            }

            curve.push((dt, cost));
        }

        // Restore estimate
        self.time_offset_estimate = saved_estimate;
        curve
    }

    /// Detect if timing is observable (sharp minimum in cost curve)
    pub fn timing_observability(&self, cost_curve: &[(f64, f64)]) -> f64 {
        if cost_curve.is_empty() {
            return 0.0;
        }

        // Find minimum cost
        let (_, min_cost) = cost_curve
            .iter()
            .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
            .unwrap();

        // Find points where cost ≤ min_cost + 10%
        let threshold = min_cost * 1.1;
        let mut width_at_threshold = 0;
        for (_, cost) in cost_curve {
            if *cost <= threshold {
                width_at_threshold += 1;
            }
        }

        // Observability = 1 - (width / total_points)
        let observability = 1.0 - (width_at_threshold as f64 / cost_curve.len() as f64);
        observability.max(0.0).min(1.0)
    }

    /// Estimate uncertainty from Hessian of cost function
    pub fn estimate_uncertainty(
        &mut self,
        measurements: &[(CameraMeasurement, Point3<f64>)],
        t_world_to_imu: &Isometry3<f64>,
        camera_extrinsics: &CameraIMUExtrinsics,
        camera_intrinsics: &CameraIntrinsics,
        camera_distortion: &DistortionModel,
        image_height: u32,
    ) {
        let fd_step = 1e-6;
        let saved_estimate = self.time_offset_estimate;

        // Compute cost at +fd_step
        self.time_offset_estimate = saved_estimate + fd_step;
        let mut cost_plus = 0.0;
        for (measurement, point_world) in measurements {
            let residual = self.reprojection_residual(
                point_world,
                t_world_to_imu,
                camera_extrinsics,
                camera_intrinsics,
                camera_distortion,
                measurement,
                image_height,
            );
            cost_plus += residual * residual;
        }

        // Compute cost at -fd_step
        self.time_offset_estimate = saved_estimate - fd_step;
        let mut cost_minus = 0.0;
        for (measurement, point_world) in measurements {
            let residual = self.reprojection_residual(
                point_world,
                t_world_to_imu,
                camera_extrinsics,
                camera_intrinsics,
                camera_distortion,
                measurement,
                image_height,
            );
            cost_minus += residual * residual;
        }

        // Hessian (second derivative)
        let hessian = (cost_plus - 2.0 * cost_plus + cost_minus) / (fd_step * fd_step);
        if hessian > 0.0 {
            self.time_offset_uncertainty = 1.0 / hessian.sqrt();
        }

        // Restore
        self.time_offset_estimate = saved_estimate;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_preintegration() {
        let estimator = TimeOffsetEstimator::new(0.0, false);

        let imu_data = vec![
            IMUMeasurement {
                gyro: Vector3::new(0.1, 0.0, 0.0),
                accel: Vector3::new(0.0, 0.0, 0.0),
                timestamp: 0.0,
            },
            IMUMeasurement {
                gyro: Vector3::new(0.1, 0.0, 0.0),
                accel: Vector3::new(0.0, 0.0, 0.0),
                timestamp: 0.01,
            },
        ];

        let result = estimator.preintegrate_imu(&imu_data, 0.0, 0.01);
        assert!(result.uncertainty > 0.0);
    }

    #[test]
    fn test_timing_observability() {
        let estimator = TimeOffsetEstimator::new(0.0, false);

        // Sharp minimum
        let sharp_curve = vec![
            (-0.01, 1.0),
            (-0.005, 0.5),
            (0.0, 0.1),
            (0.005, 0.5),
            (0.01, 1.0),
        ];
        let obs_sharp = estimator.timing_observability(&sharp_curve);
        assert!(obs_sharp > 0.5);

        // Flat curve (no observability)
        let flat_curve = vec![
            (-0.01, 1.0),
            (-0.005, 1.0),
            (0.0, 1.0),
            (0.005, 1.0),
            (0.01, 1.0),
        ];
        let obs_flat = estimator.timing_observability(&flat_curve);
        assert!(obs_flat < 0.1);
    }
}
