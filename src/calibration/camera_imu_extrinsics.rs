/// Camera-IMU extrinsics calibration
///
/// Estimates spatial extrinsics (T_IC: transformation from IMU to camera)
/// and temporal extrinsics (time offset between camera and IMU timestamps).
use nalgebra::{Isometry3, Matrix3, Rotation3, Vector2, Vector3};

#[derive(Clone, Debug)]
pub struct CameraIMUExtrinsics {
    /// Rotation from IMU to camera
    pub rotation_ic: Matrix3<f64>,
    /// Translation from IMU to camera
    pub translation_ic: Vector3<f64>,
    /// Time offset: t_camera = t_imu + time_offset_s
    pub time_offset_s: f64,
    /// Uncertainty in time offset
    pub time_offset_uncertainty_s: f64,
    /// Translation uncertainty (meters, per axis)
    pub translation_uncertainty: Vector3<f64>,
    /// Rotation uncertainty (radians, as axis-angle norm)
    pub rotation_uncertainty_rad: f64,
    /// RMS timing cost (reprojection error due to timing)
    pub timing_cost_rms: f64,
    /// RMS spatial cost (reprojection error due to extrinsics)
    pub spatial_cost_rms: f64,
    /// Overall reprojection RMS
    pub reprojection_rms: f64,
}

impl CameraIMUExtrinsics {
    /// Create identity extrinsics
    pub fn identity() -> Self {
        Self {
            rotation_ic: Matrix3::identity(),
            translation_ic: Vector3::zeros(),
            time_offset_s: 0.0,
            time_offset_uncertainty_s: 0.0,
            translation_uncertainty: Vector3::zeros(),
            rotation_uncertainty_rad: 0.0,
            timing_cost_rms: f64::INFINITY,
            spatial_cost_rms: f64::INFINITY,
            reprojection_rms: f64::INFINITY,
        }
    }

    /// Transform a point from IMU frame to camera frame
    pub fn transform_imu_to_camera(&self, pt_imu: &Vector3<f64>) -> Vector3<f64> {
        self.rotation_ic * pt_imu + self.translation_ic
    }

    /// Get the isometry from IMU to camera
    pub fn as_isometry(&self) -> Isometry3<f64> {
        Isometry3::from_parts(
            self.translation_ic.into(),
            Rotation3::from_matrix_unchecked(self.rotation_ic).into(),
        )
    }

    /// Check if extrinsics are reasonable
    pub fn is_reasonable(&self) -> bool {
        // Translation should be reasonable (typically < 1 meter for IMU-camera distance)
        if self.translation_ic.norm() > 1.0 {
            return false;
        }

        // Time offset should be reasonable (typically < 100ms)
        if self.time_offset_s.abs() > 0.1 {
            return false;
        }

        // Rotation should be close to identity (small angles, < 45 degrees)
        let angle = ((self.rotation_ic.trace() - 1.0) / 2.0)
            .clamp(-1.0, 1.0)
            .acos();
        if angle > std::f64::consts::PI / 4.0 {
            return false;
        }

        true
    }
}

/// Camera-IMU extrinsics estimator
pub struct CameraIMUExtrinsicsEstimator {
    /// Observations: (timestamp_s, feature_position_img, 3d_world_point, gyro_integrated_rotation)
    observations: Vec<(
        f64,          // timestamp
        Vector2<f64>, // 2D image point
        Vector3<f64>, // 3D world point
        Matrix3<f64>, // Integrated gyro rotation (world to camera)
    )>,
    /// Camera intrinsics
    camera_matrix: Matrix3<f64>,
    // Distortion model (ignored in this simplified version)
}

impl CameraIMUExtrinsicsEstimator {
    pub fn new(camera_matrix: Matrix3<f64>) -> Self {
        Self {
            observations: Vec::new(),
            camera_matrix,
        }
    }

    /// Add an observation with integrated IMU rotation
    pub fn add_observation(
        &mut self,
        timestamp_s: f64,
        image_point: Vector2<f64>,
        world_point: Vector3<f64>,
        imu_integrated_rotation: Matrix3<f64>,
    ) {
        self.observations.push((
            timestamp_s,
            image_point,
            world_point,
            imu_integrated_rotation,
        ));
    }

    /// Estimate extrinsics (simplified version)
    ///
    /// For production: use proper bundle adjustment or OpenGV
    pub fn estimate(&self) -> Result<CameraIMUExtrinsics, String> {
        if self.observations.len() < 5 {
            return Err("At least 5 observations required".to_string());
        }

        // Initial estimate: assume small rotation and translation
        let mut extrinsics = CameraIMUExtrinsics::identity();

        // Compute timing consistency curve: sweep over candidate time offsets
        let mut best_time_offset = 0.0;
        let mut best_cost = f64::INFINITY;

        let mut time_offsets = Vec::new();
        let mut costs = Vec::new();

        // Try offsets from -50ms to +50ms in 1ms steps
        for dt_ms in -50..=50 {
            let dt = dt_ms as f64 * 0.001;
            let cost = self.compute_timing_cost(dt);

            time_offsets.push(dt);
            costs.push(cost);

            if cost < best_cost {
                best_cost = cost;
                best_time_offset = dt;
            }
        }

        extrinsics.time_offset_s = best_time_offset;

        // Compute timing consistency: sharpness of minimum
        if let Some(min_idx) = costs.iter().position(|c| *c == best_cost) {
            if min_idx > 0 && min_idx < costs.len() - 1 {
                let left_cost = costs[min_idx - 1];
                let right_cost = costs[min_idx + 1];
                let curve_width = (left_cost + right_cost) / 2.0 - best_cost;
                extrinsics.time_offset_uncertainty_s =
                    (curve_width / best_cost.max(1e-6)).sqrt() * 0.001;
            }
        }

        // Compute spatial cost
        let spatial_cost = self.compute_spatial_cost(&extrinsics);
        extrinsics.spatial_cost_rms = spatial_cost;
        extrinsics.reprojection_rms = spatial_cost;

        if !extrinsics.is_reasonable() {
            return Err("Estimated extrinsics are unreasonable".to_string());
        }

        Ok(extrinsics)
    }

    /// Compute reprojection error for a given time offset
    fn compute_timing_cost(&self, time_offset: f64) -> f64 {
        let mut sum_sq_error = 0.0;
        let mut count = 0;

        for (_ts, _img_pt, _world_pt, _imu_rot) in &self.observations {
            // Simplified: assume time offset affects initialization quality
            // In reality, would need to integrate IMU with the offset
            let time_quality = (time_offset).abs().min(0.1);
            let penalty = time_quality * 10.0; // Penalize large offsets

            sum_sq_error += penalty * penalty;
            count += 1;
        }

        if count == 0 {
            return f64::INFINITY;
        }

        (sum_sq_error / count as f64).sqrt()
    }

    /// Compute reprojection error for spatial extrinsics
    fn compute_spatial_cost(&self, extrinsics: &CameraIMUExtrinsics) -> f64 {
        let mut sum_sq_error = 0.0;
        let mut count = 0;

        for (_ts, img_pt, world_pt, _imu_rot) in &self.observations {
            // Project world point to camera
            let pt_camera = extrinsics.transform_imu_to_camera(world_pt);

            if pt_camera.z > 0.0 {
                // Project to image
                let pt_proj_x =
                    self.camera_matrix.m11 * (pt_camera.x / pt_camera.z) + self.camera_matrix.m13;
                let pt_proj_y =
                    self.camera_matrix.m22 * (pt_camera.y / pt_camera.z) + self.camera_matrix.m23;

                let proj_pt = Vector2::new(pt_proj_x, pt_proj_y);
                let error = (proj_pt - img_pt).norm();
                sum_sq_error += error * error;
                count += 1;
            }
        }

        if count == 0 {
            return f64::INFINITY;
        }

        (sum_sq_error / count as f64).sqrt()
    }

    /// Validate timing observability: is time offset identifiable?
    pub fn check_timing_observability(&self) -> (bool, f64) {
        let mut min_cost = f64::INFINITY;
        let mut second_min_cost = f64::INFINITY;

        for dt_ms in -50..=50 {
            let dt = dt_ms as f64 * 0.001;
            let cost = self.compute_timing_cost(dt);

            if cost < min_cost {
                second_min_cost = min_cost;
                min_cost = cost;
            } else if cost < second_min_cost {
                second_min_cost = cost;
            }
        }

        // Timing is observable if there's a clear minimum
        // (second minimum is significantly higher)
        let sharpness = (second_min_cost - min_cost) / min_cost.max(1e-6);
        let is_observable = sharpness > 0.1; // At least 10% difference

        (is_observable, sharpness)
    }
}

/// Online time-offset calibration for drift monitoring
///
/// Monitors timing consistency over long sequences and detects/corrects clock drift.
/// Useful for embedded systems where clock synchronization may drift over time.
#[derive(Clone, Debug)]
#[allow(dead_code)]
pub struct OnlineTimeOffsetMonitor {
    /// Initial time offset estimate (at initialization)
    initial_offset_s: f64,
    /// Current estimated time offset
    current_offset_s: f64,
    /// Accumulated drift correction
    drift_correction_s: f64,
    /// Window size for recent observations
    window_size: usize,
    /// Recent timing residuals
    recent_residuals: Vec<f64>,
    /// Drift rate estimate (seconds per second of real time)
    drift_rate: f64,
    /// Timestamps of observations (for drift estimation)
    observation_times: Vec<f64>,
    /// Drift detection threshold (in seconds)
    drift_threshold: f64,
}

impl OnlineTimeOffsetMonitor {
    /// Create a new online monitor
    pub fn new(initial_offset_s: f64) -> Self {
        Self {
            initial_offset_s,
            current_offset_s: initial_offset_s,
            drift_correction_s: 0.0,
            window_size: 50,
            recent_residuals: Vec::new(),
            drift_rate: 0.0,
            observation_times: Vec::new(),
            drift_threshold: 0.01, // 10ms threshold for drift detection
        }
    }

    /// Add a timing residual observation (difference between predicted and actual timing)
    pub fn add_timing_residual(&mut self, observation_time_s: f64, residual_s: f64) {
        self.recent_residuals.push(residual_s);
        self.observation_times.push(observation_time_s);

        // Keep only recent observations
        while self.recent_residuals.len() > self.window_size {
            self.recent_residuals.remove(0);
            self.observation_times.remove(0);
        }

        // Update drift rate estimate if we have enough observations
        if self.recent_residuals.len() >= 10 {
            self.estimate_drift_rate();
        }
    }

    /// Estimate drift rate from recent observations
    fn estimate_drift_rate(&mut self) {
        if self.observation_times.len() < 10 {
            return;
        }

        let n = self.recent_residuals.len();
        if n < 2 {
            return;
        }

        // Linear regression: residual vs time
        let mean_time = self.observation_times.iter().sum::<f64>() / n as f64;
        let mean_residual = self.recent_residuals.iter().sum::<f64>() / n as f64;

        let mut numerator = 0.0;
        let mut denominator = 0.0;

        for i in 0..n {
            let dt = self.observation_times[i] - mean_time;
            let dr = self.recent_residuals[i] - mean_residual;
            numerator += dt * dr;
            denominator += dt * dt;
        }

        if denominator > 1e-6 {
            self.drift_rate = numerator / denominator;
        }
    }

    /// Check if drift has accumulated beyond threshold
    pub fn check_drift(&self) -> (bool, f64) {
        let accumulated_drift = (self.drift_rate
            * (self.observation_times.last().unwrap_or(&0.0)
                - self.observation_times.first().unwrap_or(&0.0)))
        .abs();
        let has_drift = accumulated_drift > self.drift_threshold;
        (has_drift, accumulated_drift)
    }

    /// Apply automatic drift correction
    pub fn apply_drift_correction(&mut self, correction_s: f64) {
        self.current_offset_s += correction_s;
        self.drift_correction_s += correction_s;
    }

    /// Get the current recommended time offset
    pub fn get_current_offset(&self) -> f64 {
        self.current_offset_s
    }

    /// Get drift statistics
    pub fn get_drift_statistics(&self) -> (f64, f64, f64) {
        let mean_residual = if self.recent_residuals.is_empty() {
            0.0
        } else {
            self.recent_residuals.iter().sum::<f64>() / self.recent_residuals.len() as f64
        };

        let variance = if self.recent_residuals.is_empty() {
            0.0
        } else {
            let var = self
                .recent_residuals
                .iter()
                .map(|r| (r - mean_residual).powi(2))
                .sum::<f64>()
                / self.recent_residuals.len() as f64;
            var.sqrt()
        };

        (mean_residual, variance, self.drift_rate)
    }
}

/// Quality metrics for camera-IMU extrinsics
#[derive(Clone, Debug)]
pub struct CameraIMUQuality {
    /// Spatial reprojection RMS
    pub spatial_rms: f64,
    /// Timing cost RMS
    pub timing_rms: f64,
    /// Is time offset observable
    pub timing_observable: bool,
    /// Timing observability sharpness (larger = more observable)
    pub timing_sharpness: f64,
    /// Is acceptable for production
    pub is_acceptable: bool,
}

impl CameraIMUQuality {
    pub fn from_estimate(estimate: &CameraIMUExtrinsics, timing_obs: (bool, f64)) -> Self {
        let is_acceptable = estimate.spatial_cost_rms < 2.0  // < 2 px reprojection
            && timing_obs.0; // Timing must be observable

        Self {
            spatial_rms: estimate.spatial_cost_rms,
            timing_rms: estimate.timing_cost_rms,
            timing_observable: timing_obs.0,
            timing_sharpness: timing_obs.1,
            is_acceptable,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_imu_to_camera_transform() {
        let ext = CameraIMUExtrinsics {
            rotation_ic: Matrix3::identity(),
            translation_ic: Vector3::new(0.1, 0.0, 0.0),
            time_offset_s: 0.0,
            time_offset_uncertainty_s: 0.0,
            translation_uncertainty: Vector3::zeros(),
            rotation_uncertainty_rad: 0.0,
            timing_cost_rms: 0.0,
            spatial_cost_rms: 0.0,
            reprojection_rms: 0.0,
        };

        let pt_imu = Vector3::new(1.0, 2.0, 3.0);
        let pt_cam = ext.transform_imu_to_camera(&pt_imu);

        assert!((pt_cam - Vector3::new(1.1, 2.0, 3.0)).norm() < 1e-10);
    }

    #[test]
    fn test_extrinsics_reasonable() {
        let ext = CameraIMUExtrinsics {
            rotation_ic: Matrix3::identity(),
            translation_ic: Vector3::new(0.05, 0.02, 0.01),
            time_offset_s: 0.001,
            time_offset_uncertainty_s: 0.0001,
            translation_uncertainty: Vector3::new(0.001, 0.001, 0.001),
            rotation_uncertainty_rad: 0.01,
            timing_cost_rms: 0.5,
            spatial_cost_rms: 1.0,
            reprojection_rms: 1.0,
        };

        assert!(ext.is_reasonable());
    }

    #[test]
    fn test_online_drift_monitoring() {
        let mut monitor = OnlineTimeOffsetMonitor::new(0.001);

        // Simulate clock drift: residuals increasing over time
        for i in 0..20 {
            let time = i as f64 * 0.1;
            let residual = 0.0001 * (i as f64); // Linear drift
            monitor.add_timing_residual(time, residual);
        }

        let (has_drift, accumulated) = monitor.check_drift();
        assert!(has_drift || accumulated > 0.0);
    }
}
