//! IMU self-calibration module for bias, scale, and noise estimation.
//!
//! This module implements manual, ground-based IMU calibration following
//! the six-pose method. Operator places drone in known orientations and
//! the system estimates bias vectors, scale matrices, and noise characteristics.

use crate::types::Float;
use nalgebra as na;
use serde::{Deserialize, Serialize};

/// IMU calibration configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ImuCalibrationConfig {
    /// Number of samples to collect per pose (default: 1000 @ 200Hz = 5 sec)
    pub samples_per_pose: usize,
    /// Number of poses to collect (default: 6 for full 6-DoF)
    pub num_poses: usize,
    /// Expected gravity magnitude in m/s² (default: 9.81)
    pub gravity_magnitude: Float,
    /// Maximum acceptable gyro drift per pose (rad/s, default: 0.01)
    pub max_gyro_drift: Float,
    /// Maximum acceptable accel drift per pose (m/s², default: 0.1)
    pub max_accel_drift: Float,
    /// Gyro clipping threshold (rad/s, default: 8.0 for typical MEMS)
    pub gyro_clip_threshold: Float,
    /// Accel clipping threshold (m/s², default: 150.0 for typical MEMS)
    pub accel_clip_threshold: Float,
}

impl Default for ImuCalibrationConfig {
    fn default() -> Self {
        Self {
            samples_per_pose: 1000,
            num_poses: 6,
            gravity_magnitude: 9.81,
            max_gyro_drift: 0.01,
            max_accel_drift: 0.1,
            gyro_clip_threshold: 8.0,
            accel_clip_threshold: 150.0,
        }
    }
}

/// Single IMU measurement for calibration
#[derive(Debug, Clone)]
pub struct ImuCalibrationSample {
    pub timestamp_ns: i64,
    pub gyro: na::Vector3<Float>,
    pub accel: na::Vector3<Float>,
}

/// Calibration result for a single pose
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PoseCalibrationResult {
    /// Pose identifier (0-5 for six-pose method)
    pub pose_id: usize,
    /// Mean gyro measurement over pose duration
    pub gyro_mean: [Float; 3],
    /// Mean accel measurement over pose duration
    pub accel_mean: [Float; 3],
    /// Gyro standard deviation (noise metric)
    pub gyro_std: [Float; 3],
    /// Accel standard deviation (noise metric)
    pub accel_std: [Float; 3],
    /// Number of samples collected
    pub sample_count: usize,
    /// Number of gyro samples clipped
    pub gyro_clipped_count: usize,
    /// Number of accel samples clipped
    pub accel_clipped_count: usize,
    /// Whether this pose passed quality checks
    pub quality_passed: bool,
}

/// Complete IMU calibration result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ImuCalibrationResult {
    /// Timestamp of calibration completion
    pub timestamp_ns: i64,
    /// Gyro bias estimate (rad/s)
    pub gyro_bias: [Float; 3],
    /// Accel bias estimate (m/s²)
    pub accel_bias: [Float; 3],
    /// Gyro scale matrix (3x3, row-major)
    pub gyro_scale: [Float; 9],
    /// Accel scale matrix (3x3, row-major)
    pub accel_scale: [Float; 9],
    /// Gyro noise density (rad/s/√Hz)
    pub gyro_noise_density: [Float; 3],
    /// Accel noise density (m/s²/√Hz)
    pub accel_noise_density: [Float; 3],
    /// Per-pose results
    pub pose_results: Vec<PoseCalibrationResult>,
    /// Overall calibration quality score (0-1)
    pub quality_score: Float,
    /// Whether all quality gates passed
    pub passed: bool,
    /// Human-readable quality report
    pub quality_report: String,
}

/// IMU calibration state machine
#[derive(Debug)]
#[allow(clippy::large_enum_variant)]
pub enum ImuCalibrationState {
    Idle,
    CollectingPose {
        pose_id: usize,
        samples: Vec<ImuCalibrationSample>,
    },
    AwaitingNextPose {
        pose_id: usize,
    },
    Computing,
    Complete(ImuCalibrationResult),
    Failed(String),
}

/// IMU calibration engine
#[derive(Debug)]
pub struct ImuCalibrator {
    config: ImuCalibrationConfig,
    state: ImuCalibrationState,
    pose_results: Vec<PoseCalibrationResult>,
}

impl ImuCalibrator {
    /// Create new calibrator with default configuration
    pub fn new() -> Self {
        Self::with_config(ImuCalibrationConfig::default())
    }

    /// Create new calibrator with custom configuration
    pub fn with_config(config: ImuCalibrationConfig) -> Self {
        Self {
            config,
            state: ImuCalibrationState::Idle,
            pose_results: Vec::new(),
        }
    }

    /// Start calibration sequence
    pub fn start(&mut self) {
        self.state = ImuCalibrationState::AwaitingNextPose { pose_id: 0 };
        self.pose_results.clear();
    }

    /// Begin collecting samples for a specific pose
    pub fn begin_pose(&mut self, pose_id: usize) -> Result<(), String> {
        match &self.state {
            ImuCalibrationState::AwaitingNextPose {
                pose_id: expected_id,
            } => {
                if pose_id != *expected_id {
                    return Err(format!("Expected pose {}, got {}", expected_id, pose_id));
                }
                self.state = ImuCalibrationState::CollectingPose {
                    pose_id,
                    samples: Vec::with_capacity(self.config.samples_per_pose),
                };
                Ok(())
            },
            _ => Err(format!("Cannot begin pose in state: {:?}", self.state)),
        }
    }

    /// Add IMU sample to current pose collection
    pub fn add_sample(&mut self, sample: ImuCalibrationSample) -> Result<(), String> {
        // Extract samples to avoid borrow checker issues
        let (pose_id, samples) = match std::mem::replace(&mut self.state, ImuCalibrationState::Idle)
        {
            ImuCalibrationState::CollectingPose {
                pose_id,
                mut samples,
            } => {
                samples.push(sample);
                (pose_id, samples)
            },
            other => {
                self.state = other;
                return Err("Not in sample collection state".to_string());
            },
        };

        // Check if we've collected enough samples
        if samples.len() >= self.config.samples_per_pose {
            let pose_result = self.compute_pose_statistics(pose_id, &samples)?;
            let quality_passed = pose_result.quality_passed;
            self.pose_results.push(pose_result);

            if !quality_passed {
                self.state =
                    ImuCalibrationState::Failed(format!("Pose {} failed quality checks", pose_id));
                return Err(format!("Pose {} failed quality checks", pose_id));
            }

            // Move to next pose or compute final result
            if pose_id + 1 >= self.config.num_poses {
                self.state = ImuCalibrationState::Computing;
                self.compute_calibration()?;
            } else {
                self.state = ImuCalibrationState::AwaitingNextPose {
                    pose_id: pose_id + 1,
                };
            }
        } else {
            // Still collecting samples
            self.state = ImuCalibrationState::CollectingPose { pose_id, samples };
        }

        Ok(())
    }

    /// Compute statistics for a completed pose
    fn compute_pose_statistics(
        &self,
        pose_id: usize,
        samples: &[ImuCalibrationSample],
    ) -> Result<PoseCalibrationResult, String> {
        if samples.is_empty() {
            return Err("No samples collected for pose".to_string());
        }

        // Compute means
        let mut gyro_sum = na::Vector3::<Float>::zeros();
        let mut accel_sum = na::Vector3::<Float>::zeros();

        for sample in samples {
            gyro_sum += sample.gyro;
            accel_sum += sample.accel;
        }

        let n = samples.len() as Float;
        let gyro_mean = gyro_sum / n;
        let accel_mean = accel_sum / n;

        // Compute standard deviations
        let mut gyro_var = na::Vector3::<Float>::zeros();
        let mut accel_var = na::Vector3::<Float>::zeros();

        for sample in samples {
            let gyro_diff = sample.gyro - gyro_mean;
            let accel_diff = sample.accel - accel_mean;
            gyro_var += gyro_diff.component_mul(&gyro_diff);
            accel_var += accel_diff.component_mul(&accel_diff);
        }

        gyro_var /= n - 1.0;
        accel_var /= n - 1.0;

        let gyro_std = gyro_var.map(|x| x.sqrt());
        let accel_std = accel_var.map(|x| x.sqrt());

        // Count clipped samples
        let mut gyro_clipped_count = 0;
        let mut accel_clipped_count = 0;

        for sample in samples {
            if sample
                .gyro
                .iter()
                .any(|&x| x.abs() > self.config.gyro_clip_threshold)
            {
                gyro_clipped_count += 1;
            }
            if sample
                .accel
                .iter()
                .any(|&x| x.abs() > self.config.accel_clip_threshold)
            {
                accel_clipped_count += 1;
            }
        }

        // Quality checks
        let gyro_drift_ok = gyro_std.iter().all(|&x| x < self.config.max_gyro_drift);
        let accel_drift_ok = accel_std.iter().all(|&x| x < self.config.max_accel_drift);
        let no_clipping = gyro_clipped_count == 0 && accel_clipped_count == 0;
        let quality_passed = gyro_drift_ok && accel_drift_ok && no_clipping;

        Ok(PoseCalibrationResult {
            pose_id,
            gyro_mean: [gyro_mean[0], gyro_mean[1], gyro_mean[2]],
            accel_mean: [accel_mean[0], accel_mean[1], accel_mean[2]],
            gyro_std: [gyro_std[0], gyro_std[1], gyro_std[2]],
            accel_std: [accel_std[0], accel_std[1], accel_std[2]],
            sample_count: samples.len(),
            gyro_clipped_count,
            accel_clipped_count,
            quality_passed,
        })
    }

    /// Compute final calibration from all poses
    fn compute_calibration(&mut self) -> Result<(), String> {
        if self.pose_results.len() < self.config.num_poses {
            return Err(format!(
                "Insufficient poses: {} < {}",
                self.pose_results.len(),
                self.config.num_poses
            ));
        }

        // Compute gyro bias (mean of all poses, should be near zero)
        let mut gyro_bias_sum = na::Vector3::<Float>::zeros();
        for pose in &self.pose_results {
            gyro_bias_sum += na::Vector3::from_row_slice(&pose.gyro_mean);
        }
        let gyro_bias = gyro_bias_sum / self.pose_results.len() as Float;

        // Compute accel bias and scale via least-squares
        // Expected: ||accel|| = g for each pose
        // Build linear system: accel_measured = S * accel_true + b
        let (accel_bias, accel_scale) = self.estimate_accel_calibration()?;

        // Gyro scale (assume identity for now; advanced method requires turntable)
        let gyro_scale = na::Matrix3::<Float>::identity();

        // Noise density (average std across poses)
        let mut gyro_noise_sum = na::Vector3::<Float>::zeros();
        let mut accel_noise_sum = na::Vector3::<Float>::zeros();

        for pose in &self.pose_results {
            gyro_noise_sum += na::Vector3::from_row_slice(&pose.gyro_std);
            accel_noise_sum += na::Vector3::from_row_slice(&pose.accel_std);
        }

        let gyro_noise_density = gyro_noise_sum / self.pose_results.len() as Float;
        let accel_noise_density = accel_noise_sum / self.pose_results.len() as Float;

        // Quality score (1.0 if all poses passed, lower if issues)
        let passed_count = self
            .pose_results
            .iter()
            .filter(|p| p.quality_passed)
            .count();
        let quality_score = passed_count as Float / self.pose_results.len() as Float;

        // Generate quality report
        let quality_report = self.generate_quality_report(
            &gyro_bias,
            &accel_bias,
            &gyro_noise_density,
            &accel_noise_density,
            quality_score,
        );

        let result = ImuCalibrationResult {
            timestamp_ns: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos() as i64,
            gyro_bias: [gyro_bias[0], gyro_bias[1], gyro_bias[2]],
            accel_bias: [accel_bias[0], accel_bias[1], accel_bias[2]],
            gyro_scale: [
                gyro_scale[(0, 0)],
                gyro_scale[(0, 1)],
                gyro_scale[(0, 2)],
                gyro_scale[(1, 0)],
                gyro_scale[(1, 1)],
                gyro_scale[(1, 2)],
                gyro_scale[(2, 0)],
                gyro_scale[(2, 1)],
                gyro_scale[(2, 2)],
            ],
            accel_scale: [
                accel_scale[(0, 0)],
                accel_scale[(0, 1)],
                accel_scale[(0, 2)],
                accel_scale[(1, 0)],
                accel_scale[(1, 1)],
                accel_scale[(1, 2)],
                accel_scale[(2, 0)],
                accel_scale[(2, 1)],
                accel_scale[(2, 2)],
            ],
            gyro_noise_density: [
                gyro_noise_density[0],
                gyro_noise_density[1],
                gyro_noise_density[2],
            ],
            accel_noise_density: [
                accel_noise_density[0],
                accel_noise_density[1],
                accel_noise_density[2],
            ],
            pose_results: self.pose_results.clone(),
            quality_score,
            passed: quality_score >= 0.9,
            quality_report,
        };

        self.state = ImuCalibrationState::Complete(result);
        Ok(())
    }

    /// Estimate accel bias and scale matrix
    fn estimate_accel_calibration(
        &self,
    ) -> Result<(na::Vector3<Float>, na::Matrix3<Float>), String> {
        // For simplicity: compute bias as mean deviation from expected gravity direction
        // Full calibration requires known orientations (6-pose method)

        // Simple approach: assume poses include at least one face-down (z = -g)
        // and compute bias + scale via least-squares

        let g = self.config.gravity_magnitude;

        // Compute mean accel across all poses
        let mut accel_sum = na::Vector3::<Float>::zeros();
        for pose in &self.pose_results {
            accel_sum += na::Vector3::from_row_slice(&pose.accel_mean);
        }
        let accel_bias = accel_sum / self.pose_results.len() as Float;

        // For scale: compute magnitude error relative to g
        // Scale matrix: assume diagonal (no cross-axis for now)
        let mut scale_diag = na::Vector3::<Float>::from_element(1.0);

        for (i, pose) in self.pose_results.iter().enumerate() {
            let accel = na::Vector3::from_row_slice(&pose.accel_mean) - accel_bias;
            let mag = accel.norm();
            if mag > 0.1 {
                let scale_estimate = g / mag;
                scale_diag[i % 3] = (scale_diag[i % 3] + scale_estimate) / 2.0;
            }
        }

        let accel_scale = na::Matrix3::from_diagonal(&scale_diag);

        Ok((accel_bias, accel_scale))
    }

    /// Generate human-readable quality report
    fn generate_quality_report(
        &self,
        gyro_bias: &na::Vector3<Float>,
        accel_bias: &na::Vector3<Float>,
        gyro_noise: &na::Vector3<Float>,
        accel_noise: &na::Vector3<Float>,
        quality_score: Float,
    ) -> String {
        let mut report = String::new();
        report.push_str("IMU Calibration Quality Report\n");
        report.push_str("==============================\n\n");

        report.push_str(&format!(
            "Gyro Bias:  [{:.6}, {:.6}, {:.6}] rad/s\n",
            gyro_bias[0], gyro_bias[1], gyro_bias[2]
        ));
        report.push_str(&format!(
            "Accel Bias: [{:.6}, {:.6}, {:.6}] m/s²\n\n",
            accel_bias[0], accel_bias[1], accel_bias[2]
        ));

        report.push_str(&format!(
            "Gyro Noise:  [{:.6}, {:.6}, {:.6}] rad/s/√Hz\n",
            gyro_noise[0], gyro_noise[1], gyro_noise[2]
        ));
        report.push_str(&format!(
            "Accel Noise: [{:.6}, {:.6}, {:.6}] m/s²/√Hz\n\n",
            accel_noise[0], accel_noise[1], accel_noise[2]
        ));

        report.push_str(&format!("Quality Score: {:.2}%\n", quality_score * 100.0));
        report.push_str(&format!(
            "Status: {}\n",
            if quality_score >= 0.9 {
                "PASSED"
            } else {
                "FAILED"
            }
        ));

        report.push_str("\nPer-Pose Results:\n");
        for pose in &self.pose_results {
            report.push_str(&format!(
                "  Pose {}: {} samples, {} gyro clips, {} accel clips - {}\n",
                pose.pose_id,
                pose.sample_count,
                pose.gyro_clipped_count,
                pose.accel_clipped_count,
                if pose.quality_passed { "PASS" } else { "FAIL" }
            ));
        }

        report
    }

    /// Get current calibration state
    pub fn state(&self) -> &ImuCalibrationState {
        &self.state
    }

    /// Get calibration result (if complete)
    pub fn result(&self) -> Option<&ImuCalibrationResult> {
        match &self.state {
            ImuCalibrationState::Complete(result) => Some(result),
            _ => None,
        }
    }

    /// Reset calibration to idle state
    pub fn reset(&mut self) {
        self.state = ImuCalibrationState::Idle;
        self.pose_results.clear();
    }
}

impl Default for ImuCalibrator {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_imu_calibrator_basic_flow() {
        let mut calibrator = ImuCalibrator::new();

        // Start calibration
        calibrator.start();
        assert!(matches!(
            calibrator.state(),
            ImuCalibrationState::AwaitingNextPose { pose_id: 0 }
        ));

        // Begin first pose
        assert!(calibrator.begin_pose(0).is_ok());
        assert!(matches!(
            calibrator.state(),
            ImuCalibrationState::CollectingPose { pose_id: 0, .. }
        ));
    }

    #[test]
    fn test_pose_statistics_computation() {
        let calibrator = ImuCalibrator::new();

        // Generate mock samples with known statistics
        let samples: Vec<ImuCalibrationSample> = (0..1000)
            .map(|i| ImuCalibrationSample {
                timestamp_ns: i * 1_000_000,
                gyro: na::Vector3::new(0.001, -0.002, 0.0015),
                accel: na::Vector3::new(0.0, 0.0, -9.81),
            })
            .collect();

        let result = calibrator.compute_pose_statistics(0, &samples).unwrap();

        assert_eq!(result.sample_count, 1000);
        assert!(result.gyro_mean[0].abs() - 0.001 < 0.0001);
        assert!(result.accel_mean[2].abs() - 9.81 < 0.01);
        assert_eq!(result.gyro_clipped_count, 0);
        assert_eq!(result.accel_clipped_count, 0);
    }

    #[test]
    fn test_clipping_detection() {
        let config = ImuCalibrationConfig {
            gyro_clip_threshold: 1.0,
            accel_clip_threshold: 20.0,
            ..Default::default()
        };
        let calibrator = ImuCalibrator::with_config(config);

        let samples = vec![
            ImuCalibrationSample {
                timestamp_ns: 0,
                gyro: na::Vector3::new(0.5, 0.5, 0.5),
                accel: na::Vector3::new(0.0, 0.0, -9.81),
            },
            ImuCalibrationSample {
                timestamp_ns: 1_000_000,
                gyro: na::Vector3::new(2.0, 0.5, 0.5), // Clipped
                accel: na::Vector3::new(0.0, 0.0, -9.81),
            },
            ImuCalibrationSample {
                timestamp_ns: 2_000_000,
                gyro: na::Vector3::new(0.5, 0.5, 0.5),
                accel: na::Vector3::new(0.0, 0.0, -25.0), // Clipped
            },
        ];

        let result = calibrator.compute_pose_statistics(0, &samples).unwrap();

        assert_eq!(result.gyro_clipped_count, 1);
        assert_eq!(result.accel_clipped_count, 1);
        assert!(!result.quality_passed);
    }
}
