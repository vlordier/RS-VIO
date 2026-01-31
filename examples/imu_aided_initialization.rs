//! Phase 8A: IMU-Aided Initialization with Metric Scale
//!
//! Implements gravity-aligned initialization using IMU data to establish
//! proper metric scale in the VIO system.

use nalgebra as na;
use rs_vio::datasets::ImuData;
use std::time::Instant;

/// Configuration for IMU-aided initialization
#[derive(Debug, Clone)]
pub struct ImuInitConfig {
    /// Minimum static IMU samples needed for initialization (typically 1-2 seconds)
    pub min_static_samples: usize,
    /// Accelerometer variance threshold for "static" detection (m²/s⁴)
    pub static_accel_variance_threshold: f64,
    /// Gravity magnitude (m/s²)
    pub gravity_magnitude: f64,
    /// Maximum gravity direction drift (radians)
    pub max_gravity_drift: f64,
}

impl Default for ImuInitConfig {
    fn default() -> Self {
        Self {
            min_static_samples: 200, // ~1 second @ 200 Hz
            static_accel_variance_threshold: 0.01,
            gravity_magnitude: 9.81,
            max_gravity_drift: 0.1, // ~6 degrees
        }
    }
}

/// IMU-aided initialization state machine
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InitializationState {
    /// Waiting for static initialization period
    WaitingForStatic,
    /// Collecting static samples to estimate gravity
    Initializing,
    /// Initialization complete, gravity estimated
    Complete,
    /// Initialization failed or aborted
    Failed,
}

/// Gravity estimation result
#[derive(Debug, Clone)]
pub struct GravityEstimate {
    /// Gravity vector in world frame (m/s²)
    pub gravity: na::Vector3<f64>,
    /// Confidence in gravity estimate (0-1)
    pub confidence: f64,
    /// Number of samples used
    pub num_samples: usize,
    /// Estimated bias of accelerometer (m/s²)
    pub accel_bias: na::Vector3<f64>,
}

/// IMU-aided initializer for metric scale
pub struct ImuAidedInitializer {
    config: ImuInitConfig,
    state: InitializationState,

    // Static period detection
    accel_samples: Vec<na::Vector3<f64>>,
    gyro_samples: Vec<na::Vector3<f64>>,
    timestamps: Vec<i64>,

    // Gravity estimation
    gravity_estimate: na::Vector3<f64>,
    last_accel_variance: f64,
    static_frame_count: usize,

    // Scale estimation
    #[allow(dead_code)]
    first_keyframe_scale: Option<f64>,
}

impl ImuAidedInitializer {
    /// Create new initializer
    pub const fn new(config: ImuInitConfig) -> Self {
        Self {
            config,
            state: InitializationState::WaitingForStatic,
            accel_samples: Vec::new(),
            gyro_samples: Vec::new(),
            timestamps: Vec::new(),
            gravity_estimate: na::Vector3::new(0.0, 0.0, -9.81),
            last_accel_variance: f64::MAX,
            static_frame_count: 0,
            first_keyframe_scale: None,
        }
    }

    /// Process IMU measurement
    pub fn process_imu(&mut self, imu: &ImuData) {
        let accel = na::Vector3::new(imu.accel[0], imu.accel[1], imu.accel[2]);
        let gyro = na::Vector3::new(imu.gyro[0], imu.gyro[1], imu.gyro[2]);

        self.accel_samples.push(accel);
        self.gyro_samples.push(gyro);
        self.timestamps.push(imu.timestamp);

        match self.state {
            InitializationState::WaitingForStatic => {
                self.detect_static_period();
            },
            InitializationState::Initializing => {
                self.update_gravity_estimate();
            },
            _ => {},
        }
    }

    /// Detect if device is in a static period
    fn detect_static_period(&mut self) {
        if self.accel_samples.len() < 20 {
            return; // Need at least ~100ms of data
        }

        // Compute accelerometer variance over recent samples
        let recent_start = (self.accel_samples.len() - 20).max(0);
        let recent_accel = &self.accel_samples[recent_start..];

        let mean_accel = recent_accel.iter().sum::<na::Vector3<f64>>() / recent_accel.len() as f64;
        let variance = recent_accel
            .iter()
            .map(|a| (a - mean_accel).norm_squared())
            .sum::<f64>()
            / recent_accel.len() as f64;

        self.last_accel_variance = variance;

        // Check if static
        if variance < self.config.static_accel_variance_threshold {
            self.static_frame_count += 1;

            // Enough samples for initialization?
            if self.static_frame_count >= self.config.min_static_samples {
                self.state = InitializationState::Initializing;
                self.update_gravity_estimate();
            }
        } else {
            self.static_frame_count = 0;
            self.accel_samples.clear();
            self.gyro_samples.clear();
            self.timestamps.clear();
        }
    }

    /// Update gravity estimate from accelerometer samples
    fn update_gravity_estimate(&mut self) {
        if self.accel_samples.len() < self.config.min_static_samples {
            return;
        }

        // Use recent samples for gravity (better convergence)
        let start = (self.accel_samples.len() - self.config.min_static_samples).max(0);
        let accel_window = &self.accel_samples[start..];

        // Compute mean acceleration
        let mean_accel = accel_window.iter().sum::<na::Vector3<f64>>() / accel_window.len() as f64;

        // Gravity points opposite to acceleration bias
        // In static case: a = g + bias → gravity = -mean_accel (normalized)
        let accel_norm = mean_accel.norm();
        if accel_norm > 0.1 {
            self.gravity_estimate = -mean_accel / accel_norm * self.config.gravity_magnitude;
            self.state = InitializationState::Complete;
        }
    }

    /// Get current gravity estimate
    pub const fn gravity(&self) -> na::Vector3<f64> {
        self.gravity_estimate
    }

    /// Get initialization state
    pub const fn state(&self) -> InitializationState {
        self.state
    }

    /// Check if gravity has converged
    pub fn is_gravity_converged(&self) -> bool {
        self.state == InitializationState::Complete
    }

    /// Get accelerometer bias estimate
    pub fn accel_bias(&self) -> na::Vector3<f64> {
        if self.accel_samples.is_empty() {
            return na::Vector3::zeros();
        }

        let mean_accel =
            self.accel_samples.iter().sum::<na::Vector3<f64>>() / self.accel_samples.len() as f64;
        mean_accel - self.gravity_estimate
    }

    /// Estimate metric scale from visual + IMU constraint
    ///
    /// After first keyframe: compare visual-only depth scale with
    /// IMU velocity integration to recover metric scale
    pub fn estimate_metric_scale(
        &mut self,
        depth_scale_visual: f64,
        imu_velocity_magnitude: f64,
    ) -> f64 {
        // Scale = IMU velocity / visual velocity estimate
        // This recovers metric scale from the monocular ambiguity
        if imu_velocity_magnitude > 0.01 {
            imu_velocity_magnitude / depth_scale_visual.max(0.01)
        } else {
            1.0 // No IMU velocity, use unit scale
        }
    }

    /// Get complete initialization report
    pub fn report(&self) -> GravityEstimate {
        GravityEstimate {
            gravity: self.gravity_estimate,
            confidence: if self.state == InitializationState::Complete {
                0.9
            } else {
                (self.static_frame_count as f64 / self.config.min_static_samples as f64).min(1.0)
            },
            num_samples: self.accel_samples.len(),
            accel_bias: self.accel_bias(),
        }
    }
}

/// Example: Run IMU-aided initialization on TUM-VI dataset
fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::builder()
        .filter_level(log::LevelFilter::Info)
        .init();

    println!("\n=== Phase 8A: IMU-Aided Initialization ===\n");

    // Load dataset
    let dataset_dir = "./datasets/tum_vi";
    let sequence_path = std::path::Path::new(dataset_dir).join("room1");

    let load_start = Instant::now();
    let sequence = rs_vio::datasets::tum_vi::TumViSequence::load(&sequence_path)?;
    println!(
        "✓ Loaded TUM-VI sequence in {:.2}s",
        load_start.elapsed().as_secs_f64()
    );
    println!("  - IMU samples: {}", sequence.imu_data.len());

    // Initialize
    let mut initializer = ImuAidedInitializer::new(ImuInitConfig::default());

    println!(
        "\nProcessing first {} IMU samples...",
        sequence.imu_data.len().min(2000)
    );

    let process_start = Instant::now();
    for (i, imu_meas) in sequence.imu_data.iter().enumerate().take(2000) {
        let imu = ImuData {
            timestamp: imu_meas.timestamp_ns as i64,
            gyro: [imu_meas.gyro_x, imu_meas.gyro_y, imu_meas.gyro_z],
            accel: [imu_meas.accel_x, imu_meas.accel_y, imu_meas.accel_z],
        };

        initializer.process_imu(&imu);

        if initializer.is_gravity_converged() {
            println!(
                "\n✓ Gravity converged after {} samples ({:.2}s)",
                i + 1,
                (imu.timestamp - sequence.imu_data[0].timestamp_ns as i64) as f64 / 1e9
            );
            break;
        }
    }

    println!(
        "  Total processing time: {:.2}s\n",
        process_start.elapsed().as_secs_f64()
    );

    // Report results
    let report = initializer.report();
    println!("=== Initialization Results ===");
    println!("State: {:?}", initializer.state());
    println!(
        "Gravity vector: [{:.4}, {:.4}, {:.4}] (m/s²)",
        report.gravity.x, report.gravity.y, report.gravity.z
    );
    println!("Gravity magnitude: {:.4} m/s²", report.gravity.norm());
    println!("Confidence: {:.1}%", report.confidence * 100.0);
    println!(
        "Accel bias: [{:.4}, {:.4}, {:.4}] (m/s²)",
        report.accel_bias.x, report.accel_bias.y, report.accel_bias.z
    );
    println!("Samples used: {}", report.num_samples);

    // Simulate metric scale recovery
    let depth_scale_visual = 0.001; // 1mm scale from monocular VO
    let imu_velocity_magnitude = 0.5; // ~0.5 m/s from gravity integration
    let metric_scale =
        initializer.estimate_metric_scale(depth_scale_visual, imu_velocity_magnitude);

    println!("\n=== Metric Scale Recovery ===");
    println!("Visual-only scale: {:.6}", depth_scale_visual);
    println!("IMU velocity magnitude: {:.3} m/s", imu_velocity_magnitude);
    println!("Recovered metric scale: {:.2}x", metric_scale);
    println!("\nThis scale factor would be applied to:");
    println!("  - Depth map: multiply by {:.2}", metric_scale);
    println!("  - Trajectory: multiply by {:.2}", metric_scale);
    println!("  - Point positions: multiply by {:.2}", metric_scale);

    Ok(())
}
