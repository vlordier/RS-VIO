//! Physical and numerical constants for RS-VIO.
//!
//! This module centralizes all magic numbers and physical constants
//! used throughout the codebase for better maintainability and testing.

/// Standard gravity magnitude [m/s²] (WGS84)
pub const GRAVITY_MAGNITUDE: f64 = 9.81007;

/// Default IMU noise parameters (typical for consumer-grade IMUs)
pub mod imu_noise {
    /// Gyroscope white noise standard deviation [rad/s]
    pub const GYRO_NOISE_STD: f64 = 1e-3;
    
    /// Gyroscope bias random walk [rad/s²]
    pub const GYRO_RANDOM_WALK_STD: f64 = 1e-4;
    
    /// Accelerometer white noise [m/s²]
    pub const ACCEL_NOISE_STD: f64 = 1e-2;
    
    /// Accelerometer bias random walk [m/s^(5/2)]
    pub const ACCEL_RANDOM_WALK_STD: f64 = 1e-3;
}

/// Numerical tolerances
pub mod tolerance {
    /// Small epsilon for numerical stability
    pub const EPSILON: f64 = 1e-10;
    
    /// Threshold for considering rotation angle as zero
    pub const ROTATION_ZERO: f64 = 1e-8;
    
    /// Threshold for considering time delta as zero
    pub const TIME_ZERO: f64 = 1e-6;

    /// Minimum depth for valid 3D points (m).
    pub const MIN_DEPTH: f64 = 0.1;

    /// Maximum depth for valid 3D points (m).
    pub const MAX_DEPTH: f64 = 100.0;
}

/// Optimization parameters
pub mod optimization {
    /// Default Huber loss threshold
    pub const HUBER_THRESHOLD: f64 = 2.0;
    
    /// Default maximum iterations for bundle adjustment
    pub const BA_MAX_ITERATIONS: u32 = 20;
    
    /// Default maximum iterations for PnP
    pub const PNP_MAX_ITERATIONS: u32 = 10;
    
    /// Convergence threshold for optimization
    pub const CONVERGENCE_THRESHOLD: f32 = 0.01;
}

/// Feature tracking parameters
pub mod feature_tracking {
    /// FAST9 threshold
    pub const FAST_THRESHOLD: u8 = 20;
    
    /// Pattern size for patch tracking (52 points)
    pub const PATTERN_SIZE: usize = 52;
    
    /// Maximum pyramid levels for tracking
    pub const MAX_PYRAMID_LEVELS: u32 = 3;
}

/// Keyframe selection thresholds
pub mod keyframe {
    /// Minimum translation for keyframe selection (m).
    pub const TRANSLATION_THRESHOLD: f64 = 0.4;

    /// Minimum rotation for keyframe selection (rad).
    pub const ROTATION_THRESHOLD: f64 = 0.25;

    /// Default sliding window size
    pub const WINDOW_SIZE: usize = 10;
}

/// Platform-specific motion constraints
pub mod motion_constraints {
    /// Quadrotor max speed (m/s).
    pub const QUAD_MAX_SPEED: f32 = 20.0;
    
    /// Quadrotor max acceleration (m/s²).
    pub const QUAD_MAX_ACCEL: f32 = 12.0;

    /// Quadrotor max tilt angle (rad) (~50°).
    pub const QUAD_MAX_TILT: f32 = 0.87;

    /// UGV max speed (m/s).
    pub const UGV_MAX_SPEED: f32 = 8.0;

    /// UGV max lateral velocity (m/s).
    pub const UGV_MAX_LATERAL: f32 = 0.8;

    /// Fixed-wing min cruise speed (m/s).
    pub const FW_MIN_SPEED: f32 = 8.0;
    
    /// Fixed-wing max cruise speed (m/s).
    pub const FW_MAX_SPEED: f32 = 35.0;
}
