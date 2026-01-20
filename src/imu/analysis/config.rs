//! Configuration constants and types for signal analysis
//!
//! Contains all constants, enums, and configuration structures used across
//! the signal analysis module.

use crate::types::Vector3;

// ============================================================================
// Constants
// ============================================================================

/// Default motor vibration threshold [m/s²]
pub const DEFAULT_MOTOR_THRESHOLD: f32 = 0.5;

/// Hysteresis factor for motor state transitions (motor-off threshold)
pub const MOTOR_OFF_HYSTERESIS: f32 = 0.5;

/// Default noise floor estimate [m/s²]
pub const DEFAULT_NOISE_FLOOR: f32 = 0.01;

/// Noise floor multiplier when motors are running
pub const MOTOR_RUNNING_NOISE_MULTIPLIER: f32 = 5.0;

/// Noise floor multiplier during transitions
pub const MOTOR_TRANSITIONING_NOISE_MULTIPLIER: f32 = 2.0;

/// Minimum samples required for motor state detection
pub const MIN_SAMPLES_FOR_DETECTION: usize = 10;

/// Minimum samples required for frequency estimation
pub const MIN_SAMPLES_FOR_FREQUENCY: usize = 20;

/// Standard gravity magnitude [m/s²]
pub const GRAVITY_MAG: f64 = 9.81;

/// Minimum gravity magnitude for direction estimation [m/s²]
pub const MIN_GRAVITY_FOR_ESTIMATION: f64 = 0.1;

/// Minimum frequency for valid rotor detection [Hz]
pub const MIN_VALID_FREQUENCY: f32 = 10.0;

/// Maximum frequency for valid rotor detection [Hz]
pub const MAX_VALID_FREQUENCY: f32 = 1000.0;

/// Peak detection threshold (fraction of last peak)
pub const PEAK_DETECTION_THRESHOLD: f64 = 0.5;

/// Bias update rate when motors are running (filtered)
pub const BIAS_UPDATE_RATE_MOTORS_RUNNING: f64 = 0.05;

/// Bias filtering coefficient when motors running
pub const BIAS_FILTER_COEFF_MOTORS_RUNNING: f64 = 0.95;

/// Harmonic extraction coefficient for motors running
pub const HARMONIC_COEFF_RUNNING: f64 = 0.3;

/// Harmonic extraction coefficient during transition
pub const HARMONIC_COEFF_TRANSITIONING: f64 = 0.4;

/// Nanoseconds to seconds conversion
pub const NS_TO_SECONDS: f64 = 1e9;

// ============================================================================
// Types
// ============================================================================

/// Motor/rotor state detection
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MotorState {
    /// Motors off - stationary, no rotor vibration
    Off,
    /// Motors running - active flight with rotor harmonics
    Running,
    /// Transitioning between states
    Transitioning,
}

/// Signal quality metrics for IMU measurements
#[derive(Debug, Clone, Copy)]
pub struct SignalQuality {
    /// Signal-to-Noise Ratio per axis [dB]
    pub snr: [f32; 3],
    /// RMS value per axis [m/s²] for accel or [rad/s] for gyro
    pub rms: [f32; 3],
    /// Peak absolute value per axis [m/s²] for accel or [rad/s] for gyro
    pub peak: [f32; 3],
    /// Detected motor state
    pub motor_state: MotorState,
    /// Estimated fundamental rotor frequency [Hz] (0 if motors off)
    pub fundamental_freq_hz: f32,
}

/// Harmonic decomposition result
#[derive(Debug, Clone)]
pub struct HarmonicDecomposition {
    /// Gravity component in sensor frame [m/s²]
    pub gravity: Vector3,
    /// Accelerometer bias [m/s²]
    pub accel_bias: Vector3,
    /// Gyroscope bias [rad/s]
    pub gyro_bias: Vector3,
    /// Fundamental frequency harmonic (f0) [m/s²]
    /// Strong when rotors running, near-zero when off
    pub fundamental_harmonic: Vector3,
    /// Higher harmonics (2f0, 3f0, ...) and residual noise [m/s²]
    pub residual_harmonics: Vec<Vector3>,
    /// Signal quality metrics (includes motor state)
    pub quality: SignalQuality,
    /// Detected motor state for this decomposition
    pub motor_state: MotorState,
}
