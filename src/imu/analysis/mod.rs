//! Signal analysis and harmonic decomposition module
//!
//! Analyzes raw IMU measurements to extract meaningful components:
//! - **Gravity Component**: Estimated from accelerometer (typically ≈ [0, 0, -9.81] m/s²)
//! - **Bias Estimation**: Constant sensor offsets (accel and gyro)
//! - **Motor State Detection**: Detect when drone rotors are on/off
//! - **Harmonic Extraction**: Frequency-domain analysis of rotor vibrations (f0, 2f0, 3f0, ...)
//! - **Signal Quality**: SNR, RMS, and peak detection with motor-state awareness
//!
//! ## Typical IMU Signal Decomposition
//!
//! **Motors OFF** (Stationary/Ground):
//! ```text
//! Raw Accel = Gravity + Bias + Noise
//!           = [0, 0, -9.81] + [b_x, b_y, b_z] + [n_x, n_y, n_z]
//! ```
//!
//! **Motors RUNNING** (Flight):
//! ```text
//! Raw Accel = Gravity + Bias + Rotor_Harmonics(f0, 2f0, 3f0, ...) + Noise
//!           = [0, 0, -9.81] + [b_x, b_y, b_z] + [h_x(f0), h_y(f0), h_z(f0)] + [n_x, n_y, n_z]
//! ```
//!
//! ## Processing Pipeline
//!
//! 1. **Raw IMU Data** (from sensor)
//! 2. **Motor State Detection** (vibration-based threshold)
//! 3. **Static Component Removal** (gravity, bias - adapted for motor state)
//! 4. **Harmonic Decomposition** (f0 extraction + higher harmonics when motors running)
//! 5. **Quality Assessment** (SNR, vibration with motor-aware thresholds)

pub mod config;
pub mod quality;
pub mod spectral;
pub mod harmonic;
pub mod motor_detection;
pub mod bias;
pub mod analyzer;
mod tests;

// Re-export public API
pub use config::{MotorState, SignalQuality, HarmonicDecomposition};
pub use analyzer::ImuSignalAnalyzer;
