//! # Camera Processing Module
//!
//! Handles camera-specific processing including rolling shutter compensation,
//! intrinsic calibration, and image coordinate transformations.
//!
//! ## Modules
//!
//! - `rolling_shutter` - Rolling shutter distortion compensation

pub mod rolling_shutter;

pub use rolling_shutter::{RollingShutterCompensator, RollingShutterConfig};
