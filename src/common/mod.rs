//! Common utilities and patterns shared across the RS-VIO codebase.
//!
//! This module contains reusable components, traits, and helpers that promote
//! DRY principles and maintainability.

pub mod config;
pub mod error;
#[macro_use]
pub mod macros;
pub mod math;
pub mod perf;
#[cfg(test)]
pub mod testing;
pub mod types;
pub mod validation;

pub use config::{Clampable, ConfigBuilder, Mergeable, Validatable};
pub use error::{ErrorCollector, OptionExt, ResultExt};
pub use math::{clamp, normalize_angle, safe_sqrt};
pub use types::{CameraId, Confidence, FeatureId, FrameId, Timestamp};
pub use validation::{validate_matrix, validate_pose, validate_quaternion};
