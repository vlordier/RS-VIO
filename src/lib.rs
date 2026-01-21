#![allow(
    clippy::unnecessary_cast,        // Casts between numeric types for clarity
    clippy::manual_clamp,            // Manual clamp patterns
    clippy::clone_on_copy,           // Rare cases with complex types
    clippy::manual_is_multiple_of,   // Manual modulo checks
    clippy::needless_range_loop,     // Range loops over indices
    clippy::should_implement_trait,  // Custom default() method names
    clippy::too_many_arguments,      // Some functions legitimately need many args
    clippy::assign_op_pattern,       // Manual += operations for clarity
    clippy::manual_map,              // Manual map patterns
    clippy::doc_lazy_continuation,   // Doc comment formatting
    clippy::empty_line_after_doc_comments, // Doc comment style
    clippy::implicit_saturating_sub, // Manual arithmetic checks
    clippy::unwrap_or_default,       // or_insert_with vs or_default patterns
    clippy::expect_used,             // Expect in specific contexts (documented)
    clippy::for_kv_map,              // Iterating over map keys/values
    clippy::new_without_default,     // Custom new() methods are preferred
    clippy::type_complexity,         // Complex types for calibration observations
    clippy::float_cmp,               // Careful float comparisons used where appropriate
    clippy::get_first,               // Direct indexing for clarity
    clippy::iter_kv_map,             // Iterating over map values explicitly
    clippy::approx_constant,         // Custom constants preferred over std::f64::consts
    clippy::unwrap_used,             // Unwrap used in hot paths after validation
    clippy::let_and_return,          // Explicit let bindings for clarity
    clippy::bind_instead_of_map,     // and_then patterns for control flow
    clippy::map_clone,               // Manual clone for explicit copying
    clippy::derivable_impls,         // Manual Default for complex initialization
    clippy::inherent_to_string,      // Custom to_string for domain-specific formatting
    clippy::useless_format,          // format! for consistency in string building
    clippy::single_char_add_str,     // push_str for consistency
    clippy::redundant_closure,       // Closures for flexibility in iteration
    clippy::field_reassign_with_default, // Config reassignment for clarity
    clippy::vec_init_then_push,      // Conditional vec building
    clippy::manual_range_contains,   // Explicit range comparisons for clarity
    clippy::collapsible_if,          // Nested ifs for readability
    clippy::needless_return,         // Explicit return for clarity
    clippy::manual_div_ceil,         // Manual div_ceil for compatibility
    clippy::neg_cmp_op_on_partial_ord, // Comparison operators in ensure! macro
    clippy::len_zero,                // len() > 0 for clarity in assertions
    clippy::assertions_on_constants, // Test scaffolding assertions
    clippy::module_inception,        // Test modules can have same name
    clippy::needless_borrows_for_generic_args, // Explicit borrows for clarity
    clippy::useless_vec,             // vec! for consistency in test data
)]
#![allow(
    rustdoc::broken_intra_doc_links,
    rustdoc::invalid_html_tags,
    rustdoc::redundant_explicit_links,
)]
//! # RS-VIO
//!
//! A Rust implementation of Visual-Inertial Odometry (VIO) for stereo cameras.
//!
//! This library provides tools for processing stereo image sequences with IMU data
//! to estimate camera motion and build 3D maps. It includes dataset players for
//! standard benchmarks like EuRoC, 4Seasons, and TUM-VI.
//!
//! ## Features
//!
//! - Stereo visual feature tracking
//! - IMU integration for motion estimation
//! - Bundle adjustment optimization
//! - Real-time visualization support
//! - Dataset players for common VIO benchmarks
//!
//! ## Usage
//!
//! ```rust,no_run
//! use rs_vio::{datasets::config::Config, estimator::Estimator};
//! use serde_yaml;
//!
//! // Load configuration
//! let config_yaml = std::fs::read_to_string("config.yaml")?;
//! let config: Config = serde_yaml::from_str(&config_yaml)?;
//!
//! // Create estimator
//! let mut estimator = Estimator::new(config, None);
//!
//! // Process frames...
//! # Ok::<(), Box<dyn std::error::Error>>(())
//! ```

// Allow intentional numeric conversions in VIO mathematics
// VIO algorithms require conversions between:
// - Pixel coordinates (u32) ↔ metric space (f32/f64)
// - Image dimensions (u32) ↔ normalized coordinates (f32)
// - Timestamps (i64) ↔ floating point time (f64)
// All conversions are intentional and documented per the VIO math
#![allow(
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::cast_sign_loss,
    clippy::cast_lossless
)]
// Default to f64 for scientific computing (standard in robotics/CV)
// Embedded systems can override with explicit f32 if needed
#![allow(clippy::default_numeric_fallback)]
// Note: const fn could be added for matrix identity and zero operations
// for compile-time computation benefits (future optimization)
#![allow(clippy::missing_const_for_fn)]

use thiserror::Error;

/// Custom error type for RS-VIO operations.
#[derive(Error, Debug)]
pub enum VIOError {
    #[error("Configuration error: {0}")]
    Config(String),
    #[error("Image processing error: {0}")]
    Image(String),
    #[error("Optimization error: {0}")]
    Optimization(String),
    #[error("Solver error: {0}")]
    Solver(String),
    #[error("Parsing error: {0}")]
    Parse(String),
    #[error("Viewer error: {0}")]
    Viewer(String),
    #[error("IO error: {0}")]
    Io(#[from] std::io::Error),
    #[error("Anyhow error: {0}")]
    Anyhow(#[from] anyhow::Error),
}

/// Result type for RS-VIO operations, defaulting to VIOError.
pub type Result<T> = std::result::Result<T, VIOError>;

/// Initialize colored logging for RS-VIO.
///
/// Sets up immediate colored output with proper formatting for all binaries.
/// Call this at the beginning of your application before any logging.
///
/// # Examples
///
/// ```rust
/// use rs_vio::init_colored_logging;
///
/// init_colored_logging();
/// log::info!("Application started");
/// ```
pub fn init_colored_logging() {
    use env_logger::Builder;
    use env_logger::Env;
    use log::LevelFilter;
    use std::io::Write;

    Builder::from_env(Env::default().default_filter_or("debug"))
        // Silence rerun noise unless it's a warning or worse
        .filter_module("rerun", LevelFilter::Warn)
        .format_timestamp_millis()
        .format(|buf, record| {
            let level = match record.level() {
                log::Level::Error => "\x1b[31mERROR\x1b[0m",
                log::Level::Warn => "\x1b[33mWARN\x1b[0m",
                log::Level::Info => "\x1b[32mINFO\x1b[0m",
                log::Level::Debug => "\x1b[34mDEBUG\x1b[0m",
                log::Level::Trace => "\x1b[36mTRACE\x1b[0m",
            };
            writeln!(
                buf,
                "[{}] [{}] {}",
                buf.timestamp_millis(),
                level,
                record.args()
            )
        })
        .try_init()
        .ok();
}

/// Initialize logging for RS-VIO.
///
/// This function sets up structured logging with appropriate levels and formatting.
/// Call this at the beginning of your application.
///
/// # Examples
///
/// ```rust
/// use rs_vio::init_logging;
///
/// init_logging();
/// ```
pub fn init_logging() {
    use tracing_subscriber::{layer::SubscriberExt, util::SubscriberInitExt};

    tracing_subscriber::registry()
        .with(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| "rs_vio=info".into()),
        )
        .with(tracing_subscriber::fmt::layer().json())
        .init();
}

pub mod calibration;
pub mod camera;
pub mod common;
pub mod datasets;
pub mod estimator;
pub mod dense_reconstruction;
pub mod evaluation;
pub mod feature_detection;
pub mod feature_tracker;
pub mod fusion;
pub mod imu;
pub mod logging;
pub mod loop_closure;
pub mod math;
pub mod multi_drone;
pub mod optimization;
pub mod platform;
pub mod traits;
pub mod types;
pub mod validation;
pub mod viewers;
pub mod vision;

// Re-export commonly used types for convenience
pub use datasets::config::Config;
pub use datasets::euroc_player::EurocPlayer;
pub use datasets::fourseasons_player::FourSeasonsPlayer;
pub use datasets::tum_vi_player::TUMVIPlayer;
pub use datasets::{PlayerConfig, PlayerResult};
pub use optimization::marginalization::MarginalizationPrior;
