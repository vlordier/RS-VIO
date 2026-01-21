pub mod accessors;
pub mod calibration;
pub mod constructor;
pub mod helpers;
pub mod imu_analysis;
pub mod processor;
/// Estimator module split into focused sub-modules for maintainability
///
/// The large Estimator implementation has been split into logical modules:
/// - state: Core struct definition and fields
/// - constructor: Initialization (new, new_with_cameras)
/// - processor: Main frame processing logic (process_frame)
/// - viewer: Visualization methods
/// - imu_analysis: IMU fundamental frequency and decomposition
/// - accessors: Getter methods and accessors
/// - calibration: Intrinsics and extrinsics calibration updates
/// - helpers: Loop closure descriptor creation
/// - tests: Unit tests
pub mod state;
pub mod tests;
pub mod viewer;

// Re-export the main Estimator type for convenience
pub use state::Estimator;
