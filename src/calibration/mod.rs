/// Complete camera + IMU calibration framework for stereo VIO systems.
/// 
/// Implements the comprehensive calibration guide:
/// - Section 1: Rolling shutter detection and readout time estimation
/// - Section 2: Camera intrinsics and stereo extrinsics (offline via OpenCV)
/// - Section 3: IMU intrinsic calibration (offline via bench measurements)
/// - Section 4: Camera-IMU extrinsics (spatial T_IC) and time offset (temporal Δt)
/// - Section 5: Timing quality assessment and operating mode recommendation
/// - Section 6: Quality report generation with acceptance decisions
/// - Section 7: Unified camera-agnostic solver (handles global/rolling, sync/unsync)
/// 
/// ## Quick Start
/// 
/// ```ignore
/// // Collect calibration data (raw images, IMU, timestamps)
/// let dataset = CalibrationDataset { ... };
/// 
/// // Create and run unified solver
/// let mut solver = UnifiedCalibrationSolver::new(UnifiedCalibrationConfig::default());
/// let (result, report) = solver.solve(
///     &dataset,
///     &camera_intrinsics,
///     &camera_distortions,
///     &camera_imu_extrinsics,
///     &imu_intrinsics,
///     &AcceptanceThresholds::standard(),
/// );
/// 
/// // Check if passed
/// if result.timing_quality.observability > 0.6 {
///     // Use tight RS model + IMU coupling
/// } else if let TimingQuality::Stable { jitter_s, .. } = result.timing_quality {
///     // Use simplified coupling
/// }
/// ```

pub mod types;
pub mod time_offset;
pub mod rolling_shutter;
pub mod unified_solver;

pub use types::{
    CameraIntrinsics, DistortionModel, CameraCalibrationResult,
    StereoExtrinsics, StereoCalibrationResult,
    IMUIntrinsics, IMUCalibrationResult,
    CameraIMUExtrinsics, CameraIMUCalibrationResult,
    RollingShutterDetectionResult,
    TimingQuality, CalibrationResult, AcceptanceThresholds,
    CalibrationQualityReport,
};

pub use time_offset::{
    TimeOffsetEstimator, CameraMeasurement, IMUMeasurement,
    PreintegrationResult,
};

pub use rolling_shutter::{
    RollingShutterDetector, LineSegment,
};

pub use unified_solver::{
    UnifiedCalibrationConfig, CalibrationDataset, UnifiedCalibrationSolver,
};
