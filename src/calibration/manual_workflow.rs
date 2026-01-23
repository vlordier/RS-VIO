//! Manual calibration workflow orchestrator
//!
//! Implements operator-guided calibration sequence with quality gates,
//! persistence, and versioning as specified in the calibration strategy.

use super::camera_intrinsics::CameraIntrinsicsCalibrator;
use super::imu_calibration::{ImuCalibrationResult, ImuCalibrator};
use super::stereo_extrinsics::StereoExtrinsicsCalibrator;
use super::time_offset::TimeOffsetEstimator;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::path::{Path, PathBuf};

/// Serializable camera intrinsics result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CameraIntrinsicsResult {
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
    pub distortion: Vec<f64>,
    pub reprojection_rms: f64,
    pub passed: bool,
}

/// Serializable stereo extrinsics result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StereoExtrinsicsResult {
    pub rotation_lr: [[f64; 3]; 3],
    pub translation_lr: [f64; 3],
    pub baseline: f64,
    pub epipolar_error_rms: f64,
    pub vertical_disparity_rms: f64,
    pub passed: bool,
}

/// Complete calibration session result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CalibrationSession {
    /// Session timestamp
    pub timestamp_ns: i64,
    /// Session ID (UUID or similar)
    pub session_id: String,
    /// Platform identifier (drone serial, mount ID, etc.)
    pub platform_id: String,
    /// Sensor serial numbers
    pub sensor_serials: HashMap<String, String>,
    /// Environmental metadata
    pub metadata: CalibrationMetadata,
    /// IMU calibration (if performed)
    pub imu_calibration: Option<ImuCalibrationResult>,
    /// Left camera intrinsics (if performed)
    pub left_camera_intrinsics: Option<CameraIntrinsicsResult>,
    /// Right camera intrinsics (if performed)
    pub right_camera_intrinsics: Option<CameraIntrinsicsResult>,
    /// Stereo extrinsics (if performed)
    pub stereo_extrinsics: Option<StereoExtrinsicsResult>,
    /// Camera-IMU time offset (if performed)
    pub time_offset: Option<TimeOffsetResult>,
    /// Quality gates status
    pub quality_gates: QualityGatesStatus,
    /// Operator notes
    pub operator_notes: String,
    /// SHA256 hash for integrity
    pub integrity_hash: String,
}

/// Environmental and operational metadata
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CalibrationMetadata {
    /// Temperature (Celsius)
    pub temperature_c: Option<f64>,
    /// Weather conditions
    pub weather: String,
    /// Motion pattern used (e.g., "figure-8", "6-pose")
    pub motion_type: String,
    /// Location/facility
    pub location: String,
    /// Operator ID
    pub operator_id: String,
    /// Hardware revision
    pub hardware_revision: String,
}

/// Time offset calibration result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TimeOffsetResult {
    /// Estimated time offset (seconds)
    pub offset_s: f64,
    /// Uncertainty (seconds)
    pub uncertainty_s: f64,
    /// Rolling shutter readout time (if detected)
    pub readout_time_s: Option<f64>,
    /// Correlation peak sharpness (SNR)
    pub correlation_snr: f64,
    /// Quality passed
    pub passed: bool,
}

/// Quality gates aggregated status
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct QualityGatesStatus {
    /// IMU quality passed
    pub imu_passed: bool,
    /// Left camera intrinsics passed
    pub left_camera_passed: bool,
    /// Right camera intrinsics passed
    pub right_camera_passed: bool,
    /// Stereo extrinsics passed
    pub stereo_passed: bool,
    /// Time offset passed
    pub time_offset_passed: bool,
    /// Overall session passed
    pub overall_passed: bool,
    /// Failures (gate name → reason)
    pub failures: HashMap<String, String>,
}

impl QualityGatesStatus {
    pub fn new() -> Self {
        Self {
            imu_passed: false,
            left_camera_passed: false,
            right_camera_passed: false,
            stereo_passed: false,
            time_offset_passed: false,
            overall_passed: false,
            failures: HashMap::new(),
        }
    }

    pub fn update_overall(&mut self) {
        self.overall_passed = self.imu_passed
            && self.left_camera_passed
            && self.right_camera_passed
            && self.stereo_passed
            && self.time_offset_passed;
    }
}

impl Default for QualityGatesStatus {
    fn default() -> Self {
        Self::new()
    }
}

/// Calibration workflow state machine

#[allow(clippy::large_enum_variant)]
#[derive(Debug)]
pub enum CalibrationWorkflowState {
    Idle,
    ImuCalibration {
        calibrator: ImuCalibrator,
    },
    LeftCameraIntrinsics {
        calibrator: CameraIntrinsicsCalibrator,
    },
    RightCameraIntrinsics {
        calibrator: CameraIntrinsicsCalibrator,
    },
    StereoExtrinsics {
        calibrator: StereoExtrinsicsCalibrator,
    },
    TimeOffset {
        estimator: TimeOffsetEstimator,
    },
    Complete(CalibrationSession),
    Failed(String),
}

/// Manual calibration workflow orchestrator
pub struct ManualCalibrationWorkflow {
    state: CalibrationWorkflowState,
    session: CalibrationSession,
    config: WorkflowConfig,
}

/// Workflow configuration
#[derive(Debug, Clone)]
pub struct WorkflowConfig {
    /// Which calibrations to perform
    pub enable_imu: bool,
    pub enable_left_camera: bool,
    pub enable_right_camera: bool,
    pub enable_stereo: bool,
    pub enable_time_offset: bool,
    /// Quality gate thresholds
    pub imu_quality_threshold: f64,
    pub camera_reprojection_threshold: f64,
    pub stereo_epipolar_threshold: f64,
    pub time_offset_uncertainty_threshold: f64,
    /// Persistence settings
    pub save_directory: PathBuf,
    pub auto_save: bool,
}

impl Default for WorkflowConfig {
    fn default() -> Self {
        Self {
            enable_imu: true,
            enable_left_camera: true,
            enable_right_camera: true,
            enable_stereo: true,
            enable_time_offset: true,
            imu_quality_threshold: 0.9,
            camera_reprojection_threshold: 0.5,
            stereo_epipolar_threshold: 1.0,
            time_offset_uncertainty_threshold: 0.002,
            save_directory: PathBuf::from("./calibration_sessions"),
            auto_save: true,
        }
    }
}

impl ManualCalibrationWorkflow {
    /// Create new workflow with default config
    pub fn new(platform_id: String) -> Self {
        Self::with_config(platform_id, WorkflowConfig::default())
    }

    /// Create new workflow with custom config
    pub fn with_config(platform_id: String, config: WorkflowConfig) -> Self {
        let session = CalibrationSession {
            timestamp_ns: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos() as i64,
            session_id: uuid::Uuid::new_v4().to_string(),
            platform_id,
            sensor_serials: HashMap::new(),
            metadata: CalibrationMetadata {
                temperature_c: None,
                weather: String::new(),
                motion_type: String::new(),
                location: String::new(),
                operator_id: String::new(),
                hardware_revision: String::new(),
            },
            imu_calibration: None,
            left_camera_intrinsics: None,
            right_camera_intrinsics: None,
            stereo_extrinsics: None,
            time_offset: None,
            quality_gates: QualityGatesStatus::new(),
            operator_notes: String::new(),
            integrity_hash: String::new(),
        };

        Self {
            state: CalibrationWorkflowState::Idle,
            session,
            config,
        }
    }

    /// Start calibration workflow
    pub fn start(&mut self) -> Result<String, String> {
        if self.config.enable_imu {
            let calibrator = ImuCalibrator::new();
            self.state = CalibrationWorkflowState::ImuCalibration { calibrator };
            Ok(
                "Starting IMU calibration. Follow prompts to place drone in required poses."
                    .to_string(),
            )
        } else {
            Err("No calibrations enabled in workflow config".to_string())
        }
    }

    /// Get current state
    pub fn state(&self) -> &CalibrationWorkflowState {
        &self.state
    }

    /// Get operator guidance for current state
    pub fn get_guidance(&self) -> String {
        match &self.state {
            CalibrationWorkflowState::Idle => {
                "Ready to start calibration. Call start() to begin.".to_string()
            },
            CalibrationWorkflowState::ImuCalibration { .. } => "IMU Calibration:\n\
                 1. Place drone on level surface\n\
                 2. Ensure no vibration or movement\n\
                 3. Follow pose sequence:\n\
                    - Pose 0: Face down (default)\n\
                    - Pose 1: 90° pitch up\n\
                    - Pose 2: 90° roll left\n\
                    - Pose 3: 90° roll right\n\
                    - Pose 4: 180° inverted\n\
                    - Pose 5: 90° pitch down\n\
                 4. Hold each pose for ~10 seconds\n\
                 5. Call begin_pose(N) then add IMU samples"
                .to_string(),
            CalibrationWorkflowState::LeftCameraIntrinsics { .. } => "Left Camera Intrinsics:\n\
                 1. Print checkerboard pattern (or use AprilGrid)\n\
                 2. Move drone in front of target with gentle motion\n\
                 3. Vary distance: 0.5m to 5m\n\
                 4. Capture 20-30 images from different angles\n\
                 5. Ensure target visible in all frames\n\
                 6. Call add_observation() for each detected corner"
                .to_string(),
            CalibrationWorkflowState::RightCameraIntrinsics { .. } => "Right Camera Intrinsics:\n\
                 (Same as left camera, but for right sensor)"
                .to_string(),
            CalibrationWorkflowState::StereoExtrinsics { .. } => "Stereo Extrinsics:\n\
                 1. Perform figure-8 or small translation motions\n\
                 2. Include near (0.5m) and far (5m) planes\n\
                 3. Track features in both cameras simultaneously\n\
                 4. Ensure good epipolar geometry coverage\n\
                 5. Call add_stereo_observation() for each match"
                .to_string(),
            CalibrationWorkflowState::TimeOffset { .. } => "Camera-IMU Time Offset:\n\
                 1. Perform slow pan/tilt motion (~30°/sec)\n\
                 2. Maintain steady angular velocity for 5 seconds per direction\n\
                 3. System will cross-correlate optical flow with gyro\n\
                 4. Call add_imu_measurement() and add_camera_measurement()"
                .to_string(),
            CalibrationWorkflowState::Complete(_) => "Calibration complete.".to_string(),
            CalibrationWorkflowState::Failed(reason) => {
                format!("Calibration failed: {}", reason)
            },
        }
    }

    /// Save calibration session to disk
    pub fn save(&mut self) -> Result<PathBuf, String> {
        // Compute integrity hash
        let serialized = serde_yaml::to_string(&self.session)
            .map_err(|e| format!("Serialization error: {}", e))?;

        use sha2::{Digest, Sha256};
        let mut hasher = Sha256::new();
        hasher.update(serialized.as_bytes());
        self.session.integrity_hash = format!("{:x}", hasher.finalize());

        // Create versioned filename
        let filename = format!(
            "calibration_{}_{}.yaml",
            self.session.platform_id,
            chrono::Utc::now().format("%Y%m%d_%H%M%S")
        );

        let path = self.config.save_directory.join(&filename);

        // Ensure directory exists
        std::fs::create_dir_all(&self.config.save_directory)
            .map_err(|e| format!("Failed to create directory: {}", e))?;

        // Write to file
        std::fs::write(&path, serialized).map_err(|e| format!("Failed to write file: {}", e))?;

        Ok(path)
    }

    /// Load calibration session from disk
    pub fn load(path: &Path) -> Result<CalibrationSession, String> {
        let contents =
            std::fs::read_to_string(path).map_err(|e| format!("Failed to read file: {}", e))?;

        let mut session: CalibrationSession =
            serde_yaml::from_str(&contents).map_err(|e| format!("Deserialization error: {}", e))?;

        // Verify integrity hash
        let stored_hash = session.integrity_hash.clone();
        session.integrity_hash = String::new();

        let serialized =
            serde_yaml::to_string(&session).map_err(|e| format!("Serialization error: {}", e))?;

        use sha2::{Digest, Sha256};
        let mut hasher = Sha256::new();
        hasher.update(serialized.as_bytes());
        let computed_hash = format!("{:x}", hasher.finalize());

        if stored_hash != computed_hash {
            return Err("Integrity check failed: hash mismatch".to_string());
        }

        session.integrity_hash = stored_hash;
        Ok(session)
    }

    /// List all calibration sessions for a platform
    pub fn list_sessions(platform_id: &str, save_dir: &Path) -> Result<Vec<PathBuf>, String> {
        let mut sessions = Vec::new();

        let entries =
            std::fs::read_dir(save_dir).map_err(|e| format!("Failed to read directory: {}", e))?;

        for entry in entries.flatten() {
            if let Some(filename) = entry.file_name().to_str() {
                if filename.starts_with(&format!("calibration_{}_", platform_id))
                    && filename.ends_with(".yaml")
                {
                    sessions.push(entry.path());
                }
            }
        }

        sessions.sort_by(|a, b| b.cmp(a)); // Most recent first
        Ok(sessions)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_workflow_creation() {
        let workflow = ManualCalibrationWorkflow::new("test_drone_001".to_string());
        assert!(matches!(workflow.state(), CalibrationWorkflowState::Idle));
    }

    #[test]
    fn test_quality_gates_overall() {
        let mut gates = QualityGatesStatus::new();
        gates.imu_passed = true;
        gates.left_camera_passed = true;
        gates.right_camera_passed = true;
        gates.stereo_passed = true;
        gates.time_offset_passed = true;
        gates.update_overall();

        assert!(gates.overall_passed);
    }

    #[test]
    fn test_session_serialization() {
        let workflow = ManualCalibrationWorkflow::new("test_drone_002".to_string());
        let serialized = serde_yaml::to_string(&workflow.session).unwrap();

        assert!(serialized.contains("platform_id"));
        assert!(serialized.contains("test_drone_002"));
    }
}
