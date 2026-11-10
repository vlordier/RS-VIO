use crate::{
    constants::GRAVITY_MAGNITUDE,
    msckf::degrade::{FrameQuality, VisualMode},
    types::{DMatrix, Float, UnitQuaternion, Vector3},
};

pub const IMU_STATE_DOF: usize = 15;
pub const CLONE_DOF: usize = 6;

/// Nominal IMU state used by the error-state filter.
#[derive(Debug, Clone)]
pub struct ImuState {
    pub orientation: UnitQuaternion,
    pub velocity: Vector3,
    pub position: Vector3,
    pub gyro_bias: Vector3,
    pub accel_bias: Vector3,
}

impl Default for ImuState {
    fn default() -> Self {
        Self {
            orientation: UnitQuaternion::identity(),
            velocity: Vector3::zeros(),
            position: Vector3::zeros(),
            gyro_bias: Vector3::zeros(),
            accel_bias: Vector3::zeros(),
        }
    }
}

/// Camera pose cloned out of the nominal IMU state for MSCKF updates.
#[derive(Debug, Clone)]
pub struct CameraClone {
    pub clone_id: usize,
    pub timestamp_s: Float,
    pub orientation: UnitQuaternion,
    pub position: Vector3,
}

impl CameraClone {
    #[must_use]
    pub fn new(
        clone_id: usize,
        timestamp_s: Float,
        orientation: UnitQuaternion,
        position: Vector3,
    ) -> Self {
        Self {
            clone_id,
            timestamp_s,
            orientation,
            position,
        }
    }
}

/// Full filter container for the parallel MSCKF estimator.
#[derive(Debug, Clone)]
pub struct FilterState {
    pub imu_state: ImuState,
    pub clones: Vec<CameraClone>,
    pub covariance: DMatrix,
    pub gravity_world: Vector3,
    pub mode: VisualMode,
    pub last_quality: FrameQuality,
}

impl Default for FilterState {
    fn default() -> Self {
        Self {
            imu_state: ImuState::default(),
            clones: Vec::new(),
            covariance: DMatrix::identity(IMU_STATE_DOF, IMU_STATE_DOF),
            gravity_world: Vector3::new(0.0, 0.0, -(GRAVITY_MAGNITUDE as Float)),
            mode: VisualMode::Init,
            last_quality: FrameQuality::default(),
        }
    }
}

impl FilterState {
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    #[must_use]
    pub fn state_dim(&self) -> usize {
        IMU_STATE_DOF + self.clones.len() * CLONE_DOF
    }

    pub fn resize_covariance_for_clones(&mut self) {
        let state_dim = self.state_dim();
        if self.covariance.nrows() == state_dim && self.covariance.ncols() == state_dim {
            return;
        }

        let mut resized = DMatrix::zeros(state_dim, state_dim);
        let rows_to_copy = self.covariance.nrows().min(state_dim);
        let cols_to_copy = self.covariance.ncols().min(state_dim);
        resized
            .view_mut((0, 0), (rows_to_copy, cols_to_copy))
            .copy_from(&self.covariance.view((0, 0), (rows_to_copy, cols_to_copy)));
        self.covariance = resized;
    }
}