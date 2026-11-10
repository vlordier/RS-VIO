use crate::{
    constants::{imu_noise, tolerance},
    datasets::ImuData,
    msckf::state::{FilterState, ImuState, IMU_STATE_DOF},
    types::{Float, Matrix3x3, SMatrix, UnitQuaternion, Vector3},
};

pub type Matrix15 = SMatrix<15, 15>;
pub type Matrix15x12 = SMatrix<15, 12>;

#[derive(Debug, Clone, Copy)]
pub struct ImuNoise {
    pub gyro_noise_std: Float,
    pub accel_noise_std: Float,
    pub gyro_random_walk_std: Float,
    pub accel_random_walk_std: Float,
}

impl Default for ImuNoise {
    fn default() -> Self {
        Self {
            gyro_noise_std: imu_noise::GYRO_NOISE_STD as Float,
            accel_noise_std: imu_noise::ACCEL_NOISE_STD as Float,
            gyro_random_walk_std: imu_noise::GYRO_RANDOM_WALK_STD as Float,
            accel_random_walk_std: imu_noise::ACCEL_RANDOM_WALK_STD as Float,
        }
    }
}

#[derive(Debug, Clone)]
pub struct PropagationStep {
    pub nominal_state: ImuState,
    pub phi: Matrix15,
    pub q_d: Matrix15,
    pub dt: Float,
}

#[must_use]
fn skew(v: &Vector3) -> Matrix3x3 {
    Matrix3x3::new(0.0, -v.z, v.y, v.z, 0.0, -v.x, -v.y, v.x, 0.0)
}

#[must_use]
fn sample_dt_s(curr: &ImuData, next: &ImuData) -> Float {
    (next.timestamp - curr.timestamp) as Float * 1e-9
}

#[must_use]
fn corrected_gyro(sample: &ImuData, bias: &Vector3) -> Vector3 {
    Vector3::new(
        sample.gyro[0] as Float - bias.x,
        sample.gyro[1] as Float - bias.y,
        sample.gyro[2] as Float - bias.z,
    )
}

#[must_use]
fn corrected_accel(sample: &ImuData, bias: &Vector3) -> Vector3 {
    Vector3::new(
        sample.accel[0] as Float - bias.x,
        sample.accel[1] as Float - bias.y,
        sample.accel[2] as Float - bias.z,
    )
}

#[must_use]
fn midpoint_measurement(state: &ImuState, curr: &ImuData, next: &ImuData, dt: Float) -> (Vector3, Vector3, UnitQuaternion) {
    let omega_curr = corrected_gyro(curr, &state.gyro_bias);
    let omega_next = corrected_gyro(next, &state.gyro_bias);
    let accel_curr = corrected_accel(curr, &state.accel_bias);
    let accel_next = corrected_accel(next, &state.accel_bias);

    let omega_mid = (omega_curr + omega_next) * 0.5;
    let accel_mid = (accel_curr + accel_next) * 0.5;
    let orientation_mid = state.orientation * UnitQuaternion::from_scaled_axis(omega_mid * (dt * 0.5));

    (omega_mid, accel_mid, orientation_mid)
}

#[must_use]
pub fn propagate_nominal_state(
    state: &ImuState,
    curr: &ImuData,
    next: &ImuData,
    gravity_world: &Vector3,
) -> Option<ImuState> {
    let dt = sample_dt_s(curr, next);
    if dt <= tolerance::TIME_ZERO as Float || !dt.is_finite() {
        return None;
    }

    let (omega_mid, accel_mid, orientation_mid) = midpoint_measurement(state, curr, next, dt);
    let delta_q = UnitQuaternion::from_scaled_axis(omega_mid * dt);
    let accel_world_mid = orientation_mid.transform_vector(&accel_mid) + gravity_world;

    Some(ImuState {
        orientation: state.orientation * delta_q,
        velocity: state.velocity + accel_world_mid * dt,
        position: state.position + state.velocity * dt + accel_world_mid * (0.5 * dt * dt),
        gyro_bias: state.gyro_bias,
        accel_bias: state.accel_bias,
    })
}

#[must_use]
pub fn imu_state_transition(
    state: &ImuState,
    curr: &ImuData,
    next: &ImuData,
) -> Option<Matrix15> {
    let dt = sample_dt_s(curr, next);
    if dt <= tolerance::TIME_ZERO as Float || !dt.is_finite() {
        return None;
    }

    let (omega_mid, accel_mid, orientation_mid) = midpoint_measurement(state, curr, next, dt);
    let rotation_mid = orientation_mid.to_rotation_matrix().into_inner();
    let accel_world_jacobian = -rotation_mid * skew(&accel_mid);

    let mut phi = Matrix15::identity();
    phi.fixed_view_mut::<3, 3>(0, 0)
        .copy_from(&UnitQuaternion::from_scaled_axis(-omega_mid * dt).to_rotation_matrix().into_inner());
    phi.fixed_view_mut::<3, 3>(0, 9)
        .copy_from(&(-Matrix3x3::identity() * dt));
    phi.fixed_view_mut::<3, 3>(3, 0)
        .copy_from(&(accel_world_jacobian * dt));
    phi.fixed_view_mut::<3, 3>(3, 12)
        .copy_from(&(-rotation_mid * dt));
    phi.fixed_view_mut::<3, 3>(6, 0)
        .copy_from(&(accel_world_jacobian * (0.5 * dt * dt)));
    phi.fixed_view_mut::<3, 3>(6, 3)
        .copy_from(&(Matrix3x3::identity() * dt));
    phi.fixed_view_mut::<3, 3>(6, 12)
        .copy_from(&(-rotation_mid * (0.5 * dt * dt)));

    Some(phi)
}

#[must_use]
pub fn process_noise_mapping(
    state: &ImuState,
    curr: &ImuData,
    next: &ImuData,
) -> Option<Matrix15x12> {
    let dt = sample_dt_s(curr, next);
    if dt <= tolerance::TIME_ZERO as Float || !dt.is_finite() {
        return None;
    }

    let (_, _, orientation_mid) = midpoint_measurement(state, curr, next, dt);
    let rotation_mid = orientation_mid.to_rotation_matrix().into_inner();

    let mut noise_jacobian = Matrix15x12::zeros();
    noise_jacobian
        .fixed_view_mut::<3, 3>(0, 0)
        .copy_from(&(-Matrix3x3::identity() * dt));
    noise_jacobian
        .fixed_view_mut::<3, 3>(3, 3)
        .copy_from(&(-rotation_mid * dt));
    noise_jacobian
        .fixed_view_mut::<3, 3>(6, 3)
        .copy_from(&(-rotation_mid * (0.5 * dt * dt)));
    noise_jacobian
        .fixed_view_mut::<3, 3>(9, 6)
        .copy_from(&(Matrix3x3::identity() * dt));
    noise_jacobian
        .fixed_view_mut::<3, 3>(12, 9)
        .copy_from(&(Matrix3x3::identity() * dt));

    Some(noise_jacobian)
}

#[must_use]
pub fn discretize_process_noise(
    state: &ImuState,
    curr: &ImuData,
    next: &ImuData,
    noise: &ImuNoise,
) -> Option<Matrix15> {
    let noise_jacobian = process_noise_mapping(state, curr, next)?;
    let mut continuous = SMatrix::<12, 12>::zeros();
    let gyro_var = noise.gyro_noise_std * noise.gyro_noise_std;
    let accel_var = noise.accel_noise_std * noise.accel_noise_std;
    let gyro_rw_var = noise.gyro_random_walk_std * noise.gyro_random_walk_std;
    let accel_rw_var = noise.accel_random_walk_std * noise.accel_random_walk_std;

    continuous
        .fixed_view_mut::<3, 3>(0, 0)
        .copy_from(&(Matrix3x3::identity() * gyro_var));
    continuous
        .fixed_view_mut::<3, 3>(3, 3)
        .copy_from(&(Matrix3x3::identity() * accel_var));
    continuous
        .fixed_view_mut::<3, 3>(6, 6)
        .copy_from(&(Matrix3x3::identity() * gyro_rw_var));
    continuous
        .fixed_view_mut::<3, 3>(9, 9)
        .copy_from(&(Matrix3x3::identity() * accel_rw_var));

    Some(noise_jacobian * continuous * noise_jacobian.transpose())
}

pub fn propagate_covariance(filter_state: &mut FilterState, phi: &Matrix15, q_d: &Matrix15) {
    filter_state.resize_covariance_for_clones();

    let state_dim = filter_state.state_dim();
    if state_dim < IMU_STATE_DOF {
        return;
    }

    let clones_dim = state_dim - IMU_STATE_DOF;
    let mut propagated = filter_state.covariance.clone();

    let imu_covariance = filter_state
        .covariance
        .view((0, 0), (IMU_STATE_DOF, IMU_STATE_DOF))
        .into_owned();
    let propagated_imu = phi * imu_covariance * phi.transpose() + q_d;
    propagated
        .view_mut((0, 0), (IMU_STATE_DOF, IMU_STATE_DOF))
        .copy_from(&propagated_imu);

    if clones_dim > 0 {
        let imu_clone_cross = filter_state
            .covariance
            .view((0, IMU_STATE_DOF), (IMU_STATE_DOF, clones_dim))
            .into_owned();
        let propagated_cross = phi * imu_clone_cross;
        propagated
            .view_mut((0, IMU_STATE_DOF), (IMU_STATE_DOF, clones_dim))
            .copy_from(&propagated_cross);
        propagated
            .view_mut((IMU_STATE_DOF, 0), (clones_dim, IMU_STATE_DOF))
            .copy_from(&propagated_cross.transpose());
    }

    filter_state.covariance = propagated;
}

#[must_use]
pub fn compute_propagation_step(
    state: &FilterState,
    curr: &ImuData,
    next: &ImuData,
    noise: &ImuNoise,
) -> Option<PropagationStep> {
    let dt = sample_dt_s(curr, next);
    if dt <= tolerance::TIME_ZERO as Float || !dt.is_finite() {
        return None;
    }

    let nominal_state = propagate_nominal_state(&state.imu_state, curr, next, &state.gravity_world)?;
    let phi = imu_state_transition(&state.imu_state, curr, next)?;
    let q_d = discretize_process_noise(&state.imu_state, curr, next, noise)?;

    Some(PropagationStep {
        nominal_state,
        phi,
        q_d,
        dt,
    })
}

#[must_use]
pub fn propagate_filter_state(
    filter_state: &mut FilterState,
    curr: &ImuData,
    next: &ImuData,
    noise: &ImuNoise,
) -> Option<Float> {
    let step = compute_propagation_step(filter_state, curr, next, noise)?;
    filter_state.imu_state = step.nominal_state;
    propagate_covariance(filter_state, &step.phi, &step.q_d);
    Some(step.dt)
}

#[cfg(test)]
mod tests {
    use super::{propagate_filter_state, ImuNoise};
    use crate::{
        constants::GRAVITY_MAGNITUDE,
        datasets::ImuData,
        msckf::state::FilterState,
    };

    #[test]
    fn propagate_static_imu_for_one_second_keeps_nominal_state_stable() {
        let mut filter_state = FilterState::new();
        let noise = ImuNoise::default();
        let accel_z = GRAVITY_MAGNITUDE;
        let dt_ns = 10_000_000_i64;

        for sample_idx in 0..100 {
            let curr = ImuData {
                timestamp: sample_idx * dt_ns,
                gyro: [0.0, 0.0, 0.0],
                accel: [0.0, 0.0, accel_z],
            };
            let next = ImuData {
                timestamp: (sample_idx + 1) * dt_ns,
                gyro: [0.0, 0.0, 0.0],
                accel: [0.0, 0.0, accel_z],
            };

            let propagated = propagate_filter_state(&mut filter_state, &curr, &next, &noise);
            assert!(propagated.is_some(), "static propagation step should succeed");
        }

        assert!(filter_state.imu_state.position.norm() < 1e-9);
        assert!(filter_state.imu_state.velocity.norm() < 1e-9);
        assert!(filter_state.imu_state.orientation.angle() < 1e-9);
        assert_eq!(filter_state.covariance.nrows(), 15);
        assert_eq!(filter_state.covariance.ncols(), 15);
    }
}