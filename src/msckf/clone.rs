use crate::{
    msckf::state::{CameraClone, FilterState, CLONE_DOF, IMU_STATE_DOF},
    types::{DMatrix, Float, SMatrix, UnitQuaternion, Vector3},
};

pub const MAX_CLONES: usize = 8;

#[derive(Debug, Clone, Copy)]
pub struct CloneConfig {
    pub min_time_between_clones_s: Float,
    pub min_rotation_rad: Float,
    pub min_translation_m: Float,
    pub max_clones: usize,
}

impl Default for CloneConfig {
    fn default() -> Self {
        Self {
            min_time_between_clones_s: 0.15,
            min_rotation_rad: 6.0_f64.to_radians() as Float,
            min_translation_m: 0.15,
            max_clones: MAX_CLONES,
        }
    }
}

type CloneJacobian = SMatrix<6, 15>;

#[must_use]
fn skew(v: &Vector3) -> crate::types::Matrix3x3 {
    crate::types::Matrix3x3::new(0.0, -v.z, v.y, v.z, 0.0, -v.x, -v.y, v.x, 0.0)
}

#[must_use]
fn next_clone_id(filter_state: &FilterState) -> usize {
    filter_state
        .clones
        .last()
        .map_or(0, |last_clone| last_clone.clone_id + 1)
}

#[must_use]
fn clone_jacobian(camera_lever_arm_world: &Vector3) -> CloneJacobian {
    let mut jacobian = CloneJacobian::zeros();
    jacobian.fixed_view_mut::<3, 3>(0, 0).fill_diagonal(1.0);
    jacobian
        .fixed_view_mut::<3, 3>(3, 0)
        .copy_from(&(-skew(camera_lever_arm_world)));
    jacobian.fixed_view_mut::<3, 3>(3, 6).fill_diagonal(1.0);
    jacobian
}

#[must_use]
fn remove_matrix_block(matrix: &DMatrix, start: usize, size: usize) -> DMatrix {
    let new_dim = matrix.nrows().saturating_sub(size);
    let mut reduced = DMatrix::zeros(new_dim, new_dim);
    let mut dst_row = 0;

    for src_row in 0..matrix.nrows() {
        if (start..start + size).contains(&src_row) {
            continue;
        }

        let mut dst_col = 0;
        for src_col in 0..matrix.ncols() {
            if (start..start + size).contains(&src_col) {
                continue;
            }
            reduced[(dst_row, dst_col)] = matrix[(src_row, src_col)];
            dst_col += 1;
        }

        dst_row += 1;
    }

    reduced
}

#[must_use]
pub fn should_create_clone(
    filter_state: &FilterState,
    timestamp_s: Float,
    orientation: &UnitQuaternion,
    position: &Vector3,
    config: &CloneConfig,
) -> bool {
    let Some(last_clone) = filter_state.clones.last() else {
        return true;
    };

    let delta_t = timestamp_s - last_clone.timestamp_s;
    if delta_t >= config.min_time_between_clones_s {
        return true;
    }

    let relative_rotation = last_clone.orientation.inverse() * *orientation;
    if relative_rotation.angle() >= config.min_rotation_rad {
        return true;
    }

    let translation_delta = *position - last_clone.position;
    translation_delta.norm() >= config.min_translation_m
}

pub fn augment_clone_covariance(filter_state: &mut FilterState, camera_lever_arm_world: &Vector3) {
    filter_state.resize_covariance_for_clones();

    let state_dim = filter_state.state_dim();
    let clone_jacobian = clone_jacobian(camera_lever_arm_world);
    let imu_covariance = filter_state
        .covariance
        .view((0, 0), (IMU_STATE_DOF, IMU_STATE_DOF))
        .into_owned();
    let existing_to_clone = filter_state
        .covariance
        .columns(0, IMU_STATE_DOF)
        .into_owned()
        * clone_jacobian.transpose();
    let clone_covariance = clone_jacobian * imu_covariance * clone_jacobian.transpose();

    let mut augmented = DMatrix::zeros(state_dim + CLONE_DOF, state_dim + CLONE_DOF);
    augmented
        .view_mut((0, 0), (state_dim, state_dim))
        .copy_from(&filter_state.covariance);
    augmented
        .view_mut((0, state_dim), (state_dim, CLONE_DOF))
        .copy_from(&existing_to_clone);
    augmented
        .view_mut((state_dim, 0), (CLONE_DOF, state_dim))
        .copy_from(&existing_to_clone.transpose());
    augmented
        .view_mut((state_dim, state_dim), (CLONE_DOF, CLONE_DOF))
        .copy_from(&clone_covariance);

    filter_state.covariance = augmented;
}

pub fn marginalize_oldest_clone(filter_state: &mut FilterState) -> Option<CameraClone> {
    if filter_state.clones.is_empty() {
        return None;
    }

    // In covariance form, exact Gaussian marginalization is the principal submatrix of the
    // retained variables, equivalent to Schur-complement elimination in information form.
    let removed_clone = filter_state.clones.remove(0);
    filter_state.resize_covariance_for_clones();
    filter_state.covariance = remove_matrix_block(&filter_state.covariance, IMU_STATE_DOF, CLONE_DOF);
    Some(removed_clone)
}

#[must_use]
pub fn add_camera_clone(
    filter_state: &mut FilterState,
    timestamp_s: Float,
    orientation: UnitQuaternion,
    position: Vector3,
    camera_lever_arm_world: Vector3,
    config: &CloneConfig,
) -> Option<usize> {
    if !should_create_clone(filter_state, timestamp_s, &orientation, &position, config) {
        return None;
    }

    while filter_state.clones.len() >= config.max_clones {
        let _ = marginalize_oldest_clone(filter_state);
    }

    let clone_id = next_clone_id(filter_state);
    let clone = CameraClone::new(clone_id, timestamp_s, orientation, position);

    augment_clone_covariance(filter_state, &camera_lever_arm_world);
    filter_state.clones.push(clone);
    Some(clone_id)
}

#[cfg(test)]
mod tests {
    use super::{add_camera_clone, CloneConfig, MAX_CLONES};
    use crate::msckf::state::{FilterState, CLONE_DOF, IMU_STATE_DOF};
    use crate::types::{UnitQuaternion, Vector3};

    #[test]
    fn add_ten_clones_never_exceeds_sixty_three_state_dofs() {
        let mut filter_state = FilterState::new();
        let config = CloneConfig::default();

        for clone_idx in 0..10 {
            filter_state.imu_state.position.x = clone_idx as f64 * 0.1;
            let position = filter_state.imu_state.position;
            let clone_id = add_camera_clone(
                &mut filter_state,
                clone_idx as f64 * 0.1,
                UnitQuaternion::identity(),
                position,
                Vector3::zeros(),
                &config,
            );
            assert!(clone_id.is_some(), "clone insertion should trigger for each synthetic step");
            assert!(filter_state.clones.len() <= MAX_CLONES);
            assert!(filter_state.state_dim() <= IMU_STATE_DOF + MAX_CLONES * CLONE_DOF);
            assert_eq!(filter_state.covariance.nrows(), filter_state.state_dim());
            assert_eq!(filter_state.covariance.ncols(), filter_state.state_dim());
        }

        assert_eq!(filter_state.clones.len(), MAX_CLONES);
        assert_eq!(filter_state.state_dim(), 63);
    }
}