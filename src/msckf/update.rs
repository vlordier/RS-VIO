use crate::{
    msckf::{
        nullspace_project, FilterState, NullspaceProjection, TrackResidual, CLONE_DOF,
        IMU_STATE_DOF,
    },
    types::{DMatrix, DVector, Float, UnitQuaternion, Vector3},
};
use rustc_hash::FxHashMap;

#[derive(Debug, Clone, Copy)]
pub struct UpdateConfig {
    pub measurement_noise_variance: Float,
    pub gate_probability: Float,
}

impl Default for UpdateConfig {
    fn default() -> Self {
        Self {
            measurement_noise_variance: 4.0,
            gate_probability: 0.99,
        }
    }
}

#[derive(Debug, Clone)]
pub struct GatedTrackUpdate {
    pub clone_ids: Vec<usize>,
    pub projected_residual: DVector,
    pub projected_h: DMatrix,
    pub measurement_covariance: DMatrix,
    pub innovation_statistic: Float,
}

#[derive(Debug, Clone)]
pub struct BatchUpdate {
    pub residual: DVector,
    pub h: DMatrix,
    pub measurement_covariance: DMatrix,
    pub accepted_tracks: usize,
    pub rejected_tracks: usize,
}

#[must_use]
fn inverse_standard_normal_cdf(probability: Float) -> Float {
    let p = probability.clamp(1.0e-6, 1.0 - 1.0e-6);
    let a: [Float; 6] = [
        -3.969_683_028_665_376e+01,
        2.209_460_984_245_205e+02,
        -2.759_285_104_469_687e+02,
        1.383_577_518_672_69e+02,
        -3.066_479_806_614_716e+01,
        2.506_628_277_459_239e+00,
    ];
    let b: [Float; 5] = [
        -5.447_609_879_822_406e+01,
        1.615_858_368_580_409e+02,
        -1.556_989_798_598_866e+02,
        6.680_131_188_771_972e+01,
        -1.328_068_155_288_572e+01,
    ];
    let c: [Float; 6] = [
        -7.784_894_002_430_293e-03,
        -3.223_964_580_411_365e-01,
        -2.400_758_277_161_838e+00,
        -2.549_732_539_343_734e+00,
        4.374_664_141_464_968e+00,
        2.938_163_982_698_783e+00,
    ];
    let d: [Float; 4] = [
        7.784_695_709_041_462e-03,
        3.224_671_290_700_398e-01,
        2.445_134_137_142_996e+00,
        3.754_408_661_907_416e+00,
    ];
    let p_low = 0.024_25;
    let p_high = 1.0 - p_low;

    if p < p_low {
        let q = (-2.0 * p.ln()).sqrt();
        return (((((c[0] * q + c[1]) * q + c[2]) * q + c[3]) * q + c[4]) * q + c[5])
            / ((((d[0] * q + d[1]) * q + d[2]) * q + d[3]) * q + 1.0);
    }

    if p <= p_high {
        let q = p - 0.5;
        let r = q * q;
        return (((((a[0] * r + a[1]) * r + a[2]) * r + a[3]) * r + a[4]) * r + a[5]) * q
            / (((((b[0] * r + b[1]) * r + b[2]) * r + b[3]) * r + b[4]) * r + 1.0);
    }

    let q = (-2.0 * (1.0 - p).ln()).sqrt();
    -(((((c[0] * q + c[1]) * q + c[2]) * q + c[3]) * q + c[4]) * q + c[5])
        / ((((d[0] * q + d[1]) * q + d[2]) * q + d[3]) * q + 1.0)
}

#[must_use]
fn chi_square_threshold(dof: usize, probability: Float) -> Float {
    let dof = dof.max(1) as Float;
    let z = inverse_standard_normal_cdf(probability);
    let correction = 1.0 - 2.0 / (9.0 * dof) + z * (2.0 / (9.0 * dof)).sqrt();
    dof * correction.powi(3)
}

#[must_use]
fn clone_column_lookup(filter_state: &FilterState) -> FxHashMap<usize, usize> {
    filter_state
        .clones
        .iter()
        .enumerate()
        .map(|(index, clone)| (clone.clone_id, IMU_STATE_DOF + index * CLONE_DOF))
        .collect()
}

#[must_use]
pub fn project_track_into_filter_state(
    filter_state: &FilterState,
    track_residual: &TrackResidual,
) -> Option<(NullspaceProjection, DMatrix)> {
    let projection = nullspace_project(
        &track_residual.h_x,
        &track_residual.h_f,
        &track_residual.residual,
    )?;
    let mut global_h = DMatrix::zeros(projection.projected_h_x.nrows(), filter_state.state_dim());
    let clone_columns = clone_column_lookup(filter_state);

    for (obs_index, clone_id) in track_residual.clone_ids.iter().enumerate() {
        let source_col = obs_index * CLONE_DOF;
        let target_col = *clone_columns.get(clone_id)?;
        let block = projection
            .projected_h_x
            .view((0, source_col), (projection.projected_h_x.nrows(), CLONE_DOF))
            .into_owned();
        let existing = global_h
            .view((0, target_col), (global_h.nrows(), CLONE_DOF))
            .into_owned();
        global_h
            .view_mut((0, target_col), (global_h.nrows(), CLONE_DOF))
            .copy_from(&(existing + block));
    }

    Some((projection, global_h))
}

#[must_use]
fn innovation_covariance(
    filter_state: &FilterState,
    h: &DMatrix,
    measurement_covariance: &DMatrix,
) -> DMatrix {
    h * &filter_state.covariance * h.transpose() + measurement_covariance
}

#[must_use]
pub fn chi_square_gate_track(
    filter_state: &FilterState,
    track_residual: &TrackResidual,
    config: &UpdateConfig,
) -> Option<GatedTrackUpdate> {
    let (projection, global_h) = project_track_into_filter_state(filter_state, track_residual)?;
    if projection.projected_residual.is_empty() {
        return None;
    }

    let measurement_covariance =
        DMatrix::identity(projection.projected_residual.nrows(), projection.projected_residual.nrows())
            * config.measurement_noise_variance;
    let innovation = innovation_covariance(filter_state, &global_h, &measurement_covariance);
    let innovation_inverse = innovation.try_inverse()?;
    let gamma_matrix = projection.projected_residual.transpose()
        * &innovation_inverse
        * &projection.projected_residual;
    let innovation_statistic = gamma_matrix[(0, 0)];
    let gate_threshold = chi_square_threshold(
        projection.projected_residual.nrows(),
        config.gate_probability,
    );

    (innovation_statistic <= gate_threshold).then_some(GatedTrackUpdate {
        clone_ids: track_residual.clone_ids.clone(),
        projected_residual: projection.projected_residual,
        projected_h: global_h,
        measurement_covariance,
        innovation_statistic,
    })
}

#[must_use]
pub fn batch_stack_gated_updates(
    accepted_updates: &[GatedTrackUpdate],
    state_dim: usize,
) -> Option<BatchUpdate> {
    if accepted_updates.is_empty() {
        return None;
    }

    let total_rows = accepted_updates
        .iter()
        .map(|update| update.projected_residual.nrows())
        .sum::<usize>();
    let mut residual = DVector::zeros(total_rows);
    let mut h = DMatrix::zeros(total_rows, state_dim);
    let mut measurement_covariance = DMatrix::zeros(total_rows, total_rows);
    let mut row_offset = 0;

    for update in accepted_updates {
        let rows = update.projected_residual.nrows();
        residual
            .rows_mut(row_offset, rows)
            .copy_from(&update.projected_residual);
        h.view_mut((row_offset, 0), (rows, state_dim))
            .copy_from(&update.projected_h);
        measurement_covariance
            .view_mut((row_offset, row_offset), (rows, rows))
            .copy_from(&update.measurement_covariance);
        row_offset += rows;
    }

    Some(BatchUpdate {
        residual,
        h,
        measurement_covariance,
        accepted_tracks: accepted_updates.len(),
        rejected_tracks: 0,
    })
}

pub fn apply_error_state_correction(filter_state: &mut FilterState, delta: &DVector) -> Option<()> {
    if delta.nrows() != filter_state.state_dim() {
        return None;
    }

    let imu_rotation = Vector3::new(delta[0], delta[1], delta[2]);
    let imu_velocity = Vector3::new(delta[3], delta[4], delta[5]);
    let imu_position = Vector3::new(delta[6], delta[7], delta[8]);
    let gyro_bias = Vector3::new(delta[9], delta[10], delta[11]);
    let accel_bias = Vector3::new(delta[12], delta[13], delta[14]);

    filter_state.imu_state.orientation =
        UnitQuaternion::from_scaled_axis(imu_rotation) * filter_state.imu_state.orientation;
    filter_state.imu_state.velocity += imu_velocity;
    filter_state.imu_state.position += imu_position;
    filter_state.imu_state.gyro_bias += gyro_bias;
    filter_state.imu_state.accel_bias += accel_bias;

    for (clone_index, clone) in filter_state.clones.iter_mut().enumerate() {
        let start = IMU_STATE_DOF + clone_index * CLONE_DOF;
        let clone_rotation = Vector3::new(delta[start], delta[start + 1], delta[start + 2]);
        let clone_position = Vector3::new(delta[start + 3], delta[start + 4], delta[start + 5]);
        clone.orientation = UnitQuaternion::from_scaled_axis(clone_rotation) * clone.orientation;
        clone.position += clone_position;
    }

    Some(())
}

pub fn ekf_update(
    filter_state: &mut FilterState,
    batch_update: &BatchUpdate,
    _config: &UpdateConfig,
) -> Option<DVector> {
    if batch_update.h.nrows() == 0 || batch_update.h.ncols() != filter_state.state_dim() {
        return None;
    }

    let covariance = filter_state.covariance.clone();
    let innovation = &batch_update.h * &covariance * batch_update.h.transpose()
        + &batch_update.measurement_covariance;
    let innovation_inverse = innovation.try_inverse()?;
    let kalman_gain = &covariance * batch_update.h.transpose() * innovation_inverse;
    let delta = -&kalman_gain * &batch_update.residual;
    apply_error_state_correction(filter_state, &delta)?;

    let identity = DMatrix::identity(filter_state.state_dim(), filter_state.state_dim());
    let correction = &identity - &kalman_gain * &batch_update.h;
    filter_state.covariance = &correction * covariance * correction.transpose()
        + &kalman_gain * &batch_update.measurement_covariance * kalman_gain.transpose();
    Some(delta)
}

#[cfg(test)]
mod tests {
    use super::{
        batch_stack_gated_updates, chi_square_gate_track, ekf_update, BatchUpdate,
        GatedTrackUpdate, UpdateConfig,
    };
    use crate::{
        msckf::{
            build_track_residual, CameraClone, FeatureTrack, FilterState, FisheyeObservationModel,
            StereoObservation, StereoRig,
        },
        types::{DMatrix, DVector, Matrix3x3, UnitQuaternion, Vector2, Vector3},
    };

    fn make_clone(clone_id: usize, position: Vector3) -> CameraClone {
        CameraClone::new(clone_id, clone_id as f64 * 0.1, UnitQuaternion::identity(), position)
    }

    fn make_observation(
        clone: &CameraClone,
        point_world: Vector3,
        rig: &StereoRig,
        camera_model: &FisheyeObservationModel,
    ) -> StereoObservation {
        let rotation_cw = clone.orientation.to_rotation_matrix().into_inner().transpose();
        let translation_cw = -rotation_cw * clone.position;
        let point_left = rotation_cw * point_world + translation_cw;
        let point_right = rig.right_from_left_rotation * point_left + rig.right_from_left_translation;
        let left_pixel = camera_model.left.project(&point_left).unwrap();
        let right_pixel = camera_model.right.project(&point_right).unwrap();

        StereoObservation {
            clone_id: clone.clone_id,
            timestamp_s: clone.timestamp_s,
            feature_id: 41,
            left_pixel,
            right_pixel,
            left_normalized: Vector2::new(point_left.x / point_left.z, point_left.y / point_left.z),
            right_normalized: Vector2::new(point_right.x / point_right.z, point_right.y / point_right.z),
            left_age: clone.clone_id as u32,
            right_age: clone.clone_id as u32,
        }
    }

    #[test]
    fn synthetic_track_update_moves_state_toward_measurement() {
        let mut filter_state = FilterState::new();
        filter_state.imu_state.position = Vector3::new(1.0, 0.0, 0.0);
        filter_state.covariance = DMatrix::identity(filter_state.state_dim(), filter_state.state_dim());

        let residual = DVector::from_vec(vec![1.0]);
        let mut h = DMatrix::zeros(1, filter_state.state_dim());
        h[(0, 6)] = 1.0;
        let measurement_covariance = DMatrix::identity(1, 1) * 1e-6;
        let batch = BatchUpdate {
            residual,
            h,
            measurement_covariance,
            accepted_tracks: 1,
            rejected_tracks: 0,
        };

        let delta = ekf_update(&mut filter_state, &batch, &UpdateConfig::default()).unwrap();
        assert!(delta[6] < 0.0);
        assert!(filter_state.imu_state.position.x.abs() < 1e-4);
        assert!(filter_state.covariance[(6, 6)] < 1.0);
    }

    #[test]
    fn batch_stacking_preserves_row_counts() {
        let update_a = GatedTrackUpdate {
            clone_ids: vec![0],
            projected_residual: DVector::from_vec(vec![1.0, 2.0]),
            projected_h: DMatrix::identity(2, 5),
            measurement_covariance: DMatrix::identity(2, 2),
            innovation_statistic: 1.0,
        };
        let update_b = GatedTrackUpdate {
            clone_ids: vec![1],
            projected_residual: DVector::from_vec(vec![3.0]),
            projected_h: DMatrix::from_row_slice(1, 5, &[0.0, 1.0, 0.0, 0.0, 0.0]),
            measurement_covariance: DMatrix::identity(1, 1) * 2.0,
            innovation_statistic: 0.5,
        };

        let batch = batch_stack_gated_updates(&[update_a, update_b], 5).unwrap();
        assert_eq!(batch.residual.nrows(), 3);
        assert_eq!(batch.h.nrows(), 3);
        assert_eq!(batch.h.ncols(), 5);
        assert_eq!(batch.measurement_covariance.nrows(), 3);
        assert_eq!(batch.accepted_tracks, 2);
    }

    #[test]
    fn chi_square_gate_accepts_small_projected_residual() {
        let rig = StereoRig::new(Matrix3x3::identity(), Vector3::new(-0.2, 0.0, 0.0));
        let camera_model = FisheyeObservationModel::default();
        let point_world = Vector3::new(0.2, 0.03, 4.0);
        let clones = vec![
            make_clone(0, Vector3::new(0.0, 0.0, 0.0)),
            make_clone(1, Vector3::new(0.1, 0.0, 0.0)),
            make_clone(2, Vector3::new(0.2, 0.0, 0.0)),
        ];
        let observations = clones
            .iter()
            .map(|clone| make_observation(clone, point_world, &rig, &camera_model))
            .collect::<Vec<_>>();
        let track = FeatureTrack {
            feature_id: 41,
            observations,
            total_age: 3,
            missed_frames: 0,
            last_seen_clone_id: Some(2),
        };

        let track_residual = build_track_residual(&track, &point_world, &clones, &rig, &camera_model).unwrap();
        let mut filter_state = FilterState::new();
        filter_state.clones = clones.clone();
        filter_state.resize_covariance_for_clones();
        filter_state.covariance = DMatrix::identity(filter_state.state_dim(), filter_state.state_dim()) * 0.1;

        let gated = chi_square_gate_track(&filter_state, &track_residual, &UpdateConfig::default()).unwrap();
        let gate_threshold = super::chi_square_threshold(
            gated.projected_residual.nrows(),
            UpdateConfig::default().gate_probability,
        );
        assert!(gated.innovation_statistic <= gate_threshold);
    }
}