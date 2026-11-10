use crate::{
    msckf::{CameraClone, FeatureTrack, StereoRig},
    types::{DMatrix, DVector, Float, Matrix3x3, Vector2, Vector3},
};
use rustc_hash::FxHashMap;

#[derive(Debug, Clone, Copy)]
pub struct EquidistantIntrinsics {
    pub fx: Float,
    pub fy: Float,
    pub cx: Float,
    pub cy: Float,
    pub k: [Float; 4],
}

impl Default for EquidistantIntrinsics {
    fn default() -> Self {
        Self {
            fx: 190.978_477_151_287_17,
            fy: 190.973_307_052_122_6,
            cx: 254.931_706_059_354_75,
            cy: 256.897_442_899_650_4,
            k: [
                0.003_482_389_402_249_343_4,
                0.000_715_034_845_216_225_7,
                -0.002_053_236_141_870_620_2,
                0.000_202_936_735_918_111_82,
            ],
        }
    }
}

#[derive(Debug, Clone)]
pub struct FisheyeObservationModel {
    pub left: EquidistantIntrinsics,
    pub right: EquidistantIntrinsics,
}

impl Default for FisheyeObservationModel {
    fn default() -> Self {
        Self {
            left: EquidistantIntrinsics::default(),
            right: EquidistantIntrinsics::default(),
        }
    }
}

#[derive(Debug, Clone)]
pub struct TrackResidual {
    pub clone_ids: Vec<usize>,
    pub residual: DVector,
    pub h_x: DMatrix,
    pub h_f: DMatrix,
}

#[must_use]
fn skew(v: &Vector3) -> Matrix3x3 {
    Matrix3x3::new(0.0, -v.z, v.y, v.z, 0.0, -v.x, -v.y, v.x, 0.0)
}

#[must_use]
fn camera_point(clone: &CameraClone, point_world: &Vector3) -> (Matrix3x3, Vector3) {
    let rotation_cw = clone.orientation.to_rotation_matrix().into_inner().transpose();
    let translation_cw = -rotation_cw * clone.position;
    let point_camera = rotation_cw * point_world + translation_cw;
    (rotation_cw, point_camera)
}

#[must_use]
fn right_camera_point(
    clone: &CameraClone,
    point_world: &Vector3,
    rig: &StereoRig,
) -> (Matrix3x3, Vector3) {
    let (rotation_lw, point_left) = camera_point(clone, point_world);
    (
        rig.right_from_left_rotation * rotation_lw,
        rig.right_from_left_rotation * point_left + rig.right_from_left_translation,
    )
}

#[must_use]
fn radial_distortion(theta: Float, intrinsics: &EquidistantIntrinsics) -> Float {
    let theta2 = theta * theta;
    let theta4 = theta2 * theta2;
    let theta6 = theta4 * theta2;
    let theta8 = theta4 * theta4;
    theta
        * (1.0
            + intrinsics.k[0] * theta2
            + intrinsics.k[1] * theta4
            + intrinsics.k[2] * theta6
            + intrinsics.k[3] * theta8)
}

#[must_use]
fn radial_distortion_derivative(theta: Float, intrinsics: &EquidistantIntrinsics) -> Float {
    let theta2 = theta * theta;
    let theta4 = theta2 * theta2;
    let theta6 = theta4 * theta2;
    let theta8 = theta4 * theta4;
    1.0
        + 3.0 * intrinsics.k[0] * theta2
        + 5.0 * intrinsics.k[1] * theta4
        + 7.0 * intrinsics.k[2] * theta6
        + 9.0 * intrinsics.k[3] * theta8
}

impl EquidistantIntrinsics {
    #[must_use]
    pub fn project(&self, point_camera: &Vector3) -> Option<Vector2> {
        let (measurement, _) = self.project_with_jacobian(point_camera)?;
        Some(measurement)
    }

    #[must_use]
    pub fn project_with_jacobian(&self, point_camera: &Vector3) -> Option<(Vector2, Matrix3x3)> {
        if point_camera.z <= 1e-9 || !point_camera.iter().all(|value| value.is_finite()) {
            return None;
        }

        let x = point_camera.x;
        let y = point_camera.y;
        let z = point_camera.z;
        let a = x / z;
        let b = y / z;
        let r2 = a * a + b * b;
        let r = r2.sqrt();

        let da_dp = Vector3::new(1.0 / z, 0.0, -x / (z * z));
        let db_dp = Vector3::new(0.0, 1.0 / z, -y / (z * z));

        if r < 1e-9 {
            let measurement = Vector2::new(self.fx * a + self.cx, self.fy * b + self.cy);
            let mut jacobian = Matrix3x3::zeros();
            jacobian.row_mut(0).copy_from(&(da_dp.transpose() * self.fx));
            jacobian.row_mut(1).copy_from(&(db_dp.transpose() * self.fy));
            return Some((measurement, jacobian));
        }

        let theta = r.atan();
        let r_d = radial_distortion(theta, self);
        let dr_d_dtheta = radial_distortion_derivative(theta, self);
        let dtheta_dr = 1.0 / (1.0 + r2);
        let dr_d_dr = dr_d_dtheta * dtheta_dr;
        let scale = r_d / r;

        let dr_da = a / r;
        let dr_db = b / r;
        let dscale_da = dr_da * (dr_d_dr * r - r_d) / r2;
        let dscale_db = dr_db * (dr_d_dr * r - r_d) / r2;

        let du_da = self.fx * (scale + a * dscale_da);
        let du_db = self.fx * (a * dscale_db);
        let dv_da = self.fy * (b * dscale_da);
        let dv_db = self.fy * (scale + b * dscale_db);

        let measurement = Vector2::new(self.fx * a * scale + self.cx, self.fy * b * scale + self.cy);
        let du_dp = da_dp * du_da + db_dp * du_db;
        let dv_dp = da_dp * dv_da + db_dp * dv_db;
        let mut jacobian = Matrix3x3::zeros();
        jacobian.row_mut(0).copy_from(&du_dp.transpose());
        jacobian.row_mut(1).copy_from(&dv_dp.transpose());

        Some((measurement, jacobian))
    }
}

fn append_measurement_block(
    residual_values: &mut Vec<Float>,
    h_x: &mut DMatrix,
    h_f: &mut DMatrix,
    row_offset: usize,
    clone_index: usize,
    observation: Vector2,
    prediction: Vector2,
    projection_jacobian: &Matrix3x3,
    rotation_cw: &Matrix3x3,
    point_camera: &Vector3,
) {
    residual_values.push(prediction.x - observation.x);
    residual_values.push(prediction.y - observation.y);

    let point_jacobian = projection_jacobian.fixed_view::<2, 3>(0, 0).into_owned();
    let pose_rotation = point_jacobian * skew(point_camera);
    let pose_translation = point_jacobian * (-rotation_cw);
    let landmark_jacobian = point_jacobian * rotation_cw;
    let clone_column = clone_index * 6;

    h_x.view_mut((row_offset, clone_column), (2, 3)).copy_from(&pose_rotation);
    h_x.view_mut((row_offset, clone_column + 3), (2, 3)).copy_from(&pose_translation);
    h_f.view_mut((row_offset, 0), (2, 3)).copy_from(&landmark_jacobian);
}

#[must_use]
pub fn build_track_residual(
    track: &FeatureTrack,
    point_world: &Vector3,
    clones: &[CameraClone],
    rig: &StereoRig,
    camera_model: &FisheyeObservationModel,
) -> Option<TrackResidual> {
    let clones_by_id = clones
        .iter()
        .cloned()
        .map(|clone| (clone.clone_id, clone))
        .collect::<FxHashMap<_, _>>();
    let valid_observations = track
        .observations
        .iter()
        .filter_map(|observation| {
            let clone = clones_by_id.get(&observation.clone_id)?;
            let (rotation_lw, point_left) = camera_point(clone, point_world);
            let (left_prediction, left_jacobian) = camera_model.left.project_with_jacobian(&point_left)?;
            let (rotation_rw, point_right) = right_camera_point(clone, point_world, rig);
            let (right_prediction, right_jacobian) = camera_model.right.project_with_jacobian(&point_right)?;
            Some((
                observation,
                observation.clone_id,
                rotation_lw,
                point_left,
                left_prediction,
                left_jacobian,
                rotation_rw,
                point_right,
                right_prediction,
                right_jacobian,
            ))
        })
        .collect::<Vec<_>>();

    if valid_observations.len() < 2 {
        return None;
    }

    let clone_ids = valid_observations
        .iter()
        .map(|(_, clone_id, ..)| *clone_id)
        .collect::<Vec<_>>();
    let mut residual_values = Vec::with_capacity(valid_observations.len() * 4);
    let mut h_x = DMatrix::zeros(valid_observations.len() * 4, valid_observations.len() * 6);
    let mut h_f = DMatrix::zeros(valid_observations.len() * 4, 3);

    for (
        obs_index,
        (
            observation,
            _,
            rotation_lw,
            point_left,
            left_prediction,
            left_jacobian,
            rotation_rw,
            point_right,
            right_prediction,
            right_jacobian,
        ),
    ) in valid_observations.iter().enumerate()
    {
        let row_offset = obs_index * 4;

        append_measurement_block(
            &mut residual_values,
            &mut h_x,
            &mut h_f,
            row_offset,
            obs_index,
            observation.left_pixel,
            *left_prediction,
            left_jacobian,
            rotation_lw,
            point_left,
        );

        append_measurement_block(
            &mut residual_values,
            &mut h_x,
            &mut h_f,
            row_offset + 2,
            obs_index,
            observation.right_pixel,
            *right_prediction,
            right_jacobian,
            rotation_rw,
            point_right,
        );
    }

    Some(TrackResidual {
        clone_ids,
        residual: DVector::from_vec(residual_values),
        h_x,
        h_f,
    })
}

#[cfg(test)]
mod tests {
    use super::{build_track_residual, EquidistantIntrinsics, FisheyeObservationModel};
    use crate::{
        msckf::{CameraClone, FeatureTrack, StereoObservation, StereoRig},
        types::{Matrix3x3, UnitQuaternion, Vector2, Vector3},
    };

    fn finite_difference_jacobian(
        intrinsics: &EquidistantIntrinsics,
        point: &Vector3,
        epsilon: f64,
    ) -> Matrix3x3 {
        let mut jacobian = Matrix3x3::zeros();
        for column in 0..3 {
            let mut plus = *point;
            let mut minus = *point;
            plus[column] += epsilon;
            minus[column] -= epsilon;
            let projected_plus = intrinsics.project(&plus).unwrap();
            let projected_minus = intrinsics.project(&minus).unwrap();
            let derivative = (projected_plus - projected_minus) / (2.0 * epsilon);
            jacobian[(0, column)] = derivative.x;
            jacobian[(1, column)] = derivative.y;
        }
        jacobian
    }

    #[test]
    fn equidistant_projection_jacobian_matches_finite_difference() {
        let intrinsics = EquidistantIntrinsics::default();
        let point = Vector3::new(0.12, -0.08, 1.5);
        let (_, analytic) = intrinsics.project_with_jacobian(&point).unwrap();
        let numeric = finite_difference_jacobian(&intrinsics, &point, 1e-6);
        let diff = analytic.fixed_view::<2, 3>(0, 0) - numeric.fixed_view::<2, 3>(0, 0);
        assert!(diff.norm() < 1e-4, "analytic jacobian diverged from numeric jacobian: {}", diff.norm());
    }

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
            feature_id: 21,
            left_pixel,
            right_pixel,
            left_normalized: Vector2::new(point_left.x / point_left.z, point_left.y / point_left.z),
            right_normalized: Vector2::new(point_right.x / point_right.z, point_right.y / point_right.z),
            left_age: clone.clone_id as u32,
            right_age: clone.clone_id as u32,
        }
    }

    #[test]
    fn track_residual_dimensions_match_clone_observations() {
        let rig = StereoRig::new(Matrix3x3::identity(), Vector3::new(-0.2, 0.0, 0.0));
        let camera_model = FisheyeObservationModel::default();
        let point_world = Vector3::new(0.4, 0.05, 4.0);
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
            feature_id: 21,
            observations,
            total_age: 3,
            missed_frames: 0,
            last_seen_clone_id: Some(2),
        };

        let residual = build_track_residual(&track, &point_world, &clones, &rig, &camera_model).unwrap();
        assert_eq!(residual.residual.nrows(), 12);
        assert_eq!(residual.h_x.nrows(), 12);
        assert_eq!(residual.h_x.ncols(), 18);
        assert_eq!(residual.h_f.nrows(), 12);
        assert_eq!(residual.h_f.ncols(), 3);
        assert!(residual.residual.norm() < 1e-9);
    }
}