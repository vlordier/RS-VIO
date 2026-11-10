use crate::{
    constants::tolerance,
    msckf::{CameraClone, FeatureTrack, StereoObservation},
    types::{DMatrix, Float, Matrix3x3, Matrix4x4, Vector2, Vector3},
};
use nalgebra::Vector4;
use rustc_hash::FxHashMap;

#[derive(Debug, Clone)]
pub struct StereoRig {
    /// Relative transform from left to right camera: x_r = R_rl * x_l + t_rl.
    pub right_from_left_rotation: Matrix3x3,
    /// Translation of the left camera origin expressed in the right camera frame.
    pub right_from_left_translation: Vector3,
}

impl StereoRig {
    #[must_use]
    pub fn new(right_from_left_rotation: Matrix3x3, right_from_left_translation: Vector3) -> Self {
        Self {
            right_from_left_rotation,
            right_from_left_translation,
        }
    }
}

impl Default for StereoRig {
    fn default() -> Self {
        Self {
            right_from_left_rotation: Matrix3x3::identity(),
            right_from_left_translation: Vector3::new(-0.1, 0.0, 0.0),
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct TriangulationConfig {
    pub max_condition_number: Float,
    pub min_depth_m: Float,
    pub max_depth_m: Float,
    pub max_reprojection_rmse: Float,
}

impl Default for TriangulationConfig {
    fn default() -> Self {
        Self {
            max_condition_number: 10_000.0,
            min_depth_m: tolerance::MIN_DEPTH as Float,
            max_depth_m: tolerance::MAX_DEPTH as Float,
            max_reprojection_rmse: 2.0,
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct TriangulatedLandmark {
    pub position_world: Vector3,
    pub stereo_seed_world: Vector3,
    pub reprojection_rmse: Float,
    pub condition_number: Float,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TriangulationFailure {
    StereoSeed,
    DltSystem,
    DltSolve,
    ConditionNumber,
    PositiveDepth,
    Reprojection,
}

#[derive(Debug, Clone, PartialEq)]
pub struct TriangulationResult {
    pub landmark: Option<TriangulatedLandmark>,
    pub failure: Option<TriangulationFailure>,
}

#[must_use]
fn projection_matrix(clone: &CameraClone) -> Matrix4x4 {
    let rotation_wc = clone.orientation.to_rotation_matrix().into_inner();
    let rotation_cw = rotation_wc.transpose();
    let translation_cw = -rotation_cw * clone.position;

    let mut projection = Matrix4x4::zeros();
    projection.fixed_view_mut::<3, 3>(0, 0).copy_from(&rotation_cw);
    projection.fixed_view_mut::<3, 1>(0, 3).copy_from(&translation_cw);
    projection[(3, 3)] = 1.0;
    projection
}

#[must_use]
fn right_projection_matrix(left_projection: &Matrix4x4, rig: &StereoRig) -> Matrix4x4 {
    let r_lw = left_projection.fixed_view::<3, 3>(0, 0).into_owned();
    let t_lw = left_projection.fixed_view::<3, 1>(0, 3).into_owned();
    let r_rw = rig.right_from_left_rotation * r_lw;
    let t_rw = rig.right_from_left_rotation * t_lw + rig.right_from_left_translation;

    let mut projection = Matrix4x4::zeros();
    projection.fixed_view_mut::<3, 3>(0, 0).copy_from(&r_rw);
    projection.fixed_view_mut::<3, 1>(0, 3).copy_from(&t_rw);
    projection[(3, 3)] = 1.0;
    projection
}

#[must_use]
fn project_normalized(projection: &Matrix4x4, point_world: &Vector3) -> Option<Vector2> {
    let point_h = Vector4::new(point_world.x, point_world.y, point_world.z, 1.0);
    let point_camera = projection.fixed_view::<3, 4>(0, 0) * point_h;
    if point_camera[2] <= 0.0 || !point_camera.iter().all(|value| value.is_finite()) {
        return None;
    }

    Some(Vector2::new(
        point_camera[0] / point_camera[2],
        point_camera[1] / point_camera[2],
    ))
}

#[must_use]
fn stereo_triangulate_in_left_frame(
    observation: &StereoObservation,
    rig: &StereoRig,
    config: &TriangulationConfig,
) -> Option<Vector3> {
    let u_l = observation.left_normalized.x;
    let v_l = observation.left_normalized.y;
    let u_r = observation.right_normalized.x;
    let v_r = observation.right_normalized.y;

    let r0 = rig.right_from_left_rotation.row(0);
    let r1 = rig.right_from_left_rotation.row(1);
    let r2 = rig.right_from_left_rotation.row(2);
    let d = r2[0] * u_l + r2[1] * v_l + r2[2];
    let a0 = u_r * d - (r0[0] * u_l + r0[1] * v_l + r0[2]);
    let a1 = v_r * d - (r1[0] * u_l + r1[1] * v_l + r1[2]);
    let b0 = rig.right_from_left_translation.x - u_r * rig.right_from_left_translation.z;
    let b1 = rig.right_from_left_translation.y - v_r * rig.right_from_left_translation.z;
    let denom = a0 * a0 + a1 * a1;

    if denom.abs() < 1e-10 {
        return None;
    }

    let depth = (a0 * b0 + a1 * b1) / denom;
    if depth <= config.min_depth_m || depth > config.max_depth_m || !depth.is_finite() {
        return None;
    }

    Some(Vector3::new(u_l * depth, v_l * depth, depth))
}

#[must_use]
pub fn stereo_depth_seed(
    observation: &StereoObservation,
    clone: &CameraClone,
    rig: &StereoRig,
    config: &TriangulationConfig,
) -> Option<Vector3> {
    let point_left = stereo_triangulate_in_left_frame(observation, rig, config)?;
    let rotation_wc = clone.orientation.to_rotation_matrix().into_inner();
    Some(rotation_wc * point_left + clone.position)
}

#[must_use]
fn build_dlt_system_for_observations(
    observations: &[&StereoObservation],
    clones_by_id: &FxHashMap<usize, CameraClone>,
    rig: &StereoRig,
) -> Option<DMatrix> {
    let mut rows = Vec::with_capacity(observations.len() * 16);
    let mut valid_observation_count = 0;

    for observation in observations {
        let clone = clones_by_id.get(&observation.clone_id)?;
        let p_left = projection_matrix(clone);
        let p_right = right_projection_matrix(&p_left, rig);

        append_projection_rows(&mut rows, &p_left, &observation.left_normalized);
        append_projection_rows(&mut rows, &p_right, &observation.right_normalized);
        valid_observation_count += 2;
    }

    (valid_observation_count >= 4).then(|| DMatrix::from_row_slice(valid_observation_count * 2, 4, &rows))
}

#[must_use]
fn positive_depth_observations<'a>(
    track: &'a FeatureTrack,
    point_world: &Vector3,
    clones_by_id: &FxHashMap<usize, CameraClone>,
    rig: &StereoRig,
    config: &TriangulationConfig,
) -> Vec<&'a StereoObservation> {
    track
        .observations
        .iter()
        .filter(|observation| {
            let Some(clone) = clones_by_id.get(&observation.clone_id) else {
                return false;
            };

            let p_left = projection_matrix(clone);
            let p_right = right_projection_matrix(&p_left, rig);
            let point_h = Vector4::new(point_world.x, point_world.y, point_world.z, 1.0);
            let left_camera = p_left.fixed_view::<3, 4>(0, 0) * point_h;
            let right_camera = p_right.fixed_view::<3, 4>(0, 0) * point_h;
            let left_depth = left_camera[2];
            let right_depth = right_camera[2];
            left_depth > config.min_depth_m
                && right_depth > config.min_depth_m
                && left_depth <= config.max_depth_m
                && right_depth <= config.max_depth_m
        })
        .collect()
}

fn append_projection_rows(storage: &mut Vec<Float>, projection: &Matrix4x4, normalized: &Vector2) {
    let p1 = projection.row(0);
    let p2 = projection.row(1);
    let p3 = projection.row(2);

    for column in 0..4 {
        storage.push(normalized.x * p3[column] - p1[column]);
    }
    for column in 0..4 {
        storage.push(normalized.y * p3[column] - p2[column]);
    }
}

#[must_use]
fn solve_dlt(system: &DMatrix) -> Option<(Vector3, Float)> {
    let svd = system.clone().svd(true, true);
    let singular_values = svd.singular_values;
    let max_singular = *singular_values.get(0)?;
    let conditioning_singular = *singular_values.get(singular_values.len().checked_sub(2)?)?;
    if conditioning_singular.abs() < 1e-12
        || !max_singular.is_finite()
        || !conditioning_singular.is_finite()
    {
        return None;
    }

    let v_t = svd.v_t?;
    let solution = v_t.row(v_t.nrows() - 1);
    let w = solution[3];
    if w.abs() < 1e-12 || !w.is_finite() {
        return None;
    }

    let point_world = Vector3::new(solution[0] / w, solution[1] / w, solution[2] / w);
    Some((point_world, max_singular / conditioning_singular))
}

#[must_use]
fn compute_reprojection_rmse(
    track: &FeatureTrack,
    point_world: &Vector3,
    clones_by_id: &FxHashMap<usize, CameraClone>,
    rig: &StereoRig,
) -> Option<Float> {
    let mut squared_error = 0.0;
    let mut sample_count = 0usize;

    for observation in &track.observations {
        let clone = clones_by_id.get(&observation.clone_id)?;
        let p_left = projection_matrix(clone);
        let p_right = right_projection_matrix(&p_left, rig);
        let Some(left_projected) = project_normalized(&p_left, point_world) else {
            continue;
        };
        let Some(right_projected) = project_normalized(&p_right, point_world) else {
            continue;
        };

        squared_error += (left_projected - observation.left_normalized).norm_squared();
        squared_error += (right_projected - observation.right_normalized).norm_squared();
        sample_count += 2;
    }

    (sample_count > 0).then(|| (squared_error / sample_count as Float).sqrt())
}

#[must_use]
fn count_positive_depth_observations(
    track: &FeatureTrack,
    point_world: &Vector3,
    clones_by_id: &FxHashMap<usize, CameraClone>,
    rig: &StereoRig,
    config: &TriangulationConfig,
) -> usize {
    positive_depth_observations(track, point_world, clones_by_id, rig, config).len()
}

#[must_use]
pub fn triangulate_track(
    track: &FeatureTrack,
    clones: &[CameraClone],
    rig: &StereoRig,
    config: &TriangulationConfig,
) -> Option<TriangulatedLandmark> {
    triangulate_track_diagnostic(track, clones, rig, config).landmark
}

#[must_use]
pub fn triangulate_track_diagnostic(
    track: &FeatureTrack,
    clones: &[CameraClone],
    rig: &StereoRig,
    config: &TriangulationConfig,
) -> TriangulationResult {
    let clones_by_id = clones
        .iter()
        .cloned()
        .map(|clone| (clone.clone_id, clone))
        .collect::<FxHashMap<_, _>>();
    let Some(first_observation) = track.observations.first() else {
        return TriangulationResult {
            landmark: None,
            failure: Some(TriangulationFailure::StereoSeed),
        };
    };
    let Some(first_clone) = clones_by_id.get(&first_observation.clone_id) else {
        return TriangulationResult {
            landmark: None,
            failure: Some(TriangulationFailure::StereoSeed),
        };
    };
    let Some(stereo_seed_world) = stereo_depth_seed(first_observation, first_clone, rig, config) else {
        return TriangulationResult {
            landmark: None,
            failure: Some(TriangulationFailure::StereoSeed),
        };
    };

    let seed_supported_observations = positive_depth_observations(
        track,
        &stereo_seed_world,
        &clones_by_id,
        rig,
        config,
    );
    if seed_supported_observations.len() < 2 {
        return TriangulationResult {
            landmark: None,
            failure: Some(TriangulationFailure::PositiveDepth),
        };
    }

    let Some(system) = build_dlt_system_for_observations(&seed_supported_observations, &clones_by_id, rig) else {
        return TriangulationResult {
            landmark: None,
            failure: Some(TriangulationFailure::DltSystem),
        };
    };
    let Some((position_world, condition_number)) = solve_dlt(&system) else {
        return TriangulationResult {
            landmark: None,
            failure: Some(TriangulationFailure::DltSolve),
        };
    };
    if condition_number > config.max_condition_number {
        return TriangulationResult {
            landmark: None,
            failure: Some(TriangulationFailure::ConditionNumber),
        };
    }
    if count_positive_depth_observations(track, &position_world, &clones_by_id, rig, config) < 2 {
        return TriangulationResult {
            landmark: None,
            failure: Some(TriangulationFailure::PositiveDepth),
        };
    }

    let Some(reprojection_rmse) = compute_reprojection_rmse(track, &position_world, &clones_by_id, rig) else {
        return TriangulationResult {
            landmark: None,
            failure: Some(TriangulationFailure::Reprojection),
        };
    };
    if reprojection_rmse > config.max_reprojection_rmse {
        return TriangulationResult {
            landmark: None,
            failure: Some(TriangulationFailure::Reprojection),
        };
    }

    TriangulationResult {
        landmark: Some(TriangulatedLandmark {
            position_world,
            stereo_seed_world,
            reprojection_rmse,
            condition_number,
        }),
        failure: None,
    }
}

#[cfg(test)]
mod tests {
    use super::{triangulate_track, StereoRig, TriangulationConfig};
    use crate::{
        msckf::{CameraClone, FeatureTrack, StereoObservation},
        types::{Matrix3x3, UnitQuaternion, Vector2, Vector3},
    };

    fn make_clone(clone_id: usize, x: f64) -> CameraClone {
        CameraClone::new(
            clone_id,
            clone_id as f64 * 0.1,
            UnitQuaternion::identity(),
            Vector3::new(x, 0.0, 0.0),
        )
    }

    fn normalized_projection(point_camera: Vector3) -> Vector2 {
        Vector2::new(point_camera.x / point_camera.z, point_camera.y / point_camera.z)
    }

    fn make_observation(clone: &CameraClone, feature_id: usize, point_world: Vector3, rig: &StereoRig) -> StereoObservation {
        let rotation_cw = clone.orientation.to_rotation_matrix().into_inner().transpose();
        let translation_cw = -rotation_cw * clone.position;
        let point_left = rotation_cw * point_world + translation_cw;
        let point_right = rig.right_from_left_rotation * point_left + rig.right_from_left_translation;
        let left = normalized_projection(point_left);
        let right = normalized_projection(point_right);

        StereoObservation {
            clone_id: clone.clone_id,
            timestamp_s: clone.timestamp_s,
            feature_id,
            left_pixel: left,
            right_pixel: right,
            left_normalized: left,
            right_normalized: right,
            left_age: clone.clone_id as u32,
            right_age: clone.clone_id as u32,
        }
    }

    #[test]
    fn triangulates_synthetic_track_with_sub_centimeter_error() {
        let rig = StereoRig::new(Matrix3x3::identity(), Vector3::new(-0.2, 0.0, 0.0));
        let point_world = Vector3::new(0.3, -0.15, 4.0);
        let clones = vec![make_clone(0, 0.0), make_clone(1, 0.1), make_clone(2, 0.2)];
        let observations = clones
            .iter()
            .map(|clone| make_observation(clone, 11, point_world, &rig))
            .collect::<Vec<_>>();
        let track = FeatureTrack {
            feature_id: 11,
            observations,
            total_age: 3,
            missed_frames: 0,
            last_seen_clone_id: Some(2),
        };

        let landmark = triangulate_track(&track, &clones, &rig, &TriangulationConfig::default())
            .expect("synthetic stereo track should triangulate");

        assert!((landmark.position_world - point_world).norm() < 0.01);
        assert!(landmark.reprojection_rmse < 1e-9);
        assert!(landmark.condition_number.is_finite());
    }
}