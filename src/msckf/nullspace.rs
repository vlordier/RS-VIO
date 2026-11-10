use crate::types::{DMatrix, DVector};
use nalgebra::linalg::SymmetricEigen;

#[derive(Debug, Clone)]
pub struct NullspaceProjection {
    pub basis: DMatrix,
    pub projected_h_x: DMatrix,
    pub projected_h_f: DMatrix,
    pub projected_residual: DVector,
}

#[must_use]
pub fn compute_left_nullspace(h_f: &DMatrix) -> Option<DMatrix> {
    if h_f.nrows() <= h_f.ncols() {
        return None;
    }

    let gram = h_f.transpose() * h_f;
    let gram_inv = gram.try_inverse()?;
    let projector = DMatrix::identity(h_f.nrows(), h_f.nrows()) - h_f * gram_inv * h_f.transpose();
    let decomposition = SymmetricEigen::new(projector);

    let basis_columns = decomposition
        .eigenvalues
        .iter()
        .enumerate()
        .filter_map(|(index, eigenvalue)| (*eigenvalue > 1e-6).then_some(index))
        .collect::<Vec<_>>();

    if basis_columns.is_empty() {
        return None;
    }

    let mut basis = DMatrix::zeros(h_f.nrows(), basis_columns.len());
    for (basis_col, eigen_col) in basis_columns.into_iter().enumerate() {
        basis
            .column_mut(basis_col)
            .copy_from(&decomposition.eigenvectors.column(eigen_col));
    }

    Some(basis)
}

#[must_use]
pub fn nullspace_project(h_x: &DMatrix, h_f: &DMatrix, residual: &DVector) -> Option<NullspaceProjection> {
    if h_x.nrows() != h_f.nrows() || residual.nrows() != h_f.nrows() {
        return None;
    }

    let basis = compute_left_nullspace(h_f)?;
    let projected_h_x = basis.transpose() * h_x;
    let projected_h_f = basis.transpose() * h_f;
    let projected_residual = basis.transpose() * residual;

    Some(NullspaceProjection {
        basis,
        projected_h_x,
        projected_h_f,
        projected_residual,
    })
}

#[cfg(test)]
mod tests {
    use super::nullspace_project;
    use crate::{
        msckf::{
            build_track_residual, CameraClone, FeatureTrack, FisheyeObservationModel,
            StereoObservation, StereoRig,
        },
        types::{Matrix3x3, UnitQuaternion, Vector2, Vector3},
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
            feature_id: 31,
            left_pixel,
            right_pixel,
            left_normalized: Vector2::new(point_left.x / point_left.z, point_left.y / point_left.z),
            right_normalized: Vector2::new(point_right.x / point_right.z, point_right.y / point_right.z),
            left_age: clone.clone_id as u32,
            right_age: clone.clone_id as u32,
        }
    }

    #[test]
    fn nullspace_projection_annihilates_landmark_jacobian() {
        let rig = StereoRig::new(Matrix3x3::identity(), Vector3::new(-0.2, 0.0, 0.0));
        let camera_model = FisheyeObservationModel::default();
        let point_world = Vector3::new(0.25, -0.12, 4.5);
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
            feature_id: 31,
            observations,
            total_age: 3,
            missed_frames: 0,
            last_seen_clone_id: Some(2),
        };

        let residual = build_track_residual(&track, &point_world, &clones, &rig, &camera_model).unwrap();
        let projection = nullspace_project(&residual.h_x, &residual.h_f, &residual.residual).unwrap();

        assert!(projection.projected_h_f.norm() < 1e-8);
        assert_eq!(projection.projected_h_x.ncols(), residual.h_x.ncols());
        assert_eq!(projection.projected_residual.nrows(), projection.projected_h_x.nrows());
    }
}