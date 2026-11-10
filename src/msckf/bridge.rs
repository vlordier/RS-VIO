use crate::msckf::{
    compute_frame_quality, update_visual_state, ConservativeUpdateCandidate,
    DegradationConfig, FilterState, GatedTrackUpdate, VisualUpdatePolicy,
};
use crate::types::Float;

#[derive(Debug, Clone)]
pub struct TrackUpdateOutcome {
    pub feature_id: usize,
    pub clone_observation_count: usize,
    pub reprojection_error_px: Float,
    pub triangulated: bool,
    pub gated_update: Option<GatedTrackUpdate>,
}

pub fn build_visual_policy_from_track_outcomes(
    filter_state: &mut FilterState,
    active_track_count: usize,
    stereo_inlier_count: usize,
    outcomes: &[TrackUpdateOutcome],
    initialized: bool,
    config: &DegradationConfig,
) -> VisualUpdatePolicy {
    let reprojection_errors = outcomes
        .iter()
        .map(|outcome| outcome.reprojection_error_px)
        .collect::<Vec<_>>();
    let conservative_candidates = outcomes
        .iter()
        .filter_map(|outcome| {
            Some(ConservativeUpdateCandidate {
                clone_observation_count: outcome.clone_observation_count,
                reprojection_error_px: outcome.reprojection_error_px,
                gated_update: outcome.gated_update.clone()?,
            })
        })
        .collect::<Vec<_>>();
    let quality = compute_frame_quality(active_track_count, stereo_inlier_count, &reprojection_errors);

    update_visual_state(filter_state, quality, initialized, &conservative_candidates, config)
}

#[cfg(test)]
mod tests {
    use super::{build_visual_policy_from_track_outcomes, TrackUpdateOutcome};
    use crate::{
        msckf::{DegradationConfig, FilterState, GatedTrackUpdate},
        types::{DMatrix, DVector},
    };

    fn gated_update() -> GatedTrackUpdate {
        GatedTrackUpdate {
            clone_ids: vec![0],
            projected_residual: DVector::zeros(2),
            projected_h: DMatrix::zeros(2, 15),
            measurement_covariance: DMatrix::identity(2, 2),
            innovation_statistic: 0.1,
        }
    }

    #[test]
    fn bridge_translates_track_outcomes_into_degradation_policy() {
        let mut filter_state = FilterState::new();
        let outcomes = (0..5)
            .map(|feature_id| TrackUpdateOutcome {
                feature_id,
                clone_observation_count: 4,
                reprojection_error_px: 1.0,
                triangulated: true,
                gated_update: Some(gated_update()),
            })
            .collect::<Vec<_>>();

        let policy = build_visual_policy_from_track_outcomes(
            &mut filter_state,
            10,
            10,
            &outcomes,
            false,
            &DegradationConfig::default(),
        );

        assert!(!policy.allow_visual_update);
        assert!(!policy.allow_clone_creation);
    }
}