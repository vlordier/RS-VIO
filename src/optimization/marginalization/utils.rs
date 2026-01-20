//! Utility functions for marginalization

use std::collections::HashMap;

use super::config::{MarginalizationConfig, ParamId};

/// Select parameters to marginalize based on age and observability
pub fn select_marginalization_candidates(
    keyframe_ids: &[usize],
    landmark_ids: &[usize],
    landmark_observations: &HashMap<usize, usize>,
    landmark_last_obs: &HashMap<usize, usize>,
    current_frame: usize,
    config: &MarginalizationConfig,
) -> (Vec<ParamId>, Vec<ParamId>) {
    let mut marg_ids = Vec::new();
    let mut keep_ids = Vec::new();

    // Marginalize oldest keyframes first
    for (i, &kf_id) in keyframe_ids.iter().enumerate() {
        let _age = current_frame - kf_id;
        if i < config.num_marginalize_per_step {
            // Mark oldest keyframes for marginalization
            marg_ids.push(ParamId::KeyframePose(kf_id));
            marg_ids.push(ParamId::KeyframeVelocity(kf_id));
            marg_ids.push(ParamId::KeyframeAccelBias(kf_id));
            marg_ids.push(ParamId::KeyframeGyroBias(kf_id));
            marg_ids.push(ParamId::KeyframeMass(kf_id));
        } else {
            keep_ids.push(ParamId::KeyframePose(kf_id));
            keep_ids.push(ParamId::KeyframeVelocity(kf_id));
            keep_ids.push(ParamId::KeyframeAccelBias(kf_id));
            keep_ids.push(ParamId::KeyframeGyroBias(kf_id));
            keep_ids.push(ParamId::KeyframeMass(kf_id));
        }
    }

    // Marginalize old or poorly observed landmarks
    for &lm_id in landmark_ids {
        let obs_count = landmark_observations.get(&lm_id).copied().unwrap_or(0);
        let last_obs = landmark_last_obs.get(&lm_id).copied().unwrap_or(0);
        let age = current_frame - last_obs;

        if obs_count < config.min_landmark_observations || age > config.landmark_age_limit {
            marg_ids.push(ParamId::Landmark(lm_id));
        } else {
            keep_ids.push(ParamId::Landmark(lm_id));
        }
    }

    // Add global parameters (never marginalized)
    keep_ids.push(ParamId::GlobalGyroBias);
    keep_ids.push(ParamId::GlobalGravity);
    keep_ids.push(ParamId::GlobalDrag);

    (marg_ids, keep_ids)
}
