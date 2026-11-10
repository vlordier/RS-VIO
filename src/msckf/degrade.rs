use crate::{
    msckf::{FilterState, GatedTrackUpdate},
    types::Float,
};

/// Visual tracking mode used by the MSCKF pipeline.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum VisualMode {
    #[default]
    Init,
    Normal,
    Degraded,
}

/// Per-frame quality summary used for mode transitions.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct FrameQuality {
    pub active_track_count: usize,
    pub stereo_inlier_ratio: Float,
    pub median_reprojection_error_px: Float,
    pub consecutive_good_frames: usize,
}

impl FrameQuality {
    #[must_use]
    pub fn new(
        active_track_count: usize,
        stereo_inlier_ratio: Float,
        median_reprojection_error_px: Float,
    ) -> Self {
        Self {
            active_track_count,
            stereo_inlier_ratio,
            median_reprojection_error_px,
            consecutive_good_frames: 0,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct DegradationConfig {
    pub degraded_track_threshold: usize,
    pub degraded_inlier_ratio_threshold: Float,
    pub degraded_reprojection_threshold_px: Float,
    pub recovered_track_threshold: usize,
    pub recovered_inlier_ratio_threshold: Float,
    pub recovered_reprojection_threshold_px: Float,
    pub recovered_good_frames_required: usize,
    pub conservative_min_clone_observations: usize,
    pub conservative_max_reprojection_error_px: Float,
    pub conservative_min_track_count: usize,
    pub conservative_measurement_covariance_scale: Float,
}

impl Default for DegradationConfig {
    fn default() -> Self {
        Self {
            degraded_track_threshold: 25,
            degraded_inlier_ratio_threshold: 0.4,
            degraded_reprojection_threshold_px: 3.0,
            recovered_track_threshold: 35,
            recovered_inlier_ratio_threshold: 0.6,
            recovered_reprojection_threshold_px: 2.0,
            recovered_good_frames_required: 5,
            conservative_min_clone_observations: 4,
            conservative_max_reprojection_error_px: 1.5,
            conservative_min_track_count: 5,
            conservative_measurement_covariance_scale: 0.5,
        }
    }
}

#[derive(Debug, Clone)]
pub struct ConservativeUpdateCandidate {
    pub clone_observation_count: usize,
    pub reprojection_error_px: Float,
    pub gated_update: GatedTrackUpdate,
}

#[derive(Debug, Clone, Default)]
pub struct VisualUpdatePolicy {
    pub allow_clone_creation: bool,
    pub allow_visual_update: bool,
    pub conservative_updates: Vec<GatedTrackUpdate>,
}

#[must_use]
fn median(values: &[Float]) -> Float {
    if values.is_empty() {
        return 0.0;
    }

    let mut sorted = values.to_vec();
    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let mid = sorted.len() / 2;
    if sorted.len() % 2 == 0 {
        (sorted[mid - 1] + sorted[mid]) * 0.5
    } else {
        sorted[mid]
    }
}

#[must_use]
pub fn compute_frame_quality(
    active_track_count: usize,
    stereo_inlier_count: usize,
    reprojection_errors_px: &[Float],
) -> FrameQuality {
    let stereo_inlier_ratio = if active_track_count == 0 {
        0.0
    } else {
        stereo_inlier_count as Float / active_track_count as Float
    };

    FrameQuality::new(
        active_track_count,
        stereo_inlier_ratio,
        median(reprojection_errors_px),
    )
}

#[must_use]
fn is_degraded_frame(quality: &FrameQuality, config: &DegradationConfig) -> bool {
    quality.active_track_count < config.degraded_track_threshold
        || quality.stereo_inlier_ratio < config.degraded_inlier_ratio_threshold
        || quality.median_reprojection_error_px > config.degraded_reprojection_threshold_px
}

#[must_use]
fn is_recovered_frame(quality: &FrameQuality, config: &DegradationConfig) -> bool {
    quality.active_track_count > config.recovered_track_threshold
        && quality.stereo_inlier_ratio > config.recovered_inlier_ratio_threshold
        && quality.median_reprojection_error_px < config.recovered_reprojection_threshold_px
}

#[must_use]
fn select_conservative_updates(
    candidates: &[ConservativeUpdateCandidate],
    config: &DegradationConfig,
) -> Vec<GatedTrackUpdate> {
    let strict_updates = candidates
        .iter()
        .filter(|candidate| {
            candidate.clone_observation_count >= config.conservative_min_clone_observations
                && candidate.reprojection_error_px < config.conservative_max_reprojection_error_px
        })
        .map(|candidate| {
            let mut gated_update = candidate.gated_update.clone();
            gated_update.measurement_covariance *= config.conservative_measurement_covariance_scale;
            gated_update
        })
        .collect::<Vec<_>>();

    (strict_updates.len() >= config.conservative_min_track_count)
        .then_some(strict_updates)
        .unwrap_or_default()
}

pub fn update_visual_state(
    filter_state: &mut FilterState,
    quality: FrameQuality,
    initialized: bool,
    conservative_candidates: &[ConservativeUpdateCandidate],
    config: &DegradationConfig,
) -> VisualUpdatePolicy {
    if !initialized {
        filter_state.mode = VisualMode::Init;
        filter_state.last_quality = FrameQuality {
            consecutive_good_frames: 0,
            ..quality
        };
        return VisualUpdatePolicy {
            allow_clone_creation: false,
            allow_visual_update: false,
            conservative_updates: Vec::new(),
        };
    }

    let previous_good_frames = filter_state.last_quality.consecutive_good_frames;
    let mut updated_quality = quality;

    match filter_state.mode {
        VisualMode::Init => {
            filter_state.mode = VisualMode::Normal;
            updated_quality.consecutive_good_frames = usize::from(is_recovered_frame(&quality, config));
        }
        VisualMode::Normal => {
            if is_degraded_frame(&quality, config) {
                filter_state.mode = VisualMode::Degraded;
                updated_quality.consecutive_good_frames = 0;
            } else {
                updated_quality.consecutive_good_frames = if is_recovered_frame(&quality, config) {
                    previous_good_frames + 1
                } else {
                    0
                };
            }
        }
        VisualMode::Degraded => {
            if is_recovered_frame(&quality, config) {
                updated_quality.consecutive_good_frames = previous_good_frames + 1;
                if updated_quality.consecutive_good_frames >= config.recovered_good_frames_required {
                    filter_state.mode = VisualMode::Normal;
                    updated_quality.consecutive_good_frames = 0;
                }
            } else {
                updated_quality.consecutive_good_frames = 0;
            }
        }
    }

    filter_state.last_quality = updated_quality;

    match filter_state.mode {
        VisualMode::Init => VisualUpdatePolicy {
            allow_clone_creation: false,
            allow_visual_update: false,
            conservative_updates: Vec::new(),
        },
        VisualMode::Normal => VisualUpdatePolicy {
            allow_clone_creation: true,
            allow_visual_update: true,
            conservative_updates: Vec::new(),
        },
        VisualMode::Degraded => {
            let conservative_updates = select_conservative_updates(conservative_candidates, config);
            VisualUpdatePolicy {
                allow_clone_creation: false,
                allow_visual_update: !conservative_updates.is_empty(),
                conservative_updates,
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{
        compute_frame_quality, update_visual_state, ConservativeUpdateCandidate,
        DegradationConfig, VisualMode,
    };
    use crate::{
        msckf::{FilterState, GatedTrackUpdate},
        types::{DMatrix, DVector},
    };

    fn gated_update(rows: usize) -> GatedTrackUpdate {
        GatedTrackUpdate {
            clone_ids: vec![0],
            projected_residual: DVector::zeros(rows),
            projected_h: DMatrix::zeros(rows, 15),
            measurement_covariance: DMatrix::identity(rows, rows),
            innovation_statistic: 0.1,
        }
    }

    #[test]
    fn frame_quality_uses_median_reprojection_and_inlier_ratio() {
        let quality = compute_frame_quality(10, 6, &[4.0, 1.0, 2.0, 3.0, 9.0]);
        assert_eq!(quality.active_track_count, 10);
        assert!((quality.stereo_inlier_ratio - 0.6).abs() < 1e-9);
        assert!((quality.median_reprojection_error_px - 3.0).abs() < 1e-9);
    }

    #[test]
    fn degraded_mode_only_allows_conservative_updates_when_strict_criteria_pass() {
        let mut filter_state = FilterState::new();
        filter_state.mode = VisualMode::Degraded;
        let quality = compute_frame_quality(20, 15, &[1.0, 1.2, 1.4]);
        let candidates = (0..5)
            .map(|_| ConservativeUpdateCandidate {
                clone_observation_count: 4,
                reprojection_error_px: 1.0,
                gated_update: gated_update(2),
            })
            .collect::<Vec<_>>();

        let policy = update_visual_state(
            &mut filter_state,
            quality,
            true,
            &candidates,
            &DegradationConfig::default(),
        );

        assert_eq!(filter_state.mode, VisualMode::Degraded);
        assert!(!policy.allow_clone_creation);
        assert!(policy.allow_visual_update);
        assert_eq!(policy.conservative_updates.len(), 5);
        assert!(policy.conservative_updates[0].measurement_covariance[(0, 0)] < 1.0);
    }

    #[test]
    fn fifty_bad_frames_trigger_degraded_and_five_good_frames_recover() {
        let config = DegradationConfig::default();
        let mut filter_state = FilterState::new();
        let good_quality = compute_frame_quality(40, 30, &[1.0, 1.2, 1.5, 1.8]);

        let init_policy = update_visual_state(&mut filter_state, good_quality, true, &[], &config);
        assert_eq!(filter_state.mode, VisualMode::Normal);
        assert!(init_policy.allow_clone_creation);
        assert!(init_policy.allow_visual_update);

        let bad_quality = compute_frame_quality(0, 0, &[]);
        for _ in 0..50 {
            let policy = update_visual_state(&mut filter_state, bad_quality, true, &[], &config);
            assert_eq!(filter_state.mode, VisualMode::Degraded);
            assert!(!policy.allow_clone_creation);
            assert!(!policy.allow_visual_update);
        }

        for _ in 0..(config.recovered_good_frames_required - 1) {
            let policy = update_visual_state(&mut filter_state, good_quality, true, &[], &config);
            assert_eq!(filter_state.mode, VisualMode::Degraded);
            assert!(!policy.allow_clone_creation);
        }

        let recovered_policy = update_visual_state(&mut filter_state, good_quality, true, &[], &config);
        assert_eq!(filter_state.mode, VisualMode::Normal);
        assert!(recovered_policy.allow_clone_creation);
        assert!(recovered_policy.allow_visual_update);
    }
}