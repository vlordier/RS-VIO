use crate::{
    estimator::Frame,
    feature_tracker::Feature,
    types::{Float, Vector2},
};
use rustc_hash::{FxHashMap, FxHashSet};

#[derive(Debug, Clone, PartialEq)]
pub struct StereoObservation {
    pub clone_id: usize,
    pub timestamp_s: Float,
    pub feature_id: usize,
    pub left_pixel: Vector2,
    pub right_pixel: Vector2,
    pub left_normalized: Vector2,
    pub right_normalized: Vector2,
    pub left_age: u32,
    pub right_age: u32,
}

impl StereoObservation {
    #[must_use]
    pub fn disparity_px(&self) -> Float {
        self.left_pixel.x - self.right_pixel.x
    }

    #[must_use]
    pub fn vertical_residual_px(&self) -> Float {
        (self.left_pixel.y - self.right_pixel.y).abs()
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct FeatureTrack {
    pub feature_id: usize,
    pub observations: Vec<StereoObservation>,
    pub total_age: u32,
    pub missed_frames: usize,
    pub last_seen_clone_id: Option<usize>,
}

impl FeatureTrack {
    #[must_use]
    pub fn new(feature_id: usize, observation: StereoObservation) -> Self {
        Self {
            feature_id,
            total_age: observation.left_age.max(observation.right_age),
            observations: vec![observation.clone()],
            missed_frames: 0,
            last_seen_clone_id: Some(observation.clone_id),
        }
    }

    pub fn push_observation(&mut self, observation: StereoObservation, max_observations: usize) {
        self.total_age = self.total_age.max(observation.left_age.max(observation.right_age));
        self.missed_frames = 0;
        self.last_seen_clone_id = Some(observation.clone_id);

        if let Some(last_observation) = self.observations.last_mut() {
            if last_observation.clone_id == observation.clone_id {
                *last_observation = observation;
                return;
            }
        }

        self.observations.push(observation);
        if self.observations.len() > max_observations {
            let overflow = self.observations.len() - max_observations;
            self.observations.drain(0..overflow);
        }
    }

    #[must_use]
    pub fn clone_observation_count(&self) -> usize {
        self.observations
            .iter()
            .map(|observation| observation.clone_id)
            .collect::<FxHashSet<_>>()
            .len()
    }

    #[must_use]
    pub fn is_mature(&self, min_clone_observations: usize) -> bool {
        self.clone_observation_count() >= min_clone_observations
    }
}

#[derive(Debug, Clone, Copy)]
pub struct TrackManagerConfig {
    pub min_mature_clone_observations: usize,
    pub max_missed_frames: usize,
    pub max_track_observations: usize,
    pub max_vertical_disparity_px: Float,
    pub min_disparity_px: Float,
}

impl Default for TrackManagerConfig {
    fn default() -> Self {
        Self {
            min_mature_clone_observations: 3,
            max_missed_frames: 2,
            max_track_observations: 16,
            max_vertical_disparity_px: 2.0,
            min_disparity_px: 0.5,
        }
    }
}

#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct TrackUpdateSummary {
    pub paired_observations: usize,
    pub created_tracks: usize,
    pub updated_tracks: usize,
    pub expired_tracks: usize,
}

#[derive(Debug, Clone, Default)]
pub struct TrackManager {
    tracks: FxHashMap<usize, FeatureTrack>,
    pub config: TrackManagerConfig,
}

impl TrackManager {
    #[must_use]
    pub fn new(config: TrackManagerConfig) -> Self {
        Self {
            tracks: FxHashMap::default(),
            config,
        }
    }

    #[must_use]
    pub fn len(&self) -> usize {
        self.tracks.len()
    }

    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.tracks.is_empty()
    }

    #[must_use]
    pub fn get(&self, feature_id: usize) -> Option<&FeatureTrack> {
        self.tracks.get(&feature_id)
    }

    pub fn remove_track(&mut self, feature_id: usize) -> Option<FeatureTrack> {
        self.tracks.remove(&feature_id)
    }

    pub fn prune_clone_observations(&mut self, removed_clone_id: usize) {
        self.tracks.retain(|_, track| {
            track.observations.retain(|observation| observation.clone_id != removed_clone_id);
            track.last_seen_clone_id = track.observations.last().map(|observation| observation.clone_id);
            !track.observations.is_empty()
        });
    }

    #[must_use]
    pub fn tracks(&self) -> Vec<&FeatureTrack> {
        self.tracks.values().collect()
    }

    #[must_use]
    pub fn mature_tracks(&self) -> Vec<&FeatureTrack> {
        self.tracks
            .values()
            .filter(|track| track.is_mature(self.config.min_mature_clone_observations))
            .collect()
    }

    pub fn ingest_frame(&mut self, frame: &Frame, clone_id: usize) -> TrackUpdateSummary {
        let timestamp_s = frame.timestamp_ns as Float * 1e-9;
        let observations = pair_stereo_observations(frame, clone_id, timestamp_s, &self.config);
        let visible_ids = observations
            .iter()
            .map(|observation| observation.feature_id)
            .collect::<FxHashSet<_>>();

        let mut summary = TrackUpdateSummary {
            paired_observations: observations.len(),
            ..TrackUpdateSummary::default()
        };

        for observation in observations {
            match self.tracks.get_mut(&observation.feature_id) {
                Some(track) => {
                    track.push_observation(observation, self.config.max_track_observations);
                    summary.updated_tracks += 1;
                }
                None => {
                    self.tracks.insert(
                        observation.feature_id,
                        FeatureTrack::new(observation.feature_id, observation),
                    );
                    summary.created_tracks += 1;
                }
            }
        }

        let expired_ids = self
            .tracks
            .iter_mut()
            .filter_map(|(&feature_id, track)| {
                if visible_ids.contains(&feature_id) {
                    return None;
                }

                track.missed_frames += 1;
                (track.missed_frames > self.config.max_missed_frames).then_some(feature_id)
            })
            .collect::<Vec<_>>();

        summary.expired_tracks = expired_ids.len();
        for feature_id in expired_ids {
            self.tracks.remove(&feature_id);
        }

        summary
    }
}

#[must_use]
fn feature_pixel(feature: &Feature) -> Vector2 {
    Vector2::new(feature.pixel_coord[0] as Float, feature.pixel_coord[1] as Float)
}

#[must_use]
fn feature_normalized(feature: &Feature) -> Vector2 {
    if feature.undistorted_coord[0] > -0.5 && feature.undistorted_coord[1] > -0.5 {
        return Vector2::new(
            feature.undistorted_coord[0] as Float,
            feature.undistorted_coord[1] as Float,
        );
    }

    feature_pixel(feature)
}

#[must_use]
fn has_valid_normalized(feature: &Feature) -> bool {
    feature.undistorted_coord[0] > -0.5 && feature.undistorted_coord[1] > -0.5
}

#[must_use]
pub fn pair_stereo_observations(
    frame: &Frame,
    clone_id: usize,
    timestamp_s: Float,
    config: &TrackManagerConfig,
) -> Vec<StereoObservation> {
    let right_by_id = frame
        .right_features()
        .iter()
        .map(|feature| (feature.feature_id, feature))
        .collect::<FxHashMap<_, _>>();

    frame
        .left_features()
        .iter()
        .filter_map(|left_feature| {
            let right_feature = right_by_id.get(&left_feature.feature_id)?;
            build_stereo_observation(left_feature, right_feature, clone_id, timestamp_s, config)
        })
        .collect()
}

#[must_use]
fn build_stereo_observation(
    left_feature: &Feature,
    right_feature: &Feature,
    clone_id: usize,
    timestamp_s: Float,
    config: &TrackManagerConfig,
) -> Option<StereoObservation> {
    let left_pixel = feature_pixel(left_feature);
    let right_pixel = feature_pixel(right_feature);
    let left_normalized = feature_normalized(left_feature);
    let right_normalized = feature_normalized(right_feature);
    let normalized_available = has_valid_normalized(left_feature) && has_valid_normalized(right_feature);
    let disparity_px = left_pixel.x - right_pixel.x;
    let vertical_residual_px = (left_pixel.y - right_pixel.y).abs();

    if !disparity_px.is_finite()
        || !vertical_residual_px.is_finite()
    {
        return None;
    }

    if !normalized_available
        && (disparity_px < config.min_disparity_px
            || vertical_residual_px > config.max_vertical_disparity_px)
    {
        return None;
    }

    Some(StereoObservation {
        clone_id,
        timestamp_s,
        feature_id: left_feature.feature_id,
        left_pixel,
        right_pixel,
        left_normalized,
        right_normalized,
        left_age: left_feature.age,
        right_age: right_feature.age,
    })
}

#[cfg(test)]
mod tests {
    use super::{pair_stereo_observations, TrackManager, TrackManagerConfig};
    use crate::{estimator::Frame, feature_tracker::Feature};

    fn make_feature(feature_id: usize, x: f32, y: f32, age: u32) -> Feature {
        let mut feature = Feature::new(feature_id, [x, y]);
        feature.age = age;
        feature
    }

    #[test]
    fn stereo_pairing_only_keeps_consistent_pairs() {
        let mut frame = Frame::new(100_000_000, 0);
        frame.add_left_feature(make_feature(1, 20.0, 10.0, 1));
        frame.add_right_feature(make_feature(1, 15.0, 10.2, 1));
        frame.add_left_feature(make_feature(2, 22.0, 11.0, 1));
        frame.add_right_feature(make_feature(2, 22.5, 20.0, 1));

        let observations = pair_stereo_observations(&frame, 3, 0.1, &TrackManagerConfig::default());
        assert_eq!(observations.len(), 1);
        assert_eq!(observations[0].feature_id, 1);
        assert!(observations[0].disparity_px() > 0.0);
    }

    #[test]
    fn stereo_pairing_keeps_unrectified_pairs_when_normalized_coords_exist() {
        let mut frame = Frame::new(100_000_000, 0);
        let mut left = make_feature(1, 220.0, 120.0, 1);
        let mut right = make_feature(1, 219.2, 126.5, 1);
        left.undistorted_coord = [0.10, 0.02];
        right.undistorted_coord = [0.04, 0.01];
        frame.add_left_feature(left);
        frame.add_right_feature(right);

        let observations = pair_stereo_observations(&frame, 3, 0.1, &TrackManagerConfig::default());
        assert_eq!(observations.len(), 1);
        assert_eq!(observations[0].feature_id, 1);
    }

    #[test]
    fn track_manager_updates_maturity_and_expires_missing_tracks() {
        let mut manager = TrackManager::new(TrackManagerConfig::default());

        for (frame_idx, timestamp_ns) in [0_i64, 100_000_000, 200_000_000].into_iter().enumerate() {
            let mut frame = Frame::new(timestamp_ns, frame_idx as i32);
            frame.add_left_feature(make_feature(7, 40.0 + frame_idx as f32, 20.0, frame_idx as u32));
            frame.add_right_feature(make_feature(7, 35.0 + frame_idx as f32, 20.1, frame_idx as u32));

            let summary = manager.ingest_frame(&frame, frame_idx);
            assert_eq!(summary.paired_observations, 1);
        }

        let track = manager.get(7).expect("track should exist after three stereo observations");
        assert!(track.is_mature(manager.config.min_mature_clone_observations));
        assert_eq!(track.clone_observation_count(), 3);

        for frame_idx in 0..3 {
            let empty_frame = Frame::new(300_000_000 + frame_idx as i64 * 100_000_000, frame_idx + 10);
            manager.ingest_frame(&empty_frame, frame_idx as usize + 10);
        }

        assert!(manager.get(7).is_none(), "track should expire after repeated misses");
    }

    #[test]
    fn pruning_marginalized_clone_observations_removes_dead_tracks() {
        let mut manager = TrackManager::new(TrackManagerConfig::default());

        for (frame_idx, timestamp_ns) in [0_i64, 100_000_000, 200_000_000].into_iter().enumerate() {
            let mut frame = Frame::new(timestamp_ns, frame_idx as i32);
            frame.add_left_feature(make_feature(7, 40.0 + frame_idx as f32, 20.0, frame_idx as u32));
            frame.add_right_feature(make_feature(7, 35.0 + frame_idx as f32, 20.1, frame_idx as u32));
            manager.ingest_frame(&frame, frame_idx);
        }

        manager.prune_clone_observations(0);
        let track = manager.get(7).expect("track should keep newer clone observations");
        assert_eq!(track.clone_observation_count(), 2);

        manager.prune_clone_observations(1);
        manager.prune_clone_observations(2);
        assert!(manager.get(7).is_none(), "track should be dropped once all clone observations are gone");
    }
}