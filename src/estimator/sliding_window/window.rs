use crate::debug_log;
use crate::estimator::keyframe_culler::{AggressiveCullingConfig, AggressiveKeyframeCuller};
use crate::estimator::point_quality::PointQualityScorer;
use crate::estimator::Frame;
use crate::optimization::loop_closure::LoopClosureConstraint;
use crate::optimization::marginalization::{MarginalizationConfig, MarginalizationManager};
use crate::optimization::tight_coupling::ImuPreintegration;
use crate::types::{Matrix4x4, Vector3};
use std::collections::{HashMap, VecDeque};

/// Sliding window of keyframes for bundle adjustment optimization.
///
/// Maintains a fixed-size window of keyframes and manages the optimization
/// of poses and 3D points across these frames.
#[derive(Debug)]
#[allow(dead_code)]
pub struct SlidingWindow {
    /// Maximum number of keyframes in the sliding window.
    pub(crate) max_frames: usize,

    /// Current keyframes in the sliding window (ordered by insertion time, oldest first).
    pub(crate) keyframes: VecDeque<Frame>,

    /// Map points stored by feature ID: HashMap<feature_id, [x, y, z]>
    /// Bounded to prevent unbounded memory growth in long-running systems
    pub(crate) map_points: HashMap<usize, [f32; 3]>,

    /// Maximum number of map points to maintain (for embedded systems)
    pub(crate) max_map_points: usize,

    /// Track observation count for each map point (for LRU eviction)
    pub(crate) map_point_observations: HashMap<usize, usize>,

    /// Marginalization manager for sliding window
    pub(crate) marginalization_manager: MarginalizationManager,

    /// Loop-closure constraints linking keyframes inside the window
    pub(crate) loop_closure_constraints: Vec<LoopClosureConstraint>,

    /// Aggressive keyframe culler for intelligent frame removal
    pub(crate) keyframe_culler: Option<AggressiveKeyframeCuller>,

    /// Point quality scorer for filtering map points
    pub(crate) point_scorer: Option<PointQualityScorer>,

    /// IMU preintegrations between consecutive keyframes
    /// Key: from_keyframe_idx, Value: ImuPreintegration to next keyframe
    pub(crate) imu_preintegrations: Vec<ImuPreintegration>,
}

impl SlidingWindow {
    #![allow(non_snake_case)]

    /// Create a new sliding window with the specified maximum number of frames.
    pub fn new(max_frames: usize) -> Self {
        Self::with_marginalization_config(max_frames, MarginalizationConfig::default(), None, None)
    }

    /// Create a new sliding window with custom marginalization configuration.
    pub fn with_marginalization_config(
        max_frames: usize,
        marg_config: MarginalizationConfig,
        culling_config: Option<AggressiveCullingConfig>,
        quality_config: Option<crate::estimator::point_quality::PointQualityConfig>,
    ) -> Self {
        const DEFAULT_MAX_MAP_POINTS: usize = 2000;

        let marg_manager = MarginalizationManager::new(marg_config);
        let keyframe_culler = culling_config.map(AggressiveKeyframeCuller::new);
        let point_scorer = quality_config.map(PointQualityScorer::new);

        Self {
            max_frames,
            keyframes: VecDeque::with_capacity(max_frames),
            map_points: HashMap::with_capacity(DEFAULT_MAX_MAP_POINTS),
            max_map_points: DEFAULT_MAX_MAP_POINTS,
            map_point_observations: HashMap::with_capacity(DEFAULT_MAX_MAP_POINTS),
            marginalization_manager: marg_manager,
            loop_closure_constraints: Vec::new(),
            keyframe_culler,
            point_scorer,
            imu_preintegrations: Vec::new(),
        }
    }

    /// Create a new sliding window from a Config file.
    pub fn from_config(config: &crate::datasets::config::Config) -> Self {
        let marg_config = MarginalizationConfig {
            enabled: config.marginalization.enabled,
            use_fej: config.marginalization.use_fej,
            damping: config.marginalization.damping,
            max_keyframes: config.marginalization.max_keyframes,
            num_marginalize_per_step: config.marginalization.num_marginalize_per_step,
            min_landmark_observations: config.marginalization.min_landmark_observations,
            landmark_age_limit: config.marginalization.landmark_age_limit,
            prior_info_scale: config.marginalization.prior_info_scale,
            hessian_approximator: config.marginalization.hessian_approximator.clone(),
            gradient_computer: config.marginalization.gradient_computer.clone(),
            prior_constructor: config.marginalization.prior_constructor.clone(),
        };

        Self::with_marginalization_config(
            config.keyframe_management.keyframe_window_size as usize,
            marg_config,
            None,
            None,
        )
    }

    /// Create a new sliding window with the default size of 16 frames.
    #[allow(clippy::should_implement_trait)]
    pub fn default() -> Self {
        Self::new(8)
    }

    /// Configure maximum map points (used by tests and tuning).
    pub fn set_max_map_points(&mut self, max_map_points: usize) {
        self.max_map_points = max_map_points.max(1);

        if self.map_points.capacity() < self.max_map_points {
            self.map_points
                .reserve(self.max_map_points - self.map_points.capacity());
            self.map_point_observations
                .reserve(self.max_map_points - self.map_point_observations.capacity());
        }

        if self.map_points.len() > self.max_map_points {
            self.evict_old_map_points();
        }
    }

    /// Current number of stored map points (for observability in tests).
    pub fn map_points_len(&self) -> usize {
        self.map_points.len()
    }

    /// Evict least recently observed map points when capacity is reached.
    pub(crate) fn evict_old_map_points(&mut self) {
        if self.map_points.len() < self.max_map_points {
            return;
        }

        while self.map_points.len() > self.max_map_points {
            let mut points_by_observations: Vec<(usize, usize)> = self
                .map_point_observations
                .iter()
                .map(|(&id, &count)| (id, count))
                .collect();

            points_by_observations.sort_by_key(|&(_, count)| count);

            let overage = self.map_points.len() - self.max_map_points;
            let batch = overage.max(self.max_map_points / 10).max(1);

            for &(id, _) in points_by_observations.iter().take(batch) {
                self.map_points.remove(&id);
                self.map_point_observations.remove(&id);
            }
        }
    }

    /// Add a keyframe to the sliding window.
    pub fn add_frame(&mut self, frame: Frame) -> bool {
        if !frame.is_keyframe {
            log::warn!(
                "[SlidingWindow] Attempted to add non-keyframe (frame_id: {})",
                frame.frame_id
            );
            return false;
        }

        if self.keyframes.len() >= self.max_frames {
            if let Some(removed_frame) = self.keyframes.pop_front() {
                debug_log!(
                    "[SlidingWindow] Removed oldest frame (frame_id: {}) to make room for new keyframe",
                    removed_frame.frame_id
                );
                let removed_id = removed_frame.frame_id as u64;
                self.loop_closure_constraints
                    .retain(|c| c.keyframe_id_1 != removed_id && c.keyframe_id_2 != removed_id);
            }
        }

        self.keyframes.push_back(frame);
        debug_log!(
            "[SlidingWindow] Added keyframe, window size: {}/{}",
            self.keyframes.len(),
            self.max_frames
        );

        true
    }

    /// Append loop-closure constraints discovered by the detector.
    pub fn add_loop_closure_constraints(&mut self, constraints: Vec<LoopClosureConstraint>) {
        self.loop_closure_constraints.extend(constraints);
    }

    /// Add IMU preintegration data between two keyframes.
    pub fn add_imu_preintegration(
        &mut self,
        from_keyframe_idx: usize,
        _to_keyframe_idx: usize,
        preintegration: ImuPreintegration,
    ) {
        if from_keyframe_idx >= self.imu_preintegrations.len() {
            self.imu_preintegrations.resize(
                from_keyframe_idx + 1,
                ImuPreintegration {
                    dt: 0.0,
                    delta_R: nalgebra::Matrix3::identity(),
                    delta_v: Vector3::zeros(),
                    delta_p: Vector3::zeros(),
                    cov_R: nalgebra::Matrix3::zeros(),
                    cov_v: nalgebra::Matrix3::zeros(),
                    cov_p: nalgebra::Matrix3::zeros(),
                    cov_R_bw: nalgebra::Matrix3::zeros(),
                    cov_v_ba: nalgebra::Matrix3::zeros(),
                    cov_p_ba: nalgebra::Matrix3::zeros(),
                },
            );
        }
        self.imu_preintegrations[from_keyframe_idx] = preintegration;
        debug_log!(
            "[SlidingWindow] Stored IMU preintegration for keyframe {} (dt={:.3}s)",
            from_keyframe_idx,
            self.imu_preintegrations[from_keyframe_idx].dt
        );
    }

    /// Clean up IMU preintegrations after marginalization.
    pub fn cleanup_after_marginalization(&mut self, marginalized_keyframe_idx: usize) {
        if self.imu_preintegrations.is_empty() {
            return;
        }

        if marginalized_keyframe_idx < self.imu_preintegrations.len() {
            self.imu_preintegrations.remove(marginalized_keyframe_idx);
        }

        debug_log!(
            "[SlidingWindow] Cleaned up IMU preintegrations after marginalization (removed idx {})",
            marginalized_keyframe_idx
        );
    }

    /// Get the current number of keyframes in the window.
    pub fn len(&self) -> usize {
        self.keyframes.len()
    }

    /// Check if the sliding window is empty.
    pub fn is_empty(&self) -> bool {
        self.keyframes.is_empty()
    }

    /// Check if the sliding window is full.
    pub fn is_full(&self) -> bool {
        self.keyframes.len() >= self.max_frames
    }

    /// Get a reference to a specific keyframe by index.
    pub fn get_frame(&self, index: usize) -> Option<&Frame> {
        self.keyframes.get(index)
    }

    /// Get mutable references to all keyframes (e.g., for updating extrinsics).
    pub fn keyframes_mut(&mut self) -> impl Iterator<Item = &mut Frame> {
        self.keyframes.iter_mut()
    }

    pub fn get_keyframe_poses(&self) -> Vec<Matrix4x4> {
        self.keyframes.iter().map(|f| f.state.T_W_B).collect()
    }

    /// Clear all keyframes from the sliding window.
    pub fn clear(&mut self) {
        self.keyframes.clear();
        self.loop_closure_constraints.clear();
        debug_log!("[SlidingWindow] Cleared all keyframes");
    }
}
