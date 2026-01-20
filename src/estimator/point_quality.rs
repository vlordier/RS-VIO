/// Point Quality Scorer and Map Manager
///
/// Implements comprehensive quality assessment for 3D map points including:
/// - Track length and consistency
/// - Observation angle statistics
/// - Reprojection error tracking
/// - Baseline diversity for triangulation
/// - Parallax and depth quality
/// - Automatic culling of poor-quality points
use crate::types::Float;
use nalgebra as na;
use std::collections::HashMap;

/// Quality metrics for a single point
#[derive(Clone, Debug)]
pub struct PointQuality {
    /// Unique point ID
    pub point_id: usize,

    /// Overall quality score (0.0-1.0)
    pub quality_score: Float,

    /// Track length (number of observations)
    pub track_length: u32,

    /// Mean observation angle (radians)
    pub mean_observation_angle: Float,

    /// Angle variance
    pub angle_variance: Float,

    /// Mean reprojection error (pixels)
    pub mean_reproj_error: Float,

    /// Triangulation baseline diversity (0.0-1.0)
    pub baseline_diversity: Float,

    /// Parallax quality (0.0-1.0)
    pub parallax_quality: Float,

    /// Depth consistency score
    pub depth_quality: Float,

    /// Age in keyframes
    pub age_keyframes: u32,

    /// Whether point should be marginalized
    pub should_cull: bool,

    /// Reason for culling (if applicable)
    pub cull_reason: Option<PointCullReason>,
}

/// Reasons for culling a point
#[derive(Clone, Debug, PartialEq)]
pub enum PointCullReason {
    ShortTrack,    // Track length below threshold
    HighError,     // Reprojection error too high
    SmallParallax, // Insufficient motion parallax
    ShallowDepth,  // Depth estimate unreliable
    Inconsistent,  // Observations inconsistent
    Old,           // Point too old (marginalization)
    Dynamic,       // Detected as moving object
}

/// Configuration for point quality scoring
#[derive(Debug, Clone)]
pub struct PointQualityConfig {
    /// Minimum track length for good point
    pub min_track_length: u32,

    /// Maximum mean reprojection error (pixels)
    pub max_reproj_error: Float,

    /// Minimum parallax angle (radians)
    pub min_parallax: Float,

    /// Minimum observation angle variance
    pub min_angle_variance: Float,

    /// Maximum point age before marginalization (keyframes)
    pub max_point_age: u32,

    /// Minimum baseline diversity (0.0-1.0)
    pub min_baseline_diversity: Float,

    /// Quality threshold for keeping point
    pub quality_threshold: Float,

    /// Enable automatic culling
    pub auto_cull: bool,

    /// Cull fraction when map is too large (0.0-1.0)
    pub cull_fraction: Float,

    /// Maximum map size before aggressive culling
    pub max_map_size: usize,

    /// Enable dynamic object detection
    pub detect_dynamic: bool,

    /// Motion inconsistency threshold
    pub motion_inconsistency_threshold: Float,
}

impl Default for PointQualityConfig {
    fn default() -> Self {
        Self {
            min_track_length: 5,
            max_reproj_error: 2.0,
            min_parallax: 0.05, // ~3 degrees
            min_angle_variance: 0.01,
            max_point_age: 30, // keyframes
            min_baseline_diversity: 0.1,
            quality_threshold: 0.3,
            auto_cull: true,
            cull_fraction: 0.2,
            max_map_size: 2000,
            detect_dynamic: true,
            motion_inconsistency_threshold: 5.0,
        }
    }
}

/// Observation record for a point
#[derive(Clone, Debug)]
pub struct PointObservation {
    /// Keyframe ID where observed
    pub keyframe_id: u32,

    /// Pixel coordinates in that frame
    pub pixel: na::Vector2<Float>,

    /// Feature track ID
    pub track_id: u32,

    /// Reprojection error in that frame
    pub reproj_error: Float,

    /// Camera pose ID when observed
    pub pose_id: u32,
}

/// 3D point with observation history
#[derive(Clone, Debug)]
pub struct TrackedPoint {
    /// Unique ID
    pub id: usize,

    /// 3D position in world frame
    pub position: na::Vector3<Float>,

    /// Observation history
    pub observations: Vec<PointObservation>,

    /// Covariance estimate (if available)
    pub covariance: Option<nalgebra::Matrix3<Float>>,

    /// First observed keyframe
    pub first_keyframe: u32,

    /// Last observed keyframe
    pub last_keyframe: u32,

    /// Triangulation baseline scores
    pub baselines: Vec<Float>,
}

impl TrackedPoint {
    /// Create new tracked point
    pub fn new(id: usize, position: na::Vector3<Float>) -> Self {
        Self {
            id,
            position,
            observations: Vec::new(),
            covariance: None,
            first_keyframe: 0,
            last_keyframe: 0,
            baselines: Vec::new(),
        }
    }

    /// Add new observation
    pub fn add_observation(
        &mut self,
        keyframe_id: u32,
        pixel: na::Vector2<Float>,
        track_id: u32,
        reproj_error: Float,
        pose_id: u32,
    ) {
        if self.observations.is_empty() {
            self.first_keyframe = keyframe_id;
        }
        self.last_keyframe = keyframe_id;

        self.observations.push(PointObservation {
            keyframe_id,
            pixel,
            track_id,
            reproj_error,
            pose_id,
        });
    }

    /// Get track length
    pub fn track_length(&self) -> u32 {
        self.observations.len() as u32
    }

    /// Get age in keyframes
    pub fn age(&self, current_keyframe: u32) -> u32 {
        current_keyframe - self.first_keyframe
    }

    /// Check if point is static (not on moving object)
    pub fn is_static(&self, config: &PointQualityConfig) -> bool {
        if !config.detect_dynamic {
            return true;
        }

        // Check for inconsistent motion
        if self.observations.len() < 3 {
            return true;
        }

        // Check if reprojection errors are increasing (suggesting motion)
        let recent_errors: Vec<Float> = self
            .observations
            .iter()
            .rev()
            .take(5)
            .map(|o| o.reproj_error)
            .collect();

        if recent_errors.len() >= 2 {
            let trend = recent_errors[0] - recent_errors[recent_errors.len() - 1];
            if trend > config.motion_inconsistency_threshold {
                return false;
            }
        }

        true
    }
}

/// Map quality statistics
#[derive(Clone, Debug)]
pub struct MapQuality {
    /// Total number of points
    pub total_points: usize,

    /// Number of good quality points
    pub good_points: usize,

    /// Number of marginal points
    pub marginal_points: usize,

    /// Number of poor points (should be culled)
    pub poor_points: usize,

    /// Mean quality score
    pub mean_quality: Float,

    /// Mean track length
    pub mean_track_length: Float,

    /// Mean reprojection error
    pub mean_reproj_error: Float,

    /// Culling recommendations
    pub cull_recommendations: Vec<usize>,
}

/// Point quality scorer and map manager
#[derive(Debug)]
pub struct PointQualityScorer {
    config: PointQualityConfig,

    /// All tracked points
    points: HashMap<usize, TrackedPoint>,

    /// Next available point ID
    next_point_id: usize,

    /// Current keyframe counter
    current_keyframe: u32,

    /// Statistics
    pub total_culled: u64,
    pub total_created: u64,
}

impl PointQualityScorer {
    /// Create new scorer
    pub fn new(config: PointQualityConfig) -> Self {
        Self {
            config,
            points: HashMap::new(),
            next_point_id: 0,
            current_keyframe: 0,
            total_culled: 0,
            total_created: 0,
        }
    }

    /// Add a new point
    pub fn add_point(&mut self, position: na::Vector3<Float>) -> usize {
        let id = self.next_point_id;
        self.next_point_id += 1;

        self.points.insert(
            id,
            TrackedPoint {
                id,
                position,
                observations: Vec::new(),
                covariance: None,
                first_keyframe: self.current_keyframe,
                last_keyframe: self.current_keyframe,
                baselines: Vec::new(),
            },
        );

        self.total_created += 1;
        id
    }

    /// Add observation to existing point
    pub fn add_observation(
        &mut self,
        point_id: usize,
        pixel: na::Vector2<Float>,
        track_id: u32,
        reproj_error: Float,
        pose_id: u32,
    ) -> bool {
        if let Some(point) = self.points.get_mut(&point_id) {
            point.add_observation(
                self.current_keyframe,
                pixel,
                track_id,
                reproj_error,
                pose_id,
            );
            true
        } else {
            false
        }
    }

    /// Update baselines from triangulation
    pub fn update_baselines(&mut self, point_id: usize, baseline: Float) {
        if let Some(point) = self.points.get_mut(&point_id) {
            point.baselines.push(baseline);
            // Keep only recent baselines
            if point.baselines.len() > 10 {
                point.baselines.remove(0);
            }
        }
    }

    /// Advance to next keyframe
    pub fn advance_keyframe(&mut self) {
        self.current_keyframe += 1;
    }

    /// Score all points and identify culling candidates
    pub fn score_all_points(&mut self) -> Vec<PointQuality> {
        let mut qualities = Vec::new();

        for (id, point) in &self.points {
            let quality = self.score_point(id, point);
            qualities.push(quality);
        }

        // Sort by quality for culling decisions
        qualities.sort_by(|a, b| a.quality_score.partial_cmp(&b.quality_score).unwrap());

        // Mark culling candidates if auto-cull enabled
        if self.config.auto_cull {
            let cull_count = (self.points.len() as Float * self.config.cull_fraction) as usize;
            for (idx, quality) in qualities.iter_mut().enumerate() {
                if idx < cull_count || quality.quality_score < self.config.quality_threshold {
                    quality.should_cull = true;
                    if quality.cull_reason.is_none() {
                        quality.cull_reason = Some(PointCullReason::Old);
                    }
                }
            }
        }

        qualities
    }

    /// Score a single point
    fn score_point(&self, id: &usize, point: &TrackedPoint) -> PointQuality {
        let track_length = point.track_length();
        let age = point.age(self.current_keyframe);

        // Compute reprojection error statistics
        let (mean_error, _error_variance) = self.compute_error_stats(point);

        // Compute observation angle statistics
        let (mean_angle, angle_var) = self.compute_angle_stats(point);

        // Compute baseline diversity
        let baseline_div = self.compute_baseline_diversity(point);

        // Compute parallax quality
        let parallax = self.compute_parallax_quality(point);

        // Compute depth quality
        let depth = self.compute_depth_quality(point);

        // Determine if point is dynamic
        let is_dynamic = !point.is_static(&self.config);

        // Compute overall quality score
        let quality_score = self.compute_overall_quality(
            track_length,
            mean_error,
            mean_angle,
            angle_var,
            baseline_div,
            parallax,
            depth,
            age,
            is_dynamic,
        );

        // Determine culling reason
        let cull_reason = if is_dynamic {
            Some(PointCullReason::Dynamic)
        } else if track_length < self.config.min_track_length {
            Some(PointCullReason::ShortTrack)
        } else if mean_error > self.config.max_reproj_error {
            Some(PointCullReason::HighError)
        } else if parallax < self.config.min_parallax {
            Some(PointCullReason::SmallParallax)
        } else if depth < 0.1 {
            Some(PointCullReason::ShallowDepth)
        } else if age > self.config.max_point_age {
            Some(PointCullReason::Old)
        } else {
            None
        };

        PointQuality {
            point_id: *id,
            quality_score,
            track_length,
            mean_observation_angle: mean_angle,
            angle_variance: angle_var,
            mean_reproj_error: mean_error,
            baseline_diversity: baseline_div,
            parallax_quality: parallax,
            depth_quality: depth,
            age_keyframes: age,
            should_cull: cull_reason.is_some() || quality_score < self.config.quality_threshold,
            cull_reason,
        }
    }

    /// Compute reprojection error statistics
    fn compute_error_stats(&self, point: &TrackedPoint) -> (Float, Float) {
        if point.observations.is_empty() {
            return (0.0, 0.0);
        }

        let n = point.observations.len() as Float;
        let mean: Float = point
            .observations
            .iter()
            .map(|o| o.reproj_error)
            .sum::<Float>()
            / n;

        let variance: Float = point
            .observations
            .iter()
            .map(|o| (o.reproj_error - mean).powi(2))
            .sum::<Float>()
            / n;

        (mean, variance.sqrt())
    }

    /// Compute observation angle statistics
    fn compute_angle_stats(&self, point: &TrackedPoint) -> (Float, Float) {
        if point.observations.len() < 2 {
            return (0.0, 0.0);
        }

        // Compute angle between consecutive observations relative to point
        let mut angles = Vec::new();

        for i in 1..point.observations.len() {
            let o1 = &point.observations[i - 1];
            let o2 = &point.observations[i];

            // Direction vectors from point to observation
            let dir1 = na::Vector3::new(o1.pixel.x, o1.pixel.y, 1.0).normalize();
            let dir2 = na::Vector3::new(o2.pixel.x, o2.pixel.y, 1.0).normalize();

            // Angle between directions
            let dot = dir1.dot(&dir2);
            let angle = dot.clamp(-1.0, 1.0).acos();
            angles.push(angle);
        }

        if angles.is_empty() {
            return (0.0, 0.0);
        }

        let n = angles.len() as Float;
        let mean: Float = angles.iter().sum::<Float>() / n;
        let variance: Float = angles.iter().map(|a| (a - mean).powi(2)).sum::<Float>() / n;

        (mean, variance.sqrt())
    }

    /// Compute baseline diversity for triangulation quality
    fn compute_baseline_diversity(&self, point: &TrackedPoint) -> Float {
        if point.baselines.len() < 2 {
            return 0.5; // Neutral for insufficient data
        }

        let n = point.baselines.len() as Float;
        let mean: Float = point.baselines.iter().sum::<Float>() / n;

        let variance: Float = point
            .baselines
            .iter()
            .map(|b| (b - mean).powi(2))
            .sum::<Float>()
            / n;

        let std_dev = variance.sqrt();

        // Normalize by mean (coefficient of variation)
        let cv = std_dev / mean.max(0.01);

        // Higher diversity is better, capped at 1.0
        cv.min(1.0)
    }

    /// Compute parallax quality from observation spread
    fn compute_parallax_quality(&self, point: &TrackedPoint) -> Float {
        if point.observations.len() < 2 {
            return 0.0;
        }

        // Compute parallax from observation directions
        let mut max_parallax = 0.0 as Float;

        for i in 0..point.observations.len() {
            for j in (i + 1)..point.observations.len() {
                let o1 = &point.observations[i];
                let o2 = &point.observations[j];

                let dir1 = na::Vector3::new(o1.pixel.x, o1.pixel.y, 1.0).normalize();
                let dir2 = na::Vector3::new(o2.pixel.x, o2.pixel.y, 1.0).normalize();

                let dot = (dir1.dot(&dir2) as Float).clamp(-1.0 as Float, 1.0 as Float);
                let parallax = (1.0 as Float - dot).acos();
                max_parallax = max_parallax.max(parallax);
            }
        }

        // Normalize by minimum threshold
        ((max_parallax / self.config.min_parallax) as Float).min(1.0 as Float) as Float
    }

    /// Compute depth quality from point position
    fn compute_depth_quality(&self, point: &TrackedPoint) -> Float {
        let depth = point.position.norm();

        // Optimal depth is around 5-10 meters
        let optimal_depth = 5.0;
        let deviation = ((depth - optimal_depth) / optimal_depth).abs();

        // Quality decreases with deviation
        (1.0 - deviation).max(0.0)
    }

    /// Compute overall quality score
    fn compute_overall_quality(
        &self,
        track_length: u32,
        reproj_error: Float,
        mean_angle: Float,
        angle_var: Float,
        baseline_div: Float,
        parallax: Float,
        depth: Float,
        age: u32,
        is_dynamic: bool,
    ) -> Float {
        if is_dynamic {
            return 0.0;
        }

        // Normalize each component to 0-1
        let track_score = (track_length as Float / 10.0).min(1.0);
        let error_score = 1.0 - (reproj_error / self.config.max_reproj_error).min(1.0);
        let angle_score = (mean_angle / std::f32::consts::PI as Float).min(1.0);
        let variance_score = (angle_var / 0.5).min(1.0);
        let baseline_score = baseline_div;
        let parallax_score = parallax;
        let depth_score = depth;
        let age_score = 1.0 - (age as Float / self.config.max_point_age as Float).min(1.0);

        // Weighted combination
        let weights = [0.2, 0.25, 0.1, 0.1, 0.1, 0.1, 0.1, 0.05];
        let values = [
            track_score,
            error_score,
            angle_score,
            variance_score,
            baseline_score,
            parallax_score,
            depth_score,
            age_score,
        ];

        values
            .iter()
            .zip(weights.iter())
            .map(|(v, w)| *v * *w)
            .sum()
    }

    /// Cull poor quality points
    pub fn cull_points(&mut self) -> Vec<usize> {
        let qualities = self.score_all_points();

        let mut culled_ids = Vec::new();
        let mut points_to_remove = Vec::new();

        for quality in qualities {
            if quality.should_cull {
                points_to_remove.push(quality.point_id);
                culled_ids.push(quality.point_id);
                self.total_culled += 1;
            }
        }

        for id in points_to_remove {
            self.points.remove(&id);
        }

        culled_ids
    }

    /// Get map quality statistics
    pub fn map_quality(&mut self) -> MapQuality {
        let qualities = self.score_all_points();

        let good = qualities.iter().filter(|q| q.quality_score >= 0.7).count();
        let marginal = qualities
            .iter()
            .filter(|q| q.quality_score >= 0.3 && q.quality_score < 0.7)
            .count();
        let poor = qualities.iter().filter(|q| q.quality_score < 0.3).count();

        let mean_quality: Float = qualities.iter().map(|q| q.quality_score).sum::<Float>()
            / qualities.len().max(1) as Float;
        let mean_track: Float = qualities
            .iter()
            .map(|q| q.track_length as Float)
            .sum::<Float>()
            / qualities.len().max(1) as Float;
        let mean_error: Float = qualities.iter().map(|q| q.mean_reproj_error).sum::<Float>()
            / qualities.len().max(1) as Float;

        let cull_rec: Vec<usize> = qualities
            .iter()
            .filter(|q| q.should_cull)
            .map(|q| q.point_id)
            .collect();

        MapQuality {
            total_points: self.points.len(),
            good_points: good,
            marginal_points: marginal,
            poor_points: poor,
            mean_quality,
            mean_track_length: mean_track,
            mean_reproj_error: mean_error,
            cull_recommendations: cull_rec,
        }
    }

    /// Get point by ID
    pub fn get_point(&self, id: usize) -> Option<&TrackedPoint> {
        self.points.get(&id)
    }

    /// Get all points
    pub fn all_points(&self) -> &HashMap<usize, TrackedPoint> {
        &self.points
    }

    /// Reset the scorer
    pub fn reset(&mut self) {
        self.points.clear();
        self.next_point_id = 0;
        self.current_keyframe = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_point_creation() {
        let config = PointQualityConfig::default();
        let mut scorer = PointQualityScorer::new(config);

        let pos = na::Vector3::new(1.0, 2.0, 3.0);
        let id = scorer.add_point(pos);

        assert_eq!(id, 0);
    }

    #[test]
    fn test_observation_tracking() {
        let config = PointQualityConfig::default();
        let mut scorer = PointQualityScorer::new(config);

        let pos = na::Vector3::new(1.0, 2.0, 5.0);
        let id = scorer.add_point(pos);

        // Add observations
        let pixel = na::Vector2::new(100.0, 200.0);
        for i in 0..5 {
            scorer.add_observation(id, pixel, i as u32, 0.5 + i as Float * 0.1, i as u32);
            scorer.advance_keyframe();
        }

        let point = scorer.get_point(id).unwrap();
        assert_eq!(point.track_length(), 5);
        assert_eq!(point.first_keyframe, 0);
        assert_eq!(point.last_keyframe, 4);
    }

    #[test]
    fn test_quality_scoring() {
        let config = PointQualityConfig::default();
        let mut scorer = PointQualityScorer::new(config);

        let pos = na::Vector3::new(1.0, 2.0, 5.0);
        let id = scorer.add_point(pos);

        // Add observations with low reprojection error
        let pixel = na::Vector2::new(100.0, 200.0);
        for i in 0..10 {
            scorer.add_observation(id, pixel, i as u32, 0.5, i as u32);
            scorer.advance_keyframe();
        }

        let qualities = scorer.score_all_points();
        assert_eq!(qualities.len(), 1);

        let quality = &qualities[0];
        assert!(quality.quality_score > 0.5);
        assert!(!quality.should_cull);
    }

    #[test]
    fn test_culling_high_error() {
        let config = PointQualityConfig {
            max_reproj_error: 1.0, // Low threshold
            ..Default::default()
        };
        let mut scorer = PointQualityScorer::new(config);

        let pos = na::Vector3::new(1.0, 2.0, 5.0);
        let id = scorer.add_point(pos);

        // Add observations with high reprojection error
        let pixel = na::Vector2::new(100.0, 200.0);
        for i in 0..5 {
            scorer.add_observation(id, pixel, i as u32, 5.0, i as u32); // High error
        }

        let qualities = scorer.score_all_points();
        let quality = &qualities[0];

        assert_eq!(quality.cull_reason, Some(PointCullReason::HighError));
        assert!(quality.should_cull);
    }

    #[test]
    fn test_map_quality() {
        let config = PointQualityConfig::default();
        let mut scorer = PointQualityScorer::new(config);

        // Add good points with varied observation positions
        for i in 0..10 {
            let pos = na::Vector3::new(i as Float, i as Float, 5.0);
            let id = scorer.add_point(pos);

            // Vary pixel positions significantly to create angle variance
            for j in 0..10 {
                let pixel = na::Vector2::new(100.0 + j as Float * 10.0, 200.0 + j as Float * 5.0);
                scorer.add_observation(id, pixel, (i * 10 + j) as u32, 0.5, j as u32);
                scorer.advance_keyframe();
            }
        }

        let _qualities = scorer.score_all_points();

        let quality = scorer.map_quality();
        assert_eq!(quality.total_points, 10);
        // Check that we have at least some points with reasonable quality
        let reasonable = quality.good_points + quality.marginal_points;
        assert!(
            reasonable > 0,
            "no points with quality >= 0.3, good={}, marginal={}",
            quality.good_points,
            quality.marginal_points
        );
    }

    #[test]
    fn test_reset() {
        let config = PointQualityConfig::default();
        let mut scorer = PointQualityScorer::new(config);

        let pos = na::Vector3::new(1.0, 2.0, 3.0);
        scorer.add_point(pos);
        scorer.advance_keyframe();

        assert!(scorer.all_points().len() == 1);
        assert!(scorer.current_keyframe == 1);

        scorer.reset();

        assert!(scorer.all_points().len() == 0);
        assert!(scorer.current_keyframe == 0);
    }
}
