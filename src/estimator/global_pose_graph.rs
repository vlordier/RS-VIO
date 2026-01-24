use std::collections::{BTreeMap, HashMap};
use crate::types::{Matrix3x3, Matrix4x4, Matrix6, Vector3, Isometry3};
use crate::estimator::Frame;
use crate::optimization::loop_closure::LoopClosureConstraint;
use crate::optimization::tight_coupling::ImuPreintegration;
use std::time::Instant;

/// Global pose graph backend for full SLAM.
///
/// Maintains all historical keyframe poses and accumulates loop closure
/// constraints. When loop closures form strong clusters, triggers global
/// bundle adjustment to correct the entire trajectory.
#[derive(Debug)]
pub struct GlobalPoseGraph {
    /// All historical keyframe poses indexed by frame ID
    /// Uses BTreeMap for ordered iteration and efficient range queries
    pub keyframe_poses: BTreeMap<u64, GlobalKeyframe>,

    /// All loop closure constraints from detector
    pub loop_closure_edges: Vec<LoopClosureEdge>,

    /// IMU preintegration edges between consecutive keyframes
    pub imu_edges: Vec<ImuEdge>,

    /// Map points from global optimization
    /// Key: feature_id, Value: point position and metadata
    pub map_points: HashMap<usize, GlobalMapPoint>,

    /// Configuration for optimization behavior
    pub config: GlobalPoseGraphConfig,

    /// Statistics and monitoring
    pub stats: GraphStatistics,

    /// Counter for new closures since last optimization
    pub new_closures_since_last_opt: usize,

    /// Track when last optimization occurred
    pub last_optimization_time: Option<Instant>,
}

/// A keyframe with associated pose and covariance
#[derive(Debug, Clone)]
pub struct GlobalKeyframe {
    pub id: u64,
    pub T_W_B: Matrix4x4,
    pub covariance: Matrix6,
    pub timestamp_ns: i64,
    pub is_marginalized: bool,
    /// Feature observations in left camera (2D normalized coordinates)
    /// Key: feature_id, Value: (x, y) in normalized coordinates
    pub left_feature_observations: Vec<(usize, (f64, f64))>,
    /// Feature observations in right camera
    pub right_feature_observations: Vec<(usize, (f64, f64))>,
    /// Body-to-left-camera calibration transform (T_C_B)
    pub T_B_Cl: Matrix4x4,
    /// Body-to-right-camera calibration transform (T_C_B)
    pub T_B_Cr: Matrix4x4,
}

/// Loop closure edge between two keyframes
#[derive(Debug, Clone)]
pub struct LoopClosureEdge {
    pub from_id: u64,
    pub to_id: u64,
    pub T_from_to: Isometry3,  // Relative transform from->to
    pub covariance: Matrix6,
    pub strength: f32,  // Quality metric from descriptor matching
}

/// IMU preintegration edge
#[derive(Debug, Clone)]
pub struct ImuEdge {
    pub from_id: u64,
    pub to_id: u64,
    pub preintegration: ImuPreintegration,
}

/// Global map point with observations
#[derive(Debug, Clone)]
pub struct GlobalMapPoint {
    pub id: usize,
    pub position: Vector3,
    pub descriptor: Vec<u8>,
    pub observations: Vec<(u64, usize)>,  // (keyframe_id, feature_id)
    pub covariance: Matrix3x3,
}

/// Configuration for global pose graph optimization
#[derive(Debug, Clone)]
pub struct GlobalPoseGraphConfig {
    /// Minimum loop closures before triggering optimization
    pub closure_threshold: usize,
    /// Maximum poses to keep in active set before marginalization
    pub max_poses_before_marginalization: usize,
    /// Maximum iterations for global BA
    pub max_iterations: usize,
    /// Cost tolerance for convergence
    pub cost_tolerance: f64,
    /// Enable detailed logging
    pub enable_logging: bool,
}

impl Default for GlobalPoseGraphConfig {
    fn default() -> Self {
        Self {
            closure_threshold: 5,
            max_poses_before_marginalization: 500,
            max_iterations: 50,
            cost_tolerance: 1e-7,
            enable_logging: false,
        }
    }
}

/// Statistics about the global pose graph
#[derive(Debug, Clone, Default)]
pub struct GraphStatistics {
    pub num_poses: usize,
    pub num_map_points: usize,
    pub num_loop_closures: usize,
    pub last_optimization_time_ms: f64,
    pub last_optimization_iterations: usize,
    pub total_optimizations: usize,
}

impl GlobalPoseGraph {
    /// Create a new global pose graph
    pub fn new(config: GlobalPoseGraphConfig) -> Self {
        Self {
            keyframe_poses: BTreeMap::new(),
            loop_closure_edges: Vec::new(),
            imu_edges: Vec::new(),
            map_points: HashMap::new(),
            config,
            stats: GraphStatistics::default(),
            new_closures_since_last_opt: 0,
            last_optimization_time: None,
        }
    }

    /// Add a new keyframe pose from the sliding window
    pub fn add_keyframe_pose(&mut self, frame: &Frame) {
        let keyframe_id = frame.frame_id as u64;
        
        // Create identity covariance for now (will be updated by optimization)
        let covariance = Matrix6::identity() * 1e-2;

        // Extract feature observations from left and right cameras
        // Use undistorted coordinates (normalized by camera intrinsics during optimization)
        let left_feature_observations: Vec<(usize, (f64, f64))> = frame
            .left_features
            .iter()
            .map(|feat| {
                // Use undistorted coordinates
                (feat.feature_id, (feat.undistorted_coord[0] as f64, feat.undistorted_coord[1] as f64))
            })
            .collect();

        let right_feature_observations: Vec<(usize, (f64, f64))> = frame
            .right_features
            .iter()
            .map(|feat| {
                (feat.feature_id, (feat.undistorted_coord[0] as f64, feat.undistorted_coord[1] as f64))
            })
            .collect();

        let keyframe = GlobalKeyframe {
            id: keyframe_id,
            T_W_B: frame.state.T_W_B,
            covariance,
            timestamp_ns: frame.timestamp_ns,
            is_marginalized: false,
            left_feature_observations,
            right_feature_observations,
            T_B_Cl: frame.state.T_B_Cl,
            T_B_Cr: frame.state.T_B_Cr,
        };

        self.keyframe_poses.insert(keyframe_id, keyframe);
        self.stats.num_poses = self.keyframe_poses.len();

        if self.config.enable_logging {
            log::debug!("[GlobalPoseGraph] Added keyframe {} (total: {})", 
                keyframe_id, self.stats.num_poses);
        }
    }

    /// Add a loop closure constraint
    pub fn add_loop_closure_constraint(&mut self, constraint: LoopClosureConstraint) {
        let edge = LoopClosureEdge {
            from_id: constraint.keyframe_id_1,
            to_id: constraint.keyframe_id_2,
            T_from_to: constraint.relative_pose,
            covariance: constraint.information_matrix.try_inverse()
                .unwrap_or_else(|| Matrix6::identity() * 1e-1),
            strength: 0.9,  // TODO: Use from constraint
        };

        self.loop_closure_edges.push(edge);
        self.new_closures_since_last_opt += 1;
        self.stats.num_loop_closures = self.loop_closure_edges.len();

        if self.config.enable_logging {
            log::debug!("[GlobalPoseGraph] Added loop closure {} -> {} (total: {})",
                constraint.keyframe_id_1, constraint.keyframe_id_2,
                self.stats.num_loop_closures);
        }
    }

    /// Add multiple loop closure constraints
    pub fn add_loop_closure_constraints(&mut self, constraints: Vec<LoopClosureConstraint>) {
        for constraint in constraints {
            self.add_loop_closure_constraint(constraint);
        }
    }

    /// Add IMU preintegration edge
    pub fn add_imu_edge(&mut self, from_id: u64, to_id: u64, preint: ImuPreintegration) {
        let edge = ImuEdge {
            from_id,
            to_id,
            preintegration: preint,
        };
        self.imu_edges.push(edge);
    }

    /// Add a map point observation
    pub fn add_map_point(
        &mut self,
        point_id: usize,
        position: Vector3,
        descriptor: Vec<u8>,
    ) {
        let point = GlobalMapPoint {
            id: point_id,
            position,
            descriptor,
            observations: Vec::new(),
            covariance: Matrix3x3::identity() * 1e-2,
        };

        self.map_points.insert(point_id, point);
        self.stats.num_map_points = self.map_points.len();
    }

    /// Add observation of map point from keyframe
    pub fn add_map_point_observation(
        &mut self,
        point_id: usize,
        keyframe_id: u64,
        feature_id: usize,
    ) {
        if let Some(point) = self.map_points.get_mut(&point_id) {
            point.observations.push((keyframe_id, feature_id));
        }
    }

    /// Check if we should trigger global optimization
    /// Returns (should_optimize, reason)
    pub fn should_optimize(&self) -> (bool, String) {
        // Threshold 1: New loop closures
        if self.new_closures_since_last_opt >= self.config.closure_threshold {
            return (
                true,
                format!(
                    "{} new loop closures >= threshold {}",
                    self.new_closures_since_last_opt, self.config.closure_threshold
                ),
            );
        }

        // Threshold 2: Memory pressure
        if self.keyframe_poses.len() > self.config.max_poses_before_marginalization {
            return (
                true,
                format!(
                    "{} poses > max {}",
                    self.keyframe_poses.len(),
                    self.config.max_poses_before_marginalization
                ),
            );
        }

        (false, "No optimization trigger".to_string())
    }

    /// Get pose of a keyframe
    pub fn get_pose(&self, keyframe_id: u64) -> Option<Matrix4x4> {
        self.keyframe_poses.get(&keyframe_id).map(|kf| kf.T_W_B)
    }

    /// Get all poses for updating sliding window
    pub fn get_all_poses(&self) -> Vec<(u64, Matrix4x4)> {
        self.keyframe_poses
            .iter()
            .map(|(id, kf)| (*id, kf.T_W_B))
            .collect()
    }

    /// Get map point position
    pub fn get_map_point(&self, point_id: usize) -> Option<Vector3> {
        self.map_points.get(&point_id).map(|p| p.position)
    }

    /// Number of keyframes in graph
    pub fn num_poses(&self) -> usize {
        self.keyframe_poses.len()
    }

    /// Number of loop closure constraints
    pub fn num_loop_closures(&self) -> usize {
        self.loop_closure_edges.len()
    }

    /// Get statistics
    pub fn stats(&self) -> &GraphStatistics {
        &self.stats
    }

    /// Apply optimized poses back to sliding window
    pub fn apply_poses_to_frames(&self, frames: &mut [Frame]) {
        for frame in frames {
            if let Some(pose) = self.get_pose(frame.frame_id as u64) {
                frame.state.T_W_B = pose;
            }
        }
    }

    /// Marginalize oldest poses when memory pressure exists
    pub fn marginalize_oldest(&mut self, num_to_marginalize: usize) {
        let num_to_remove = num_to_marginalize.min(self.keyframe_poses.len());
        
        if num_to_remove == 0 {
            return;
        }

        let ids_to_remove: Vec<u64> = self
            .keyframe_poses
            .iter()
            .take(num_to_remove)
            .map(|(id, _)| *id)
            .collect();

        for id in ids_to_remove {
            self.keyframe_poses.remove(&id);
            // In a real implementation, we would compute marginalization priors
            // and store them for future optimizations
        }

        self.stats.num_poses = self.keyframe_poses.len();

        if self.config.enable_logging {
            log::debug!("[GlobalPoseGraph] Marginalized {} poses", num_to_remove);
        }
    }

    /// Clear entire graph (for testing)
    pub fn clear(&mut self) {
        self.keyframe_poses.clear();
        self.loop_closure_edges.clear();
        self.imu_edges.clear();
        self.map_points.clear();
        self.new_closures_since_last_opt = 0;
        self.stats = GraphStatistics::default();
    }
}

/// Result of global optimization
#[derive(Debug, Clone)]
pub struct OptimizationResult {
    pub iterations: usize,
    pub final_cost: f64,
    pub converged: bool,
    pub optimization_time_ms: f64,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_create_graph() {
        let config = GlobalPoseGraphConfig::default();
        let graph = GlobalPoseGraph::new(config);

        assert_eq!(graph.num_poses(), 0);
        assert_eq!(graph.num_loop_closures(), 0);
    }

    #[test]
    fn test_should_optimize_closure_threshold() {
        let config = GlobalPoseGraphConfig {
            closure_threshold: 3,
            ..Default::default()
        };
        let mut graph = GlobalPoseGraph::new(config);

        // Add 5 poses directly without needing full Frame objects
        for i in 0..5 {
            let keyframe = GlobalKeyframe {
                id: i as u64,
                T_W_B: Matrix4x4::identity(),
                covariance: Matrix6::identity(),
                timestamp_ns: i as i64,
                is_marginalized: false,
                left_feature_observations: Vec::new(),
                right_feature_observations: Vec::new(),
                T_B_Cl: Matrix4x4::identity(),
                T_B_Cr: Matrix4x4::identity(),
            };
            graph.keyframe_poses.insert(i as u64, keyframe);
            graph.stats.num_poses += 1;
        }

        // Initially no optimization needed
        let (should_opt, _) = graph.should_optimize();
        assert!(!should_opt);

        // Add loop closures
        for i in 0..3 {
            let edge = LoopClosureEdge {
                from_id: i as u64,
                to_id: (4 - i) as u64,
                T_from_to: Isometry3::identity(),
                covariance: Matrix6::identity(),
                strength: 0.9,
            };
            graph.loop_closure_edges.push(edge);
            graph.new_closures_since_last_opt += 1;
        }
        graph.stats.num_loop_closures = graph.loop_closure_edges.len();

        // Now should optimize
        let (should_opt, reason) = graph.should_optimize();
        assert!(should_opt, "Reason: {}", reason);
    }

    #[test]
    fn test_marginalize_poses() {
        let config = GlobalPoseGraphConfig::default();
        let mut graph = GlobalPoseGraph::new(config);

        // Add 10 poses directly
        for i in 0..10 {
            let keyframe = GlobalKeyframe {
                id: i as u64,
                T_W_B: Matrix4x4::identity(),
                covariance: Matrix6::identity(),
                timestamp_ns: i as i64,
                is_marginalized: false,
                left_feature_observations: Vec::new(),
                right_feature_observations: Vec::new(),
                T_B_Cl: Matrix4x4::identity(),
                T_B_Cr: Matrix4x4::identity(),
            };
            graph.keyframe_poses.insert(i as u64, keyframe);
        }
        graph.stats.num_poses = 10;

        assert_eq!(graph.num_poses(), 10);

        // Marginalize first 5
        graph.marginalize_oldest(5);
        assert_eq!(graph.num_poses(), 5);

        // Verify old ones are gone
        assert!(graph.get_pose(0).is_none());
        assert!(graph.get_pose(5).is_some());
    }

    #[test]
    fn test_map_points() {
        let config = GlobalPoseGraphConfig::default();
        let mut graph = GlobalPoseGraph::new(config);

        let point = Vector3::new(1.0, 2.0, 3.0);
        graph.add_map_point(0, point, vec![]);

        let retrieved = graph.get_map_point(0);
        assert!(retrieved.is_some());
        assert_eq!(retrieved.unwrap(), point);
    }

    #[test]
    fn test_clear() {
        let config = GlobalPoseGraphConfig::default();
        let mut graph = GlobalPoseGraph::new(config);

        let keyframe = GlobalKeyframe {
            id: 0,
            T_W_B: Matrix4x4::identity(),
            covariance: Matrix6::identity(),
            timestamp_ns: 0,
            is_marginalized: false,
            left_feature_observations: Vec::new(),
            right_feature_observations: Vec::new(),
            T_B_Cl: Matrix4x4::identity(),
            T_B_Cr: Matrix4x4::identity(),
        };

        graph.keyframe_poses.insert(0, keyframe);
        graph.stats.num_poses = 1;
        assert_eq!(graph.num_poses(), 1);

        graph.clear();
        assert_eq!(graph.num_poses(), 0);
    }
}
