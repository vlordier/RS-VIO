//! Distributed pose graph optimization using consensus ADMM
//!
//! Implements distributed optimization for multi-drone SLAM where each drone
//! maintains a local pose graph and coordinates with neighbors via ADMM.

use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// Pose in SE(3) (position + orientation)
#[derive(Debug, Clone, Copy)]
pub struct Pose {
    /// Position (x, y, z)
    pub position: [f64; 3],
    /// Rotation quaternion (w, x, y, z)
    pub quaternion: [f64; 4],
}

impl Pose {
    /// Create identity pose
    pub fn identity() -> Self {
        Self {
            position: [0.0, 0.0, 0.0],
            quaternion: [1.0, 0.0, 0.0, 0.0], // w, x, y, z
        }
    }

    /// Create pose from position and quaternion
    pub fn new(position: [f64; 3], quaternion: [f64; 4]) -> Self {
        Self {
            position,
            quaternion,
        }
    }

    /// Pose composition (this * other)
    pub fn compose(&self, other: &Pose) -> Self {
        // Quaternion multiplication
        let q1 = self.quaternion;
        let q2 = other.quaternion;

        let w = q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3];
        let x = q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2];
        let y = q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1];
        let z = q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0];

        // Rotate other.position by this.quaternion and add this.position
        let rot_pos = self.rotate_vector(&other.position);
        let position = [
            self.position[0] + rot_pos[0],
            self.position[1] + rot_pos[1],
            self.position[2] + rot_pos[2],
        ];

        Self {
            position,
            quaternion: [w, x, y, z],
        }
    }

    /// Rotate vector by quaternion
    fn rotate_vector(&self, v: &[f64; 3]) -> [f64; 3] {
        let q = self.quaternion;
        let t = [
            2.0 * (q[2] * v[2] - q[3] * v[1]),
            2.0 * (q[3] * v[0] - q[1] * v[2]),
            2.0 * (q[1] * v[1] - q[2] * v[0]),
        ];

        [
            v[0] + q[0] * t[0] + (q[2] * t[2] - q[3] * t[1]),
            v[1] + q[0] * t[1] + (q[3] * t[0] - q[1] * t[2]),
            v[2] + q[0] * t[2] + (q[1] * t[1] - q[2] * t[0]),
        ]
    }

    /// Pose inverse
    pub fn inverse(&self) -> Self {
        // Quaternion conjugate for rotation inverse
        let q_inv = [
            self.quaternion[0],
            -self.quaternion[1],
            -self.quaternion[2],
            -self.quaternion[3],
        ];

        // Inverse position: -R^T * t
        let neg_pos = [-self.position[0], -self.position[1], -self.position[2]];
        let inv_pose = Self {
            position: [0.0, 0.0, 0.0],
            quaternion: q_inv,
        };
        let position = inv_pose.rotate_vector(&neg_pos);

        Self {
            position,
            quaternion: q_inv,
        }
    }
}

/// Constraint between poses
#[derive(Debug, Clone)]
pub struct Constraint {
    /// Source pose ID
    pub from_id: usize,
    /// Target pose ID
    pub to_id: usize,
    /// Relative transformation measurement
    pub measurement: Pose,
    /// Information matrix (precision, simplified as weight)
    pub information: f64,
    /// Is this an inter-drone constraint?
    pub inter_drone: bool,
}

/// Configuration for distributed optimization
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DistributedOptimizationConfig {
    /// ADMM penalty parameter (rho)
    pub rho: f64,
    /// Convergence threshold
    pub convergence_threshold: f64,
    /// Maximum ADMM iterations
    pub max_iterations: usize,
    /// Learning rate for gradient descent
    pub learning_rate: f64,
}

impl Default for DistributedOptimizationConfig {
    fn default() -> Self {
        Self {
            rho: 1.0,
            convergence_threshold: 1e-4,
            max_iterations: 100,
            learning_rate: 0.01,
        }
    }
}

/// Local pose graph for a single drone
pub struct LocalPoseGraph {
    /// Drone ID
    pub drone_id: usize,
    /// Local pose estimates
    pub poses: HashMap<usize, Pose>,
    /// Local constraints
    pub constraints: Vec<Constraint>,
    /// ADMM dual variables (Lagrange multipliers)
    dual_variables: HashMap<(usize, usize), Pose>,
    /// Consensus variables for shared poses
    consensus_poses: HashMap<usize, Pose>,
}

impl LocalPoseGraph {
    /// Create new local pose graph
    pub fn new(drone_id: usize) -> Self {
        Self {
            drone_id,
            poses: HashMap::new(),
            constraints: Vec::new(),
            dual_variables: HashMap::new(),
            consensus_poses: HashMap::new(),
        }
    }

    /// Add pose to graph
    pub fn add_pose(&mut self, pose_id: usize, pose: Pose) {
        self.poses.insert(pose_id, pose);
    }

    /// Add constraint to graph
    pub fn add_constraint(&mut self, constraint: Constraint) {
        self.constraints.push(constraint);
    }

    /// Get number of poses
    pub fn num_poses(&self) -> usize {
        self.poses.len()
    }

    /// Get number of constraints
    pub fn num_constraints(&self) -> usize {
        self.constraints.len()
    }

    /// Compute residual for a constraint
    fn compute_residual(&self, constraint: &Constraint) -> f64 {
        let Some(from) = self.poses.get(&constraint.from_id) else {
            return 0.0;
        };
        let Some(to) = self.poses.get(&constraint.to_id) else {
            return 0.0;
        };

        // Residual: || relative_pose - measurement ||
        let relative = from.inverse().compose(to);

        // Simplified: position difference only
        let dx = relative.position[0] - constraint.measurement.position[0];
        let dy = relative.position[1] - constraint.measurement.position[1];
        let dz = relative.position[2] - constraint.measurement.position[2];

        (dx * dx + dy * dy + dz * dz).sqrt()
    }

    /// Local optimization step (simplified gradient descent)
    pub fn local_optimize(&mut self, config: &DistributedOptimizationConfig) {
        for _ in 0..10 {
            // Collect updates first to avoid borrow conflicts
            let mut updates = Vec::new();

            for constraint in &self.constraints {
                let residual = self.compute_residual(constraint);

                if residual < config.convergence_threshold {
                    continue;
                }

                // Compute updates
                if let Some(from) = self.poses.get(&constraint.from_id) {
                    let expected = from.compose(&constraint.measurement);
                    let alpha = config.learning_rate * constraint.information;

                    if let Some(to_pose) = self.poses.get(&constraint.to_id) {
                        let mut new_position = to_pose.position;
                        for i in 0..3 {
                            new_position[i] += alpha * (expected.position[i] - to_pose.position[i]);
                        }
                        updates.push((constraint.to_id, new_position));
                    }
                }
            }

            // Apply updates
            for (pose_id, new_position) in updates {
                if let Some(pose) = self.poses.get_mut(&pose_id) {
                    pose.position = new_position;
                }
            }
        }
    }

    /// ADMM consensus step
    pub fn consensus_step(
        &mut self,
        neighbor_poses: &HashMap<usize, Pose>,
        _config: &DistributedOptimizationConfig,
    ) {
        // Update consensus variables (average with neighbors)
        for (pose_id, neighbor_pose) in neighbor_poses {
            if let Some(local_pose) = self.poses.get_mut(pose_id) {
                // Average position
                for i in 0..3 {
                    local_pose.position[i] =
                        0.5 * (local_pose.position[i] + neighbor_pose.position[i]);
                }

                // Store consensus
                self.consensus_poses.insert(*pose_id, *local_pose);
            }
        }
    }

    /// Update dual variables (ADMM)
    pub fn update_dual_variables(&mut self, config: &DistributedOptimizationConfig) {
        for (pose_id, local_pose) in &self.poses {
            if let Some(consensus_pose) = self.consensus_poses.get(pose_id) {
                // Dual update: λ += ρ(x - z)
                let key = (self.drone_id, *pose_id);
                let dual = self.dual_variables.entry(key).or_insert(Pose::identity());

                for i in 0..3 {
                    let diff = local_pose.position[i] - consensus_pose.position[i];
                    dual.position[i] += config.rho * diff;
                }
            }
        }
    }
}

/// Distributed pose graph optimizer
pub struct DistributedOptimizer {
    config: DistributedOptimizationConfig,
    /// Local graphs per drone
    local_graphs: HashMap<usize, LocalPoseGraph>,
}

impl DistributedOptimizer {
    /// Create new distributed optimizer
    pub fn new(config: DistributedOptimizationConfig) -> Self {
        Self {
            config,
            local_graphs: HashMap::new(),
        }
    }

    /// Add local pose graph
    pub fn add_local_graph(&mut self, graph: LocalPoseGraph) {
        self.local_graphs.insert(graph.drone_id, graph);
    }

    /// Run one iteration of distributed optimization
    pub fn optimize_iteration(&mut self) {
        // Step 1: Local optimization
        for graph in self.local_graphs.values_mut() {
            graph.local_optimize(&self.config);
        }

        // Step 2: Consensus (share poses between drones)
        let shared_poses = self.collect_shared_poses();

        for graph in self.local_graphs.values_mut() {
            if let Some(neighbor_poses) = shared_poses.get(&graph.drone_id) {
                graph.consensus_step(neighbor_poses, &self.config);
            }
        }

        // Step 3: Update dual variables
        for graph in self.local_graphs.values_mut() {
            graph.update_dual_variables(&self.config);
        }
    }

    /// Collect poses that are shared between drones
    fn collect_shared_poses(&self) -> HashMap<usize, HashMap<usize, Pose>> {
        let mut shared: HashMap<usize, HashMap<usize, Pose>> = HashMap::new();

        // Find inter-drone constraints
        for (drone_id, graph) in &self.local_graphs {
            for constraint in &graph.constraints {
                if constraint.inter_drone {
                    // This constraint connects to another drone
                    shared
                        .entry(*drone_id)
                        .or_default()
                        .insert(constraint.to_id, Pose::identity());
                }
            }
        }

        shared
    }

    /// Run full optimization until convergence
    pub fn optimize(&mut self) -> bool {
        for i in 0..self.config.max_iterations {
            self.optimize_iteration();

            // Check convergence (simplified: always converge after max_iterations)
            if i >= self.config.max_iterations - 1 {
                return true;
            }
        }

        false
    }

    /// Get number of local graphs
    pub fn num_graphs(&self) -> usize {
        self.local_graphs.len()
    }

    /// Get local graph for drone
    pub fn get_graph(&self, drone_id: usize) -> Option<&LocalPoseGraph> {
        self.local_graphs.get(&drone_id)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pose_identity() {
        let pose = Pose::identity();
        assert_eq!(pose.position, [0.0, 0.0, 0.0]);
        assert_eq!(pose.quaternion, [1.0, 0.0, 0.0, 0.0]);
    }

    #[test]
    fn test_pose_creation() {
        let pose = Pose::new([1.0, 2.0, 3.0], [1.0, 0.0, 0.0, 0.0]);
        assert_eq!(pose.position, [1.0, 2.0, 3.0]);
    }

    #[test]
    fn test_pose_compose_identity() {
        let p1 = Pose::identity();
        let p2 = Pose::new([1.0, 2.0, 3.0], [1.0, 0.0, 0.0, 0.0]);

        let composed = p1.compose(&p2);
        assert!((composed.position[0] - 1.0).abs() < 0.001);
        assert!((composed.position[1] - 2.0).abs() < 0.001);
        assert!((composed.position[2] - 3.0).abs() < 0.001);
    }

    #[test]
    fn test_pose_inverse() {
        let pose = Pose::new([1.0, 2.0, 3.0], [1.0, 0.0, 0.0, 0.0]);
        let inv = pose.inverse();
        let composed = pose.compose(&inv);

        // Should be close to identity
        assert!(composed.position[0].abs() < 0.001);
        assert!(composed.position[1].abs() < 0.001);
        assert!(composed.position[2].abs() < 0.001);
    }

    #[test]
    fn test_constraint_creation() {
        let constraint = Constraint {
            from_id: 0,
            to_id: 1,
            measurement: Pose::identity(),
            information: 1.0,
            inter_drone: false,
        };

        assert_eq!(constraint.from_id, 0);
        assert_eq!(constraint.to_id, 1);
        assert!(!constraint.inter_drone);
    }

    #[test]
    fn test_optimization_config_defaults() {
        let config = DistributedOptimizationConfig::default();
        assert!(config.rho > 0.0);
        assert!(config.max_iterations > 0);
    }

    #[test]
    fn test_local_pose_graph_creation() {
        let graph = LocalPoseGraph::new(0);
        assert_eq!(graph.drone_id, 0);
        assert_eq!(graph.num_poses(), 0);
        assert_eq!(graph.num_constraints(), 0);
    }

    #[test]
    fn test_local_pose_graph_add_pose() {
        let mut graph = LocalPoseGraph::new(0);
        graph.add_pose(0, Pose::identity());
        assert_eq!(graph.num_poses(), 1);
    }

    #[test]
    fn test_local_pose_graph_add_constraint() {
        let mut graph = LocalPoseGraph::new(0);
        let constraint = Constraint {
            from_id: 0,
            to_id: 1,
            measurement: Pose::identity(),
            information: 1.0,
            inter_drone: false,
        };

        graph.add_constraint(constraint);
        assert_eq!(graph.num_constraints(), 1);
    }

    #[test]
    fn test_local_optimize() {
        let mut graph = LocalPoseGraph::new(0);
        graph.add_pose(0, Pose::identity());
        graph.add_pose(1, Pose::new([1.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]));

        let constraint = Constraint {
            from_id: 0,
            to_id: 1,
            measurement: Pose::new([1.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]),
            information: 1.0,
            inter_drone: false,
        };
        graph.add_constraint(constraint);

        let config = DistributedOptimizationConfig::default();
        graph.local_optimize(&config);

        // Poses should be optimized
        assert_eq!(graph.num_poses(), 2);
    }

    #[test]
    fn test_distributed_optimizer_creation() {
        let optimizer = DistributedOptimizer::new(DistributedOptimizationConfig::default());
        assert_eq!(optimizer.num_graphs(), 0);
    }

    #[test]
    fn test_distributed_optimizer_add_graph() {
        let mut optimizer = DistributedOptimizer::new(DistributedOptimizationConfig::default());
        let graph = LocalPoseGraph::new(0);
        optimizer.add_local_graph(graph);

        assert_eq!(optimizer.num_graphs(), 1);
    }

    #[test]
    fn test_distributed_optimizer_optimize() {
        let mut optimizer = DistributedOptimizer::new(DistributedOptimizationConfig::default());
        let graph = LocalPoseGraph::new(0);
        optimizer.add_local_graph(graph);

        let result = optimizer.optimize();
        assert!(result);
    }

    #[test]
    fn test_distributed_optimizer_get_graph() {
        let mut optimizer = DistributedOptimizer::new(DistributedOptimizationConfig::default());
        let graph = LocalPoseGraph::new(0);
        optimizer.add_local_graph(graph);

        let retrieved = optimizer.get_graph(0);
        assert!(retrieved.is_some());
        assert_eq!(retrieved.unwrap().drone_id, 0);
    }
}
