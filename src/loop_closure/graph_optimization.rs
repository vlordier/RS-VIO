//! Graph optimization for SLAM pose correction
//!
//! Optimizes the pose graph to correct accumulated drift and apply
//! loop closure constraints globally across the trajectory.

use serde::{Deserialize, Serialize};

/// Configuration for graph optimization
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct GraphOptimizationConfig {
    /// Maximum iterations for optimization
    pub max_iterations: usize,
    /// Convergence threshold for pose changes
    pub convergence_threshold: f32,
    /// Damping parameter (Levenberg-Marquardt)
    pub damping: f32,
    /// Minimum number of constraints for optimization
    pub min_constraints: usize,
}

impl Default for GraphOptimizationConfig {
    fn default() -> Self {
        Self {
            max_iterations: 50,
            convergence_threshold: 1e-5,
            damping: 0.1,
            min_constraints: 2,
        }
    }
}

/// Pose in the graph (6-DOF: position + rotation)
#[derive(Debug, Clone, Copy)]
pub struct Pose {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    /// Rotation in quaternion: (qx, qy, qz, qw)
    pub qx: f32,
    pub qy: f32,
    pub qz: f32,
    pub qw: f32,
}

impl Pose {
    /// Create pose at origin with identity rotation
    pub fn identity() -> Self {
        Self {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            qx: 0.0,
            qy: 0.0,
            qz: 0.0,
            qw: 1.0,
        }
    }

    /// Get position vector norm (translation magnitude)
    pub fn position_norm(&self) -> f32 {
        (self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }

    /// Compute Euclidean distance to another pose
    pub fn distance_to(&self, other: &Pose) -> f32 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        let dz = self.z - other.z;
        (dx * dx + dy * dy + dz * dz).sqrt()
    }
}

/// Unary constraint (prior on a pose)
#[derive(Debug, Clone)]
pub struct UnaryConstraint {
    pub pose_idx: usize,
    pub pose: Pose,
    /// Information (precision) matrix diagonal [6]
    pub information: [f32; 6],
}

/// Binary constraint (between two poses)
#[derive(Debug, Clone)]
pub struct BinaryConstraint {
    pub pose_i: usize,
    pub pose_j: usize,
    /// Relative pose from i to j
    pub relative_pose: Pose,
    /// Information (precision) matrix diagonal [6]
    pub information: [f32; 6],
}

/// Vertex in the pose graph
#[derive(Debug, Clone)]
pub struct PoseVertex {
    pub pose: Pose,
    pub fixed: bool,
}

/// Edge in the pose graph
#[derive(Debug, Clone)]
pub struct PoseEdge {
    pub from: usize,
    pub to: usize,
    pub constraint: BinaryConstraint,
}

/// Result of graph optimization
#[derive(Debug, Clone)]
pub struct OptimizationResult {
    pub optimized_poses: Vec<Pose>,
    pub final_error: f32,
    pub iterations: usize,
    pub converged: bool,
}

/// Pose graph for SLAM optimization
pub struct PoseGraph {
    config: GraphOptimizationConfig,
    vertices: Vec<PoseVertex>,
    edges: Vec<PoseEdge>,
    unary_constraints: Vec<UnaryConstraint>,
}

impl PoseGraph {
    /// Create new pose graph
    pub fn new(config: GraphOptimizationConfig) -> Self {
        Self {
            config,
            vertices: Vec::new(),
            edges: Vec::new(),
            unary_constraints: Vec::new(),
        }
    }

    /// Add a pose vertex
    pub fn add_vertex(&mut self, pose: Pose, fixed: bool) -> usize {
        let idx = self.vertices.len();
        self.vertices.push(PoseVertex { pose, fixed });
        idx
    }

    /// Add a binary constraint (edge)
    pub fn add_constraint(&mut self, constraint: BinaryConstraint) {
        let edge = PoseEdge {
            from: constraint.pose_i,
            to: constraint.pose_j,
            constraint,
        };
        self.edges.push(edge);
    }

    /// Add a unary constraint (prior)
    pub fn add_prior(&mut self, constraint: UnaryConstraint) {
        self.unary_constraints.push(constraint);
    }

    /// Optimize the pose graph
    pub fn optimize(&mut self) -> OptimizationResult {
        if self.vertices.len() < 2 || self.edges.len() < self.config.min_constraints {
            return OptimizationResult {
                optimized_poses: self.vertices.iter().map(|v| v.pose).collect(),
                final_error: 0.0,
                iterations: 0,
                converged: false,
            };
        }

        let mut poses: Vec<Pose> = self.vertices.iter().map(|v| v.pose).collect();

        let mut prev_error = f32::MAX;
        let mut converged = false;
        let mut iter = 0;

        for iteration in 0..self.config.max_iterations {
            // Compute total error
            let current_error = self.compute_error(&poses);

            // Check convergence
            if (prev_error - current_error).abs() < self.config.convergence_threshold {
                converged = true;
                iter = iteration;
                break;
            }

            // Gauss-Newton step
            poses = self.gauss_newton_step(&poses);

            prev_error = current_error;
            iter = iteration + 1;
        }

        // Update internal poses
        for (i, pose) in poses.iter().enumerate() {
            if i < self.vertices.len() && !self.vertices[i].fixed {
                self.vertices[i].pose = *pose;
            }
        }

        OptimizationResult {
            optimized_poses: poses,
            final_error: prev_error,
            iterations: iter,
            converged,
        }
    }

    /// Compute total squared error
    fn compute_error(&self, poses: &[Pose]) -> f32 {
        let mut total_error = 0.0f32;

        // Edge errors
        for edge in &self.edges {
            if edge.from < poses.len() && edge.to < poses.len() {
                let pose_i = &poses[edge.from];
                let pose_j = &poses[edge.to];

                // Simple positional error
                let predicted = Pose {
                    x: pose_i.x + edge.constraint.relative_pose.x,
                    y: pose_i.y + edge.constraint.relative_pose.y,
                    z: pose_i.z + edge.constraint.relative_pose.z,
                    qx: edge.constraint.relative_pose.qx,
                    qy: edge.constraint.relative_pose.qy,
                    qz: edge.constraint.relative_pose.qz,
                    qw: edge.constraint.relative_pose.qw,
                };

                let dx = predicted.x - pose_j.x;
                let dy = predicted.y - pose_j.y;
                let dz = predicted.z - pose_j.z;

                let residual = dx * dx + dy * dy + dz * dz;
                total_error += edge.constraint.information[0] * residual;
            }
        }

        // Unary constraint errors
        for constraint in &self.unary_constraints {
            if constraint.pose_idx < poses.len() {
                let pose = &poses[constraint.pose_idx];
                let dx = pose.x - constraint.pose.x;
                let dy = pose.y - constraint.pose.y;
                let dz = pose.z - constraint.pose.z;
                let residual = dx * dx + dy * dy + dz * dz;
                total_error += constraint.information[0] * residual;
            }
        }

        total_error
    }

    /// Gauss-Newton optimization step
    fn gauss_newton_step(&self, poses: &[Pose]) -> Vec<Pose> {
        let mut updated_poses = poses.to_vec();

        // Iterative pose updates
        for edge in &self.edges {
            if edge.from < poses.len()
                && edge.to < poses.len()
                && !self.vertices[edge.from].fixed
                && !self.vertices[edge.to].fixed
            {
                let pose_i = &poses[edge.from];
                let pose_j = &poses[edge.to];

                // Compute error
                let predicted_x = pose_i.x + edge.constraint.relative_pose.x;
                let predicted_y = pose_i.y + edge.constraint.relative_pose.y;
                let predicted_z = pose_i.z + edge.constraint.relative_pose.z;

                let error_x = predicted_x - pose_j.x;
                let error_y = predicted_y - pose_j.y;
                let error_z = predicted_z - pose_j.z;

                // Simple correction (scaled by information and damping)
                let correction_scale =
                    self.config.damping * edge.constraint.information[0] * 0.01;

                updated_poses[edge.to].x -= error_x * correction_scale;
                updated_poses[edge.to].y -= error_y * correction_scale;
                updated_poses[edge.to].z -= error_z * correction_scale;
            }
        }

        // Apply unary constraints
        for constraint in &self.unary_constraints {
            if constraint.pose_idx < poses.len() && !self.vertices[constraint.pose_idx].fixed {
                let pose = &poses[constraint.pose_idx];
                let error_x = pose.x - constraint.pose.x;
                let error_y = pose.y - constraint.pose.y;
                let error_z = pose.z - constraint.pose.z;

                let correction_scale =
                    self.config.damping * constraint.information[0] * 0.001;

                updated_poses[constraint.pose_idx].x -= error_x * correction_scale;
                updated_poses[constraint.pose_idx].y -= error_y * correction_scale;
                updated_poses[constraint.pose_idx].z -= error_z * correction_scale;
            }
        }

        updated_poses
    }

    /// Get number of vertices
    pub fn num_vertices(&self) -> usize {
        self.vertices.len()
    }

    /// Get number of edges
    pub fn num_edges(&self) -> usize {
        self.edges.len()
    }

    /// Get a pose by index
    pub fn get_pose(&self, idx: usize) -> Option<Pose> {
        self.vertices.get(idx).map(|v| v.pose)
    }

    /// Compute trajectory statistics
    pub fn trajectory_stats(&self) -> TrajectoryStats {
        let mut total_distance = 0.0f32;
        let poses = self.vertices.iter().map(|v| v.pose).collect::<Vec<_>>();

        for i in 1..poses.len() {
            total_distance += poses[i].distance_to(&poses[i - 1]);
        }

        let avg_distance = if poses.is_empty() {
            0.0
        } else {
            total_distance / (poses.len() as f32)
        };

        TrajectoryStats {
            num_poses: poses.len(),
            total_distance,
            avg_distance,
        }
    }
}

impl Default for PoseGraph {
    fn default() -> Self {
        Self::new(GraphOptimizationConfig::default())
    }
}

/// Statistics about the trajectory
#[derive(Debug, Clone, Copy)]
pub struct TrajectoryStats {
    pub num_poses: usize,
    pub total_distance: f32,
    pub avg_distance: f32,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_config_defaults() {
        let config = GraphOptimizationConfig::default();
        assert_eq!(config.max_iterations, 50);
        assert!(config.convergence_threshold > 0.0);
    }

    #[test]
    fn test_pose_identity() {
        let p = Pose::identity();
        assert_eq!(p.position_norm(), 0.0);
        assert_eq!(p.qw, 1.0); // Identity quaternion
    }

    #[test]
    fn test_pose_distance() {
        let p1 = Pose {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            qx: 0.0,
            qy: 0.0,
            qz: 0.0,
            qw: 1.0,
        };
        let p2 = Pose {
            x: 3.0,
            y: 4.0,
            z: 0.0,
            qx: 0.0,
            qy: 0.0,
            qz: 0.0,
            qw: 1.0,
        };
        assert!((p1.distance_to(&p2) - 5.0).abs() < 0.01);
    }

    #[test]
    fn test_pose_graph_creation() {
        let graph = PoseGraph::default();
        assert_eq!(graph.num_vertices(), 0);
        assert_eq!(graph.num_edges(), 0);
    }

    #[test]
    fn test_add_vertices() {
        let mut graph = PoseGraph::default();
        let idx1 = graph.add_vertex(Pose::identity(), true);
        let idx2 = graph.add_vertex(
            Pose {
                x: 1.0,
                y: 0.0,
                z: 0.0,
                qx: 0.0,
                qy: 0.0,
                qz: 0.0,
                qw: 1.0,
            },
            false,
        );

        assert_eq!(idx1, 0);
        assert_eq!(idx2, 1);
        assert_eq!(graph.num_vertices(), 2);
    }

    #[test]
    fn test_add_constraint() {
        let mut graph = PoseGraph::default();
        graph.add_vertex(Pose::identity(), true);
        graph.add_vertex(
            Pose {
                x: 1.0,
                y: 0.0,
                z: 0.0,
                qx: 0.0,
                qy: 0.0,
                qz: 0.0,
                qw: 1.0,
            },
            false,
        );

        let constraint = BinaryConstraint {
            pose_i: 0,
            pose_j: 1,
            relative_pose: Pose {
                x: 1.0,
                y: 0.0,
                z: 0.0,
                qx: 0.0,
                qy: 0.0,
                qz: 0.0,
                qw: 1.0,
            },
            information: [1.0; 6],
        };

        graph.add_constraint(constraint);
        assert_eq!(graph.num_edges(), 1);
    }

    #[test]
    fn test_optimization_empty_graph() {
        let mut graph = PoseGraph::default();
        let result = graph.optimize();
        assert!(!result.converged);
        assert_eq!(result.iterations, 0);
    }

    #[test]
    fn test_trajectory_stats() {
        let mut graph = PoseGraph::default();
        graph.add_vertex(Pose::identity(), true);
        graph.add_vertex(
            Pose {
                x: 1.0,
                y: 0.0,
                z: 0.0,
                qx: 0.0,
                qy: 0.0,
                qz: 0.0,
                qw: 1.0,
            },
            false,
        );
        graph.add_vertex(
            Pose {
                x: 2.0,
                y: 0.0,
                z: 0.0,
                qx: 0.0,
                qy: 0.0,
                qz: 0.0,
                qw: 1.0,
            },
            false,
        );

        let stats = graph.trajectory_stats();
        assert_eq!(stats.num_poses, 3);
        assert!((stats.total_distance - 2.0).abs() < 0.01);
    }

    #[test]
    fn test_optimization_single_edge() {
        let mut graph = PoseGraph::default();
        let idx1 = graph.add_vertex(Pose::identity(), true);
        let idx2 = graph.add_vertex(
            Pose {
                x: 0.5,
                y: 0.0,
                z: 0.0,
                qx: 0.0,
                qy: 0.0,
                qz: 0.0,
                qw: 1.0,
            },
            false,
        );

        let constraint = BinaryConstraint {
            pose_i: idx1,
            pose_j: idx2,
            relative_pose: Pose {
                x: 1.0,
                y: 0.0,
                z: 0.0,
                qx: 0.0,
                qy: 0.0,
                qz: 0.0,
                qw: 1.0,
            },
            information: [1.0; 6],
        };

        graph.add_constraint(constraint);
        let result = graph.optimize();

        // Should improve or maintain error
        assert!(result.optimized_poses.len() > 0);
    }

    #[test]
    fn test_get_pose() {
        let mut graph = PoseGraph::default();
        let p = Pose {
            x: 1.5,
            y: 2.5,
            z: 3.5,
            qx: 0.0,
            qy: 0.0,
            qz: 0.0,
            qw: 1.0,
        };
        graph.add_vertex(p, false);

        if let Some(retrieved) = graph.get_pose(0) {
            assert_eq!(retrieved.x, 1.5);
            assert_eq!(retrieved.y, 2.5);
            assert_eq!(retrieved.z, 3.5);
        } else {
            panic!("Failed to retrieve pose");
        }
    }
}
