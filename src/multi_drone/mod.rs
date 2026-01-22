//! Multi-drone SLAM with boids swarm coordination

pub mod boids;
pub mod distributed_optimization;
pub mod exploration;
pub mod map_merging;

pub use boids::{Boid, BoidsConfig, BoidsSwarm};
pub use distributed_optimization::{
    Constraint, DistributedOptimizationConfig, DistributedOptimizer, LocalPoseGraph, Pose,
};
pub use exploration::{ExplorationConfig, ExplorationManager, ExplorationTask, Frontier, OccupancyGrid};
pub use map_merging::{DroneMap, MapMerger, MapMergingConfig, Point3D, Transform};
