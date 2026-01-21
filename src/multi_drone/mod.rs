//! Multi-drone SLAM with boids swarm coordination

pub mod boids;
pub mod map_merging;

pub use boids::{Boid, BoidsConfig, BoidsSwarm, Vector3};
pub use map_merging::{DroneMap, MapMerger, MapMergingConfig, Point3D, Transform};
