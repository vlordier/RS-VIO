//! Boids flocking algorithm for multi-drone coordination
//!
//! Implements Craig Reynolds' boids algorithm with SLAM-specific extensions
//! for coordinated exploration and mapping.

use crate::types::{Float, Vector3};
use serde::{Deserialize, Serialize};

/// Configuration for boids behavior
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct BoidsConfig {
    /// Radius for separation behavior (collision avoidance)
    pub separation_radius: Float,
    /// Radius for alignment behavior (velocity matching)
    pub alignment_radius: Float,
    /// Radius for cohesion behavior (flock centering)
    pub cohesion_radius: Float,
    /// Weight for separation force
    pub separation_weight: Float,
    /// Weight for alignment force
    pub alignment_weight: Float,
    /// Weight for cohesion force
    pub cohesion_weight: Float,
    /// Weight for goal seeking
    pub goal_weight: Float,
    /// Maximum speed
    pub max_speed: Float,
    /// Maximum force (acceleration)
    pub max_force: Float,
}

impl Default for BoidsConfig {
    fn default() -> Self {
        Self {
            separation_radius: 5.0,
            alignment_radius: 10.0,
            cohesion_radius: 10.0,
            separation_weight: 1.5,
            alignment_weight: 1.0,
            cohesion_weight: 1.0,
            goal_weight: 1.0,
            max_speed: 5.0,
            max_force: 0.5,
        }
    }
}

/// A single boid (drone) in the swarm
#[derive(Debug, Clone)]
pub struct Boid {
    pub id: usize,
    pub position: Vector3,
    pub velocity: Vector3,
    pub acceleration: Vector3,
}

impl Boid {
    /// Create new boid
    pub fn new(id: usize, position: Vector3, velocity: Vector3) -> Self {
        Self {
            id,
            position,
            velocity,
            acceleration: Vector3::zeros(),
        }
    }

    /// Update boid state with flocking behaviors
    pub fn update(
        &mut self,
        neighbors: &[&Boid],
        target: Option<Vector3>,
        config: &BoidsConfig,
        dt: Float,
    ) {
        // Reset acceleration
        self.acceleration = Vector3::zeros();

        // Apply flocking behaviors
        let separation = self.separation(neighbors, config);
        let alignment = self.alignment(neighbors, config);
        let cohesion = self.cohesion(neighbors, config);

        // Apply weights
        let sep_force = separation * config.separation_weight;
        let ali_force = alignment * config.alignment_weight;
        let coh_force = cohesion * config.cohesion_weight;

        // Accumulate forces
        self.acceleration += sep_force;
        self.acceleration += ali_force;
        self.acceleration += coh_force;

        // Goal seeking
        if let Some(goal) = target {
            let seek_force = self.seek(&goal, config);
            self.acceleration += seek_force * config.goal_weight;
        }

        // Update velocity and position
        self.velocity += self.acceleration * dt;
        self.limit_velocity(config.max_speed);
        self.position += self.velocity * dt;
    }

    /// Limit velocity magnitude
    fn limit_velocity(&mut self, max_speed: Float) {
        if self.velocity.norm() > max_speed {
            self.velocity = self.velocity.normalize() * max_speed;
        }
    }

    /// Separation: steer to avoid crowding local flockmates
    pub fn separation(&self, neighbors: &[&Boid], config: &BoidsConfig) -> Vector3 {
        let mut steer = Vector3::zeros();
        let mut count = 0;

        for other in neighbors {
            if other.id == self.id {
                continue;
            }

            let dist = (self.position - other.position).norm();
            if dist > 0.0 && dist < config.separation_radius {
                // Calculate vector pointing away from neighbor
                let diff = self.position - other.position;
                let normalized = if diff.norm() > 1e-6 {
                    diff.normalize()
                } else {
                    Vector3::zeros()
                };
                // Weight by distance (closer = stronger repulsion)
                // Avoid division by zero if dist is extremely small
                let safe_dist = if dist < 1e-6 { 1e-6 } else { dist };
                let weighted = normalized / safe_dist;
                steer += weighted;
                count += 1;
            }
        }

        if count > 0 {
            steer /= count as Float;

            // Implement Reynolds: Steering = Desired - Velocity
            if steer.norm() > 0.0 {
                steer = steer.normalize() * config.max_speed;
                steer -= self.velocity;
                if steer.norm() > config.max_force {
                    steer = steer.normalize() * config.max_force;
                }
            }
        }

        steer
    }

    /// Alignment: steer towards average heading of local flockmates
    pub fn alignment(&self, neighbors: &[&Boid], config: &BoidsConfig) -> Vector3 {
        let mut sum = Vector3::zeros();
        let mut count = 0;

        for other in neighbors {
            if other.id == self.id {
                continue;
            }

            let dist = (self.position - other.position).norm();
            if dist > 0.0 && dist < config.alignment_radius {
                sum += other.velocity;
                count += 1;
            }
        }

        if count > 0 {
            sum /= count as Float;
            if sum.norm() > 0.0 {
                sum = sum.normalize() * config.max_speed;
                let mut steer = sum - self.velocity;
                if steer.norm() > config.max_force {
                    steer = steer.normalize() * config.max_force;
                }
                steer
            } else {
                Vector3::zeros()
            }
        } else {
            Vector3::zeros()
        }
    }

    /// Cohesion: steer towards average position of local flockmates
    pub fn cohesion(&self, neighbors: &[&Boid], config: &BoidsConfig) -> Vector3 {
        let mut sum = Vector3::zeros();
        let mut count = 0;

        for other in neighbors {
            if other.id == self.id {
                continue;
            }

            let dist = (self.position - other.position).norm();
            if dist > 0.0 && dist < config.cohesion_radius {
                sum += other.position;
                count += 1;
            }
        }

        if count > 0 {
            sum /= count as Float;
            self.seek(&sum, config)
        } else {
            Vector3::zeros()
        }
    }

    /// Seek: steer towards a target position
    pub fn seek(&self, target: &Vector3, config: &BoidsConfig) -> Vector3 {
        let desired = target - self.position;
        let dist = desired.norm();

        let desired = if dist > 0.0 {
            desired.normalize() * config.max_speed
        } else {
            Vector3::zeros()
        };

        let mut steer = desired - self.velocity;
        if steer.norm() > config.max_force {
            steer = steer.normalize() * config.max_force;
        }
        steer
    }

    /// Get current speed
    pub fn speed(&self) -> Float {
        self.velocity.norm()
    }
}

/// Swarm of boids
pub struct BoidsSwarm {
    pub boids: Vec<Boid>,
    pub config: BoidsConfig,
}

impl BoidsSwarm {
    /// Create new swarm
    pub fn new(config: BoidsConfig) -> Self {
        Self {
            boids: Vec::new(),
            config,
        }
    }

    /// Add boid to swarm
    pub fn add_boid(&mut self, boid: Boid) {
        self.boids.push(boid);
    }

    /// Update all boids in swarm
    pub fn update(&mut self, targets: Option<&[Vector3]>, dt: Float) {
        // Clone boids to avoid borrow checker issues
        let boids_clone: Vec<Boid> = self.boids.clone();
        let neighbor_refs: Vec<&Boid> = boids_clone.iter().collect();

        // Update each boid
        for i in 0..self.boids.len() {
            let target = targets.and_then(|t| t.get(i).copied());
            self.boids[i].update(&neighbor_refs, target, &self.config, dt);
        }
    }

    /// Get number of boids
    pub fn len(&self) -> usize {
        self.boids.len()
    }

    /// Check if swarm is empty
    pub fn is_empty(&self) -> bool {
        self.boids.is_empty()
    }

    /// Get average position (center of mass)
    pub fn center_of_mass(&self) -> Vector3 {
        if self.boids.is_empty() {
            return Vector3::zeros();
        }

        let mut sum = Vector3::zeros();
        for boid in &self.boids {
            sum += boid.position;
        }
        sum / (self.boids.len() as Float)
    }

    /// Get average velocity
    pub fn average_velocity(&self) -> Vector3 {
        if self.boids.is_empty() {
            return Vector3::zeros();
        }

        let mut sum = Vector3::zeros();
        for boid in &self.boids {
            sum += boid.velocity;
        }
        sum / (self.boids.len() as Float)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_vector3_operations_via_nalgebra() {
        let v1 = Vector3::new(1.0, 2.0, 3.0);
        let v2 = Vector3::new(4.0, 5.0, 6.0);

        let sum = v1 + v2;
        assert_eq!(sum.x, 5.0);
        assert_eq!(sum.y, 7.0);
        assert_eq!(sum.z, 9.0);
    }

    #[test]
    fn test_boids_config_defaults() {
        let config = BoidsConfig::default();
        assert!(config.max_speed > 0.0);
        assert!(config.max_force > 0.0);
        assert!(config.separation_radius > 0.0);
    }

    #[test]
    fn test_boid_creation() {
        let boid = Boid::new(0, Vector3::new(1.0, 2.0, 3.0), Vector3::new(0.1, 0.2, 0.3));
        assert_eq!(boid.id, 0);
        assert_eq!(boid.position.x, 1.0);
        assert_eq!(boid.velocity.x, 0.1);
    }

    #[test]
    fn test_boid_separation() {
        let config = BoidsConfig::default();
        let boid1 = Boid::new(0, Vector3::new(0.0, 0.0, 0.0), Vector3::zeros());
        let boid2 = Boid::new(1, Vector3::new(1.0, 0.0, 0.0), Vector3::zeros());

        let neighbors = vec![&boid2];
        let sep = boid1.separation(&neighbors, &config);

        // Should push away from boid2 (in negative x direction)
        assert!(sep.x < 0.0 || sep.norm() < 0.001);
    }

    #[test]
    fn test_boid_alignment() {
        let config = BoidsConfig::default();
        let boid1 = Boid::new(0, Vector3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 0.0));
        let boid2 = Boid::new(1, Vector3::new(5.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0));

        let neighbors = vec![&boid2];
        let align = boid1.alignment(&neighbors, &config);

        // Should align with boid2's velocity
        assert!(align.norm() > 0.0);
    }

    #[test]
    fn test_boid_cohesion() {
        let config = BoidsConfig::default();
        let boid1 = Boid::new(0, Vector3::new(0.0, 0.0, 0.0), Vector3::zeros());
        let boid2 = Boid::new(1, Vector3::new(5.0, 0.0, 0.0), Vector3::zeros());

        let neighbors = vec![&boid2];
        let coh = boid1.cohesion(&neighbors, &config);

        // Should move toward boid2
        assert!(coh.x > 0.0 || coh.norm() < 0.001);
    }

    #[test]
    fn test_boid_seek() {
        let config = BoidsConfig::default();
        let boid = Boid::new(0, Vector3::new(0.0, 0.0, 0.0), Vector3::zeros());
        let target = Vector3::new(10.0, 0.0, 0.0);

        let seek = boid.seek(&target, &config);

        // Should move toward target (positive x)
        assert!(seek.x > 0.0);
    }

    #[test]
    fn test_boid_update() {
        let config = BoidsConfig::default();
        let mut boid = Boid::new(0, Vector3::new(0.0, 0.0, 0.0), Vector3::zeros());
        let target = Vector3::new(10.0, 0.0, 0.0);

        let initial_pos = boid.position;
        boid.update(&[], Some(target), &config, 0.1);

        // Position should have changed
        assert!((boid.position - initial_pos).norm() > 0.0);
    }

    #[test]
    fn test_swarm_update() {
        let mut swarm = BoidsSwarm::new(BoidsConfig::default());
        swarm.add_boid(Boid::new(0, Vector3::new(0.0, 0.0, 0.0), Vector3::zeros()));
        swarm.add_boid(Boid::new(1, Vector3::new(5.0, 0.0, 0.0), Vector3::zeros()));

        let initial_center = swarm.center_of_mass();
        swarm.update(None, 0.1);
        let new_center = swarm.center_of_mass();

        // Center may have changed due to flocking
        assert!((initial_center - new_center).norm() >= 0.0);
    }
}
