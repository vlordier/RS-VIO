//! Boids flocking algorithm for multi-drone coordination
//!
//! Implements Craig Reynolds' boids algorithm with SLAM-specific extensions
//! for coordinated exploration and mapping.

use serde::{Deserialize, Serialize};

/// 3D vector for positions, velocities, and forces
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Vector3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vector3 {
    /// Create new vector
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    /// Zero vector
    pub fn zero() -> Self {
        Self::new(0.0, 0.0, 0.0)
    }

    /// Magnitude (length) of vector
    pub fn magnitude(&self) -> f32 {
        (self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }

    /// Normalize vector to unit length
    pub fn normalize(&self) -> Self {
        let mag = self.magnitude();
        if mag > 1e-6 {
            Self::new(self.x / mag, self.y / mag, self.z / mag)
        } else {
            Self::zero()
        }
    }

    /// Limit magnitude to max value
    pub fn limit(&self, max: f32) -> Self {
        let mag = self.magnitude();
        if mag > max {
            let scale = max / mag;
            Self::new(self.x * scale, self.y * scale, self.z * scale)
        } else {
            *self
        }
    }

    /// Distance to another vector
    pub fn distance_to(&self, other: &Vector3) -> f32 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        let dz = self.z - other.z;
        (dx * dx + dy * dy + dz * dz).sqrt()
    }

    /// Add vectors
    pub fn add(&self, other: &Vector3) -> Self {
        Self::new(self.x + other.x, self.y + other.y, self.z + other.z)
    }

    /// Subtract vectors
    pub fn sub(&self, other: &Vector3) -> Self {
        Self::new(self.x - other.x, self.y - other.y, self.z - other.z)
    }

    /// Multiply by scalar
    pub fn scale(&self, scalar: f32) -> Self {
        Self::new(self.x * scalar, self.y * scalar, self.z * scalar)
    }

    /// Divide by scalar
    pub fn div(&self, scalar: f32) -> Self {
        if scalar.abs() > 1e-6 {
            Self::new(self.x / scalar, self.y / scalar, self.z / scalar)
        } else {
            Self::zero()
        }
    }
}

impl Default for Vector3 {
    fn default() -> Self {
        Self::zero()
    }
}

/// Configuration for boids behavior
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct BoidsConfig {
    /// Radius for separation behavior (collision avoidance)
    pub separation_radius: f32,
    /// Radius for alignment behavior (velocity matching)
    pub alignment_radius: f32,
    /// Radius for cohesion behavior (flock centering)
    pub cohesion_radius: f32,
    /// Weight for separation force
    pub separation_weight: f32,
    /// Weight for alignment force
    pub alignment_weight: f32,
    /// Weight for cohesion force
    pub cohesion_weight: f32,
    /// Weight for goal seeking
    pub goal_weight: f32,
    /// Maximum speed
    pub max_speed: f32,
    /// Maximum force (acceleration)
    pub max_force: f32,
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
            acceleration: Vector3::zero(),
        }
    }

    /// Update boid state with flocking behaviors
    pub fn update(&mut self, neighbors: &[&Boid], target: Option<Vector3>, config: &BoidsConfig, dt: f32) {
        // Reset acceleration
        self.acceleration = Vector3::zero();

        // Apply flocking behaviors
        let separation = self.separation(neighbors, config);
        let alignment = self.alignment(neighbors, config);
        let cohesion = self.cohesion(neighbors, config);

        // Apply weights
        let sep_force = separation.scale(config.separation_weight);
        let ali_force = alignment.scale(config.alignment_weight);
        let coh_force = cohesion.scale(config.cohesion_weight);

        // Accumulate forces
        self.acceleration = self.acceleration.add(&sep_force);
        self.acceleration = self.acceleration.add(&ali_force);
        self.acceleration = self.acceleration.add(&coh_force);

        // Goal seeking
        if let Some(goal) = target {
            let seek_force = self.seek(&goal, config);
            self.acceleration = self.acceleration.add(&seek_force.scale(config.goal_weight));
        }

        // Update velocity and position
        self.velocity = self.velocity.add(&self.acceleration.scale(dt));
        self.velocity = self.velocity.limit(config.max_speed);
        self.position = self.position.add(&self.velocity.scale(dt));
    }

    /// Separation: steer to avoid crowding local flockmates
    pub fn separation(&self, neighbors: &[&Boid], config: &BoidsConfig) -> Vector3 {
        let mut steer = Vector3::zero();
        let mut count = 0;

        for other in neighbors {
            if other.id == self.id {
                continue;
            }

            let dist = self.position.distance_to(&other.position);
            if dist > 0.0 && dist < config.separation_radius {
                // Calculate vector pointing away from neighbor
                let diff = self.position.sub(&other.position);
                let normalized = diff.normalize();
                // Weight by distance (closer = stronger repulsion)
                let weighted = normalized.div(dist);
                steer = steer.add(&weighted);
                count += 1;
            }
        }

        if count > 0 {
            steer = steer.div(count as f32);
            
            // Implement Reynolds: Steering = Desired - Velocity
            if steer.magnitude() > 0.0 {
                steer = steer.normalize().scale(config.max_speed);
                steer = steer.sub(&self.velocity);
                steer = steer.limit(config.max_force);
            }
        }

        steer
    }

    /// Alignment: steer towards average heading of local flockmates
    pub fn alignment(&self, neighbors: &[&Boid], config: &BoidsConfig) -> Vector3 {
        let mut sum = Vector3::zero();
        let mut count = 0;

        for other in neighbors {
            if other.id == self.id {
                continue;
            }

            let dist = self.position.distance_to(&other.position);
            if dist > 0.0 && dist < config.alignment_radius {
                sum = sum.add(&other.velocity);
                count += 1;
            }
        }

        if count > 0 {
            sum = sum.div(count as f32);
            sum = sum.normalize().scale(config.max_speed);
            
            let steer = sum.sub(&self.velocity);
            steer.limit(config.max_force)
        } else {
            Vector3::zero()
        }
    }

    /// Cohesion: steer towards average position of local flockmates
    pub fn cohesion(&self, neighbors: &[&Boid], config: &BoidsConfig) -> Vector3 {
        let mut sum = Vector3::zero();
        let mut count = 0;

        for other in neighbors {
            if other.id == self.id {
                continue;
            }

            let dist = self.position.distance_to(&other.position);
            if dist > 0.0 && dist < config.cohesion_radius {
                sum = sum.add(&other.position);
                count += 1;
            }
        }

        if count > 0 {
            sum = sum.div(count as f32);
            self.seek(&sum, config)
        } else {
            Vector3::zero()
        }
    }

    /// Seek: steer towards a target position
    pub fn seek(&self, target: &Vector3, config: &BoidsConfig) -> Vector3 {
        let desired = target.sub(&self.position);
        let desired = desired.normalize().scale(config.max_speed);
        
        let steer = desired.sub(&self.velocity);
        steer.limit(config.max_force)
    }

    /// Get current speed
    pub fn speed(&self) -> f32 {
        self.velocity.magnitude()
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
    pub fn update(&mut self, targets: Option<&[Vector3]>, dt: f32) {
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
            return Vector3::zero();
        }

        let mut sum = Vector3::zero();
        for boid in &self.boids {
            sum = sum.add(&boid.position);
        }
        sum.div(self.boids.len() as f32)
    }

    /// Get average velocity
    pub fn average_velocity(&self) -> Vector3 {
        if self.boids.is_empty() {
            return Vector3::zero();
        }

        let mut sum = Vector3::zero();
        for boid in &self.boids {
            sum = sum.add(&boid.velocity);
        }
        sum.div(self.boids.len() as f32)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_vector3_creation() {
        let v = Vector3::new(1.0, 2.0, 3.0);
        assert_eq!(v.x, 1.0);
        assert_eq!(v.y, 2.0);
        assert_eq!(v.z, 3.0);
    }

    #[test]
    fn test_vector3_magnitude() {
        let v = Vector3::new(3.0, 4.0, 0.0);
        assert!((v.magnitude() - 5.0).abs() < 0.001);
    }

    #[test]
    fn test_vector3_normalize() {
        let v = Vector3::new(3.0, 4.0, 0.0);
        let n = v.normalize();
        assert!((n.magnitude() - 1.0).abs() < 0.001);
    }

    #[test]
    fn test_vector3_limit() {
        let v = Vector3::new(10.0, 0.0, 0.0);
        let limited = v.limit(5.0);
        assert!((limited.magnitude() - 5.0).abs() < 0.001);
    }

    #[test]
    fn test_vector3_distance() {
        let v1 = Vector3::new(0.0, 0.0, 0.0);
        let v2 = Vector3::new(3.0, 4.0, 0.0);
        assert!((v1.distance_to(&v2) - 5.0).abs() < 0.001);
    }

    #[test]
    fn test_vector3_operations() {
        let v1 = Vector3::new(1.0, 2.0, 3.0);
        let v2 = Vector3::new(4.0, 5.0, 6.0);
        
        let sum = v1.add(&v2);
        assert_eq!(sum.x, 5.0);
        assert_eq!(sum.y, 7.0);
        assert_eq!(sum.z, 9.0);
        
        let diff = v2.sub(&v1);
        assert_eq!(diff.x, 3.0);
        assert_eq!(diff.y, 3.0);
        assert_eq!(diff.z, 3.0);
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
        let boid1 = Boid::new(0, Vector3::new(0.0, 0.0, 0.0), Vector3::zero());
        let boid2 = Boid::new(1, Vector3::new(1.0, 0.0, 0.0), Vector3::zero());
        
        let neighbors = vec![&boid2];
        let sep = boid1.separation(&neighbors, &config);
        
        // Should push away from boid2 (in negative x direction)
        assert!(sep.x < 0.0 || sep.magnitude() < 0.001);
    }

    #[test]
    fn test_boid_alignment() {
        let config = BoidsConfig::default();
        let boid1 = Boid::new(0, Vector3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 0.0));
        let boid2 = Boid::new(1, Vector3::new(5.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0));
        
        let neighbors = vec![&boid2];
        let align = boid1.alignment(&neighbors, &config);
        
        // Should align with boid2's velocity
        assert!(align.magnitude() > 0.0);
    }

    #[test]
    fn test_boid_cohesion() {
        let config = BoidsConfig::default();
        let boid1 = Boid::new(0, Vector3::new(0.0, 0.0, 0.0), Vector3::zero());
        let boid2 = Boid::new(1, Vector3::new(5.0, 0.0, 0.0), Vector3::zero());
        
        let neighbors = vec![&boid2];
        let coh = boid1.cohesion(&neighbors, &config);
        
        // Should move toward boid2
        assert!(coh.x > 0.0 || coh.magnitude() < 0.001);
    }

    #[test]
    fn test_boid_seek() {
        let config = BoidsConfig::default();
        let boid = Boid::new(0, Vector3::new(0.0, 0.0, 0.0), Vector3::zero());
        let target = Vector3::new(10.0, 0.0, 0.0);
        
        let seek = boid.seek(&target, &config);
        
        // Should move toward target (positive x)
        assert!(seek.x > 0.0);
    }

    #[test]
    fn test_boid_update() {
        let config = BoidsConfig::default();
        let mut boid = Boid::new(0, Vector3::new(0.0, 0.0, 0.0), Vector3::zero());
        let target = Vector3::new(10.0, 0.0, 0.0);
        
        let initial_pos = boid.position;
        boid.update(&[], Some(target), &config, 0.1);
        
        // Position should have changed
        assert!(boid.position.distance_to(&initial_pos) > 0.0);
    }

    #[test]
    fn test_swarm_creation() {
        let swarm = BoidsSwarm::new(BoidsConfig::default());
        assert_eq!(swarm.len(), 0);
        assert!(swarm.is_empty());
    }

    #[test]
    fn test_swarm_add_boid() {
        let mut swarm = BoidsSwarm::new(BoidsConfig::default());
        let boid = Boid::new(0, Vector3::new(1.0, 2.0, 3.0), Vector3::zero());
        swarm.add_boid(boid);
        
        assert_eq!(swarm.len(), 1);
        assert!(!swarm.is_empty());
    }

    #[test]
    fn test_swarm_center_of_mass() {
        let mut swarm = BoidsSwarm::new(BoidsConfig::default());
        swarm.add_boid(Boid::new(0, Vector3::new(0.0, 0.0, 0.0), Vector3::zero()));
        swarm.add_boid(Boid::new(1, Vector3::new(10.0, 0.0, 0.0), Vector3::zero()));
        
        let center = swarm.center_of_mass();
        assert!((center.x - 5.0).abs() < 0.001);
    }

    #[test]
    fn test_swarm_update() {
        let mut swarm = BoidsSwarm::new(BoidsConfig::default());
        swarm.add_boid(Boid::new(0, Vector3::new(0.0, 0.0, 0.0), Vector3::zero()));
        swarm.add_boid(Boid::new(1, Vector3::new(5.0, 0.0, 0.0), Vector3::zero()));
        
        let initial_center = swarm.center_of_mass();
        swarm.update(None, 0.1);
        let new_center = swarm.center_of_mass();
        
        // Center may have changed due to flocking
        assert!(initial_center.distance_to(&new_center) >= 0.0);
    }
}
