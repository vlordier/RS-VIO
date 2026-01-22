/// Arena-backed allocation for feature tracking
/// 
/// This module provides efficient, zero-copy allocation strategies for
/// feature tracking data structures using typed-arena for automatic lifetime management.
///
/// ## Benefits
/// - **Zero-copy**: References don't require cloning
/// - **Cache-friendly**: All allocations are contiguous
/// - **Automatic cleanup**: All data freed when arena is dropped
/// - **Deterministic latency**: No unpredictable GC pauses
///
/// ## Typical Usage
/// ```rust,ignore
/// let arena = FeatureTrackingArena::new(num_features);
/// for feature in features {
///     let track = arena.alloc_track(feature);
///     tracker.add_track(track);
/// }
/// // All tracks freed automatically when arena dropped
/// ```

use typed_arena::Arena;

/// Feature track arena for zero-copy track allocation
pub struct FeatureTrackingArena {
    points_arena: Arena<[f32; 2]>,
    velocity_arena: Arena<[f32; 2]>,
    age_arena: Arena<u32>,
    confidence_arena: Arena<f32>,
}

impl FeatureTrackingArena {
    /// Create a new feature tracking arena with capacity hints
    pub fn new(capacity: usize) -> Self {
        Self {
            points_arena: Arena::with_capacity(capacity),
            velocity_arena: Arena::with_capacity(capacity),
            age_arena: Arena::with_capacity(capacity),
            confidence_arena: Arena::with_capacity(capacity),
        }
    }

    /// Allocate and store a tracked point with metadata
    /// Returns a handle that can be used to access the point
    pub fn alloc_point(&self, point: [f32; 2]) -> &'_ [f32; 2] {
        self.points_arena.alloc(point)
    }

    /// Allocate and store a velocity vector
    pub fn alloc_velocity(&self, velocity: [f32; 2]) -> &'_ [f32; 2] {
        self.velocity_arena.alloc(velocity)
    }

    /// Allocate track age (in frames)
    pub fn alloc_age(&self, age: u32) -> &'_ u32 {
        self.age_arena.alloc(age)
    }

    /// Allocate tracking confidence [0, 1]
    pub fn alloc_confidence(&self, confidence: f32) -> &'_ f32 {
        self.confidence_arena.alloc(confidence.clamp(0.0, 1.0))
    }

    /// Get arena statistics for monitoring allocation patterns
    pub fn stats(&self) -> ArenaStats {
        ArenaStats {
            points_allocated: self.points_arena.len(),
            velocity_allocated: self.velocity_arena.len(),
            age_allocated: self.age_arena.len(),
            confidence_allocated: self.confidence_arena.len(),
        }
    }
}

/// Statistics about arena allocations
#[derive(Clone, Debug)]
pub struct ArenaStats {
    pub points_allocated: usize,
    pub velocity_allocated: usize,
    pub age_allocated: usize,
    pub confidence_allocated: usize,
}

impl ArenaStats {
    pub fn total_allocations(&self) -> usize {
        self.points_allocated
            + self.velocity_allocated
            + self.age_allocated
            + self.confidence_allocated
    }
}

/// Descriptor arena for feature descriptors
pub struct DescriptorArena {
    binary_arena: Arena<Vec<u8>>,
    float_arena: Arena<Vec<f32>>,
}

impl DescriptorArena {
    pub fn new(capacity: usize) -> Self {
        Self {
            binary_arena: Arena::with_capacity(capacity),
            float_arena: Arena::with_capacity(capacity),
        }
    }

    /// Allocate a binary descriptor (e.g., ORB 256-bit)
    pub fn alloc_binary(&self, descriptor: Vec<u8>) -> &'_ Vec<u8> {
        self.binary_arena.alloc(descriptor)
    }

    /// Allocate a float descriptor (e.g., SIFT 128-dim)
    pub fn alloc_float(&self, descriptor: Vec<f32>) -> &'_ Vec<f32> {
        self.float_arena.alloc(descriptor)
    }
}

/// IMU data arena for efficient IMU measurement storage
pub struct ImuDataArena {
    timestamp_arena: Arena<i64>,
    accel_arena: Arena<[f32; 3]>,
    gyro_arena: Arena<[f32; 3]>,
}

impl ImuDataArena {
    pub fn new(capacity: usize) -> Self {
        Self {
            timestamp_arena: Arena::with_capacity(capacity),
            accel_arena: Arena::with_capacity(capacity),
            gyro_arena: Arena::with_capacity(capacity),
        }
    }

    /// Allocate IMU timestamp
    pub fn alloc_timestamp(&self, ts: i64) -> &'_ i64 {
        self.timestamp_arena.alloc(ts)
    }

    /// Allocate accelerometer reading
    pub fn alloc_accel(&self, accel: [f32; 3]) -> &'_ [f32; 3] {
        self.accel_arena.alloc(accel)
    }

    /// Allocate gyroscope reading
    pub fn alloc_gyro(&self, gyro: [f32; 3]) -> &'_ [f32; 3] {
        self.gyro_arena.alloc(gyro)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_feature_arena_allocation() {
        let arena = FeatureTrackingArena::new(100);

        let pt1 = arena.alloc_point([10.0, 20.0]);
        let pt2 = arena.alloc_point([30.0, 40.0]);

        assert_eq!(pt1[0], 10.0);
        assert_eq!(pt2[1], 40.0);
    }

    #[test]
    fn test_descriptor_arena() {
        let arena = DescriptorArena::new(50);

        let desc1 = arena.alloc_binary(vec![0x12, 0x34, 0x56]);
        let desc2 = arena.alloc_float(vec![0.1, 0.2, 0.3]);

        assert_eq!(desc1[0], 0x12);
        assert_eq!(desc2.len(), 3);
    }

    #[test]
    fn test_imu_arena() {
        let arena = ImuDataArena::new(1000);

        let ts = arena.alloc_timestamp(1000000);
        let accel = arena.alloc_accel([0.1, 0.2, 0.3]);
        let gyro = arena.alloc_gyro([0.01, 0.02, 0.03]);

        assert_eq!(*ts, 1000000);
        assert_eq!(accel[0], 0.1);
        assert_eq!(gyro[2], 0.03);
    }

    #[test]
    fn test_arena_stats() {
        let arena = FeatureTrackingArena::new(10);

        arena.alloc_point([1.0, 2.0]);
        arena.alloc_velocity([0.1, 0.2]);
        arena.alloc_age(5);
        arena.alloc_confidence(0.9);

        let stats = arena.stats();
        assert_eq!(stats.total_allocations(), 4);
    }
}
