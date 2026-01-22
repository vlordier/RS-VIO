/// Arena allocation integration for hot-path tracking and optimization
///
/// This module provides concrete integration patterns for using arena allocators
/// in the most performance-critical paths: feature tracking, descriptor matching,
/// and IMU data processing.

use crate::common::arena::{FeatureTrackingArena, DescriptorArena, ImuDataArena};
use std::fmt;

/// Context for arena-backed feature tracking operations
pub struct FeatureTrackingContext<'arena> {
    arena: &'arena FeatureTrackingArena,
    feature_count: usize,
    allocation_stats: AllocationStats,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct AllocationStats {
    /// Total features allocated in current frame
    pub features_allocated: usize,
    /// Total bytes allocated for points
    pub points_bytes: usize,
    /// Total bytes allocated for velocities
    pub velocity_bytes: usize,
    /// Total bytes allocated for metadata (age, confidence)
    pub metadata_bytes: usize,
}

impl AllocationStats {
    /// Get total bytes allocated across all arenas
    pub fn total_bytes(&self) -> usize {
        self.points_bytes + self.velocity_bytes + self.metadata_bytes
    }

    /// Estimate allocations saved compared to heap allocation
    /// (rough: 56 bytes per Vec per allocation + metadata)
    pub fn estimated_heap_saved(&self) -> usize {
        self.features_allocated * 56
    }
}

impl<'arena> FeatureTrackingContext<'arena> {
    /// Create a new tracking context with a reference to the arena
    pub fn new(arena: &'arena FeatureTrackingArena) -> Self {
        Self {
            arena,
            feature_count: 0,
            allocation_stats: AllocationStats::default(),
        }
    }

    /// Allocate a new feature point in the arena
    ///
    /// # Returns
    /// A reference to the allocated point that lives as long as the arena
    pub fn allocate_point(&mut self, x: f32, y: f32) -> &'arena [f32; 2] {
        let point = self.arena.alloc_point([x, y]);
        self.feature_count += 1;
        self.allocation_stats.features_allocated += 1;
        self.allocation_stats.points_bytes += 8; // 2 × f32
        point
    }

    /// Allocate a new velocity vector in the arena
    pub fn allocate_velocity(&mut self, vx: f32, vy: f32) -> &'arena [f32; 2] {
        let velocity = self.arena.alloc_velocity([vx, vy]);
        self.allocation_stats.velocity_bytes += 8;
        velocity
    }

    /// Allocate age and confidence metrics
    pub fn allocate_metadata(&mut self, age: u32, confidence: f32) -> (&'arena u32, &'arena f32) {
        let age_ref = self.arena.alloc_age(age);
        let conf_ref = self.arena.alloc_confidence(confidence);
        self.allocation_stats.metadata_bytes += 8; // 4 bytes age + 4 bytes confidence
        (age_ref, conf_ref)
    }

    /// Get the current allocation stats
    pub fn stats(&self) -> AllocationStats {
        self.allocation_stats
    }

    /// Log allocation efficiency
    pub fn log_efficiency(&self) {
        let heap_saved = self.allocation_stats.estimated_heap_saved();
        let actual_used = self.allocation_stats.total_bytes();
        let savings_ratio = if actual_used > 0 {
            heap_saved as f32 / actual_used as f32
        } else {
            0.0
        };

        log::debug!(
            "Arena tracking stats: {} features, {} bytes used, ~{} bytes heap saved ({:.1}x)",
            self.feature_count,
            actual_used,
            heap_saved,
            savings_ratio
        );
    }
}

impl fmt::Debug for FeatureTrackingContext<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("FeatureTrackingContext")
            .field("feature_count", &self.feature_count)
            .field("allocation_stats", &self.allocation_stats)
            .finish()
    }
}

/// Context for arena-backed descriptor allocation
pub struct DescriptorContext<'arena> {
    arena: &'arena DescriptorArena,
    descriptor_count: usize,
}

impl<'arena> DescriptorContext<'arena> {
    pub fn new(arena: &'arena DescriptorArena) -> Self {
        Self {
            arena,
            descriptor_count: 0,
        }
    }

    /// Allocate a binary descriptor (e.g., ORB, BRIEF)
    pub fn allocate_binary_descriptor(&mut self, descriptor: Vec<u8>) -> &'arena Vec<u8> {
        self.descriptor_count += 1;
        self.arena.alloc_binary(descriptor)
    }

    /// Allocate a float descriptor (e.g., SIFT, SURF)
    pub fn allocate_float_descriptor(&mut self, descriptor: Vec<f32>) -> &'arena Vec<f32> {
        self.descriptor_count += 1;
        self.arena.alloc_float(descriptor)
    }

    pub fn descriptor_count(&self) -> usize {
        self.descriptor_count
    }
}

/// Context for arena-backed IMU data allocation
pub struct ImuContext<'arena> {
    arena: &'arena ImuDataArena,
    sample_count: usize,
}

impl<'arena> ImuContext<'arena> {
    pub fn new(arena: &'arena ImuDataArena) -> Self {
        Self {
            arena,
            sample_count: 0,
        }
    }

    /// Allocate an IMU sample (timestamp, accel, gyro)
    pub fn allocate_sample(
        &mut self,
        timestamp: i64,
        accel: [f32; 3],
        gyro: [f32; 3],
    ) -> ImuSampleRefs<'arena> {
        self.sample_count += 1;
        ImuSampleRefs {
            timestamp: self.arena.alloc_timestamp(timestamp),
            accel: self.arena.alloc_accel(accel),
            gyro: self.arena.alloc_gyro(gyro),
        }
    }

    pub fn sample_count(&self) -> usize {
        self.sample_count
    }
}

/// References to arena-allocated IMU sample data
#[derive(Debug, Clone, Copy)]
pub struct ImuSampleRefs<'arena> {
    pub timestamp: &'arena i64,
    pub accel: &'arena [f32; 3],
    pub gyro: &'arena [f32; 3],
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_feature_tracking_context() {
        let arena = FeatureTrackingArena::new(100);
        let mut ctx = FeatureTrackingContext::new(&arena);

        let p1 = ctx.allocate_point(10.5, 20.3);
        assert_eq!(p1[0], 10.5);
        assert_eq!(p1[1], 20.3);

        let v1 = ctx.allocate_velocity(1.5, 2.5);
        assert_eq!(v1[0], 1.5);
        assert_eq!(v1[1], 2.5);

        let (age, conf) = ctx.allocate_metadata(5, 0.95);
        assert_eq!(*age, 5);
        assert_eq!(*conf, 0.95);

        assert_eq!(ctx.feature_count, 1);
        let stats = ctx.stats();
        assert_eq!(stats.features_allocated, 1);
        assert!(stats.total_bytes() > 0);
    }

    #[test]
    fn test_allocation_stats_efficiency() {
        let arena = FeatureTrackingArena::new(100);
        let mut ctx = FeatureTrackingContext::new(&arena);

        for i in 0..100 {
            ctx.allocate_point(i as f32, (i * 2) as f32);
        }

        let stats = ctx.stats();
        assert_eq!(stats.features_allocated, 100);
        assert!(stats.estimated_heap_saved() > stats.total_bytes());
    }

    #[test]
    fn test_descriptor_context() {
        let arena = DescriptorArena::new(100);
        let mut ctx = DescriptorContext::new(&arena);

        let binary = vec![0xAB, 0xCD, 0xEF];
        let desc_ref = ctx.allocate_binary_descriptor(binary);
        assert_eq!(desc_ref[0], 0xAB);
        assert_eq!(ctx.descriptor_count(), 1);

        let float = vec![0.1, 0.5, 0.9];
        let float_ref = ctx.allocate_float_descriptor(float);
        assert_eq!(float_ref[1], 0.5);
        assert_eq!(ctx.descriptor_count(), 2);
    }

    #[test]
    fn test_imu_context() {
        let arena = ImuDataArena::new(100);
        let mut ctx = ImuContext::new(&arena);

        let sample = ctx.allocate_sample(1000, [1.0, 2.0, 3.0], [0.1, 0.2, 0.3]);
        assert_eq!(*sample.timestamp, 1000);
        assert_eq!(sample.accel[0], 1.0);
        assert_eq!(sample.gyro[2], 0.3);
        assert_eq!(ctx.sample_count(), 1);
    }
}
