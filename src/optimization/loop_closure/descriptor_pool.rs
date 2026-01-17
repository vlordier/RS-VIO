//! # Descriptor Buffer Pooling
//!
//! Provides reusable descriptor buffer management to eliminate allocation overhead
//! during hot-path loop closure matching.
//!
//! ## Motivation
//!
//! Loop closure detection processes keyframe descriptors in a tight loop:
//! ```text
//! New Keyframe → Extract ORB/LightGlue → Match Candidates (100-200 comparisons)
//!                    ↓                          ↓
//!              Allocate Vec<u8>          Allocate binary buffers
//! ```
//!
//! Each match allocates temporary descriptor buffers (~32-256 bytes).
//! Over 200 candidates, this adds 6-50 KB of allocations per keyframe.
//!
//! ## Solution
//!
//! Pre-allocate a pool of descriptors at detector creation time.
//! Matching reuses buffers via borrowing, eliminating allocations.
//!
//! ## Design
//!
//! - **`OrbBinaryPool`**: Reusable [u8; 32] buffers for ORB matching
//! - **`FloatDescriptorPool`**: Reusable Vec<Float> buffers for LightGlue
//! - **`HybridDescriptorPool`**: Combined pool supporting both formats
//! - **Ownership**: Owned by `LoopClosureDetector`, borrowed during matching

use crate::types::Float;
use std::sync::{Arc, Mutex as StdMutex};

/// Abstract descriptor buffer pool
pub trait DescriptorPool: Send + Sync {
    /// Get a borrowed descriptor buffer for matching.
    /// Buffers should be cleared by the pool before returning.
    fn acquire(&self) -> Arc<StdMutex<Vec<Float>>>;

    /// Release descriptor buffer back to pool (clears and reuses).
    fn release(&self, _buffer: Arc<StdMutex<Vec<Float>>>) {
        // Default: do nothing, buffer will be dropped
        // Subclasses can override to reuse in free list
    }

    /// Reset pool state (clear all cached buffers if any)
    fn reset(&self);

    /// Current utilization (for monitoring)
    fn utilization(&self) -> f32;
}

/// ORB binary descriptor pool: stores reusable [u8; 32] buffers
///
/// ORB descriptors are fixed-size (256 bits = 32 bytes), making them
/// ideal for pre-allocation. This pool eliminates allocations during
/// descriptor conversion and matching.
#[derive(Debug)]
pub struct OrbBinaryPool {
    /// Pre-allocated binary descriptor buffers (reused across matches)
    buffers: StdMutex<Vec<Vec<u8>>>,
    /// Configuration
    #[allow(dead_code)]
    capacity: usize,
    acquired: std::sync::atomic::AtomicUsize,
}

impl OrbBinaryPool {
    /// Create new ORB binary pool with capacity for N concurrent descriptors
    pub fn new(capacity: usize) -> Self {
        let mut buffers = Vec::with_capacity(capacity);
        for _ in 0..capacity {
            buffers.push(vec![0u8; 32]); // Pre-allocate 32-byte buffers
        }

        Self {
            buffers: StdMutex::new(buffers),
            capacity,
            acquired: std::sync::atomic::AtomicUsize::new(0),
        }
    }

    /// Acquire a binary descriptor buffer from the pool
    /// Returns a mutable reference to a 32-byte buffer
    pub fn acquire_binary(&self) -> Option<Vec<u8>> {
        let mut bufs = self.buffers.lock().expect("mutex poisoned");
        if let Some(buf) = bufs.pop() {
            self.acquired.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
            return Some(buf);
        }
        None
    }

    /// Release a binary descriptor buffer back to pool
    pub fn release_binary(&self, mut buffer: Vec<u8>) {
        buffer.clear();
        buffer.resize(32, 0);
        let mut bufs = self.buffers.lock().expect("mutex poisoned");
        bufs.push(buffer);
        self.acquired.fetch_sub(1, std::sync::atomic::Ordering::Relaxed);
    }

    /// Current number of acquired buffers
    pub fn acquired_count(&self) -> usize {
        self.acquired.load(std::sync::atomic::Ordering::Relaxed)
    }
}

/// Float descriptor pool: stores reusable Vec<Float> buffers for LightGlue/SIFT-like descriptors
///
/// Handles variable-length descriptors (128-256 floats). Buffers are pre-sized
/// to common descriptor lengths and reused during matching.
#[derive(Debug)]
pub struct FloatDescriptorPool {
    /// Pre-allocated float descriptor buffers
    buffers: StdMutex<Vec<Vec<Float>>>,
    /// Capacity per buffer (e.g., 256 for LightGlue)
    buffer_size: usize,
    /// Maximum concurrent acquisitions
    max_concurrent: usize,
    acquired: std::sync::atomic::AtomicUsize,
}

impl FloatDescriptorPool {
    /// Create new float descriptor pool
    pub fn new(buffer_size: usize, max_concurrent: usize) -> Self {
        let mut buffers = Vec::with_capacity(max_concurrent);
        for _ in 0..max_concurrent {
            buffers.push(Vec::with_capacity(buffer_size));
        }

        Self {
            buffers: StdMutex::new(buffers),
            buffer_size,
            max_concurrent,
            acquired: std::sync::atomic::AtomicUsize::new(0),
        }
    }

    /// Acquire a float descriptor buffer from the pool
    pub fn acquire_float(&self) -> Option<Vec<Float>> {
        let mut bufs = self.buffers.lock().expect("mutex poisoned");
        if let Some(mut buf) = bufs.pop() {
            buf.clear();
            self.acquired.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
            return Some(buf);
        }
        None
    }

    /// Release a float descriptor buffer back to pool
    pub fn release_float(&self, mut buffer: Vec<Float>) {
        buffer.clear();
        let mut bufs = self.buffers.lock().expect("mutex poisoned");
        if buffer.capacity() >= self.buffer_size && bufs.len() < self.max_concurrent {
            bufs.push(buffer);
            self.acquired.fetch_sub(1, std::sync::atomic::Ordering::Relaxed);
        }
        // Otherwise: buffer is dropped if pool is full or capacity mismatch
    }

    /// Current number of acquired buffers
    pub fn acquired_count(&self) -> usize {
        self.acquired.load(std::sync::atomic::Ordering::Relaxed)
    }

    /// Available buffers in pool
    pub fn available_count(&self) -> usize {
        self.buffers.lock().expect("mutex poisoned").len()
    }
}

/// Combined descriptor pool supporting both ORB binary and float formats
#[derive(Debug)]
pub struct HybridDescriptorPool {
    /// Binary descriptor pool for ORB
    binary_pool: Option<Arc<OrbBinaryPool>>,
    /// Float descriptor pool for LightGlue/SIFT
    float_pool: Option<Arc<FloatDescriptorPool>>,
}

impl HybridDescriptorPool {
    /// Create new hybrid pool with both ORB and float support
    pub fn new(orb_capacity: Option<usize>, float_capacity: Option<(usize, usize)>) -> Self {
        let binary_pool = orb_capacity.map(OrbBinaryPool::new).map(Arc::new);
        let float_pool = float_capacity
            .map(|(size, concurrent)| FloatDescriptorPool::new(size, concurrent))
            .map(Arc::new);

        Self {
            binary_pool,
            float_pool,
        }
    }

    /// Get ORB binary pool (if available)
    pub fn binary(&self) -> Option<&Arc<OrbBinaryPool>> {
        self.binary_pool.as_ref()
    }

    /// Get float descriptor pool (if available)
    pub fn float(&self) -> Option<&Arc<FloatDescriptorPool>> {
        self.float_pool.as_ref()
    }

    /// Reset all pools
    pub fn reset(&self) {
        if let Some(_binary) = &self.binary_pool {
            // Binary pool has no explicit reset (buffers auto-clear on release)
        }
        if let Some(_float) = &self.float_pool {
            // Float pool has no explicit reset (buffers auto-clear on release)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_orb_binary_pool_lifecycle() {
        let pool = OrbBinaryPool::new(2);

        // Acquire first buffer
        let buf1 = pool.acquire_binary();
        assert!(buf1.is_some());
        assert_eq!(pool.acquired_count(), 1);

        // Acquire second buffer
        let buf2 = pool.acquire_binary();
        assert!(buf2.is_some());
        assert_eq!(pool.acquired_count(), 2);

        // Pool exhausted
        let buf3 = pool.acquire_binary();
        assert!(buf3.is_none());

        // Release first buffer
        pool.release_binary(buf1.unwrap());
        assert_eq!(pool.acquired_count(), 1);

        // Now can acquire again
        let buf4 = pool.acquire_binary();
        assert!(buf4.is_some());
        assert_eq!(pool.acquired_count(), 2);
    }

    #[test]
    fn test_float_pool_lifecycle() {
        let pool = FloatDescriptorPool::new(128, 2);

        let buf1 = pool.acquire_float();
        assert!(buf1.is_some());
        assert_eq!(pool.acquired_count(), 1);

        let buf2 = pool.acquire_float();
        assert!(buf2.is_some());
        assert_eq!(pool.acquired_count(), 2);

        let buf3 = pool.acquire_float();
        assert!(buf3.is_none());

        let mut buf = buf1.unwrap();
        buf.push(1.0);
        pool.release_float(buf);
        assert_eq!(pool.acquired_count(), 1);
        assert_eq!(pool.available_count(), 1);
    }

    #[test]
    fn test_hybrid_pool() {
        let pool = HybridDescriptorPool::new(Some(2), Some((128, 2)));

        assert!(pool.binary().is_some());
        assert!(pool.float().is_some());

        let _binary = pool.binary().unwrap().acquire_binary();
        let _float = pool.float().unwrap().acquire_float();

        pool.reset();
    }
}
