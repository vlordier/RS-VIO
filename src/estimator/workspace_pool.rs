//! Workspace Pooling Module
//!
//! Provides reusable workspaces to eliminate allocations in the hotpath.
//! Critical for real-time performance in VIO systems.
//!
//! Key optimizations:
//! - Pre-allocated buffers for feature tracking
//! - Image pyramid reuse
//! - Descriptor storage pooling
//! - Lock-free atomic operations for hard real-time guarantees

use crate::{
    estimator::{FrameWorkspace, WorkspaceConfig},
    traits::ResourcePool,
};
use std::sync::{atomic::{AtomicBool, Ordering}, Arc, Mutex};
use std::cell::UnsafeCell;

/// Configuration for `WorkspacePool` used by the generic `ResourcePool` trait.
#[derive(Debug, Clone)]
pub struct WorkspacePoolConfig {
    pub workspace: WorkspaceConfig,
    pub initial_size: usize,
    pub max_size: usize,
}

impl Default for WorkspacePoolConfig {
    fn default() -> Self {
        Self {
            workspace: WorkspaceConfig::default(),
            initial_size: 4,
            max_size: 8,
        }
    }
}

/// Lock-free slot in the workspace pool
struct PoolSlot {
    workspace: UnsafeCell<Option<FrameWorkspace>>,
    available: AtomicBool,
}

#[allow(unsafe_code)]
unsafe impl Sync for PoolSlot {}

impl PoolSlot {
    fn new(workspace: FrameWorkspace) -> Self {
        Self {
            workspace: UnsafeCell::new(Some(workspace)),
            available: AtomicBool::new(true),
        }
    }

    fn empty() -> Self {
        Self {
            workspace: UnsafeCell::new(None),
            available: AtomicBool::new(false),
        }
    }

    /// Try to acquire this slot's workspace (lock-free)
    fn try_acquire(&self) -> Option<FrameWorkspace> {
        // Try to atomically claim this slot
        if self.available.compare_exchange(
            true,
            false,
            Ordering::Acquire,
            Ordering::Relaxed,
        ).is_ok() {
            // We successfully claimed it, extract the workspace
            #[allow(unsafe_code)]
            unsafe {
                (*self.workspace.get()).take()
            }
        } else {
            None
        }
    }

    /// Try to release workspace back to this slot (lock-free)
    #[allow(clippy::result_large_err)]
    fn try_release(&self, workspace: FrameWorkspace) -> Result<(), FrameWorkspace> {
        // Try to atomically claim this slot for release
        if self.available.compare_exchange(
            false, // Slot must be empty (not available)
            true,  // Mark as available after filling
            Ordering::Release,
            Ordering::Relaxed,
        ).is_ok() {
            // We successfully claimed an empty slot, fill it
            #[allow(unsafe_code)]
            unsafe {
                *self.workspace.get() = Some(workspace);
            }
            Ok(())
        } else {
            // Slot is already full
            Err(workspace)
        }
    }
}

/// Lock-free pool of reusable frame workspaces
///
/// Reduces allocation overhead in the hotpath by maintaining
/// a pool of pre-allocated workspace buffers with lock-free operations
/// for hard real-time guarantees.
///
/// Uses a fixed-size array with atomic operations to provide O(1)
/// lock-free acquire/release with zero contention in the common case.
pub struct WorkspacePool {
    /// Fixed-size array of workspace slots
    slots: Vec<PoolSlot>,
    /// Configuration for creating new workspaces
    config: WorkspaceConfig,
    /// Maximum pool size
    max_pool_size: usize,
    /// Fallback mutex-protected overflow storage
    overflow: Mutex<Vec<FrameWorkspace>>,
}

impl WorkspacePool {
    /// Create new lock-free workspace pool
    pub fn new(config: WorkspaceConfig, initial_size: usize, max_size: usize) -> Self {
        let mut slots = Vec::with_capacity(max_size);

        // Pre-allocate initial workspaces in lock-free slots
        for _ in 0..initial_size {
            slots.push(PoolSlot::new(FrameWorkspace::new(config.clone())));
        }

        // Fill remaining slots with empty markers
        for _ in initial_size..max_size {
            slots.push(PoolSlot::empty());
        }

        Self {
            slots,
            config,
            max_pool_size: max_size,
            overflow: Mutex::new(Vec::new()),
        }
    }

    /// Acquire a frame workspace from pool (lock-free fast path)
    ///
    /// If pool is empty, creates a new workspace.
    /// This is the hotpath for frame processing with zero lock contention.
    #[inline]
    pub fn acquire_frame_workspace(&self) -> PooledFrameWorkspace<'_> {
        // Fast path: try to acquire from lock-free slots
        for slot in &self.slots {
            if let Some(workspace) = slot.try_acquire() {
                return PooledFrameWorkspace {
                    workspace: Some(workspace),
                    pool: self,
                };
            }
        }

        // Fallback path: check overflow storage (rare)
        let workspace = if let Ok(mut overflow) = self.overflow.try_lock() {
            overflow.pop()
        } else {
            None
        };

        // Last resort: allocate new workspace
        let workspace = workspace.unwrap_or_else(|| FrameWorkspace::new(self.config.clone()));

        PooledFrameWorkspace {
            workspace: Some(workspace),
            pool: self,
        }
    }

    /// Return a frame workspace to the pool (lock-free fast path)
    #[inline]
    fn release_frame_workspace(&self, mut workspace: FrameWorkspace) {
        workspace.reset(); // Clear contents but keep allocations

        // Fast path: try to release to any empty lock-free slot
        for slot in &self.slots {
            match slot.try_release(workspace) {
                Ok(()) => return, // Successfully released
                Err(ws) => workspace = ws, // Slot was full, try next
            }
        }

        // If all slots are full, drop the workspace (pool at capacity)
        // Note: overflow is not used for storage to maintain strict pool size limit
    }

    /// Get current pool size (approximate, may race)
    pub fn pool_size(&self) -> usize {
        let mut count = 0;
        for slot in &self.slots {
            if slot.available.load(Ordering::Relaxed) {
                count += 1;
            }
        }
        count
    }

    /// Clear all pooled workspaces (for testing)
    #[cfg(test)]
    pub fn clear(&self) {
        for slot in &self.slots {
            // Try to acquire and drop
            let _ = slot.try_acquire();
        }
        if let Ok(mut overflow) = self.overflow.lock() {
            overflow.clear();
        }
    }
}

/// RAII wrapper for pooled frame workspace
///
/// Automatically returns workspace to pool on drop
pub struct PooledFrameWorkspace<'a> {
    workspace: Option<FrameWorkspace>,
    pool: &'a WorkspacePool,
}

impl<'a> PooledFrameWorkspace<'a> {
    /// Get mutable reference to workspace
    #[inline]
    #[allow(clippy::unwrap_used)] // Workspace is guaranteed Some by construction
    pub fn get_mut(&mut self) -> &mut FrameWorkspace {
        self.workspace.as_mut().unwrap()
    }

    /// Get reference to workspace
    #[inline]
    #[allow(clippy::unwrap_used)] // Workspace is guaranteed Some by construction
    pub fn get(&self) -> &FrameWorkspace {
        self.workspace.as_ref().unwrap()
    }
}

impl<'a> Drop for PooledFrameWorkspace<'a> {
    fn drop(&mut self) {
        if let Some(workspace) = self.workspace.take() {
            self.pool.release_frame_workspace(workspace);
        }
    }
}

lazy_static::lazy_static! {
    /// Global workspace pool instance
    ///
    /// Thread-safe singleton for workspace pooling across the application
    static ref GLOBAL_WORKSPACE_POOL: Arc<WorkspacePool> = {
        let config = WorkspaceConfig::default();
        Arc::new(WorkspacePool::new(config, 4, 8))
    };
}

/// Get the global workspace pool
pub fn global_pool() -> &'static Arc<WorkspacePool> {
    &GLOBAL_WORKSPACE_POOL
}

impl ResourcePool for WorkspacePool {
    type Resource = FrameWorkspace;
    type Config = WorkspacePoolConfig;

    fn new(config: Self::Config) -> Self {
        Self::new(config.workspace, config.initial_size, config.max_size)
    }

    fn acquire(&self) -> Self::Resource {
        // Try lock-free slots first
        for slot in &self.slots {
            if let Some(workspace) = slot.try_acquire() {
                return workspace;
            }
        }

        // Try overflow
        if let Ok(mut overflow) = self.overflow.lock() {
            if let Some(workspace) = overflow.pop() {
                return workspace;
            }
        }

        // Allocate new
        FrameWorkspace::new(self.config.clone())
    }

    fn try_acquire(&self) -> Option<Self::Resource> {
        // Try lock-free slots first
        for slot in &self.slots {
            if let Some(workspace) = slot.try_acquire() {
                return Some(workspace);
            }
        }

        // Try overflow
        if let Ok(mut overflow) = self.overflow.try_lock() {
            return overflow.pop();
        }

        None
    }

    fn release(&self, mut resource: Self::Resource) {
        resource.reset();
        self.release_frame_workspace(resource);
    }

    fn reset(&self) {
        for slot in &self.slots {
            let _ = slot.try_acquire();
        }
        if let Ok(mut overflow) = self.overflow.lock() {
            overflow.clear();
        }
    }

    fn utilization(&self) -> f32 {
        if self.max_pool_size == 0 {
            return 0.0;
        }
        let available = self.available();
        let used = self.max_pool_size.saturating_sub(available);
        used as f32 / self.max_pool_size as f32
    }

    fn capacity(&self) -> usize {
        self.max_pool_size
    }

    fn available(&self) -> usize {
        self.pool_size()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pool_creation() {
        let config = WorkspaceConfig::default();
        let pool = WorkspacePool::new(config, 2, 4);
        let frame_size = pool.pool_size();
        assert_eq!(frame_size, 2);
    }

    #[test]
    fn test_acquire_and_release_frame() {
        let config = WorkspaceConfig::default();
        let pool = WorkspacePool::new(config, 2, 4);
        {
            let _workspace1 = pool.acquire_frame_workspace();
            let _workspace2 = pool.acquire_frame_workspace();
            let frame_size = pool.pool_size();
            assert_eq!(frame_size, 0); // Both acquired
        }
        // Both should be returned
        let frame_size = pool.pool_size();
        assert_eq!(frame_size, 2);
    }

    #[test]
    fn test_acquire_more_than_pool_size() {
        let config = WorkspaceConfig::default();
        let pool = WorkspacePool::new(config, 1, 2);
        let _w1 = pool.acquire_frame_workspace();
        let _w2 = pool.acquire_frame_workspace(); // Creates new one
        let _w3 = pool.acquire_frame_workspace(); // Creates new one

        let frame_size = pool.pool_size();
        assert_eq!(frame_size, 0); // All acquired
    }

    #[test]
    fn test_pool_max_size_enforcement() {
        let config = WorkspaceConfig::default();
        let pool = WorkspacePool::new(config, 0, 2);
        {
            let _w1 = pool.acquire_frame_workspace();
            let _w2 = pool.acquire_frame_workspace();
            let _w3 = pool.acquire_frame_workspace();
            let _w4 = pool.acquire_frame_workspace();
        }
        // Only 2 should be kept (max pool size)
        let frame_size = pool.pool_size();
        assert_eq!(frame_size, 2);
    }

    #[test]
    fn test_global_pool() {
        let pool1 = global_pool();
        let pool2 = global_pool();
        assert!(Arc::ptr_eq(pool1, pool2));
    }
}
