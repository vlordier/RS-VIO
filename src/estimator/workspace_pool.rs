//! Workspace Pooling Module
//!
//! Provides reusable workspaces to eliminate allocations in the hotpath.
//! Critical for real-time performance in VIO systems.
//!
//! Key optimizations:
//! - Pre-allocated buffers for feature tracking
//! - Image pyramid reuse
//! - Descriptor storage pooling

use crate::{
    estimator::{FrameWorkspace, WorkspaceConfig},
    traits::ResourcePool,
};
use std::sync::{Arc, Mutex};

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

/// Pool of reusable frame workspaces
///
/// Reduces allocation overhead in the hotpath by maintaining
/// a pool of pre-allocated workspace buffers.
pub struct WorkspacePool {
    frame_workspaces: Mutex<Vec<FrameWorkspace>>,
    config: WorkspaceConfig,
    max_pool_size: usize,
}

impl WorkspacePool {
    /// Create new workspace pool
    pub fn new(config: WorkspaceConfig, initial_size: usize, max_size: usize) -> Self {
        let mut frame_workspaces = Vec::with_capacity(initial_size);
        for _ in 0..initial_size {
            frame_workspaces.push(FrameWorkspace::new(config.clone()));
        }

        Self {
            frame_workspaces: Mutex::new(frame_workspaces),
            config,
            max_pool_size: max_size,
        }
    }

    /// Acquire a frame workspace from pool
    ///
    /// If pool is empty, creates a new workspace.
    /// This is the hotpath for frame processing.
    #[inline]
    #[allow(clippy::unwrap_used)] // Mutex poisoning is unrecoverable
    pub fn acquire_frame_workspace(&self) -> PooledFrameWorkspace<'_> {
        let workspace = {
            let mut pool = self.frame_workspaces.lock().unwrap();
            pool.pop()
                .unwrap_or_else(|| FrameWorkspace::new(self.config.clone()))
        };

        PooledFrameWorkspace {
            workspace: Some(workspace),
            pool: self,
        }
    }

    /// Return a frame workspace to the pool
    #[inline]
    #[allow(clippy::unwrap_used)] // Mutex poisoning is unrecoverable
    fn release_frame_workspace(&self, mut workspace: FrameWorkspace) {
        workspace.reset(); // Clear contents but keep allocations
        let mut pool = self.frame_workspaces.lock().unwrap();
        if pool.len() < self.max_pool_size {
            pool.push(workspace);
        }
        // Otherwise drop the workspace (pool is full)
    }

    /// Get current pool size
    #[allow(clippy::unwrap_used)] // Mutex poisoning is unrecoverable
    pub fn pool_size(&self) -> usize {
        self.frame_workspaces.lock().unwrap().len()
    }

    /// Clear all pooled workspaces (for testing)
    #[cfg(test)]
    #[allow(clippy::unwrap_used)] // Test code
    pub fn clear(&self) {
        self.frame_workspaces.lock().unwrap().clear();
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
        let mut pool = self
            .frame_workspaces
            .lock()
            .expect("workspace pool mutex poisoned");
        pool.pop()
            .unwrap_or_else(|| FrameWorkspace::new(self.config.clone()))
    }

    fn try_acquire(&self) -> Option<Self::Resource> {
        let mut pool = self
            .frame_workspaces
            .lock()
            .expect("workspace pool mutex poisoned");
        pool.pop()
    }

    fn release(&self, mut resource: Self::Resource) {
        resource.reset();
        let mut pool = self
            .frame_workspaces
            .lock()
            .expect("workspace pool mutex poisoned");
        if pool.len() < self.max_pool_size {
            pool.push(resource);
        }
    }

    fn reset(&self) {
        self.frame_workspaces
            .lock()
            .expect("workspace pool mutex poisoned")
            .clear();
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
        self.frame_workspaces
            .lock()
            .expect("workspace pool mutex poisoned")
            .len()
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
