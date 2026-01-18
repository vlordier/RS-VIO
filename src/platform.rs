//! Platform-specific runtime configuration and detection.
//!
//! This module provides minimal platform detection utilities for runtime optimization.
//! Primarily used for determining embedded vs desktop environments to optimize
//! memory allocation strategies and thread management.

/// Detect if running on an embedded platform.
///
/// Returns true if running on ARM-based embedded systems (aarch64 or arm).
/// This is used to adjust memory allocation strategies and buffer pool sizes.
#[inline]
pub fn is_embedded() -> bool {
    cfg!(target_arch = "aarch64") || cfg!(target_arch = "arm")
}

/// Check if GPU acceleration is available.
///
/// Currently returns false as GPU acceleration is not yet implemented.
/// Reserved for future CUDA/OpenCL support.
#[inline]
pub fn has_gpu() -> bool {
    false
}

/// Get number of available CPU cores for parallel processing.
///
/// Used by the feature tracker and optimization modules to determine
/// optimal thread pool sizes for parallel operations.
#[inline]
pub fn num_cores() -> usize {
    std::thread::available_parallelism()
        .map(|n| n.get())
        .unwrap_or(1)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_platform_detection() {
        // Should not panic
        let _ = is_embedded();
        let _ = has_gpu();
        let cores = num_cores();
        assert!(cores > 0, "Should detect at least one core");
    }
}
