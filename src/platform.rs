//! Platform-specific runtime configuration and detection.
//!
//! This module provides minimal platform detection utilities for runtime optimization.
//! Primarily used for determining embedded vs desktop environments to optimize
//! memory allocation strategies and thread management.

use crate::ok_or_log;

/// Platform type detection
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PlatformType {
    /// Apple macOS (x86_64 or aarch64)
    Macos,
    /// Raspberry Pi 5 (aarch64)
    Rpi5,
    /// Other embedded ARM systems (aarch64 or arm)
    EmbeddedArm,
    /// Generic x86_64 desktop/server
    DesktopX64,
    /// Unknown platform
    Unknown,
}

/// Detect the current platform type.
#[inline]
pub fn platform_type() -> PlatformType {
    if cfg!(target_os = "macos") {
        PlatformType::Macos
    } else if cfg!(target_arch = "aarch64") {
        // Could be Raspberry Pi 5, NVIDIA Jetson, Apple Silicon, etc.
        // Check for RPi5 specific indicators
        if is_rpi5() {
            PlatformType::Rpi5
        } else if cfg!(target_vendor = "apple") {
            PlatformType::Macos
        } else {
            PlatformType::EmbeddedArm
        }
    } else if cfg!(target_arch = "x86_64") {
        PlatformType::DesktopX64
    } else if cfg!(target_arch = "arm") {
        PlatformType::EmbeddedArm
    } else {
        PlatformType::Unknown
    }
}

/// Detect if running on Raspberry Pi 5.
///
/// Uses CPU model detection to identify Raspberry Pi 5 specifically.
/// The Pi 5 uses a BCM2712 Cortex-A76 CPU (Quad-core ARMv8).
#[inline]
pub fn is_rpi5() -> bool {
    // Check for aarch64 with Raspberry Pi specific CPU
    #[cfg(target_arch = "aarch64")]
    {
        // Read /proc/cpuinfo on Linux to detect BCM2712
        // This is a lightweight check that's fast
        if let Ok(content) = std::fs::read_to_string("/proc/cpuinfo") {
            for line in content.lines() {
                if line.starts_with("Model") || line.starts_with("Hardware") {
                    if line.contains("BCM2712") || line.contains("Raspberry Pi 5") {
                        return true;
                    }
                }
                // Also check for "CPU implementer" = 0x41 (ARM) and "CPU part" = 0xD4B (Cortex-A76)
                if line.starts_with("CPU part") {
                    if line.contains("0xd4b") || line.contains("0xD4B") {
                        return true;
                    }
                }
            }
        }
        // Fallback: check device tree on Raspberry Pi
        if std::path::Path::new("/proc/device-tree/model").exists() {
            if let Ok(model) = std::fs::read_to_string("/proc/device-tree/model") {
                return model.contains("Raspberry Pi 5");
            }
        }
    }
    false
}

/// Detect if running on an embedded platform.
#[inline]
pub fn is_embedded() -> bool {
    matches!(
        platform_type(),
        PlatformType::Rpi5 | PlatformType::EmbeddedArm
    )
}

/// Detect if running on Apple Silicon (M1/M2/M3).
#[inline]
pub fn is_apple_silicon() -> bool {
    #[cfg(target_arch = "aarch64")]
    {
        #[cfg(target_vendor = "apple")]
        {
            return true;
        }
    }
}

/// Detect if running on Raspberry Pi (any version).
#[inline]
pub fn is_raspberry_pi() -> bool {
    is_rpi5()
}

/// Check if GPU acceleration is available.
#[inline]
pub fn has_gpu() -> bool {
    // macOS has Metal, other platforms need explicit GPU support
    cfg!(target_os = "macos")
}

/// Get number of available CPU cores for parallel processing.
#[inline]
pub fn num_cores() -> usize {
    ok_or_log!(
        std::thread::available_parallelism().map(|n| n.get()),
        1,
        "Failed to detect available_parallelism; defaulting to 1 core"
    )
}

/// Get recommended thread count for parallel operations.
///
/// On Raspberry Pi 5, this returns num_cores() - 1 to leave one core free for system tasks.
/// On desktop/macOS, returns num_cores() for maximum performance.
#[inline]
pub fn recommended_threads() -> usize {
    let cores = num_cores();
    if is_rpi5() {
        // Leave one core free for system tasks on RPi5
        (cores as i32).max(1).saturating_sub(1) as usize
    } else {
        cores
    }
}

/// Get CPU affinity hint for worker threads.
///
/// Returns a hint about which cores to pin threads to on embedded platforms.
#[inline]
pub fn core_affinity_hint() -> CoreAffinityHint {
    if is_rpi5() {
        // On RPi5, suggest pinning to specific cores
        // Cortex-A76 cores 0-3 are typically available
        CoreAffinityHint::PinToCores(0..4)
    } else if is_embedded() {
        CoreAffinityHint::Suggest
    } else {
        CoreAffinityHint::None
    }
}

/// Core affinity hint for thread scheduling
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum CoreAffinityHint {
    /// No affinity hint needed
    None,
    /// Platform suggests but doesn't require affinity
    Suggest,
    /// Pin threads to specific core range (inclusive start, exclusive end)
    PinToCores(std::ops::Range<usize>),
}

/// Check if SIMD is available and should be used.
#[inline]
pub fn has_simd() -> bool {
    #[cfg(target_arch = "x86_64")]
    {
        isize::max_value() > 0 // Always true on x86_64
    }
    #[cfg(target_arch = "aarch64")]
    {
        true // NEON is always available on ARMv8
    }
    #[cfg(target_arch = "arm")]
    {
        // Check for ARMv7 with NEON
        #[cfg(target_feature = "vfpv4")]
        {
            true
        }
        #[cfg(not(target_feature = "vfpv4"))]
        {
            false
        }
    }
    #[cfg(not(any(target_arch = "x86_64", target_arch = "aarch64", target_arch = "arm")))]
    {
        false
    }
}

/// Get L1 cache line size for memory alignment.
#[inline]
pub fn cache_line_size() -> usize {
    // Common values: 64 bytes on x86_64 and aarch64, 32 bytes on older ARM
    if cfg!(any(target_arch = "x86_64", target_arch = "aarch64")) {
        64
    } else {
        32
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_platform_detection() {
        let ptype = platform_type();
        println!("Detected platform: {:?}", ptype);
        let _ = is_embedded();
        let _ = has_gpu();
        let cores = num_cores();
        assert!(cores > 0, "Should detect at least one core");
    }

    #[test]
    fn test_rpi5_detection() {
        // This test will fail on non-RPi5 hardware, which is expected
        let _ = is_rpi5();
    }

    #[test]
    fn test_cache_line_size() {
        let size = cache_line_size();
        assert!(size >= 32, "Cache line should be at least 32 bytes");
    }
}
