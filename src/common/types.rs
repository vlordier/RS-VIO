//! Common type aliases and newtype wrappers.
//!
//! This module provides type-safe wrappers and aliases for commonly used types
//! throughout the codebase to improve type safety and reduce boilerplate.

use std::time::Duration;

/// Frame ID for tracking individual frames in the sliding window.
///
/// Using a newtype instead of raw `usize` provides type safety and prevents
/// accidentally mixing frame IDs with other indices.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct FrameId(pub usize);

impl FrameId {
    /// Create a new FrameId.
    pub fn new(id: usize) -> Self {
        Self(id)
    }

    /// Get the raw ID value.
    pub fn value(&self) -> usize {
        self.0
    }

    /// Get the next sequential frame ID.
    pub fn next(&self) -> Self {
        Self(self.0 + 1)
    }

    /// Get the previous frame ID, if any.
    pub fn prev(&self) -> Option<Self> {
        self.0.checked_sub(1).map(Self)
    }
}

impl From<usize> for FrameId {
    fn from(id: usize) -> Self {
        Self(id)
    }
}

impl From<FrameId> for usize {
    fn from(id: FrameId) -> Self {
        id.0
    }
}

impl std::fmt::Display for FrameId {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Frame({})", self.0)
    }
}

/// Feature ID for tracking individual features across frames.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct FeatureId(pub usize);

impl FeatureId {
    pub fn new(id: usize) -> Self {
        Self(id)
    }

    pub fn value(&self) -> usize {
        self.0
    }
}

impl From<usize> for FeatureId {
    fn from(id: usize) -> Self {
        Self(id)
    }
}

impl From<FeatureId> for usize {
    fn from(id: FeatureId) -> Self {
        id.0
    }
}

impl std::fmt::Display for FeatureId {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Feature({})", self.0)
    }
}

/// Camera ID for multi-camera systems.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct CameraId(pub usize);

impl CameraId {
    pub fn new(id: usize) -> Self {
        Self(id)
    }

    pub fn value(&self) -> usize {
        self.0
    }

    /// Left camera (camera 0).
    pub const LEFT: Self = Self(0);

    /// Right camera (camera 1).
    pub const RIGHT: Self = Self(1);
}

impl From<usize> for CameraId {
    fn from(id: usize) -> Self {
        Self(id)
    }
}

impl From<CameraId> for usize {
    fn from(id: CameraId) -> Self {
        id.0
    }
}

impl std::fmt::Display for CameraId {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Camera({})", self.0)
    }
}

/// Timestamp with nanosecond precision.
///
/// Wraps a Duration to provide semantic meaning and helper methods.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct Timestamp(Duration);

impl Timestamp {
    /// Create a new timestamp from seconds.
    pub fn from_secs(secs: f64) -> Self {
        Self(Duration::from_secs_f64(secs))
    }

    /// Create a new timestamp from nanoseconds.
    pub fn from_nanos(nanos: u64) -> Self {
        Self(Duration::from_nanos(nanos))
    }

    /// Get the timestamp as seconds.
    pub fn as_secs(&self) -> f64 {
        self.0.as_secs_f64()
    }

    /// Get the timestamp as nanoseconds.
    pub fn as_nanos(&self) -> u128 {
        self.0.as_nanos()
    }

    /// Get the underlying Duration.
    pub fn as_duration(&self) -> Duration {
        self.0
    }

    /// Calculate the time difference between two timestamps.
    pub fn duration_since(&self, other: &Timestamp) -> Duration {
        self.0.saturating_sub(other.0)
    }

    /// Add a duration to this timestamp.
    pub fn add(&self, duration: Duration) -> Self {
        Self(self.0 + duration)
    }
}

impl From<Duration> for Timestamp {
    fn from(duration: Duration) -> Self {
        Self(duration)
    }
}

impl From<Timestamp> for Duration {
    fn from(ts: Timestamp) -> Self {
        ts.0
    }
}

impl std::fmt::Display for Timestamp {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{:.6}s", self.as_secs())
    }
}

/// A confidence score in the range [0.0, 1.0].
///
/// This newtype ensures that confidence values are always valid
/// and provides semantic meaning to floating-point scores.
#[derive(Clone, Copy, Debug, PartialEq, PartialOrd)]
pub struct Confidence(f64);

impl Confidence {
    /// Minimum confidence value (0.0).
    pub const MIN: Self = Self(0.0);

    /// Maximum confidence value (1.0).
    pub const MAX: Self = Self(1.0);

    /// Medium confidence (0.5).
    pub const MEDIUM: Self = Self(0.5);

    /// Create a new confidence score, clamping to [0.0, 1.0].
    pub fn new(value: f64) -> Self {
        Self(value.clamp(0.0, 1.0))
    }

    /// Create a confidence score without validation.
    ///
    /// # Safety
    ///
    /// The caller must ensure the value is in [0.0, 1.0].
    pub fn new_unchecked(value: f64) -> Self {
        debug_assert!(
            (0.0..=1.0).contains(&value),
            "Confidence out of range: {}",
            value
        );
        Self(value)
    }

    /// Get the raw confidence value.
    pub fn value(&self) -> f64 {
        self.0
    }

    /// Check if confidence is high (>= 0.75).
    pub fn is_high(&self) -> bool {
        self.0 >= 0.75
    }

    /// Check if confidence is medium (>= 0.5).
    pub fn is_medium(&self) -> bool {
        self.0 >= 0.5
    }

    /// Check if confidence is low (< 0.5).
    pub fn is_low(&self) -> bool {
        self.0 < 0.5
    }
}

impl From<f64> for Confidence {
    fn from(value: f64) -> Self {
        Self::new(value)
    }
}

impl From<Confidence> for f64 {
    fn from(c: Confidence) -> Self {
        c.0
    }
}

impl std::fmt::Display for Confidence {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{:.2}%", self.0 * 100.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_frame_id() {
        let frame = FrameId::new(42);
        assert_eq!(frame.value(), 42);
        assert_eq!(frame.next().value(), 43);
        assert_eq!(frame.prev().unwrap().value(), 41);

        let frame0 = FrameId::new(0);
        assert_eq!(frame0.prev(), None);
    }

    #[test]
    fn test_feature_id() {
        let feat: FeatureId = 123.into();
        assert_eq!(feat.value(), 123);
        assert_eq!(usize::from(feat), 123);
    }

    #[test]
    fn test_camera_id() {
        assert_eq!(CameraId::LEFT.value(), 0);
        assert_eq!(CameraId::RIGHT.value(), 1);
    }

    #[test]
    fn test_timestamp() {
        let ts1 = Timestamp::from_secs(1.5);
        let ts2 = Timestamp::from_secs(2.0);

        assert_eq!(ts1.as_secs(), 1.5);
        assert_eq!(ts2.duration_since(&ts1), Duration::from_secs_f64(0.5));
    }

    #[test]
    fn test_confidence() {
        let c1 = Confidence::new(0.8);
        assert!(c1.is_high());
        assert!(c1.is_medium());
        assert!(!c1.is_low());

        let c2 = Confidence::new(1.5); // Clamped to 1.0
        assert_eq!(c2.value(), 1.0);

        let c3 = Confidence::new(-0.5); // Clamped to 0.0
        assert_eq!(c3.value(), 0.0);
    }

    #[test]
    fn test_confidence_display() {
        let c = Confidence::new(0.756);
        let s = format!("{}", c);
        assert!(s.contains("75.60%"));
    }
}
