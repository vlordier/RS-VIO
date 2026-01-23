//! Common types for drone swarm operations.
//!
//! This module provides the foundational types for multi-drone coordination,
//! including drone identity, timestamps, and basic message structures.

use serde::{Deserialize, Serialize};
use std::fmt;

/// Unique identifier for a drone in the swarm.
///
/// Provides type-safe drone identification to prevent mixing drone IDs
/// with other numeric identifiers like frame IDs or feature IDs.
///
/// # Examples
///
/// ```
/// use rs_vio::swarm::DroneId;
///
/// let drone = DroneId::new(1);
/// assert_eq!(drone.value(), 1);
/// assert!(!drone.is_broadcast());
///
/// let broadcast = DroneId::broadcast();
/// assert!(broadcast.is_broadcast());
/// ```
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
pub struct DroneId(u32);

impl DroneId {
    /// Create a new drone ID.
    pub fn new(id: u32) -> Self {
        Self(id)
    }

    /// Get the numeric ID value.
    pub fn value(&self) -> u32 {
        self.0
    }

    /// Broadcast ID that reaches all drones in the swarm.
    pub fn broadcast() -> Self {
        Self(u32::MAX)
    }

    /// Check if this is a broadcast ID.
    pub fn is_broadcast(&self) -> bool {
        self.0 == u32::MAX
    }

    /// Check if this is a valid unicast ID.
    pub fn is_unicast(&self) -> bool {
        !self.is_broadcast()
    }
}

impl From<u32> for DroneId {
    fn from(id: u32) -> Self {
        Self(id)
    }
}

impl From<DroneId> for u32 {
    fn from(id: DroneId) -> Self {
        id.0
    }
}

impl fmt::Display for DroneId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.is_broadcast() {
            write!(f, "Drone(broadcast)")
        } else {
            write!(f, "Drone({})", self.0)
        }
    }
}

/// Message version for compatibility checking across drones.
///
/// Ensures that drones running different software versions can detect
/// incompatibilities and handle them gracefully.
///
/// # Compatibility Rules
///
/// - Patch-level changes (z in x.y.z) are always compatible
/// - Minor version changes are backward compatible (older >= newer)
/// - Major version changes require exact match
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct MessageVersion {
    /// Major version (breaking changes)
    pub major: u16,
    /// Minor version (backward-compatible features)
    pub minor: u16,
    /// Patch version (bug fixes)
    pub patch: u16,
}

impl MessageVersion {
    /// Create a new message version.
    pub fn new(major: u16, minor: u16, patch: u16) -> Self {
        Self {
            major,
            minor,
            patch,
        }
    }

    /// Current protocol version (should match package version)
    pub const CURRENT: Self = Self {
        major: 0,
        minor: 2,
        patch: 0,
    };

    /// Check if another version is compatible with this one.
    ///
    /// Compatibility is bidirectional:
    /// - `v1.is_compatible(&v2)` == `v2.is_compatible(&v1)`
    pub fn is_compatible(&self, other: &MessageVersion) -> bool {
        // Major version must match (breaking changes)
        if self.major != other.major {
            return false;
        }

        // Minor versions must be compatible (both must support the lower version)
        // This allows v0.2.0 to talk to v0.3.0 by using v0.2.0 features
        let min_minor = self.minor.min(other.minor);
        let max_minor = self.minor.max(other.minor);

        // If versions differ in minor, check if both can fall back
        // In practice, this means both must exist at the minimum version
        min_minor <= max_minor && (min_minor == max_minor || max_minor - min_minor <= 1)
    }

    /// Get a human-readable version string.
    pub fn to_string_verbose(&self) -> String {
        format!("{}.{}.{}", self.major, self.minor, self.patch)
    }
}

impl fmt::Display for MessageVersion {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}.{}.{}", self.major, self.minor, self.patch)
    }
}

impl PartialOrd for MessageVersion {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for MessageVersion {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.major
            .cmp(&other.major)
            .then(self.minor.cmp(&other.minor))
            .then(self.patch.cmp(&other.patch))
    }
}

/// Network partition state for a drone.
///
/// Tracks which drones are reachable and which ones are partitioned away.
/// This is essential for handling network splits gracefully.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum NetworkPartitionState {
    /// Connected to all known drones
    Connected,
    /// Lost connection to some drones
    PartitionedFrom(Vec<DroneId>),
    /// Isolated from swarm
    Isolated,
}

impl NetworkPartitionState {
    /// Check if connected to a specific drone.
    pub fn is_connected_to(&self, drone: DroneId) -> bool {
        match self {
            Self::Connected => true,
            Self::PartitionedFrom(partitioned) => !partitioned.contains(&drone),
            Self::Isolated => false,
        }
    }

    /// Get list of disconnected drones, if any.
    pub fn disconnected_drones(&self) -> Vec<DroneId> {
        match self {
            Self::Connected => Vec::new(),
            Self::PartitionedFrom(drones) => drones.clone(),
            Self::Isolated => Vec::new(),
        }
    }
}

impl fmt::Display for NetworkPartitionState {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Connected => write!(f, "Connected"),
            Self::PartitionedFrom(drones) => {
                write!(f, "PartitionedFrom(")?;
                for (i, drone) in drones.iter().enumerate() {
                    if i > 0 {
                        write!(f, ", ")?;
                    }
                    write!(f, "{}", drone.value())?;
                }
                write!(f, ")")
            },
            Self::Isolated => write!(f, "Isolated"),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_drone_id_creation() {
        let drone = DroneId::new(42);
        assert_eq!(drone.value(), 42);
        assert!(!drone.is_broadcast());
        assert!(drone.is_unicast());
    }

    #[test]
    fn test_broadcast_drone_id() {
        let broadcast = DroneId::broadcast();
        assert!(broadcast.is_broadcast());
        assert!(!broadcast.is_unicast());
    }

    #[test]
    fn test_drone_id_from_u32() {
        let drone: DroneId = 5u32.into();
        assert_eq!(drone.value(), 5);
    }

    #[test]
    fn test_message_version_compatibility() {
        let v1 = MessageVersion::new(0, 2, 0);
        let v2 = MessageVersion::new(0, 2, 1);
        let v3 = MessageVersion::new(0, 3, 0);
        let v4 = MessageVersion::new(1, 0, 0);

        // Patch changes are compatible
        assert!(v1.is_compatible(&v2));
        assert!(v2.is_compatible(&v1));

        // Minor version changes are compatible (within 1)
        assert!(v1.is_compatible(&v3));
        assert!(v3.is_compatible(&v1));

        // Major version changes are incompatible
        assert!(!v1.is_compatible(&v4));
        assert!(!v4.is_compatible(&v1));
    }

    #[test]
    fn test_message_version_current() {
        assert_eq!(MessageVersion::CURRENT.major, 0);
        assert_eq!(MessageVersion::CURRENT.minor, 2);
    }

    #[test]
    fn test_network_partition_state() {
        let drone1 = DroneId::new(1);
        let drone2 = DroneId::new(2);

        let connected = NetworkPartitionState::Connected;
        assert!(connected.is_connected_to(drone1));
        assert!(connected.is_connected_to(drone2));

        let partitioned = NetworkPartitionState::PartitionedFrom(vec![drone2]);
        assert!(partitioned.is_connected_to(drone1));
        assert!(!partitioned.is_connected_to(drone2));

        let isolated = NetworkPartitionState::Isolated;
        assert!(!isolated.is_connected_to(drone1));
        assert!(!isolated.is_connected_to(drone2));
    }
}
