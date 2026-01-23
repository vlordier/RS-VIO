//! Drone swarm coordination and multi-drone system support.
//!
//! This module provides types and abstractions for coordinating multiple
//! VIO systems running on different drones in a swarm, including:
//!
//! - **Telemetry**: Health monitoring, performance metrics, and event logging
//! - **Distributed**: Message types, consensus mechanisms, and network awareness
//! - **Types**: Fundamental swarm concepts (DroneId, versioning, partitioning)
//!
//! # Example: Monitoring Swarm Health
//!
//! ```rust,no_run
//! use rs_vio::swarm::{DroneId, HealthStatus, SwarmState};
//! use rs_vio::common::Timestamp;
//!
//! let this_drone = DroneId::new(1);
//! let mut swarm = SwarmState::new(this_drone);
//!
//! // Add known drones
//! swarm.add_drone(DroneId::new(2));
//! swarm.add_drone(DroneId::new(3));
//!
//! // Get health status
//! let ts = Timestamp::from_secs(0.0);
//! let health = HealthStatus::new(ts.as_secs());
//! println!("Swarm health: {}", health.state);
//! ```

pub mod distributed;
pub mod telemetry;
pub mod types;

// Re-export commonly used types
pub use distributed::{
    ConsensusStats, LoopClosureConsensus, LoopClosureVote, MessagePayload, SwarmMessage,
    SwarmState, TraceContext,
};
pub use telemetry::{ComponentHealth, HealthState, HealthStatus, TelemetryFrame, VIOEvent};
pub use types::MessageVersion;
pub use types::{DroneId, NetworkPartitionState};
