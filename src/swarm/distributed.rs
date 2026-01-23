//! Distributed system message types for swarm communication.
//!
//! Provides type-safe message envelopes, consensus structures, and synchronization
//! primitives for multi-drone coordination.

use crate::common::Timestamp as TimestampType;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

use super::types::{DroneId, MessageVersion, NetworkPartitionState};

/// Trait context for distributed tracing.
///
/// Enables propagation of trace IDs across drone boundaries for debugging
/// and performance analysis of distributed operations.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct TraceContext {
    /// Unique trace ID (UUID-like)
    pub trace_id: String,
    /// Parent span ID if nested
    pub parent_span_id: Option<String>,
    /// Current span ID
    pub span_id: String,
    /// Baggage items for context propagation
    pub baggage: HashMap<String, String>,
}

impl TraceContext {
    /// Create a new root trace context.
    pub fn root(trace_id: String, span_id: String) -> Self {
        Self {
            trace_id,
            parent_span_id: None,
            span_id,
            baggage: HashMap::new(),
        }
    }

    /// Create a child span within this trace.
    pub fn child_span(&self, child_span_id: String) -> Self {
        Self {
            trace_id: self.trace_id.clone(),
            parent_span_id: Some(self.span_id.clone()),
            span_id: child_span_id,
            baggage: self.baggage.clone(),
        }
    }

    /// Add baggage item for context propagation.
    pub fn with_baggage(mut self, key: String, value: String) -> Self {
        self.baggage.insert(key, value);
        self
    }
}

/// Message payload types for swarm communication.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum MessagePayload {
    /// Localization update from one drone to others
    LocalizationUpdate {
        /// Current pose position [x, y, z]
        position: [f64; 3],
        /// Quaternion [w, x, y, z]
        orientation: [f64; 4],
        /// Covariance diagonal (6D)
        covariance_diagonal: [f64; 6],
        /// ID of the keyframe that generated this pose
        keyframe_id: u32,
        /// Estimated drift rate (m/s)
        drift_rate: Option<f64>,
    },

    /// Loop closure candidate for consensus
    LoopClosureCandidate {
        /// Frame ID in source drone's history
        source_frame_id: u32,
        /// Frame ID in target drone's history
        target_frame_id: u32,
        /// Estimated transform position [x, y, z]
        position: [f64; 3],
        /// Estimated transform quaternion [w, x, y, z]
        orientation: [f64; 4],
        /// Confidence score [0, 1]
        confidence: f64,
    },

    /// Request to merge pose graphs
    MapMergingRequest {
        /// Serialized pose graph from requester
        graph_data: Vec<u8>,
        /// Frame IDs used for alignment
        alignment_frames: Vec<u32>,
        /// Proposed alignment position [x, y, z]
        proposed_position: Option<[f64; 3]>,
        /// Proposed alignment quaternion [w, x, y, z]
        proposed_orientation: Option<[f64; 4]>,
    },

    /// Response to map merging request
    MapMergingResponse {
        /// Whether merge was accepted
        accepted: bool,
        /// Computed alignment position if accepted [x, y, z]
        alignment_position: Option<[f64; 3]>,
        /// Computed alignment quaternion if accepted [w, x, y, z]
        alignment_orientation: Option<[f64; 4]>,
        /// Error reason if rejected
        error: Option<String>,
    },

    /// Heartbeat for liveness detection
    Heartbeat {
        /// Health status at source
        health_ok: bool,
        /// Memory usage in MB
        memory_usage_mb: u32,
        /// Processing latency (ms)
        latency_ms: f64,
    },

    /// Request for pose at specific timestamp
    PoseQuery {
        /// Requested timestamp (seconds since epoch)
        timestamp_secs: f64,
        /// Expected response format
        format: u8,
    },

    /// Response with requested pose
    PoseQueryResponse {
        /// Requested timestamp (seconds since epoch)
        timestamp_secs: f64,
        /// Pose position if available [x, y, z]
        position: Option<[f64; 3]>,
        /// Pose quaternion if available [w, x, y, z]
        orientation: Option<[f64; 4]>,
        /// Confidence in pose [0, 1]
        confidence: f64,
    },
}

impl MessagePayload {
    /// Get human-readable payload type.
    pub fn payload_type(&self) -> &'static str {
        match self {
            Self::LocalizationUpdate { .. } => "LocalizationUpdate",
            Self::LoopClosureCandidate { .. } => "LoopClosureCandidate",
            Self::MapMergingRequest { .. } => "MapMergingRequest",
            Self::MapMergingResponse { .. } => "MapMergingResponse",
            Self::Heartbeat { .. } => "Heartbeat",
            Self::PoseQuery { .. } => "PoseQuery",
            Self::PoseQueryResponse { .. } => "PoseQueryResponse",
        }
    }
}

/// Complete message envelope for swarm communication.
///
/// Wraps payload with necessary metadata for routing, versioning, and tracing.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SwarmMessage {
    /// Source drone ID
    pub sender: DroneId,
    /// Message sequence number (for deduplication)
    pub sequence_number: u64,
    /// Timestamp when message was created (seconds since epoch)
    pub timestamp_secs: f64,
    /// Protocol version (for compatibility checking)
    pub version: MessageVersion,
    /// Actual message content
    pub payload: MessagePayload,
    /// Distributed trace context
    pub trace_context: Option<TraceContext>,
}

impl SwarmMessage {
    /// Create a new swarm message.
    pub fn new(
        sender: DroneId,
        sequence_number: u64,
        timestamp: TimestampType,
        payload: MessagePayload,
    ) -> Self {
        Self {
            sender,
            sequence_number,
            timestamp_secs: timestamp.as_secs(),
            version: MessageVersion::CURRENT,
            payload,
            trace_context: None,
        }
    }

    /// Add trace context to message.
    pub fn with_trace(mut self, context: TraceContext) -> Self {
        self.trace_context = Some(context);
        self
    }

    /// Check if message version is compatible with current version.
    pub fn is_version_compatible(&self) -> bool {
        self.version.is_compatible(&MessageVersion::CURRENT)
    }

    /// Get size estimate for network transmission (bytes).
    pub fn estimated_size_bytes(&self) -> usize {
        // Rough estimate: header ~50 bytes + payload ~100-3000 bytes
        // All fixed-size payloads: position (24B) + quaternion (32B) + overhead
        match &self.payload {
            MessagePayload::LocalizationUpdate { .. } => 180, // 24 + 32 + 48 + overhead
            MessagePayload::LoopClosureCandidate { .. } => 120, // 24 + 32 + overhead
            MessagePayload::MapMergingRequest { graph_data, .. } => 100 + graph_data.len(),
            MessagePayload::MapMergingResponse { .. } => 100, // conditional 56B
            MessagePayload::Heartbeat { .. } => 50,
            MessagePayload::PoseQuery { .. } => 40,
            MessagePayload::PoseQueryResponse { .. } => 100, // conditional 56B
        }
    }
}

/// Consensus decision on a loop closure candidate.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct LoopClosureVote {
    /// Drone voting
    pub voter_id: DroneId,
    /// Loop closure candidate
    pub candidate_frame_id: u32,
    /// Confidence in closure [0, 1]
    pub confidence: f64,
    /// When vote was cast (seconds since epoch)
    pub timestamp_secs: f64,
}

impl LoopClosureVote {
    /// Create a new vote.
    pub fn new(
        voter_id: DroneId,
        candidate_frame_id: u32,
        confidence: f64,
        timestamp: TimestampType,
    ) -> Self {
        Self {
            voter_id,
            candidate_frame_id,
            confidence: confidence.clamp(0.0, 1.0),
            timestamp_secs: timestamp.as_secs(),
        }
    }

    /// Check if this vote is affirmative (above threshold).
    pub fn is_affirmative(&self, threshold: f64) -> bool {
        self.confidence >= threshold
    }
}

/// Consensus state for loop closure acceptance.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct LoopClosureConsensus {
    /// Votes received so far
    pub votes: Vec<LoopClosureVote>,
    /// Minimum confidence threshold [0, 1]
    pub confidence_threshold: f64,
    /// Minimum approval ratio [0, 1] (e.g., 0.8 = 80%)
    pub approval_threshold: f64,
}

impl LoopClosureConsensus {
    /// Create a new consensus tracker.
    pub fn new(confidence_threshold: f64, approval_threshold: f64) -> Self {
        Self {
            votes: Vec::new(),
            confidence_threshold: confidence_threshold.clamp(0.0, 1.0),
            approval_threshold: approval_threshold.clamp(0.0, 1.0),
        }
    }

    /// Add a vote.
    pub fn add_vote(&mut self, vote: LoopClosureVote) {
        self.votes.push(vote);
    }

    /// Check if consensus has been reached to accept the closure.
    pub fn is_accepted(&self) -> bool {
        if self.votes.is_empty() {
            return false;
        }

        let affirmative = self
            .votes
            .iter()
            .filter(|v| v.is_affirmative(self.confidence_threshold))
            .count() as f64;

        let approval_rate = affirmative / self.votes.len() as f64;
        approval_rate >= self.approval_threshold
    }

    /// Get consensus statistics.
    pub fn stats(&self) -> ConsensusStats {
        if self.votes.is_empty() {
            return ConsensusStats {
                total_votes: 0,
                affirmative_votes: 0,
                approval_rate: 0.0,
                avg_confidence: 0.0,
            };
        }

        let affirmative = self
            .votes
            .iter()
            .filter(|v| v.is_affirmative(self.confidence_threshold))
            .count();

        let avg_confidence =
            self.votes.iter().map(|v| v.confidence).sum::<f64>() / self.votes.len() as f64;

        ConsensusStats {
            total_votes: self.votes.len(),
            affirmative_votes: affirmative,
            approval_rate: affirmative as f64 / self.votes.len() as f64,
            avg_confidence,
        }
    }
}

/// Statistics about consensus voting.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ConsensusStats {
    /// Total votes received
    pub total_votes: usize,
    /// Number of affirmative votes
    pub affirmative_votes: usize,
    /// Fraction of votes that were affirmative
    pub approval_rate: f64,
    /// Average confidence across all votes
    pub avg_confidence: f64,
}

/// Swarm state tracking for a drone.
#[derive(Clone, Debug)]
pub struct SwarmState {
    /// This drone's ID
    pub this_drone: DroneId,
    /// Known drones in swarm
    pub known_drones: Vec<DroneId>,
    /// Network connectivity status
    pub partition_state: NetworkPartitionState,
    /// Last heartbeat time from each drone (seconds since epoch)
    pub last_heartbeat: HashMap<DroneId, f64>,
    /// Heartbeat timeout threshold (seconds)
    pub heartbeat_timeout_secs: f64,
}

impl SwarmState {
    /// Create a new swarm state.
    pub fn new(this_drone: DroneId) -> Self {
        Self {
            this_drone,
            known_drones: Vec::new(),
            partition_state: NetworkPartitionState::Isolated,
            last_heartbeat: HashMap::new(),
            heartbeat_timeout_secs: 5.0,
        }
    }

    /// Add a known drone.
    pub fn add_drone(&mut self, drone: DroneId) {
        if !self.known_drones.contains(&drone) {
            self.known_drones.push(drone);
        }
    }

    /// Check if connected to a specific drone.
    pub fn is_connected_to(&self, drone: DroneId, now: TimestampType) -> bool {
        if let Some(last_hb_secs) = self.last_heartbeat.get(&drone) {
            let elapsed = (now.as_secs() - last_hb_secs).max(0.0);
            elapsed < self.heartbeat_timeout_secs
        } else {
            false
        }
    }

    /// Update heartbeat from a drone.
    pub fn update_heartbeat(&mut self, drone: DroneId, timestamp: TimestampType) {
        self.last_heartbeat.insert(drone, timestamp.as_secs());
    }

    /// Recompute partition state based on heartbeats.
    pub fn recompute_partition_state(&mut self, now: TimestampType) {
        let connected: Vec<DroneId> = self
            .known_drones
            .iter()
            .filter(|d| self.is_connected_to(**d, now))
            .copied()
            .collect();

        if connected.len() == self.known_drones.len() {
            self.partition_state = NetworkPartitionState::Connected;
        } else {
            let disconnected: Vec<DroneId> = self
                .known_drones
                .iter()
                .filter(|d| !connected.contains(d))
                .copied()
                .collect();
            self.partition_state = NetworkPartitionState::PartitionedFrom(disconnected);
        }
    }

    /// Get list of unreachable drones.
    pub fn unreachable_drones(&self) -> Vec<DroneId> {
        self.partition_state.disconnected_drones()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_trace_context_creation() {
        let ctx = TraceContext::root("trace-123".to_string(), "span-1".to_string());
        assert_eq!(ctx.trace_id, "trace-123");
        assert_eq!(ctx.span_id, "span-1");
        assert!(ctx.parent_span_id.is_none());
    }

    #[test]
    fn test_trace_context_child() {
        let root = TraceContext::root("trace-123".to_string(), "span-1".to_string());
        let child = root.child_span("span-2".to_string());
        assert_eq!(child.trace_id, "trace-123");
        assert_eq!(child.parent_span_id, Some("span-1".to_string()));
        assert_eq!(child.span_id, "span-2");
    }

    #[test]
    fn test_swarm_message_creation() {
        let ts = TimestampType::from_secs(1.0);
        let sender = DroneId::new(1);
        let payload = MessagePayload::Heartbeat {
            health_ok: true,
            memory_usage_mb: 245,
            latency_ms: 6.8,
        };

        let msg = SwarmMessage::new(sender, 0, ts, payload);
        assert_eq!(msg.sender, sender);
        assert_eq!(msg.sequence_number, 0);
        assert!(msg.is_version_compatible());
    }

    #[test]
    fn test_loop_closure_vote() {
        let ts = TimestampType::from_secs(0.0);
        let vote = LoopClosureVote::new(DroneId::new(1), 42, 0.95, ts);
        assert_eq!(vote.candidate_frame_id, 42);
        assert!(vote.is_affirmative(0.8));
        assert!(!vote.is_affirmative(0.96));
    }

    #[test]
    fn test_loop_closure_consensus() {
        let ts = TimestampType::from_secs(0.0);
        let mut consensus = LoopClosureConsensus::new(0.7, 0.8);

        // Add 4 votes: 3 affirmative (confidence >= 0.7), 1 negative
        consensus.add_vote(LoopClosureVote::new(DroneId::new(1), 42, 0.9, ts));
        consensus.add_vote(LoopClosureVote::new(DroneId::new(2), 42, 0.85, ts));
        consensus.add_vote(LoopClosureVote::new(DroneId::new(3), 42, 0.75, ts));
        consensus.add_vote(LoopClosureVote::new(DroneId::new(4), 42, 0.5, ts));

        // 3 affirmative out of 4 = 75% < 80%, so should not be accepted
        assert!(!consensus.is_accepted());

        // Add one more affirmative vote
        consensus.add_vote(LoopClosureVote::new(DroneId::new(5), 42, 0.8, ts));

        // 4 affirmative out of 5 = 80%, now should be accepted
        assert!(consensus.is_accepted());

        let stats = consensus.stats();
        assert_eq!(stats.total_votes, 5);
        assert_eq!(stats.affirmative_votes, 4);
        assert!(stats.approval_rate >= 0.8);
    }

    #[test]
    fn test_swarm_state_creation() {
        let drone = DroneId::new(1);
        let state = SwarmState::new(drone);
        assert_eq!(state.this_drone, drone);
        assert_eq!(state.known_drones.len(), 0);
    }

    #[test]
    fn test_swarm_state_add_drone() {
        let drone = DroneId::new(1);
        let mut state = SwarmState::new(drone);
        state.add_drone(DroneId::new(2));
        state.add_drone(DroneId::new(3));

        assert_eq!(state.known_drones.len(), 2);
    }
}
