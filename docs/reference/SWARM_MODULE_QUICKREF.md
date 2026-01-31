# Swarm Module Quick Reference

## Quick Start

The swarm module provides enterprise-grade infrastructure for multi-drone VIO coordination. Enable it with:

```bash
cargo build --features swarm
```

## Common Usage Patterns

### 1. Create a Drone Identity

```rust
use rs_vio::swarm::DroneId;

let drone = DroneId::new(1);
let broadcast = DroneId::broadcast();
```

### 2. Send a Localization Update

```rust
use rs_vio::swarm::{SwarmMessage, MessagePayload, DroneId};
use rs_vio::common::Timestamp;

let pose_update = MessagePayload::LocalizationUpdate {
    position: [1.0, 2.0, 3.0],
    orientation: [1.0, 0.0, 0.0, 0.0], // [w, x, y, z]
    covariance_diagonal: [0.01; 6],
    keyframe_id: 42,
    drift_rate: Some(0.05),
};

let msg = SwarmMessage::new(
    DroneId::new(1),
    sequence_number,
    Timestamp::from_secs(now),
    pose_update,
);
```

### 3. Monitor System Health

```rust
use rs_vio::swarm::{HealthStatus, HealthState};

let health = HealthStatus::new(now_secs);

if health.state == HealthState::Unhealthy {
    // Take corrective action
}

let reliability = health.compute_reliability();
```

### 4. Consensus Loop Closures

```rust
use rs_vio::swarm::{LoopClosureConsensus, LoopClosureVote, DroneId};

let mut consensus = LoopClosureConsensus::new(
    0.7,  // confidence_threshold
    0.8,  // approval_threshold (80% of votes must agree)
);

consensus.add_vote(LoopClosureVote::new(
    DroneId::new(1),
    frame_id,
    confidence,
    timestamp,
));

if consensus.is_accepted() {
    // Accept the loop closure
}
```

### 5. Track Network Connectivity

```rust
use rs_vio::swarm::{SwarmState, DroneId};

let mut state = SwarmState::new(DroneId::new(1));
state.add_drone(DroneId::new(2));
state.add_drone(DroneId::new(3));

state.update_heartbeat(DroneId::new(2), timestamp);
state.recompute_partition_state(now);

let unreachable = state.unreachable_drones();
```

## Type Reference

### Core Types
- **DroneId**: Unique drone identifier with broadcast support
- **MessageVersion**: Semantic versioning for protocol compatibility
- **NetworkPartitionState**: Network connectivity state tracking

### Telemetry Types
- **HealthStatus**: Overall system health (4 component health monitors)
- **HealthState**: Health enum (Healthy | Degraded | Unhealthy)
- **ComponentHealth**: Per-component metrics
- **TelemetryFrame**: Per-frame performance metrics
- **VIOEvent**: Event types for monitoring (8 variants)

### Distributed Types
- **SwarmMessage**: Message envelope with sender, version, payload
- **MessagePayload**: 7 message types (Localization, LoopClosure, MapMerging, Heartbeat, PoseQuery)
- **TraceContext**: Distributed tracing context for debugging
- **LoopClosureConsensus**: Threshold-based voting for loop closure acceptance
- **SwarmState**: Swarm connectivity and heartbeat tracking

### Error Handling
- **ErrorCategory**: Transient | Permanent | Degraded
- **VIOError variants**: VersionIncompatible, NetworkPartition, ConsensusFailed (plus 3 categorized variants)
- **VIOError methods**: category(), is_retryable(), is_permanent(), is_degradable()

## Feature Flags

```toml
[features]
swarm = ["distributed", "full-telemetry"]  # All swarm features
distributed = []                            # Distributed system types only
full-telemetry = []                         # Telemetry collection
health-checks = []                          # Health monitoring
embedded = []                               # Embedded platform optimizations
jetson-optimized = []                       # Jetson-specific optimizations
```

## Serialization

All types support serde:
```rust
use serde_json;

let msg = SwarmMessage::new(...);
let json = serde_json::to_string(&msg)?;
let deserialized: SwarmMessage = serde_json::from_str(&json)?;
```

## Testing

Run the full swarm test suite:
```bash
cargo test --lib swarm::
```

Run a specific test:
```bash
cargo test --lib swarm::distributed::tests::test_loop_closure_consensus
```

## Integration (Phase 2)

Once Estimator integration is added, you'll be able to:

```rust
// Not yet implemented
let health = estimator.health();
let telemetry = estimator.telemetry();
```

## Error Handling Pattern

```rust
use rs_vio::VIOError;

match vio_error {
    e if e.is_retryable() => {
        // Safe to retry
    },
    e if e.is_permanent() => {
        // Should not retry
    },
    e if e.is_degradable() => {
        // Continue with reduced functionality
    },
    _ => {
        // Unknown error
    }
}
```

## Performance Tips

1. **Message Size**: Use `estimated_size_bytes()` to estimate transmission overhead
2. **Consensus Thresholds**: Set approval_threshold based on swarm size for faster convergence
3. **Heartbeat Timeout**: Adjust heartbeat_timeout_secs based on network latency
4. **Trace Context**: Only create trace contexts for high-priority operations

## Documentation

Full documentation:
```bash
cargo doc --lib --open
```

Navigate to `rs_vio::swarm` to see all types and methods with examples.
