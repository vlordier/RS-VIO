# RS-VIO Remaining Work Roadmap
## Visual-Inertial Odometry → SLAM → Multi-Agent Swarm System

**Last Updated:** 2026-01-24  
**Status:** Actively Developed  
**Current Phase:** Integration & Hardening

---

## Executive Summary

RS-VIO is a comprehensive Visual-Inertial Odometry (VIO) system that can be extended into a full SLAM system with multi-drone swarm coordination. Here's what's **DONE**, **IN PROGRESS**, and **TODO**.

---

## Current Architecture Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                    RS-VIO COMPLETE STACK                        │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  LAYER 1: SINGLE-AGENT VIO (✅ COMPLETE)                        │
│  ├─ Feature Detection & Tracking (stereo + optical flow)        │
│  ├─ IMU Fusion & Preintegration                                 │
│  ├─ Sliding Window Bundle Adjustment                            │
│  ├─ Loop Closure Detection                                      │
│  └─ Visualization (Rerun integration)                           │
│                                                                  │
│  LAYER 2: SLAM FOUNDATIONS (🔄 IN PROGRESS)                     │
│  ├─ Global Pose Graph Construction                              │
│  ├─ Loop Closure Constraints                                    │
│  ├─ Dense Map Building                                          │
│  ├─ Map Merging Infrastructure                                  │
│  └─ Online Calibration                                          │
│                                                                  │
│  LAYER 3: MULTI-AGENT SWARM (⏳ PLANNED)                         │
│  ├─ Distributed Optimization                                    │
│  ├─ Map Merging (already have modules)                          │
│  ├─ Boids Coordination (already have modules)                   │
│  ├─ Exploration Planning                                        │
│  └─ Communication Protocol                                      │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

---

## LAYER 1: Single-Agent VIO (✅ COMPLETE)

### Current Status: 782/782 Tests Passing

**What's Done:**

✅ **Feature Detection & Tracking**
- Stereo feature detection with FAST corners
- Optical flow tracking (LK algorithm)
- Subpixel refinement (92% retention rate)
- Rolling shutter compensation
- Async feature detection

✅ **IMU Fusion**
- Gyroscope integration (rotation estimation)
- Accelerometer integration (translation with gravity)
- Vibration filtering (learned model)
- Motor state detection
- Online bias estimation
- IMU-Camera time synchronization

✅ **Pose Estimation**
- Sliding window bundle adjustment (15-20 frames)
- Levenberg-Marquardt optimization
- SE(3) manifold optimization
- PnP-based motion tracking
- Keyframe selection (visual + IMU triggers)

✅ **Loop Closure**
- ORB descriptor extraction
- Loop closure detection
- Constraint addition (10+ per keyframe)
- Graph optimization

✅ **Calibration**
- Camera intrinsics (online learning)
- Stereo baseline
- IMU intrinsics (accelerometer, gyroscope)
- Camera-IMU extrinsics (T_B_C)
- Time offset
- Rolling shutter parameters

✅ **Multi-Dataset Support**
- EuRoC (MH and V sequences)
- TUM VI (accuracy benchmark)
- 4Seasons (outdoor, illumination change)
- Live camera input

✅ **Visualization**
- Rerun integration for real-time display
- Map point visualization
- Keyframe trajectory
- Feature tracking display
- IMU metrics display
- Loop closure visualization

### Remaining VIO Tasks (Minor Polish)

❓ **Performance Optimization**
- [ ] GPU acceleration for feature detection
- [ ] Real-time dense reconstruction
- [ ] Memory pooling for temporary allocations
- [ ] Vectorization of critical loops

---

## LAYER 2: SLAM Foundations (🔄 IN PROGRESS)

### What's Done:

✅ **Pose Graph Infrastructure**
- Local pose graph per frame (in sliding window)
- Bundle adjustment factors implemented
- Loop closure constraints added automatically

✅ **Dense Reconstruction Module** (exists in src/)
- Structure from motion
- Photometric consistency checking
- Depth map fusion from multiple views

✅ **Map Merging Framework** (exists in src/multi_drone/)
- `MapMerger` structure
- `DroneMap` representation
- `Transform` constraints
- Multi-drone map alignment

✅ **Evaluation Framework**
- Trajectory evaluation (TUM VI benchmark)
- Accuracy metrics (ATE, RPE, scale)
- Pose interpolation
- Ground truth comparison

### Core SLAM Gaps (Must Do)

**[PRIORITY: HIGH]** Global Pose Graph

Current: Local pose graph (sliding window) + loop closure constraints  
Needed: Global pose graph combining all constraints

**What to do:**
- [ ] Extend `SlidingWindow` to build global pose graph
- [ ] Add graph-SLAM marginalization (preserve loop constraints)
- [ ] Implement Schur complement for efficient solving
- [ ] Add loop closure edge types (SE3Constraint, AbsolutePose)
- [ ] Create `GlobalPoseGraph` structure

**Files to modify:**
- `src/estimator/sliding_window.rs` - Add global graph construction
- `src/optimization/` - Add global BA factors
- `src/` - New file: `global_pose_graph.rs`

**Estimated effort:** 1-2 weeks  
**Tests needed:** 5-10 integration tests

---

**[PRIORITY: HIGH]** Loop Closure Optimization

Current: Loop closure detection works, constraints added, but not globally optimized  
Needed: Global optimization when significant loop closure occurs

**What to do:**
- [ ] Detect loop closure "events" (strong constraint clusters)
- [ ] Trigger global optimization routine
- [ ] Re-optimize entire pose graph when loop detected
- [ ] Update all historical poses (not just sliding window)
- [ ] Marginalize old poses properly

**Files to modify:**
- `src/estimator/estimator/processor.rs` - Add loop closure event detection
- `src/optimization/` - Add global optimization solver
- `src/estimator/sliding_window.rs` - Add pose update after global opt

**Estimated effort:** 1 week  
**Tests needed:** Loop closure integration tests

---

**[PRIORITY: MEDIUM]** Map Merging Integration

Current: Map merging code exists as stub  
Needed: Complete map alignment and fusion

**What to do:**
- [ ] Implement point cloud registration (ICP or feature-based)
- [ ] Compute transform between two drone maps
- [ ] Validate map compatibility (overlap detection)
- [ ] Fuse points using uncertainty weighting
- [ ] Handle landmark ID conflicts
- [ ] Update pose graph with merge constraints

**Files to modify:**
- `src/multi_drone/map_merging.rs` - Complete implementation
- `src/optimization/` - Add map merge factors

**Estimated effort:** 2 weeks  
**Tests needed:** Map merging benchmarks

---

**[PRIORITY: MEDIUM]** Dense Reconstruction

Current: Module exists, structure from motion implemented  
Needed: Integration with main pipeline and real-time streaming

**What to do:**
- [ ] Connect dense reconstruction to pose graph updates
- [ ] Stream depth maps to Rerun for visualization
- [ ] Implement octomap integration for occupancy
- [ ] Add mesh extraction (Poisson surface reconstruction)
- [ ] Performance optimization (GPU if needed)

**Files to modify:**
- `src/dense_reconstruction/` - Integration hooks
- `src/estimator/estimator/processor.rs` - Trigger dense reconstruction

**Estimated effort:** 2 weeks  
**Tests needed:** Reconstruction quality metrics

---

## LAYER 3: Multi-Agent Swarm (⏳ PLANNED)

### What's Already Implemented:

✅ **Boids Coordination** (exists in src/multi_drone/boids.rs)
```rust
pub struct Boid {
    position: Vector3,
    velocity: Vector3,
    rotation: Matrix3x3,
    ...
}

pub struct BoidsSwarm {
    boids: Vec<Boid>,
    config: BoidsConfig,
    ...
}
```

- Separation (collision avoidance)
- Alignment (velocity matching)
- Cohesion (swarm centering)
- Goal seeking
- Max speed/acceleration constraints

✅ **Distributed Optimization** (exists in src/multi_drone/distributed_optimization.rs)
```rust
pub struct DistributedOptimizer {
    local_graph: LocalPoseGraph,
    constraints: Vec<Constraint>,
    ...
}
```

- Local pose graphs per drone
- Constraint accumulation
- Asynchronous optimization
- Consensus averaging

✅ **Exploration Planning** (exists in src/multi_drone/exploration.rs)
```rust
pub struct ExplorationManager {
    grid: OccupancyGrid,
    frontiers: Vec<Frontier>,
    ...
}
```

- Frontier detection
- Occupancy grid mapping
- Goal assignment
- Coverage planning

### Swarm Integration Tasks (Must Do)

**[PRIORITY: HIGH]** Inter-Drone Communication

Current: No communication infrastructure  
Needed: Network layer for pose/map exchange

**What to do:**
- [ ] Define communication message types (Protobuf or bincode)
  - PoseUpdate (T_W_B, covariance, timestamp)
  - MapUpdate (points, descriptors, timestamps)
  - LoopClosureAlert (drone_a, drone_b, T_ab)
  - MapMergeRequest (overlap detection)
- [ ] Implement message queue (mpsc channels for sim, network for real)
- [ ] Add communication simulator (network delay, packet loss)
- [ ] ROS2 integration layer (optional)

**Architecture:**
```
Drone 1 (Estimator) → DroneNetwork { send/recv queues } ← Drone 2 (Estimator)
                                          ↓
                              Distributed Optimizer
```

**Files to create:**
- `src/multi_drone/communication.rs` - Message types & protocol
- `src/multi_drone/network_simulator.rs` - Simulation layer
- `src/multi_drone/ros_bridge.rs` - ROS2 integration (optional)

**Estimated effort:** 2 weeks  
**Tests needed:** Network simulator tests, protocol tests

---

**[PRIORITY: HIGH]** Distributed Pose Graph Optimization

Current: Distributed optimizer exists, but integration missing  
Needed: Connected distributed solving across multiple drones

**What to do:**
- [ ] Each drone maintains local pose graph
- [ ] Map merges trigger constraint addition
- [ ] Implement ADMM (Alternating Direction Method of Multipliers)
  - Each drone optimizes locally
  - Consensus step via network communication
  - Iterate until convergence
- [ ] Handle asynchronous updates (drones optimize at different rates)
- [ ] Implement relative pose exchange format
- [ ] Verify convergence properties

**Algorithm Flow:**
```
Drone 1: Optimize local PG → Send relative poses to Drone 2
Drone 2: Receive, optimize local PG → Send back to Drone 1
...repeat until ADMM convergence
```

**Files to modify/create:**
- `src/multi_drone/distributed_optimization.rs` - Implement ADMM
- `src/multi_drone/communication.rs` - Relative pose messages
- New tests for multi-drone optimization

**Estimated effort:** 3 weeks  
**Tests needed:** 2-drone, 3-drone, n-drone convergence tests

---

**[PRIORITY: MEDIUM]** Coordinated Exploration

Current: Exploration manager exists, coordination missing  
Needed: Swarm-level frontier assignment and path planning

**What to do:**
- [ ] Global frontier detection across all drones
- [ ] Frontier value estimation (information gain)
- [ ] Task assignment (Hungarian algorithm or auction)
  - Minimize total distance
  - Avoid collisions
  - Maintain coverage
- [ ] Trajectory planning to assigned frontiers
  - Bezier curve generation
  - Collision checking with other drones
  - IMU constraints (max acceleration)
- [ ] Real-time replanning on new observations

**Files to modify:**
- `src/multi_drone/exploration.rs` - Complete explorer
- `src/multi_drone/boids.rs` - Goal-seeking integration
- New path planning module

**Estimated effort:** 2-3 weeks  
**Tests needed:** Coverage benchmarks, task assignment tests

---

**[PRIORITY: MEDIUM]** Swarm Simulation & Evaluation

Current: No full swarm simulator  
Needed: Benchmark swarm on multi-agent datasets

**What to do:**
- [ ] Create swarm simulator
  - Multiple EKalman filter "drones"
  - Simulated communication
  - Network effects (delay, packet loss)
- [ ] Generate synthetic multi-drone datasets
- [ ] Create evaluation metrics
  - Map accuracy per drone
  - Merged map accuracy
  - Communication overhead
  - Convergence time
- [ ] Benchmark: 2 drone, 4 drone, 8 drone scenarios

**Files to create:**
- `tests/multi_drone_simulator.rs`
- `examples/swarm_benchmark.rs`
- `tests/datasets/multi_drone/`

**Estimated effort:** 2 weeks  
**Tests needed:** Swarm-level integration tests

---

## Implementation Sequence (Recommended)

### Phase 1: SLAM Hardening (4 weeks)
```
Week 1: Global Pose Graph
Week 2: Loop Closure Optimization  
Week 3: Map Merging Integration
Week 4: Testing & debugging
```

**Outcome:** Single-drone SLAM fully functional with global optimization

---

### Phase 2: Swarm Communication (2-3 weeks)
```
Week 1: Message protocol & simulator
Week 2: Distributed optimizer ADMM
Week 3: Integration testing
```

**Outcome:** Two drones can exchange poses and optimize jointly

---

### Phase 3: Coordinated Swarm (3-4 weeks)
```
Week 1: Exploration planning
Week 2: Path planning & collision avoidance
Week 3: Swarm simulator
Week 4: Benchmarking & tuning
```

**Outcome:** Full swarm system with coordinated exploration

---

## Dependency Graph

```
VIO (✅ DONE)
  ↓
SLAM Global Graph (⏳ NEXT)
  ├→ Loop Closure Optimization (⏳)
  └→ Dense Reconstruction (⏳)
       ↓
Communication Infrastructure (⏳ NEXT AFTER SLAM)
  ├→ Distributed Optimization (⏳)
  └→ Map Merging (⏳)
       ↓
Coordinated Exploration (⏳ FINAL)
  ├→ Path Planning (⏳)
  └→ Swarm Control (⏳)
```

---

## Key Files to Modify

### SLAM Phase
- `src/estimator/sliding_window.rs` - Global graph construction
- `src/estimator/estimator/processor.rs` - Loop closure event detection
- `src/optimization/` - Global BA solver
- `src/dense_reconstruction/` - Integration hooks

### Swarm Phase
- `src/multi_drone/communication.rs` - Message protocol
- `src/multi_drone/distributed_optimization.rs` - ADMM solver
- `src/multi_drone/exploration.rs` - Task assignment
- `src/multi_drone/boids.rs` - Swarm control
- `src/multi_drone/map_merging.rs` - Map fusion

### Testing
- `tests/` - Integration test suites
- `examples/` - Demonstration examples
- New benchmarks for each phase

---

## Technical Challenges & Solutions

### Challenge 1: Global Optimization Scalability
**Problem:** Pose graph grows unbounded with time  
**Solution:** Pose graph compression/marginalization after loop closure

### Challenge 2: Asynchronous Map Updates
**Problem:** Drones receive data at different rates  
**Solution:** Timestamped constraints with temporal coherency checking

### Challenge 3: Communication Bandwidth
**Problem:** Sending full point clouds is expensive  
**Solution:** Compress with descriptors, send diff only, use compression

### Challenge 4: Distributed Convergence
**Problem:** ADMM may not converge with asynchronous updates  
**Solution:** Implement asynchronous ADMM variant with convergence guarantees

### Challenge 5: Loop Closure Conflicts
**Problem:** Multiple drones may see same area differently  
**Solution:** Confidence-weighted constraint merging + outlier rejection

---

## Testing Strategy

### Unit Tests (Per Component)
- Pose graph construction
- Loop closure detection
- Map merging algorithm
- ADMM solver convergence

### Integration Tests
- Full SLAM pipeline with synthetic data
- 2-drone coordination with known ground truth
- Network simulator with packet loss
- Exploration planning with dynamic obstacles

### Benchmarks
- Trajectory accuracy (ATE/RPE)
- Computational cost (latency, memory)
- Communication overhead (bytes/second)
- Convergence time (iterations to 1mm accuracy)
- Exploration coverage (% mapped area)

### Real-World Validation
- Actual drone hardware (when available)
- Multi-drone outdoor experiments
- High-dynamics scenarios
- GPS-denied environments

---

## Milestone Checkpoints

### Milestone 1: Single-Drone SLAM (End of Phase 1)
- ✅ Global pose graph fully operational
- ✅ Loop closures globally optimized
- ✅ Dense map building works
- ✅ Map quality ≥ ORB-SLAM2 on benchmarks

**Success Metric:** ATE < 1% on TUM VI dataset

---

### Milestone 2: Two-Drone Coordination (End of Phase 2)
- ✅ Network communication established
- ✅ Distributed optimization converges
- ✅ Map merging produces valid result
- ✅ Joint pose graph better than independent

**Success Metric:** Merged map drift < independent estimates

---

### Milestone 3: Full Swarm System (End of Phase 3)
- ✅ N-drone swarm coordinates
- ✅ Exploration achieves >95% coverage
- ✅ Shared map consensus achieved
- ✅ Scales to 8+ drones

**Success Metric:** Coverage gain vs single drone, communication cost

---

## Resource Estimates

| Phase | Duration | Effort | Team |
|-------|----------|--------|------|
| Phase 1 (SLAM) | 4 weeks | ~200 hours | 1-2 engineers |
| Phase 2 (Comms) | 3 weeks | ~150 hours | 1-2 engineers |
| Phase 3 (Swarm) | 4 weeks | ~200 hours | 1-2 engineers |
| **Total** | **11 weeks** | **~550 hours** | **1-2 people** |

---

## Success Criteria

✅ **VIO Phase (DONE)**
- [x] 782 unit tests pass
- [x] All features compile
- [x] Visual tracking robust (91%+ retention)
- [x] IMU fusion working

✅ **SLAM Phase (IN PROGRESS)**
- [ ] Global pose graph operational
- [ ] Loop closures globally optimized
- [ ] Dense maps building
- [ ] ATE < 1% on benchmarks

⏳ **Swarm Phase (PLANNED)**
- [ ] 2-drone coordination works
- [ ] Distributed ADMM converges
- [ ] Map merging produces results
- [ ] 4+ drones coordinate
- [ ] Exploration coverage > 95%

---

## Questions to Address

1. **Single agent SLAM**: How to transition from sliding window to global graph efficiently?
2. **Map merging**: How to handle feature ID collisions across drones?
3. **Synchronization**: How to keep drones time-synchronized without GPS?
4. **Communication**: Real network or simulation initially?
5. **Evaluation**: Existing multi-drone datasets to validate against?
6. **Hardware**: Target: simulation, single drone, or hardware swarm?

---

## Conclusion

RS-VIO has a solid VIO foundation (✅ COMPLETE). The path to full SLAM + swarm coordination is clear:

1. **Short term (4 weeks):** Complete single-drone SLAM with global optimization
2. **Medium term (3 weeks):** Add multi-drone communication & distributed solving
3. **Long term (4 weeks):** Coordinated exploration and full swarm system

**Total effort:** ~550 hours over 11 weeks for a 1-2 person team

The infrastructure is mostly in place. The work is integration, testing, and hardening.

---

**Next Action:** Choose priority (single-drone SLAM vs multi-drone focus) and begin Phase 1 or Phase 2.
