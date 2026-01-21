# Phase 9: Multi-Drone SLAM with Boids Swarm - Planning

## Overview

Phase 9 implements **Multi-Drone SLAM with Boids Swarm Intelligence** - enabling coordinated mapping by multiple drones using flocking behavior for efficient exploration.

**Status**: 🔄 IN PROGRESS
**Prerequisites**: ✅ Phases 1-8 complete

---

## Boids Algorithm Integration

### Core Boids Behaviors
1. **Separation**: Avoid collisions with nearby drones
2. **Alignment**: Match velocity with nearby drones
3. **Cohesion**: Move toward average position of nearby drones
4. **Goal Seeking**: Navigate toward exploration targets

### SLAM-Specific Extensions
5. **Coverage Maximization**: Explore unmapped areas
6. **Loop Closure Coordination**: Share detected loops
7. **Map Merging**: Combine individual maps efficiently

---

## Architecture

```
Multi-Drone SLAM with Boids
├─ 9.1: Boids Coordination (NEW)
│  ├─ Separation (collision avoidance)
│  ├─ Alignment (velocity matching)
│  ├─ Cohesion (flock together)
│  ├─ Goal seeking (exploration targets)
│  └─ Obstacle avoidance
│
├─ 9.2: Map Merging (NEW)
│  ├─ Place recognition across drones
│  ├─ Relative pose estimation
│  ├─ Map alignment and fusion
│  └─ Conflict resolution
│
├─ 9.3: Distributed Optimization (NEW)
│  ├─ Distributed pose graph
│  ├─ Consensus-based optimization
│  ├─ Communication protocol
│  └─ Network topology handling
│
└─ 9.4: Exploration Strategy (NEW)
   ├─ Frontier detection
   ├─ Task allocation
   ├─ Coverage metrics
   └─ Coordination efficiency

OUTPUTS:
├─ Merged global map
├─ Coordinated exploration paths
└─ Swarm state (positions, velocities)
```

---

## Implementation Plan

### 9.1: Boids Coordination (Priority 1)
**Purpose**: Implement flocking behavior for coordinated drone movement

**Components**:
- Boid agent with position, velocity, acceleration
- Separation force (avoid collisions)
- Alignment force (match neighbors)
- Cohesion force (stay together)
- Goal seeking (navigate to targets)
- Force combination and velocity updates

**API Design**:
```rust
pub struct Boid {
    pub id: usize,
    pub position: Vector3,
    pub velocity: Vector3,
    pub acceleration: Vector3,
}

pub struct BoidsConfig {
    pub separation_radius: f32,
    pub alignment_radius: f32,
    pub cohesion_radius: f32,
    pub max_speed: f32,
    pub max_force: f32,
}

impl Boid {
    pub fn update(&mut self, neighbors: &[&Boid], target: Option<Vector3>);
    pub fn separation(&self, neighbors: &[&Boid]) -> Vector3;
    pub fn alignment(&self, neighbors: &[&Boid]) -> Vector3;
    pub fn cohesion(&self, neighbors: &[&Boid]) -> Vector3;
}
```

**Tests**:
- Boid creation and updates
- Separation force computation
- Alignment force computation
- Cohesion force computation
- Goal seeking behavior
- Force combination
- Velocity limits
- Multi-boid simulation

**Estimated**: 300-350 lines, 8-10 tests

### 9.2: Map Merging (Priority 2)
**Purpose**: Merge maps from multiple drones into global map

**Components**:
- Inter-drone place recognition
- Relative transformation estimation
- Map alignment (ICP, feature-based)
- Conflict resolution (timestamps, uncertainty)
- Merged map representation

**API Design**:
```rust
pub struct DroneMap {
    pub drone_id: usize,
    pub poses: Vec<Pose>,
    pub landmarks: Vec<Point3D>,
    pub timestamp: f64,
}

pub struct MapMerger {
    config: MapMergeConfig,
}

impl MapMerger {
    pub fn merge_maps(&self, map1: &DroneMap, map2: &DroneMap) -> MergedMap;
    pub fn estimate_relative_transform(&self, map1: &DroneMap, map2: &DroneMap) -> Option<Transform>;
    pub fn resolve_conflicts(&self, maps: &[DroneMap]) -> DroneMap;
}
```

**Tests**:
- Map creation and management
- Relative transform estimation
- Map alignment
- Conflict resolution
- Merged map validation

**Estimated**: 350-400 lines, 8-10 tests

### 9.3: Distributed Optimization (Priority 3)
**Purpose**: Optimize pose graph across multiple drones with consensus

**Components**:
- Distributed pose graph (local + shared nodes)
- Consensus ADMM optimization
- Communication messages
- Convergence detection
- Network partition handling

**API Design**:
```rust
pub struct DistributedPoseGraph {
    pub drone_id: usize,
    pub local_nodes: Vec<PoseNode>,
    pub shared_nodes: Vec<PoseNode>,
    pub edges: Vec<Edge>,
}

pub struct ConsensusOptimizer {
    config: ConsensusConfig,
}

impl ConsensusOptimizer {
    pub fn optimize_distributed(&mut self, graph: &mut DistributedPoseGraph, messages: &[Message]);
    pub fn prepare_message(&self, graph: &DistributedPoseGraph) -> Message;
    pub fn update_from_consensus(&mut self, graph: &mut DistributedPoseGraph, consensus: &[Pose]);
}
```

**Tests**:
- Distributed graph construction
- Message creation and parsing
- Consensus updates
- Convergence detection
- Network partition handling

**Estimated**: 350-400 lines, 8-10 tests

### 9.4: Exploration Strategy (Priority 4)
**Purpose**: Coordinate exploration to maximize coverage

**Components**:
- Frontier detection (unknown boundaries)
- Utility-based frontier ranking
- Task allocation (auction or greedy)
- Coverage metrics
- Coordination overhead tracking

**API Design**:
```rust
pub struct Frontier {
    pub position: Vector3,
    pub size: f32,
    pub utility: f32,
}

pub struct ExplorationPlanner {
    config: ExplorationConfig,
}

impl ExplorationPlanner {
    pub fn detect_frontiers(&self, map: &DroneMap) -> Vec<Frontier>;
    pub fn assign_targets(&self, frontiers: &[Frontier], drones: &[Boid]) -> Vec<(usize, Frontier)>;
    pub fn compute_coverage(&self, maps: &[DroneMap]) -> f32;
}
```

**Tests**:
- Frontier detection
- Utility computation
- Task allocation
- Coverage metrics
- Efficiency analysis

**Estimated**: 250-300 lines, 6-8 tests

---

## Total Estimates

| Component | Lines | Tests | Priority |
|-----------|-------|-------|----------|
| 9.1 Boids Coordination | 300-350 | 8-10 | 1 |
| 9.2 Map Merging | 350-400 | 8-10 | 2 |
| 9.3 Distributed Optimization | 350-400 | 8-10 | 3 |
| 9.4 Exploration Strategy | 250-300 | 6-8 | 4 |
| **Total** | **1250-1450** | **30-38** | - |

---

## Integration with Phases 1-8

```
Phase 7 Loop Closure → Inter-drone loop detection
Phase 8 Dense Maps → Merged 3D reconstruction
Phase 6 Features → Map alignment anchors
Boids Coordination → Efficient exploration
```

---

## Performance Targets

| Operation | Target | Notes |
|-----------|--------|-------|
| Boids update | <10ms | Per drone, 100Hz capable |
| Map merge | <1s | Pairwise merge |
| Consensus iteration | <50ms | Per drone |
| Frontier detection | <100ms | Per drone |

---

## Implementation Order

1. **Phase 9.1: Boids Coordination** ← Start here
2. **Phase 9.2: Map Merging**
3. **Phase 9.4: Exploration Strategy**
4. **Phase 9.3: Distributed Optimization**

---

*Planning for multi-drone SLAM with swarm intelligence*
*Ready to implement boids coordination*
