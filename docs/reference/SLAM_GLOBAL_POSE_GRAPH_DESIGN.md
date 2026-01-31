# SLAM Global Pose Graph Design Document

**Date:** 2026-01-24
**Status:** Design Phase
**Target:** Complete global pose graph for single-drone SLAM

---

## Current Architecture Analysis

### What We Have (Sliding Window VIO)

**SlidingWindow Structure** (`src/estimator/sliding_window/window.rs`):
```rust
pub struct SlidingWindow {
    max_frames: usize,                              // Fixed window size (8-20 frames)
    keyframes: VecDeque<Frame>,                     // Only recent frames
    map_points: HashMap<usize, [f32; 3]>,          // Current point cloud
    loop_closure_constraints: Vec<LoopClosureConstraint>,  // Detected closures
    marginalization_manager: MarginalizationManager, // Schur complement
    imu_preintegrations: Vec<ImuPreintegration>,    // IMU factors
}
```

**Optimization** (`src/estimator/sliding_window/optimization.rs`):
- Builds local `Problem` with only active keyframes
- Uses `LevenbergMarquardtConfig` with Sparse Schur Complement
- Optimization factors:
  - `BundleAdjustmentFactor` - Visual residuals (reprojection)
  - `InterKeyframeImuFactor` - IMU preintegration constraints
  - `LoopClosurePoseFactor` - Relative pose constraints
  - `PnPFactor` - Non-keyframe motion tracking
  - Marginalization priors (from previous window)

**What Works Now:**
- ✅ Local BA over 8-20 keyframes (front-end)
- ✅ Loop closures detected (10+/keyframe in test)
- ✅ IMU preintegration constraints
- ✅ Marginalization with FEJ (First Estimate Jacobian)

**What's Missing:**
- ❌ Global pose graph (all historical keyframes)
- ❌ Global optimization trigger logic
- ❌ Memory management for old poses/points
- ❌ Pose graph serialization/reload
- ❌ Consistent global coordinate frame across loop closures

---

## Design: GlobalPoseGraph

### Goal
Add a **global pose graph** backend that:
1. Stores **all** historical keyframe poses
2. Accumulates **all** loop closure constraints
3. Optimizes **entire graph** when loop closure detected
4. Maintains consistency with sliding window front-end

### Architecture Diagram

```
┌────────────────────────────────────────────────────────────────┐
│                       VIO PIPELINE                              │
├────────────────────────────────────────────────────────────────┤
│                                                                  │
│  FRONT-END: SlidingWindow (8-20 keyframes)                     │
│  ├─ Track features                                              │
│  ├─ Detect new keyframes                                        │
│  ├─ Run local BA                                                │
│  └─ Detect loop closures (candidate pairs)                     │
│         │                                                        │
│         ├─→ Loop closure verified? (descriptor matching)        │
│         │   └─→ Add to GlobalPoseGraph                          │
│         │                                                        │
│  BACK-END: GlobalPoseGraph (all historical poses)              │
│  ├─ Store all keyframe poses                                    │
│  ├─ Accumulate loop closure constraints                         │
│  ├─ Detect "loop closure event" (multiple strong closures)     │
│  │   └─→ Trigger global optimization                           │
│  ├─ Global BA solver (full pose graph)                          │
│  └─ Update sliding window with corrected poses                  │
│       └─→ Marginalization priors propagate fixes               │
│                                                                  │
└────────────────────────────────────────────────────────────────┘
```

---

## GlobalPoseGraph Implementation

### 1. Data Structure

```rust
/// Global pose graph backend for SLAM
pub struct GlobalPoseGraph {
    /// All historical keyframe poses: keyframe_id -> T_W_B
    keyframe_poses: BTreeMap<u64, Pose>,  // BTreeMap for ordered access

    /// All historical 3D map points
    map_points: HashMap<usize, MapPoint>,

    /// Loop closure constraints: (from_id, to_id) -> RelativePose
    loop_closure_edges: Vec<LoopClosureEdge>,

    /// IMU preintegration edges between consecutive keyframes
    imu_edges: Vec<ImuEdge>,

    /// Configuration for optimization
    config: GlobalPoseGraphConfig,

    /// Statistics for analysis
    stats: GraphStatistics,
}

pub struct Pose {
    id: u64,
    T_W_B: Matrix4x4,
    covariance: Matrix6x6,  // Pose uncertainty
    timestamp_ns: i64,
    is_marginalized: bool,
}

pub struct MapPoint {
    id: usize,
    position: Vector3,
    descriptor: Vec<u8>,      // ORB descriptor
    observations: Vec<(u64, usize)>,  // (keyframe_id, feature_id)
    covariance: Matrix3x3,
}

pub struct LoopClosureEdge {
    from_id: u64,
    to_id: u64,
    T_from_to: Matrix4x4,  // Relative transform
    covariance: Matrix6x6,   // Uncertainty of relative pose
    strength: f32,           // Descriptor matching quality
}

pub struct ImuEdge {
    from_id: u64,
    to_id: u64,
    preintegration: ImuPreintegration,
}

pub struct GraphStatistics {
    num_poses: usize,
    num_map_points: usize,
    num_loop_closures: usize,
    last_optimization_time_ms: f64,
    total_optimization_iterations: usize,
}
```

### 2. Core Methods

```rust
impl GlobalPoseGraph {
    /// Create a new global pose graph
    pub fn new(config: GlobalPoseGraphConfig) -> Self { }

    /// Add a new keyframe pose from sliding window
    pub fn add_keyframe_pose(&mut self, keyframe_id: u64, T_W_B: Matrix4x4) { }

    /// Add loop closure constraint
    pub fn add_loop_closure_constraint(&mut self, edge: LoopClosureEdge) { }

    /// Add IMU preintegration constraint
    pub fn add_imu_constraint(&mut self, edge: ImuEdge) { }

    /// Add map point observation
    pub fn add_map_point(&mut self, point: MapPoint) { }

    /// Detect if we should trigger global optimization
    /// Returns (should_optimize, reason)
    pub fn should_optimize(&self) -> (bool, String) { }

    /// Run global bundle adjustment on entire pose graph
    pub fn optimize(&mut self) -> Result<OptimizationResult> { }

    /// Get updated pose for a keyframe (after global optimization)
    pub fn get_pose(&self, keyframe_id: u64) -> Option<&Pose> { }

    /// Get all poses for sliding window update
    pub fn get_all_poses(&self) -> Vec<(u64, Matrix4x4)> { }

    /// Marginalize old poses when memory pressure exists
    pub fn marginalize_oldest_poses(&mut self, num_to_marginalize: usize) { }

    /// Get statistics
    pub fn stats(&self) -> &GraphStatistics { }
}
```

### 3. Loop Closure Event Detection

**When to trigger global optimization:**

Option A: **Simple Threshold**
- Trigger when ≥ N new loop closures added since last optimization
- Default: N = 5

Option B: **Closure Cluster**
- Loop closures form a "strong cluster" if they connect previously unconnected regions
- Example: Closures between (pose 0-10) and (pose 50-60) suggests revisiting old area
- More sophisticated but requires graph analysis

Option C: **Information Gain**
- Estimate reduction in uncertainty by solving global graph
- Trigger when potential ATE improvement > threshold

**Implementation Plan:** Start with Option A, advance to B if needed.

```rust
pub fn should_optimize(&self) -> (bool, String) {
    // Option A: Simple threshold
    if self.new_closures_since_last_optimization >= self.config.closure_threshold {
        return (true, format!(
            "{} new loop closures (threshold: {})",
            self.new_closures_since_last_optimization,
            self.config.closure_threshold
        ));
    }

    // Option B: Memory pressure
    if self.keyframe_poses.len() > self.config.max_poses_before_marginalization {
        return (true, format!(
            "{} poses exceeds max {}",
            self.keyframe_poses.len(),
            self.config.max_poses_before_marginalization
        ));
    }

    (false, "No trigger conditions met".to_string())
}
```

### 4. Optimization Solver

**Build Optimization Problem:**
1. Add pose variables: SE(3) variables for all keyframe poses
2. Add landmark variables: 3D points for map points
3. Add residuals:
   - Visual factors (reprojection errors)
   - Loop closure factors (relative pose constraints)
   - IMU factors (preintegration constraints)
   - Marginalization priors (from previous optimizations)
4. Set Huber loss for robustness
5. Solve with LM + Schur complement

**Code Structure (similar to sliding window optimization):**

```rust
fn optimize_global(&mut self) -> Result<OptimizationResult> {
    let problem = Problem::new();

    // Add pose variables
    for (pose_id, pose) in &self.keyframe_poses {
        problem.add_variable(
            format!("POSE_{}", pose_id),
            ManifoldType::SE3,
            pose.T_W_B,
        );
    }

    // Add landmark variables
    for (lm_id, point) in &self.map_points {
        problem.add_variable(
            format!("LM_{}", lm_id),
            ManifoldType::Euclidean3D,
            point.position,
        );
    }

    // Add residuals
    self.add_visual_factors(&problem);
    self.add_loop_closure_factors(&problem);
    self.add_imu_factors(&problem);
    self.add_marginalization_priors(&problem);

    // Solve
    let solver_config = LevenbergMarquardtConfig::new()
        .with_linear_solver_type(LinearSolverType::SparseSchurComplement)
        .with_max_iterations(50)
        .with_cost_tolerance(1e-7);

    let optimizer = LevenbergMarquardt::new(solver_config);
    let result = optimizer.solve(problem)?;

    // Extract updated poses
    self.extract_updated_poses(&result);

    Ok(OptimizationResult { ... })
}
```

---

## Integration with Sliding Window

### How They Work Together

**Before:** SlidingWindow optimizes locally (last 8-20 frames)
**After:**
1. SlidingWindow optimizes locally (unchanged)
2. New keyframe added to GlobalPoseGraph
3. Loop closures detected → added to GlobalPoseGraph
4. GlobalPoseGraph detects optimization trigger
5. GlobalPoseGraph runs full BA
6. Updated poses → SlidingWindow marginalization priors
7. SlidingWindow continues with global context

### Data Flow

```
processor.rs:process_frame()
    │
    ├─→ sliding_window.add_frame()
    │       │
    │       └─→ sliding_window.optimize()  // Local BA
    │
    ├─→ loop_closure.detect()
    │       │
    │       └─→ global_pose_graph.add_loop_closure_constraint()
    │
    ├─→ global_pose_graph.should_optimize()?
    │       │
    │       └─→ YES: global_pose_graph.optimize()  // Global BA
    │                   │
    │                   └─→ Extract corrected poses
    │                   └─→ Update sliding window priors
    │
    └─→ Next frame processing
```

### Pose Update Mechanism

```rust
impl GlobalPoseGraph {
    pub fn apply_optimized_poses_to_sliding_window(
        &self,
        sliding_window: &mut SlidingWindow,
    ) {
        // Update all keyframe poses in sliding window with corrected global poses
        for (idx, frame) in sliding_window.keyframes_mut().enumerate() {
            if let Some(pose) = self.get_pose(frame.frame_id as u64) {
                frame.state.T_W_B = pose.T_W_B;
                frame.state.covariance = pose.covariance;
            }
        }

        // Update map points with corrected positions
        for (feature_id, point_xyz) in sliding_window.map_points.iter_mut() {
            if let Some(gp) = self.map_points.get(feature_id) {
                *point_xyz = [gp.position.x, gp.position.y, gp.position.z];
            }
        }
    }
}
```

---

## Memory Management

### Problem
Global pose graph grows unbounded:
- 1 hour at 30 FPS = 108,000 frames
- Even with 10 keyframes/sec = 36,000 keyframes
- Each pose: ~256 bytes; each point: ~128 bytes
- Total: ~9.7 GB per hour (unsustainable for embedded)

### Solution: Selective Marginalization

**Strategy:**
1. Keep all loop closure constraints
2. Marginalize old poses (older than N seconds)
3. Compress marginalized poses → prior factors
4. Store in SparseMatrix for efficient solving

**Implementation:**

```rust
pub fn marginalize_oldest_poses(&mut self, num_to_marginalize: usize) {
    let poses_to_remove: Vec<u64> = self.keyframe_poses
        .iter()
        .take(num_to_marginalize)
        .map(|(id, _)| *id)
        .collect();

    for pose_id in poses_to_remove {
        // Compute marginalization prior
        let prior = self.compute_marginalization_prior(pose_id);

        // Store prior for future optimizations
        self.marginalization_priors.push(prior);

        // Remove pose from active set
        self.keyframe_poses.remove(&pose_id);
    }
}

struct MarginalizationPrior {
    associated_poses: Vec<u64>,
    H: SparseMatrix,           // Hessian of marginalized variables
    b: DVector,                 // Gradient vector
}
```

---

## Configuration

```rust
pub struct GlobalPoseGraphConfig {
    /// Minimum loop closures before triggering optimization
    pub closure_threshold: usize,  // Default: 5

    /// Maximum poses to keep in active set
    pub max_poses_before_marginalization: usize,  // Default: 500

    /// Maximum optimization iterations
    pub max_iterations: usize,  // Default: 50

    /// Cost tolerance for convergence
    pub cost_tolerance: f64,  // Default: 1e-7

    /// Enable visualization
    pub enable_visualization: bool,  // Default: false

    /// Log level
    pub log_level: LogLevel,
}
```

---

## Testing Strategy

### Unit Tests
```rust
#[cfg(test)]
mod tests {
    // Test adding poses
    #[test]
    fn test_add_poses() { }

    // Test loop closure constraints
    #[test]
    fn test_add_loop_closure() { }

    // Test optimization trigger logic
    #[test]
    fn test_should_optimize() { }

    // Test pose retrieval
    #[test]
    fn test_get_pose() { }

    // Test marginalization
    #[test]
    fn test_marginalize_poses() { }
}
```

### Integration Tests
```rust
// Test with sliding window
#[test]
fn test_sliding_window_integration() {
    let mut sw = SlidingWindow::new(10);
    let mut gpg = GlobalPoseGraph::new(config);

    // Add frames to sliding window
    // Detect loop closures
    // Verify global optimization fixes trajectory
}

// Test with real dataset (TUM VI)
#[test]
fn test_with_tum_vi_dataset() {
    // Load dataset
    // Process frames
    // Verify ATE < 1%
}
```

---

## Files to Create/Modify

### New Files
- `src/estimator/global_pose_graph.rs` - GlobalPoseGraph struct + methods (400-500 lines)
- `src/estimator/global_pose_graph/tests.rs` - Unit tests (300-400 lines)
- `src/estimator/mod.rs` - Export global_pose_graph module

### Modified Files
- `src/estimator/estimator/state.rs` - Add `global_pose_graph: GlobalPoseGraph` to Estimator
- `src/estimator/estimator/processor.rs` - Call `global_pose_graph.add_*` methods, trigger optimization
- `src/estimator/mod.rs` - Re-export GlobalPoseGraph
- Tests to verify full integration

---

## Implementation Phases

### Phase 1: Core Data Structure (2 days)
- Create GlobalPoseGraph struct
- Implement add_pose, add_constraint, get_pose methods
- Basic tests

### Phase 2: Optimization Logic (3 days)
- Build optimization problem
- Implement global BA solver
- Test with synthetic data

### Phase 3: Integration (2 days)
- Connect to processor.rs
- Handle pose updates
- Test with sliding window

### Phase 4: Testing & Tuning (2 days)
- Comprehensive tests
- Benchmark on TUM VI
- Performance optimization

**Total:** ~1-2 weeks for Phase 1 (Global Pose Graph)

---

## Success Criteria

✅ **Functional Goals**
- [ ] GlobalPoseGraph compiles and tests pass
- [ ] Global BA solver converges on synthetic data
- [ ] Sliding window integration works
- [ ] Loop closures trigger optimization

✅ **Performance Goals**
- [ ] Global BA completes in < 5 sec (50 iterations)
- [ ] Memory usage < 2GB for 1 hour flight
- [ ] No deadlock or race conditions

✅ **Accuracy Goals**
- [ ] ATE < 1% on TUM VI (single-drone SLAM)
- [ ] Loop closures reduce drift by > 50%
- [ ] Map consistency after loop closure

---

## Next Steps

1. **Review Design** - Get feedback on architecture
2. **Create GlobalPoseGraph struct** - Core data structure
3. **Implement basic methods** - add_pose, add_constraint, get_pose
4. **Build optimization solver** - Full BA on global graph
5. **Integrate with processor** - Connect to main pipeline
6. **Test and benchmark** - Verify on real data
