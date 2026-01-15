## Loop-closure Pose-graph Integration

Adds loop-closure constraint integration into the VIO optimization pipeline with a pose-graph relative pose factor.

### Core Factor
- **LoopClosurePoseFactor**: SE3 relative pose residual with information matrix weighting
  - Residual: `sqrt(info) * Log(T_meas^{-1} * T_1 * T_2^{-1})`
  - Jacobian: small-angle approximation for efficiency
  - Unit tests: perfect measurement (zero residual) and translation error

### Pipeline Integration
- **SlidingWindow**: stores loop-closure constraints; prunes when KFs drop
  - `add_loop_closure_constraints`: inject detector output
  - Constraints added with Huber loss during optimization
  - Frame ID to window index mapping for constraint lookup

### Estimator
- **Loop-closure detector**: initialized with default config
- **Keyframe descriptor**: BoW using feature stats + pose embedding (10-dim)
- Called after each keyframe insertion, constraints injected before optimization

### Testing
- Unit tests for pose factor (zero residual, translation error)
- Estimator creation test confirms detector initialization

### Dependencies
Built on top of PR #7 (loop-closure detection module with RANSAC/Hamming matchers).
