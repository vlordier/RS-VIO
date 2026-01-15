## Loop Closure Detection Feature

Implements loop closure detection for global consistency in large-scale SLAM.

### Key Components
- LoopClosureDetector with configurable thresholds and frame-gap enforcement
- KeyframeDatabase with cosine similarity candidate search and pruning
- LoopClosureConstraint with information matrix estimated from match quality

### Highlights
- Cosine similarity-based descriptor matching (normalized [0,1])
- Top-K candidate retrieval with distance thresholding
- Constraint generation with SE(3) relative pose and information matrix scaling
- Database size limits to control memory footprint
- 9 comprehensive unit tests added

### Stats
- Lines added: 492
- Files: new src/optimization/loop_closure.rs; updated src/optimization/mod.rs
- Build: cargo build (clean)
- Tests: 160 passing (151 existing + 9 new)

### Next
- Optionally add advanced descriptors (ORB/BRIEF/CNN), fast ANN search, epipolar/homography verification, and pose graph backend integration.
