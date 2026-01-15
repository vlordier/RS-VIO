## Loop Closure Detection Feature

Implements loop closure detection for global consistency in large-scale SLAM.

### Key Components
- LoopClosureDetector with configurable thresholds and frame/time gating
- KeyframeDatabase with cosine similarity candidate search and pruning
- LoopClosureConstraint with anisotropic information matrix from inliers/similarity
- Traits: DescriptorMatcher and GeometricVerifier for pluggable TOP design (default CosineMatcher + SimpleRelativePoseVerifier)

### Highlights
- Cosine similarity-based matcher via trait; swappable implementations
- Geometric verifier trait; default relative-pose checker with similarity gating
- Time-based gating (optional ns) plus frame-gap gating
- Constraint generation with anisotropic information matrix (translation/rotation sigmas)
- Database size limits to control memory footprint
- 15 comprehensive unit tests (added matcher/verifier/time/aniso coverage)

### Stats
- Lines added: ~630 total
- Files: new src/optimization/loop_closure.rs; updated src/optimization/mod.rs
- Build: cargo build (clean)
- Tests: 166 passing (151 baseline + 15 loop-closure tests)

### Next
- Optionally add advanced descriptors (ORB/BRIEF/CNN), fast ANN search, epipolar/homography verification, and pose graph backend integration.
