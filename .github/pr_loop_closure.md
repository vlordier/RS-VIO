## Loop Closure Detection Feature

Implements loop closure detection for global consistency in large-scale SLAM with full traits-oriented programming (TOP) design.

### Key Components
- **LoopClosureDetector** with configurable thresholds and frame/time gating
- **KeyframeDatabase** with BTreeMap-based indexing and pruning
- **LoopClosureConstraint** with anisotropic information matrix from inlier ratio/similarity
- **Traits**: `DescriptorMatcher` and `GeometricVerifier` for pluggable TOP design

### Implementations

**Matchers** (trait `DescriptorMatcher`):
- `CosineMatcher`: Cosine similarity on f64 descriptors (default, fastest)
- `HammingMatcher`: Binary XOR distance for ORB/BRIEF-style descriptors

**Verifiers** (trait `GeometricVerifier`):
- `SimpleRelativePoseVerifier`: Similarity threshold check (default, sub-10μs)
- `RansacEpipolarVerifier`: RANSAC with epipolar geometry and configurable iterations (1000 default)

### Highlights
- **Traits-Oriented Programming**: Pluggable matcher/verifier via `new_with()` constructor
- **Time-based gating**: Configurable `min_time_gap_ns` with frame-gap fallback
- **Anisotropic information matrix**: Separate translation (0.25m) / rotation (0.05rad) sigmas
- **Memory-safe**: BTreeMap database, no unsafe code, trait objects with Send+Sync bounds
- **Real-time capable**: Database size limits (5000 default), early-exit gating, RANSAC budget
- **Comprehensive testing**: 19 unit tests covering gating, thresholds, traits, RANSAC, Hamming
- **Benchmarked**: Criterion benchmarks for matcher/verifier latency and database scaling

### Stats
- Lines added: ~880 total (loop_closure.rs) + 270 (benchmarks)
- Files: new `src/optimization/loop_closure.rs`, `benches/loop_closure.rs`; updated `src/optimization/mod.rs`, `Cargo.toml`
- Build: cargo build (clean), no warnings
- Tests: **19 loop-closure tests passing**, 170 total library tests

### Benchmarks
Criterion benchmarks included for:
- Matcher comparison (Cosine vs Hamming)
- Verifier comparison (Simple vs RANSAC with varying iterations)
- Database scaling (10/50/100/500 keyframes)
- End-to-end loop closure detection

Expected performance (release build):
- Cosine matching: <100μs per candidate
- Hamming matching: <50μs per candidate
- Simple verification: <10μs per candidate
- RANSAC verification (1000 iters): <1ms per candidate

### Next
- Optionally add CNN descriptors, approximate nearest neighbor (ANN) search libraries (HNSW/FAISS), integration with pose graph backend, and loop closure triggering in VIO pipeline.
