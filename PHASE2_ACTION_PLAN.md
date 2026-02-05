# develop-old Integration: Phase 2 Action Plan

## Executive Summary

Phase 1 ✅ (Config PRs) is complete. This document outlines Phase 2-4 implementation specifics with concrete action items.

---

## Phase 2: Evaluation Framework Redesign (5-7 days)

### Current Status
- develop has: Output module with basic metrics
- develop-old has: Full evaluation module (deleted in develop) with:
  - Trajectory evaluation tools
  - RPE (Relative Pose Error) calculation
  - Ground truth comparison
  - Performance metrics

### Problem Statement
Cannot cherry-pick evaluation module directly because:
1. Module path changed (`src/evaluation/` → unknown in develop)
2. Data structures likely modified
3. Integration point unclear in current architecture

### Solution Strategy

#### Step 1: Analyze Existing Metrics (Day 1)
```bash
# 1a. Check what metrics output currently produces
grep -r "metric\|eval\|error" src/output/ --include="*.rs"

# 1b. Review output module structure
ls -la src/output/

# 1c. Find where evaluation/metrics are used
grep -r "evaluation\|trajectory\|RPE" src/ --include="*.rs"
```

**Deliverable**: Metrics interface summary document

#### Step 2: Design Evaluation API (Day 1-2)
Create new `src/evaluation/` module compatible with develop:

```rust
// src/evaluation/mod.rs
pub trait TrajectoryEvaluator {
    fn compute_rpe(&self, estimated: &[Pose], ground_truth: &[Pose]) -> Result<MetricSet>;
    fn compute_ate(&self, estimated: &[Pose], ground_truth: &[Pose]) -> Result<MetricSet>;
}

pub struct MetricSet {
    pub mean_rpe: f64,
    pub std_rpe: f64,
    pub mean_ate: f64,
    pub std_ate: f64,
}
```

**Deliverable**: New evaluation module design doc + trait definitions

#### Step 3: Implement Core Metrics (Day 2-3)
Port only the math/algorithms from develop-old:
- RPE calculation (pose-to-pose error)
- ATE calculation (absolute trajectory error)
- Error statistics (mean, std, median)
- Ground truth alignment

**Files to create**:
- `src/evaluation/rpe.rs` - Relative pose error
- `src/evaluation/ate.rs` - Absolute trajectory error
- `src/evaluation/metrics.rs` - Metric aggregation

#### Step 4: Integration & Testing (Day 3-4)
- Add evaluation output to binary/tests
- Validate with existing datasets
- Create benchmark comparisons

#### Step 5: PR Preparation (Day 4-5)
- Ensure clean separation from core VIO
- Add documentation with examples
- Create quick-start guide

**Expected PR**:
- Title: "feat: evaluation framework for trajectory metrics"
- Lines: 300-400
- Files: 4-5 new files in `src/evaluation/`

---

## Phase 3: Performance Features & Utilities (3-4 days)

### Feature 1: Frame Stride Optimization

**Source**: develop-old commits related to frame skipping

**Goal**: Allow processing every Nth frame for CPU-only targets

**Implementation**:
```bash
# Create utility module
src/utilities/frame_stride.rs

# Add config parameter
camera.frame_stride: 1  # Process every 1st frame (default)
                       # 2 = process every 2nd frame (50% throughput)
```

**PR Scope**: 
- Configuration parameter addition
- Frame skip logic in feature tracker
- Benchmark documentation

---

### Feature 2: CPU Profiling Configurations

**Status**: Already partially done in PR #37

**Remaining**:
- Add 4Seasons CPU-optimized config
- Add EuRoC CPU-optimized config
- Document tuning methodology
- Create CPU tuning guide

**PR Scope**:
- New YAML config files
- Tuning methodology document
- Performance baseline comparison

---

### Feature 3: Benchmarking Infrastructure

**Source**: develop-old benchmarking commits

**What exists in develop**:
- `tests/slam_phase2c_benchmarking.rs`

**What to add**:
- Benchmark result comparison tools
- Automated performance reporting
- Dataset-specific benchmarks

**PR Scope**:
- Benchmarking utilities (not modifying tests)
- Performance result parsing
- Report generation

---

## Phase 4: Documentation & Cleanup (2-3 days)

### Feature 1: Archive Organization

**Status**: Already done in develop-old

**Action**: Document archive structure and reference important sections
- Create `.archive/README.md` explaining organization
- Link to specific session reports
- Note completed features

### Feature 2: Phase Summary Integration

**Status**: Documentation exists but scattered

**Action**: Create unified phase timeline:
```markdown
# RS-VIO Development Phases

## Phase 1: SLAM Foundation (Completed)
- Visual factor integration
- Pose graph optimization
- Keyframe management

## Phase 2: IMU Integration (Completed)
- IMU factor implementation
- Fusion initialization
- Benchmarking infrastructure

## Phase 3: Performance Optimization (Completed)
- CPU realtime profiles
- Feature detection optimization
- Marginalization efficiency

## Phase 4: Advanced Features (In Progress)
- Teacher-student learning
- Evaluation framework
- Async concurrency

## Phase 5+: Future Work
- (Define as needed)
```

---

## Risk Assessment by Phase

### Phase 2 Risks
| Risk | Probability | Impact | Mitigation |
|------|------------|--------|-----------|
| Data structure mismatch | High | Medium | Design API first, don't copy code |
| Performance regression | Medium | High | Comprehensive benchmarking |
| API incompleteness | Medium | Low | Extensible trait design |

### Phase 3 Risks
| Risk | Probability | Impact | Mitigation |
|------|------------|--------|-----------|
| Frame stride breaks stereo | Medium | High | Test with all datasets |
| Config tuning incomplete | High | Low | Document as WIP |
| Benchmark tools conflicts | Low | Medium | Separate from test files |

### Phase 4 Risks
| Risk | Probability | Impact | Mitigation |
|------|------------|--------|-----------|
| Archive incompleteness | Low | Low | Document what's available |
| Phase documentation gaps | Medium | Low | Add notes where unclear |

---

## Daily Schedule for Next 2 Weeks

### Week 2 (Next 5 days)

**Day 1: Evaluation Analysis**
- Analyze current output module
- Design evaluation API
- Create design document

**Day 2: Evaluation Implementation Starts**
- Implement RPE calculation
- Add ATE calculation
- Create test suite

**Day 3: Evaluation Completion**
- Complete metrics module
- Add comprehensive tests
- Benchmark validation

**Day 4: PR Review Prep**
- Polish code
- Add documentation
- Prepare benchmark comparisons

**Day 5: Create PR #42**
- Push evaluation framework PR
- Resolve any CI issues
- Request reviews

### Week 3 (Next 3-5 days)

**Day 1: CPU Features**
- Create CPU-optimized configs
- Add frame stride support
- Document tuning guide

**Day 2-3: Performance PRs**
- Create benchmarking utilities PR
- Create performance configs PR
- Validation & testing

**Day 4: Archive Documentation**
- Organize archive structure
- Create index document
- Link phase reports

**Day 5: Final Integration**
- Ensure all PRs pass CI
- Document complete integration
- Create summary report

---

## Success Criteria

- [ ] Phase 2 PR passes CI (evaluation framework)
- [ ] Phase 3 PRs pass CI (2-3 performance features)
- [ ] Phase 4 documentation is organized
- [ ] Zero architectural regressions
- [ ] Performance benchmarks comparable to develop-old
- [ ] All new code has test coverage
- [ ] Documentation is complete and accurate
- [ ] Community can understand migration decisions

---

## Technical Debt & Future Work

### Known Issues to Address Later
- Evaluation module completeness (more metrics)
- Stereo matching performance optimization
- Async architecture finalization
- GPU acceleration roadmap

### Not Included in This Plan
- Full async/concurrency refactor (wrong approach)
- C++ optimization layer (deprecated)
- SLAM Phase 1-3 improvements (check current research first)
- Feature matching algorithm changes

---

## References

- **Strategy Document**: [MIGRATION_STRATEGY_ANALYSIS.md](MIGRATION_STRATEGY_ANALYSIS.md)
- **Progress Tracker**: [MIGRATION_PROGRESS.md](MIGRATION_PROGRESS.md)
- **Open PRs**: #39 (configs), #40 (strategy), #41 (progress)
- **Source**: develop-old (176 commits)
- **Target**: develop

---

## Questions & Decisions Needed

1. **Evaluation Priority**: Should we implement evaluation framework or defer?
   - Recommendation: High priority (useful for benchmarking)

2. **Frame Stride Scope**: Full feature or configuration-only?
   - Recommendation: Configuration first, feature later if needed

3. **Benchmarking Tools**: Keep in tests or separate module?
   - Recommendation: Separate module to avoid test conflicts

4. **Archive Retention**: Keep all historical docs or cleanup?
   - Recommendation: Keep organized in `.archive/` with index
