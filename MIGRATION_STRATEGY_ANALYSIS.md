# develop-old to develop Migration Strategy Analysis

## Executive Summary

After analysis of the 176-commit develop-old branch, we've identified architectural incompatibilities that prevent straightforward cherry-picking of major features. This document outlines the findings and a recommended path forward.

## Key Findings

### ✅ Successfully Ported (3 PRs Completed)
1. **PR #36**: Extended configuration profiles
   - Calibration-specific parameters
   - IMU tuning variants
   - Status: **MERGED**

2. **PR #37**: TUM-VI specific parameters
   - Dataset-specific optimizations
   - Camera calibration
   - Status: **MERGED**

3. **PR #39**: Fusion and teacher model configs
   - IMU fusion variants
   - Teacher-student training configurations
   - Status: **OPEN**

### ❌ Incompatible Modules (Cannot Port Directly)

The following modules were deleted in develop (likely due to architectural refactoring) and cannot be cherry-picked:

| Module | Affected Commits | Reason |
|--------|------------------|--------|
| `src/evaluation/` | 4a1f28f, a1ee93c | Evaluation framework deleted; uses separate metrics approach |
| `src/optimization/cpp_*` | Multiple | C++ optimization layer removed; pure Rust approach adopted |
| Phase 1-3 SLAM code | 9c78069, 7364475 | Core SLAM refactored; incompatible data structures |
| `src/feature_tracker/stereo_*` | Multiple | Stereo matching redesigned |
| Async/Phase 4 code | Multiple | Concurrency model changed significantly |

### ⚠️ Partially Compatible Features

| Feature | Status | Notes |
|---------|--------|-------|
| Config files (CPU realtime, 4seasons) | Conflicts | Can manually recreate without test modifications |
| Documentation (Phase reports) | Clean | Can add directly |
| Benchmarking scripts | Conflicts | Test framework changes in develop |
| Performance optimizations | Blocked | Depend on deleted modules |

## Recommended Migration Path

### Phase 1: Configuration & Documentation (IMMEDIATE)
- ✅ PR #39: Fusion/teacher configs (ready)
- Add separate PR for CPU realtime config (manual creation to avoid test conflicts)
- Port documentation files from develop-old

### Phase 2: Evaluation Framework Recreation (MEDIUM PRIORITY)
Rather than cherry-picking deleted evaluation code:
1. Analyze existing metrics in develop's output module
2. Design API-compatible evaluation interfaces
3. Implement trajectory RPE separately
4. Create clean PR for new evaluation tools

### Phase 3: Performance Features (LOWER PRIORITY)
- Frame stride optimization (separately from test modifications)
- Benchmarking utilities (recreate for current test framework)
- CPU profile configurations (manual YAML files)

### Phase 4: Skip or Redesign
The following should be skipped or redesigned:
- SLAM Phase 1-3 improvements (incompatible architecture)
- C++ optimization wrappers (wrong approach for current design)
- Full async/concurrency refactor (Phase 4 - already in develop direction)
- Old stereo matching code (redesigned in develop)

## Implementation Strategy

### For Each Feature Category:

**Configuration Files**
```
1. Identify YAML files in develop-old/config/
2. Check if they exist in develop
3. If conflicts: manually recreate with current parameters
4. Create single PR per logical group (e.g., "CPU profiles", "Dataset configs")
```

**Documentation**
```
1. Identify markdown files in develop-old root
2. Update dates/context references
3. Create separate PR per phase/topic
4. Include migration notes where relevant
```

**Code Features**
```
1. Analyze dependencies in develop-old
2. If depends on deleted modules: redesign for current architecture
3. If self-contained: cherry-pick to temporary branch
4. Resolve conflicts manually by reimplementing for develop patterns
5. Create minimal, focused PR
```

## Metrics & Criteria

For each commit in develop-old, evaluate:

| Criterion | Green Light | Red Light |
|-----------|-----------|----------|
| File existence | All files exist in develop | Files deleted in develop |
| Dependencies | Uses only existing modules | Depends on deleted modules |
| Test impact | No test modifications needed | Requires test rewrite |
| Conflict resolution | < 5 files affected | > 10 conflicts |
| Value ratio | Clear user benefit | Marginal improvement |

## Estimated Timeline

- **Phase 1 (Configs)**: 2-3 days → 2-3 PRs
- **Phase 2 (Evaluation)**: 5-7 days → 1-2 PRs
- **Phase 3 (Performance)**: 3-4 days → 2-3 PRs
- **Phase 4 (Skip)**: 0 days → 0 PRs

**Total Estimated**: 10-15 working days for 5-8 additional PRs

## Risk Assessment

### High Risk (Skip)
- Porting deleted modules requires full reimplementation
- SLAM phase improvements may conflict with current research direction
- Async code changes may regress current stability

### Medium Risk (Manual Recreation)
- Test file modifications require careful verification
- Benchmark utilities need current framework compatibility check
- Documentation may reference deleted code

### Low Risk (Safe to Port)
- Configuration files (pure YAML, no code deps)
- Documentation additions
- Comments/doc string improvements

## Conclusion

Rather than attempting a "big bang" port of 176 commits, **strategic, targeted PRs** focusing on:
1. **New configurations** (config files with no code dependencies)
2. **Documentation improvements** (standalone markdown files)
3. **New evaluation tools** (redesigned for current architecture)

This approach provides:
- ✅ Better code review (smaller, focused PRs)
- ✅ Lower merge conflict risk
- ✅ Clearer commit history
- ✅ Ability to validate each improvement independently
- ✅ No risk of reintroducing deleted code

**Next Steps**: Begin with PR #39 (fusion configs), then manually create CPU realtime config PR, then design evaluation framework PR.
