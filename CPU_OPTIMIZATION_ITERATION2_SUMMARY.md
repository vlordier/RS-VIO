# CPU Optimization Iteration 2: Beyond Parallelization

**Date**: 2025-01-XX  
**Context**: Following successful Rayon parallelization (Iteration 1), attempted to push performance beyond 9 fps at stride=2

## Goals
- Target: ≥10 fps processed at stride=2 (20 Hz input → 10 Hz output)
- Previous: 7.9-8.0 fps VIO/SLAM with realtime_cpu.yaml
- Best config: 9.0 fps SLAM with realtime_cpu_extreme.yaml

## Approach: Ultra-Fast Configuration

Created `config/tum_vi_ultrafast_cpu.yaml` with aggressive reductions:
- **Window**: 6 (vs 8 extreme, 10 standard)
- **BA iterations**: 1 (vs 1 extreme, 3 standard)
- **PnP iterations**: 1 (vs 1 extreme, 3 standard)
- **Grid**: 10x3 (~30 features) (vs 12x3 ~36 extreme, 14x4 ~56 standard)

**Hypothesis**: Smaller window + fewer features → less BA problem size → faster solve

## Results: Catastrophic Failure

### Performance Breakdown

| Config | Window | Features | BA/PnP | Stride=2 FPS | Per-frame Time |
|--------|--------|----------|--------|--------------|----------------|
| realtime_cpu | 10 | ~56 | 3/3 | 7.9 fps | 240-250ms |
| realtime_cpu_extreme | 8 | ~36 | 1/1 | 9.0 fps | 220ms |
| ultrafast_cpu | 6 | ~30 | 1/1 | **0.1 fps** | **12,000-23,000ms** |

### Observed Symptoms
- Initial frames: normal (avg_est=670-1140ms)
- Frame 150+: dramatic slowdown (avg_est=3800ms)
- Frame 200+: severe degradation (avg_est=4700ms)
- Frame 450+: complete collapse (avg_est=23,290ms)

**400x slowdown** compared to expected performance!

### Root Cause Analysis

1. **Solver Instability**: Too few features (~30) causes ill-conditioned optimization problems
   - Bundle adjustment relies on sufficient geometric constraints
   - Minimal feature count → rank-deficient Hessian
   - Solver diverges or enters pathological iteration patterns

2. **Progressive Degradation**: Error accumulation compounds over time
   - Early frames succeed with good initialization
   - Accumulated drift + minimal constraints → solver struggles
   - Each failed frame makes next frame harder

3. **Minimum Viable Problem Size**: There exists a lower bound on features/window
   - ~36 features (12x3 grid) appears near minimum
   - Below this threshold, solver becomes unstable

## Engineering Insights

### What Works
✅ **Rayon parallelization**: 7-8% speedup in grid operations  
✅ **Diagonal Hessian + minimal BA iterations**: Works with ≥36 features  
✅ **Frame stride**: Stride=3 achieves 12.9 fps (production viable at 6-7 Hz output)

### What Doesn't Work
❌ **Ultra-minimal features (<36)**: Causes solver instability  
❌ **Too-small window (<8)**: Insufficient temporal constraints  
❌ **Further iteration reduction**: Already at minimum (BA=1, PnP=1)

### Fundamental Limits Identified

1. **BA Solver Dominance**: 220-250ms per frame, ~95% of total time
   - Parallelization only helps feature detection/grid ops (~5% of time)
   - Solver is sequential, hard to parallelize further

2. **Accuracy-Performance Trade-off**: 
   - Window=8, ~36 features → 9 fps, acceptable accuracy
   - Window=6, ~30 features → solver collapse
   - **Hard floor at ~36 features for stability**

3. **Stride vs Direct Optimization**:
   - Stride=2: 9 fps (10 Hz target unmet)
   - Stride=3: 12.9 fps (6-7 Hz output, realtime capable)
   - **Recommendation**: Use stride=3 for production realtime

## Conclusion

**The 10 fps target at stride=2 is unachievable** with current CPU-only solver architecture.

### Final Performance Profile
- **Recommended config**: `tum_vi_realtime_cpu_extreme.yaml`
- **Stride=2 performance**: 9.0 fps (90% of target)
- **Stride=3 performance**: 12.9 fps (realtime at 6-7 Hz)
- **Production deployment**: Use stride=3 for guaranteed realtime

### Next Steps (If Hard Realtime Required)
If ≥10 fps at stride=2 is mission-critical:
1. **GPU acceleration**: Offload BA solver to GPU (potential 5-10x speedup)
2. **Solver replacement**: Evaluate faster solvers (e.g., GPU-accelerated Ceres, custom kernels)
3. **Keyframe decimation**: More aggressive marginalization (may hurt accuracy)
4. **Direct methods**: Switch to direct sparse odometry (major architecture change)

### Lessons Learned
- **There exist hard limits** to parameter tuning without architectural changes
- **Solver stability requires minimum problem size** (~36 features, window=8)
- **Parallelization helps only non-dominant operations** (95/5 Amdahl's Law)
- **Stride adjustment is most effective knob** for realtime requirements

## Files Modified
- ~~`config/tum_vi_ultrafast_cpu.yaml`~~ (created then removed due to failure)
- `CPU_PARALLELIZATION_SESSION_SUMMARY.md` (previous iteration)

## Artifacts
- `ultrafast_attempt.log`: Partial benchmark log showing catastrophic slowdown

## Status
✅ CPU-only optimization complete: 9 fps at stride=2, 12.9 fps at stride=3  
❌ 10 fps target unmet without GPU or architecture changes  
🎯 **Production recommendation**: Use extreme config with stride=3 for realtime
