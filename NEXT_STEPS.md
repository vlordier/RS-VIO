# Next Steps: Quick Reference

**Current State**: Fusion module ✅ complete  
**Total Remaining Work**: 5 phases, ~280-370 hours (1-2 developer-quarters)

---

## Immediate Next Steps (This Week)

### Option A: Auto-Calibration (Recommended for Production)
**Why**: Blocking production deployment; enables manual, controlled calibration.  
**Effort**: 57-70 hours  
**Starting Point**: [IMPLEMENTATION_ROADMAP.md](IMPLEMENTATION_ROADMAP.md#phase-5-auto-calibration-system-high-priority) Phase 5

**Sub-tasks** (in order):
1. IMU self-calibration (12-15h)
2. Camera intrinsics (15-18h)
3. Stereo extrinsics (12-15h)
4. Time offset estimation (10-12h)
5. Quality gates + workflows (8-10h)

### Option B: Feature Detection Enhancements (Complementary)
**Why**: Improves tracking robustness; enables SuperPoint integration.  
**Effort**: 65-80 hours  
**Starting Point**: [IMPLEMENTATION_ROADMAP.md](IMPLEMENTATION_ROADMAP.md#phase-6-feature-detection-sota-high-priority) Phase 6

**Sub-tasks** (in order):
1. Track-first, detect-to-fill (15-18h)
2. Per-feature uncertainty (12-14h)
3. SuperPoint integration (18-22h)
4. LightGlue matching (12-16h)

### Option C: Integration Testing (Fastest Validation)
**Why**: Validates existing work; establishes baselines for future optimization.  
**Effort**: 38-48 hours  
**Starting Point**: [IMPLEMENTATION_ROADMAP.md](IMPLEMENTATION_ROADMAP.md#phase-9-integration-testing--benchmarking-high-priority) Phase 9

**Sub-tasks** (in order):
1. Metrics framework (8-10h)
2. EuRoC benchmark suite (10-12h)
3. Regression testing / CI (6-8h)
4. Platform validation (8-10h)

---

## Recommended Sequence

### Week 1-2: Phase 5 (Auto-Calibration)
- Foundation for production
- No dependencies on other work
- Directly unblocks deployment

### Week 3: Phase 9 (Testing)
- Validate existing system
- Establish performance baselines
- Enable regression detection

### Week 4-5: Phase 6 (Feature Detection)
- Enhance tracking quality
- Enable GPU support (SuperPoint/LightGlue)
- Improve relocalization robustness

### Weeks 6+: Phases 7-8 (Optimization)
- Real-time budget tuning
- Advanced fusion methods
- Platform-specific optimization

---

## Decision Matrix

| Goal | Choose | Est. Time | When |
|------|--------|-----------|------|
| **Ship to production** | Phase 5 | 2 weeks | Now |
| **Validate quality** | Phase 9 | 1.5 weeks | After Phase 5 |
| **Improve tracking** | Phase 6 | 2-2.5 weeks | Parallel with Phase 9 |
| **Optimize latency** | Phase 7 | 1 week | After Phases 5-6 |
| **SOTA geometric SR** | Phase 8 | 2 weeks | Later (nice-to-have) |

---

## File to Read Next

**📄 [IMPLEMENTATION_ROADMAP.md](IMPLEMENTATION_ROADMAP.md)**
- Complete breakdown of all phases
- Technical requirements per sub-task
- Dependencies and sequencing
- Known limitations and questions

---

## Commands to Get Started

```bash
# View full roadmap
cat IMPLEMENTATION_ROADMAP.md

# Start Phase 5 (auto-calibration)
# Create new branch:
git checkout -b feature/auto-calibration

# Start Phase 6 (feature detection)
# Create new branch:
git checkout -b feature/feature-detection-sota

# Start Phase 9 (testing)
# Create new branch:
git checkout -b feature/integration-testing
```

---

**Last Updated**: January 21, 2026  
**Status**: Ready for Phase 5 implementation
