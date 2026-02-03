# RS-VIO Documentation Index

**Status**: Phase 3 - Optimization Complete ✅  
**Current Focus**: Batch processing, SIMD optimization, and real-time performance  

---

## 📖 Essential Documentation

### Quick Start (5 minutes)
1. **[README.md](README.md)** - Project overview
2. **[QUICKSTART.md](QUICKSTART.md)** - Setup and first run

### Understanding the System
- **[IMU_ARCHITECTURE_ANALYSIS.md](IMU_ARCHITECTURE_ANALYSIS.md)** - System architecture and design
- **[TIGHT_COUPLING_ROADMAP.md](TIGHT_COUPLING_ROADMAP.md)** - Visual-inertial coupling details
- **[IMU_README.md](IMU_README.md)** - IMU module overview
- **[IMU_IMPLEMENTATION_COMPLETE.md](IMU_IMPLEMENTATION_COMPLETE.md)** - Implementation status
- **[IMU_STATUS.md](IMU_STATUS.md)** - Current system status

### Phase 3: Batch Processing & SIMD Optimization
- **[OPTIMIZATION_BATCH_SIMD.md](OPTIMIZATION_BATCH_SIMD.md)** - Technical deep dive
- **[BATCH_PROCESSING_GUIDE.md](BATCH_PROCESSING_GUIDE.md)** - API reference and usage
- **[PHASE3_SUMMARY.md](PHASE3_SUMMARY.md)** - Work summary
- **[PHASE3_VERIFICATION.md](PHASE3_VERIFICATION.md)** - Verification checklist

### Operations & Development
- **[CONTRIBUTING.md](CONTRIBUTING.md)** - How to contribute
- **[DEPLOYMENT_CHECKLIST.md](DEPLOYMENT_CHECKLIST.md)** - Deployment guide
- **[SAFETY.md](SAFETY.md)** - Safety considerations
- **[SECURITY.md](SECURITY.md)** - Security notes

### Evaluation & Datasets
- **[DATASETS.md](DATASETS.md)** - Supported datasets
- **[VERIFICATION_PROOFS.md](VERIFICATION_PROOFS.md)** - Test verification
- **[TEST_EVALUATION_RESULTS.md](TEST_EVALUATION_RESULTS.md)** - Evaluation results

### Maintenance
- **[CHANGELOG.md](CHANGELOG.md)** - Version history
- **[DEPENDENCY_MAINTENANCE.md](DEPENDENCY_MAINTENANCE.md)** - Dependency management

---

## 🎯 Use Case Navigation

### I want to...

**Get started quickly**
→ [README.md](README.md) → [QUICKSTART.md](QUICKSTART.md)

**Understand the VIO system**
→ [IMU_ARCHITECTURE_ANALYSIS.md](IMU_ARCHITECTURE_ANALYSIS.md) → [TIGHT_COUPLING_ROADMAP.md](TIGHT_COUPLING_ROADMAP.md)

**Use batch processing**
→ [BATCH_PROCESSING_GUIDE.md](BATCH_PROCESSING_GUIDE.md) → [OPTIMIZATION_BATCH_SIMD.md](OPTIMIZATION_BATCH_SIMD.md)

**Deploy to production**
→ [DEPLOYMENT_CHECKLIST.md](DEPLOYMENT_CHECKLIST.md) → [SAFETY.md](SAFETY.md) → [SECURITY.md](SECURITY.md)

**Evaluate system performance**
→ [DATASETS.md](DATASETS.md) → [VERIFICATION_PROOFS.md](VERIFICATION_PROOFS.md) → [TEST_EVALUATION_RESULTS.md](TEST_EVALUATION_RESULTS.md)

**Contribute code**
→ [CONTRIBUTING.md](CONTRIBUTING.md) → [IMU_ARCHITECTURE_ANALYSIS.md](IMU_ARCHITECTURE_ANALYSIS.md)

**Track changes**
→ [CHANGELOG.md](CHANGELOG.md)

---

## 📦 Documentation Structure

### Core (7 files)
- Essential architecture and implementation docs
- Directly relevant to current code

### Phase 3 (4 files)
- Batch processing and SIMD optimization
- Current focus of development

### Operations (6 files)
- Deployment, safety, security
- Production concerns

### Evaluation (3 files)
- Testing and verification
- Performance evaluation

### Maintenance (3 files)
- History and dependencies
- Project management

---

## 🗂️ Archived Documentation

Historical and phase-specific documentation (48 files) is available in `.archived/` for reference:

- **Implementation phases**: Phase 2 completion, Phase 2A/2B details
- **Analysis & reviews**: Critical reviews, executive summaries
- **Proof of concept**: Dataset proofs, performance proof, reruns
- **Features**: CPU parallelization, feature detection, build migration
- **Strategies**: Validation plans, benchmark reports
- **Other**: Cleanup records, session summaries

Access archived docs:
```bash
ls .archived/       # List all archived files
cat .archived/FILE  # View specific archived file
```

---

## ✨ Current System

**Architecture**: Tight visual-inertial coupling via bundle adjustment  
**IMU Integration**: Preintegrated measurements with bias feedback  
**Optimization**: Batch processing and SIMD-ready operations  
**Status**: Production-ready, fully tested (100/100 tests passing)  

---

## 🚀 Key Features

- ✅ Real-time visual-inertial odometry
- ✅ High-rate IMU preintegration
- ✅ Bundle adjustment with IMU factors
- ✅ Bias estimation and feedback
- ✅ Batch processing (33% latency improvement)
- ✅ SIMD optimization framework
- ✅ Multi-core parallelization (3.4x scaling)

---

## 📞 Quick Reference

| Need | Document |
|------|----------|
| Setup | [QUICKSTART.md](QUICKSTART.md) |
| Architecture | [IMU_ARCHITECTURE_ANALYSIS.md](IMU_ARCHITECTURE_ANALYSIS.md) |
| API | [BATCH_PROCESSING_GUIDE.md](BATCH_PROCESSING_GUIDE.md) |
| Deployment | [DEPLOYMENT_CHECKLIST.md](DEPLOYMENT_CHECKLIST.md) |
| Testing | [VERIFICATION_PROOFS.md](VERIFICATION_PROOFS.md) |
| Datasets | [DATASETS.md](DATASETS.md) |
| Contributing | [CONTRIBUTING.md](CONTRIBUTING.md) |
| Status | [IMU_STATUS.md](IMU_STATUS.md) |
| Changes | [CHANGELOG.md](CHANGELOG.md) |

---

**Total Docs**: 23 active + 48 archived = 71 total  
**Last Updated**: February 3, 2026  
**Maintained By**: VIO Development Team
