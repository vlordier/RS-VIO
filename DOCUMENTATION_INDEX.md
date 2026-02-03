# RS-VIO Documentation Index

**Status**: Phase 3 - Batch Processing & SIMD ✅  
**Active Docs**: 12 essential files (current code only)

---

## 📖 Documentation

### Getting Started
- **[README.md](README.md)** - Overview & setup
- **[QUICKSTART.md](QUICKSTART.md)** - First run

### Code Documentation
- **[IMU_README.md](IMU_README.md)** - IMU module guide
- **[OPTIMIZATION_BATCH_SIMD.md](OPTIMIZATION_BATCH_SIMD.md)** - Batch processing & SIMD implementation

### Operations
- **[CONTRIBUTING.md](CONTRIBUTING.md)** - Contribution guidelines
- **[DEPLOYMENT_CHECKLIST.md](DEPLOYMENT_CHECKLIST.md)** - Deployment steps
- **[CHANGELOG.md](CHANGELOG.md)** - Version history
- **[DEPENDENCY_MAINTENANCE.md](DEPENDENCY_MAINTENANCE.md)** - Dependency updates

### Critical Information
- **[SAFETY.md](SAFETY.md)** - Safety considerations
- **[SECURITY.md](SECURITY.md)** - Security notes
- **[DATASETS.md](DATASETS.md)** - Supported datasets

---

## 🎯 Quick Navigation

**New to the project?** → [README.md](README.md) → [QUICKSTART.md](QUICKSTART.md)

**Developing code?** → [CONTRIBUTING.md](CONTRIBUTING.md) → [IMU_README.md](IMU_README.md) → [OPTIMIZATION_BATCH_SIMD.md](OPTIMIZATION_BATCH_SIMD.md)

**Deploying?** → [DEPLOYMENT_CHECKLIST.md](DEPLOYMENT_CHECKLIST.md) → [SAFETY.md](SAFETY.md) → [SECURITY.md](SECURITY.md)

**Using datasets?** → [DATASETS.md](DATASETS.md)

---

## 📦 What's Archived?

Historical documentation (Phase 2, Phase 3 completion markers, evaluations) is in `.archived/` for reference. Active docs focus on current code implementation.

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
