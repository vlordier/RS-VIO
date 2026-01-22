# 📚 RS-VIO Documentation Index

**Last Updated**: January 22, 2026 (Historical cleanup and archive indexing complete)  
**Status**: 33 active documents | 88 archived | Production-ready, lean & focused

---

## 📋 Current Implementation Status

✅ **All core visual-inertial odometry features implemented and production-ready**

**Ready for deployment**: Full VIO pipeline with IMU integration, geometric verification, vibration filtering, rolling shutter compensation, and loop closure.

**What's left to do**: See [REMAINING_WORK.md](REMAINING_WORK.md) for optional features and [TODO_INVENTORY.md](TODO_INVENTORY.md) for detailed implementation tracking.

**Understanding the archive**: 
- [.archive/INDEX.md](.archive/INDEX.md) - Navigation guide to archived documentation
- [HISTORICAL_INSIGHTS.md](HISTORICAL_INSIGHTS.md) - Key learnings from development history

---

## 🚀 Quick Start (New Users)

| Document | Time | Purpose |
|----------|------|---------|
| [README.md](README.md) | 5 min | Project overview & features |
| [QUICKSTART.md](QUICKSTART.md) | 10 min | Getting started guide |
| [CONFIGURATION_GUIDE.md](CONFIGURATION_GUIDE.md) | 15 min | Configure the VIO pipeline |
| [DATASETS.md](DATASETS.md) | 5 min | Dataset setup instructions |

---

## 🏗️ Architecture & Design

| Document | Purpose |
|----------|---------|
| **For detailed architecture**: See [source code](src/) and `cargo doc --open` |
| **For implementation status**: See [REMAINING_WORK.md](REMAINING_WORK.md) |

> **Note**: For historical architecture information, see [.archive/historical/](../.archive/historical/)

---

## 🔧 Active Features & Implementation

### IMU Denoising (✅ Complete & Integrated)
Start with: [IMU_DENOISING_INDEX.md](IMU_DENOISING_INDEX.md)

| Document | Purpose |
|----------|---------|
| [INTEGRATION_GUIDE.md](INTEGRATION_GUIDE.md) | Integration instructions with examples |

### Fusion Framework (✅ Complete & Integrated)
Start with: [FUSION_DOCUMENTATION_INDEX.md](FUSION_DOCUMENTATION_INDEX.md)

| Document | Purpose |
|----------|---------|
| [FUSION_QUICKSTART.md](FUSION_QUICKSTART.md) | Quick start guide |
| [FUSION_ARCHITECTURE.md](FUSION_ARCHITECTURE.md) | Architecture overview |
| [FUSION_BEST_PRACTICES.md](FUSION_BEST_PRACTICES.md) | Best practices & patterns |
| [FUSION_CONFIG_QUICKREF.md](FUSION_CONFIG_QUICKREF.md) | Configuration quick reference |

### Calibration (✅ Complete & Integrated)
See [.archive/completed-phases/](../.archive/completed-phases/) for calibration implementation details

### Stereo Matching (✅ Implemented)
| Document | Purpose |
|----------|---------|
| [STEREO_SUPER_RESOLUTION_QUICKREF.md](STEREO_SUPER_RESOLUTION_QUICKREF.md) | Super-resolution quick reference |

See [.archive/completed-phases/](../.archive/completed-phases/) for stereo matching implementation details

### Higher-Order Filtering (✅ Implemented)
| Document | Purpose |
|----------|---------|
| [HIGHER_ORDER_QUICK_REFERENCE.md](HIGHER_ORDER_QUICK_REFERENCE.md) | Quick reference |
| [HIGHER_ORDER_STATUS_REPORT.md](HIGHER_ORDER_STATUS_REPORT.md) | Current status |

See [.archive/](../.archive/) for detailed implementation and architecture documentation

### Other Features
| Document | Purpose |
|----------|---------|
| [IMU_MOTOR_STATE_DETECTION.md](IMU_MOTOR_STATE_DETECTION.md) | Motor state detection |

**Complete Implementation Details** archived in [.archive/](../.archive/):
- Calibration framework & metrics
- Stereo matching & super-resolution
- Higher-order filtering
- Configurable pipeline & performance optimizations
- Benchmark results & technical references

---

## 📖 Quick References

These cheat sheets provide quick access to common information:

| Document | Use For |
|----------|---------|
| [QUICK_REFERENCE.md](QUICK_REFERENCE.md) | General quick reference |
| [IMU_QUICK_REFERENCE.md](IMU_QUICK_REFERENCE.md) | IMU-specific quick reference |
| [FUSION_QUICK_REFERENCE.md](FUSION_QUICK_REFERENCE.md) | Fusion module quick reference |
| [MARGINALIZATION_QUICK_REFERENCE.md](MARGINALIZATION_QUICK_REFERENCE.md) | Marginalization reference |
| [HIGHER_ORDER_QUICK_REFERENCE.md](HIGHER_ORDER_QUICK_REFERENCE.md) | Higher-order filtering reference |
| [SCREENSHOT_QUICK_REFERENCE.md](SCREENSHOT_QUICK_REFERENCE.md) | Screenshot visualization reference |
| [STEREO_SUPER_RESOLUTION_QUICKREF.md](STEREO_SUPER_RESOLUTION_QUICKREF.md) | SSR quick reference |

---

## 🎨 Visualization & Debugging

| Document | Purpose |
|----------|---------|
| [VISUALIZATION_GUIDE.md](VISUALIZATION_GUIDE.md) | Complete visualization guide |
| [VISUALIZATION_QUICKSTART.md](VISUALIZATION_QUICKSTART.md) | Quick start for visualization |
| [IMU_VISUALIZATION_GUIDE.md](IMU_VISUALIZATION_GUIDE.md) | IMU visualization specific |
| [IMU_VISUALIZATION_IMPLEMENTATION.md](IMU_VISUALIZATION_IMPLEMENTATION.md) | Implementation details |

---

## 📊 Benchmarking & Performance

| Document | Purpose |
|----------|---------|
| [BENCHMARKING.md](BENCHMARKING.md) | Complete benchmarking guide |
| [FUSION_BENCHMARK_RESULTS.md](FUSION_BENCHMARK_RESULTS.md) | Fusion benchmark results |
| [REALTIME_BENCHMARK_RESULTS.md](REALTIME_BENCHMARK_RESULTS.md) | Real-time benchmark results |
| [PERFORMANCE_OPTIMIZATIONS.md](PERFORMANCE_OPTIMIZATIONS.md) | Performance optimization strategies |

---

## 🔐 Safety & Quality

| Document | Purpose |
|----------|---------|
| [README.md](README.md#Safety--Embedded-Systems) | Safety & build profiles section |
| [SECURITY.md](SECURITY.md) | Security considerations |
| [SAFETY.md](SAFETY.md) | Safety considerations |

---

## 🛠️ Development & Integration

| Document | Purpose |
|----------|---------|
| [CONTRIBUTING.md](CONTRIBUTING.md) | How to contribute |
| [CHANGELOG.md](CHANGELOG.md) | Project changelog |

---

## 📦 Test Improvements

| Document | Purpose |
|----------|---------|
| [IMU_TEST_IMPROVEMENTS.md](IMU_TEST_IMPROVEMENTS.md) | IMU testing improvements |

---

## 🗂️ Archived Documentation

**For historical context**, see [.archive/README.md](.archive/README.md)

The archive contains:
- **Completed Phases** (.archive/completed-phases/) - Phase 4-9 documentation
- **Validation Reports** (.archive/validation-reports/) - Integration & test results
- **Optimization Reports** (.archive/optimization-reports/) - Performance analysis
- **Session Reports** (.archive/session-reports/) - Historical work summaries
- **Roadmaps** (.archive/roadmaps/) - Future planning documents
- **Historical** (.archive/historical/) - Older implementation details

---

## 📊 Documentation Statistics

| Category | Count | Status |
|----------|-------|--------|
| **Quick Start** | 4 | ✅ Current |
| **Core Documentation** | 8 | ✅ Current |
| **Feature Guides** | 20+ | ✅ Current |
| **Quick References** | 7 | ✅ Current |
| **Visualization** | 4 | ✅ Current |
| **Performance** | 4 | ✅ Current |
| **Archived** | 84 | 📦 Historical |

---

## 🎯 Common Tasks

### "I'm new to RS-VIO"
1. Read [README.md](README.md)
2. Follow [QUICKSTART.md](QUICKSTART.md)
3. Explore [CONFIGURATION_GUIDE.md](CONFIGURATION_GUIDE.md)

### "I want to understand the architecture"
1. Start with [ARCHITECTURE.md](ARCHITECTURE.md)
2. Read [EXECUTION_GUIDE.md](EXECUTION_GUIDE.md)
3. Check specific feature docs above

### "I'm implementing a feature"
1. Find the feature in the list above
2. Read the implementation summary
3. Check INTEGRATION_GUIDE for integration steps
4. Review BENCHMARKING.md to verify performance

### "I'm optimizing performance"
1. Review [PERFORMANCE_OPTIMIZATIONS.md](PERFORMANCE_OPTIMIZATIONS.md)
2. Check [BENCHMARKING.md](BENCHMARKING.md)
3. Profile using visualization tools in [VISUALIZATION_GUIDE.md](VISUALIZATION_GUIDE.md)

### "I want historical context"
- See [.archive/README.md](.archive/README.md)
- Browse the appropriate archive subdirectory

---

## 📝 Notes

- **ARCHITECTURE.md** has a warning about some sections being outdated; use with source code
- Most feature documentation is current and production-ready
- Quick references are excellent starting points for each feature
- Archive maintains all historical context without cluttering active development

