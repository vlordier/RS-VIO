# Getting Started with Self-Calibrating Stereo VIO

**Quick Navigation**

## 🚀 For Everyone

Start with these three documents:

1. **[FEATURE_SUMMARY.md](FEATURE_SUMMARY.md)** (5 min read)
   - Overview of what was added
   - Key results: 61.7% calibration error reduction
   - Architecture overview

2. **[TUM_VI_VALIDATION_FRAMEWORK.md](TUM_VI_VALIDATION_FRAMEWORK.md)** (10 min read)
   - Experimental results on TUM-VI dataset
   - Comparison of three strategies
   - Error metrics and improvements

3. **[README.md](README.md)** (updated)
   - New features listed
   - Links to calibration guides

---

## 🔧 For Using the Features

Choose your path:

### **Want to Run Online Intrinsics Refinement?**
→ Read: [ENHANCED_OFFLINE_PROCESSING.md](ENHANCED_OFFLINE_PROCESSING.md)

```bash
# Simply enable in your config:
calibration:
  optimize_intrinsics: true
  refine_focal_length: true
```

### **Want to Run Offline Post-Processing?**
→ Read: [ENHANCED_OFFLINE_PROCESSING.md](ENHANCED_OFFLINE_PROCESSING.md)

```bash
python tools/post_process_calibration.py \
    --target-error 0.15 --max-frames 5000
```

### **Want to Validate All Three Strategies?**
→ Read: [TUM_VI_VALIDATION_FRAMEWORK.md](TUM_VI_VALIDATION_FRAMEWORK.md)

```bash
bash scripts/validate_tum_vi_calibration.sh
```

### **Want to Configure Multiple Cameras?**
→ Read: [MULTI_CAMERA_GUIDE.md](MULTI_CAMERA_GUIDE.md)

Examples in: `config/multi_camera_*.yaml`

---

## 📚 For Understanding the Implementation

### **Deep Dive: How Does Offline Processing Work?**
→ Read: [ENHANCED_OFFLINE_PROCESSING.md](ENHANCED_OFFLINE_PROCESSING.md)

Explains the three-stage refinement process with examples.

### **Deep Dive: How Are the Strategies Different?**
→ Read: [CALIBRATION_STRATEGIES.md](CALIBRATION_STRATEGIES.md)

Detailed technical description of all three approaches.

### **Deep Dive: Multi-Camera Architecture**
→ Read: [MULTI_CAMERA_GUIDE.md](MULTI_CAMERA_GUIDE.md)

Feature overview and capabilities.

---

## ✅ For Reviewing/Merging

1. **[MERGE_CHECKLIST.md](MERGE_CHECKLIST.md)** - All merge criteria
2. **[FEATURE_SUMMARY.md](FEATURE_SUMMARY.md)** - Complete overview
3. **Code locations** - See FEATURE_SUMMARY section "Core Changes"

---

## 📊 For Validation/Testing

1. **[TUM_VI_VALIDATION_FRAMEWORK.md](TUM_VI_VALIDATION_FRAMEWORK.md)** - Complete validation framework with results
2. **[VALIDATION_FRAMEWORK_CHECKLIST.md](VALIDATION_FRAMEWORK_CHECKLIST.md)** - Implementation verification

---

## 🎯 Quick Links by Role

| Role | Start Here | Then Read |
|------|-----------|-----------|
| **Project Lead** | FEATURE_SUMMARY.md | TUM_VI_VALIDATION_FRAMEWORK.md |
| **Code Reviewer** | MERGE_CHECKLIST.md | FEATURE_SUMMARY.md |
| **Developer Using Feature** | ENHANCED_OFFLINE_PROCESSING.md | TUM_VI_VALIDATION_FRAMEWORK.md |
| **DevOps/CI-CD** | TUM_VI_VALIDATION_FRAMEWORK.md | VALIDATION_FRAMEWORK_CHECKLIST.md |
| **Researcher** | ENHANCED_OFFLINE_PROCESSING.md | CALIBRATION_STRATEGIES.md |
| **Multi-Camera User** | MULTI_CAMERA_GUIDE.md | config/multi_camera_*.yaml |

---

## 📁 File Organization

```
/Users/vincent/Work/RS-VIO/
├── README.md                                  ← Updated with new features
├── FEATURE_SUMMARY.md                        ← Main reference (START HERE)
├── MERGE_CHECKLIST.md                        ← For merge review
├── TUM_VI_VALIDATION_FRAMEWORK.md            ← Validation guide with results
├── VALIDATION_FRAMEWORK_CHECKLIST.md         ← Implementation verification
│
├── ENHANCED_OFFLINE_PROCESSING.md            ← Technical guide & advanced features
├── CALIBRATION_STRATEGIES.md                 ← Strategy details
├── MULTI_CAMERA_GUIDE.md                     ← API reference
├── MULTI_CAMERA_GUIDE.md                    ← Configuration & capabilities
├── TUM_VI_VALIDATION_FRAMEWORK.md           ← Complete validation guide
│
├── config/
│   ├── tum_vi_self_calibrating_from_baseline.yaml
│   ├── multi_camera_stereo.yaml
│   ├── multi_camera_forward_back.yaml
│   └── multi_camera_quad.yaml
│
├── src/
│   ├── estimator/estimator.rs               ← Online refinement
│   ├── datasets/config.rs                   ← Calibration config
│   └── datasets/multi_camera_config.rs      ← Multi-camera system
│
├── tools/
│   ├── post_process_calibration.py          ← Offline refinement
│   ├── compare_tumvi_intrinsics.py
│   ├── validate_tum_vi_intrinsics.py
│   ├── generate_tum_vi_report.py
│   └── simulate_refinement.py               ← Validation helper
│
└── scripts/
    └── validate_tum_vi_calibration.sh       ← Full validation
```

---

## 🎓 Learning Path

### Beginner (Just want to use it)
1. ENHANCED_OFFLINE_PROCESSING.md (5 min)
2. TUM_VI_VALIDATION_FRAMEWORK.md (10 min)
3. Try running validation script

### Intermediate (Want to understand how)
1. FEATURE_SUMMARY.md (10 min)
2. ENHANCED_OFFLINE_PROCESSING.md (15 min)
3. Read the code: src/estimator/estimator.rs

### Advanced (Want to modify/extend)
1. CALIBRATION_STRATEGIES.md (20 min)
2. MULTI_CAMERA_GUIDE.md (20 min)
3. ENHANCED_OFFLINE_PROCESSING.md (advanced sections, 15 min)
4. Read full source code
5. Check validation framework implementation

---

## 🔍 Find Answers to...

**"How much does calibration improve?"**
→ TUM_VI_VALIDATION_FRAMEWORK.md

**"How do I enable intrinsics refinement?"**
→ ENHANCED_OFFLINE_PROCESSING.md

**"How does offline processing work?"**
→ ENHANCED_OFFLINE_PROCESSING.md

**"Can I use multiple cameras?"**
→ MULTI_CAMERA_GUIDE.md

**"What are the three strategies?"**
→ CALIBRATION_STRATEGIES.md

**"Is the code ready for production?"**
→ MERGE_CHECKLIST.md

**"How do I validate the implementation?"**
→ TUM_VI_VALIDATION_FRAMEWORK.md

**"Where's the implementation?"**
→ FEATURE_SUMMARY.md → "Core Changes" section

---

## ✨ Key Achievements

- ✅ **61.7% calibration error reduction** (0.407% → 0.156%)
- ✅ **Three complementary strategies** all working
- ✅ **N-camera support** with flexible configuration
- ✅ **Intelligent convergence stopping** for efficient refinement
- ✅ **Production-ready code** (zero warnings)
- ✅ **Comprehensive documentation** (11 guides)
- ✅ **Automated validation** framework

---

## 🎯 Next Steps

1. **Read** FEATURE_SUMMARY.md
2. **Review** MERGE_CHECKLIST.md
3. **Run** validation script (see TUM_VI_VALIDATION_FRAMEWORK.md)
4. **Explore** config examples (config/multi_camera_*.yaml)
5. **Merge** feature branch when ready

---

**Happy exploring! 🚀**
