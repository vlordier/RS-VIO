# Hot Path Memory Allocation Review - Summary

## Executive Summary

**Review Date:** 2026-02-06
**Branch:** feature/async-pipeline
**Status:** ✅ Async wrapper fixed, 🔴 Core estimator needs work

### Allocation Budget Analysis

| Component | Before Fix | After Fix | Status |
|-----------|-----------|-----------|--------|
| AsyncEstimator wrapper | ~100 bytes/frame | **0 bytes** ✅ | FIXED |
| Priority queue | Variable (reallocations) | **0 bytes** ✅ | FIXED |
| Telemetry | 0 bytes | **0 bytes** ✅ | GOOD |
| **Estimator core** | **4MB/frame** | **4MB/frame** 🔴 | CRITICAL |
| **Total** | **~4MB** | **~4MB** | **NEEDS WORK** |

## ✅ Issues Fixed

### 1. Backlog Error String Allocation (CRITICAL)
- **Location:** [async_wrapper.rs:611](src/estimator/async_wrapper.rs#L611)
- **Problem:** `anyhow::anyhow!()` with `format!()` allocated String on every dropped frame
- **Fix:** Static error constant `BACKLOG_ERROR`
- **Impact:** **0 bytes** per error (was ~100 bytes)

### 2. BinaryHeap Growth Allocations (HIGH)
- **Location:** [async_wrapper.rs:158](src/estimator/async_wrapper.rs#L158)
- **Problem:** Queue started at capacity 0, reallocated during growth
- **Fix:** `BinaryHeap::with_capacity(max_pending_frames)`
- **Impact:** **0 runtime allocations** (was unpredictable)

**Code Changes:**
```diff
+ const BACKLOG_ERROR: &str = "Frame skipped - backlog limit exceeded";

- let mut queue: BinaryHeap<PrioritizedCommand> = BinaryHeap::new();
+ let capacity = if async_config_clone.max_pending_frames > 0 {
+     async_config_clone.max_pending_frames
+ } else {
+     async_config_clone.channel_capacity
+ };
+ let mut queue: BinaryHeap<PrioritizedCommand> = BinaryHeap::with_capacity(capacity);
```

## 🔴 Critical Issues Identified (NOT FIXED - REQUIRES DESIGN CHANGE)

### Issue #3: Image Buffer Cloning (CRITICAL)
- **Location:** [estimator.rs:168,181](src/estimator/estimator.rs#L168)
- **Problem:** `GrayImage::from_raw()` requires owned `Vec<u8>`, forcing `to_vec()` clone
- **Impact:** **~4MB allocation per frame** (1920x1080 stereo @ 8-bit)
- **Priority:** **P0 - BLOCKS REAL-TIME EMBEDDED DEPLOYMENT**

**Current Code:**
```rust
let left_img = GrayImage::from_raw(img_w, img_h, left_image.to_vec())?;   // 2MB
let right_img = GrayImage::from_raw(img_w, img_h, right_image.to_vec())?; // 2MB
```

**Proposed Solutions:**

#### Option A: Image Buffer Pool (Best for Embedded)
```rust
pub struct Estimator {
    // ... existing fields
    image_buffer_pool: ImageBufferPool,
}

pub struct ImageBufferPool {
    left_buffer: Vec<u8>,
    right_buffer: Vec<u8>,
}

impl ImageBufferPool {
    pub fn new(width: usize, height: usize) -> Self {
        Self {
            left_buffer: vec![0u8; width * height],
            right_buffer: vec![0u8; width * height],
        }
    }

    pub fn load_images(&mut self, left_src: &[u8], right_src: &[u8]) {
        self.left_buffer.copy_from_slice(left_src);
        self.right_buffer.copy_from_slice(right_src);
    }
}
```

#### Option B: Zero-Copy Wrapper (Minimal Changes)
```rust
pub struct BorrowedImage<'a> {
    width: u32,
    height: u32,
    data: &'a [u8],
}

// Modify patch tracker to accept &[u8] instead of GrayImage
```

#### Option C: In-Place Processing (Most Invasive)
```rust
pub fn process_frame(
    &mut self,
    left_image: &[u8],    // Borrow, don't own
    right_image: &[u8],   // Borrow, don't own
    // ...
) -> Result<()>
```

### Issue #4: IMU Data Clone (MEDIUM)
- **Location:** [estimator.rs:206](src/estimator/estimator.rs#L206)
- **Problem:** `imu.to_vec()` clones IMU measurements
- **Impact:** **~500 bytes per frame** (typical 20-50 samples @ 48 bytes each)
- **Fix:** Use pre-allocated buffer or borrow

## Performance Impact Estimates

### Latency Impact (Worst Case)
```
Memory Allocation Latency:
├─ 4MB allocation (malloc):        ~100-500 µs
├─ 4MB copy (memcpy):               ~50-100 µs
├─ Cache pollution:                 ~20-50 µs
└─ GC pressure (if any):            Variable
───────────────────────────────────────────────
TOTAL: ~170-650 µs per frame

30 FPS target: 33.3ms budget per frame
Allocation overhead: 0.5-2% (ACCEPTABLE for desktop)
                     UNACCEPTABLE for embedded (jitter)
```

### Memory Bandwidth Impact
```
At 30 FPS:
- Image copies: 4MB × 30 = 120 MB/s
- IMU copies:   0.5KB × 30 = 15 KB/s
- Total:        ~120 MB/s memory bandwidth

Impact on embedded systems:
- Competes with DMA transfers
- Pollutes L2/L3 cache
- Increases power consumption
```

## Real-Time Safety Certification

### Current Status: ⚠️ PARTIAL

| Criterion | Status | Notes |
|-----------|--------|-------|
| Deterministic execution | 🔴 FAIL | 4MB allocation causes jitter |
| No dynamic allocation in hot path | 🔴 FAIL | Image cloning |
| Bounded memory usage | ✅ PASS | With backlog cap |
| Lock-free telemetry | ✅ PASS | Ring buffers |
| Panic recovery | ✅ PASS | Worker exits cleanly |
| Priority scheduling | ✅ PASS | BinaryHeap with preemption |

### For DO-178C / IEC 61508 Compliance:
- ❌ **FAILS** - Dynamic allocation in safety-critical path
- ✅ Panic recovery mechanism present
- ✅ Bounded resource usage (with fixes)
- ❌ Requires static allocation proof

## Recommendations

### Immediate Actions (This PR)
1. ✅ **DONE:** Fix backlog error allocation
2. ✅ **DONE:** Pre-allocate priority queue
3. 📝 **DOCUMENTED:** Image allocation issue identified

### Next PR (High Priority)
1. 🔴 **CRITICAL:** Implement image buffer pool in Estimator
2. ⚠️ Fix IMU data clone
3. 🔍 Audit patch tracking for allocations
4. 📊 Add dhat profiling to benchmark suite

### Long-Term (Embedded Deployment)
1. Profile with dhat on target hardware
2. Implement zero-copy image processing chain
3. Consider arena allocators for temporary buffers
4. Add real-time jitter tests to CI
5. Document memory layout and allocation strategy

## Test Results

**All tests passing:** ✅ 212 library tests + 36 integration tests
**Pre-commit hooks:** ✅ All passed (rustfmt, clippy, shellcheck)
**Allocations verified:** Manual code review (dhat profiling recommended)

## Files Modified

- `src/estimator/async_wrapper.rs` - Fixed backlog error and queue allocation
- `HOTPATH_ALLOCATION_ANALYSIS.md` - Detailed technical analysis
- `HOTPATH_REVIEW_SUMMARY.md` - This document

## Conclusion

**AsyncEstimator is now allocation-free in its hot path** ✅, but the **underlying Estimator::process_frame still allocates ~4MB per frame** 🔴. This makes the system **unsuitable for hard real-time embedded deployment** without further optimization.

**Recommendation:** Proceed with async pipeline merge, but **mark image allocation fix as P0 issue** for next sprint before embedded deployment.
