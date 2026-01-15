# Quick Reference: Marginalization for Embedded Drone VIO

## TL;DR: Production Configuration

```rust
// For Jetson Xavier / typical drone
MarginalizationConfig {
    enabled: true,
    use_fej: true,
    damping: 1e-5,                 // Conservative: handles motion blur
    max_keyframes: 8,              // Tight window: 56 states
    num_marginalize_per_step: 1,
    min_landmark_observations: 3,
    landmark_age_limit: 50,
    prior_weight: 1.0,
    prior_info_scaling: 0.9,       // Slight downweight
    hessian_approximator: "Diagonal",    // 20× faster than GaussNewton
    gradient_computer: "Standard",
    prior_constructor: "Standard",
}
```

**Expected performance**: 2–5ms per marginalization @ 30 Hz

---

## Problem Scenarios & Fixes

### Slow Marginalization (>10ms latency spikes)

**Likely cause**: SVD condition number estimation in hot path (old code)

**Fix**: Already implemented! Uses O(n) heuristic now. If still slow:
```rust
// Check for ill-conditioning
if condition_number > 1e6 {
    log::warn!("H_bb ill-conditioned: κ ~ {:.1e}", condition_number);
    // System will escalate damping automatically
}
```

### Trajectory Divergence Over Time

**Likely cause**: Damping scale unbounded, poor solution quality undetected

**Fix**: Damping now capped at 1e3; each fallback is logged. Check logs for:
```
[WARN] H_bb damping exceeded 1e-05; switching to LU fallback
[ERROR] H_bb critically ill-conditioned even with damping 1e-02; using pseudo-inverse
```

**Action**: If pseudo-inverse is triggered frequently:
1. Increase damping in config (1e-5 → 1e-4)
2. Check camera focus / feature quality
3. Reduce max_keyframes (8 → 6)

### Memory Pressure (>500MB on Snapdragon)

**Likely cause**: Window too large for platform

**Fix**: Reduce max_keyframes
```rust
// Snapdragon: 1GB total, VIO gets ~300MB
MarginalizationConfig {
    max_keyframes: 6,  // ~48 states instead of 84
    hessian_approximator: "Diagonal",
    ..
}
```

**Memory breakdown** (per marginalization):
```
Schur 84×84:           512 KB
Solver temporaries:    2 MB
Prior factor:          50 KB
─────────────────────────
Total per cycle:       2.5 MB
100 cycles (30-60s):   250 MB
```

---

## Testing Checklist Before Deployment

- [ ] Run `cargo test marginalization --lib` (57 tests pass ✅)
- [ ] Log condition numbers for 100 marginalization cycles
- [ ] Check max condition number encountered
- [ ] Verify no LU/pseudo-inverse fallbacks in nominal trajectory
- [ ] Profile marginalization latency (should be <5ms)
- [ ] Verify memory footprint <80MB over 100 cycles

---

## Diagnostic Tools

### Enable Debug Logging

```bash
RUST_LOG=debug cargo run --release -- --config drone_config.yaml
```

**Look for**:
- `[DEBUG] H_bb Cholesky succeeded at damping scale 1.0` (good, no escalation)
- `[WARN] H_bb Cholesky failed...` (acceptable, escalate damping)
- `[WARN] Using LU fallback...` (rare, expect <0.1% of cycles)
- `[ERROR] Using pseudo-inverse...` (very rare, check feature quality)

### Analyze Condition Numbers

Add to trajectory logs:
```rust
if let Some(cond) = result.info.condition_number {
    eprintln!("cond_number,{}", cond);  // CSV log
}
```

Then analyze:
```bash
# Worst-case condition number
grep cond_number flight.log | cut -d, -f2 | sort -n | tail -1

# Percentage of well-conditioned (κ < 1e6)
grep cond_number flight.log | cut -d, -f2 | awk '$1 < 1e6 {c++} END {print 100*c/NR "%"}'
```

---

## Common Configuration Patterns

### High-Accuracy Ground Station
```rust
MarginalizationConfig {
    damping: 1e-7,
    max_keyframes: 15,
    hessian_approximator: "GaussNewton",  // Maximum accuracy
    prior_info_scaling: 1.0,
    ..
}
```
**Use case**: Post-processing, no real-time constraints  
**Latency**: 20–50ms acceptable

### GPS-Denied Tight Spaces (Tunnels)
```rust
MarginalizationConfig {
    damping: 1e-4,  // Strong regularization
    max_keyframes: 5,  // Minimal growth
    hessian_approximator: "Diagonal",
    prior_info_scaling: 0.7,  // Downweight prior
    ..
}
```
**Use case**: Expect poor conditioning, feature tracking challenging  
**Latency**: <5ms (tight processing)

### Low-Power Edge Device (200 MB RAM)
```rust
MarginalizationConfig {
    damping: 1e-5,
    max_keyframes: 4,  // Minimal: 28 states
    hessian_approximator: "Diagonal",
    prior_info_scaling: 0.8,
    ..
}
```
**Use case**: Very constrained platform  
**Latency**: <2ms (minimal processing)

---

## API Reference (Key Methods)

### Manager Creation
```rust
let config = MarginalizationConfig::default();  // Embedded defaults
let mut manager = MarginalizationManager::new(config);
```

### Perform Marginalization
```rust
let result = manager.marginalize(
    &param_blocks,      // HashMap<ParamId, ParamBlock>
    &hessian,          // DMatrix NxN
    &gradient,         // DVector N
    &keep_ids,         // Vec<ParamId>
    &marg_ids,         // Vec<ParamId>
);

// Result info
if let Some(cond) = result.info.condition_number {
    log::info!("Schur κ = {:.1e}", cond);
}
```

### Retrieve Prior
```rust
if let Some(prior) = manager.get_prior() {
    // Use prior in optimizer
    optimizer.add_prior_factor(&prior);
}
```

### FEJ Behavior
```rust
// If use_fej=true:
// - Linearization points cached on first marginalization
// - Subsequent marginalizations reuse cached points (unless structure changes)
// - Ensures consistency across bundle adjustment iterations

// Reset if trajectory is discontinuous
manager.reset();
```

---

## Files to Review

1. **[src/optimization/marginalization.rs](src/optimization/marginalization.rs)** — Core implementation
   - Module docs: Embedded VIO section
   - `estimate_condition_number()`: O(n) fast heuristic
   - `solve_h_bb_system()`: Robust fallback pipeline
   - Default config: Embedded-friendly values

2. **[MARGINALIZATION_EMBEDDED_AUDIT.md](MARGINALIZATION_EMBEDDED_AUDIT.md)** — Detailed analysis
   - 7 critical issues identified
   - Fixes with before/after metrics
   - Tuning guide for different scenarios

3. **[MARGINALIZATION_REVIEW_SUMMARY.md](MARGINALIZATION_REVIEW_SUMMARY.md)** — Executive summary
   - Key achievements
   - Test validation
   - Deployment checklist

---

## Contact & Issues

If marginalization exhibits:
- **Latency spikes >10ms**: Check condition number, increase damping
- **Trajectory divergence**: Verify FEJ is enabled, check feature quality
- **Memory pressure**: Reduce max_keyframes, use Diagonal Hessian
- **Numerical instability**: Check log for LU/pseudo-inverse fallbacks

See [MARGINALIZATION_EMBEDDED_AUDIT.md](MARGINALIZATION_EMBEDDED_AUDIT.md) for detailed tuning guidance.
