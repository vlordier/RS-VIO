# Tracing Logging Quick Reference

## TL;DR

Replace env_logger with `tracing` + non-blocking I/O for real-time systems. Use atomic counters in hot loops.

## One-Minute Setup

```rust
// In main.rs
use rs_vio::init_tracing_logging;

fn main() -> anyhow::Result<()> {
    // Non-blocking rolling logs to ./logs/vio.log.*
    let _guard = init_tracing_logging("./logs", "vio")?;
    
    tracing::info!("VIO starting");
    run_vio();
    Ok(())
}
```

## Hot Loop (Never Block)

```rust
use rs_vio::TelemetryCounters;
use std::sync::Arc;

let counters = Arc::new(TelemetryCounters::new());

for frame in frames {
    // These are ~10 ns each, NEVER block
    counters.record_frame();
    counters.record_features_detected(256);
    counters.record_imu_measurement();
    
    process_frame(frame);  // 33 ms @ 30 fps
}
```

## Telemetry Thread (Can Log)

```rust
let counters_clone = Arc::clone(&counters);

std::thread::spawn(move || {
    loop {
        std::thread::sleep(Duration::from_secs(1));  // 1 Hz
        
        let report = counters_clone.swap_and_report();
        
        // This CAN block on I/O, but only happens every 1 second
        tracing::info!(
            frames = report.frames,
            features = report.features_tracked,
            imu = report.imu_measurements,
            "telemetry"
        );
    }
});
```

## Control Log Level

```bash
# Default (info)
cargo run --release

# Debug (rs_vio crate only)
RUST_LOG=rs_vio=debug cargo run --release

# Verbose (everything)
RUST_LOG=trace cargo run --release

# Multiple crates
RUST_LOG=rs_vio=debug,rerun=warn cargo run --release
```

## TelemetryCounters API

```rust
let counters = TelemetryCounters::new();

// Record events (from hot loop - never blocks)
counters.record_frame();
counters.record_dropped_frame();
counters.record_features_detected(count);
counters.record_features_tracked(count);
counters.record_imu_measurement();
counters.record_loop_closure();

// Reset & get report (from telemetry thread - can block)
let report = counters.swap_and_report();

// Report fields
report.frames                  // u64
report.frames_dropped          // u64
report.features_detected       // u64
report.features_tracked        // u64
report.imu_measurements        // u64
report.loop_closures           // u64

// Display
println!("{}", report);  // Pretty-printed report
```

## Tracing Spans (Optional)

```rust
use tracing::instrument;

// Automatic span around function
#[instrument(skip(imu))]
fn integrate_imu(imu: &ImuBuffer) {
    // Logged as: integerate_imu{imu_count=42} event
    tracing::debug!("processing IMU");
}

// Manual spans
let span = tracing::info_span!("vio_step", frame_id = 42);
let _guard = span.enter();

process_frame();  // Logged with frame_id context
```

## Log Output Example

```
2026-02-05T14:02:15.593960Z  INFO main ThreadId(01) main.rs:21: VIO starting
2026-02-05T14:02:50.552511Z  INFO ThreadId(04) main.rs:100: telemetry frames=31 features_tracked=2640 imu=28
```

## Performance Rules

| Context | Action |
|---------|--------|
| Hot loop | Use `counters.record_*()` (atomics) |
| Telemetry thread | Use `tracing::info!()` (can block) |
| Frequency | 1-5 Hz for telemetry |
| Overhead | < 100 ns per frame in hot loop |

## Files

- **Guide**: [TRACING_LOGGING.md](TRACING_LOGGING.md)
- **Example**: [examples/tracing_demo.rs](examples/tracing_demo.rs)
- **Code**: [src/logging/tracing_config.rs](src/logging/tracing_config.rs)

## Why Not Log in Hot Loop?

```rust
// ❌ BAD: Can block 10-1000+ microseconds
for frame in frames {
    log::info!("Processing {}", frame);  // Stalls!
    process_frame(frame);  // 33 ms goal becomes 50+ ms
}

// ✅ GOOD: Never blocks, ~10 ns
for frame in frames {
    counters.record_frame();  // Atomic, returns immediately
    process_frame(frame);  // Always ~33 ms
}

// Then report separately at 1 Hz (1000 ms later)
info!("FPS: {}", frames_per_second);  // Can block, but only once per second
```

## Integration Checklist

- [ ] Add `let _guard = init_tracing_logging(...)?;` to main
- [ ] Create `TelemetryCounters` in estimator
- [ ] Replace log calls in hot loop with `counters.record_*()` calls
- [ ] Create telemetry thread that calls `swap_and_report()` at 1 Hz
- [ ] Test with `RUST_LOG=debug cargo run`
- [ ] Check logs in `./logs/` directory
- [ ] Verify no blocking in hot loop (check frame timing)

## See Also

- Full docs: [TRACING_LOGGING.md](TRACING_LOGGING.md)
- Custom metrics: [REALTIME_LOGGING.md](REALTIME_LOGGING.md)
- Crate docs: `cargo doc --open` → search "logging"

---

**For questions**: Check examples/tracing_demo.rs for complete working code.
