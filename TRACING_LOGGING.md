# Production-Grade Logging with Tracing

## Overview

Enhanced the RS-VIO logging system with `tracing` + non-blocking I/O for real-time systems. This provides:

- **Structured logging** with spans and structured fields
- **Non-blocking I/O** (hot loops never stall on file writes)
- **Lock-free telemetry** via atomic counters
- **Runtime log level control** via `RUST_LOG` environment variable
- **Rolling file appenders** for automatic log rotation
- **Zero-overhead** trace/debug in release builds

## Why Tracing for Real-Time?

### Problem with Traditional Logging
```rust
// BAD: Blocks hot loop if disk is slow or buffer full
for frame in frames {
    log::info!("Processing frame {}", frame);  // Can block!
    process_frame(frame);
}
```

### Solution: Tracing + Atomic Counters
```rust
// GOOD: No logging in hot loop, non-blocking telemetry
static FRAMES: AtomicU64 = AtomicU64::new(0);

for frame in frames {
    FRAMES.fetch_add(1, Ordering::Relaxed);  // ~10 ns, never blocks
    process_frame(frame);
}

// Periodically (1-5 Hz) from low-priority thread:
let count = FRAMES.swap(0, Ordering::Relaxed);
info!("FPS: {}", count);  // This CAN block, but only at 1 Hz
```

## Quick Start

### Initialize Logging

```rust
use rs_vio::init_tracing_logging;

fn main() -> anyhow::Result<()> {
    // Non-blocking rolling daily logs in ./logs/vio.log.*
    let _guard = init_tracing_logging("./logs", "vio")?;
    
    tracing::info!("VIO starting");
    Ok(())
}
```

### Control Log Levels at Runtime

```bash
# Default (info level)
cargo run --release --bin run_euroc

# Debug only for rs_vio crate
RUST_LOG=rs_vio=debug cargo run --release

# Trace everything (verbose)
RUST_LOG=trace cargo run --release

# Multiple modules
RUST_LOG=rs_vio=debug,rerun=warn cargo run --release
```

### Hot-Loop Telemetry (Zero Blocking)

```rust
use rs_vio::TelemetryCounters;
use std::sync::Arc;

let counters = Arc::new(TelemetryCounters::new());
let counters_clone = Arc::clone(&counters);

// VIO thread (hot loop - NEVER logs)
std::thread::spawn(move || {
    loop {
        counters_clone.record_frame();
        counters_clone.record_features_detected(256);
        counters_clone.record_imu_measurement();
        process_frame();  // ~33 ms for 30 fps
    }
});

// Telemetry thread (1-5 Hz - CAN log)
std::thread::spawn(move || {
    loop {
        std::thread::sleep(Duration::from_secs(1));
        
        let report = counters.swap_and_report();
        tracing::info!(
            frames = report.frames,
            features = report.features_detected,
            "telemetry"
        );
    }
});
```

## API Reference

### `init_tracing_logging`

```rust
pub fn init_tracing_logging(
    log_dir: &str,
    app_name: &str,
) -> Result<LoggingGuard>
```

Initialize tracing with:
- **Non-blocking** writes (uses background worker thread)
- **Rolling daily** log files (`app_name.log.YYYY-MM-DD`)
- **Compact format** with timestamps, thread IDs, line numbers
- **Auto file creation** if directory doesn't exist

Returns `LoggingGuard` which must be kept alive for the program duration.

### `TelemetryCounters`

Lock-free atomic counter structure for hot-loop instrumentation:

```rust
pub struct TelemetryCounters {
    pub frames: AtomicU64,              // Processed frames
    pub frames_dropped: AtomicU64,       // Failed frames
    pub features_detected: AtomicU64,    // Features found
    pub features_tracked: AtomicU64,     // Features tracked
    pub imu_measurements: AtomicU64,     // IMU samples processed
    pub loop_closures: AtomicU64,        // Loop closure count
}
```

Methods (all `Ordering::Relaxed` for performance):
- `record_frame()` - Increment frame counter
- `record_dropped_frame()` - Log dropped frame
- `record_features_detected(count)` - Add feature count
- `record_features_tracked(count)` - Add tracked features
- `record_imu_measurement()` - Increment IMU counter
- `record_loop_closure()` - Increment closure count
- `swap_and_report()` - Get and reset all counters

### `TelemetryReport`

Result from `swap_and_report()`:

```rust
pub struct TelemetryReport {
    pub frames: u64,
    pub frames_dropped: u64,
    pub features_detected: u64,
    pub features_tracked: u64,
    pub imu_measurements: u64,
    pub loop_closures: u64,
}

// Display impl for easy logging:
println!("{}", report);
```

## Architecture

### Non-Blocking I/O Pattern

```
┌─────────────────────┐
│  Hot Loop Thread    │
│  - Process frames   │
│  - NO logging calls │
│  - Atomic increments│
└──────────┬──────────┘
           │ Relaxed atomic ops
           │ (no blocking)
           ▼
┌──────────────────────────┐
│  Atomic Counters         │
│  (shared memory)         │
└──────────┬───────────────┘
           │
           ▲ Read & reset
           │ (1-5 Hz)
┌──────────┴──────────┐
│ Telemetry Thread    │
│  - Low priority     │
│  - Logs periodically│
│  - Can block on I/O │
└──────────┬──────────┘
           │ info!, debug!, trace!
           ▼
┌──────────────────────┐
│ Non-Blocking Writer  │
│  - Background thread │
│  - Ring buffer       │
│  - File I/O          │
└──────────┬───────────┘
           │
           ▼
    ┌─────────────────┐
    │  Rolling Files  │
    │ vio.log.2026-02-05
    │ vio.log.2026-02-06
    │       ...
    └─────────────────┘
```

### Code Flow

1. **Main thread** calls `init_tracing_logging()`
   - Creates log directory
   - Spawns background writer thread
   - Sets up rolling daily appender

2. **Hot loop thread** runs VIO pipeline
   - Calls `counters.record_*()`
   - These are atomic operations (~10-100 ns)
   - Never blocks, never waits

3. **Telemetry thread** periodically reports
   - Sleeps 1 second (or 5 Hz for 200 ms)
   - Calls `counters.swap_and_report()`
   - Logs via `tracing::info!()` etc.
   - Background thread queues to file

4. **Background writer thread** (spawned by tracing-appender)
   - Drains internal ring buffer
   - Writes to disk without blocking producers

## Performance Characteristics

### Latency (Hot Loop)

| Operation | Cost |
|-----------|------|
| `record_frame()` | ~10 ns |
| `record_features_detected(256)` | ~10 ns |
| `record_imu_measurement()` | ~10 ns |
| Logging call (if NOT in hot loop) | 100-1000 ns (+ I/O) |

### Throughput

- **Hot loop overhead**: < 100 ns per frame operation
- **Telemetry interval**: 1-5 Hz (typically 1 Hz)
- **I/O blocking**: 0 ns in hot loop, delegated to background thread

### Memory Usage

- Counters struct: 48 bytes (6 × `AtomicU64`)
- Per report: 48 bytes
- Ring buffer: Typically ~16-64 KB (tracing-appender internal)

## Integration with VIO Pipeline

### In Estimator Loop

```rust
use rs_vio::TelemetryCounters;
use std::sync::Arc;

pub struct Estimator {
    telemetry: Arc<TelemetryCounters>,
}

impl Estimator {
    pub fn process_frame(&mut self, image: &Image) -> Result<Pose> {
        self.telemetry.record_frame();
        
        let features = self.detect_features(image);
        self.telemetry.record_features_detected(features.len() as u64);
        
        let tracked = self.track_features(&features);
        self.telemetry.record_features_tracked(tracked as u64);
        
        // Process IMU
        for _ in 0..self.imu_buffer.len() {
            self.telemetry.record_imu_measurement();
        }
        
        // ... rest of VIO processing
        Ok(pose)
    }
}
```

### In Main Loop

```rust
fn main() -> anyhow::Result<()> {
    let _log_guard = init_tracing_logging("./logs", "vio")?;
    
    let mut estimator = create_estimator();
    let counters = Arc::new(TelemetryCounters::new());
    
    // Telemetry reporting (low priority)
    let telemetry_counters = Arc::clone(&counters);
    std::thread::spawn(move || {
        loop {
            std::thread::sleep(Duration::from_secs(1));
            
            let report = telemetry_counters.swap_and_report();
            tracing::info!(
                frames = report.frames,
                features_tracked = report.features_tracked,
                imu_measurements = report.imu_measurements,
                "vio_telemetry"
            );
        }
    });
    
    // Main VIO loop (hot path)
    for image in image_stream {
        estimator.process_frame(&image, &counters)?;
    }
    
    Ok(())
}
```

## Best Practices

### 1. **No Logging in Hot Loops**

```rust
// ❌ WRONG: Blocks frame processing
for frame in frames {
    log::info!("Frame {}", frame_id);  // Can stall!
    process_frame(frame);
}

// ✅ RIGHT: Atomic update, log later
for frame in frames {
    counters.record_frame();           // ~10 ns
    process_frame(frame);
}
```

### 2. **Use Atomic Counters Everywhere**

```rust
// ✅ Good: Track everything you need
counters.record_features_detected(num_detected);
counters.record_features_tracked(num_tracked);
counters.record_imu_measurement();

// Report periodically
let report = counters.swap_and_report();
info!("Processed {} features", report.features_tracked);
```

### 3. **Telemetry at 1-5 Hz**

```rust
// ✅ Right frequency
std::thread::sleep(Duration::from_secs(1));     // 1 Hz (good)
std::thread::sleep(Duration::from_millis(200)); // 5 Hz (ok)

// ❌ Too fast (overhead)
std::thread::sleep(Duration::from_millis(10));  // 100 Hz (no!)
```

### 4. **Control Log Levels at Runtime**

```bash
# Production (minimal logging)
RUST_LOG=info ./vio

# Debugging (specific crate)
RUST_LOG=rs_vio=debug ./vio

# Troubleshooting (everything)
RUST_LOG=trace ./vio
```

## Testing

All telemetry tests pass:

```bash
cargo test --lib logging
# test result: ok. 9 passed
```

Example test output:

```
test logging::tracing_config::tests::test_telemetry_counters ... ok
test logging::tracing_config::tests::test_telemetry_drop_rate ... ok
```

## Log File Format

Each log entry includes:
- **ISO 8601 timestamp**: `2026-02-05T14:02:15.593960Z`
- **Log level**: `INFO`, `DEBUG`, `WARN`, `ERROR`
- **Thread ID**: `ThreadId(01)`, `ThreadId(03)`
- **Source location**: `tracing_demo:21` (file:line)
- **Message**: Human-readable event
- **Structured fields**: `frames=31 dropped=0 features_detected=4096`

Example:
```
2026-02-05T14:02:15.593960Z  INFO main ThreadId(01) tracing_demo:21: VIO starting
2026-02-05T14:02:50.552511Z  INFO ThreadId(04) tracing_demo:91: telemetry frames=31 dropped=0 features_detected=4096 features_tracked=2640
```

## See Also

- [examples/tracing_demo.rs](../../examples/tracing_demo.rs) - Complete working example
- [src/logging/tracing_config.rs](../../src/logging/tracing_config.rs) - Implementation
- [REALTIME_LOGGING.md](../../REALTIME_LOGGING.md) - Custom metrics module docs
