# Production Deployment Guide for Async VIO Pipeline

## Overview

This guide covers deploying the hardened async VIO pipeline to production environments, including error handling, metrics collection, timeout tuning, and failure recovery.

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                 Input Frame Stream                           │
└────────────┬────────────────────────────────────────────────┘
             │
             ├─────────────────────────────────────────────────┐
             │                                                 │
    ┌────────▼─────────┐                              ┌──────▼──────────┐
    │  ResilientAsync  │                              │  ResilientAsync │
    │  FeatureDetector │◄─────────────────────────────┤    Optimizer    │
    │    (timeout)     │      [Detection Queue]       │   (timeout)     │
    │    [metrics]     │                              │   [metrics]     │
    └────────┬─────────┘                              └────────┬────────┘
             │                                                 │
             │                 ┌──────────────────────────────┘
             │                 │
             │          ┌──────▼──────────┐
             │          │  PipelineMetrics│
             │          │ (observability) │
             │          └─────────────────┘
             │
    ┌────────▼─────────┐
    │  Error Recovery  │
    │  - Timeout       │
    │  - Transient     │
    │  - Skip frame    │
    └──────────────────┘
```

## Components

### 1. ResilientAsyncFeatureDetector<LEVELS>

Provides timeout protection and validation for feature detection.

#### Configuration

```rust
use rs_vio::estimator::{ResilientAsyncFeatureDetector, PipelineMetrics};
use rs_vio::datasets::config::FeatureDetectionConfig;

let config = FeatureDetectionConfig::default();
let metrics = PipelineMetrics::new();

let detector = ResilientAsyncFeatureDetector::<3>::new(&config, Some(metrics))
    .with_timeout(100)          // Detection timeout in ms
    .with_min_features(50);     // Warn if below threshold
```

#### Error Handling

- **Timeout (> 100ms)**: Logs timeout error, records recovery, continues
- **Detection Failure**: Records error, metrics track failure rate
- **Low Features (< 50)**: Warning logged, still processes frame

#### Usage

```rust
// Takes owned GrayImage
match detector.detect_features(left_frame, right_frame, &mut vio_frame).await {
    Ok(feature_count) => {
        println!("Detected {} features", feature_count);
    }
    Err(PipelineError::Timeout { stage, limit_ms, elapsed_ms }) => {
        eprintln!("Detection timeout: {}ms > {}ms limit", elapsed_ms, limit_ms);
        // Pipeline continues - frame is skipped gracefully
    }
    Err(e) => eprintln!("Detection error: {}", e),
}
```

### 2. ResilientAsyncOptimizer

Provides timeout protection for the optimization/BA stage.

#### Configuration

```rust
use rs_vio::estimator::{ResilientAsyncOptimizer, PipelineMetrics};
use rs_vio::datasets::config::Config;

let config = Config::load("config/tum_vi.yaml")?;
let metrics = PipelineMetrics::new();

let optimizer = ResilientAsyncOptimizer::new(&config, Some(metrics))
    .with_timeout(500);  // Optimization timeout in ms
```

#### Error Handling

- **Timeout (> 500ms)**: Records timeout, skips BA, continues with next frame
- **Optimization Failure**: Records error, metrics track failure rate
- **Transient Errors**: Automatic retry with exponential backoff

#### Usage

```rust
match optimizer.add_frame_and_optimize(frame.clone(), true).await {
    Ok(success) => {
        if success {
            println!("Frame added and optimized");
        } else {
            println!("Frame added (sliding window full)");
        }
    }
    Err(PipelineError::Timeout { .. }) => {
        eprintln!("Optimization timeout - skipping BA");
        // Continue without optimization
    }
    Err(e) => eprintln!("Optimization error: {}", e),
}
```

### 3. PipelineMetrics

Provides real-time observability into pipeline performance and health.

#### Metric Types

**Stage Metrics** (Detection and Optimization):
- Frame count
- Average/min/max latency (µs)
- Error count

**Frame Metrics** (Per-frame):
- Frame ID and timestamp
- Detection/optimization time
- Queue depth
- Success flag
- Error message (if any)

**Aggregate Metrics**:
- Total frames processed
- Total errors and error rate
- Recovered errors (automatic retries)
- Queue depth (current and max)

#### Usage

```rust
let metrics = PipelineMetrics::new();

// In detection worker
metrics.record_detection(elapsed_us, error);

// In optimization worker  
metrics.record_optimization(elapsed_us, error);

// Track queue pressure
metrics.set_queue_depth(current_depth);

// Record frame-level data
metrics.record_frame(FrameMetrics {
    frame_id: 42,
    timestamp_ns: 1000000,
    detection_time_us: 15000,
    optimization_time_us: 50000,
    e2e_time_us: 65000,
    queue_depth: 2,
    success: true,
    error_message: None,
});

// Get summary report
println!("{}", metrics.summary());
// Output:
// Pipeline Metrics Summary:
// ├─ Total Frames: 1000
// ├─ Total Errors: 2 (0.20%)
// ├─ Recovered Errors: 2
// ├─ Queue Depth: 1/4
// ├─ Detection Stage:
// │  ├─ Avg: 15.2µs, Min: 10µs, Max: 45µs
// │  └─ Errors: 0
// └─ Optimization Stage:
//    ├─ Avg: 125.3µs, Min: 100µs, Max: 200µs
//    └─ Errors: 2
```

## Error Handling Patterns

### 1. PipelineError Variants

```rust
pub enum PipelineError {
    /// Mutex was poisoned (task panicked)
    PoisonedState(String),
    
    /// Channel closed unexpectedly
    ChannelClosed(String),
    
    /// Processing exceeded timeout
    Timeout {
        stage: String,
        limit_ms: u64,
        elapsed_ms: u64,
    },
    
    /// Feature detection failure
    FeatureDetectionFailed(String),
    
    /// Optimization failure
    OptimizationFailed(String),
    
    /// Buffer overflowed
    BufferOverflow(String),
    
    /// Configuration error
    ConfigError(String),
    
    /// Recoverable (can retry)
    Transient(String),
    
    /// Unrecoverable
    Fatal(String),
}
```

### 2. Recovery Strategies

The framework automatically determines recovery strategy based on error type:

```rust
pub enum RecoveryStrategy {
    /// Retry with exponential backoff (up to max_attempts)
    Retry { max_attempts: u32, backoff_ms: u64 },
    
    /// Skip frame and continue
    Skip,
    
    /// Skip and slow down pipeline
    SlowDown { backoff_ms: u64 },
    
    /// Gracefully shutdown
    Shutdown,
}
```

**Error-to-Strategy Mapping**:

| Error | Strategy | Max Retries | Backoff |
|-------|----------|------------|---------|
| Transient | Retry | 3 | 50ms exponential |
| Timeout | SlowDown | — | 33ms |
| Feature Failure | Skip | — | — |
| Optimization Failure | Retry | 2 | 100ms exponential |
| Poisoned State | Retry | 1 | 0ms |
| Config Error | Shutdown | — | — |
| Fatal | Shutdown | — | — |

### 3. Implementing Custom Error Handling

```rust
use rs_vio::estimator::{PipelineError, RecoveryStrategy, RecoveryContext};

async fn handle_detection_error(
    err: PipelineError,
    frame_id: u64,
) {
    let strategy = RecoveryStrategy::for_error(&err);
    let mut ctx = RecoveryContext::new(frame_id, "feature_detection".to_string());
    
    loop {
        match strategy {
            RecoveryStrategy::Retry { max_attempts, backoff_ms } => {
                if ctx.can_retry(&strategy) {
                    ctx.next_attempt();
                    let delay = strategy.backoff_for_attempt(ctx.attempt);
                    tokio::time::sleep(delay).await;
                    // Retry detection
                    break;
                }
            }
            RecoveryStrategy::Skip => {
                println!("Skipping frame {}", frame_id);
                break;
            }
            RecoveryStrategy::SlowDown { backoff_ms } => {
                println!("Pipeline slowdown: {}ms", backoff_ms);
                tokio::time::sleep(std::time::Duration::from_millis(backoff_ms)).await;
                break;
            }
            RecoveryStrategy::Shutdown => {
                eprintln!("Fatal error - shutting down pipeline");
                std::process::exit(1);
            }
        }
    }
}
```

## Timeout Tuning

### Feature Detection

**Default**: 100ms
**Adjust based on**:
- Image resolution (larger = longer)
- Number of pyramids/levels (LEVELS generic parameter)
- CPU performance
- Load patterns

```rust
// Conservative for embedded/low-power
.with_timeout(200)

// Aggressive for high-performance systems
.with_timeout(50)
```

### Optimization (Bundle Adjustment)

**Default**: 500ms
**Adjust based on**:
- Sliding window size
- Number of map points
- Number of keyframes
- BA solver iterations

```rust
// For real-time (skip BA if too slow)
.with_timeout(100)

// For offline/batch processing
.with_timeout(2000)
```

## Deployment Checklist

- [ ] Configure timeouts for your hardware (see Timeout Tuning above)
- [ ] Create PipelineMetrics instance for observability
- [ ] Implement metrics collection to your monitoring system
- [ ] Set up logging to capture PipelineError variants
- [ ] Test error paths (see Testing section below)
- [ ] Monitor error rates and recovered errors in production
- [ ] Adjust recovery strategies based on error patterns
- [ ] Document custom error handlers for your environment

## Testing Error Scenarios

### 1. Timeout Simulation

```rust
#[tokio::test]
async fn test_detection_timeout() {
    let detector = ResilientAsyncFeatureDetector::<3>::new(&config, None)
        .with_timeout(1);  // 1ms timeout
    
    let large_image = GrayImage::new(4096, 4096);
    
    match detector.detect_features(large_image.clone(), large_image, &mut frame).await {
        Err(PipelineError::Timeout { stage, .. }) => {
            assert_eq!(stage, "feature_detection");
        }
        _ => panic!("Expected timeout"),
    }
}
```

### 2. Metrics Verification

```rust
#[tokio::test]
async fn test_error_tracking() {
    let metrics = PipelineMetrics::new();
    let optimizer = ResilientAsyncOptimizer::new(&config, Some(metrics.clone()));
    
    let initial_recovered = metrics.recovered_errors();
    
    // Trigger an error scenario...
    
    let final_recovered = metrics.recovered_errors();
    assert!(final_recovered > initial_recovered);
}
```

## Integration with Main Estimator

### Pipeline Integration Pattern

```rust
use rs_vio::estimator::{
    ResilientAsyncFeatureDetector, ResilientAsyncOptimizer,
    PipelineMetrics, FrameMetrics,
};

struct VIOPipeline {
    detector: ResilientAsyncFeatureDetector<3>,
    optimizer: ResilientAsyncOptimizer,
    metrics: PipelineMetrics,
}

impl VIOPipeline {
    pub async fn process_frame(&mut self, left: GrayImage, right: GrayImage) {
        let mut frame = Frame::new(/*...*/);
        
        // Detect features with timeout
        match self.detector.detect_features(left, right, &mut frame).await {
            Ok(feature_count) => {
                // Add to optimizer
                match self.optimizer.add_frame_and_optimize(frame.clone(), true).await {
                    Ok(_success) => {
                        self.metrics.record_frame(FrameMetrics {
                            frame_id: frame.frame_id as u64,
                            timestamp_ns: frame.timestamp as i64,
                            detection_time_us: 0, // Get from metrics
                            optimization_time_us: 0,
                            e2e_time_us: 0,
                            queue_depth: 0,
                            success: true,
                            error_message: None,
                        });
                    }
                    Err(e) => {
                        eprintln!("Optimization error: {}", e);
                    }
                }
            }
            Err(e) => {
                eprintln!("Detection error: {}", e);
            }
        }
    }
    
    pub fn print_metrics(&self) {
        println!("{}", self.metrics.summary());
    }
}
```

## Monitoring and Alerting

### Key Metrics to Monitor

1. **Error Rate**: Total errors / total frames
   - Alert if > 1% (indicates systemic issue)

2. **Timeout Rate**: Timeout errors / total frames
   - Alert if > 0.5% (timeouts too aggressive)

3. **Queue Depth**: Current frames in pipeline
   - Alert if max_queue_depth growing unbounded

4. **Latency**: P50, P90, P99 from FrameMetrics history
   - Alert if P99 > timeout threshold

5. **Recovered Errors**: Auto-recovered errors count
   - Indicates transient failures, may need investigation

### Example Prometheus Metrics

```rust
// Export to Prometheus/Grafana
let total_frames = metrics.total_frames_processed() as i64;
let total_errors = metrics.total_errors() as i64;
let recovered = metrics.recovered_errors() as i64;
let queue_depth = metrics.queue_depth() as i64;

println!(
    "vio_pipeline_frames_total {{}} {}",
    total_frames
);
println!(
    "vio_pipeline_errors_total {{}} {}",
    total_errors
);
println!(
    "vio_pipeline_recovered_total {{}} {}",
    recovered
);
println!(
    "vio_pipeline_queue_depth {{}} {}",
    queue_depth
);
```

## Troubleshooting

### "Optimization timeout in 250ms > 500ms limit"

**Diagnosis**: Bundle adjustment taking too long
**Solutions**:
1. Increase timeout: `.with_timeout(1000)`
2. Reduce optimization frequency (skip BA on some frames)
3. Reduce sliding window size
4. Run on faster hardware

### High timeout rate (> 0.5%)

**Diagnosis**: Timeouts not matching hardware performance
**Solutions**:
1. Profile feature detection and optimization separately
2. Adjust timeouts based on P99 latency from metrics
3. Check for CPU/memory contention
4. Verify image resolution matches expected

### Error rate spiking

**Diagnosis**: Could be transient load or systemic issue
**Solutions**:
1. Check recovered_errors count
2. If recovered > errors, transient and handled
3. If recovered < errors, some errors unhandled
4. Implement exponential backoff in application

## References

- [async_feature_detection.rs](src/estimator/async_feature_detection.rs) - Base detector
- [async_optimization.rs](src/estimator/async_optimization.rs) - Base optimizer
- [error_handling.rs](src/estimator/error_handling.rs) - Error types and recovery
- [pipeline_metrics.rs](src/estimator/pipeline_metrics.rs) - Observability
- [resilient_async_optimizer.rs](src/estimator/resilient_async_optimizer.rs) - Timeout wrapper
- [resilient_async_feature_detector.rs](src/estimator/resilient_async_feature_detector.rs) - Timeout wrapper

## Support

For issues or questions, refer to:
- Integration tests: [tests/pipeline_e2e.rs](tests/pipeline_e2e.rs)
- Benchmarks: [benches/tum_vi_async_pipeline.rs](benches/tum_vi_async_pipeline.rs)
- Error handling tests: [src/estimator/error_handling.rs#tests](src/estimator/error_handling.rs)
