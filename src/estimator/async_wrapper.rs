//! Async wrapper for sequential estimator pipeline
//!
//! Provides async/await interface for the synchronous Estimator,
//! enabling non-blocking frame submission and result retrieval.

use crate::datasets::config::Config;
use crate::datasets::{CameraModelType, ImuData};
use crate::estimator::{
    Estimator, FailureRecoveryTracker, LatencyHistogram, ProcessingMetrics,
    StreamingPatternAnalyzer,
};
use crate::viewers::Viewer;
use anyhow::Result;
use std::cmp::Ordering;
use std::collections::BinaryHeap;
use std::panic::{self, AssertUnwindSafe};
use std::sync::{Arc, Mutex, MutexGuard};
use std::time::{Duration, Instant};
use tokio::sync::{mpsc, oneshot};
use tokio::time::timeout;

/// Lock a mutex, recovering from poison (worker thread panicked but we continue).
fn lock_or_recover<T>(mutex: &Mutex<T>) -> MutexGuard<'_, T> {
    match mutex.lock() {
        Ok(guard) => guard,
        Err(err) => err.into_inner(),
    }
}

/// Static error message for backlog limit (no allocation in hot path)
const BACKLOG_ERROR: &str = "Frame skipped - backlog limit exceeded";

/// Static error message for estimator panic recovery
const ESTIMATOR_PANIC_ERROR: &str = "Estimator panicked while processing frame";

/// Static error message for test panic
#[cfg(test)]
const TEST_PANIC_ERROR: &str = "AsyncEstimator test panic triggered";

/// Configuration for async estimator real-time behavior
#[derive(Clone, Debug)]
pub struct AsyncConfig {
    /// Channel capacity for command queue
    pub channel_capacity: usize,
    /// Timeout for processing individual frames (ms)
    pub frame_timeout_ms: u64,
    /// Maximum number of pending frames before skipping
    pub max_pending_frames: usize,
    /// Enable frame skipping when behind schedule
    pub enable_frame_skipping: bool,
    /// Processing time budget per frame (ms) - 0 disables budget
    pub frame_budget_ms: u64,
    /// Priority for keyframe processing (higher = more important)
    pub keyframe_priority: u8,
    /// Priority for regular frame processing
    pub regular_frame_priority: u8,
}

impl Default for AsyncConfig {
    fn default() -> Self {
        Self {
            channel_capacity: 32,
            frame_timeout_ms: 5000, // 5 seconds for processing (more realistic for testing)
            max_pending_frames: 8,
            enable_frame_skipping: true,
            frame_budget_ms: 30, // Leave buffer for 30fps
            keyframe_priority: 10,
            regular_frame_priority: 5,
        }
    }
}

#[derive(Debug)]
enum Command {
    ProcessFrame {
        frame_id: i64,
        left_image: Vec<u8>,
        right_image: Vec<u8>,
        timestamp_ns: i64,
        imu_data: Option<Vec<ImuData>>,
        priority: u8,
        is_keyframe: bool,
        respond_to: oneshot::Sender<Result<()>>,
    },
    #[cfg(test)]
    TestPanic {
        respond_to: oneshot::Sender<Result<()>>,
    },
    Shutdown(oneshot::Sender<()>),
}

#[derive(Debug)]
struct PrioritizedCommand {
    priority: u8,
    sequence: u64,
    command: Command,
}

impl PartialEq for PrioritizedCommand {
    fn eq(&self, other: &Self) -> bool {
        self.priority == other.priority && self.sequence == other.sequence
    }
}

impl Eq for PrioritizedCommand {}

impl PartialOrd for PrioritizedCommand {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for PrioritizedCommand {
    fn cmp(&self, other: &Self) -> Ordering {
        match self.priority.cmp(&other.priority) {
            Ordering::Equal => other.sequence.cmp(&self.sequence),
            ordering => ordering,
        }
    }
}

/// Async wrapper around sequential Estimator
///
/// Spawns blocking Estimator operations on dedicated tokio blocking threads
/// to prevent starving other async tasks.
pub struct AsyncEstimator {
    command_tx: mpsc::Sender<Command>,
    config: AsyncConfig,
    join_handle: Option<std::thread::JoinHandle<()>>,
    metrics: Arc<Mutex<ProcessingMetrics>>,
    latency_histogram: Arc<Mutex<LatencyHistogram>>,
    streaming_analyzer: Arc<Mutex<StreamingPatternAnalyzer>>,
    failure_tracker: Arc<Mutex<FailureRecoveryTracker>>,
}

impl AsyncEstimator {
    /// Create new async estimator running on a dedicated worker thread.
    pub fn new_with_cameras(
        config: Config,
        viewer: Option<Box<dyn Viewer>>,
        left_cam: Option<CameraModelType>,
        right_cam: Option<CameraModelType>,
    ) -> Self {
        Self::new_with_cameras_and_async_config(
            config,
            viewer,
            left_cam,
            right_cam,
            AsyncConfig::default(),
        )
    }

    /// Create new async estimator with custom async configuration.
    pub fn new_with_cameras_and_async_config(
        config: Config,
        viewer: Option<Box<dyn Viewer>>,
        left_cam: Option<CameraModelType>,
        right_cam: Option<CameraModelType>,
        async_config: AsyncConfig,
    ) -> Self {
        let (command_tx, mut command_rx) = mpsc::channel(async_config.channel_capacity);

        let metrics = Arc::new(Mutex::new(ProcessingMetrics::default()));
        let latency_histogram = Arc::new(Mutex::new(LatencyHistogram::default()));
        let streaming_analyzer = Arc::new(Mutex::new(StreamingPatternAnalyzer::default()));
        let failure_tracker = Arc::new(Mutex::new(FailureRecoveryTracker::default()));

        let metrics_thread = Arc::clone(&metrics);
        let latency_thread = Arc::clone(&latency_histogram);
        let streaming_thread = Arc::clone(&streaming_analyzer);
        let failure_thread = Arc::clone(&failure_tracker);
        let async_config_clone = async_config.clone();

        // Thread spawn failure is unrecoverable — no worker means no VIO pipeline
        #[allow(clippy::expect_used)]
        let join_handle = std::thread::Builder::new()
            .name("vio-estimator".into())
            .spawn(move || {
                let mut estimator =
                    Estimator::new_with_cameras(config, viewer, left_cam, right_cam);
                // Pre-allocate queue to avoid allocations during runtime
                let capacity = if async_config_clone.max_pending_frames > 0 {
                    async_config_clone.max_pending_frames
                } else {
                    async_config_clone.channel_capacity
                };
                let mut queue: BinaryHeap<PrioritizedCommand> = BinaryHeap::with_capacity(capacity);
                let mut sequence: u64 = 0;

                loop {
                    let command = match command_rx.blocking_recv() {
                        Some(command) => command,
                        None => break,
                    };

                    enqueue_command(
                        command,
                        &mut queue,
                        &mut sequence,
                        &async_config_clone,
                        &metrics_thread,
                        &streaming_thread,
                    );
                    while let Ok(command) = command_rx.try_recv() {
                        enqueue_command(
                            command,
                            &mut queue,
                            &mut sequence,
                            &async_config_clone,
                            &metrics_thread,
                            &streaming_thread,
                        );
                    }

                    while let Some(prioritized) = queue.pop() {
                        match prioritized.command {
                            Command::ProcessFrame {
                                frame_id,
                                left_image,
                                right_image,
                                timestamp_ns,
                                imu_data,
                                respond_to,
                                ..
                            } => {
                                let frame_start = Instant::now();
                                let result = panic::catch_unwind(AssertUnwindSafe(|| {
                                    estimator.set_viewer_frame(frame_id);
                                    estimator.process_frame(
                                        &left_image,
                                        &right_image,
                                        timestamp_ns,
                                        imu_data.as_deref(),
                                    )
                                }));
                                let processing_time = frame_start.elapsed();
                                let latency_ns = processing_time.as_nanos() as u64;

                                match result {
                                    Ok(frame_result) => {
                                        if frame_result.is_ok() {
                                            let mut m = lock_or_recover(&metrics_thread);
                                            m.frames_processed += 1;
                                            m.total_processing_time_ns += latency_ns;
                                            if m.frames_processed == 1 {
                                                m.min_latency_ns = latency_ns;
                                                m.max_latency_ns = latency_ns;
                                            } else {
                                                if latency_ns < m.min_latency_ns {
                                                    m.min_latency_ns = latency_ns;
                                                }
                                                if latency_ns > m.max_latency_ns {
                                                    m.max_latency_ns = latency_ns;
                                                }
                                            }
                                            m.avg_latency_ns =
                                                m.total_processing_time_ns / m.frames_processed;
                                            if processing_time
                                                > Duration::from_millis(
                                                    async_config_clone.frame_timeout_ms,
                                                )
                                            {
                                                m.deadline_misses += 1;
                                            }
                                            if async_config_clone.frame_budget_ms > 0
                                                && processing_time
                                                    > Duration::from_millis(
                                                        async_config_clone.frame_budget_ms,
                                                    )
                                            {
                                                m.budget_violations += 1;
                                            }
                                            drop(m);

                                            lock_or_recover(&latency_thread).record(latency_ns);

                                            lock_or_recover(&failure_thread).mark_worker_healthy();
                                        }

                                        let _ = respond_to.send(frame_result);
                                    },
                                    Err(_) => {
                                        let now_ns = timestamp_ns;
                                        lock_or_recover(&failure_thread).record_panic_recovery(now_ns);
                                        let _ = respond_to
                                            .send(Err(anyhow::anyhow!(ESTIMATOR_PANIC_ERROR)));
                                        // Keep worker alive: estimator state may be inconsistent,
                                        // but abandoning the thread permanently kills VIO.
                                        log::error!(
                                            "[AsyncEstimator] panic recovered; worker continues"
                                        );
                                        continue;
                                    },
                                }
                            },
                            Command::Shutdown(respond_to) => {
                                let _ = respond_to.send(());
                                return;
                            },
                            #[cfg(test)]
                            Command::TestPanic { respond_to } => {
                                let result = panic::catch_unwind(AssertUnwindSafe(|| {
                                    panic!("AsyncEstimator test panic");
                                }));

                                match result {
                                    Ok(_) => {
                                        let _ = respond_to.send(Ok(()));
                                    },
                                    Err(_) => {
                                        lock_or_recover(&failure_thread).record_panic_recovery(0);
                                        let _ =
                                            respond_to.send(Err(anyhow::anyhow!(TEST_PANIC_ERROR)));
                                        return;
                                    },
                                }
                            },
                        }

                        while let Ok(command) = command_rx.try_recv() {
                            enqueue_command(
                                command,
                                &mut queue,
                                &mut sequence,
                                &async_config_clone,
                                &metrics_thread,
                                &streaming_thread,
                            );
                        }
                    }
                }
            })
            .expect("failed to spawn VIO estimator thread");

        Self {
            command_tx,
            config: async_config,
            join_handle: Some(join_handle),
            metrics,
            latency_histogram,
            streaming_analyzer,
            failure_tracker,
        }
    }

    /// Process frame asynchronously with real-time constraints
    pub async fn process_frame_async(
        &self,
        frame_id: i64,
        left_image: Vec<u8>,
        right_image: Vec<u8>,
        timestamp_ns: i64,
        imu_data: Option<Vec<ImuData>>,
    ) -> Result<()> {
        self.process_frame_async_with_priority(
            frame_id,
            left_image,
            right_image,
            timestamp_ns,
            imu_data,
            false,
        )
        .await
    }

    /// Process frame asynchronously with priority control
    pub async fn process_frame_async_with_priority(
        &self,
        frame_id: i64,
        left_image: Vec<u8>,
        right_image: Vec<u8>,
        timestamp_ns: i64,
        imu_data: Option<Vec<ImuData>>,
        is_keyframe: bool,
    ) -> Result<()> {
        let priority = if is_keyframe {
            self.config.keyframe_priority
        } else {
            self.config.regular_frame_priority
        };

        let (respond_to, response_rx) = oneshot::channel();
        let deadline = Instant::now() + Duration::from_millis(self.config.frame_timeout_ms);

        let send_budget = deadline.saturating_duration_since(Instant::now());
        if send_budget == Duration::from_secs(0) {
            return Err(anyhow::anyhow!(
                "Frame {} send timed out after {}ms",
                frame_id,
                self.config.frame_timeout_ms
            ));
        }

        // Try to send command with timeout
        let send_result = timeout(
            send_budget,
            self.command_tx.send(Command::ProcessFrame {
                frame_id,
                left_image,
                right_image,
                timestamp_ns,
                imu_data,
                priority,
                is_keyframe,
                respond_to,
            }),
        )
        .await;

        match send_result {
            Ok(Ok(())) => {
                // Command sent successfully, wait for response with timeout
                let response_budget = deadline.saturating_duration_since(Instant::now());
                if response_budget == Duration::from_secs(0) {
                    return Err(anyhow::anyhow!(
                        "Frame {} processing timed out after {}ms",
                        frame_id,
                        self.config.frame_timeout_ms
                    ));
                }
                match timeout(response_budget, response_rx).await {
                    Ok(Ok(result)) => result,
                    Ok(Err(_)) => Err(anyhow::anyhow!("AsyncEstimator response channel dropped")),
                    Err(_) => Err(anyhow::anyhow!(
                        "Frame {} processing timed out after {}ms",
                        frame_id,
                        self.config.frame_timeout_ms
                    )),
                }
            },
            Ok(Err(_)) => Err(anyhow::anyhow!("AsyncEstimator worker closed")),
            Err(_) => {
                // Channel is full or timed out - implement frame skipping
                if self.config.enable_frame_skipping {
                    Err(anyhow::anyhow!(
                        "Frame {} skipped - channel full/timeout (capacity: {})",
                        frame_id,
                        self.config.channel_capacity
                    ))
                } else {
                    Err(anyhow::anyhow!(
                        "Frame {} send timed out after {}ms",
                        frame_id,
                        self.config.frame_timeout_ms
                    ))
                }
            },
        }
    }

    #[cfg(test)]
    async fn trigger_test_panic(&self) -> Result<()> {
        let (respond_to, response_rx) = oneshot::channel();
        let deadline = Instant::now() + Duration::from_millis(self.config.frame_timeout_ms);

        let send_budget = deadline.saturating_duration_since(Instant::now());
        if send_budget == Duration::from_secs(0) {
            return Err(anyhow::anyhow!(
                "Test panic send timed out after {}ms",
                self.config.frame_timeout_ms
            ));
        }

        let send_result = timeout(
            send_budget,
            self.command_tx.send(Command::TestPanic { respond_to }),
        )
        .await;

        match send_result {
            Ok(Ok(())) => {
                let response_budget = deadline.saturating_duration_since(Instant::now());
                if response_budget == Duration::from_secs(0) {
                    return Err(anyhow::anyhow!(
                        "Test panic response timed out after {}ms",
                        self.config.frame_timeout_ms
                    ));
                }
                match timeout(response_budget, response_rx).await {
                    Ok(Ok(result)) => result,
                    Ok(Err(_)) => Err(anyhow::anyhow!("Test panic response channel dropped")),
                    Err(_) => Err(anyhow::anyhow!(
                        "Test panic response timed out after {}ms",
                        self.config.frame_timeout_ms
                    )),
                }
            },
            Ok(Err(_)) => Err(anyhow::anyhow!("AsyncEstimator worker closed")),
            Err(_) => Err(anyhow::anyhow!(
                "Test panic send timed out after {}ms",
                self.config.frame_timeout_ms
            )),
        }
    }

    /// Check if estimator can accept more frames (for backpressure)
    pub fn can_accept_frame(&self) -> bool {
        !self.command_tx.is_closed() && self.command_tx.capacity() > 0
    }

    /// Get current async configuration
    pub const fn config(&self) -> &AsyncConfig {
        &self.config
    }

    /// Get current processing metrics
    pub fn metrics(&self) -> ProcessingMetrics {
        lock_or_recover(&self.metrics).clone()
    }

    /// Get current latency histogram
    pub fn latency_histogram(&self) -> LatencyHistogram {
        lock_or_recover(&self.latency_histogram).clone()
    }

    /// Get current streaming pattern description
    pub fn streaming_status(&self) -> String {
        lock_or_recover(&self.streaming_analyzer).description()
    }

    /// Get failure recovery status
    pub fn failure_status(&self) -> String {
        lock_or_recover(&self.failure_tracker).status()
    }

    /// Shutdown the async estimator gracefully
    pub async fn shutdown(mut self) {
        if let Some(handle) = self.join_handle.take() {
            let (respond_to, response_rx) = oneshot::channel();
            let shutdown_timeout = Duration::from_millis(1000); // 1 second timeout for shutdown

            match timeout(
                shutdown_timeout,
                self.command_tx.send(Command::Shutdown(respond_to)),
            )
            .await
            {
                Ok(Ok(())) => {
                    let _ = timeout(shutdown_timeout, response_rx).await;
                },
                _ => {
                    // Shutdown command failed or timed out
                    log::warn!("AsyncEstimator shutdown timed out");
                },
            }

            let _ = handle.join();
        }
    }
}

fn enqueue_command(
    command: Command,
    queue: &mut BinaryHeap<PrioritizedCommand>,
    sequence: &mut u64,
    config: &AsyncConfig,
    metrics: &Arc<Mutex<ProcessingMetrics>>,
    streaming: &Arc<Mutex<StreamingPatternAnalyzer>>,
) {
    match command {
        Command::ProcessFrame {
            frame_id,
            left_image,
            right_image,
            timestamp_ns,
            imu_data,
            priority,
            is_keyframe,
            respond_to,
        } => {
            lock_or_recover(streaming).update(timestamp_ns);

            let backlog = queue.len();
            let over_limit = config.max_pending_frames > 0 && backlog >= config.max_pending_frames;

            if over_limit {
                lock_or_recover(metrics).frames_skipped += 1;
                // Use static error to avoid allocation in hot path
                let _ = respond_to.send(Err(anyhow::anyhow!(BACKLOG_ERROR)));
                return;
            }

            queue.push(PrioritizedCommand {
                priority,
                sequence: *sequence,
                command: Command::ProcessFrame {
                    frame_id,
                    left_image,
                    right_image,
                    timestamp_ns,
                    imu_data,
                    priority,
                    is_keyframe,
                    respond_to,
                },
            });
            *sequence = sequence.wrapping_add(1);
        },
        Command::Shutdown(respond_to) => {
            queue.push(PrioritizedCommand {
                priority: u8::MAX,
                sequence: *sequence,
                command: Command::Shutdown(respond_to),
            });
            *sequence = sequence.wrapping_add(1);
        },
        #[cfg(test)]
        Command::TestPanic { respond_to } => {
            queue.push(PrioritizedCommand {
                priority: u8::MAX,
                sequence: *sequence,
                command: Command::TestPanic { respond_to },
            });
            *sequence = sequence.wrapping_add(1);
        },
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
#[path = "async_wrapper_tests.rs"]
mod tests;

