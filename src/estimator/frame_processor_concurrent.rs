//! Concurrent frame processor with task pipelining
//!
//! Manages multiple VIO pipeline stages as independent async tasks
//! with message-passing channels for communication.

use crate::datasets::ImuData;
use crate::estimator::Frame;
use crate::{Result, VIOError};
use std::sync::Arc;
use tokio::sync::mpsc;
use std::collections::BTreeMap;

/// Configuration for concurrent processor
#[derive(Clone, Debug)]
pub struct ConcurrentConfig {
    /// Maximum frames in pipeline simultaneously
    pub pipeline_depth: usize,
    /// Enable frame reordering to maintain causal ordering
    pub maintain_order: bool,
    /// Timeout for processing individual frames (ms)
    pub frame_timeout_ms: u64,
    /// Number of worker threads for feature detection
    pub feature_workers: usize,
    /// Number of worker threads for optimization
    pub optimization_workers: usize,
    /// Optional simulated work delay (ms) for testing/benchmarks
    pub simulated_work_ms: Option<u64>,
    /// Optional jitter to force out-of-order completion (ms, applied to even ids)
    pub simulated_jitter_ms: Option<u64>,
}

impl Default for ConcurrentConfig {
    fn default() -> Self {
        Self {
            pipeline_depth: 4,
            maintain_order: true,
            frame_timeout_ms: 33,
            feature_workers: 2,
            optimization_workers: 1,
            simulated_work_ms: None,
            simulated_jitter_ms: None,
        }
    }
}

/// Frame with metadata for concurrent processing
#[derive(Clone)]
pub struct ConcurrentFrame {
    pub id: u64,
    pub timestamp_ns: i64,
    pub frame: Arc<Frame>,
    pub imu_data: Option<Vec<ImuData>>,
}

/// Processing result with sequence information
pub struct ProcessingResult {
    pub frame_id: u64,
    pub frame: Arc<Frame>,
    pub processed_at_ns: i64,
    pub success: bool,
    pub error: Option<String>,
    pub feature_count: usize,
}

/// Concurrent VIO frame processor
pub struct ConcurrentFrameProcessor {
    config: ConcurrentConfig,
    frame_id: u64,
    // Channel for submitting frames
    frame_tx: mpsc::Sender<ConcurrentFrame>,
    // Channel for results (in order if maintain_order=true)
    result_rx: mpsc::Receiver<ProcessingResult>,
    // Reordering buffer if maintain_order=true
    reorder_buffer: BTreeMap<u64, ProcessingResult>,
    next_output_id: u64,
}

impl ConcurrentFrameProcessor {
    /// Create new concurrent processor
    pub fn new(config: ConcurrentConfig) -> (Self, ProcessingHandle) {
        let (frame_tx, frame_rx) = mpsc::channel(config.pipeline_depth);
        let (result_tx, result_rx) = mpsc::channel(config.pipeline_depth);

        let processor = Self {
            config: config.clone(),
            frame_id: 0,
            frame_tx,
            result_rx,
            reorder_buffer: BTreeMap::new(),
            next_output_id: 0,
        };

        let handle = ProcessingHandle {
            config,
            frame_rx: Arc::new(tokio::sync::Mutex::new(frame_rx)),
            result_tx,
            task_set: tokio::task::JoinSet::new(),
        };

        (processor, handle)
    }

    /// Submit frame for processing
    pub async fn process_frame(
        &mut self,
        frame: Arc<Frame>,
        imu_data: Option<Vec<ImuData>>,
        timestamp_ns: i64,
    ) -> Result<u64> {
        let concurrent_frame = ConcurrentFrame {
            id: self.frame_id,
            timestamp_ns,
            frame,
            imu_data,
        };

        let frame_id = self.frame_id;
        self.frame_id += 1;

        self.frame_tx
            .send(concurrent_frame)
            .await
            .map_err(|_| VIOError::Transient("Frame channel closed".to_string()))?;

        Ok(frame_id)
    }

    /// Receive next result (in order if configured)
    pub async fn recv_result(&mut self) -> Result<ProcessingResult> {
        if self.config.maintain_order {
            self.recv_ordered().await
        } else {
            self.recv_unordered().await
        }
    }

    async fn recv_unordered(&mut self) -> Result<ProcessingResult> {
        self.result_rx
            .recv()
            .await
            .ok_or_else(|| VIOError::Transient("Result channel closed".to_string()))
    }

    async fn recv_ordered(&mut self) -> Result<ProcessingResult> {
        loop {
            // Check if next expected result is in buffer
            if let Some(result) = self.reorder_buffer.remove(&self.next_output_id) {
                self.next_output_id += 1;
                return Ok(result);
            }

            // Try to receive next result
            match self.result_rx.recv().await {
                Some(result) => {
                    if result.frame_id == self.next_output_id {
                        // Perfect: next in order
                        self.next_output_id += 1;
                        return Ok(result);
                    } else {
                        // Out of order: buffer it
                        self.reorder_buffer.insert(result.frame_id, result);
                    }
                }
                None => {
                    return Err(VIOError::Transient("Result channel closed".to_string()));
                }
            }
        }
    }

    /// Get number of frames currently in pipeline
    pub fn queue_depth(&self) -> usize {
        (self.frame_id - self.next_output_id) as usize
    }
}

/// Handle for managing pipeline tasks
pub struct ProcessingHandle {
    config: ConcurrentConfig,
    frame_rx: Arc<tokio::sync::Mutex<mpsc::Receiver<ConcurrentFrame>>>,
    result_tx: mpsc::Sender<ProcessingResult>,
    task_set: tokio::task::JoinSet<()>,
}

impl ProcessingHandle {
    /// Start concurrent processing tasks
    pub fn start(&mut self) {
        let (detected_tx, detected_rx) = mpsc::channel(self.config.pipeline_depth);
        let detected_rx = Arc::new(tokio::sync::Mutex::new(detected_rx));

        // Spawn feature detection workers
        for _ in 0..self.config.feature_workers {
            let frame_rx = Arc::clone(&self.frame_rx);
            let detected_tx = detected_tx.clone();
            let worker_config = self.config.clone();

            self.task_set.spawn(async move {
                Self::feature_detection_worker(frame_rx, detected_tx, worker_config).await;
            });
        }

        // Spawn optimization workers
        for _ in 0..self.config.optimization_workers {
            let detected_rx = Arc::clone(&detected_rx);
            let result_tx = self.result_tx.clone();
            let worker_config = self.config.clone();

            self.task_set.spawn(async move {
                Self::optimization_worker(detected_rx, result_tx, worker_config).await;
            });
        }
    }

    /// Feature detection worker task
    async fn feature_detection_worker(
        frame_rx: Arc<tokio::sync::Mutex<mpsc::Receiver<ConcurrentFrame>>>,
        detected_tx: mpsc::Sender<(ConcurrentFrame, usize)>,
        config: ConcurrentConfig,
    ) {
        loop {
            let frame = {
                let mut rx = frame_rx.lock().await;
                rx.recv().await
            };
            
            match frame {
                Some(frame) => {
                    if let Some(delay_ms) = config.simulated_work_ms {
                        let jitter = if let Some(jitter_ms) = config.simulated_jitter_ms {
                            if frame.id % 2 == 0 { jitter_ms } else { 0 }
                        } else {
                            0
                        };
                        tokio::time::sleep(std::time::Duration::from_millis(delay_ms + jitter)).await;
                    }

                    let feature_count = frame.frame.left_features.len();

                    // TODO: Implement actual feature detection
                    if detected_tx
                        .send((frame, feature_count))
                        .await
                        .is_err()
                    {
                        break;
                    }
                }
                None => break,
            }
        }
    }

    /// Optimization worker task (placeholder for real BA/pose refinement)
    async fn optimization_worker(
        detected_rx: Arc<tokio::sync::Mutex<mpsc::Receiver<(ConcurrentFrame, usize)>>>,
        result_tx: mpsc::Sender<ProcessingResult>,
        config: ConcurrentConfig,
    ) {
        loop {
            let detected = {
                let mut rx = detected_rx.lock().await;
                rx.recv().await
            };

            match detected {
                Some((frame, feature_count)) => {
                    let start = std::time::Instant::now();

                    if let Some(delay_ms) = config.simulated_work_ms {
                        let jitter = if let Some(jitter_ms) = config.simulated_jitter_ms {
                            if frame.id % 2 == 0 { jitter_ms } else { 0 }
                        } else {
                            0
                        };
                        tokio::time::sleep(std::time::Duration::from_millis(delay_ms + jitter)).await;
                    }

                    let processed_at_ns = frame.timestamp_ns + start.elapsed().as_nanos() as i64;

                    let result = ProcessingResult {
                        frame_id: frame.id,
                        frame: frame.frame,
                        processed_at_ns,
                        success: true,
                        error: None,
                        feature_count,
                    };

                    if result_tx.send(result).await.is_err() {
                        break;
                    }
                }
                None => break,
            }
        }
    }

    /// Shutdown concurrent processor gracefully
    pub async fn shutdown(mut self) {
        // Close all channels
        drop(self.frame_rx);
        drop(self.result_tx);

        // Wait for all tasks to complete
        while self.task_set.join_next().await.is_some() {}
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_processor_creation() {
        let config = ConcurrentConfig::default();
        let (processor, _handle) = ConcurrentFrameProcessor::new(config);
        assert_eq!(processor.queue_depth(), 0);
        assert_eq!(processor.next_output_id, 0);
    }

    #[test]
    fn test_config_defaults() {
        let config = ConcurrentConfig::default();
        assert_eq!(config.pipeline_depth, 4);
        assert_eq!(config.feature_workers, 2);
        assert_eq!(config.optimization_workers, 1);
        assert!(config.maintain_order);
    }

    #[test]
    fn test_reorder_buffer_ordering() {
        let mut buffer = BTreeMap::new();
        let mut next_id = 0;

        // Simulate out-of-order insertion
        buffer.insert(1, 11);
        buffer.insert(0, 10);
        buffer.insert(3, 13);
        buffer.insert(2, 12);

        // Verify ordering on retrieval
        let ids: Vec<_> = buffer.keys().copied().collect();
        for id in ids {
            let val = buffer.remove(&id).unwrap();
            assert_eq!(id, next_id);
            assert_eq!(val, 10 + next_id as i32);
            next_id += 1;
        }
    }
}
