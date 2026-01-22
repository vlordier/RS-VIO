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
}

impl Default for ConcurrentConfig {
    fn default() -> Self {
        Self {
            pipeline_depth: 4,
            maintain_order: true,
            frame_timeout_ms: 33,
            feature_workers: 2,
            optimization_workers: 1,
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
    pub fn start(mut self) {
        // Spawn feature detection workers
        for _ in 0..self.config.feature_workers {
            let frame_rx = Arc::clone(&self.frame_rx);
            let result_tx = self.result_tx.clone();

            self.task_set.spawn(async move {
                Self::feature_detection_worker(frame_rx, result_tx).await;
            });
        }
    }

    /// Feature detection worker task
    async fn feature_detection_worker(
        frame_rx: Arc<tokio::sync::Mutex<mpsc::Receiver<ConcurrentFrame>>>,
        result_tx: mpsc::Sender<ProcessingResult>,
    ) {
        loop {
            let frame = {
                let mut rx = frame_rx.lock().await;
                rx.recv().await
            };
            
            match frame {
                Some(frame) => {
                    let start = std::time::Instant::now();

                    // TODO: Implement actual feature detection
                    // For now: mock processing
                    let result = ProcessingResult {
                        frame_id: frame.id,
                        frame: frame.frame,
                        processed_at_ns: frame.timestamp_ns + start.elapsed().as_nanos() as i64,
                        success: true,
                        error: None,
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

    #[tokio::test]
    async fn test_processor_creation() {
        let config = ConcurrentConfig::default();
        let (processor, handle) = ConcurrentFrameProcessor::new(config);
        assert_eq!(processor.queue_depth(), 0);
        handle.shutdown().await;
    }

    #[tokio::test]
    async fn test_frame_ordering() {
        let config = ConcurrentConfig {
            maintain_order: true,
            ..Default::default()
        };
        let (processor, handle) = ConcurrentFrameProcessor::new(config);
        
        // Note: Can't fully test without real frame data
        // This is a structure test only
        assert_eq!(processor.next_output_id, 0);
        
        handle.shutdown().await;
    }
}
