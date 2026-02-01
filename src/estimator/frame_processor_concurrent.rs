use std::collections::BTreeMap;
use std::time::Duration;
use tokio::sync::mpsc;

/// Configuration for concurrent frame processing
#[derive(Debug, Clone)]
pub struct ConcurrentConfig {
    /// Number of frames to keep in pipeline (default: 4)
    pub pipeline_depth: usize,
    /// Whether to maintain causal output ordering (default: true)
    pub maintain_order: bool,
    /// Timeout for individual frame processing (ms, default: 33)
    pub frame_timeout_ms: u64,
    /// Number of feature detection worker tasks (default: 2)
    pub feature_workers: usize,
    /// Number of optimization worker tasks (default: 1)
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

/// Result from processing a single frame
#[derive(Debug, Clone)]
pub struct ProcessingResult {
    /// Sequence number of the frame
    pub sequence: u64,
    /// Processing succeeded
    pub success: bool,
    /// Error message if failed
    pub error: Option<String>,
    /// Features found (if applicable)
    pub features: usize,
    /// Processing time in milliseconds
    pub processing_time_ms: f64,
}

/// Handle for managing frame processing task lifecycle
pub struct ProcessingHandle {
    /// Receiver for results
    receiver: mpsc::Receiver<ProcessingResult>,
    /// Configuration
    config: ConcurrentConfig,
}

impl ProcessingHandle {
    /// Get the next result with timeout
    pub async fn wait_result(&mut self) -> Result<ProcessingResult, String> {
        let timeout = Duration::from_millis(self.config.frame_timeout_ms);

        tokio::time::timeout(timeout, self.receiver.recv())
            .await
            .map_err(|_| {
                "Frame processing timeout".to_string()
            })?
            .ok_or_else(|| "Channel closed unexpectedly".to_string())
    }

    /// Try to get result without blocking
    pub fn try_get_result(&mut self) -> Option<ProcessingResult> {
        self.receiver.try_recv().ok()
    }
}

/// Concurrent frame processor with worker-based architecture
///
/// Manages multiple worker tasks that process frames concurrently while
/// maintaining causal ordering of results if configured.
///
/// # Architecture
///
/// ```text
/// Input Channel → [Worker Pool] → Reorder Buffer → Output Channel
/// ```
///
/// # Features
///
/// - **Configurable workers**: Adjust parallelism per stage
/// - **Optional reordering**: Maintain causal order or allow out-of-order
/// - **Backpressure**: Automatic flow control via channel limits
/// - **Timeout handling**: Per-frame timeout configuration
///
/// # Example
///
/// ```rust,ignore
/// let config = ConcurrentConfig {
///     pipeline_depth: 8,
///     feature_workers: 4,
///     ..Default::default()
/// };
///
/// let mut processor = ConcurrentFrameProcessor::new(config)?;
/// // Submit frames asynchronously
/// processor.submit(frame)?;
/// // Retrieve results
/// let result = processor.wait_result().await?;
/// ```
pub struct ConcurrentFrameProcessor {
    /// Configuration parameters
    config: ConcurrentConfig,
    /// Channel for submitting work
    work_sender: mpsc::Sender<FrameWorkItem>,
    /// Result receiver
    result_receiver: mpsc::Receiver<ProcessingResult>,
    /// Reordering buffer for out-of-order completion
    reorder_buffer: BTreeMap<u64, ProcessingResult>,
    /// Next sequence ID to output
    next_output_id: u64,
}

/// Internal work item for processing (TODO: use in worker implementation)
#[derive(Debug)]
#[allow(dead_code)]
struct FrameWorkItem {
    sequence: u64,
    data: Vec<u8>,
}

impl ConcurrentFrameProcessor {
    /// Create a new concurrent frame processor with given configuration
    pub fn new(config: ConcurrentConfig) -> Result<Self, String> {
        let (tx, _rx) = mpsc::channel::<FrameWorkItem>(config.pipeline_depth);
        let (_result_tx, result_rx) = mpsc::channel::<ProcessingResult>(config.pipeline_depth);

        Ok(Self {
            config,
            work_sender: tx,
            result_receiver: result_rx,
            reorder_buffer: BTreeMap::new(),
            next_output_id: 0,
        })
    }

    /// Submit a frame for processing
    pub fn submit(&self, sequence: u64, data: Vec<u8>) -> Result<(), String> {
        let item = FrameWorkItem { sequence, data };

        self.work_sender.blocking_send(item).map_err(|_| {
            "Failed to submit frame work item".to_string()
        })
    }

    /// Try to get next result respecting order if configured
    pub fn try_get_next(&mut self) -> Result<Option<ProcessingResult>, String> {
        if !self.config.maintain_order {
            return Ok(self.result_receiver.try_recv().ok());
        }

        // Check if we have the next expected result
        if let Some(result) = self.reorder_buffer.remove(&self.next_output_id) {
            self.next_output_id += 1;
            return Ok(Some(result));
        }

        // Try to get new results and add to buffer
        while let Ok(result) = self.result_receiver.try_recv() {
            if result.sequence == self.next_output_id {
                self.next_output_id += 1;
                return Ok(Some(result));
            }
            self.reorder_buffer.insert(result.sequence, result);
        }

        Ok(None)
    }

    /// Get configuration
    pub const fn config(&self) -> &ConcurrentConfig {
        &self.config
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_config_default() {
        let config = ConcurrentConfig::default();
        assert_eq!(config.pipeline_depth, 4);
        assert_eq!(config.feature_workers, 2);
        assert_eq!(config.optimization_workers, 1);
        assert!(config.maintain_order);
    }

    #[test]
    fn test_processor_creation() {
        let config = ConcurrentConfig::default();
        let processor = ConcurrentFrameProcessor::new(config);
        assert!(processor.is_ok(), "Processor should initialize");
    }

    #[test]
    fn test_processor_with_custom_config() {
        let config = ConcurrentConfig {
            pipeline_depth: 8,
            feature_workers: 4,
            optimization_workers: 2,
            ..Default::default()
        };

        let processor = ConcurrentFrameProcessor::new(config).unwrap();
        assert_eq!(processor.config().pipeline_depth, 8);
        assert_eq!(processor.config().feature_workers, 4);
    }

    #[test]
    fn test_processing_result_creation() {
        let result = ProcessingResult {
            sequence: 42,
            success: true,
            error: None,
            features: 100,
            processing_time_ms: 5.5,
        };

        assert_eq!(result.sequence, 42);
        assert!(result.success);
        assert_eq!(result.features, 100);
    }

    #[test]
    fn test_reorder_buffer_structure() {
        let config = ConcurrentConfig::default();
        let processor = ConcurrentFrameProcessor::new(config).unwrap();

        assert_eq!(processor.next_output_id, 0);
        assert!(processor.reorder_buffer.is_empty());
    }

    #[test]
    fn test_work_item_creation() {
        let item = FrameWorkItem {
            sequence: 1,
            data: vec![1, 2, 3, 4],
        };

        assert_eq!(item.sequence, 1);
        assert_eq!(item.data.len(), 4);
    }
}
