//! Frame processing abstraction for composable pipeline stages
//!
//! This module defines traits for building flexible, composable frame processing pipelines
//! using the **Strategy** and **Decorator** patterns.

use crate::datasets::FrameContext;
use crate::debug_log;
use crate::estimator::Estimator;
use crate::trace_log;
use crate::Result;

/// Trait for a single frame processing step
///
/// This trait enables composition-based design where different processing stages
/// can be combined and configured independently. Each processor handles one aspect
/// of frame processing (e.g., loading, processing, tracking).
///
/// # Design Pattern
/// Implements the **Strategy Pattern** to encapsulate different processing algorithms
/// and make them interchangeable.
pub trait FrameProcessor: Send + Sync {
    /// Process a single frame through this stage
    ///
    /// # Arguments
    /// * `estimator` - Mutable reference to the estimator (may be modified)
    /// * `context` - Mutable frame context (may be updated)
    ///
    /// # Returns
    /// Processing time in milliseconds, or an error if processing failed
    fn process(&self, estimator: &mut Estimator, context: &mut FrameContext) -> Result<f64>;

    /// Get a human-readable name for this processor
    fn name(&self) -> &'static str;

    /// Get a description of what this processor does
    fn description(&self) -> &'static str {
        "Frame processor stage"
    }
}

/// Composable pipeline that chains multiple frame processors
///
/// This allows building complex processing workflows from simpler, reusable components.
/// The pipeline executes each processor in sequence, accumulating timing information.
pub struct ProcessingPipeline {
    processors: Vec<Box<dyn FrameProcessor>>,
    #[cfg(feature = "debug-logging")]
    name: String,
}

impl ProcessingPipeline {
    /// Create a new empty processing pipeline
    #[cfg(feature = "debug-logging")]
    pub fn new(name: impl Into<String>) -> Self {
        ProcessingPipeline {
            processors: Vec::new(),
            name: name.into(),
        }
    }

    #[cfg(not(feature = "debug-logging"))]
    pub fn new(_name: impl Into<String>) -> Self {
        ProcessingPipeline {
            processors: Vec::new(),
        }
    }

    /// Add a processor to the pipeline
    ///
    /// Processors are executed in the order they are added.
    pub fn add_processor(mut self, processor: Box<dyn FrameProcessor>) -> Self {
        self.processors.push(processor);
        self
    }

    /// Execute the pipeline on a frame
    ///
    /// Returns the total processing time in milliseconds.
    pub fn execute(&self, estimator: &mut Estimator, context: &mut FrameContext) -> Result<f64> {
        let mut total_time = 0.0;

        for processor in &self.processors {
            debug_log!(
                "[Pipeline::{}] Running stage: {}",
                self.name,
                processor.name()
            );
            let stage_time = processor.process(estimator, context)?;
            total_time += stage_time;

            trace_log!(
                "[Pipeline::{}] Stage '{}' took {:.2}ms",
                self.name,
                processor.name(),
                stage_time
            );
        }

        Ok(total_time)
    }

    /// Get the number of processors in the pipeline
    pub fn processor_count(&self) -> usize {
        self.processors.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct MockProcessor {
        name: &'static str,
        processing_time_ms: f64,
    }

    impl MockProcessor {
        fn new(name: &'static str, time_ms: f64) -> Self {
            MockProcessor {
                name,
                processing_time_ms: time_ms,
            }
        }
    }

    impl FrameProcessor for MockProcessor {
        fn process(&self, _estimator: &mut Estimator, _context: &mut FrameContext) -> Result<f64> {
            Ok(self.processing_time_ms)
        }

        fn name(&self) -> &'static str {
            self.name
        }
    }

    #[test]
    fn test_empty_pipeline() {
        let pipeline = ProcessingPipeline::new("test");
        assert_eq!(pipeline.processor_count(), 0);
    }

    #[test]
    fn test_pipeline_with_single_processor() {
        let pipeline = ProcessingPipeline::new("test")
            .add_processor(Box::new(MockProcessor::new("mock", 10.5)));
        assert_eq!(pipeline.processor_count(), 1);
    }

    #[test]
    fn test_pipeline_with_multiple_processors() {
        let pipeline = ProcessingPipeline::new("test")
            .add_processor(Box::new(MockProcessor::new("mock1", 10.0)))
            .add_processor(Box::new(MockProcessor::new("mock2", 20.0)));

        assert_eq!(pipeline.processor_count(), 2);
    }
}
