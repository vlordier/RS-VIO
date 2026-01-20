/// Core metric types and definitions
///
/// Defines fundamental data structures for storing and computing binned metrics.

/// Per-bin accuracy metrics
#[derive(Clone, Debug)]
pub struct BinMetrics {
    /// Mean accuracy/error
    pub mean: f32,

    /// Standard deviation
    pub std: f32,

    /// Min value in bin
    pub min: f32,

    /// Max value in bin
    pub max: f32,

    /// Median value
    pub median: f32,

    /// Sample count
    pub count: usize,

    /// Percentile 95
    pub p95: f32,

    /// Percentile 99
    pub p99: f32,
}

impl BinMetrics {
    pub fn new() -> Self {
        Self {
            mean: 0.0,
            std: 0.0,
            min: 0.0,
            max: 0.0,
            median: 0.0,
            count: 0,
            p95: 0.0,
            p99: 0.0,
        }
    }

    pub fn add_sample(&mut self, _value: f32) {
        // Simplified: just track count for now
        // Finalize should be called to compute statistics
        self.count += 1;
    }

    /// Compute final statistics from samples
    pub fn finalize(&mut self, samples: &[f32]) {
        if samples.is_empty() {
            return;
        }

        self.count = samples.len();

        let sum: f32 = samples.iter().sum();
        self.mean = sum / samples.len() as f32;

        let variance: f32 = samples
            .iter()
            .map(|v| (v - self.mean).powi(2))
            .sum::<f32>()
            / samples.len() as f32;
        self.std = variance.sqrt();

        self.min = samples
            .iter()
            .copied()
            .min_by(|a, b| a.partial_cmp(b).unwrap())
            .unwrap_or(0.0);

        self.max = samples
            .iter()
            .copied()
            .max_by(|a, b| a.partial_cmp(b).unwrap())
            .unwrap_or(0.0);

        let mut sorted = samples.to_vec();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());

        self.median = sorted[sorted.len() / 2];

        let p95_idx = (sorted.len() as f32 * 0.95) as usize;
        self.p95 = sorted.get(p95_idx).copied().unwrap_or(0.0);

        let p99_idx = (sorted.len() as f32 * 0.99) as usize;
        self.p99 = sorted.get(p99_idx).copied().unwrap_or(0.0);
    }
}

impl Default for BinMetrics {
    fn default() -> Self {
        Self::new()
    }
}
