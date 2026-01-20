/// Motion mode enumeration for adaptive filter behavior
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MotionMode {
    /// Hover mode: low motion, tighter filters
    Hover,
    /// Aggressive mode: high motion, looser filters
    Aggressive,
}
