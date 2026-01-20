//! # ORB (Oriented FAST and Rotated BRIEF) Descriptor
//!
//! Implements the ORB descriptor from:
//! "ORB: An Efficient Alternative to SIFT or SURF" (Rublee et al., 2011)
//!
//! Features:
//! - Rotation-invariant: Orientation computed from intensity centroid
//! - Scale-invariant: Multi-scale image pyramid
//! - Efficient: Binary descriptor (256 bits = 32 bytes)
//! - Fast matching: Hamming distance computation
//!
//! ## Algorithm Overview
//!
//! 1. **Corner Detection**: FAST9 corners on image pyramid
//! 2. **Orientation Estimation**: Intensity centroid method
//! 3. **Descriptor Extraction**: Rotated BRIEF pattern
//! 4. **Matching**: Hamming distance with ratio test

mod descriptor;
mod detector;
mod extractor;
mod types;

#[cfg(test)]
mod tests;

// Re-export main public API
pub use extractor::OrbExtractor;
pub use types::{OrbConfig, OrbFeature};
