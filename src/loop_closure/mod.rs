//! # Loop Closure Detection Module
//!
//! Implements robust loop closure detection for global SLAM consistency.
//!
//! ## Overview
//!
//! This module detects when the robot revisits a previously mapped location
//! and creates constraints in the pose graph to correct accumulated drift.
//!
//! ## Components
//!
//! - **Place Recognition** (7.1) - Find revisited locations via descriptor hashing
//! - **Geometric Verification** (7.2) - Confirm loops with epipolar geometry
//! - **Constraint Refinement** (7.3) - Optimize relative pose estimates
//! - **Graph Optimization** (7.4) - Correct trajectory globally
//!
//! ## Performance
//!
//! - **Place Recognition**: <50ms per query
//! - **Geometric Verification**: <200ms per candidate pair
//! - **Total Latency**: <500ms per loop
//!
//! ## Usage
//!
//! ```rust,ignore
//! use rs_vio::loop_closure::{
//!     PlaceRecognitionDatabase, PlaceRecognitionConfig,
//!     GeometricVerifier, GeometricVerificationConfig,
//! };
//!
//! let mut db = PlaceRecognitionDatabase::new(PlaceRecognitionConfig::default());
//! db.add_keyframe(0, &descriptors)?;
//! let candidates = db.query_candidates(&query_descriptors)?;
//!
//! let verifier = GeometricVerifier::new(GeometricVerificationConfig::default());
//! let result = verifier.verify_loop_closure(&matches);
//! ```

pub mod constraint_refinement;
pub mod geometric_verification;
pub mod graph_optimization;
pub mod place_recognition;

pub use constraint_refinement::{
    ConstraintRefiner, ConstraintRefinementConfig, InformationMatrix, RefinementResult,
    SE3Transform,
};
pub use geometric_verification::{
    EssentialMatrix, FeatureMatchGeom, GeometricVerificationConfig, GeometricVerifier,
    VerificationResult,
};
pub use graph_optimization::{
    BinaryConstraint, GraphOptimizationConfig, OptimizationResult, Pose, PoseEdge, PoseGraph,
    PoseVertex, TrajectoryStats, UnaryConstraint,
};
pub use place_recognition::{
    DatabaseStatistics, LoopCandidate, PlaceRecognitionConfig, PlaceRecognitionDatabase,
    PlaceStatistics,
};
