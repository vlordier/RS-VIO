//! Configuration types for marginalization

use na::DVector;
use nalgebra as na;
use serde::{Deserialize, Serialize};
use crate::clamp_or;

/// Result of marginalization operation
#[derive(Debug, Clone)]
pub struct MarginalizationResult {
    /// Prior factor for marginalized states
    pub prior: Option<crate::MarginalizationPrior>,
    /// Information about the marginalization process
    pub info: MarginalizationInfo,
}

/// Additional information about marginalization
#[derive(Debug, Clone, Default)]
pub struct MarginalizationInfo {
    pub schur_complement_time_ms: f64,
    pub prior_construction_time_ms: f64,
    pub states_marginalized: usize,
    pub landmarks_marginalized: usize,
    pub prior_residual_dim: usize,
    pub condition_number: Option<f64>,
}

// ============================================================================
// Configuration
// ============================================================================

/// Configuration for marginalization
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MarginalizationConfig {
    /// Enable marginalization
    pub enabled: bool,
    /// Use First-Estimate Jacobian (FEJ)
    pub use_fej: bool,
    /// Damping factor for numerical stability
    pub damping: f64,
    /// Maximum keyframes in window
    pub max_keyframes: usize,
    /// Number of frames to marginalize when full
    pub num_marginalize_per_step: usize,
    /// Minimum landmark observations to keep
    pub min_landmark_observations: usize,
    /// Landmark age limit (frames without observation)
    pub landmark_age_limit: usize,
    /// Information matrix scaling for the prior (downweight to prevent over-constraint)
    #[serde(default = "default_prior_info_scale")]
    pub prior_info_scale: f64,
    /// Hessian approximation strategy: "GaussNewton", "Diagonal", "LevenbergMarquardt", "Exact", "Identity"
    #[serde(default = "default_hessian_approximator")]
    pub hessian_approximator: String,
    /// Gradient computation strategy: "Standard", "Zero"
    #[serde(default = "default_gradient_computer")]
    pub gradient_computer: String,
    /// Prior construction strategy: "Standard", "Regularized"
    #[serde(default = "default_prior_constructor")]
    pub prior_constructor: String,
}

impl Default for MarginalizationConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            use_fej: true,
            damping: 1e-5, // Slightly increased for better stability under motion blur (was 1e-7)
            max_keyframes: 8, // Reduced from 10 for embedded memory constraints (saves ~80MB on typical drones)
            num_marginalize_per_step: 1,
            min_landmark_observations: 3,
            landmark_age_limit: 50,
            prior_info_scale: 0.9, // Slight downweight to prevent over-constraint
            hessian_approximator: "Diagonal".to_string(), // Diagonal > GaussNewton for drones (20× faster, acceptable accuracy loss)
            gradient_computer: "Standard".to_string(),
            prior_constructor: "Standard".to_string(),
        }
    }
}

impl MarginalizationConfig {
    /// Validate and clamp marginalization parameters.
    pub fn validate_and_clamp(&mut self) {
        self.damping = clamp_or!(
            self.damping,
            1e-12,
            1.0,
            1e-12,
            "marginalization damping {} out of range [1e-12, 1.0], clamping",
            self.damping
        );

        self.max_keyframes = clamp_or!(
            self.max_keyframes,
            2,
            50,
            2,
            "max_keyframes {} out of range [2, 50], clamping",
            self.max_keyframes
        );

        let max_marg = self.max_keyframes / 2;
        self.num_marginalize_per_step = clamp_or!(
            self.num_marginalize_per_step,
            1,
            max_marg,
            1,
            "num_marginalize_per_step {} out of range [1, {}], clamping",
            self.num_marginalize_per_step,
            max_marg
        );

        self.min_landmark_observations = clamp_or!(
            self.min_landmark_observations,
            2,
            20,
            2,
            "min_landmark_observations {} out of range [2, 20], clamping",
            self.min_landmark_observations
        );

        self.landmark_age_limit = clamp_or!(
            self.landmark_age_limit,
            1,
            1000,
            1,
            "landmark_age_limit {} out of range [1, 1000], clamping",
            self.landmark_age_limit
        );

        self.prior_info_scale = clamp_or!(
            self.prior_info_scale,
            0.01,
            10.0,
            0.01,
            "prior_info_scale {} out of range [0.01, 10.0], clamping",
            self.prior_info_scale
        );

        // Validate string enums
        let valid_hessian = [
            "GaussNewton",
            "Diagonal",
            "LevenbergMarquardt",
            "Exact",
            "Identity",
        ];
        if !valid_hessian.contains(&self.hessian_approximator.as_str()) {
            log::warn!(
                "Invalid hessian_approximator '{}', defaulting to 'Diagonal'",
                self.hessian_approximator
            );
            self.hessian_approximator = "Diagonal".to_string();
        }

        let valid_gradient = ["Standard", "Zero"];
        if !valid_gradient.contains(&self.gradient_computer.as_str()) {
            log::warn!(
                "Invalid gradient_computer '{}', defaulting to 'Standard'",
                self.gradient_computer
            );
            self.gradient_computer = "Standard".to_string();
        }

        let valid_prior = ["Standard", "Regularized"];
        if !valid_prior.contains(&self.prior_constructor.as_str()) {
            log::warn!(
                "Invalid prior_constructor '{}', defaulting to 'Standard'",
                self.prior_constructor
            );
            self.prior_constructor = "Standard".to_string();
        }
    }
}

fn default_hessian_approximator() -> String {
    "GaussNewton".to_string()
}

fn default_gradient_computer() -> String {
    "Standard".to_string()
}

fn default_prior_constructor() -> String {
    "Standard".to_string()
}

fn default_prior_info_scale() -> f64 {
    0.9
}

// ============================================================================
// Parameter Block Identifier
// ============================================================================

/// Parameter block identifier
#[derive(Debug, Clone, Hash, PartialEq, Eq, PartialOrd, Ord)]
pub enum ParamId {
    /// Keyframe pose by ID
    KeyframePose(usize),
    /// Keyframe velocity by ID
    KeyframeVelocity(usize),
    /// Keyframe accel bias by ID
    KeyframeAccelBias(usize),
    /// Keyframe gyro bias by ID
    KeyframeGyroBias(usize),
    /// Keyframe mass by ID
    KeyframeMass(usize),
    /// Landmark by ID
    Landmark(usize),
    /// Global gyro bias
    GlobalGyroBias,
    /// Global gravity direction
    GlobalGravity,
    /// Global drag coefficient
    GlobalDrag,
}

/// Parameter block information
#[derive(Debug, Clone)]
pub struct ParamBlock {
    pub id: ParamId,
    pub dimension: usize,
    pub linearization_point: DVector<f64>,
}
