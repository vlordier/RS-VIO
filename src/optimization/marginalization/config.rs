//! Configuration types for marginalization

use na::DVector;
use nalgebra as na;
use serde::{Deserialize, Serialize};

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
        // Clamp damping to [1e-12, 1.0]
        if self.damping < 1e-12 {
            log::warn!(
                "marginalization damping {} too small, clamping to 1e-12",
                self.damping
            );
            self.damping = 1e-12;
        }
        if self.damping > 1.0 {
            log::warn!(
                "marginalization damping {} too large, clamping to 1.0",
                self.damping
            );
            self.damping = 1.0;
        }

        // Clamp max_keyframes to [2, 50]
        if self.max_keyframes < 2 {
            log::warn!(
                "max_keyframes {} too small, clamping to 2",
                self.max_keyframes
            );
            self.max_keyframes = 2;
        }
        if self.max_keyframes > 50 {
            log::warn!(
                "max_keyframes {} too large, clamping to 50",
                self.max_keyframes
            );
            self.max_keyframes = 50;
        }

        // Clamp num_marginalize_per_step to [1, max_keyframes/2]
        if self.num_marginalize_per_step == 0 {
            log::warn!("num_marginalize_per_step cannot be 0, setting to 1");
            self.num_marginalize_per_step = 1;
        }
        let max_marg = self.max_keyframes / 2;
        if self.num_marginalize_per_step > max_marg {
            log::warn!(
                "num_marginalize_per_step {} too large, clamping to {}",
                self.num_marginalize_per_step,
                max_marg
            );
            self.num_marginalize_per_step = max_marg;
        }

        // Clamp min_landmark_observations to [2, 20]
        if self.min_landmark_observations < 2 {
            log::warn!(
                "min_landmark_observations {} too small, clamping to 2",
                self.min_landmark_observations
            );
            self.min_landmark_observations = 2;
        }
        if self.min_landmark_observations > 20 {
            log::warn!(
                "min_landmark_observations {} too large, clamping to 20",
                self.min_landmark_observations
            );
            self.min_landmark_observations = 20;
        }

        // Clamp landmark_age_limit to [1, 1000]
        if self.landmark_age_limit == 0 {
            log::warn!("landmark_age_limit cannot be 0, setting to 1");
            self.landmark_age_limit = 1;
        }
        if self.landmark_age_limit > 1000 {
            log::warn!(
                "landmark_age_limit {} too large, clamping to 1000",
                self.landmark_age_limit
            );
            self.landmark_age_limit = 1000;
        }

        // Clamp prior_info_scale to [0.01, 10.0]
        if self.prior_info_scale < 0.01 {
            log::warn!(
                "prior_info_scale {} too small, clamping to 0.01",
                self.prior_info_scale
            );
            self.prior_info_scale = 0.01;
        }
        if self.prior_info_scale > 10.0 {
            log::warn!(
                "prior_info_scale {} too large, clamping to 10.0",
                self.prior_info_scale
            );
            self.prior_info_scale = 10.0;
        }

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
