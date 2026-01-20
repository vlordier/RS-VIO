//! Prior and cache types for marginalization

use nalgebra as na;
use na::DVector;
use std::collections::HashMap;

use super::config::ParamId;

/// Marginalization prior factor data
#[derive(Debug, Clone)]
pub struct MarginalizationPrior {
    /// Parameter IDs involved in the prior (in order)
    pub param_ids: Vec<ParamId>,
    /// Residual dimension
    pub residual_dim: usize,
    /// Residual vector (precomputed at linearization)
    pub residual: DVector<f64>,
    /// Information matrix (sparse, stored as dense for simplicity)
    pub information: na::DMatrix<f64>,
    /// Damping applied
    pub damping: f64,
    /// Linearization points (for FEJ)
    pub linearization_points: HashMap<ParamId, DVector<f64>>,
}

/// Linearization points cache for FEJ
#[derive(Debug, Clone, Default)]
pub struct FejCache {
    pub(crate) points: HashMap<ParamId, DVector<f64>>,
    structure_hash: u64,
}

impl FejCache {
    pub fn new() -> Self {
        Self {
            points: HashMap::new(),
            structure_hash: 0,
        }
    }

    pub fn set_point(&mut self, id: &ParamId, point: DVector<f64>) {
        self.points.insert(id.clone(), point);
    }

    pub fn get_point(&self, id: &ParamId) -> Option<&DVector<f64>> {
        self.points.get(id)
    }

    pub fn contains(&self, id: &ParamId) -> bool {
        self.points.contains_key(id)
    }

    pub fn update_structure_hash(&mut self, hash: u64) {
        self.structure_hash = hash;
    }

    pub fn structure_hash(&self) -> u64 {
        self.structure_hash
    }
}

/// Statistics for monitoring
#[derive(Debug, Clone, Default)]
pub struct MarginalizationStats {
    pub total_marginalizations: usize,
    pub total_prior_dim: usize,
    pub avg_schur_time_ms: f64,
    pub avg_prior_construction_ms: f64,
}
