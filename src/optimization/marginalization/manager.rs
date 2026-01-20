//! Main marginalization manager with trait-based composition

use nalgebra as na;
use na::{linalg::LU, linalg::SVD, DMatrix, DVector};
use std::collections::{HashMap, HashSet};

use super::approximators::{
    DiagonalApproximator, ExactHessianApproximator, GaussNewtonApproximator,
    LevenbergMarquardtApproximator, IdentityApproximator, RegularizedPriorConstructor,
    StandardGradientComputer, StandardPriorConstructor, ZeroGradientComputer,
};
use super::config::{MarginalizationConfig, MarginalizationInfo, MarginalizationResult, ParamBlock, ParamId};
use super::prior::{FejCache, MarginalizationPrior, MarginalizationStats};
use super::traits::{GradientComputer, HessianApproximator, PriorConstructor};

/// Main marginalization manager with trait composition.
///
/// This struct composes the various approximation strategies through traits,
/// allowing for flexible and testable marginalization.
#[derive(Debug)]
pub struct MarginalizationManager {
    pub(crate) config: MarginalizationConfig,
    prior: Option<MarginalizationPrior>,
    pub(crate) fej_cache: FejCache,
    pub(crate) marginalized_params: HashSet<ParamId>,
    pub stats: MarginalizationStats,
    hessian_approximator: Box<dyn HessianApproximator>,
    gradient_computer: Box<dyn GradientComputer>,
    prior_constructor: Box<dyn PriorConstructor>,
}

impl MarginalizationManager {
    /// Create new marginalization manager with default strategies
    pub fn new(config: MarginalizationConfig) -> Self {
        let mut manager = Self {
            config,
            prior: None,
            fej_cache: FejCache::new(),
            marginalized_params: HashSet::new(),
            stats: MarginalizationStats::default(),
            hessian_approximator: Box::new(GaussNewtonApproximator::default()),
            gradient_computer: Box::new(StandardGradientComputer),
            prior_constructor: Box::new(StandardPriorConstructor),
        };

        manager.apply_config_strategies();
        manager
    }

    /// Create with default config and strategies
    pub fn default() -> Self {
        Self::new(MarginalizationConfig::default())
    }

    /// Apply strategy selection from configuration strings.
    pub fn apply_config_strategies(&mut self) {
        match self.config.hessian_approximator.as_str() {
            "Diagonal" => self.set_hessian_approximator(Box::new(DiagonalApproximator::default())),
            "LevenbergMarquardt" => {
                self.set_hessian_approximator(Box::new(LevenbergMarquardtApproximator::default()))
            },
            "Identity" => self.set_hessian_approximator(Box::new(IdentityApproximator)),
            "Exact" => self.set_hessian_approximator(Box::new(ExactHessianApproximator)),
            _ => self.set_hessian_approximator(Box::new(GaussNewtonApproximator::default())),
        }

        match self.config.gradient_computer.as_str() {
            "Zero" => self.set_gradient_computer(Box::new(ZeroGradientComputer)),
            _ => self.set_gradient_computer(Box::new(StandardGradientComputer)),
        }

        match self.config.prior_constructor.as_str() {
            "Regularized" => {
                self.set_prior_constructor(Box::new(RegularizedPriorConstructor::default()))
            },
            _ => self.set_prior_constructor(Box::new(StandardPriorConstructor)),
        }
    }

    /// Set Hessian approximator strategy
    pub fn set_hessian_approximator(&mut self, approximator: Box<dyn HessianApproximator>) {
        self.hessian_approximator = approximator;
    }

    /// Set gradient computer strategy
    pub fn set_gradient_computer(&mut self, computer: Box<dyn GradientComputer>) {
        self.gradient_computer = computer;
    }

    /// Set prior constructor strategy
    pub fn set_prior_constructor(&mut self, constructor: Box<dyn PriorConstructor>) {
        self.prior_constructor = constructor;
    }

    /// Get current Hessian approximator name
    pub fn hessian_approximator_name(&self) -> &str {
        self.hessian_approximator.name()
    }

    /// Get current gradient computer name
    pub fn gradient_computer_name(&self) -> &str {
        self.gradient_computer.name()
    }

    /// Get current prior constructor name
    pub fn prior_constructor_name(&self) -> &str {
        self.prior_constructor.name()
    }

    /// Check if prior exists
    pub fn has_prior(&self) -> bool {
        self.prior.is_some()
    }

    /// Check if marginalization should be performed
    pub fn should_marginalize(&self, window_size: usize) -> bool {
        window_size >= self.config.max_keyframes
    }

    /// Check if a parameter was marginalized
    pub fn is_marginalized(&self, id: &ParamId) -> bool {
        self.marginalized_params.contains(id)
    }

    /// Reset marginalization state
    pub fn reset(&mut self) {
        self.prior = None;
        self.fej_cache = FejCache::new();
        self.marginalized_params.clear();
    }

    /// Get the stored prior (for adding to optimizer)
    pub fn get_prior(&self) -> Option<&MarginalizationPrior> {
        self.prior.as_ref()
    }

    /// Get mutable prior reference (for updating)
    pub fn get_prior_mut(&mut self) -> Option<&mut MarginalizationPrior> {
        self.prior.as_mut()
    }

    /// Set the prior (called after marginalization)
    pub fn set_prior(&mut self, prior: MarginalizationPrior) {
        self.prior = Some(prior);
    }

    /// Compute approximate Hessian using configured strategy
    pub fn compute_approximate_hessian(
        &mut self,
        residuals: &DVector<f64>,
        param_dim: usize,
        jacobians: Option<&Vec<DMatrix<f64>>>,
    ) -> DMatrix<f64> {
        self.hessian_approximator
            .compute_hessian(residuals, param_dim, jacobians)
    }

    /// Compute gradient using configured strategy
    pub fn compute_gradient(
        &self,
        residuals: &DVector<f64>,
        param_dim: usize,
        jacobians: Option<&Vec<DMatrix<f64>>>,
    ) -> DVector<f64> {
        self.gradient_computer
            .compute_gradient(residuals, param_dim, jacobians)
    }

    /// Perform marginalization with composed strategies
    pub fn marginalize(
        &mut self,
        param_blocks: &HashMap<ParamId, ParamBlock>,
        hessian: &DMatrix<f64>,
        gradient: &DVector<f64>,
        keep_ids: &[ParamId],
        marg_ids: &[ParamId],
    ) -> MarginalizationResult {
        if !self.config.enabled {
            log::debug!("Marginalization disabled; skipping prior construction");
            return MarginalizationResult {
                prior: None,
                info: MarginalizationInfo::default(),
            };
        }

        let _start_time = std::time::Instant::now();

        // Build index maps FIRST
        let (keep_indices, marg_indices) = self.build_index_maps(param_blocks, keep_ids, marg_ids);

        // EARLY EXIT: If no parameters to marginalize, return before expensive computation
        if marg_indices.is_empty() {
            log::debug!("No parameters to marginalize; early exit");
            return MarginalizationResult {
                prior: None,
                info: MarginalizationInfo::default(),
            };
        }

        let _n_keep = keep_indices.len();
        let n_marg = marg_indices.len();

        #[allow(clippy::mutable_key_type)]
        {
            let declared = param_blocks.len();
            let accounted = keep_ids.len() + marg_ids.len();
            if declared != accounted {
                log::warn!(
                    "Param block coverage mismatch: {} declared vs {} accounted (keep+marg)",
                    declared,
                    accounted
                );
            }
        }

        // Compute total parameter dimension as sum of all block dimensions
        let total_params: usize = param_blocks.values().map(|b| b.dimension).sum();

        // Validate dimensions
        assert_eq!(
            hessian.nrows(),
            total_params,
            "Hessian dimension mismatch: expected {}, got {}",
            total_params,
            hessian.nrows()
        );
        assert_eq!(hessian.ncols(), total_params, "Hessian must be square");
        assert_eq!(gradient.len(), total_params, "Gradient dimension mismatch");

        // Partition Hessian: H = [H_aa H_ab; H_ba H_bb]
        // where a = marginalized, b = kept
        let (H_aa, H_ab, H_ba, H_bb) = self.partition_hessian(
            hessian,
            &keep_indices,
            &marg_indices,
            param_blocks,
            keep_ids,
            marg_ids,
        );

        // Partition gradient: b = [b_a; b_b]
        let (b_a, b_b) = self.partition_gradient(
            gradient,
            &keep_indices,
            &marg_indices,
            param_blocks,
            keep_ids,
            marg_ids,
        );

        // Ensure FEJ cache is consistent with current structure before constructing the prior
        self.update_fej_cache(param_blocks, keep_ids, marg_ids);

        // Compute Schur complement: S = H_aa - H_ab * H_bb^-1 * H_ba
        let schur_start = std::time::Instant::now();

        // Solve H_bb * X = [H_ba | b_b] using a stable decomposition pipeline
        let (H_bb_inv_H_ba, H_bb_inv_b_b) = self.solve_h_bb_system(&H_bb, &H_ba, &b_b);

        // Schur complement computation
        let schur_complement = &H_aa - &H_ab * &H_bb_inv_H_ba;

        // Reduced gradient: b_eff = b_a - H_ab * H_bb^-1 * b_b
        let reduced_gradient = &b_a - &H_ab * &H_bb_inv_b_b;

        let schur_time = schur_start.elapsed().as_secs_f64() * 1000.0;
        self.stats.avg_schur_time_ms =
            (self.stats.avg_schur_time_ms * self.stats.total_marginalizations as f64 + schur_time)
                / (self.stats.total_marginalizations + 1) as f64;

        // Construct prior using configured constructor
        let prior_construction_start = std::time::Instant::now();

        // Collect parameter IDs that were kept (the prior applies to the reduced system)
        // The Schur complement gives us the information about the kept parameters
        let prior_param_ids: Vec<ParamId> = keep_ids.to_vec();

        // If no parameters were kept, return empty result
        if prior_param_ids.is_empty() {
            log::debug!("No parameters to marginalize");
            return MarginalizationResult {
                prior: None,
                info: MarginalizationInfo {
                    schur_complement_time_ms: 0.0,
                    prior_construction_time_ms: 0.0,
                    states_marginalized: 0,
                    landmarks_marginalized: 0,
                    prior_residual_dim: 0,
                    condition_number: None,
                },
            };
        }

        log::debug!(
            "Marginalizing {} parameter blocks, Schur complement dim: {}x{}, reduced gradient dim: {}",
            prior_param_ids.len(),
            schur_complement.nrows(),
            schur_complement.ncols(),
            reduced_gradient.len()
        );

        let prior = self.prior_constructor.construct_prior(
            &schur_complement,
            &reduced_gradient,
            &prior_param_ids,
            schur_complement.nrows(),
            &self.config,
            &self.fej_cache.points,
        );

        let prior_time = prior_construction_start.elapsed().as_secs_f64() * 1000.0;
        self.stats.avg_prior_construction_ms = (self.stats.avg_prior_construction_ms
            * self.stats.total_marginalizations as f64
            + prior_time)
            / (self.stats.total_marginalizations + 1) as f64;

        // Update marginalized parameters
        for id in marg_ids {
            self.marginalized_params.insert(id.clone());
        }

        // Update statistics
        self.stats.total_marginalizations += 1;
        if let Some(ref prior) = prior {
            self.stats.total_prior_dim += prior.param_ids.len();
        }

        MarginalizationResult {
            prior,
            info: MarginalizationInfo {
                schur_complement_time_ms: schur_time,
                prior_construction_time_ms: prior_time,
                states_marginalized: n_marg,
                landmarks_marginalized: marg_ids
                    .iter()
                    .filter(|id| matches!(id, ParamId::Landmark(_)))
                    .filter(|id| param_blocks.contains_key(id))
                    .count(),
                prior_residual_dim: schur_complement.nrows(),
                condition_number: self.estimate_condition_number(&schur_complement),
            },
        }
    }

    /// Build index maps for partitioning
    fn build_index_maps(
        &self,
        param_blocks: &HashMap<ParamId, ParamBlock>,
        keep_ids: &[ParamId],
        marg_ids: &[ParamId],
    ) -> (Vec<usize>, Vec<usize>) {
        let mut param_to_index: HashMap<ParamId, usize> = HashMap::new();
        let mut current_idx = 0usize;

        for id in keep_ids {
            if let Some(block) = param_blocks.get(id) {
                param_to_index.insert(id.clone(), current_idx);
                current_idx += block.dimension;
            }
        }

        for id in marg_ids {
            if let Some(block) = param_blocks.get(id) {
                param_to_index.insert(id.clone(), current_idx);
                current_idx += block.dimension;
            }
        }

        let keep_indices: Vec<usize> = keep_ids
            .iter()
            .filter_map(|id| param_to_index.get(id).copied())
            .collect();

        let marg_indices: Vec<usize> = marg_ids
            .iter()
            .filter_map(|id| param_to_index.get(id).copied())
            .collect();

        (keep_indices, marg_indices)
    }

    /// Partition Hessian matrix
    fn partition_hessian(
        &self,
        H: &DMatrix<f64>,
        keep_indices: &[usize],
        marg_indices: &[usize],
        param_blocks: &HashMap<ParamId, ParamBlock>,
        keep_ids: &[ParamId],
        marg_ids: &[ParamId],
    ) -> (DMatrix<f64>, DMatrix<f64>, DMatrix<f64>, DMatrix<f64>) {
        let keep_elem_indices =
            self.expand_block_indices(keep_indices, param_blocks, keep_ids, marg_ids);
        let marg_elem_indices =
            self.expand_block_indices(marg_indices, param_blocks, keep_ids, marg_ids);

        let H_aa = self.extract_dense_submatrix(H, &marg_elem_indices, &marg_elem_indices);
        let H_ab = self.extract_dense_submatrix(H, &marg_elem_indices, &keep_elem_indices);
        let H_ba = self.extract_dense_submatrix(H, &keep_elem_indices, &marg_elem_indices);
        let H_bb = self.extract_dense_submatrix(H, &keep_elem_indices, &keep_elem_indices);
        (H_aa, H_ab, H_ba, H_bb)
    }

    fn expand_block_indices(
        &self,
        block_indices: &[usize],
        param_blocks: &HashMap<ParamId, ParamBlock>,
        keep_ids: &[ParamId],
        marg_ids: &[ParamId],
    ) -> Vec<usize> {
        let mut param_vec: Vec<(usize, usize)> = Vec::new();
        let mut current_pos = 0usize;

        for id in keep_ids.iter() {
            if let Some(block) = param_blocks.get(id) {
                param_vec.push((current_pos, block.dimension));
                current_pos += block.dimension;
            }
        }
        for id in marg_ids.iter() {
            if let Some(block) = param_blocks.get(id) {
                param_vec.push((current_pos, block.dimension));
                current_pos += block.dimension;
            }
        }

        let mut element_indices = Vec::new();
        for &block_start in block_indices {
            for &(start, dim) in &param_vec {
                if start == block_start {
                    for i in 0..dim {
                        element_indices.push(start + i);
                    }
                    break;
                }
            }
        }
        element_indices
    }

    fn extract_dense_submatrix(
        &self,
        matrix: &DMatrix<f64>,
        row_indices: &[usize],
        col_indices: &[usize],
    ) -> DMatrix<f64> {
        let nrows = row_indices.len();
        let ncols = col_indices.len();
        let mut result = DMatrix::zeros(nrows, ncols);

        for (i, &row_idx) in row_indices.iter().enumerate() {
            for (j, &col_idx) in col_indices.iter().enumerate() {
                result[(i, j)] = matrix[(row_idx, col_idx)];
            }
        }
        result
    }

    /// Partition gradient vector
    fn partition_gradient(
        &self,
        b: &DVector<f64>,
        keep_indices: &[usize],
        marg_indices: &[usize],
        param_blocks: &HashMap<ParamId, ParamBlock>,
        keep_ids: &[ParamId],
        marg_ids: &[ParamId],
    ) -> (DVector<f64>, DVector<f64>) {
        let keep_elem_indices =
            self.expand_block_indices(keep_indices, param_blocks, keep_ids, marg_ids);
        let marg_elem_indices =
            self.expand_block_indices(marg_indices, param_blocks, keep_ids, marg_ids);

        let b_a: DVector<f64> = DVector::from_iterator(
            marg_elem_indices.len(),
            marg_elem_indices.iter().map(|&i| b[i]),
        );
        let b_b: DVector<f64> = DVector::from_iterator(
            keep_elem_indices.len(),
            keep_elem_indices.iter().map(|&i| b[i]),
        );
        (b_a, b_b)
    }

    /// Estimate condition number using fast O(n) heuristic.
    ///
    /// Uses Frobenius norm divided by minimum diagonal element. This is better than
    /// the prior Frobenius/trace ratio because it properly detects diagonal dominance issues.
    /// Still O(n), still an approximation, but correctly identifies singular matrices.
    ///
    /// Returns None if matrix is singular (min_diag <= 1e-14).
    pub fn estimate_condition_number(&self, matrix: &DMatrix<f64>) -> Option<f64> {
        if matrix.nrows() == 0 || matrix.ncols() == 0 {
            return None;
        }

        let frob = matrix.norm();
        let min_diag = (0..matrix.nrows().min(matrix.ncols()))
            .map(|i| matrix[(i, i)].abs())
            .fold(f64::INFINITY, f64::min);

        // If min diagonal is too small, matrix is effectively singular
        if min_diag > 1e-14 {
            Some(frob / min_diag)
        } else {
            // Matrix is singular; fall back to SVD for diagnostic (offline only)
            self.estimate_condition_number_svd(matrix)
        }
    }

    /// Estimate condition number using SVD (expensive, for offline diagnostics only).
    /// **Do not use in real-time loops.**
    #[allow(dead_code)] // Used in benchmarks / offline tools
    fn estimate_condition_number_svd(&self, matrix: &DMatrix<f64>) -> Option<f64> {
        if matrix.nrows() == 0 || matrix.ncols() == 0 {
            return None;
        }

        // Full SVD is O(n³); only acceptable for offline analysis
        let svd = SVD::new(matrix.clone(), false, false);
        let singulars = svd.singular_values;
        if singulars.is_empty() {
            return None;
        }
        let max_sv = singulars.max();
        let min_sv = singulars
            .iter()
            .copied()
            .filter(|sv| *sv > f64::EPSILON * max_sv)
            .fold(f64::INFINITY, f64::min);
        if min_sv.is_finite() && min_sv > 0.0 {
            Some(max_sv / min_sv)
        } else {
            None
        }
    }

    /// Solve H_bb * X = [H_ba | b_b] with a robust fallback pipeline.
    ///
    /// Strategy: Try fast methods first (Cholesky), escalate damping, then resort to slower but more robust
    /// methods (LU, pseudo-inverse) only if necessary.
    ///
    /// Memory: Minimize clones to avoid heap fragmentation on embedded devices.
    /// Time: Target <5ms on Jetson Xavier for typical 84×84 Schur blocks.
    fn solve_h_bb_system(
        &self,
        H_bb: &DMatrix<f64>,
        H_ba: &DMatrix<f64>,
        b_b: &DVector<f64>,
    ) -> (DMatrix<f64>, DVector<f64>) {
        const MAX_DAMPING_SCALE: f64 = 1e3; // Prevent unbounded regularization

        if H_bb.nrows() == 0 {
            return (
                DMatrix::zeros(H_bb.nrows(), H_ba.ncols()),
                DVector::zeros(b_b.len()),
            );
        }

        let mut regularized = H_bb.clone();
        let mut damping_scale = 1.0;

        // Attempt 1: Cholesky with base damping (fastest path, ~0.5ms for 84×84)
        Self::add_diagonal_damping(&mut regularized, self.config.damping);
        if let Some(chol) = na::linalg::Cholesky::new(regularized.clone()) {
            return (chol.solve(H_ba), chol.solve(b_b));
        }

        // Attempt 2: Cholesky with escalated damping (single re-clone per attempt)
        for attempt in 1..4 {
            damping_scale *= 10.0;
            if damping_scale > MAX_DAMPING_SCALE {
                log::warn!(
                    "H_bb damping exceeded {:.0e} (scale {:.0e}); switching to LU fallback",
                    self.config.damping * MAX_DAMPING_SCALE,
                    damping_scale / 10.0
                );
                break; // Exit loop, proceed to LU attempt
            }

            // Re-clone only on escalation attempt, not on every iteration
            regularized = H_bb.clone();
            Self::add_diagonal_damping(&mut regularized, self.config.damping * damping_scale);

            if let Some(chol) = na::linalg::Cholesky::new(regularized.clone()) {
                if attempt > 0 {
                    log::debug!(
                        "H_bb Cholesky succeeded at damping scale {:.0e}",
                        damping_scale
                    );
                }
                return (chol.solve(H_ba), chol.solve(b_b));
            }
        }

        // Attempt 3: LU factorization (slower, more robust)
        let lu = LU::new(regularized.clone());
        if lu.is_invertible() {
            if let (Some(mat_sol), Some(vec_sol)) = (lu.solve(H_ba), lu.solve(b_b)) {
                log::warn!(
                    "Using LU fallback for H_bb (damping scale {:.0e}); solution quality degraded",
                    damping_scale
                );
                return (mat_sol, vec_sol);
            }
        }

        // Fallback: Pseudo-inverse (slowest, last resort; solution error expected ~1e-6)
        log::error!(
            "H_bb critically ill-conditioned even with damping {:.0e}; using pseudo-inverse",
            damping_scale
        );
        let pinv = self.pseudo_inverse(&regularized);
        let H_bb_inv_H_ba = &pinv * H_ba;
        let H_bb_inv_b_b = &pinv * b_b;
        (H_bb_inv_H_ba, H_bb_inv_b_b)
    }

    /// Compute pseudo-inverse via SVD with conservative rank detection.
    ///
    /// **Warning**: This is the last-resort solver; solution accuracy is already degraded.
    /// Threshold uses relative tolerance 1e-10 (IEEE double precision standard) rather than
    /// machine epsilon to avoid inverting tiny singular values that would cause huge errors.
    fn pseudo_inverse(&self, matrix: &DMatrix<f64>) -> DMatrix<f64> {
        let svd = SVD::new(matrix.clone(), true, true);
        let (Some(u), Some(v_t)) = (svd.u, svd.v_t) else {
            log::warn!("SVD decomposition failed; returning identity pseudo-inverse");
            return DMatrix::identity(matrix.nrows(), matrix.ncols());
        };

        let mut s_inv = DMatrix::zeros(v_t.nrows(), u.ncols());
        let singulars = svd.singular_values;
        if singulars.is_empty() {
            return DMatrix::identity(matrix.nrows(), matrix.ncols());
        }

        let max_sv = singulars.max();
        // Conservative tolerance: 1e-10 × max_sv (relative tolerance).
        // Avoids machine-epsilon threshold which would invert singular values with huge reciprocals.
        let tol = 1e-10 * max_sv;

        let mut rank = 0;
        for (i, sv) in singulars.iter().enumerate() {
            if *sv > tol {
                s_inv[(i, i)] = 1.0 / sv;
                rank += 1;
            }
        }

        if rank < singulars.len() {
            log::warn!(
                "Pseudo-inverse: effective rank {} / {}; {} singular values dropped (tol={:.2e})",
                rank,
                singulars.len(),
                singulars.len() - rank,
                tol
            );
        }

        v_t.transpose() * s_inv * u.transpose()
    }

    fn add_diagonal_damping(matrix: &mut DMatrix<f64>, damping: f64) {
        for i in 0..matrix.nrows().min(matrix.ncols()) {
            matrix[(i, i)] += damping;
        }
    }

    fn update_fej_cache(
        &mut self,
        param_blocks: &HashMap<ParamId, ParamBlock>,
        keep_ids: &[ParamId],
        marg_ids: &[ParamId],
    ) {
        let structure_hash = Self::compute_structure_hash(param_blocks);
        let active_ids: HashSet<ParamId> = keep_ids
            .iter()
            .cloned()
            .chain(marg_ids.iter().cloned())
            .collect();

        if self.config.use_fej {
            if self.fej_cache.structure_hash() != structure_hash {
                self.fej_cache
                    .points
                    .retain(|id, _| active_ids.contains(id));
                self.fej_cache.update_structure_hash(structure_hash);
            }

            for id in active_ids.iter() {
                if !self.fej_cache.contains(id) {
                    if let Some(block) = param_blocks.get(id) {
                        self.fej_cache
                            .set_point(id, block.linearization_point.clone());
                    }
                }
            }
        } else {
            self.fej_cache.points.clear();
            for id in active_ids.iter() {
                if let Some(block) = param_blocks.get(id) {
                    self.fej_cache
                        .set_point(id, block.linearization_point.clone());
                }
            }
            self.fej_cache.update_structure_hash(structure_hash);
        }
    }

    /// Compute structure hash from parameter blocks (includes dimensions).
    /// This ensures cache invalidation if either IDs or dimensions change.
    fn compute_structure_hash(param_blocks: &HashMap<ParamId, ParamBlock>) -> u64 {
        use std::hash::{Hash, Hasher};
        let mut hasher = std::collections::hash_map::DefaultHasher::new();
        let mut entries: Vec<_> = param_blocks.iter().collect();
        entries.sort_by(|a, b| a.0.cmp(b.0)); // Sort by ID
        for (id, block) in entries {
            id.hash(&mut hasher);
            block.dimension.hash(&mut hasher); // ← Include dimension!
        }
        hasher.finish()
    }

    /// Perform marginalization with approximation
    pub fn marginalize_with_approximation(
        &mut self,
        param_blocks: &HashMap<ParamId, ParamBlock>,
        residuals: &DVector<f64>,
        jacobians: Option<&Vec<DMatrix<f64>>>,
        keep_ids: &[ParamId],
        marg_ids: &[ParamId],
    ) -> Option<MarginalizationPrior> {
        // Compute total parameter dimension
        let total_dim: usize = param_blocks.values().map(|b| b.dimension).sum();

        // Compute approximate Hessian and gradient using strategies
        let hessian = self.compute_approximate_hessian(residuals, total_dim, jacobians);
        let gradient = self.compute_gradient(residuals, total_dim, jacobians);

        // Perform marginalization
        let result = self.marginalize(param_blocks, &hessian, &gradient, keep_ids, marg_ids);

        result.prior
    }
}
