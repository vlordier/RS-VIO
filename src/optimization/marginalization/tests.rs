#[cfg(test)]
#[allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    clippy::bool_assert_comparison,
    clippy::field_reassign_with_default
)]
mod tests {
    use std::collections::HashMap;
    use nalgebra as na;

    use crate::optimization::marginalization::{
        approximators::*,
        config::*,
        manager::MarginalizationManager,
        prior::*,
        traits::{GradientComputer, HessianApproximator, PriorConstructor},
        utils::select_marginalization_candidates,
    };

    #[test]
    fn test_manager_creation() {
        let manager = MarginalizationManager::default();
        assert!(!manager.has_prior());
        assert!(!manager.has_prior());
        assert_eq!(manager.hessian_approximator_name(), "Diagonal"); // Changed: embedded default
        assert_eq!(manager.gradient_computer_name(), "Standard");
        assert_eq!(manager.prior_constructor_name(), "Standard");
        assert_eq!(manager.stats.total_marginalizations, 0);
    }

    #[test]
    fn test_manager_creation_with_custom_config() {
        let config = MarginalizationConfig {
            enabled: true,
            use_fej: false,
            damping: 1e-5,
            max_keyframes: 15,
            num_marginalize_per_step: 2,
            min_landmark_observations: 5,
            landmark_age_limit: 100,
            prior_info_scale: 2.0,
            hessian_approximator: "Diagonal".to_string(),
            gradient_computer: "Zero".to_string(),
            prior_constructor: "Regularized".to_string(),
        };
        let manager = MarginalizationManager::new(config.clone());
        assert_eq!(manager.config.enabled, true);
        assert_eq!(manager.config.use_fej, false);
        assert_eq!(manager.config.damping, 1e-5);
        assert_eq!(manager.config.max_keyframes, 15);
        assert_eq!(manager.config.num_marginalize_per_step, 2);
    }

    #[test]
    fn test_set_approximators() {
        let mut manager = MarginalizationManager::default();
        manager.set_hessian_approximator(Box::new(DiagonalApproximator::new(1e-8)));
        assert_eq!(manager.hessian_approximator_name(), "Diagonal");

        manager.set_gradient_computer(Box::new(ZeroGradientComputer));
        assert_eq!(manager.gradient_computer_name(), "Zero");

        manager.set_prior_constructor(Box::new(RegularizedPriorConstructor::new(1e-8)));
        assert_eq!(manager.prior_constructor_name(), "Regularized");
    }

    #[test]
    fn test_should_marginalize() {
        let manager = MarginalizationManager::default();
        assert!(!manager.should_marginalize(5));
        assert!(!manager.should_marginalize(7));
        assert!(manager.should_marginalize(8)); // max_keyframes changed from 10 to 8 (embedded default)
        assert!(manager.should_marginalize(15));
    }

    #[test]
    fn test_reset() {
        let mut manager = MarginalizationManager::default();
        assert!(!manager.has_prior());

        let prior = crate::MarginalizationPrior {
            param_ids: vec![ParamId::KeyframePose(0)],
            residual_dim: 7,
            residual: na::DVector::zeros(7),
            information: na::DMatrix::identity(7, 7),
            damping: 1e-7,
            linearization_points: HashMap::new(),
        };
        manager.set_prior(prior);
        assert!(manager.has_prior());

        manager.reset();
        assert!(!manager.has_prior());
    }

    #[test]
    fn test_is_marginalized() {
        let mut manager = MarginalizationManager::default();
        assert!(!manager.is_marginalized(&ParamId::KeyframePose(0)));

        manager.marginalized_params.insert(ParamId::KeyframePose(0));
        assert!(manager.is_marginalized(&ParamId::KeyframePose(0)));
        assert!(!manager.is_marginalized(&ParamId::KeyframeVelocity(0)));
    }

    #[test]
    fn test_get_prior() {
        let mut manager = MarginalizationManager::default();
        assert!(manager.get_prior().is_none());

        let prior = crate::MarginalizationPrior {
            param_ids: vec![ParamId::KeyframePose(0)],
            residual_dim: 7,
            residual: na::DVector::from_vec(vec![1.0; 7]),
            information: na::DMatrix::identity(7, 7),
            damping: 1e-7,
            linearization_points: HashMap::new(),
        };
        manager.set_prior(prior.clone());

        let retrieved = manager.get_prior().expect("Should have prior");
        assert_eq!(retrieved.param_ids, prior.param_ids);
        assert_eq!(retrieved.residual_dim, prior.residual_dim);
    }

    #[test]
    fn test_fej_cache() {
        let mut cache = FejCache::new();
        assert!(cache.get_point(&ParamId::KeyframePose(0)).is_none());
        assert!(!cache.contains(&ParamId::KeyframePose(0)));

        let point = na::DVector::from_vec(vec![1.0, 2.0, 3.0]);
        cache.set_point(&ParamId::KeyframePose(0), point.clone());
        assert!(cache.contains(&ParamId::KeyframePose(0)));

        let retrieved = cache
            .get_point(&ParamId::KeyframePose(0))
            .expect("Should exist");
        assert_eq!(retrieved, &point);

        cache.update_structure_hash(12345);
        assert_eq!(cache.structure_hash(), 12345);
    }

    #[test]
    fn test_param_block() {
        let block = ParamBlock {
            id: ParamId::KeyframePose(5),
            dimension: 7,
            linearization_point: na::DVector::from_vec(vec![1.0; 7]),
        };
        assert_eq!(block.dimension, 7);
        assert!(matches!(block.id, ParamId::KeyframePose(5)));
    }

    #[test]
    fn test_marginalization_info() {
        let info = MarginalizationInfo {
            schur_complement_time_ms: 1.5,
            prior_construction_time_ms: 0.5,
            states_marginalized: 3,
            landmarks_marginalized: 10,
            prior_residual_dim: 21,
            condition_number: Some(1e6),
        };
        assert_eq!(info.states_marginalized, 3);
        assert_eq!(info.landmarks_marginalized, 10);
        assert_eq!(info.condition_number, Some(1e6));

        let empty_info = MarginalizationInfo::default();
        assert_eq!(empty_info.schur_complement_time_ms, 0.0);
        assert!(empty_info.condition_number.is_none());
    }

    #[test]
    fn test_marginalization_result() {
        let prior = crate::MarginalizationPrior {
            param_ids: vec![ParamId::KeyframePose(0)],
            residual_dim: 7,
            residual: na::DVector::zeros(7),
            information: na::DMatrix::identity(7, 7),
            damping: 1e-7,
            linearization_points: HashMap::new(),
        };
        let result = MarginalizationResult {
            prior: Some(prior),
            info: MarginalizationInfo::default(),
        };
        assert!(result.prior.is_some());

        let empty_result: MarginalizationResult = MarginalizationResult {
            prior: None,
            info: MarginalizationInfo::default(),
        };
        assert!(empty_result.prior.is_none());
    }

    #[test]
    fn test_gauss_newton_approximator_fallback() {
        let approximator = GaussNewtonApproximator::default();
        let residuals = na::DVector::from_vec(vec![1.0, 2.0, 3.0]);
        let hessian = approximator.compute_hessian(&residuals, 3, None);

        assert_eq!(hessian.nrows(), 3);
        assert_eq!(hessian.ncols(), 3);
        assert!(hessian.iter().all(|&x| x >= 0.0));
        assert!(hessian[(0, 0)] > 0.0);
        assert!(hessian[(1, 1)] > 0.0);
        assert!(hessian[(2, 2)] > 0.0);
        for i in 0..3 {
            for j in 0..3 {
                if i != j {
                    assert!(
                        (hessian[(i, j)]).abs() < 1e-10,
                        "Off-diagonal [{}, {}] = {} should be ~0",
                        i,
                        j,
                        hessian[(i, j)]
                    );
                }
            }
        }
    }

    #[test]
    fn test_gauss_newton_approximator_with_jacobians() {
        let approximator = GaussNewtonApproximator::default();
        let residuals = na::DVector::from_vec(vec![1.0, 2.0]);
        let jacobians: Vec<na::DMatrix<f64>> = vec![
            na::DMatrix::from_vec(2, 4, vec![1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0]),
            na::DMatrix::from_vec(2, 4, vec![0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0]),
        ];
        let hessian = approximator.compute_hessian(&residuals, 4, Some(&jacobians));

        assert_eq!(hessian.nrows(), 4);
        assert_eq!(hessian.ncols(), 4);
        let jt_j: na::DMatrix<f64> =
            jacobians[0].transpose() * &jacobians[0] + jacobians[1].transpose() * &jacobians[1];
        assert!((hessian - jt_j).norm() < 1e-10);
    }

    #[test]
    fn test_gauss_newton_empty_jacobians() {
        let approximator = GaussNewtonApproximator::default();
        let residuals = na::DVector::from_vec(vec![1.0, 2.0, 3.0]);
        let hessian = approximator.compute_hessian(&residuals, 3, Some(&vec![]));

        assert_eq!(hessian.nrows(), 3);
        assert_eq!(hessian.ncols(), 3);
        assert!(hessian.iter().all(|&x| x == 0.0));
    }

    #[test]
    fn test_identity_approximator() {
        let approximator = IdentityApproximator;
        let residuals = na::DVector::from_vec(vec![1.0, 2.0, 3.0]);
        let hessian = approximator.compute_hessian(&residuals, 3, None);

        assert_eq!(hessian.nrows(), 3);
        assert_eq!(hessian.ncols(), 3);
        let scale = residuals.norm() / residuals.len() as f64;
        let expected = na::DMatrix::identity(3, 3) * scale.max(1e-6);
        assert!((hessian - expected).norm() < 1e-10);
    }

    #[test]
    fn test_identity_approximator_with_jacobians_ignored() {
        let approximator = IdentityApproximator;
        let residuals = na::DVector::from_vec(vec![1.0, 2.0, 3.0]);
        let jacobians: Vec<na::DMatrix<f64>> = vec![na::DMatrix::from_vec(
            3,
            4,
            vec![1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
        )];
        let hessian = approximator.compute_hessian(&residuals, 4, Some(&jacobians));

        assert_eq!(hessian.nrows(), 4);
        assert_eq!(hessian.ncols(), 4);
        for i in 0..4 {
            for j in 0..4 {
                if i == j {
                    assert!(hessian[(i, j)] > 0.0);
                } else {
                    assert!((hessian[(i, j)]).abs() < 1e-10);
                }
            }
        }
    }

    #[test]
    fn test_diagonal_approximator_fallback() {
        let approximator = DiagonalApproximator::new(1e-8);
        let residuals = na::DVector::from_vec(vec![1.0, 2.0, 3.0]);
        let hessian = approximator.compute_hessian(&residuals, 3, None);

        assert_eq!(hessian.nrows(), 3);
        assert_eq!(hessian.ncols(), 3);
        let scale = residuals.norm() / residuals.len() as f64;
        let expected = na::DMatrix::identity(3, 3) * scale.max(1e-8);
        assert!((hessian - expected).norm() < 1e-10);
    }

    #[test]
    fn test_diagonal_approximator_with_jacobians() {
        let approximator = DiagonalApproximator::new(1e-8);
        let residuals = na::DVector::from_vec(vec![1.0, 2.0, 3.0]);
        let jacobians: Vec<na::DMatrix<f64>> = vec![na::DMatrix::from_vec(
            3,
            4,
            vec![1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
        )];
        let hessian = approximator.compute_hessian(&residuals, 4, Some(&jacobians));

        assert_eq!(hessian.nrows(), 4);
        assert_eq!(hessian.ncols(), 4);
        for i in 0..4 {
            for j in 0..4 {
                if i != j {
                    assert!((hessian[(i, j)]).abs() < 1e-10);
                }
            }
        }
        assert!(hessian[(0, 0)] > 0.0);
        assert!(hessian[(1, 1)] > 0.0);
        assert!(hessian[(2, 2)] > 0.0);
        assert!(hessian[(3, 3)] > 0.0);
    }

    #[test]
    fn test_diagonal_min_enforcement() {
        let approximator = DiagonalApproximator::new(1e-5);
        let residuals = na::DVector::from_vec(vec![0.0, 0.0, 0.0]);
        let jacobians: Vec<na::DMatrix<f64>> = vec![na::DMatrix::from_vec(
            3,
            4,
            vec![
                1e-10, 0.0, 0.0, 0.0, 0.0, 1e-10, 0.0, 0.0, 0.0, 0.0, 1e-10, 0.0,
            ],
        )];
        let hessian = approximator.compute_hessian(&residuals, 4, Some(&jacobians));

        for i in 0..4 {
            assert!(hessian[(i, i)] >= 1e-5 - 1e-15);
        }
    }

    #[test]
    fn test_exact_hessian_approximator() {
        let approximator = ExactHessianApproximator;
        let residuals = na::DVector::from_vec(vec![1.0, 2.0]);
        let jacobians: Vec<na::DMatrix<f64>> =
            vec![na::DMatrix::from_vec(2, 3, vec![1.0, 0.0, 0.0, 0.0, 1.0, 0.0])];
        let hessian = approximator.compute_hessian(&residuals, 3, Some(&jacobians));

        assert_eq!(hessian.nrows(), 3);
        assert_eq!(hessian.ncols(), 3);
        let expected = jacobians[0].transpose() * &jacobians[0];
        assert!((hessian - expected).norm() < 1e-10);
    }

    #[test]
    fn test_exact_hessian_approximator_fallback() {
        let approximator = ExactHessianApproximator;
        let residuals = na::DVector::from_vec(vec![1.0, 2.0, 3.0]);
        let hessian = approximator.compute_hessian(&residuals, 3, None);

        assert_eq!(hessian.nrows(), 3);
        assert_eq!(hessian.ncols(), 3);
        let scale = residuals.norm() / residuals.len() as f64;
        let expected = na::DMatrix::identity(3, 3) * scale.max(1e-6);
        assert!((hessian - expected).norm() < 1e-10);
    }

    #[test]
    fn test_lm_approximator_fallback() {
        let lm = LevenbergMarquardtApproximator::default();
        let residuals = na::DVector::from_vec(vec![1.0, 2.0, 3.0]);
        let hessian = lm.compute_hessian(&residuals, 4, None);

        assert_eq!(hessian.nrows(), 4);
        assert_eq!(hessian.ncols(), 4);
        for i in 0..4 {
            assert!(hessian[(i, i)] > 0.0, "Diagonal {} should be positive", i);
        }
    }

    #[test]
    fn test_lm_approximator_with_jacobians() {
        let lm = LevenbergMarquardtApproximator::default();
        let residuals = na::DVector::from_vec(vec![1.0, 2.0]);
        let jacobians: Vec<na::DMatrix<f64>> =
            vec![na::DMatrix::from_vec(2, 3, vec![1.0, 0.0, 0.0, 0.0, 1.0, 0.0])];
        let hessian = lm.compute_hessian(&residuals, 3, Some(&jacobians));

        assert_eq!(hessian.nrows(), 3);
        assert_eq!(hessian.ncols(), 3);
        let base_hessian = jacobians[0].transpose() * &jacobians[0];
        for i in 0..3 {
            assert!(
                hessian[(i, i)] >= base_hessian[(i, i)],
                "LM should not decrease diagonal elements"
            );
        }
    }

    #[test]
    fn test_lm_approximator_adaptive_damping() {
        let lm = LevenbergMarquardtApproximator::new(
            Box::new(GaussNewtonApproximator::default()),
            2.0,
        );
        let residuals = na::DVector::from_vec(vec![1.0, 2.0, 3.0]);
        let hessian = lm.compute_hessian(&residuals, 4, None);

        let gn = GaussNewtonApproximator::default();
        let base_hessian = gn.compute_hessian(&residuals, 4, None);

        for i in 0..4 {
            assert!(
                hessian[(i, i)] > base_hessian[(i, i)],
                "LM damping should increase diagonal"
            );
        }
    }

    #[test]
    fn test_standard_gradient_computer_fallback() {
        let computer = StandardGradientComputer;
        let residuals = na::DVector::from_vec(vec![1.0, 2.0, 3.0]);
        let gradient = computer.compute_gradient(&residuals, 3, None);

        assert_eq!(gradient.nrows(), 3);
        let mean = (1.0 + 2.0 + 3.0) / 3.0;
        for i in 0..3 {
            assert!(
                (gradient[i] - mean).abs() < 1e-10,
                "Gradient[{}] = {}, expected {}",
                i,
                gradient[i],
                mean
            );
        }
    }

    #[test]
    fn test_standard_gradient_computer_with_jacobians() {
        let computer = StandardGradientComputer;
        let residuals = na::DVector::from_vec(vec![1.0, 2.0]);
        let jacobians: Vec<na::DMatrix<f64>> = vec![
            na::DMatrix::from_vec(2, 3, vec![1.0, 0.0, 0.0, 0.0, 1.0, 0.0]),
            na::DMatrix::from_vec(2, 3, vec![0.0, 0.0, 1.0, 0.0, 0.0, 1.0]),
        ];
        let gradient = computer.compute_gradient(&residuals, 3, Some(&jacobians));

        assert_eq!(gradient.nrows(), 3);
        let expected =
            jacobians[0].transpose() * &residuals + jacobians[1].transpose() * &residuals;
        assert!((gradient - expected).norm() < 1e-10);
    }

    #[test]
    fn test_standard_gradient_computer_empty_jacobians() {
        let computer = StandardGradientComputer;
        let residuals = na::DVector::from_vec(vec![1.0, 2.0, 3.0]);
        let gradient = computer.compute_gradient(&residuals, 3, Some(&vec![]));

        assert_eq!(gradient.nrows(), 3);
        assert!(gradient.iter().all(|&x| x == 0.0));
    }

    #[test]
    fn test_zero_gradient_computer() {
        let computer = ZeroGradientComputer;
        let residuals = na::DVector::from_vec(vec![1.0, 2.0, 3.0]);
        let gradient = computer.compute_gradient(&residuals, 5, None);

        assert_eq!(gradient.nrows(), 5);
        assert!(gradient.iter().all(|&x: &f64| x.abs() < 1e-10));
    }

    #[test]
    fn test_zero_gradient_computer_with_jacobians() {
        let computer = ZeroGradientComputer;
        let residuals = na::DVector::from_vec(vec![1.0, 2.0, 3.0]);
        let jacobians: Vec<na::DMatrix<f64>> = vec![na::DMatrix::from_vec(
            3,
            4,
            vec![1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
        )];
        let gradient = computer.compute_gradient(&residuals, 4, Some(&jacobians));

        assert_eq!(gradient.nrows(), 4);
        assert!(gradient.iter().all(|&x: &f64| x.abs() < 1e-10));
    }

    #[test]
    fn test_standard_prior_constructor() {
        let constructor = StandardPriorConstructor;
        let schur = na::DMatrix::identity(3, 3);
        let gradient = na::DVector::from_vec(vec![1.0, 2.0, 3.0]);
        let param_ids = vec![ParamId::KeyframePose(0)];

        let prior = constructor.construct_prior(
            &schur,
            &gradient,
            &param_ids,
            3,
            &MarginalizationConfig::default(),
            &HashMap::new(),
        );

        assert!(prior.is_some());
        let prior = prior.unwrap();
        assert_eq!(prior.param_ids, param_ids);
        assert_eq!(prior.residual_dim, 3);
        assert!((prior.residual - gradient).norm() < 1e-10);
        // Information matrix is scaled by prior_info_scale (default 0.9)
        let expected_info = schur * 0.9; // prior_info_scale default is 0.9
        assert!((prior.information - expected_info).norm() < 1e-10);
    }

    #[test]
    fn test_standard_prior_constructor_with_scaling() {
        let constructor = StandardPriorConstructor;
        let schur = na::DMatrix::identity(3, 3);
        let gradient = na::DVector::from_vec(vec![1.0, 2.0, 3.0]);
        let param_ids = vec![ParamId::KeyframePose(0)];

        let mut config = MarginalizationConfig::default();
        config.prior_info_scale = 2.0;

        let prior =
            constructor.construct_prior(&schur, &gradient, &param_ids, 3, &config, &HashMap::new());

        assert!(prior.is_some());
        let prior = prior.unwrap();
        let expected_info = schur * 2.0;
        assert!((prior.information - expected_info).norm() < 1e-10);
    }

    #[test]
    fn test_regularized_prior_constructor() {
        let constructor = RegularizedPriorConstructor::new(1e-6);
        let mut schur = na::DMatrix::zeros(3, 3);
        schur[(0, 0)] = 1e-10;
        schur[(1, 1)] = 1e-10;
        schur[(2, 2)] = 1e-10;
        let gradient = na::DVector::from_vec(vec![1.0, 2.0, 3.0]);
        let param_ids = vec![ParamId::KeyframePose(0)];

        let prior = constructor.construct_prior(
            &schur,
            &gradient,
            &param_ids,
            3,
            &MarginalizationConfig::default(),
            &HashMap::new(),
        );

        assert!(prior.is_some());
        let prior = prior.unwrap();
        for i in 0..3 {
            assert!(
                prior.information[(i, i)] >= 1e-6,
                "Diagonal [{}, {}] = {} should be >= 1e-6",
                i,
                i,
                prior.information[(i, i)]
            );
        }
    }

    #[test]
    fn test_regularized_prior_constructor_no_regularization_needed() {
        let constructor = RegularizedPriorConstructor::new(1e-6);
        let schur = na::DMatrix::identity(3, 3) * 1e-3;
        let gradient = na::DVector::from_vec(vec![1.0, 2.0, 3.0]);
        let param_ids = vec![ParamId::KeyframePose(0)];

        let prior = constructor.construct_prior(
            &schur,
            &gradient,
            &param_ids,
            3,
            &MarginalizationConfig::default(),
            &HashMap::new(),
        );

        assert!(prior.is_some());
        let prior = prior.unwrap();
        for i in 0..3 {
            assert!(
                prior.information[(i, i)] >= 1e-6,
                "Diagonal should be at least min_eigenvalue"
            );
        }
    }

    #[test]
    fn test_marginalization_empty_param_blocks() {
        let mut manager = MarginalizationManager::default();
        let param_blocks = HashMap::new();
        let residuals = na::DVector::from_vec(vec![0.1, 0.2]);
        let jacobians: Vec<na::DMatrix<f64>> = vec![na::DMatrix::from_vec(2, 0, vec![])];
        let keep_ids: Vec<ParamId> = vec![];
        let marg_ids: Vec<ParamId> = vec![];

        let prior = manager.marginalize_with_approximation(
            &param_blocks,
            &residuals,
            Some(&jacobians),
            &keep_ids,
            &marg_ids,
        );

        assert!(prior.is_none());
    }

    #[test]
    fn test_marginalization_result_with_prior() {
        let mut manager = MarginalizationManager::default();
        let mut param_blocks = HashMap::new();
        param_blocks.insert(
            ParamId::KeyframePose(0),
            ParamBlock {
                id: ParamId::KeyframePose(0),
                dimension: 7,
                linearization_point: na::DVector::zeros(7),
            },
        );
        param_blocks.insert(
            ParamId::KeyframePose(1),
            ParamBlock {
                id: ParamId::KeyframePose(1),
                dimension: 7,
                linearization_point: na::DVector::zeros(7),
            },
        );

        let _residuals = na::DVector::from_vec(vec![0.1; 14]);
        let keep_ids = vec![ParamId::KeyframePose(1)];
        let marg_ids = vec![ParamId::KeyframePose(0)];

        let result = manager.marginalize(
            &param_blocks,
            &na::DMatrix::identity(14, 14),
            &na::DVector::zeros(14),
            &keep_ids,
            &marg_ids,
        );

        assert!(result.prior.is_some());
        let prior = result.prior.unwrap();
        assert_eq!(prior.param_ids.len(), 1);
        assert_eq!(prior.param_ids[0], ParamId::KeyframePose(0));
        assert_eq!(prior.residual_dim, 7);
        assert!(result.info.schur_complement_time_ms >= 0.0);
        assert_eq!(result.info.states_marginalized, 1);
    }

    #[test]
    fn test_marginalization_all_params_kept() {
        let mut manager = MarginalizationManager::default();
        let mut param_blocks = HashMap::new();
        param_blocks.insert(
            ParamId::KeyframePose(0),
            ParamBlock {
                id: ParamId::KeyframePose(0),
                dimension: 7,
                linearization_point: na::DVector::zeros(7),
            },
        );

        let _residuals = na::DVector::from_vec(vec![0.1; 7]);
        let keep_ids = vec![ParamId::KeyframePose(0)];
        let marg_ids: Vec<ParamId> = vec![];

        let result = manager.marginalize(
            &param_blocks,
            &na::DMatrix::identity(7, 7),
            &na::DVector::zeros(7),
            &keep_ids,
            &marg_ids,
        );

        assert!(result.prior.is_none());
        assert_eq!(result.info.states_marginalized, 0);
    }

    #[test]
    fn test_marginalization_multiple_param_blocks() {
        let mut manager = MarginalizationManager::default();
        let mut param_blocks = HashMap::new();
        param_blocks.insert(
            ParamId::KeyframePose(0),
            ParamBlock {
                id: ParamId::KeyframePose(0),
                dimension: 7,
                linearization_point: na::DVector::zeros(7),
            },
        );
        param_blocks.insert(
            ParamId::KeyframeVelocity(0),
            ParamBlock {
                id: ParamId::KeyframeVelocity(0),
                dimension: 3,
                linearization_point: na::DVector::zeros(3),
            },
        );
        param_blocks.insert(
            ParamId::KeyframePose(1),
            ParamBlock {
                id: ParamId::KeyframePose(1),
                dimension: 7,
                linearization_point: na::DVector::zeros(7),
            },
        );
        param_blocks.insert(
            ParamId::KeyframeVelocity(1),
            ParamBlock {
                id: ParamId::KeyframeVelocity(1),
                dimension: 3,
                linearization_point: na::DVector::zeros(3),
            },
        );

        let keep_ids = vec![ParamId::KeyframePose(1), ParamId::KeyframeVelocity(1)];
        let marg_ids = vec![ParamId::KeyframePose(0), ParamId::KeyframeVelocity(0)];

        let result = manager.marginalize(
            &param_blocks,
            &na::DMatrix::identity(20, 20),
            &na::DVector::zeros(20),
            &keep_ids,
            &marg_ids,
        );

        assert!(result.prior.is_some());
        let prior = result.prior.unwrap();
        assert_eq!(prior.param_ids.len(), 2);
        assert_eq!(prior.residual_dim, 10);
    }

    #[test]
    fn test_marginalization_fallback_hessian() {
        let mut manager = MarginalizationManager::default();
        manager.set_hessian_approximator(Box::new(IdentityApproximator));

        let mut param_blocks = HashMap::new();
        param_blocks.insert(
            ParamId::KeyframePose(0),
            ParamBlock {
                id: ParamId::KeyframePose(0),
                dimension: 7,
                linearization_point: na::DVector::zeros(7),
            },
        );
        param_blocks.insert(
            ParamId::KeyframePose(1),
            ParamBlock {
                id: ParamId::KeyframePose(1),
                dimension: 7,
                linearization_point: na::DVector::zeros(7),
            },
        );

        let residuals = na::DVector::from_vec(vec![1.0; 14]);
        let keep_ids = vec![ParamId::KeyframePose(1)];
        let marg_ids = vec![ParamId::KeyframePose(0)];

        let result = manager.marginalize_with_approximation(
            &param_blocks,
            &residuals,
            None,
            &keep_ids,
            &marg_ids,
        );

        assert!(result.is_some());
    }

    #[test]
    fn test_marginalization_singular_hessian_handling() {
        let mut manager = MarginalizationManager::default();
        let mut param_blocks = HashMap::new();
        param_blocks.insert(
            ParamId::KeyframePose(0),
            ParamBlock {
                id: ParamId::KeyframePose(0),
                dimension: 7,
                linearization_point: na::DVector::zeros(7),
            },
        );
        param_blocks.insert(
            ParamId::KeyframePose(1),
            ParamBlock {
                id: ParamId::KeyframePose(1),
                dimension: 7,
                linearization_point: na::DVector::zeros(7),
            },
        );

        let singular_hessian = na::DMatrix::zeros(14, 14);
        let keep_ids = vec![ParamId::KeyframePose(1)];
        let marg_ids = vec![ParamId::KeyframePose(0)];

        let result = manager.marginalize(
            &param_blocks,
            &singular_hessian,
            &na::DVector::zeros(14),
            &keep_ids,
            &marg_ids,
        );

        assert!(result.prior.is_some());
    }

    #[test]
    fn test_marginalization_disabled_skips_prior() {
        let mut config = MarginalizationConfig::default();
        config.enabled = false;
        let mut manager = MarginalizationManager::new(config);

        let mut param_blocks = HashMap::new();
        param_blocks.insert(
            ParamId::KeyframePose(0),
            ParamBlock {
                id: ParamId::KeyframePose(0),
                dimension: 1,
                linearization_point: na::DVector::from_vec(vec![1.0]),
            },
        );

        let keep_ids = vec![ParamId::KeyframePose(0)];
        let marg_ids: Vec<ParamId> = vec![];
        let result = manager.marginalize(
            &param_blocks,
            &na::DMatrix::identity(1, 1),
            &na::DVector::zeros(1),
            &keep_ids,
            &marg_ids,
        );

        assert!(result.prior.is_none());
        assert_eq!(manager.stats.total_marginalizations, 0);
    }

    #[test]
    fn test_fej_uses_first_linearization_point() {
        let mut manager = MarginalizationManager::default();

        let mut param_blocks = HashMap::new();
        param_blocks.insert(
            ParamId::KeyframePose(0),
            ParamBlock {
                id: ParamId::KeyframePose(0),
                dimension: 1,
                linearization_point: na::DVector::from_vec(vec![1.0]),
            },
        );
        param_blocks.insert(
            ParamId::KeyframePose(1),
            ParamBlock {
                id: ParamId::KeyframePose(1),
                dimension: 1,
                linearization_point: na::DVector::from_vec(vec![2.0]),
            },
        );

        let keep_ids = vec![ParamId::KeyframePose(1)];
        let marg_ids = vec![ParamId::KeyframePose(0)];
        manager.marginalize(
            &param_blocks,
            &na::DMatrix::identity(2, 2),
            &na::DVector::zeros(2),
            &keep_ids,
            &marg_ids,
        );

        // Second call with different linearization points; FEJ should keep the first set
        let mut updated_blocks = HashMap::new();
        updated_blocks.insert(
            ParamId::KeyframePose(0),
            ParamBlock {
                id: ParamId::KeyframePose(0),
                dimension: 1,
                linearization_point: na::DVector::from_vec(vec![5.0]),
            },
        );
        updated_blocks.insert(
            ParamId::KeyframePose(1),
            ParamBlock {
                id: ParamId::KeyframePose(1),
                dimension: 1,
                linearization_point: na::DVector::from_vec(vec![6.0]),
            },
        );

        manager.marginalize(
            &updated_blocks,
            &na::DMatrix::identity(2, 2),
            &na::DVector::zeros(2),
            &keep_ids,
            &marg_ids,
        );

        let cached = manager
            .fej_cache
            .get_point(&ParamId::KeyframePose(0))
            .expect("FEJ cache missing param");
        assert!((cached[0] - 1.0).abs() < 1e-12);
    }

    #[test]
    fn test_fej_disabled_updates_linearization_points() {
        let mut config = MarginalizationConfig::default();
        config.use_fej = false;
        let mut manager = MarginalizationManager::new(config);

        let mut param_blocks = HashMap::new();
        param_blocks.insert(
            ParamId::KeyframePose(0),
            ParamBlock {
                id: ParamId::KeyframePose(0),
                dimension: 1,
                linearization_point: na::DVector::from_vec(vec![1.0]),
            },
        );
        param_blocks.insert(
            ParamId::KeyframePose(1),
            ParamBlock {
                id: ParamId::KeyframePose(1),
                dimension: 1,
                linearization_point: na::DVector::from_vec(vec![2.0]),
            },
        );

        let keep_ids = vec![ParamId::KeyframePose(1)];
        let marg_ids = vec![ParamId::KeyframePose(0)];
        manager.marginalize(
            &param_blocks,
            &na::DMatrix::identity(2, 2),
            &na::DVector::zeros(2),
            &keep_ids,
            &marg_ids,
        );

        let mut updated_blocks = HashMap::new();
        updated_blocks.insert(
            ParamId::KeyframePose(0),
            ParamBlock {
                id: ParamId::KeyframePose(0),
                dimension: 1,
                linearization_point: na::DVector::from_vec(vec![9.0]),
            },
        );
        updated_blocks.insert(
            ParamId::KeyframePose(1),
            ParamBlock {
                id: ParamId::KeyframePose(1),
                dimension: 1,
                linearization_point: na::DVector::from_vec(vec![8.0]),
            },
        );

        manager.marginalize(
            &updated_blocks,
            &na::DMatrix::identity(2, 2),
            &na::DVector::zeros(2),
            &keep_ids,
            &marg_ids,
        );

        let cached = manager
            .fej_cache
            .get_point(&ParamId::KeyframePose(0))
            .expect("FEJ cache missing param");
        assert!((cached[0] - 9.0).abs() < 1e-12);
    }

    #[test]
    fn test_select_marginalization_candidates_empty() {
        let (marg_ids, keep_ids) = select_marginalization_candidates(
            &[],
            &[],
            &HashMap::new(),
            &HashMap::new(),
            0,
            &MarginalizationConfig::default(),
        );

        assert!(marg_ids.is_empty());
        assert_eq!(keep_ids.len(), 3);
        assert!(keep_ids.contains(&ParamId::GlobalGyroBias));
        assert!(keep_ids.contains(&ParamId::GlobalGravity));
        assert!(keep_ids.contains(&ParamId::GlobalDrag));
    }

    #[test]
    fn test_select_marginalization_candidates_keyframes() {
        let keyframe_ids = vec![0, 1, 2, 3, 4, 5, 6, 7, 8, 9];
        let config = MarginalizationConfig {
            num_marginalize_per_step: 1,
            ..Default::default()
        };
        let (marg_ids, keep_ids) = select_marginalization_candidates(
            &keyframe_ids,
            &[],
            &HashMap::new(),
            &HashMap::new(),
            10,
            &config,
        );

        assert_eq!(marg_ids.len(), 5);
        assert!(marg_ids.contains(&ParamId::KeyframePose(0)));
        assert!(marg_ids.contains(&ParamId::KeyframeVelocity(0)));
        assert!(marg_ids.contains(&ParamId::KeyframeAccelBias(0)));
        assert!(marg_ids.contains(&ParamId::KeyframeGyroBias(0)));
        assert!(marg_ids.contains(&ParamId::KeyframeMass(0)));

        assert_eq!(keep_ids.len(), 5 * 9 + 3);
    }

    #[test]
    fn test_select_marginalization_candidates_landmarks_old() {
        let landmark_ids = vec![0, 1, 2];
        let mut landmark_observations = HashMap::new();
        landmark_observations.insert(0, 2);
        landmark_observations.insert(1, 5);
        landmark_observations.insert(2, 10);

        let mut landmark_last_obs = HashMap::new();
        landmark_last_obs.insert(0, 8);
        landmark_last_obs.insert(1, 8);
        landmark_last_obs.insert(2, 8);

        let config = MarginalizationConfig {
            min_landmark_observations: 3,
            landmark_age_limit: 5,
            ..Default::default()
        };

        let (marg_ids, keep_ids) = select_marginalization_candidates(
            &[],
            &landmark_ids,
            &landmark_observations,
            &landmark_last_obs,
            10,
            &config,
        );

        assert!(keep_ids.contains(&ParamId::Landmark(1)));
        assert!(keep_ids.contains(&ParamId::Landmark(2)));
        assert!(marg_ids.contains(&ParamId::Landmark(0)));
    }

    #[test]
    fn test_select_marginalization_candidates_landmarks_poorly_observed() {
        let landmark_ids = vec![0, 1];
        let mut landmark_observations = HashMap::new();
        landmark_observations.insert(0, 2);
        landmark_observations.insert(1, 5);

        let mut landmark_last_obs = HashMap::new();
        landmark_last_obs.insert(0, 1);
        landmark_last_obs.insert(1, 1);

        let config = MarginalizationConfig {
            min_landmark_observations: 3,
            landmark_age_limit: 100,
            ..Default::default()
        };

        let (marg_ids, _) = select_marginalization_candidates(
            &[],
            &landmark_ids,
            &landmark_observations,
            &landmark_last_obs,
            5,
            &config,
        );

        assert!(marg_ids.contains(&ParamId::Landmark(0)));
    }

    #[test]
    fn test_marginalization_stats_tracking() {
        let mut manager = MarginalizationManager::default();
        assert_eq!(manager.stats.total_marginalizations, 0);

        let mut param_blocks = HashMap::new();
        param_blocks.insert(
            ParamId::KeyframePose(0),
            ParamBlock {
                id: ParamId::KeyframePose(0),
                dimension: 7,
                linearization_point: na::DVector::zeros(7),
            },
        );
        param_blocks.insert(
            ParamId::KeyframePose(1),
            ParamBlock {
                id: ParamId::KeyframePose(1),
                dimension: 7,
                linearization_point: na::DVector::zeros(7),
            },
        );

        let keep_ids = vec![ParamId::KeyframePose(1)];
        let marg_ids = vec![ParamId::KeyframePose(0)];

        for _ in 0..3 {
            manager.marginalize(
                &param_blocks,
                &na::DMatrix::identity(14, 14),
                &na::DVector::zeros(14),
                &keep_ids,
                &marg_ids,
            );
        }

        assert_eq!(manager.stats.total_marginalizations, 3);
        assert!(manager.stats.avg_schur_time_ms >= 0.0);
        assert!(manager.stats.avg_prior_construction_ms >= 0.0);
    }

    #[test]
    fn test_marginalization_fej_cache_update() {
        let mut manager = MarginalizationManager::default();
        let mut param_blocks = HashMap::new();
        param_blocks.insert(
            ParamId::KeyframePose(0),
            ParamBlock {
                id: ParamId::KeyframePose(0),
                dimension: 7,
                linearization_point: na::DVector::from_vec(vec![1.0; 7]),
            },
        );
        param_blocks.insert(
            ParamId::KeyframePose(1),
            ParamBlock {
                id: ParamId::KeyframePose(1),
                dimension: 7,
                linearization_point: na::DVector::from_vec(vec![2.0; 7]),
            },
        );

        let keep_ids = vec![ParamId::KeyframePose(1)];
        let marg_ids = vec![ParamId::KeyframePose(0)];

        manager.marginalize(
            &param_blocks,
            &na::DMatrix::identity(14, 14),
            &na::DVector::zeros(14),
            &keep_ids,
            &marg_ids,
        );

        assert!(manager.fej_cache.contains(&ParamId::KeyframePose(0)));
        assert!(manager.fej_cache.contains(&ParamId::KeyframePose(1)));
        let point = manager
            .fej_cache
            .get_point(&ParamId::KeyframePose(0))
            .unwrap();
        assert!((point - na::DVector::from_vec(vec![1.0; 7])).norm() < 1e-10);
    }

    #[test]
    fn test_hessian_approximator_clone() {
        let approximator: Box<dyn crate::optimization::marginalization::traits::HessianApproximator> =
            Box::new(GaussNewtonApproximator::new(1e-5));
        let cloned = approximator.clone_box();
        assert_eq!(cloned.name(), "GaussNewton");

        let lm: Box<dyn crate::optimization::marginalization::traits::HessianApproximator> =
            Box::new(LevenbergMarquardtApproximator::default());
        let cloned_lm = lm.clone_box();
        assert_eq!(cloned_lm.name(), "LevenbergMarquardt");
    }

    #[test]
    fn test_gradient_computer_clone() {
        let computer: Box<dyn crate::optimization::marginalization::traits::GradientComputer> =
            Box::new(StandardGradientComputer);
        let cloned = computer.clone_box();
        assert_eq!(cloned.name(), "Standard");
    }

    #[test]
    fn test_prior_constructor_clone() {
        let constructor: Box<dyn crate::optimization::marginalization::traits::PriorConstructor> =
            Box::new(RegularizedPriorConstructor::new(1e-8));
        let cloned = constructor.clone_box();
        assert_eq!(cloned.name(), "Regularized");
    }

    #[test]
    fn test_marginalization_with_landmarks() {
        let mut manager = MarginalizationManager::default();
        let mut param_blocks = HashMap::new();
        param_blocks.insert(
            ParamId::KeyframePose(0),
            ParamBlock {
                id: ParamId::KeyframePose(0),
                dimension: 7,
                linearization_point: na::DVector::zeros(7),
            },
        );
        param_blocks.insert(
            ParamId::Landmark(0),
            ParamBlock {
                id: ParamId::Landmark(0),
                dimension: 3,
                linearization_point: na::DVector::zeros(3),
            },
        );
        param_blocks.insert(
            ParamId::KeyframePose(1),
            ParamBlock {
                id: ParamId::KeyframePose(1),
                dimension: 7,
                linearization_point: na::DVector::zeros(7),
            },
        );
        param_blocks.insert(
            ParamId::Landmark(1),
            ParamBlock {
                id: ParamId::Landmark(1),
                dimension: 3,
                linearization_point: na::DVector::zeros(3),
            },
        );

        let keep_ids = vec![ParamId::KeyframePose(1), ParamId::Landmark(1)];
        let marg_ids = vec![ParamId::KeyframePose(0), ParamId::Landmark(0)];

        println!("DEBUG: marg_ids = {:?}", marg_ids);
        let landmark_count = marg_ids
            .iter()
            .filter(|id| matches!(id, ParamId::Landmark(_)))
            .count();
        println!("DEBUG: landmark count in marg_ids = {}", landmark_count);

        let result = manager.marginalize(
            &param_blocks,
            &na::DMatrix::identity(20, 20),
            &na::DVector::zeros(20),
            &keep_ids,
            &marg_ids,
        );

        assert!(result.prior.is_some());
        let prior = result.prior.unwrap();
        assert_eq!(prior.param_ids.len(), 2);
        assert_eq!(result.info.landmarks_marginalized, 1);
        assert_eq!(result.info.states_marginalized, 2);
    }

    #[test]
    fn test_condition_number_estimation() {
        let manager = MarginalizationManager::default();
        let well_conditioned = na::DMatrix::identity(3, 3);
        let cond = manager.estimate_condition_number(&well_conditioned);
        assert!(cond.is_some());
        let cond_val = cond.unwrap();
        assert!(cond_val > 0.0, "Condition number should be positive");

        let ill_conditioned = na::DMatrix::from_diagonal(&na::DVector::from_vec(vec![1e6, 1.0, 1e-6]));
        let cond_ill = manager.estimate_condition_number(&ill_conditioned);
        assert!(cond_ill.is_some());
        let cond_ill_val = cond_ill.unwrap();
        assert!(
            cond_ill_val > cond_val,
            "Ill-conditioned should have higher condition number than well-conditioned, got {} vs {}",
            cond_ill_val,
            cond_val
        );
    }

    #[test]
    fn test_condition_number_empty_matrix() {
        let manager = MarginalizationManager::default();
        let empty = na::DMatrix::zeros(0, 0);
        assert!(manager.estimate_condition_number(&empty).is_none());

        let non_square = na::DMatrix::zeros(3, 4);
        assert!(manager.estimate_condition_number(&non_square).is_none());
    }

    #[test]
    fn test_get_prior_mut() {
        let mut manager = MarginalizationManager::default();
        assert!(manager.get_prior_mut().is_none());

        let prior = crate::MarginalizationPrior {
            param_ids: vec![ParamId::KeyframePose(0)],
            residual_dim: 7,
            residual: na::DVector::from_vec(vec![1.0; 7]),
            information: na::DMatrix::identity(7, 7),
            damping: 1e-7,
            linearization_points: HashMap::new(),
        };
        manager.set_prior(prior);

        let prior_mut = manager.get_prior_mut().expect("Should have prior");
        prior_mut.damping = 1e-5;

        let prior_after = manager.get_prior().expect("Should have prior");
        assert_eq!(prior_after.damping, 1e-5);
    }

    #[test]
    fn test_config_default_values() {
        let config = MarginalizationConfig::default();
        assert!(config.enabled);
        assert!(config.use_fej);
        assert_eq!(config.damping, 1e-5); // Changed: better stability under motion blur
        assert_eq!(config.max_keyframes, 8); // Changed: embedded memory constraint
        assert_eq!(config.num_marginalize_per_step, 1);
        assert_eq!(config.min_landmark_observations, 3);
        assert_eq!(config.landmark_age_limit, 50);
        assert_eq!(config.prior_info_scale, 0.9); // Changed: prevent over-constraint
        assert_eq!(config.hessian_approximator, "Diagonal".to_string()); // Changed: faster for drones
    }

    #[test]
    fn test_param_id_variants() {
        let ids = vec![
            ParamId::KeyframePose(0),
            ParamId::KeyframeVelocity(1),
            ParamId::KeyframeAccelBias(2),
            ParamId::KeyframeGyroBias(3),
            ParamId::KeyframeMass(4),
            ParamId::Landmark(5),
            ParamId::GlobalGyroBias,
            ParamId::GlobalGravity,
            ParamId::GlobalDrag,
        ];

        for id in &ids {
            let cloned = id.clone();
            assert_eq!(format!("{:?}", id), format!("{:?}", cloned));
        }
    }
}
