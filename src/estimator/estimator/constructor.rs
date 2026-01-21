use super::state::Estimator;
use crate::calibration::online_intrinsics::OnlineIntrinsicsRefiner;
use crate::datasets::config::Config;
use crate::datasets::CameraModelType;
use crate::estimator::keyframe_culler::AggressiveCullingConfig;
use crate::estimator::point_quality::PointQualityConfig;
use crate::estimator::{FrameWorkspace, SlidingWindow, WorkspaceConfig};
use crate::feature_tracker::StereoPatchTracker;
use crate::fl;
use crate::fusion::depth_aware_fusion::{DepthAwareFusion, DepthAwareFusionConfig};
use crate::fusion::rotation_stabilizer::{RotationStabilizer, RotationStabilizerConfig};
use crate::imu::{
    DenoiseConfig, ExtrinsicCalibrator, HigherOrderFilter, HigherOrderFilterConfig,
    ImuAidedKeyframeSelector, ImuBiasEstimator, ImuConfig, ImuDenoiseFilter, ImuMotionPredictor,
    ImuPreintegrator, VelocityEstimator,
};
use crate::optimization::loop_closure::{LoopClosureConfig, LoopClosureDetector};
use crate::types::Float;
use crate::viewers::Viewer;
use crate::vision::{StereoSuperResolutionConfig, StereoSuperResolver};
use nalgebra as na;
use std::io::Write;
use std::time::Duration;

impl Estimator {
    #![allow(non_snake_case)]

    /// Create a new estimator configured with camera intrinsics and distortion
    /// loaded from the YAML configuration.
    pub fn new(config: Config, viewer: Option<Box<dyn Viewer>>) -> Self {
        Self::new_with_cameras(config, viewer, None, None)
    }

    /// Create a new estimator with optional camera models.
    /// If camera models are provided, they will be used; otherwise, they will be created from config.
    pub fn new_with_cameras(
        config: Config,
        viewer: Option<Box<dyn Viewer>>,
        left_cam: Option<CameraModelType>,
        right_cam: Option<CameraModelType>,
    ) -> Self {
        // Use provided cameras or create from config
        let (left_cam, right_cam) = match (left_cam, right_cam) {
            (Some(l), Some(r)) => (l, r),
            _ => crate::datasets::create_camera_models_from_config(&config),
        };

        let feature_config = config.feature_detection.clone();

        // Compute the transformation from left to right (T_C1_C0) as in compute_stereo.
        // Cast f64 config values to Float for f32/f64 compatibility
        let T_B_Cl = na::Matrix4::from_row_slice(
            &config
                .camera
                .T_B_Cl
                .iter()
                .map(|&x| x as Float)
                .collect::<Vec<_>>(),
        );
        let T_B_Cr = na::Matrix4::from_row_slice(
            &config
                .camera
                .T_B_Cr
                .iter()
                .map(|&x| x as Float)
                .collect::<Vec<_>>(),
        );
        let keyframe_window_size = config.keyframe_management.keyframe_window_size as usize;
        let processing_timeout_ms = config.keyframe_management.processing_timeout_ms;

        // Initialize aggressive keyframe culling with tuned defaults.
        // These are algorithm parameters that are typically set once during tuning
        // rather than per-dataset configuration. See AggressiveCullingConfig for details.
        let culling_config = Some(AggressiveCullingConfig {
            min_parallax_rad: fl!(0.05),
            min_observations: 20,
            max_similar_poses: 3,
            pose_similarity_translation: 0.1,
            enable_parallax_culling: true,
            enable_observation_culling: true,
            enable_redundancy_culling: true,
            enable_age_culling: true,
            max_age: 10,
            reserve_fraction: 0.7,
        });

        // Initialize point quality scorer with tuned defaults.
        // These parameters control map point filtering and are platform-independent.
        // See PointQualityConfig for parameter descriptions.
        let quality_config = Some(PointQualityConfig {
            min_track_length: 3,
            min_parallax: 0.05,
            max_point_age: 30,
            min_baseline_diversity: 0.1,
            quality_threshold: 0.3,
            auto_cull: true,
            cull_fraction: 0.2,
            max_map_size: 2000,
            detect_dynamic: true,
            motion_inconsistency_threshold: 5.0,
            min_angle_variance: 0.1,
            max_reproj_error: 2.0,
        });

        // Initialize online intrinsics refiner
        let initial_intrinsics = crate::calibration::CameraIntrinsics {
            fx: config.camera.left_intrinsics[0] as f64,
            fy: config.camera.left_intrinsics[1] as f64,
            cx: config.camera.left_intrinsics[2] as f64,
            cy: config.camera.left_intrinsics[3] as f64,
            width: config.camera.image_width,
            height: config.camera.image_height,
        };
        let intrinsics_refiner = Some(OnlineIntrinsicsRefiner::new(initial_intrinsics));

        // Initialize IMU components
        let imu_config = ImuConfig::default();
        // Initialize loop-closure detector with default config
        let loop_config = LoopClosureConfig::default();
        Estimator {
            frame_id_counter: 0,
            frames_since_last_keyframe: 0,
            enable_debug_output: true,
            config: config.clone(),
            stereo_patch_tracker: StereoPatchTracker::<6>::from_config(&feature_config),
            sliding_window: SlidingWindow::with_marginalization_config(
                keyframe_window_size,
                crate::optimization::marginalization::MarginalizationConfig::default(),
                culling_config,
                quality_config,
            ),
            viewer,
            left_cam,
            right_cam,
            T_B_Cl,
            T_B_Cr,
            trajectory: Vec::new(),
            // Default: 100ms deadline for 10Hz operation (with margin)
            // For 30Hz target 33ms, use Duration::from_millis(30)
            max_frame_processing_time: Duration::from_millis(processing_timeout_ms),
            // IMU components
            imu_preintegrator: ImuPreintegrator::new(imu_config.clone()),
            imu_motion_predictor: ImuMotionPredictor::new(imu_config.clone()),
            velocity_estimator: VelocityEstimator::new(imu_config.clone()),
            extrinsic_calibrator: ExtrinsicCalibrator::new(T_B_Cl),
            keyframe_selector: ImuAidedKeyframeSelector::new(
                fl!(config.keyframe_management.translation_threshold),
                fl!(config.keyframe_management.rotation_threshold),
            ),
            current_imu_preintegration: None,
            last_imu_timestamp: None,
            imu_measurement_count: 0,
            current_velocity: na::Vector3::<Float>::zeros(),
            velocity_estimator_initialized: false,
            bias_estimator: ImuBiasEstimator::new(imu_config.clone()),
            is_initializing: true,
            loop_closure_detector: LoopClosureDetector::new(loop_config.clone()),
            orb_extractor: if loop_config.descriptor_type == "orb" {
                use crate::optimization::loop_closure::orb::{OrbConfig, OrbExtractor};
                Some(OrbExtractor::new(OrbConfig::default()))
            } else {
                None
            },
            frame_workspace: FrameWorkspace::new(WorkspaceConfig {
                max_image_width: config.camera.image_width,
                max_image_height: config.camera.image_height,
                // Approximate capacity using grid_cols * max_features_per_grid
                max_features_per_frame: (feature_config.grid_cols
                    * feature_config.max_features_per_grid)
                    as usize,
                max_imu_samples: 200,
                capacity_headroom: 1.2,
            }),
            frame_count: 0,
            f0_log_writer: std::sync::Mutex::new(
                std::fs::File::create("/tmp/f0_data.csv")
                    .ok()
                    .and_then(|mut f| {
                        let _ = writeln!(f, "frame,timestamp_ns,f0_hz");
                        Some(f)
                    })
            ),
            spectrum_log_writer: std::sync::Mutex::new(
                std::fs::File::create("/tmp/gyro_spectrum.csv")
                    .ok()
                    .and_then(|mut f| {
                        let _ = writeln!(f, "frame,timestamp_ns,gyro_x_rms,gyro_y_rms,gyro_z_rms,gyro_magnitude_rms");
                        Some(f)
                    })
            ),
            // Initialize IMU denoising filter with default configuration
            denoise_filter: ImuDenoiseFilter::new(DenoiseConfig::default()),
            // Initialize higher-order filter for jerk/snap and f0 analysis
            higher_order_filter: HigherOrderFilter::new(HigherOrderFilterConfig::default()),
            // Initialize stereo super-resolution refinement
            stereo_super_resolver: StereoSuperResolver::new(StereoSuperResolutionConfig::default()),
            // Initialize online intrinsics refiner
            intrinsics_refiner,
            // Initialize fusion buffer and strategy based on config
            fusion_frame_buffer: std::collections::VecDeque::with_capacity(5),
            fusion_strategy: match config.debug.fusion_strategy.to_lowercase().as_str() {
                "none" => None,
                "rotation" => Some(Box::new(RotationStabilizer::new(
                    RotationStabilizerConfig::default(),
                ))),
                _ => Some(Box::new(DepthAwareFusion::new(
                    DepthAwareFusionConfig::default(),
                ))),
            },
        }
    }
}
