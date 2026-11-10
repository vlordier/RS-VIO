use crate::{
    datasets::{config::Config, create_camera_models_from_config, CameraModelType, ImuData},
    estimator::Frame,
    feature_tracker::{orb::OrbDetector, StereoPatchTracker},
    msckf::{
        add_camera_clone, batch_stack_gated_updates, build_track_residual,
        chi_square_gate_track, compute_frame_quality, ekf_update, initialize_filter_state,
        marginalize_oldest_clone, pair_stereo_observations, should_create_clone,
        triangulate_track_diagnostic, TriangulationFailure,
        update_visual_state, CloneConfig,
        ConservativeUpdateCandidate, DegradationConfig, EquidistantIntrinsics,
        FisheyeObservationModel, FilterState, ImuNoise, InitializationConfig, StereoRig,
        TrackManager, TrackManagerConfig, TrackUpdateOutcome, TriangulationConfig,
        UpdateConfig, VisualMode, propagate_filter_state,
    },
    types::{Float, Matrix4x4, UnitQuaternion, Vector3},
};
use anyhow::{anyhow, Result};
use image::GrayImage;
use nalgebra034;

#[derive(Debug, Clone)]
pub struct MsckfFrameResult {
    pub frame_id: usize,
    pub timestamp_ns: i64,
    pub initialized: bool,
    pub mode: VisualMode,
    pub clone_created: Option<usize>,
    pub paired_observations: usize,
    pub mature_tracks: usize,
    pub triangulated_tracks: usize,
    pub triangulation_seed_failures: usize,
    pub triangulation_dlt_failures: usize,
    pub triangulation_condition_failures: usize,
    pub triangulation_depth_failures: usize,
    pub triangulation_reprojection_failures: usize,
    pub accepted_tracks: usize,
    pub rejected_tracks: usize,
    pub state_dim: usize,
}

pub struct MsckfPipeline {
    config: Config,
    stereo_patch_tracker: StereoPatchTracker<6>,
    track_manager: TrackManager,
    filter_state: FilterState,
    stereo_rig: StereoRig,
    camera_model: FisheyeObservationModel,
    clone_config: CloneConfig,
    triangulation_config: TriangulationConfig,
    update_config: UpdateConfig,
    degradation_config: DegradationConfig,
    init_config: InitializationConfig,
    imu_noise: ImuNoise,
    frame_id_counter: usize,
    feat_capacity: usize,
    left_undist_lut: Vec<[f32; 2]>,
    right_undist_lut: Vec<[f32; 2]>,
    lut_width: usize,
    lut_height: usize,
    previous_frame: Option<Frame>,
    imu_init_samples: Vec<ImuData>,
    pub pose_log: Vec<(i64, Matrix4x4)>,
    pub initialized: bool,
}

fn build_undist_lut(cam: &CameraModelType, width: u32, height: u32) -> Vec<[f32; 2]> {
    let mut lut = Vec::with_capacity((width * height) as usize);
    for y in 0..height {
        for x in 0..width {
            let bearing = cam
                .as_camera_model()
                .unproject_one(&nalgebra034::Vector2::new(x as f64, y as f64));
            let normalized = if bearing[2].abs() > 1.0e-9 {
                [
                    (bearing[0] / bearing[2]) as f32,
                    (bearing[1] / bearing[2]) as f32,
                ]
            } else {
                [-1.0, -1.0]
            };
            lut.push(normalized);
        }
    }
    lut
}

#[inline(always)]
fn bilinear_lut(lut: &[[f32; 2]], width: usize, height: usize, px: f32, py: f32) -> [f32; 2] {
    let ix = (px as usize).min(width - 2);
    let iy = (py as usize).min(height - 2);
    let dx = px - ix as f32;
    let dy = py - iy as f32;
    let w11 = dx * dy;
    let w00 = 1.0 - dx - dy + w11;
    let w10 = dx - w11;
    let w01 = dy - w11;
    let base = iy * width + ix;
    let (v00, v10, v01, v11) = unsafe {
        (
            *lut.get_unchecked(base),
            *lut.get_unchecked(base + 1),
            *lut.get_unchecked(base + width),
            *lut.get_unchecked(base + width + 1),
        )
    };
    [
        w00 * v00[0] + w10 * v10[0] + w01 * v01[0] + w11 * v11[0],
        w00 * v00[1] + w10 * v10[1] + w01 * v01[1] + w11 * v11[1],
    ]
}

fn matrix4_from_row_slice(values: &[f64]) -> Matrix4x4 {
    Matrix4x4::from_row_slice(values)
}

fn stereo_rig_from_config(config: &Config) -> StereoRig {
    let t_b_cl = matrix4_from_row_slice(&config.camera.T_B_Cl);
    let t_b_cr = matrix4_from_row_slice(&config.camera.T_B_Cr);
    let t_cr_cl = t_b_cr.try_inverse().unwrap_or_else(Matrix4x4::identity) * t_b_cl;
    StereoRig::new(
        t_cr_cl.fixed_view::<3, 3>(0, 0).into_owned(),
        t_cr_cl.fixed_view::<3, 1>(0, 3).into_owned(),
    )
}

fn fisheye_model_from_config(config: &Config) -> FisheyeObservationModel {
    let left = EquidistantIntrinsics {
        fx: config.camera.left_intrinsics.first().copied().unwrap_or(190.0) as Float,
        fy: config.camera.left_intrinsics.get(1).copied().unwrap_or(190.0) as Float,
        cx: config.camera.left_intrinsics.get(2).copied().unwrap_or(256.0) as Float,
        cy: config.camera.left_intrinsics.get(3).copied().unwrap_or(256.0) as Float,
        k: [
            config.camera.left_distortion.first().copied().unwrap_or(0.0) as Float,
            config.camera.left_distortion.get(1).copied().unwrap_or(0.0) as Float,
            config.camera.left_distortion.get(2).copied().unwrap_or(0.0) as Float,
            config.camera.left_distortion.get(3).copied().unwrap_or(0.0) as Float,
        ],
    };
    let right = EquidistantIntrinsics {
        fx: config.camera.right_intrinsics.first().copied().unwrap_or(190.0) as Float,
        fy: config.camera.right_intrinsics.get(1).copied().unwrap_or(190.0) as Float,
        cx: config.camera.right_intrinsics.get(2).copied().unwrap_or(256.0) as Float,
        cy: config.camera.right_intrinsics.get(3).copied().unwrap_or(256.0) as Float,
        k: [
            config.camera.right_distortion.first().copied().unwrap_or(0.0) as Float,
            config.camera.right_distortion.get(1).copied().unwrap_or(0.0) as Float,
            config.camera.right_distortion.get(2).copied().unwrap_or(0.0) as Float,
            config.camera.right_distortion.get(3).copied().unwrap_or(0.0) as Float,
        ],
    };
    FisheyeObservationModel { left, right }
}

fn left_camera_pose_from_imu(config: &Config, orientation_w_i: &UnitQuaternion, position_w_i: &Vector3) -> (UnitQuaternion, Vector3) {
    let t_b_cl = matrix4_from_row_slice(&config.camera.T_B_Cl);
    let rotation_i_cl: crate::types::Matrix3x3 = t_b_cl.fixed_view::<3, 3>(0, 0).into_owned();
    let translation_i_cl: Vector3 = t_b_cl.fixed_view::<3, 1>(0, 3).into_owned();
    let orientation_w_cl = *orientation_w_i * UnitQuaternion::from_matrix(&rotation_i_cl);
    let position_w_cl = *position_w_i + orientation_w_i.transform_vector(&translation_i_cl);
    (orientation_w_cl, position_w_cl)
}

fn pose_matrix(orientation: &UnitQuaternion, position: &Vector3) -> Matrix4x4 {
    let mut pose = Matrix4x4::identity();
    pose.fixed_view_mut::<3, 3>(0, 0)
        .copy_from(&orientation.to_rotation_matrix().into_inner());
    pose.fixed_view_mut::<3, 1>(0, 3).copy_from(position);
    pose
}

impl MsckfPipeline {
    #[must_use]
    pub fn new(config: Config) -> Self {
        let (left_cam, right_cam) = create_camera_models_from_config(&config);
        let img_w = config.camera.image_width;
        let img_h = config.camera.image_height;
        let grid_size = config.feature_detection.grid_cols;
        let feat_capacity = ((img_w / grid_size + 1) * (img_h / grid_size + 1)) as usize;

        Self {
            stereo_patch_tracker: StereoPatchTracker::<6>::new(
                grid_size,
                config.feature_detection.optical_flow_max_iterations,
                config.feature_detection.optical_flow_convergence_threshold,
            ),
            track_manager: TrackManager::new(TrackManagerConfig::default()),
            filter_state: FilterState::new(),
            stereo_rig: stereo_rig_from_config(&config),
            camera_model: fisheye_model_from_config(&config),
            clone_config: CloneConfig::default(),
            triangulation_config: TriangulationConfig::default(),
            update_config: UpdateConfig::default(),
            degradation_config: DegradationConfig::default(),
            init_config: InitializationConfig::default(),
            imu_noise: ImuNoise {
                gyro_noise_std: config.imu.gyro_noise_std as Float,
                accel_noise_std: config.imu.accel_noise_std as Float,
                gyro_random_walk_std: config.imu.gyro_random_walk_std as Float,
                accel_random_walk_std: config.imu.accel_random_walk_std as Float,
            },
            config,
            frame_id_counter: 0,
            feat_capacity,
            left_undist_lut: build_undist_lut(&left_cam, img_w, img_h),
            right_undist_lut: build_undist_lut(&right_cam, img_w, img_h),
            lut_width: img_w as usize,
            lut_height: img_h as usize,
            previous_frame: None,
            imu_init_samples: Vec::new(),
            pose_log: Vec::new(),
            initialized: false,
        }
    }

    fn track_frame(
        &mut self,
        left_image: Vec<u8>,
        right_image: Vec<u8>,
        timestamp_ns: i64,
    ) -> Result<Frame> {
        let img_w = self.config.camera.image_width;
        let img_h = self.config.camera.image_height;
        let left_img = GrayImage::from_raw(img_w, img_h, left_image)
            .ok_or_else(|| anyhow!("failed to construct left image"))?;
        let right_img = GrayImage::from_raw(img_w, img_h, right_image)
            .ok_or_else(|| anyhow!("failed to construct right image"))?;

        self.frame_id_counter += 1;
        let mut frame = Frame::from_stereo_images(
            timestamp_ns,
            self.frame_id_counter as i32,
            matrix4_from_row_slice(&self.config.camera.T_B_Cl),
            matrix4_from_row_slice(&self.config.camera.T_B_Cr),
            self.feat_capacity,
        );

        self.stereo_patch_tracker
            .process_frame(&left_img, &right_img, &mut frame);

        for feature in &mut frame.left_features {
            feature.undistorted_coord = bilinear_lut(
                &self.left_undist_lut,
                self.lut_width,
                self.lut_height,
                feature.pixel_coord[0],
                feature.pixel_coord[1],
            );
        }
        for feature in &mut frame.right_features {
            feature.undistorted_coord = bilinear_lut(
                &self.right_undist_lut,
                self.lut_width,
                self.lut_height,
                feature.pixel_coord[0],
                feature.pixel_coord[1],
            );
        }

        let orb_detector = OrbDetector::with_defaults();
        if !frame.left_features.is_empty() {
            let points = frame
                .left_features
                .iter()
                .map(|feature| (feature.pixel_coord[0], feature.pixel_coord[1]))
                .collect::<Vec<_>>();
            frame.left_descriptors = orb_detector.compute_descriptors_for_points(&left_img, &points);
        }
        if !frame.right_features.is_empty() {
            let points = frame
                .right_features
                .iter()
                .map(|feature| (feature.pixel_coord[0], feature.pixel_coord[1]))
                .collect::<Vec<_>>();
            frame.right_descriptors = orb_detector.compute_descriptors_for_points(&right_img, &points);
        }

        Ok(frame)
    }

    pub fn process_frame(
        &mut self,
        left_image: Vec<u8>,
        right_image: Vec<u8>,
        timestamp_ns: i64,
        imu_data: Option<&[ImuData]>,
    ) -> Result<MsckfFrameResult> {
        let frame = self.track_frame(left_image, right_image, timestamp_ns)?;
        self.process_tracked_frame(frame, imu_data)
    }

    pub fn process_tracked_frame(
        &mut self,
        current_frame: Frame,
        imu_data: Option<&[ImuData]>,
    ) -> Result<MsckfFrameResult> {
        if let Some(imu_samples) = imu_data {
            self.imu_init_samples.extend_from_slice(imu_samples);
            for pair in imu_samples.windows(2) {
                let _ = propagate_filter_state(
                    &mut self.filter_state,
                    &pair[0],
                    &pair[1],
                    &self.imu_noise,
                );
            }
        }

        let active_track_count = current_frame.left_features().len();
        let frame_observations = pair_stereo_observations(
            &current_frame,
            self.filter_state
                .clones
                .last()
                .map_or(0, |clone| clone.clone_id),
            current_frame.timestamp_ns as Float * 1e-9,
            &self.track_manager.config,
        );
        let paired_observations = frame_observations.len();

        if !self.initialized {
            if let Some(previous_frame) = self.previous_frame.as_ref() {
                if initialize_filter_state(
                    &mut self.filter_state,
                    &self.imu_init_samples,
                    previous_frame,
                    &current_frame,
                    &self.stereo_rig,
                    &self.init_config,
                )
                .is_some()
                {
                    self.initialized = true;
                }
            }

            self.pose_log.push((
                current_frame.timestamp_ns,
                pose_matrix(
                    &self.filter_state.imu_state.orientation,
                    &self.filter_state.imu_state.position,
                ),
            ));
            self.previous_frame = Some(current_frame);

            return Ok(MsckfFrameResult {
                frame_id: self.frame_id_counter,
                timestamp_ns: self.pose_log.last().map_or(0, |entry| entry.0),
                initialized: self.initialized,
                mode: self.filter_state.mode,
                clone_created: None,
                paired_observations,
                mature_tracks: 0,
                triangulated_tracks: 0,
                triangulation_seed_failures: 0,
                triangulation_dlt_failures: 0,
                triangulation_condition_failures: 0,
                triangulation_depth_failures: 0,
                triangulation_reprojection_failures: 0,
                accepted_tracks: 0,
                rejected_tracks: 0,
                state_dim: self.filter_state.state_dim(),
            });
        }

        let (orientation_w_cl, position_w_cl) = left_camera_pose_from_imu(
            &self.config,
            &self.filter_state.imu_state.orientation,
            &self.filter_state.imu_state.position,
        );
        let camera_lever_arm_world = position_w_cl - self.filter_state.imu_state.position;

        let wants_clone = self.filter_state.mode != VisualMode::Degraded
            && should_create_clone(
                &self.filter_state,
                current_frame.timestamp_ns as Float * 1e-9,
                &orientation_w_cl,
                &position_w_cl,
                &self.clone_config,
            );
        if wants_clone && self.filter_state.clones.len() >= self.clone_config.max_clones {
            if let Some(removed_clone) = marginalize_oldest_clone(&mut self.filter_state) {
                self.track_manager.prune_clone_observations(removed_clone.clone_id);
            }
        }

        let clone_created = if wants_clone {
            add_camera_clone(
                &mut self.filter_state,
                current_frame.timestamp_ns as Float * 1e-9,
                orientation_w_cl,
                position_w_cl,
                camera_lever_arm_world,
                &self.clone_config,
            )
        } else {
            None
        };

        if let Some(active_clone_id) = clone_created.or_else(|| {
            self.filter_state
                .clones
                .last()
                .map(|clone| clone.clone_id)
        }) {
            let _ = self.track_manager.ingest_frame(&current_frame, active_clone_id);
        }

        let mature_tracks = self.track_manager.mature_tracks();
        let mut triangulation_seed_failures = 0usize;
        let mut triangulation_dlt_failures = 0usize;
        let mut triangulation_condition_failures = 0usize;
        let mut triangulation_depth_failures = 0usize;
        let mut triangulation_reprojection_failures = 0usize;
        let outcomes = mature_tracks
            .iter()
            .map(|track| {
                let triangulation = triangulate_track_diagnostic(
                    track,
                    &self.filter_state.clones,
                    &self.stereo_rig,
                    &self.triangulation_config,
                );
                match triangulation.failure {
                    Some(TriangulationFailure::StereoSeed) => triangulation_seed_failures += 1,
                    Some(TriangulationFailure::DltSystem | TriangulationFailure::DltSolve) => {
                        triangulation_dlt_failures += 1;
                    }
                    Some(TriangulationFailure::ConditionNumber) => {
                        triangulation_condition_failures += 1;
                    }
                    Some(TriangulationFailure::PositiveDepth) => {
                        triangulation_depth_failures += 1;
                    }
                    Some(TriangulationFailure::Reprojection) => {
                        triangulation_reprojection_failures += 1;
                    }
                    None => {}
                }
                let triangulated = triangulation.landmark;
                let gated_update = triangulated.as_ref().and_then(|landmark| {
                    let residual = build_track_residual(
                        track,
                        &landmark.position_world,
                        &self.filter_state.clones,
                        &self.stereo_rig,
                        &self.camera_model,
                    )?;
                    chi_square_gate_track(&self.filter_state, &residual, &self.update_config)
                });

                TrackUpdateOutcome {
                    feature_id: track.feature_id,
                    clone_observation_count: track.clone_observation_count(),
                    reprojection_error_px: triangulated.as_ref().map_or(
                        self.degradation_config.degraded_reprojection_threshold_px + 1.0,
                        |landmark| landmark.reprojection_rmse,
                    ),
                    triangulated: triangulated.is_some(),
                    gated_update,
                }
            })
            .collect::<Vec<_>>();
        let triangulated_tracks = outcomes
            .iter()
            .filter(|outcome| outcome.triangulated)
            .count();

        let conservative_candidates = outcomes
            .iter()
            .filter_map(|outcome| {
                Some(ConservativeUpdateCandidate {
                    clone_observation_count: outcome.clone_observation_count,
                    reprojection_error_px: outcome.reprojection_error_px,
                    gated_update: outcome.gated_update.clone()?,
                })
            })
            .collect::<Vec<_>>();
        let quality = compute_frame_quality(
            active_track_count,
            paired_observations,
            &[],
        );
        let policy = update_visual_state(
            &mut self.filter_state,
            quality,
            self.initialized,
            &conservative_candidates,
            &self.degradation_config,
        );
        let accepted_updates = if !policy.conservative_updates.is_empty() {
            policy.conservative_updates.clone()
        } else if policy.allow_visual_update {
            outcomes
                .iter()
                .filter_map(|outcome| outcome.gated_update.clone())
                .collect::<Vec<_>>()
        } else {
            Vec::new()
        };
        let rejected_tracks = outcomes
            .iter()
            .filter(|outcome| outcome.gated_update.is_none())
            .count();

        if let Some(batch) = batch_stack_gated_updates(&accepted_updates, self.filter_state.state_dim()) {
            let _ = ekf_update(&mut self.filter_state, &batch, &self.update_config);
        }

        self.pose_log.push((
            current_frame.timestamp_ns,
            pose_matrix(
                &self.filter_state.imu_state.orientation,
                &self.filter_state.imu_state.position,
            ),
        ));
        let timestamp_ns = current_frame.timestamp_ns;
        self.previous_frame = Some(current_frame);

        Ok(MsckfFrameResult {
            frame_id: self.frame_id_counter,
            timestamp_ns,
            initialized: self.initialized,
            mode: self.filter_state.mode,
            clone_created,
            paired_observations,
            mature_tracks: mature_tracks.len(),
            triangulated_tracks,
            triangulation_seed_failures,
            triangulation_dlt_failures,
            triangulation_condition_failures,
            triangulation_depth_failures,
            triangulation_reprojection_failures,
            accepted_tracks: accepted_updates.len(),
            rejected_tracks,
            state_dim: self.filter_state.state_dim(),
        })
    }

    #[must_use]
    pub fn filter_state(&self) -> &FilterState {
        &self.filter_state
    }
}

#[cfg(test)]
mod tests {
    use super::MsckfPipeline;
    use crate::{
        datasets::{
            config::{
                CameraConfig, Config, FeatureDetectionConfig, ImuConfig,
                KeyframeManagementConfig, OptimizationConfig,
            },
            ImuData,
        },
        estimator::Frame,
        feature_tracker::Feature,
        msckf::VisualMode,
    };

    fn test_config() -> Config {
        Config {
            camera: CameraConfig {
                image_width: 512,
                image_height: 512,
                left_intrinsics: vec![190.0, 190.0, 256.0, 256.0],
                left_distortion: vec![0.0, 0.0, 0.0, 0.0],
                right_intrinsics: vec![190.0, 190.0, 256.0, 256.0],
                right_distortion: vec![0.0, 0.0, 0.0, 0.0],
                left_model: Some("equidistant".to_string()),
                right_model: Some("equidistant".to_string()),
                T_B_Cl: vec![
                    1.0, 0.0, 0.0, 0.0,
                    0.0, 1.0, 0.0, 0.0,
                    0.0, 0.0, 1.0, 0.0,
                    0.0, 0.0, 0.0, 1.0,
                ],
                T_B_Cr: vec![
                    1.0, 0.0, 0.0, 0.2,
                    0.0, 1.0, 0.0, 0.0,
                    0.0, 0.0, 1.0, 0.0,
                    0.0, 0.0, 0.0, 1.0,
                ],
            },
            keyframe_management: KeyframeManagementConfig {
                keyframe_window_size: 10,
                translation_threshold: 0.4,
                rotation_threshold: 0.25,
                imu_gyro_kf_threshold: 2.0,
                imu_accel_kf_threshold: 50.0,
            },
            feature_detection: FeatureDetectionConfig {
                grid_cols: 30,
                max_features_per_grid: 20,
                optical_flow_max_iterations: 30,
                optical_flow_convergence_threshold: 0.005,
            },
            optimization: OptimizationConfig {
                bundle_adjustment_max_iterations: 20,
                pnp_max_iterations: 10,
                cam_imu_td_s: 0.0,
                cam_imu_td_online_refinement: false,
            },
            imu: ImuConfig::default(),
        }
    }

    fn imu_sample(timestamp: i64) -> ImuData {
        ImuData {
            timestamp,
            gyro: [0.0, 0.0, 0.0],
            accel: [0.0, 0.0, 9.81007],
        }
    }

    fn build_frame(timestamp_ns: i64, frame_id: i32, offset_x: f32) -> Frame {
        let mut frame = Frame::new(timestamp_ns, frame_id);
        for feature_id in 0..40 {
            let x = -0.2 + feature_id as f32 * 0.02 + offset_x;
            let y = -0.1 + feature_id as f32 * 0.01;
            let z = 4.0 + feature_id as f32 * 0.02;
            let left_normalized = [x / z, y / z];
            let right_normalized = [(x - 0.2) / z, y / z];
            let mut left = Feature::new(
                feature_id,
                [256.0 + 190.0 * left_normalized[0], 256.0 + 190.0 * left_normalized[1]],
            );
            let mut right = Feature::new(
                feature_id,
                [256.0 + 190.0 * right_normalized[0], 256.0 + 190.0 * right_normalized[1]],
            );
            left.undistorted_coord = left_normalized;
            right.undistorted_coord = right_normalized;
            left.age = 3;
            right.age = 3;
            frame.add_left_feature(left);
            frame.add_right_feature(right);
        }
        frame
    }

    #[test]
    fn pipeline_initializes_and_transitions_to_normal() {
        let mut pipeline = MsckfPipeline::new(test_config());
        let imu = (0..120)
            .map(|index| imu_sample(index * 5_000_000))
            .collect::<Vec<_>>();

        let first = pipeline
            .process_tracked_frame(build_frame(0, 0, 0.0), Some(&imu[..60]))
            .unwrap();
        assert!(!first.initialized);

        let second = pipeline
            .process_tracked_frame(build_frame(100_000_000, 1, -0.1), Some(&imu[60..]))
            .unwrap();
        assert!(second.initialized);
        assert_eq!(second.mode, VisualMode::Normal);
    }

    #[test]
    fn pipeline_keeps_creating_clones_after_initialization() {
        let mut pipeline = MsckfPipeline::new(test_config());
        let imu = (0..200)
            .map(|index| imu_sample(index * 5_000_000))
            .collect::<Vec<_>>();

        let _ = pipeline
            .process_tracked_frame(build_frame(0, 0, 0.0), Some(&imu[..60]))
            .unwrap();
        let _ = pipeline
            .process_tracked_frame(build_frame(100_000_000, 1, -0.1), Some(&imu[60..120]))
            .unwrap();

        let third = pipeline
            .process_tracked_frame(build_frame(200_000_000, 2, -0.2), Some(&imu[120..160]))
            .unwrap();
        let fourth = pipeline
            .process_tracked_frame(build_frame(300_000_000, 3, -0.3), Some(&imu[160..]))
            .unwrap();

        assert_eq!(third.mode, VisualMode::Normal);
        assert_eq!(fourth.mode, VisualMode::Normal);
        assert!(third.clone_created.is_some());
        assert!(fourth.clone_created.is_some());
        assert!(fourth.state_dim >= 27);
    }
}