//! Minimal scaffolding for the MSCKF-lite estimator.

pub mod bridge;
pub mod clone;
pub mod degrade;
pub mod frontend;
pub mod init;
pub mod nullspace;
pub mod pipeline;
pub mod propagation;
pub mod residual;
pub mod state;
pub mod triangulate;
pub mod update;

pub use bridge::{
	build_visual_policy_from_track_outcomes, TrackUpdateOutcome,
};
pub use clone::{
	add_camera_clone, augment_clone_covariance, marginalize_oldest_clone, should_create_clone,
	CloneConfig, MAX_CLONES,
};
pub use degrade::{
	compute_frame_quality, update_visual_state, ConservativeUpdateCandidate,
	DegradationConfig, FrameQuality, VisualMode, VisualUpdatePolicy,
};
pub use frontend::{
	pair_stereo_observations, FeatureTrack, StereoObservation, TrackManager, TrackManagerConfig,
};
pub use init::{
	apply_initialization_result, estimate_gravity_alignment, estimate_gyro_bias_from_static_segment,
	estimate_visual_bootstrap_velocity, initialize_filter_state, InitializationConfig,
	InitializationResult,
};
pub use nullspace::{compute_left_nullspace, nullspace_project, NullspaceProjection};
pub use pipeline::{MsckfFrameResult, MsckfPipeline};
pub use propagation::{
	propagate_filter_state, propagate_nominal_state, ImuNoise, Matrix15, Matrix15x12,
};
pub use residual::{
	build_track_residual, EquidistantIntrinsics, FisheyeObservationModel, TrackResidual,
};
pub use state::{CameraClone, FilterState, ImuState, CLONE_DOF, IMU_STATE_DOF};
pub use triangulate::{
	stereo_depth_seed, triangulate_track, triangulate_track_diagnostic, StereoRig,
	TriangulatedLandmark, TriangulationConfig, TriangulationFailure, TriangulationResult,
};
pub use update::{
	apply_error_state_correction, batch_stack_gated_updates, chi_square_gate_track,
	ekf_update, project_track_into_filter_state, BatchUpdate, GatedTrackUpdate,
	UpdateConfig,
};