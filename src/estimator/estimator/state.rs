use crate::calibration::online_intrinsics::OnlineIntrinsicsRefiner;
use crate::datasets::config::Config;
use crate::datasets::CameraModelType;
use crate::estimator::frame_processor::Frontend;
use crate::estimator::global_pose_graph::GlobalPoseGraph;
use crate::estimator::imu_processor::ImuProcessor;
use crate::estimator::sliding_window::Backend;
use crate::estimator::FrameWorkspace;
use crate::fusion::FusionStrategyImpl;
use crate::optimization::loop_closure::LoopClosureDetector;
use crate::types::Matrix4x4;
use crate::viewers::Viewer;
use crate::vision::StereoSuperResolver;
use std::time::Duration;

pub struct Estimator {
    pub frame_id_counter: u64,
    pub frames_since_last_keyframe: u64,
    pub enable_debug_output: bool,
    pub(crate) config: Config,
    pub frontend: Frontend<6>,
    pub backend: Backend,
    pub global_pose_graph: GlobalPoseGraph,
    pub imu_processor: ImuProcessor,
    pub loop_closure_detector: LoopClosureDetector,
    pub viewer: Option<Box<dyn Viewer>>,
    pub left_cam: CameraModelType,
    pub right_cam: CameraModelType,
    pub T_B_Cl: Matrix4x4,
    pub T_B_Cr: Matrix4x4,
    pub trajectory: Vec<(i64, Matrix4x4)>,
    pub max_frame_processing_time: Duration,
    pub frame_workspace: FrameWorkspace,
    pub frame_count: u64,
    pub f0_log_writer: std::sync::Mutex<Option<std::fs::File>>,
    pub spectrum_log_writer: std::sync::Mutex<Option<std::fs::File>>,
    pub stereo_super_resolver: StereoSuperResolver,
    pub intrinsics_refiner: Option<OnlineIntrinsicsRefiner>,
    /// Frame buffer for fusion (Arc-wrapped to avoid expensive clones)
    pub fusion_frame_buffer: std::collections::VecDeque<std::sync::Arc<crate::estimator::Frame>>,
    pub fusion_strategy: Option<Box<dyn FusionStrategyImpl>>,
}
