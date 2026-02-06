//! Async wrapper for sequential estimator pipeline
//!
//! Provides async/await interface for the synchronous Estimator,
//! enabling non-blocking frame submission and result retrieval.

use crate::datasets::config::Config;
use crate::datasets::{CameraModelType, ImuData};
use crate::estimator::Estimator;
use crate::viewers::Viewer;
use anyhow::Result;
use tokio::sync::{mpsc, oneshot};

enum Command {
    ProcessFrame {
        frame_id: i64,
        left_image: Vec<u8>,
        right_image: Vec<u8>,
        timestamp_ns: i64,
        imu_data: Option<Vec<ImuData>>,
        respond_to: oneshot::Sender<Result<()>>,
    },
    Shutdown(oneshot::Sender<()>),
}

/// Async wrapper around sequential Estimator
///
/// Spawns blocking Estimator operations on dedicated tokio blocking threads
/// to prevent starving other async tasks.
pub struct AsyncEstimator {
    command_tx: mpsc::Sender<Command>,
    join_handle: Option<std::thread::JoinHandle<()>>,
}

impl AsyncEstimator {
    /// Create new async estimator running on a dedicated worker thread.
    pub fn new_with_cameras(
        config: Config,
        viewer: Option<Box<dyn Viewer>>,
        left_cam: Option<CameraModelType>,
        right_cam: Option<CameraModelType>,
    ) -> Self {
        let (command_tx, mut command_rx) = mpsc::channel::<Command>(32);

        let join_handle = std::thread::spawn(move || {
            let mut estimator = Estimator::new_with_cameras(config, viewer, left_cam, right_cam);

            while let Some(command) = command_rx.blocking_recv() {
                match command {
                    Command::ProcessFrame {
                        frame_id,
                        left_image,
                        right_image,
                        timestamp_ns,
                        imu_data,
                        respond_to,
                    } => {
                        estimator.set_viewer_frame(frame_id);
                        let result = estimator.process_frame(
                            &left_image,
                            &right_image,
                            timestamp_ns,
                            imu_data.as_deref(),
                        );
                        let _ = respond_to.send(result);
                    },
                    Command::Shutdown(respond_to) => {
                        let _ = respond_to.send(());
                        break;
                    },
                }
            }
        });

        Self {
            command_tx,
            join_handle: Some(join_handle),
        }
    }

    /// Process frame asynchronously (placeholder)
    pub async fn process_frame_async(
        &self,
        frame_id: i64,
        left_image: Vec<u8>,
        right_image: Vec<u8>,
        timestamp_ns: i64,
        imu_data: Option<Vec<ImuData>>,
    ) -> Result<()> {
        let (respond_to, response_rx) = oneshot::channel();

        self.command_tx
            .send(Command::ProcessFrame {
                frame_id,
                left_image,
                right_image,
                timestamp_ns,
                imu_data,
                respond_to,
            })
            .await
            .map_err(|_| anyhow::anyhow!("AsyncEstimator worker closed"))?;

        response_rx
            .await
            .map_err(|_| anyhow::anyhow!("AsyncEstimator response dropped"))?
    }

    /// Get current state asynchronously (placeholder)
    /// Shutdown the async estimator gracefully
    pub async fn shutdown(mut self) {
        if let Some(handle) = self.join_handle.take() {
            let (respond_to, response_rx) = oneshot::channel();
            let _ = self.command_tx.send(Command::Shutdown(respond_to)).await;
            let _ = response_rx.await;
            let _ = handle.join();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn test_async_estimator_creation() {
        // Smoke test: ensure async estimator can be created and shutdown.
        use crate::datasets::config::{
            CameraConfig, Config, FeatureDetectionConfig, KeyframeManagementConfig,
            OptimizationConfig,
        };

        let config = Config {
            camera: CameraConfig {
                image_width: 640,
                image_height: 480,
                left_intrinsics: vec![500.0, 500.0, 320.0, 240.0],
                left_distortion: vec![0.0, 0.0, 0.0, 0.0, 0.0],
                right_intrinsics: vec![500.0, 500.0, 320.0, 240.0],
                right_distortion: vec![0.0, 0.0, 0.0, 0.0, 0.0],
                left_model: None,
                right_model: None,
                T_B_Cl: vec![
                    1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
                ],
                T_B_Cr: vec![
                    1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
                ],
            },
            keyframe_management: KeyframeManagementConfig {
                keyframe_window_size: 8,
                translation_threshold: 0.2,
                rotation_threshold: 0.1,
            },
            feature_detection: FeatureDetectionConfig {
                grid_cols: 16,
                max_features_per_grid: 80,
                optical_flow_max_iterations: 30,
                optical_flow_convergence_threshold: 0.01,
            },
            optimization: OptimizationConfig {
                bundle_adjustment_max_iterations: 5,
                pnp_max_iterations: 5,
            },
            calibration: None,
        };

        let estimator = AsyncEstimator::new_with_cameras(config, None, None, None);
        estimator.shutdown().await;
    }

    #[tokio::test]
    async fn test_async_estimator_process_frame_single() {
        use crate::datasets::config::{
            CameraConfig, Config, FeatureDetectionConfig, KeyframeManagementConfig,
            OptimizationConfig,
        };

        let config = Config {
            camera: CameraConfig {
                image_width: 640,
                image_height: 480,
                left_intrinsics: vec![500.0, 500.0, 320.0, 240.0],
                left_distortion: vec![0.0, 0.0, 0.0, 0.0, 0.0],
                right_intrinsics: vec![500.0, 500.0, 320.0, 240.0],
                right_distortion: vec![0.0, 0.0, 0.0, 0.0, 0.0],
                left_model: None,
                right_model: None,
                T_B_Cl: vec![
                    1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
                ],
                T_B_Cr: vec![
                    1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
                ],
            },
            keyframe_management: KeyframeManagementConfig {
                keyframe_window_size: 8,
                translation_threshold: 0.2,
                rotation_threshold: 0.1,
            },
            feature_detection: FeatureDetectionConfig {
                grid_cols: 16,
                max_features_per_grid: 80,
                optical_flow_max_iterations: 30,
                optical_flow_convergence_threshold: 0.01,
            },
            optimization: OptimizationConfig {
                bundle_adjustment_max_iterations: 5,
                pnp_max_iterations: 5,
            },
            calibration: None,
        };

        let estimator = AsyncEstimator::new_with_cameras(config, None, None, None);

        // Create dummy images
        let dummy_left = vec![128u8; 640 * 480];
        let dummy_right = vec![128u8; 640 * 480];

        // Process a single frame
        let result = estimator
            .process_frame_async(0, dummy_left, dummy_right, 1_000_000_000, None)
            .await;

        // Should succeed (even with dummy data)
        assert!(
            result.is_ok(),
            "process_frame_async should succeed with dummy images"
        );

        estimator.shutdown().await;
    }

    #[tokio::test]
    async fn test_async_estimator_multiple_frames() {
        use crate::datasets::config::{
            CameraConfig, Config, FeatureDetectionConfig, KeyframeManagementConfig,
            OptimizationConfig,
        };

        let config = Config {
            camera: CameraConfig {
                image_width: 640,
                image_height: 480,
                left_intrinsics: vec![500.0, 500.0, 320.0, 240.0],
                left_distortion: vec![0.0, 0.0, 0.0, 0.0, 0.0],
                right_intrinsics: vec![500.0, 500.0, 320.0, 240.0],
                right_distortion: vec![0.0, 0.0, 0.0, 0.0, 0.0],
                left_model: None,
                right_model: None,
                T_B_Cl: vec![
                    1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
                ],
                T_B_Cr: vec![
                    1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
                ],
            },
            keyframe_management: KeyframeManagementConfig {
                keyframe_window_size: 8,
                translation_threshold: 0.2,
                rotation_threshold: 0.1,
            },
            feature_detection: FeatureDetectionConfig {
                grid_cols: 16,
                max_features_per_grid: 80,
                optical_flow_max_iterations: 30,
                optical_flow_convergence_threshold: 0.01,
            },
            optimization: OptimizationConfig {
                bundle_adjustment_max_iterations: 5,
                pnp_max_iterations: 5,
            },
            calibration: None,
        };

        let estimator = AsyncEstimator::new_with_cameras(config, None, None, None);

        // Process multiple frames
        for frame_id in 0..5 {
            let dummy_left = vec![128u8; 640 * 480];
            let dummy_right = vec![128u8; 640 * 480];
            let timestamp = 1_000_000_000 + (frame_id * 33_000_000); // ~30 fps

            let result = estimator
                .process_frame_async(frame_id as i64, dummy_left, dummy_right, timestamp, None)
                .await;

            assert!(
                result.is_ok(),
                "process_frame_async should succeed for frame {}",
                frame_id
            );
        }

        estimator.shutdown().await;
    }

    #[tokio::test]
    async fn test_async_estimator_channel_robustness() {
        use crate::datasets::config::{
            CameraConfig, Config, FeatureDetectionConfig, KeyframeManagementConfig,
            OptimizationConfig,
        };
        use std::sync::Arc;

        let config = Config {
            camera: CameraConfig {
                image_width: 640,
                image_height: 480,
                left_intrinsics: vec![500.0, 500.0, 320.0, 240.0],
                left_distortion: vec![0.0, 0.0, 0.0, 0.0, 0.0],
                right_intrinsics: vec![500.0, 500.0, 320.0, 240.0],
                right_distortion: vec![0.0, 0.0, 0.0, 0.0, 0.0],
                left_model: None,
                right_model: None,
                T_B_Cl: vec![
                    1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
                ],
                T_B_Cr: vec![
                    1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
                ],
            },
            keyframe_management: KeyframeManagementConfig {
                keyframe_window_size: 8,
                translation_threshold: 0.2,
                rotation_threshold: 0.1,
            },
            feature_detection: FeatureDetectionConfig {
                grid_cols: 16,
                max_features_per_grid: 80,
                optical_flow_max_iterations: 30,
                optical_flow_convergence_threshold: 0.01,
            },
            optimization: OptimizationConfig {
                bundle_adjustment_max_iterations: 5,
                pnp_max_iterations: 5,
            },
            calibration: None,
        };

        let estimator = Arc::new(AsyncEstimator::new_with_cameras(config, None, None, None));

        // Test concurrent submissions (stress test the channel)
        let mut handles = vec![];
        for i in 0..3 {
            let est_clone = Arc::clone(&estimator);
            let handle = tokio::spawn(async move {
                let dummy_left = vec![128u8; 640 * 480];
                let dummy_right = vec![128u8; 640 * 480];
                let timestamp = 1_000_000_000 + (i * 33_000_000);

                est_clone
                    .process_frame_async(i as i64, dummy_left, dummy_right, timestamp, None)
                    .await
            });
            handles.push(handle);
        }

        // Wait for all tasks
        for handle in handles {
            let result = handle.await;
            assert!(result.is_ok(), "concurrent frame submission should succeed");
        }

        // cleanup happens when Arc is dropped at end of test
    }
}
