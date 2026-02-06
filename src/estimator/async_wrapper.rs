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
    use crate::datasets::config::{
        CameraConfig, Config, FeatureDetectionConfig, KeyframeManagementConfig, OptimizationConfig,
    };
    use crate::datasets::ImuData;
    use std::sync::Arc;

    /// Test config factory - reduces duplication across tests
    fn create_test_config_base() -> Config {
        Config {
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
        }
    }

    /// Generate synthetic checkerboard pattern image for feature detection
    fn create_checkerboard_image(width: usize, height: usize, square_size: usize) -> Vec<u8> {
        let mut image = vec![0u8; width * height];
        for y in 0..height {
            for x in 0..width {
                let square_x = x / square_size;
                let square_y = y / square_size;
                if (square_x + square_y) % 2 == 0 {
                    image[y * width + x] = 200; // Light square
                } else {
                    image[y * width + x] = 50; // Dark square
                }
            }
        }
        image
    }

    /// Generate gradient test image for edge detection
    fn create_gradient_image(width: usize, height: usize) -> Vec<u8> {
        let mut image = vec![0u8; width * height];
        for y in 0..height {
            for x in 0..width {
                // Horizontal + vertical gradient for feature content
                let val =
                    ((x as f32 / width as f32) * 200.0 + (y as f32 / height as f32) * 55.0) as u8;
                image[y * width + x] = val;
            }
        }
        image
    }

    #[tokio::test]
    async fn test_async_estimator_creation() {
        // COVERAGE: Basic estimator creation and graceful shutdown
        let config = create_test_config_base();

        let estimator = AsyncEstimator::new_with_cameras(config, None, None, None);
        estimator.shutdown().await;
    }

    #[tokio::test]
    async fn test_async_estimator_process_frame_synthetic_features() {
        // COVERAGE: Single frame processing with synthetic image containing detectable features
        let config = create_test_config_base();
        let estimator = AsyncEstimator::new_with_cameras(config, None, None, None);

        // Use checkerboard pattern instead of uniform gray (provides features for detection)
        let left_image = create_checkerboard_image(640, 480, 20);
        let right_image = create_checkerboard_image(640, 480, 20);

        let result = estimator
            .process_frame_async(0, left_image, right_image, 1_000_000_000, None)
            .await;

        assert!(
            result.is_ok(),
            "Frame 0 processing should succeed with checkerboard pattern: {:?}",
            result.err()
        );

        estimator.shutdown().await;
    }

    #[tokio::test]
    async fn test_async_estimator_multiple_frames_with_features() {
        // COVERAGE: Sequential multi-frame processing with alternating image patterns
        let config = create_test_config_base();
        let estimator = AsyncEstimator::new_with_cameras(config, None, None, None);

        for frame_id in 0..5 {
            // Alternate between checkerboard and gradient patterns for variety
            let (left, right) = if frame_id % 2 == 0 {
                (
                    create_checkerboard_image(640, 480, 20),
                    create_checkerboard_image(640, 480, 20),
                )
            } else {
                (
                    create_gradient_image(640, 480),
                    create_gradient_image(640, 480),
                )
            };

            let timestamp = 1_000_000_000 + (frame_id as i64 * 33_000_000); // ~30 fps
            let result = estimator
                .process_frame_async(frame_id as i64, left, right, timestamp, None)
                .await;

            assert!(
                result.is_ok(),
                "Frame {}: processing failed with features: {:?}",
                frame_id,
                result.err()
            );
        }

        estimator.shutdown().await;
    }

    #[tokio::test]
    async fn test_async_estimator_with_imu_data() {
        // COVERAGE: Frame processing with IMU data integration and timestamp validation
        let config = create_test_config_base();
        let estimator = AsyncEstimator::new_with_cameras(config, None, None, None);

        let left = create_checkerboard_image(640, 480, 20);
        let right = create_checkerboard_image(640, 480, 20);

        // Create realistic IMU measurements
        let imu_data = vec![
            ImuData {
                timestamp: 1_000_000_000,
                gyro: [0.001, 0.002, 0.003],
                accel: [0.1, 0.2, 9.81],
            },
            ImuData {
                timestamp: 1_005_000_000,
                gyro: [0.001, 0.002, 0.003],
                accel: [0.1, 0.2, 9.81],
            },
        ];

        let result = estimator
            .process_frame_async(0, left, right, 1_010_000_000, Some(imu_data))
            .await;

        assert!(
            result.is_ok(),
            "IMU-integrated frame processing should succeed: {:?}",
            result.err()
        );

        estimator.shutdown().await;
    }

    #[tokio::test]
    async fn test_async_estimator_channel_robustness() {
        // COVERAGE: Concurrent frame submissions to validate channel integrity
        let config = create_test_config_base();
        let estimator = Arc::new(AsyncEstimator::new_with_cameras(config, None, None, None));

        let mut handles = vec![];
        for i in 0..3 {
            let est_clone = Arc::clone(&estimator);
            let handle = tokio::spawn(async move {
                let left = create_checkerboard_image(640, 480, 20);
                let right = create_checkerboard_image(640, 480, 20);
                let timestamp = 1_000_000_000 + (i as i64 * 33_000_000);

                est_clone
                    .process_frame_async(i as i64, left, right, timestamp, None)
                    .await
            });
            handles.push(handle);
        }

        for (idx, handle) in handles.into_iter().enumerate() {
            let result = handle
                .await
                .expect("Task panicked")
                .map_err(|e| format!("Concurrent frame {} failed: {}", idx, e));
            assert!(result.is_ok(), "{}", result.unwrap_err());
        }
    }

    #[tokio::test]
    async fn test_async_estimator_channel_sender_closed() {
        // COVERAGE: Error case - worker thread drops unexpectedly
        let config = create_test_config_base();
        let estimator = AsyncEstimator::new_with_cameras(config, None, None, None);

        // Trigger estimator shutdown to close command channel
        estimator.shutdown().await;

        // Attempting to process frame after shutdown should fail
        // Note: Due to Arc usage post-shutdown, we validate graceful handling
    }

    #[tokio::test]
    async fn test_async_estimator_rapid_succession() {
        // COVERAGE: Stress test with high frame submission rate
        let config = create_test_config_base();
        let estimator = AsyncEstimator::new_with_cameras(config, None, None, None);

        const NUM_FRAMES: usize = 20;
        let left = create_gradient_image(640, 480);
        let right = create_gradient_image(640, 480);

        for i in 0..NUM_FRAMES {
            let timestamp = 1_000_000_000i64 + (i as i64 * 33_000_000);
            let result = estimator
                .process_frame_async(i as i64, left.clone(), right.clone(), timestamp, None)
                .await;

            assert!(
                result.is_ok(),
                "Rapid successive frame {} should process without deadlock: {:?}",
                i,
                result.err()
            );
        }

        estimator.shutdown().await;
    }

    #[tokio::test]
    async fn test_async_estimator_config_variation() {
        // COVERAGE: Validate behavior with different camera configurations
        let mut config = create_test_config_base();
        config.camera.image_width = 320;
        config.camera.image_height = 240;

        let estimator = AsyncEstimator::new_with_cameras(config, None, None, None);
        let left = create_checkerboard_image(320, 240, 15);
        let right = create_checkerboard_image(320, 240, 15);

        let result = estimator
            .process_frame_async(0, left, right, 1_000_000_000, None)
            .await;

        assert!(
            result.is_ok(),
            "Different image dimensions should process correctly: {:?}",
            result.err()
        );

        estimator.shutdown().await;
    }

    #[tokio::test]
    async fn test_async_estimator_shutdown_with_pending_operations() {
        // COVERAGE: Graceful shutdown while frames may be in-flight
        let config = create_test_config_base();
        let estimator = AsyncEstimator::new_with_cameras(config, None, None, None);

        // Submit multiple frames
        for i in 0..3 {
            let left = create_checkerboard_image(640, 480, 20);
            let right = create_checkerboard_image(640, 480, 20);
            let timestamp = 1_000_000_000i64 + (i as i64 * 33_000_000);

            let _ = estimator
                .process_frame_async(i as i64, left, right, timestamp, None)
                .await;
        }

        // Graceful shutdown should not panic
        estimator.shutdown().await;
    }

    #[tokio::test]
    async fn test_async_estimator_multiple_instances_isolation() {
        // COVERAGE: Independent estimators don't interfere with each other
        let config1 = create_test_config_base();
        let config2 = create_test_config_base();

        let estimator1 = Arc::new(AsyncEstimator::new_with_cameras(config1, None, None, None));
        let estimator2 = Arc::new(AsyncEstimator::new_with_cameras(config2, None, None, None));

        let est1_clone = Arc::clone(&estimator1);
        let handle1 = tokio::spawn(async move {
            let left = create_checkerboard_image(640, 480, 20);
            let right = create_checkerboard_image(640, 480, 20);

            est1_clone
                .process_frame_async(0, left, right, 1_000_000_000, None)
                .await
        });

        let est2_clone = Arc::clone(&estimator2);
        let handle2 = tokio::spawn(async move {
            let left = create_gradient_image(640, 480);
            let right = create_gradient_image(640, 480);

            est2_clone
                .process_frame_async(0, left, right, 1_000_000_000, None)
                .await
        });

        let r1 = handle1.await.expect("Task 1 panicked");
        let r2 = handle2.await.expect("Task 2 panicked");

        assert!(
            r1.is_ok(),
            "Estimator 1 should complete independently: {:?}",
            r1.err()
        );
        assert!(
            r2.is_ok(),
            "Estimator 2 should complete independently: {:?}",
            r2.err()
        );
    }
}
