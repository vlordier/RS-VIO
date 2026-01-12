#![allow(
  clippy::unwrap_used,
  clippy::expect_used,
  clippy::panic,
  clippy::float_cmp
)]

use rs_vio::datasets::euroc_player::EurocPlayer;
use rs_vio::datasets::fourseasons_player::FourSeasonsPlayer;
use rs_vio::datasets::player_trait::DatasetPlayer;
use rs_vio::datasets::tum_vi_player::TUMVIPlayer;
use rs_vio::types::Matrix4x4;
use rs_vio::*;
use std::fs;

#[cfg(test)]
mod euroc_player_tests {
    use super::*;

    #[test]
    fn test_euroc_load_imu_data_valid_format() {
        let temp_dir = tempfile::TempDir::new().unwrap();
        let dataset_path = temp_dir.path().to_str().unwrap();

        let imu_content = "#timestamp [ns],omega_x [rad/s],omega_y [rad/s],omega_z [rad/s],alpha_x [m/s^2],alpha_y [m/s^2],alpha_z [m/s^2]\n1000000000,0.001,0.002,0.003,9.8,9.81,9.82\n2000000000,0.004,0.005,0.006,9.79,9.80,9.81\n";
        let imu_path = temp_dir.path().join("mav0/imu0/data.csv");

        // Create parent directories first
        fs::create_dir_all(imu_path.parent().unwrap()).unwrap();
        fs::write(&imu_path, imu_content).unwrap();

        fs::create_dir_all(temp_dir.path().join("mav0/cam0/data")).unwrap();

        let player = EurocPlayer::new();
        let result = player.load_imu_data(dataset_path, &[], 0, 0);

        assert!(result.is_ok());
    }

    #[test]
    fn test_euroc_load_imu_data_file_not_found() {
        let temp_dir = tempfile::TempDir::new().unwrap();
        let dataset_path = temp_dir.path().to_str().unwrap();

        // EuRoC does not check file existence - it returns an error when trying to open
        // This is expected behavior
        let player = EurocPlayer::new();
        let result = player.load_imu_data(dataset_path, &[], 0, 0);

        // File doesn't exist so this will return an error
        assert!(result.is_err());
    }

    #[test]
    fn test_euroc_save_trajectories_creates_file() {
        let temp_dir = tempfile::TempDir::new().unwrap();
        let dataset_path = temp_dir.path().to_str().unwrap();
        let trajectory_path = temp_dir.path().join("trajectory.txt");

        let yaml_config = r#"
camera:
  image_width: 640
  image_height: 480
  left_intrinsics: [500.0, 500.0, 320.0, 240.0]
  left_distortion: [0.0, 0.0, 0.0, 0.0]
  right_intrinsics: [500.0, 500.0, 320.0, 240.0]
  right_distortion: [0.0, 0.0, 0.0, 0.0]
  left_model: pinhole-radtan
  T_B_Cl: [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
  T_B_Cr: [1.0, 0.0, 0.0, 0.1, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
keyframe_management:
  keyframe_window_size: 5
  translation_threshold: 0.1
  rotation_threshold: 0.1
feature_detection:
  grid_size: 10
  max_features_per_grid: 50
  optical_flow_max_iterations: 30
  optical_flow_convergence_threshold: 0.01
optimization:
  bundle_adjustment_max_iterations: 10
  pnp_max_iterations: 8
"#;
        let config: datasets::config::Config = serde_yaml::from_str(yaml_config).unwrap();
        let estimator = estimator::Estimator::new(config, None);

        let context = datasets::FrameContext {
            current_idx: 0,
            processed_frames: 1,
            previous_frame_timestamp: 1000000000,
            step_mode: false,
            auto_play: true,
            advance_frame: true,
        };

        let player = EurocPlayer::new();
        player.save_trajectories(&estimator, &context, dataset_path);

        assert!(trajectory_path.exists());
    }

    #[test]
    fn test_euroc_initialize_estimator() {
        let yaml_config = r#"
camera:
  image_width: 640
  image_height: 480
  left_intrinsics: [500.0, 500.0, 320.0, 240.0]
  left_distortion: [0.0, 0.0, 0.0, 0.0]
  right_intrinsics: [500.0, 500.0, 320.0, 240.0]
  right_distortion: [0.0, 0.0, 0.0, 0.0]
  left_model: pinhole-radtan
  T_B_Cl: [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
  T_B_Cr: [1.0, 0.0, 0.0, 0.1, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
keyframe_management:
  keyframe_window_size: 5
  translation_threshold: 0.1
  rotation_threshold: 0.1
feature_detection:
  grid_size: 10
  max_features_per_grid: 50
  optical_flow_max_iterations: 30
  optical_flow_convergence_threshold: 0.01
optimization:
  bundle_adjustment_max_iterations: 10
  pnp_max_iterations: 8
"#;
        let config: datasets::config::Config = serde_yaml::from_str(yaml_config).unwrap();
        let mut estimator = estimator::Estimator::new(config, None);

        let player = EurocPlayer::new();
        player.initialize_estimator(&mut estimator, &[]);
    }
}

#[cfg(test)]
mod fourseasons_player_tests {
    use super::*;

    #[test]
    fn test_fourseasons_load_imu_data_valid_format() {
        let temp_dir = tempfile::TempDir::new().unwrap();
        let dataset_path = temp_dir.path().to_str().unwrap();

        let imu_content = "1000000000 0.001 0.002 0.003 9.8 9.81 9.82\n2000000000 0.004 0.005 0.006 9.79 9.80 9.81\n";
        let imu_path = temp_dir.path().join("imu.txt");
        fs::write(&imu_path, imu_content).unwrap();

        fs::create_dir_all(temp_dir.path().join("mav0/cam0/data")).unwrap();

        let player = FourSeasonsPlayer::new();
        let result = player.load_imu_data(dataset_path, &[], 0, 0);

        assert!(result.is_ok());
    }

    #[test]
    fn test_fourseasons_load_imu_data_file_not_found() {
        let temp_dir = tempfile::TempDir::new().unwrap();
        let dataset_path = temp_dir.path().to_str().unwrap();

        let player = FourSeasonsPlayer::new();
        let result = player.load_imu_data(dataset_path, &[], 0, 0);

        assert!(result.is_ok());
    }

    #[test]
    fn test_fourseasons_save_trajectories_creates_file() {
        let temp_dir = tempfile::TempDir::new().unwrap();
        let dataset_path = temp_dir.path().to_str().unwrap();
        let trajectory_path = temp_dir.path().join("trajectory.txt");

        let yaml_config = r#"
camera:
  image_width: 640
  image_height: 480
  left_intrinsics: [500.0, 500.0, 320.0, 240.0]
  left_distortion: [0.0, 0.0, 0.0, 0.0]
  right_intrinsics: [500.0, 500.0, 320.0, 240.0]
  right_distortion: [0.0, 0.0, 0.0, 0.0]
  left_model: pinhole-radtan
  T_B_Cl: [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
  T_B_Cr: [1.0, 0.0, 0.0, 0.1, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
keyframe_management:
  keyframe_window_size: 5
  translation_threshold: 0.1
  rotation_threshold: 0.1
feature_detection:
  grid_size: 10
  max_features_per_grid: 50
  optical_flow_max_iterations: 30
  optical_flow_convergence_threshold: 0.01
optimization:
  bundle_adjustment_max_iterations: 10
  pnp_max_iterations: 8
"#;
        let config: datasets::config::Config = serde_yaml::from_str(yaml_config).unwrap();
        let estimator = estimator::Estimator::new(config, None);

        let context = datasets::FrameContext {
            current_idx: 0,
            processed_frames: 1,
            previous_frame_timestamp: 1000000000,
            step_mode: false,
            auto_play: true,
            advance_frame: true,
        };

        let player = FourSeasonsPlayer::new();
        player.save_trajectories(&estimator, &context, dataset_path);

        assert!(trajectory_path.exists());
    }

    #[test]
    fn test_fourseasons_initialize_estimator() {
        let yaml_config = r#"
camera:
  image_width: 640
  image_height: 480
  left_intrinsics: [500.0, 500.0, 320.0, 240.0]
  left_distortion: [0.0, 0.0, 0.0, 0.0]
  right_intrinsics: [500.0, 500.0, 320.0, 240.0]
  right_distortion: [0.0, 0.0, 0.0, 0.0]
  left_model: pinhole-radtan
  T_B_Cl: [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
  T_B_Cr: [1.0, 0.0, 0.0, 0.1, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
keyframe_management:
  keyframe_window_size: 5
  translation_threshold: 0.1
  rotation_threshold: 0.1
feature_detection:
  grid_size: 10
  max_features_per_grid: 50
  optical_flow_max_iterations: 30
  optical_flow_convergence_threshold: 0.01
optimization:
  bundle_adjustment_max_iterations: 10
  pnp_max_iterations: 8
"#;
        let config: datasets::config::Config = serde_yaml::from_str(yaml_config).unwrap();
        let mut estimator = estimator::Estimator::new(config, None);

        let player = FourSeasonsPlayer::new();
        player.initialize_estimator(&mut estimator, &[]);
    }
}

#[cfg(test)]
mod tumvi_player_tests {
    use super::*;

    #[test]
    fn test_tumvi_load_imu_data_valid_format() {
        let temp_dir = tempfile::TempDir::new().unwrap();
        let dataset_path = temp_dir.path().to_str().unwrap();

        let imu_content = "#timestamp [ns],omega_x [rad/s],omega_y [rad/s],omega_z [rad/s],alpha_x [m/s^2],alpha_y [m/s^2],alpha_z [m/s^2]\n1000000000,0.001,0.002,0.003,9.8,9.81,9.82\n2000000000,0.004,0.005,0.006,9.79,9.80,9.81\n";
        let imu_path = temp_dir.path().join("mav0/imu0/data.csv");

        // Create parent directories first
        fs::create_dir_all(imu_path.parent().unwrap()).unwrap();
        fs::write(&imu_path, imu_content).unwrap();

        fs::create_dir_all(temp_dir.path().join("mav0/cam0/data")).unwrap();

        let player = TUMVIPlayer::new();
        let result = player.load_imu_data(dataset_path, &[], 0, 0);

        assert!(result.is_ok());
    }

    #[test]
    fn test_tumvi_load_imu_data_file_not_found() {
        let temp_dir = tempfile::TempDir::new().unwrap();
        let dataset_path = temp_dir.path().to_str().unwrap();

        let player = TUMVIPlayer::new();
        let result = player.load_imu_data(dataset_path, &[], 0, 0);

        assert!(result.is_ok());
    }

    #[test]
    fn test_tumvi_save_trajectories_creates_file() {
        let temp_dir = tempfile::TempDir::new().unwrap();
        let dataset_path = temp_dir.path().to_str().unwrap();
        let trajectory_path = temp_dir.path().join("trajectory.txt");

        let yaml_config = r#"
camera:
  image_width: 640
  image_height: 480
  left_intrinsics: [500.0, 500.0, 320.0, 240.0]
  left_distortion: [0.0, 0.0, 0.0, 0.0]
  right_intrinsics: [500.0, 500.0, 320.0, 240.0]
  right_distortion: [0.0, 0.0, 0.0, 0.0]
  left_model: pinhole-radtan
  T_B_Cl: [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
  T_B_Cr: [1.0, 0.0, 0.0, 0.1, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
keyframe_management:
  keyframe_window_size: 5
  translation_threshold: 0.1
  rotation_threshold: 0.1
feature_detection:
  grid_size: 10
  max_features_per_grid: 50
  optical_flow_max_iterations: 30
  optical_flow_convergence_threshold: 0.01
optimization:
  bundle_adjustment_max_iterations: 10
  pnp_max_iterations: 8
"#;
        let config: datasets::config::Config = serde_yaml::from_str(yaml_config).unwrap();
        let estimator = estimator::Estimator::new(config, None);

        let context = datasets::FrameContext {
            current_idx: 0,
            processed_frames: 1,
            previous_frame_timestamp: 1000000000,
            step_mode: false,
            auto_play: true,
            advance_frame: true,
        };

        let player = TUMVIPlayer::new();
        player.save_trajectories(&estimator, &context, dataset_path);

        assert!(trajectory_path.exists());
    }

    #[test]
    fn test_tumvi_initialize_estimator() {
        let yaml_config = r#"
camera:
  image_width: 640
  image_height: 480
  left_intrinsics: [500.0, 500.0, 320.0, 240.0]
  left_distortion: [0.0, 0.0, 0.0, 0.0]
  right_intrinsics: [500.0, 500.0, 320.0, 240.0]
  right_distortion: [0.0, 0.0, 0.0, 0.0]
  left_model: pinhole-radtan
  T_B_Cl: [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
  T_B_Cr: [1.0, 0.0, 0.0, 0.1, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
keyframe_management:
  keyframe_window_size: 5
  translation_threshold: 0.1
  rotation_threshold: 0.1
feature_detection:
  grid_size: 10
  max_features_per_grid: 50
  optical_flow_max_iterations: 30
  optical_flow_convergence_threshold: 0.01
optimization:
  bundle_adjustment_max_iterations: 10
  pnp_max_iterations: 8
"#;
        let config: datasets::config::Config = serde_yaml::from_str(yaml_config).unwrap();
        let mut estimator = estimator::Estimator::new(config, None);

        let player = TUMVIPlayer::new();
        player.initialize_estimator(&mut estimator, &[]);
    }
}

#[cfg(test)]
mod trajectory_tests {
    use super::*;

    #[test]
    fn test_get_trajectory_empty() {
        let yaml_config = r#"
camera:
  image_width: 640
  image_height: 480
  left_intrinsics: [500.0, 500.0, 320.0, 240.0]
  left_distortion: [0.0, 0.0, 0.0, 0.0]
  right_intrinsics: [500.0, 500.0, 320.0, 240.0]
  right_distortion: [0.0, 0.0, 0.0, 0.0]
  left_model: pinhole-radtan
  T_B_Cl: [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
  T_B_Cr: [1.0, 0.0, 0.0, 0.1, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
keyframe_management:
  keyframe_window_size: 5
  translation_threshold: 0.1
  rotation_threshold: 0.1
feature_detection:
  grid_size: 10
  max_features_per_grid: 50
  optical_flow_max_iterations: 30
  optical_flow_convergence_threshold: 0.01
optimization:
  bundle_adjustment_max_iterations: 10
  pnp_max_iterations: 8
"#;
        let config: datasets::config::Config = serde_yaml::from_str(yaml_config).unwrap();
        let estimator = estimator::Estimator::new(config, None);

        let trajectory = estimator.get_trajectory();
        assert!(trajectory.is_empty());
    }

    #[test]
    fn test_get_trajectory_returns_correct_type() {
        let yaml_config = r#"
camera:
  image_width: 640
  image_height: 480
  left_intrinsics: [500.0, 500.0, 320.0, 240.0]
  left_distortion: [0.0, 0.0, 0.0, 0.0]
  right_intrinsics: [500.0, 500.0, 320.0, 240.0]
  right_distortion: [0.0, 0.0, 0.0, 0.0]
  left_model: pinhole-radtan
  T_B_Cl: [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
  T_B_Cr: [1.0, 0.0, 0.0, 0.1, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
keyframe_management:
  keyframe_window_size: 5
  translation_threshold: 0.1
  rotation_threshold: 0.1
feature_detection:
  grid_size: 10
  max_features_per_grid: 50
  optical_flow_max_iterations: 30
  optical_flow_convergence_threshold: 0.01
optimization:
  bundle_adjustment_max_iterations: 10
  pnp_max_iterations: 8
"#;
        let config: datasets::config::Config = serde_yaml::from_str(yaml_config).unwrap();
        let estimator = estimator::Estimator::new(config, None);

        let trajectory = estimator.get_trajectory();
        let _: &Vec<Matrix4x4> = trajectory;
    }
}

#[cfg(test)]
mod imu_retrieval_tests {
    use super::*;

    #[test]
    fn test_get_imu_data_between_frames_returns_empty() {
        let player_euroc = EurocPlayer::new();
        let player_4seasons = FourSeasonsPlayer::new();
        let player_tumvi = TUMVIPlayer::new();

        let result_euroc = player_euroc.get_imu_data_between_frames(0, 1000000000);
        let result_4seasons = player_4seasons.get_imu_data_between_frames(0, 1000000000);
        let result_tumvi = player_tumvi.get_imu_data_between_frames(0, 1000000000);

        assert!(result_euroc.is_empty());
        assert!(result_4seasons.is_empty());
        assert!(result_tumvi.is_empty());
    }

    #[test]
    fn test_euroc_imu_data_timestamp_filtering() {
        let temp_dir = tempfile::TempDir::new().unwrap();
        let dataset_path = temp_dir.path().to_str().unwrap();

        // Create IMU data with specific timestamps
        let imu_content = "#timestamp [ns],omega_x [rad/s],omega_y [rad/s],omega_z [rad/s],alpha_x [m/s^2],alpha_y [m/s^2],alpha_z [m/s^2]\n1000000000,0.001,0.002,0.003,9.8,9.81,9.82\n2000000000,0.004,0.005,0.006,9.79,9.80,9.81\n3000000000,0.007,0.008,0.009,9.78,9.79,9.80\n";
        let imu_path = temp_dir.path().join("mav0/imu0/data.csv");
        fs::create_dir_all(imu_path.parent().unwrap()).unwrap();
        fs::write(&imu_path, imu_content).unwrap();

        let player = EurocPlayer::new();
        player.load_imu_data(dataset_path, &[], 0, 0).unwrap();

        // Query: no data before first frame
        let result = player.get_imu_data_between_frames(0, 500000000);
        assert!(result.is_empty());

        // Query: should get t=1000000000 (exclusive start, inclusive end)
        let result = player.get_imu_data_between_frames(500000000, 1000000000);
        assert_eq!(result.len(), 1);
        assert_eq!(result[0].timestamp, 1000000000);

        // Query: should get t=2000000000 (t=1000000000 is excluded because > is exclusive)
        let result = player.get_imu_data_between_frames(1000000000, 2500000000);
        assert_eq!(result.len(), 1);
        assert_eq!(result[0].timestamp, 2000000000);

        // Query: should get all three
        let result = player.get_imu_data_between_frames(0, 4000000000);
        assert_eq!(result.len(), 3);

        // Verify IMU values are parsed correctly
        let result = player.get_imu_data_between_frames(0, 4000000000);
        assert_eq!(result[0].gyro, [0.001, 0.002, 0.003]);
        assert_eq!(result[0].accel, [9.8, 9.81, 9.82]);
        assert_eq!(result[1].gyro, [0.004, 0.005, 0.006]);
        assert_eq!(result[2].gyro, [0.007, 0.008, 0.009]);
    }

    #[test]
    fn test_fourseasons_imu_data_timestamp_filtering() {
        let temp_dir = tempfile::TempDir::new().unwrap();
        let dataset_path = temp_dir.path().to_str().unwrap();

        // Create 4Seasons IMU data (whitespace separated)
        let imu_content = "1000000000 0.001 0.002 0.003 9.8 9.81 9.82\n2000000000 0.004 0.005 0.006 9.79 9.80 9.81\n";
        let imu_path = temp_dir.path().join("imu.txt");
        fs::write(&imu_path, imu_content).unwrap();

        let player = FourSeasonsPlayer::new();
        player.load_imu_data(dataset_path, &[], 0, 0).unwrap();

        // Query between timestamps
        let result = player.get_imu_data_between_frames(1000000000, 2000000000);
        assert_eq!(result.len(), 1);
        assert_eq!(result[0].timestamp, 2000000000);
        assert_eq!(result[0].gyro, [0.004, 0.005, 0.006]);
    }

    #[test]
    fn test_tumvi_imu_data_timestamp_filtering() {
        let temp_dir = tempfile::TempDir::new().unwrap();
        let dataset_path = temp_dir.path().to_str().unwrap();

        // Create TUM-VI IMU data
        let imu_content = "#timestamp [ns],omega_x [rad/s],omega_y [rad/s],omega_z [rad/s],alpha_x [m/s^2],alpha_y [m/s^2],alpha_z [m/s^2]\n1000000000,0.001,0.002,0.003,9.8,9.81,9.82\n2500000000,0.010,0.020,0.030,9.7,9.71,9.72\n";
        let imu_path = temp_dir.path().join("mav0/imu0/data.csv");
        fs::create_dir_all(imu_path.parent().unwrap()).unwrap();
        fs::write(&imu_path, imu_content).unwrap();

        let player = TUMVIPlayer::new();
        player.load_imu_data(dataset_path, &[], 0, 0).unwrap();

        // Query between timestamps
        let result = player.get_imu_data_between_frames(1000000000, 3000000000);
        assert_eq!(result.len(), 1);
        assert_eq!(result[0].timestamp, 2500000000);
        assert_eq!(result[0].gyro, [0.010, 0.020, 0.030]);
    }
}
