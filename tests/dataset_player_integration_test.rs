// Integration tests for dataset players

use rs_vio::datasets::{ImageData, ImuData, PlayerConfig};

#[test]
fn test_player_config_creation() {
    let config = PlayerConfig {
        config_path: "/test/config.yaml".to_string(),
        dataset_path: "/test/euroc/MH_01_easy".to_string(),
        enable_statistics: true,
        enable_console_statistics: false,
        step_mode: false,
        stats_output_path: Some("/test/stats.csv".to_string()),
    };

    assert_eq!(config.config_path, "/test/config.yaml");
    assert_eq!(config.dataset_path, "/test/euroc/MH_01_easy");
    assert!(config.enable_statistics);
    assert!(!config.enable_console_statistics);
    assert!(!config.step_mode);
    assert!(config.stats_output_path.is_some());
}

#[test]
fn test_player_config_minimal() {
    let config = PlayerConfig {
        config_path: "config.yaml".to_string(),
        dataset_path: "/data".to_string(),
        enable_statistics: false,
        enable_console_statistics: false,
        step_mode: true,
        stats_output_path: None,
    };

    assert!(config.step_mode);
    assert!(config.stats_output_path.is_none());
}

#[test]
fn test_image_data_creation() {
    let img = ImageData {
        timestamp: 1234567890,
        filename: "cam0/data/000001.png".to_string(),
    };

    assert_eq!(img.timestamp, 1234567890);
    assert!(img.filename.contains("cam0"));
}

#[test]
fn test_imu_data_creation() {
    let imu = ImuData {
        timestamp: 1234567890,
        gyro: [0.1, 0.2, 0.3],
        accel: [9.8, 0.1, 0.0],
    };

    assert_eq!(imu.timestamp, 1234567890);
    let eps = 1e-9;
    assert!((imu.gyro[0] - 0.1).abs() < eps);
    assert!((imu.accel[0] - 9.8).abs() < eps);
}

#[test]
fn test_imu_data_clone() {
    let imu1 = ImuData {
        timestamp: 1000,
        gyro: [1.0, 2.0, 3.0],
        accel: [4.0, 5.0, 6.0],
    };

    let imu2 = imu1.clone();
    assert_eq!(imu1.timestamp, imu2.timestamp);
    let eps = 1e-9;
    assert!(imu1
        .gyro
        .iter()
        .zip(imu2.gyro.iter())
        .all(|(a, b)| (*a - *b).abs() < eps));
    assert!(imu1
        .accel
        .iter()
        .zip(imu2.accel.iter())
        .all(|(a, b)| (*a - *b).abs() < eps));
}

#[test]
fn test_image_data_clone() {
    let img1 = ImageData {
        timestamp: 5000,
        filename: "test.png".to_string(),
    };

    let img2 = img1.clone();
    assert_eq!(img1.timestamp, img2.timestamp);
    assert_eq!(img1.filename, img2.filename);
}

// Note: Actual dataset player tests (EuRoC, TUM-VI, 4Seasons) would require
// real dataset files or extensive mocking. These basic struct tests verify
// that the data types work correctly. Full integration tests should be done
// with actual datasets in a separate test environment.
