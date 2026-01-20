use super::*;
use crate::estimator::Frame;
use crate::optimization::marginalization::ParamId;
use crate::optimization::tight_coupling::ImuPreintegration;
use crate::types::{Matrix4x4, Vector3};

#[test]
fn evicts_when_over_capacity() {
    let mut window = SlidingWindow::new(4);
    window.set_max_map_points(10);

    for id in 0..20 {
        window.map_points.insert(id, [0.0, 0.0, 0.0]);
        window.map_point_observations.insert(id, id);
    }

    window.evict_old_map_points();

    assert!(window.map_points_len() <= 10);
    for _id in 10..20 {
        assert!(window.map_points.contains_key(&(_id as usize)));
    }
}

#[test]
fn triangulate_stereo_parallel_rays() {
    let left_obs = Vector3::new(1.0, 0.0, 1.0);
    let right_obs = Vector3::new(1.0, 0.0, 1.0);

    let T_W_B = Matrix4x4::identity();
    let T_B_Cl = Matrix4x4::identity();
    let T_B_Cr = Matrix4x4::new(
        1.0, 0.0, 0.0, 0.1, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
    );

    let result = SlidingWindow::triangulate_stereo(left_obs, right_obs, T_W_B, T_B_Cl, T_B_Cr);

    assert!(result.is_none());
}

#[test]
fn triangulate_stereo_recovers_forward_point() {
    let baseline = 0.1;
    let true_point = Vector3::new(0.0, 0.0, 5.0);

    let left_obs = Vector3::new(
        true_point.x / true_point.z,
        true_point.y / true_point.z,
        1.0,
    );
    let right_obs = Vector3::new(
        (true_point.x - baseline) / true_point.z,
        true_point.y / true_point.z,
        1.0,
    );

    let T_W_B = Matrix4x4::identity();
    let T_B_Cl = Matrix4x4::identity();
    let T_B_Cr = Matrix4x4::new(
        1.0, 0.0, 0.0, baseline, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
    );

    let result = SlidingWindow::triangulate_stereo(left_obs, right_obs, T_W_B, T_B_Cl, T_B_Cr);

    let p = result.expect("Triangulation should succeed for a forward point");
    assert!(p.z > 0.0);
    assert!((p.x - true_point.x).abs() < 1e-3);
    assert!((p.y - true_point.y).abs() < 1e-3);
    assert!((p.z - true_point.z).abs() < 1e-2);
}

#[test]
fn triangulate_stereo_behind_camera() {
    let left_obs = Vector3::new(-0.5, 0.0, 1.0);
    let right_obs = Vector3::new(-0.4, 0.0, 1.0);

    let T_W_B = Matrix4x4::identity();
    let T_B_Cl = Matrix4x4::identity();
    let T_B_Cr = Matrix4x4::new(
        1.0, 0.0, 0.0, 0.1, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
    );

    let result = SlidingWindow::triangulate_stereo(left_obs, right_obs, T_W_B, T_B_Cl, T_B_Cr);

    assert!(result.is_none());
}

#[test]
fn add_imu_preintegration_stores_data() {
    let mut window = SlidingWindow::new(4);

    let preintegration = ImuPreintegration {
        dt: 0.05,
        delta_R: nalgebra::Matrix3::identity(),
        delta_v: Vector3::new(0.1, 0.0, 0.0),
        delta_p: Vector3::new(0.005, 0.0, 0.0),
        cov_R: nalgebra::Matrix3::identity() * 1e-4,
        cov_v: nalgebra::Matrix3::identity() * 1e-4,
        cov_p: nalgebra::Matrix3::identity() * 1e-6,
        cov_R_bw: nalgebra::Matrix3::zeros(),
        cov_v_ba: nalgebra::Matrix3::zeros(),
        cov_p_ba: nalgebra::Matrix3::zeros(),
    };

    window.add_imu_preintegration(0, 1, preintegration.clone());

    assert_eq!(window.imu_preintegrations.len(), 1);
    assert!((window.imu_preintegrations[0].dt - 0.05).abs() < 1e-10);
    assert!((window.imu_preintegrations[0].delta_v.x - 0.1).abs() < 1e-10);
}

#[test]
fn add_imu_preintegration_multiple_pairs() {
    let mut window = SlidingWindow::new(5);

    for i in 0..3 {
        let preintegration = ImuPreintegration {
            dt: 0.1 * (i as f64 + 1.0),
            delta_R: nalgebra::Matrix3::identity(),
            delta_v: Vector3::new(0.1 * (i as f64 + 1.0), 0.0, 0.0),
            delta_p: Vector3::new(0.005 * (i as f64 + 1.0), 0.0, 0.0),
            cov_R: nalgebra::Matrix3::identity() * 1e-4,
            cov_v: nalgebra::Matrix3::identity() * 1e-4,
            cov_p: nalgebra::Matrix3::identity() * 1e-6,
            cov_R_bw: nalgebra::Matrix3::zeros(),
            cov_v_ba: nalgebra::Matrix3::zeros(),
            cov_p_ba: nalgebra::Matrix3::zeros(),
        };

        window.add_imu_preintegration(i, i + 1, preintegration);
    }

    assert_eq!(window.imu_preintegrations.len(), 3);
    assert!((window.imu_preintegrations[0].dt - 0.1).abs() < 1e-10);
    assert!((window.imu_preintegrations[1].dt - 0.2).abs() < 1e-10);
    assert!((window.imu_preintegrations[2].dt - 0.3).abs() < 1e-10);
}

#[test]
fn add_imu_preintegration_overwrites_existing() {
    let mut window = SlidingWindow::new(4);

    let preintegration1 = ImuPreintegration {
        dt: 0.05,
        delta_R: nalgebra::Matrix3::identity(),
        delta_v: Vector3::zeros(),
        delta_p: Vector3::zeros(),
        cov_R: nalgebra::Matrix3::identity() * 1e-4,
        cov_v: nalgebra::Matrix3::identity() * 1e-4,
        cov_p: nalgebra::Matrix3::identity() * 1e-6,
        cov_R_bw: nalgebra::Matrix3::zeros(),
        cov_v_ba: nalgebra::Matrix3::zeros(),
        cov_p_ba: nalgebra::Matrix3::zeros(),
    };

    window.add_imu_preintegration(0, 1, preintegration1.clone());

    let preintegration2 = ImuPreintegration {
        dt: 0.1,
        delta_R: nalgebra::Matrix3::identity(),
        delta_v: Vector3::new(0.5, 0.0, 0.0),
        delta_p: Vector3::new(0.05, 0.0, 0.0),
        cov_R: nalgebra::Matrix3::identity() * 1e-4,
        cov_v: nalgebra::Matrix3::identity() * 1e-4,
        cov_p: nalgebra::Matrix3::identity() * 1e-6,
        cov_R_bw: nalgebra::Matrix3::zeros(),
        cov_v_ba: nalgebra::Matrix3::zeros(),
        cov_p_ba: nalgebra::Matrix3::zeros(),
    };

    window.add_imu_preintegration(0, 1, preintegration2.clone());

    assert_eq!(window.imu_preintegrations.len(), 1);
    assert!((window.imu_preintegrations[0].dt - 0.1).abs() < 1e-10);
    assert!((window.imu_preintegrations[0].delta_v.x - 0.5).abs() < 1e-10);
}

#[test]
fn imu_preintegrations_empty_initially() {
    let window = SlidingWindow::new(4);
    assert!(window.imu_preintegrations.is_empty());
}

#[test]
fn sliding_window_with_velocity_states() {
    let mut window = SlidingWindow::new(4);

    let frame = Frame::from_stereo_images(
        0,
        0,
        crate::types::CameraFactory::opencv5(
            500.0, 500.0, 320.0, 240.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0,
        ),
        crate::types::CameraFactory::opencv5(
            500.0, 500.0, 320.0, 240.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0,
        ),
        Matrix4x4::identity(),
        Matrix4x4::identity(),
    );

    assert!(window.add_frame(frame));
    assert_eq!(window.len(), 1);
}

#[test]
fn build_param_blocks_includes_velocity() {
    let mut window = SlidingWindow::new(4);

    let frame = Frame::from_stereo_images(
        0,
        0,
        crate::types::CameraFactory::opencv5(
            500.0, 500.0, 320.0, 240.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0,
        ),
        crate::types::CameraFactory::opencv5(
            500.0, 500.0, 320.0, 240.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0,
        ),
        Matrix4x4::identity(),
        Matrix4x4::identity(),
    );

    window.add_frame(frame);

    let param_blocks = window.build_param_blocks_for_marginalization();

    assert!(param_blocks.contains_key(&ParamId::KeyframePose(0)));
    assert!(param_blocks.contains_key(&ParamId::KeyframeVelocity(0)));

    let vel_block = param_blocks.get(&ParamId::KeyframeVelocity(0)).unwrap();
    assert_eq!(vel_block.dimension, 3);
}

#[test]
fn cleanup_after_marginalization_removes_stale_preintegration() {
    let mut window = SlidingWindow::new(4);

    for i in 0..3 {
        let preintegration = ImuPreintegration {
            dt: 0.1 * (i + 1) as f64,
            delta_R: nalgebra::Matrix3::identity(),
            delta_v: Vector3::zeros(),
            delta_p: Vector3::zeros(),
            cov_R: nalgebra::Matrix3::identity() * 1e-4,
            cov_v: nalgebra::Matrix3::identity() * 1e-4,
            cov_p: nalgebra::Matrix3::identity() * 1e-6,
            cov_R_bw: nalgebra::Matrix3::zeros(),
            cov_v_ba: nalgebra::Matrix3::zeros(),
            cov_p_ba: nalgebra::Matrix3::zeros(),
        };
        window.add_imu_preintegration(i, i + 1, preintegration);
    }

    assert_eq!(window.imu_preintegrations.len(), 3);

    window.cleanup_after_marginalization(0);

    assert_eq!(window.imu_preintegrations.len(), 2);
    assert!((window.imu_preintegrations[0].dt - 0.2).abs() < 1e-10);
}

#[test]
fn cleanup_after_marginalization_empty_when_no_preintegrations() {
    let mut window = SlidingWindow::new(4);
    assert!(window.imu_preintegrations.is_empty());

    window.cleanup_after_marginalization(0);

    assert!(window.imu_preintegrations.is_empty());
}

#[test]
fn param_id_variants_are_correct() {
    let pose = ParamId::KeyframePose(0);
    let vel = ParamId::KeyframeVelocity(0);
    let landmark = ParamId::Landmark(0);

    assert_ne!(pose, vel);
    assert_ne!(vel, landmark);
    assert_ne!(pose, landmark);
}
