use crate::{
    constants::GRAVITY_MAGNITUDE,
    datasets::ImuData,
    estimator::Frame,
    msckf::{FilterState, StereoRig, VisualMode},
    types::{Float, UnitQuaternion, Vector2, Vector3},
};
use rustc_hash::FxHashMap;

#[derive(Debug, Clone, Copy)]
pub struct InitializationConfig {
    pub static_window_duration_s: Float,
    pub gyro_static_variance_threshold: Float,
    pub accel_magnitude_tolerance: Float,
    pub min_bootstrap_tracks: usize,
    pub fallback_gyro_bias_sigma: Float,
    pub fallback_accel_bias_sigma: Float,
}

impl Default for InitializationConfig {
    fn default() -> Self {
        Self {
            static_window_duration_s: 0.5,
            gyro_static_variance_threshold: 0.01,
            accel_magnitude_tolerance: 0.5,
            min_bootstrap_tracks: 20,
            fallback_gyro_bias_sigma: 0.1,
            fallback_accel_bias_sigma: 0.5,
        }
    }
}

#[derive(Debug, Clone)]
pub struct InitializationResult {
    pub used_static_segment: bool,
    pub gyro_bias: Vector3,
    pub accel_bias: Vector3,
    pub velocity_world: Vector3,
    pub orientation_world_from_imu: UnitQuaternion,
    pub gravity_world: Vector3,
    pub matched_feature_count: usize,
}

#[must_use]
fn mean_vector(samples: &[Vector3]) -> Option<Vector3> {
    if samples.is_empty() {
        return None;
    }
    let sum = samples.iter().copied().fold(Vector3::zeros(), |acc, sample| acc + sample);
    Some(sum / samples.len() as Float)
}

#[must_use]
fn sample_window<'a>(imu_samples: &'a [ImuData], duration_s: Float) -> &'a [ImuData] {
    if imu_samples.is_empty() {
        return &[];
    }
    let start_time = imu_samples[0].timestamp;
    let end_time = start_time + (duration_s * 1e9) as i64;
    let count = imu_samples
        .iter()
        .take_while(|sample| sample.timestamp <= end_time)
        .count();
    &imu_samples[..count.max(imu_samples.len().min(1))]
}

#[must_use]
fn vector_from_array(values: [f64; 3]) -> Vector3 {
    Vector3::new(values[0] as Float, values[1] as Float, values[2] as Float)
}

#[must_use]
fn variance(samples: &[Vector3], mean: &Vector3) -> Float {
    if samples.is_empty() {
        return 0.0;
    }

    samples
        .iter()
        .map(|sample| (*sample - mean).norm_squared())
        .sum::<Float>()
        / samples.len() as Float
}

#[must_use]
pub fn estimate_gyro_bias_from_static_segment(
    imu_samples: &[ImuData],
    config: &InitializationConfig,
) -> Option<(Vector3, bool)> {
    let startup_window = sample_window(imu_samples, config.static_window_duration_s);
    if startup_window.is_empty() {
        return None;
    }

    let gyro_samples = startup_window
        .iter()
        .map(|sample| vector_from_array(sample.gyro))
        .collect::<Vec<_>>();
    let mean = mean_vector(&gyro_samples)?;
    let is_static = variance(&gyro_samples, &mean) < config.gyro_static_variance_threshold;
    Some((if is_static { mean } else { Vector3::zeros() }, is_static))
}

#[must_use]
fn rotation_between_vectors(from: &Vector3, to: &Vector3) -> Option<UnitQuaternion> {
    let from_norm = from.try_normalize(1e-9)?;
    let to_norm = to.try_normalize(1e-9)?;
    let cross = from_norm.cross(&to_norm);
    let dot = from_norm.dot(&to_norm).clamp(-1.0, 1.0);

    if dot > 1.0 - 1e-9 {
        return Some(UnitQuaternion::identity());
    }

    if dot < -1.0 + 1e-9 {
        let fallback_axis = if from_norm.x.abs() < 0.9 {
            from_norm.cross(&Vector3::new(1.0, 0.0, 0.0))
        } else {
            from_norm.cross(&Vector3::new(0.0, 1.0, 0.0))
        };
        return fallback_axis
            .try_normalize(1e-9)
            .map(|axis| UnitQuaternion::from_scaled_axis(axis * std::f64::consts::PI as Float));
    }

    let angle = dot.acos();
    cross
        .try_normalize(1e-9)
        .map(|axis| UnitQuaternion::from_scaled_axis(axis * angle))
}

#[must_use]
pub fn estimate_gravity_alignment(
    imu_samples: &[ImuData],
    config: &InitializationConfig,
) -> Option<(UnitQuaternion, Vector3)> {
    let startup_window = sample_window(imu_samples, config.static_window_duration_s);
    if startup_window.is_empty() {
        return None;
    }

    let accel_samples = startup_window
        .iter()
        .map(|sample| vector_from_array(sample.accel))
        .collect::<Vec<_>>();
    let mean_accel = mean_vector(&accel_samples)?;
    let gravity_magnitude = mean_accel.norm();
    if (gravity_magnitude - GRAVITY_MAGNITUDE as Float).abs() > config.accel_magnitude_tolerance {
        return None;
    }

    let orientation = rotation_between_vectors(&mean_accel, &Vector3::new(0.0, 0.0, GRAVITY_MAGNITUDE as Float))?;
    Some((orientation, Vector3::new(0.0, 0.0, -(GRAVITY_MAGNITUDE as Float))))
}

#[must_use]
fn integrate_gyro_orientation(
    imu_samples: &[ImuData],
    gyro_bias: &Vector3,
) -> UnitQuaternion {
    let mut orientation_delta = UnitQuaternion::identity();
    for pair in imu_samples.windows(2) {
        let curr = &pair[0];
        let next = &pair[1];
        let dt = (next.timestamp - curr.timestamp) as Float * 1e-9;
        if dt <= 0.0 {
            continue;
        }
        let omega = vector_from_array(curr.gyro) - *gyro_bias;
        orientation_delta *= UnitQuaternion::from_scaled_axis(omega * dt);
    }
    orientation_delta
}

#[must_use]
fn pair_frame_observations(frame: &Frame) -> FxHashMap<usize, (Vector2, Vector2)> {
    let right_by_id = frame
        .right_features()
        .iter()
        .map(|feature| (feature.feature_id, feature))
        .collect::<FxHashMap<_, _>>();

    frame
        .left_features()
        .iter()
        .filter_map(|left_feature| {
            let right_feature = right_by_id.get(&left_feature.feature_id)?;
            Some((
                left_feature.feature_id,
                (
                    feature_normalized(left_feature),
                    feature_normalized(right_feature),
                ),
            ))
        })
        .collect()
}

#[must_use]
fn feature_normalized(feature: &crate::feature_tracker::Feature) -> Vector2 {
    if feature.undistorted_coord[0] > -0.5 && feature.undistorted_coord[1] > -0.5 {
        return Vector2::new(
            feature.undistorted_coord[0] as Float,
            feature.undistorted_coord[1] as Float,
        );
    }

    Vector2::new(feature.pixel_coord[0] as Float, feature.pixel_coord[1] as Float)
}

#[must_use]
fn stereo_depth(normalized_left: &Vector2, normalized_right: &Vector2, rig: &StereoRig) -> Option<Vector3> {
    let r0 = rig.right_from_left_rotation.row(0);
    let r1 = rig.right_from_left_rotation.row(1);
    let r2 = rig.right_from_left_rotation.row(2);
    let u_l = normalized_left.x;
    let v_l = normalized_left.y;
    let u_r = normalized_right.x;
    let v_r = normalized_right.y;
    let d = r2[0] * u_l + r2[1] * v_l + r2[2];
    let a0 = u_r * d - (r0[0] * u_l + r0[1] * v_l + r0[2]);
    let a1 = v_r * d - (r1[0] * u_l + r1[1] * v_l + r1[2]);
    let b0 = rig.right_from_left_translation.x - u_r * rig.right_from_left_translation.z;
    let b1 = rig.right_from_left_translation.y - v_r * rig.right_from_left_translation.z;
    let denom = a0 * a0 + a1 * a1;
    if denom.abs() < 1e-10 {
        return None;
    }
    let depth = (a0 * b0 + a1 * b1) / denom;
    (depth > 0.1).then(|| Vector3::new(u_l * depth, v_l * depth, depth))
}

#[must_use]
fn imu_velocity_constraint(
    imu_samples: &[ImuData],
    orientation_world_from_imu: &UnitQuaternion,
    accel_bias: &Vector3,
    gravity_world: &Vector3,
) -> Vector3 {
    let mut velocity = Vector3::zeros();
    for pair in imu_samples.windows(2) {
        let curr = &pair[0];
        let next = &pair[1];
        let dt = (next.timestamp - curr.timestamp) as Float * 1e-9;
        if dt <= 0.0 {
            continue;
        }
        let accel_world = orientation_world_from_imu.transform_vector(&(vector_from_array(curr.accel) - *accel_bias)) + gravity_world;
        velocity += accel_world * dt;
    }
    velocity
}

#[must_use]
pub fn estimate_visual_bootstrap_velocity(
    first_frame: &Frame,
    second_frame: &Frame,
    imu_samples_between: &[ImuData],
    rig: &StereoRig,
    gyro_bias: &Vector3,
    accel_bias: &Vector3,
    orientation_world_from_imu: &UnitQuaternion,
    gravity_world: &Vector3,
    config: &InitializationConfig,
) -> Option<(Vector3, usize)> {
    let first_pairs = pair_frame_observations(first_frame);
    let second_pairs = pair_frame_observations(second_frame);
    let delta_t = (second_frame.timestamp_ns - first_frame.timestamp_ns) as Float * 1e-9;
    if delta_t <= 0.0 {
        return None;
    }

    let rotation_delta = integrate_gyro_orientation(imu_samples_between, gyro_bias);
    let rotation_21 = rotation_delta.to_rotation_matrix().into_inner();
    let imu_velocity = imu_velocity_constraint(
        imu_samples_between,
        orientation_world_from_imu,
        accel_bias,
        gravity_world,
    );

    let mut displacements = Vec::new();
    for (feature_id, (first_left, first_right)) in &first_pairs {
        let Some((second_left, second_right)) = second_pairs.get(feature_id) else {
            continue;
        };
        let Some(point_first) = stereo_depth(first_left, first_right, rig) else {
            continue;
        };
        let Some(point_second) = stereo_depth(second_left, second_right, rig) else {
            continue;
        };
        displacements.push(point_first - rotation_21.transpose() * point_second);
    }

    if displacements.len() < config.min_bootstrap_tracks {
        return None;
    }

    let visual_velocity = mean_vector(&displacements)? / delta_t;
    let constrained_velocity = if imu_velocity.norm() > 1e-4 {
        visual_velocity * 0.8 + imu_velocity * 0.2
    } else {
        visual_velocity
    };

    Some((constrained_velocity, displacements.len()))
}

pub fn apply_initialization_result(
    filter_state: &mut FilterState,
    result: &InitializationResult,
    config: &InitializationConfig,
) {
    filter_state.imu_state.orientation = result.orientation_world_from_imu;
    filter_state.imu_state.velocity = result.velocity_world;
    filter_state.imu_state.position = Vector3::zeros();
    filter_state.imu_state.gyro_bias = result.gyro_bias;
    filter_state.imu_state.accel_bias = result.accel_bias;
    filter_state.gravity_world = result.gravity_world;
    filter_state.mode = VisualMode::Normal;

    if !result.used_static_segment {
        filter_state.covariance[(9, 9)] = config.fallback_gyro_bias_sigma * config.fallback_gyro_bias_sigma;
        filter_state.covariance[(10, 10)] = config.fallback_gyro_bias_sigma * config.fallback_gyro_bias_sigma;
        filter_state.covariance[(11, 11)] = config.fallback_gyro_bias_sigma * config.fallback_gyro_bias_sigma;
        filter_state.covariance[(12, 12)] = config.fallback_accel_bias_sigma * config.fallback_accel_bias_sigma;
        filter_state.covariance[(13, 13)] = config.fallback_accel_bias_sigma * config.fallback_accel_bias_sigma;
        filter_state.covariance[(14, 14)] = config.fallback_accel_bias_sigma * config.fallback_accel_bias_sigma;
    }
}

#[must_use]
pub fn initialize_filter_state(
    filter_state: &mut FilterState,
    imu_samples: &[ImuData],
    first_frame: &Frame,
    second_frame: &Frame,
    rig: &StereoRig,
    config: &InitializationConfig,
) -> Option<InitializationResult> {
    let (gyro_bias, used_static_segment) = estimate_gyro_bias_from_static_segment(imu_samples, config)?;
    let (orientation_world_from_imu, gravity_world) = estimate_gravity_alignment(imu_samples, config)?;
    let accel_bias = Vector3::zeros();
    let (velocity_world, matched_feature_count) = estimate_visual_bootstrap_velocity(
        first_frame,
        second_frame,
        imu_samples,
        rig,
        &gyro_bias,
        &accel_bias,
        &orientation_world_from_imu,
        &gravity_world,
        config,
    )?;

    let result = InitializationResult {
        used_static_segment,
        gyro_bias,
        accel_bias,
        velocity_world,
        orientation_world_from_imu,
        gravity_world,
        matched_feature_count,
    };
    apply_initialization_result(filter_state, &result, config);
    Some(result)
}

#[cfg(test)]
mod tests {
    use super::{
        estimate_gravity_alignment, estimate_gyro_bias_from_static_segment,
        estimate_visual_bootstrap_velocity, initialize_filter_state, InitializationConfig,
    };
    use crate::{
        datasets::ImuData,
        estimator::Frame,
        feature_tracker::Feature,
        msckf::{FilterState, StereoRig},
        types::{Matrix3x3, Vector3},
    };

    fn imu_sample(timestamp_ns: i64, gyro: [f64; 3], accel: [f64; 3]) -> ImuData {
        ImuData { timestamp: timestamp_ns, gyro, accel }
    }

    fn make_feature(feature_id: usize, normalized: [f32; 2], age: u32) -> Feature {
        let mut feature = Feature::new(feature_id, normalized);
        feature.undistorted_coord = normalized;
        feature.age = age;
        feature
    }

    fn build_stereo_frame(
        timestamp_ns: i64,
        frame_id: i32,
        left_points: &[[f32; 2]],
        right_points: &[[f32; 2]],
    ) -> Frame {
        let mut frame = Frame::new(timestamp_ns, frame_id);
        for (feature_id, (left, right)) in left_points.iter().zip(right_points.iter()).enumerate() {
            frame.add_left_feature(make_feature(feature_id, *left, 3));
            frame.add_right_feature(make_feature(feature_id, *right, 3));
        }
        frame
    }

    #[test]
    fn static_segment_estimates_gyro_bias_and_gravity_alignment() {
        let config = InitializationConfig::default();
        let imu_samples = (0..100)
            .map(|index| {
                imu_sample(
                    index * 5_000_000,
                    [0.01, -0.02, 0.03],
                    [0.0, 0.0, 9.81007],
                )
            })
            .collect::<Vec<_>>();

        let (gyro_bias, used_static) = estimate_gyro_bias_from_static_segment(&imu_samples, &config).unwrap();
        let (orientation, gravity_world) = estimate_gravity_alignment(&imu_samples, &config).unwrap();

        assert!(used_static);
        assert!((gyro_bias - Vector3::new(0.01, -0.02, 0.03)).norm() < 1e-9);
        let aligned = orientation.transform_vector(&Vector3::new(0.0, 0.0, 9.81007));
        assert!((aligned - Vector3::new(0.0, 0.0, 9.81007)).norm() < 1e-6);
        assert_eq!(gravity_world, Vector3::new(0.0, 0.0, -9.81007));
    }

    #[test]
    fn non_static_segment_falls_back_to_zero_gyro_bias() {
        let config = InitializationConfig::default();
        let imu_samples = (0..100)
            .map(|index| {
                let gyro = if index % 2 == 0 { [1.0, 0.0, 0.0] } else { [-1.0, 0.0, 0.0] };
                imu_sample(index * 5_000_000, gyro, [0.0, 0.0, 9.81007])
            })
            .collect::<Vec<_>>();

        let (gyro_bias, used_static) = estimate_gyro_bias_from_static_segment(&imu_samples, &config).unwrap();
        assert!(!used_static);
        assert_eq!(gyro_bias, Vector3::zeros());
    }

    #[test]
    fn stereo_bootstrap_estimates_velocity_from_two_frames() {
        let config = InitializationConfig::default();
        let rig = StereoRig::new(Matrix3x3::identity(), Vector3::new(-0.2, 0.0, 0.0));
        let imu_samples = (0..20)
            .map(|index| imu_sample(index * 5_000_000, [0.0, 0.0, 0.0], [0.0, 0.0, 9.81007]))
            .collect::<Vec<_>>();

        let mut left_1 = Vec::new();
        let mut right_1 = Vec::new();
        let mut left_2 = Vec::new();
        let mut right_2 = Vec::new();
        for idx in 0..20 {
            let x = -0.2 + idx as f32 * 0.02;
            let y = -0.1 + idx as f32 * 0.01;
            let z = 4.0 + idx as f32 * 0.02;
            left_1.push([x / z, y / z]);
            right_1.push([(x - 0.2) / z, y / z]);
            let x2 = x - 0.1;
            left_2.push([x2 / z, y / z]);
            right_2.push([(x2 - 0.2) / z, y / z]);
        }
        let first_frame = build_stereo_frame(0, 0, &left_1, &right_1);
        let second_frame = build_stereo_frame(100_000_000, 1, &left_2, &right_2);

        let (velocity, matched_features) = estimate_visual_bootstrap_velocity(
            &first_frame,
            &second_frame,
            &imu_samples,
            &rig,
            &Vector3::zeros(),
            &Vector3::zeros(),
            &crate::types::UnitQuaternion::identity(),
            &Vector3::new(0.0, 0.0, -9.81007),
            &config,
        )
        .unwrap();

        assert!(matched_features >= 20);
        assert!((velocity.x - 1.0).abs() < 1e-2);
    }

    #[test]
    fn initialize_filter_state_sets_mode_normal_and_inflates_fallback_covariance() {
        let config = InitializationConfig::default();
        let rig = StereoRig::new(Matrix3x3::identity(), Vector3::new(-0.2, 0.0, 0.0));
        let imu_samples = (0..100)
            .map(|index| {
                let gyro = if index % 2 == 0 { [1.0, 0.0, 0.0] } else { [-1.0, 0.0, 0.0] };
                imu_sample(index * 5_000_000, gyro, [0.0, 0.0, 9.81007])
            })
            .collect::<Vec<_>>();

        let mut left_1 = Vec::new();
        let mut right_1 = Vec::new();
        let mut left_2 = Vec::new();
        let mut right_2 = Vec::new();
        for idx in 0..20 {
            let x = -0.2 + idx as f32 * 0.02;
            let y = -0.1 + idx as f32 * 0.01;
            let z = 4.0 + idx as f32 * 0.02;
            left_1.push([x / z, y / z]);
            right_1.push([(x - 0.2) / z, y / z]);
            let x2 = x - 0.1;
            left_2.push([x2 / z, y / z]);
            right_2.push([(x2 - 0.2) / z, y / z]);
        }
        let first_frame = build_stereo_frame(0, 0, &left_1, &right_1);
        let second_frame = build_stereo_frame(100_000_000, 1, &left_2, &right_2);
        let mut filter_state = FilterState::new();

        let result = initialize_filter_state(
            &mut filter_state,
            &imu_samples,
            &first_frame,
            &second_frame,
            &rig,
            &config,
        )
        .unwrap();

        assert!(!result.used_static_segment);
        assert_eq!(filter_state.mode, crate::msckf::VisualMode::Normal);
        assert!((filter_state.covariance[(9, 9)] - config.fallback_gyro_bias_sigma.powi(2)).abs() < 1e-9);
        assert!((filter_state.covariance[(12, 12)] - config.fallback_accel_bias_sigma.powi(2)).abs() < 1e-9);
        assert!(result.matched_feature_count >= 20);
    }
}