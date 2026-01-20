//! Common viewer logging helpers to reduce code bloat.
//!
//! This module provides reusable patterns for Rerun logging operations,
//! reducing duplication and binary size from viewer code.

use rerun::{RecordingStream, components::Color};
use crate::types::Float;

/// Helper for logging 3D points with optional colors.
#[inline]
pub fn log_points_3d(
    rec: &RecordingStream,
    entity_path: &str,
    points: &[[Float; 3]],
    colors: Option<&[Color]>,
) {
    let rerun_points: Vec<_> = points
        .iter()
        .map(|p| [p[0] as f32, p[1] as f32, p[2] as f32])
        .collect();
    
    let mut builder = rerun::Points3D::new(rerun_points);
    
    if let Some(colors) = colors {
        builder = builder.with_colors(colors.to_vec());
    }
    
    rec.log(entity_path, &builder).ok();
}

/// Helper for logging 2D points with optional colors and radii.
pub fn log_points_2d(
    rec: &RecordingStream,
    entity_path: &str,
    points: &[[Float; 2]],
    colors: Option<&[Color]>,
    radii: Option<&[Float]>,
) {
    let rerun_points: Vec<_> = points
        .iter()
        .map(|p| [p[0] as f32, p[1] as f32])
        .collect();
    
    let mut builder = rerun::Points2D::new(rerun_points);
    
    if let Some(colors) = colors {
        builder = builder.with_colors(colors.to_vec());
    }
    
    if let Some(radii) = radii {
        let radii_f32: Vec<f32> = radii.iter().map(|&r| r as f32).collect();
        builder = builder.with_radii(radii_f32);
    }
    
    rec.log(entity_path, &builder).ok();
}

/// Helper for logging line strips.
pub fn log_line_strip_3d(
    rec: &RecordingStream,
    entity_path: &str,
    points: &[[Float; 3]],
    color: Option<Color>,
) {
    let rerun_points: Vec<_> = points
        .iter()
        .map(|p| [p[0] as f32, p[1] as f32, p[2] as f32])
        .collect();
    
    let mut builder = rerun::LineStrips3D::new([rerun_points]);
    
    if let Some(color) = color {
        builder = builder.with_colors([color]);
    }
    
    rec.log(entity_path, &builder).ok();
}

/// Helper for logging text annotations.
pub fn log_text(
    rec: &RecordingStream,
    entity_path: &str,
    text: String,
) {
    rec.log(
        entity_path,
        &rerun::TextDocument::new(text),
    )
    .ok();
}

/// Helper for logging transforms.
pub fn log_transform_3d(
    rec: &RecordingStream,
    entity_path: &str,
    transform: &crate::types::Matrix4x4,
) {
    // Extract rotation and translation
    let rotation = transform.fixed_view::<3, 3>(0, 0);
    let translation = transform.fixed_view::<3, 1>(0, 3);
    
    let trans: [f32; 3] = [
        translation[0] as f32,
        translation[1] as f32,
        translation[2] as f32,
    ];
    
    // Convert rotation matrix to quaternion for Rerun
    let rot_na = nalgebra::Rotation3::from_matrix_unchecked(nalgebra::Matrix3::new(
        rotation[(0, 0)], rotation[(0, 1)], rotation[(0, 2)],
        rotation[(1, 0)], rotation[(1, 1)], rotation[(1, 2)],
        rotation[(2, 0)], rotation[(2, 1)], rotation[(2, 2)],
    ));
    let quat = nalgebra::UnitQuaternion::from_rotation_matrix(&rot_na);
    let quat_array = [quat.w as f32, quat.i as f32, quat.j as f32, quat.k as f32];
    
    rec.log(
        entity_path,
        &rerun::Transform3D::from_translation_rotation(
            trans,
            rerun::Quaternion::from_xyzw(quat_array),
        ),
    )
    .ok();
}
