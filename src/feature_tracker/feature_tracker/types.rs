/// Feature and quality tracking data structures

#[derive(Debug, Clone, Copy)]
pub struct Feature {
    /// Unique identifier of this feature (within the current frame or globally).
    pub feature_id: usize,

    /// Pixel coordinate in the left image (u, v).
    pub pixel_coord: [f32; 2],

    /// Undistorted pixel coordinate (u, v). `[-1, -1]` means invalid.
    pub undistorted_coord: [f32; 2],

    /// Stereo disparity (pixels). `None` if not estimated.
    pub disparity: Option<f32>,

    /// Disparity uncertainty (pixels). `None` if not estimated.
    pub disparity_uncertainty: Option<f32>,

    /// Final photometric error from refinement.
    pub photometric_error: Option<f32>,

    /// Peak sharpness of the correlation surface.
    pub peak_sharpness: Option<f32>,

    /// Tracking quality metrics
    pub quality: FeatureQuality,
}

#[derive(Debug, Clone, Copy)]
pub struct FeatureQuality {
    /// Tracking confidence score (0.0 to 1.0, higher is better)
    pub confidence: f32,

    /// Number of consecutive frames this feature has been tracked
    pub age: u32,

    /// Average residual error from optical flow
    pub residual_error: f32,

    /// Motion consistency score (0.0 to 1.0, higher is better)
    pub motion_consistency: f32,

    /// Geometric validation score from RANSAC (0.0 to 1.0, higher is better)
    pub geometric_consistency: f32,

    /// Whether this feature is considered reliable for triangulation
    pub is_reliable: bool,
}

impl Default for FeatureQuality {
    fn default() -> Self {
        Self {
            confidence: 1.0,
            age: 1,
            residual_error: 0.0,
            motion_consistency: 1.0,
            geometric_consistency: 1.0,
            is_reliable: true,
        }
    }
}

impl Feature {
    pub fn new(feature_id: usize, pixel_coord: [f32; 2]) -> Self {
        Self {
            feature_id,
            pixel_coord,
            undistorted_coord: [-1.0, -1.0],
            disparity: None,
            disparity_uncertainty: None,
            photometric_error: None,
            peak_sharpness: None,
            quality: FeatureQuality::default(),
        }
    }

    /// Create a new feature with specified quality metrics
    pub fn new_with_quality(
        feature_id: usize,
        pixel_coord: [f32; 2],
        quality: FeatureQuality,
    ) -> Self {
        Self {
            feature_id,
            pixel_coord,
            undistorted_coord: [-1.0, -1.0],
            disparity: None,
            disparity_uncertainty: None,
            photometric_error: None,
            peak_sharpness: None,
            quality,
        }
    }
}
