use super::state::Estimator;
use crate::estimator::Frame;
use crate::fl;
use crate::optimization::loop_closure::KeyframeDescriptor;
use crate::types::{Float, Matrix4x4};

impl Estimator {
    /// Create a keyframe descriptor for loop-closure detection
    pub fn create_keyframe_descriptor(
        &self,
        keyframe_id: u64,
        timestamp: i64,
        frame: &Frame,
        pose: Matrix4x4,
    ) -> KeyframeDescriptor {
        // Convert Matrix4x4 to Isometry3 (used in both branches)
        let t = pose.fixed_view::<3, 1>(0, 3);
        let translation = nalgebra::Translation3::from(t.clone_owned());
        let rotation =
            nalgebra::Rotation3::from_matrix_unchecked(pose.fixed_view::<3, 3>(0, 0).into_owned());
        let pose_isometry = nalgebra::Isometry3::from_parts(
            translation,
            nalgebra::UnitQuaternion::from_rotation_matrix(&rotation),
        );

        // Try ORB descriptor if extractor is available
        if self.orb_extractor.is_some() {
            // Use ORB features from left image
            // Note: In a real implementation, you would pass the actual grayscale image data
            // For now, we'll extract features from the first frame's features
            let num_features = frame.left_features.len().max(frame.right_features.len());

            // Create a simple descriptor from feature statistics combined with presence flag
            let mut descriptor = vec![fl!(0.0); 10];
            descriptor[0] = if self.orb_extractor.is_some() {
                fl!(1.0)
            } else {
                fl!(0.0)
            }; // ORB enabled flag
            descriptor[1] = num_features as Float / fl!(200.0); // Normalized feature count

            // Add left image feature statistics
            if !frame.left_features.is_empty() {
                let avg_x: f32 = frame
                    .left_features
                    .iter()
                    .map(|f| f.pixel_coord[0])
                    .sum::<f32>()
                    / frame.left_features.len() as f32;
                let avg_y: f32 = frame
                    .left_features
                    .iter()
                    .map(|f| f.pixel_coord[1])
                    .sum::<f32>()
                    / frame.left_features.len() as f32;
                descriptor[2] = avg_x as Float / self.config.camera.image_width as Float;
                descriptor[3] = avg_y as Float / self.config.camera.image_height as Float;
                descriptor[4] = frame.left_features.len() as Float / fl!(200.0);
            }

            // Add right image feature statistics
            if !frame.right_features.is_empty() {
                let avg_x: f32 = frame
                    .right_features
                    .iter()
                    .map(|f| f.pixel_coord[0])
                    .sum::<f32>()
                    / frame.right_features.len() as f32;
                let avg_y: f32 = frame
                    .right_features
                    .iter()
                    .map(|f| f.pixel_coord[1])
                    .sum::<f32>()
                    / frame.right_features.len() as f32;
                descriptor[5] = avg_x as Float / self.config.camera.image_width as Float;
                descriptor[6] = avg_y as Float / self.config.camera.image_height as Float;
                descriptor[7] = frame.right_features.len() as Float / fl!(200.0);
            }

            // Pose-derived features
            descriptor[8] = (t[0] / fl!(10.0)).tanh();
            descriptor[9] = (t[1] / fl!(10.0)).tanh();

            KeyframeDescriptor {
                keyframe_id,
                timestamp,
                descriptor,
                num_features,
                pose: pose_isometry,
            }
        } else {
            // Fallback to simple descriptor
            let num_features = frame.left_features.len().max(frame.right_features.len());

            // Create a simple descriptor from feature statistics (10-dim vector)
            let mut descriptor = vec![fl!(0.0); 10];

            if !frame.left_features.is_empty() {
                let avg_x: f32 = frame
                    .left_features
                    .iter()
                    .map(|f| f.pixel_coord[0])
                    .sum::<f32>()
                    / frame.left_features.len() as f32;
                let avg_y: f32 = frame
                    .left_features
                    .iter()
                    .map(|f| f.pixel_coord[1])
                    .sum::<f32>()
                    / frame.left_features.len() as f32;
                descriptor[0] = avg_x as Float / self.config.camera.image_width as Float;
                descriptor[1] = avg_y as Float / self.config.camera.image_height as Float;

                // Add feature distribution stats
                descriptor[2] = frame.left_features.len() as Float / fl!(200.0);
                // Normalized feature count
            }

            if !frame.right_features.is_empty() {
                let avg_x: f32 = frame
                    .right_features
                    .iter()
                    .map(|f| f.pixel_coord[0])
                    .sum::<f32>()
                    / frame.right_features.len() as f32;
                let avg_y: f32 = frame
                    .right_features
                    .iter()
                    .map(|f| f.pixel_coord[1])
                    .sum::<f32>()
                    / frame.right_features.len() as f32;
                descriptor[3] = avg_x as Float / self.config.camera.image_width as Float;
                descriptor[4] = avg_y as Float / self.config.camera.image_height as Float;

                descriptor[5] = frame.right_features.len() as Float / fl!(200.0);
            }

            // Fill remaining dimensions with pose-derived features
            descriptor[6] = (t[0] / fl!(10.0)).tanh(); // Position features (bounded)
            descriptor[7] = (t[1] / fl!(10.0)).tanh();
            descriptor[8] = (t[2] / fl!(10.0)).tanh();
            descriptor[9] = num_features as Float / fl!(200.0);

            KeyframeDescriptor {
                keyframe_id,
                timestamp,
                descriptor,
                num_features,
                pose: pose_isometry,
            }
        }
    }
}
