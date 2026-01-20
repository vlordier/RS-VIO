//! Aggressive Keyframe Culling
//!
//! Provides intelligent keyframe removal strategies.

use crate::estimator::Frame;
use crate::types::Float;
use std::collections::HashMap;

/// Configuration for aggressive keyframe culling
#[derive(Debug, Clone)]
pub struct AggressiveCullingConfig {
    pub min_parallax_rad: Float,
    pub min_observations: u32,
    pub max_similar_poses: usize,
    pub pose_similarity_translation: Float,
    pub enable_parallax_culling: bool,
    pub enable_observation_culling: bool,
    pub enable_redundancy_culling: bool,
    pub enable_age_culling: bool,
    pub max_age: u32,
    pub reserve_fraction: Float,
}

impl Default for AggressiveCullingConfig {
    fn default() -> Self {
        Self {
            min_parallax_rad: 0.05,
            min_observations: 20,
            max_similar_poses: 3,
            pose_similarity_translation: 0.1,
            enable_parallax_culling: true,
            enable_observation_culling: true,
            enable_redundancy_culling: true,
            enable_age_culling: true,
            max_age: 10,
            reserve_fraction: 0.7,
        }
    }
}

/// Result of keyframe culling analysis
#[derive(Debug, Clone)]
pub struct CullingAnalysis {
    pub to_remove: Vec<usize>,
    pub reasons: Vec<CullingReason>,
    pub information_loss: Float,
    pub frames_removed: usize,
}

#[derive(Debug, Clone, PartialEq)]
pub enum CullingReason {
    LowParallax,
    LowObservations,
    Redundant,
    Old,
    MakeRoom,
}

#[derive(Debug, Clone)]
pub struct AggressiveKeyframeCuller {
    config: AggressiveCullingConfig,
}

impl AggressiveKeyframeCuller {
    #[inline]
    pub fn new(config: AggressiveCullingConfig) -> Self {
        Self { config }
    }

    #[inline]
    pub fn select_keyframes_to_remove(
        &self,
        keyframes: &[&Frame],
        _map_points: &HashMap<usize, [f32; 3]>,
        current_window_size: usize,
        max_window_size: usize,
    ) -> CullingAnalysis {
        if keyframes.is_empty() {
            return CullingAnalysis {
                to_remove: Vec::new(),
                reasons: Vec::new(),
                information_loss: 0.0,
                frames_removed: 0,
            };
        }

        let reserve_size = (max_window_size as Float * self.config.reserve_fraction) as usize;
        let target_size = reserve_size.max(2);
        let needed_removals = current_window_size.saturating_sub(target_size);
        if needed_removals == 0 {
            return CullingAnalysis {
                to_remove: Vec::new(),
                reasons: Vec::new(),
                information_loss: 0.0,
                frames_removed: 0,
            };
        }

        // Fast path: collect culling candidates
        let mut candidates: Vec<(usize, Float, CullingReason)> =
            Vec::with_capacity(needed_removals);
        let min_obs = if self.config.enable_observation_culling {
            Some(self.config.min_observations)
        } else {
            None
        };
        let max_age = if self.config.enable_age_culling {
            Some(self.config.max_age)
        } else {
            None
        };

        let last_idx = keyframes.len().saturating_sub(1);
        for (idx, &frame) in keyframes.iter().enumerate() {
            if candidates.len() >= needed_removals {
                break;
            }
            // Skip first and last frame (oldest and newest)
            if idx == 0 || idx == last_idx {
                continue;
            }

            // Check observation count
            if let Some(min_o) = min_obs {
                let obs_count = frame.left_features.len() + frame.right_features.len();
                if obs_count < min_o as usize {
                    candidates.push((idx, 3.0, CullingReason::LowObservations));
                    continue;
                }
            }

            // Check age
            if let Some(max_a) = max_age {
                let age = keyframes.len() - idx - 1;
                if age >= max_a as usize {
                    candidates.push((idx, 2.0, CullingReason::Old));
                    continue;
                }
            }
        }

        if candidates.is_empty() {
            // Fallback: remove oldest frame
            return CullingAnalysis {
                to_remove: vec![0],
                reasons: vec![CullingReason::Old],
                information_loss: 1.0 / keyframes.len() as Float,
                frames_removed: 1,
            };
        }

        // Sort by score descending (higher score = better candidate for removal)
        candidates.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap());

        let to_remove: Vec<usize> = candidates.iter().map(|(i, _, _)| *i).collect();
        let reasons: Vec<CullingReason> = candidates.iter().map(|(_, _, r)| r.clone()).collect();
        let frames_removed = to_remove.len();

        CullingAnalysis {
            to_remove,
            reasons,
            information_loss: frames_removed as Float / keyframes.len() as Float,
            frames_removed,
        }
    }
}

pub struct FifoCuller;

impl FifoCuller {
    pub fn select_keyframes_to_remove(
        &self,
        keyframes: &[Frame],
        _map_points: &HashMap<usize, [f32; 3]>,
        _current_window_size: usize,
        max_window_size: usize,
    ) -> CullingAnalysis {
        let to_remove: Vec<usize> = if keyframes.len() > max_window_size {
            (0..keyframes.len() - max_window_size).collect()
        } else {
            Vec::new()
        };

        CullingAnalysis {
            to_remove: to_remove.clone(),
            reasons: vec![CullingReason::Old; to_remove.len()],
            information_loss: 0.0,
            frames_removed: to_remove.len(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_config_defaults() {
        let config = AggressiveCullingConfig::default();
        assert_eq!(config.min_parallax_rad, 0.05);
        assert_eq!(config.min_observations, 20);
    }

    #[test]
    fn test_empty_keyframes() {
        let culler = AggressiveKeyframeCuller::new(AggressiveCullingConfig::default());
        let map_points = HashMap::new();

        let result = culler.select_keyframes_to_remove(&[], &map_points, 0, 10);
        assert!(result.to_remove.is_empty());
        assert_eq!(result.information_loss, 0.0);
    }
}
