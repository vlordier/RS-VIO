//! Telemetry and health monitoring for VIO system.
//!
//! This module provides types for tracking estimator health, performance metrics,
//! and diagnostic information essential for fleet monitoring and debugging.

use crate::types::Float;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// Overall health state of the VIO estimator.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum HealthState {
    /// All systems nominal
    Healthy,
    /// Operating but with reduced capability
    Degraded,
    /// Not operational
    Unhealthy,
}

impl HealthState {
    /// Check if the system can continue operation.
    pub fn is_operational(&self) -> bool {
        matches!(self, Self::Healthy | Self::Degraded)
    }

    /// Check if the system is in a nominal state.
    pub fn is_nominal(&self) -> bool {
        matches!(self, Self::Healthy)
    }
}

impl std::fmt::Display for HealthState {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Healthy => write!(f, "Healthy"),
            Self::Degraded => write!(f, "Degraded"),
            Self::Unhealthy => write!(f, "Unhealthy"),
        }
    }
}

/// Health status of a specific component.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ComponentHealth {
    /// Whether the component is operational
    pub operational: bool,
    /// Error rate in the last N frames [0, 1]
    pub error_rate: Float,
    /// 95th percentile latency in milliseconds
    pub latency_p95_ms: Float,
    /// Last error message, if any
    pub last_error: Option<String>,
    /// Timestamp of last update (seconds since epoch)
    pub last_update_secs: f64,
}

impl ComponentHealth {
    /// Create a new healthy component status.
    pub fn new(timestamp_secs: f64) -> Self {
        Self {
            operational: true,
            error_rate: 0.0,
            latency_p95_ms: 0.0,
            last_error: None,
            last_update_secs: timestamp_secs,
        }
    }

    /// Mark component as failed with error message.
    pub fn mark_failure(mut self, error: String, timestamp_secs: f64) -> Self {
        self.operational = false;
        self.last_error = Some(error);
        self.last_update_secs = timestamp_secs;
        self
    }

    /// Update error rate (smoothed over window).
    pub fn with_error_rate(mut self, rate: Float) -> Self {
        self.error_rate = rate.clamp(0.0, 1.0);
        self
    }

    /// Update latency metric.
    pub fn with_latency(mut self, latency_ms: Float) -> Self {
        self.latency_p95_ms = latency_ms;
        self
    }
}

/// Complete health status for the VIO estimator.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct HealthStatus {
    /// Overall system state
    pub state: HealthState,

    /// Estimator pipeline health
    pub estimator: ComponentHealth,
    /// Feature detection health
    pub feature_detector: ComponentHealth,
    /// Solver health
    pub solver: ComponentHealth,
    /// IMU integration health
    pub imu_integration: ComponentHealth,

    /// Estimated reliability [0, 1]
    pub reliability: Float,
    /// Diagnostic information
    pub diagnostics: HashMap<String, String>,
}

impl HealthStatus {
    /// Create a new healthy status.
    pub fn new(timestamp_secs: f64) -> Self {
        Self {
            state: HealthState::Healthy,
            estimator: ComponentHealth::new(timestamp_secs),
            feature_detector: ComponentHealth::new(timestamp_secs),
            solver: ComponentHealth::new(timestamp_secs),
            imu_integration: ComponentHealth::new(timestamp_secs),
            reliability: 1.0,
            diagnostics: HashMap::new(),
        }
    }

    /// Compute overall reliability from component health.
    pub fn compute_reliability(&self) -> Float {
        let component_scores = vec![
            (1.0 - self.estimator.error_rate) * 0.3,
            (1.0 - self.feature_detector.error_rate) * 0.25,
            (1.0 - self.solver.error_rate) * 0.25,
            (1.0 - self.imu_integration.error_rate) * 0.2,
        ];
        component_scores.iter().sum::<f64>().clamp(0.0, 1.0) as Float
    }

    /// Mark the system as degraded.
    pub fn mark_degraded(mut self, reason: String) -> Self {
        self.state = HealthState::Degraded;
        self.diagnostics
            .insert("degradation_reason".to_string(), reason);
        self
    }

    /// Mark the system as unhealthy.
    pub fn mark_unhealthy(mut self, reason: String) -> Self {
        self.state = HealthState::Unhealthy;
        self.diagnostics
            .insert("failure_reason".to_string(), reason);
        self
    }
}

/// Per-frame performance metrics.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct TelemetryFrame {
    /// Frame ID
    pub frame_id: u32,
    /// Number of features detected
    pub num_features: u32,
    /// Number of feature matches
    pub num_matches: u32,
    /// Solver computation time (ms)
    pub solver_time_ms: Float,
    /// Estimated pose drift (m/s)
    pub drift_estimate: Option<Float>,
    /// Sliding window size
    pub window_size: u32,
    /// Number of keyframes in map
    pub num_keyframes: u32,
    /// IMU health flag
    pub imu_healthy: bool,
    /// Number of loop closure candidates
    pub num_loop_closure_candidates: u32,
}

impl TelemetryFrame {
    /// Get average processing time across all operations (ms).
    pub fn avg_processing_time_ms(&self) -> Float {
        self.solver_time_ms
    }

    /// Check if this frame had sufficient features.
    pub fn has_sufficient_features(&self, min_features: u32) -> bool {
        self.num_features >= min_features
    }

    /// Check if tracking quality is good based on match ratio.
    pub fn has_good_tracking(&self) -> bool {
        if self.num_features == 0 {
            return false;
        }
        let match_ratio = self.num_matches as Float / self.num_features as Float;
        match_ratio > 0.5
    }
}

/// VIO event types for monitoring.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum VIOEvent {
    /// Lost visual tracking
    LostTracking { frame_id: u32, reason: String },
    /// Recovered visual tracking
    RecoveredTracking { frame_id: u32, recovery_time_ms: Float },
    /// Loop closure detected
    LoopClosureFound {
        frame_id: u32,
        loop_frame_id: u32,
        confidence: Float,
    },
    /// Drift detected in estimate
    DriftDetected {
        frame_id: u32,
        drift_rate: Float,
    },
    /// Initialization complete
    InitializationComplete { frame_id: u32, duration_ms: Float },
    /// Configuration changed
    ConfigurationChanged { old_config: String, new_config: String },
    /// Resource pressure (CPU/memory)
    ResourcePressure {
        frame_id: u32,
        memory_usage_mb: u32,
        cpu_load: Float,
    },
    /// Error occurred
    Error {
        frame_id: u32,
        error_type: String,
        description: String,
    },
}

impl VIOEvent {
    /// Get the frame ID associated with this event.
    pub fn frame_id(&self) -> Option<u32> {
        match self {
            Self::LostTracking { frame_id, .. } => Some(*frame_id),
            Self::RecoveredTracking { frame_id, .. } => Some(*frame_id),
            Self::LoopClosureFound { frame_id, .. } => Some(*frame_id),
            Self::DriftDetected { frame_id, .. } => Some(*frame_id),
            Self::InitializationComplete { frame_id, .. } => Some(*frame_id),
            Self::ConfigurationChanged { .. } => None,
            Self::ResourcePressure { frame_id, .. } => Some(*frame_id),
            Self::Error { frame_id, .. } => Some(*frame_id),
        }
    }

    /// Get the human-readable event type name.
    pub fn event_type_name(&self) -> &'static str {
        match self {
            Self::LostTracking { .. } => "LostTracking",
            Self::RecoveredTracking { .. } => "RecoveredTracking",
            Self::LoopClosureFound { .. } => "LoopClosureFound",
            Self::DriftDetected { .. } => "DriftDetected",
            Self::InitializationComplete { .. } => "InitializationComplete",
            Self::ConfigurationChanged { .. } => "ConfigurationChanged",
            Self::ResourcePressure { .. } => "ResourcePressure",
            Self::Error { .. } => "Error",
        }
    }
}

impl std::fmt::Display for VIOEvent {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::LostTracking { frame_id, reason } => {
                write!(f, "LostTracking(frame={}, reason={})", frame_id, reason)
            }
            Self::RecoveredTracking {
                frame_id,
                recovery_time_ms,
            } => {
                write!(f, "RecoveredTracking(frame={}, time={}ms)", frame_id, recovery_time_ms)
            }
            Self::LoopClosureFound {
                frame_id,
                loop_frame_id,
                confidence,
            } => {
                write!(
                    f,
                    "LoopClosureFound(frame={}, loop={}, conf={})",
                    frame_id, loop_frame_id, confidence
                )
            }
            Self::DriftDetected {
                frame_id,
                drift_rate,
            } => {
                write!(f, "DriftDetected(frame={}, drift={})", frame_id, drift_rate)
            }
            Self::InitializationComplete {
                frame_id,
                duration_ms,
            } => {
                write!(f, "InitializationComplete(frame={}, duration={}ms)", frame_id, duration_ms)
            }
            Self::ConfigurationChanged {
                old_config,
                new_config,
            } => {
                write!(f, "ConfigurationChanged({} -> {})", old_config, new_config)
            }
            Self::ResourcePressure {
                frame_id,
                memory_usage_mb,
                cpu_load,
            } => {
                write!(
                    f,
                    "ResourcePressure(frame={}, mem={}MB, cpu={})",
                    frame_id, memory_usage_mb, cpu_load
                )
            }
            Self::Error {
                frame_id,
                error_type,
                description,
            } => {
                write!(f, "Error(frame={}, type={}, desc={})", frame_id, error_type, description)
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_health_state_operational() {
        assert!(HealthState::Healthy.is_operational());
        assert!(HealthState::Degraded.is_operational());
        assert!(!HealthState::Unhealthy.is_operational());
    }

    #[test]
    fn test_health_state_nominal() {
        assert!(HealthState::Healthy.is_nominal());
        assert!(!HealthState::Degraded.is_nominal());
        assert!(!HealthState::Unhealthy.is_nominal());
    }

    #[test]
    fn test_component_health_new() {
        let ch = ComponentHealth::new(0.0);
        assert!(ch.operational);
        assert_eq!(ch.error_rate, 0.0);
        assert!(ch.last_error.is_none());
    }

    #[test]
    fn test_component_health_mark_failure() {
        let ch = ComponentHealth::new(0.0).mark_failure("test error".to_string(), 1.0);
        assert!(!ch.operational);
        assert_eq!(ch.last_error, Some("test error".to_string()));
        assert_eq!(ch.last_update_secs, 1.0);
    }

    #[test]
    fn test_health_status_compute_reliability() {
        let hs = HealthStatus::new(0.0);
        let rel = hs.compute_reliability();
        assert!(rel > 0.9 && rel <= 1.0);
    }

    #[test]
    fn test_telemetry_frame_tracking() {
        let tf = TelemetryFrame {
            frame_id: 0,
            num_features: 100,
            num_matches: 60,
            solver_time_ms: 5.0,
            drift_estimate: None,
            window_size: 10,
            num_keyframes: 5,
            imu_healthy: true,
            num_loop_closure_candidates: 0,
        };
        assert!(tf.has_good_tracking());
        assert!(tf.has_sufficient_features(50));
    }

    #[test]
    fn test_vio_event_display() {
        let event = VIOEvent::LostTracking {
            frame_id: 42,
            reason: "insufficient features".to_string(),
        };
        assert_eq!(event.event_type_name(), "LostTracking");
        assert_eq!(event.frame_id(), Some(42));
        assert!(event.to_string().contains("42"));
    }

    #[test]
    fn test_vio_event_config_change() {
        let event = VIOEvent::ConfigurationChanged {
            old_config: "mode1".to_string(),
            new_config: "mode2".to_string(),
        };
        assert_eq!(event.frame_id(), None);
    }

    #[test]
    fn test_vio_event_resource_pressure() {
        let event = VIOEvent::ResourcePressure {
            frame_id: 10,
            memory_usage_mb: 512,
            cpu_load: 0.8,
        };
        assert_eq!(event.event_type_name(), "ResourcePressure");
        assert_eq!(event.frame_id(), Some(10));
    }
}
