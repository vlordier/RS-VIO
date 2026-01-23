/// Circuit Breaker Pattern Implementation for VIO Pipeline
/// 
/// Implements the circuit breaker pattern with three states:
/// - Closed: Normal operation, requests pass through
/// - Open: Failure threshold exceeded, requests fail fast
/// - HalfOpen: Testing recovery, limited requests allowed
/// 
/// Enables graceful degradation and prevention of cascade failures
/// in distributed drone swarms.

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

/// Circuit Breaker State
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CircuitState {
    /// Normal operation - requests pass through
    Closed,
    /// Failure threshold exceeded - requests fail fast
    Open,
    /// Testing recovery - limited requests allowed
    HalfOpen,
}

impl std::fmt::Display for CircuitState {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            CircuitState::Closed => write!(f, "Closed"),
            CircuitState::Open => write!(f, "Open"),
            CircuitState::HalfOpen => write!(f, "HalfOpen"),
        }
    }
}

/// Configuration for CircuitBreaker behavior
#[derive(Clone, Debug)]
pub struct CircuitBreakerConfig {
    /// Failure threshold before opening circuit (0.0-1.0)
    pub failure_ratio_threshold: f64,
    /// Minimum samples before considering ratio
    pub min_samples_for_evaluation: usize,
    /// Timeout before attempting half-open (ms)
    pub recovery_timeout_ms: u64,
    /// Maximum requests allowed in half-open state
    pub half_open_max_requests: usize,
}

impl Default for CircuitBreakerConfig {
    fn default() -> Self {
        Self {
            failure_ratio_threshold: 0.5,
            min_samples_for_evaluation: 10,
            recovery_timeout_ms: 5000,
            half_open_max_requests: 3,
        }
    }
}

/// Swarm Health Status for multi-drone coordination
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum SwarmHealthStatus {
    /// All drones healthy
    Healthy,
    /// Some drones degraded (> 20% failure rate)
    Degraded,
    /// Multiple drones failing (> 50% failure rate)
    Critical,
    /// All drones or mesh failed
    Failed,
}

impl std::fmt::Display for SwarmHealthStatus {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            SwarmHealthStatus::Healthy => write!(f, "Healthy"),
            SwarmHealthStatus::Degraded => write!(f, "Degraded"),
            SwarmHealthStatus::Critical => write!(f, "Critical"),
            SwarmHealthStatus::Failed => write!(f, "Failed"),
        }
    }
}

/// Circuit breaker state tracking
struct CircuitBreakerState {
    state: CircuitState,
    failure_count: usize,
    success_count: usize,
    last_state_change: u64,
    half_open_requests: usize,
    swarm_status: SwarmHealthStatus,
}

impl CircuitBreakerState {
    fn failure_ratio(&self) -> f64 {
        let total = self.failure_count + self.success_count;
        if total == 0 {
            0.0
        } else {
            self.failure_count as f64 / total as f64
        }
    }
}

/// Circuit Breaker for preventing cascade failures
pub struct CircuitBreaker {
    config: Arc<Mutex<CircuitBreakerConfig>>,
    state: Arc<Mutex<CircuitBreakerState>>,
    total_requests: Arc<AtomicU64>,
    total_failures: Arc<AtomicU64>,
}

impl CircuitBreaker {
    /// Create new circuit breaker with default config
    pub fn new() -> Self {
        Self::with_config(CircuitBreakerConfig::default())
    }

    /// Create circuit breaker with custom config
    pub fn with_config(config: CircuitBreakerConfig) -> Self {
        let state = CircuitBreakerState {
            state: CircuitState::Closed,
            failure_count: 0,
            success_count: 0,
            last_state_change: current_time_ms(),
            half_open_requests: 0,
            swarm_status: SwarmHealthStatus::Healthy,
        };

        Self {
            config: Arc::new(Mutex::new(config)),
            state: Arc::new(Mutex::new(state)),
            total_requests: Arc::new(AtomicU64::new(0)),
            total_failures: Arc::new(AtomicU64::new(0)),
        }
    }

    /// Record a successful operation
    pub fn record_success(&self) {
        self.total_requests.fetch_add(1, Ordering::Relaxed);
        
        let mut state = self.state.lock().unwrap();
        state.success_count += 1;
        let config = self.config.lock().unwrap();

        match state.state {
            CircuitState::Closed => {
                // Check if we should open due to accumulated failures
                let total = state.failure_count + state.success_count;
                if total >= config.min_samples_for_evaluation {
                    let ratio = state.failure_ratio();
                    if ratio > config.failure_ratio_threshold {
                        state.state = CircuitState::Open;
                        state.last_state_change = current_time_ms();
                    }
                }
            }
            CircuitState::Open => {
                // Attempt recovery in half-open state
                let elapsed = current_time_ms() - state.last_state_change;
                if elapsed >= config.recovery_timeout_ms {
                    state.state = CircuitState::HalfOpen;
                    state.failure_count = 0;
                    state.success_count = 1;
                    state.half_open_requests = 1;
                    state.last_state_change = current_time_ms();
                }
            }
            CircuitState::HalfOpen => {
                state.half_open_requests += 1;
                // After successful requests, close circuit
                if state.half_open_requests >= 3 {
                    state.state = CircuitState::Closed;
                    state.failure_count = 0;
                    state.success_count = 0;
                    state.last_state_change = current_time_ms();
                }
            }
        }
    }

    /// Record a failed operation
    pub fn record_failure(&self) {
        self.total_requests.fetch_add(1, Ordering::Relaxed);
        self.total_failures.fetch_add(1, Ordering::Relaxed);

        let mut state = self.state.lock().unwrap();
        let config = self.config.lock().unwrap();

        // In half-open state, any failure reopens circuit immediately
        if state.state == CircuitState::HalfOpen {
            state.state = CircuitState::Open;
            state.failure_count = 1;
            state.success_count = 0;
            state.last_state_change = current_time_ms();
            return;
        }

        state.failure_count += 1;

        // Check if we should open the circuit
        let total = state.failure_count + state.success_count;
        if total >= config.min_samples_for_evaluation {
            let ratio = state.failure_ratio();
            if ratio > config.failure_ratio_threshold {
                state.state = CircuitState::Open;
                state.last_state_change = current_time_ms();
            }
        }
    }

    /// Get current circuit state
    pub fn state(&self) -> CircuitState {
        self.state.lock().unwrap().state
    }

    /// Check if operation is allowed
    pub fn is_operation_allowed(&self) -> bool {
        let state = self.state.lock().unwrap();
        match state.state {
            CircuitState::Closed => true,
            CircuitState::Open => {
                let config = self.config.lock().unwrap();
                let elapsed = current_time_ms() - state.last_state_change;
                elapsed >= config.recovery_timeout_ms
            }
            CircuitState::HalfOpen => {
                let config = self.config.lock().unwrap();
                state.half_open_requests < config.half_open_max_requests
            }
        }
    }

    /// Get failure ratio (0.0-1.0)
    pub fn failure_ratio(&self) -> f64 {
        self.state.lock().unwrap().failure_ratio()
    }

    /// Get total requests processed
    pub fn total_requests(&self) -> u64 {
        self.total_requests.load(Ordering::Relaxed)
    }

    /// Get total failures recorded
    pub fn total_failures(&self) -> u64 {
        self.total_failures.load(Ordering::Relaxed)
    }

    /// Get swarm health status
    pub fn swarm_status(&self) -> SwarmHealthStatus {
        self.state.lock().unwrap().swarm_status.clone()
    }

    /// Set swarm health status (for multi-drone coordination)
    pub fn set_swarm_status(&self, status: SwarmHealthStatus) {
        self.state.lock().unwrap().swarm_status = status;
    }

    /// Reset circuit breaker to initial state
    pub fn reset(&self) {
        let mut state = self.state.lock().unwrap();
        state.state = CircuitState::Closed;
        state.failure_count = 0;
        state.success_count = 0;
        state.last_state_change = current_time_ms();
        state.half_open_requests = 0;
        state.swarm_status = SwarmHealthStatus::Healthy;
    }

    /// Broadcast health to swarm (returns true if healthy)
    pub fn broadcast_health_status(&self) -> bool {
        let state = self.state.lock().unwrap();
        state.state == CircuitState::Closed
    }

    /// Get statistics for monitoring
    pub fn get_statistics(&self) -> CircuitBreakerStats {
        let state = self.state.lock().unwrap();
        CircuitBreakerStats {
            state: state.state,
            failure_ratio: state.failure_ratio(),
            total_failures: state.failure_count,
            total_successes: state.success_count,
            swarm_status: state.swarm_status.clone(),
        }
    }
}

impl Default for CircuitBreaker {
    fn default() -> Self {
        Self::new()
    }
}

impl Clone for CircuitBreaker {
    fn clone(&self) -> Self {
        Self {
            config: Arc::clone(&self.config),
            state: Arc::clone(&self.state),
            total_requests: Arc::clone(&self.total_requests),
            total_failures: Arc::clone(&self.total_failures),
        }
    }
}

/// Statistics snapshot for monitoring
#[derive(Clone, Debug)]
pub struct CircuitBreakerStats {
    pub state: CircuitState,
    pub failure_ratio: f64,
    pub total_failures: usize,
    pub total_successes: usize,
    pub swarm_status: SwarmHealthStatus,
}

impl std::fmt::Display for CircuitBreakerStats {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "CircuitBreaker[state={}, ratio={:.2}%, failures={}, successes={}, swarm={}]",
            self.state,
            self.failure_ratio * 100.0,
            self.total_failures,
            self.total_successes,
            self.swarm_status
        )
    }
}

/// Get current time in milliseconds since UNIX epoch
fn current_time_ms() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or(Duration::from_secs(0))
        .as_millis() as u64
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_circuit_breaker_creation() {
        let cb = CircuitBreaker::new();
        assert_eq!(cb.state(), CircuitState::Closed);
        assert_eq!(cb.failure_ratio(), 0.0);
        assert_eq!(cb.total_requests(), 0);
    }

    #[test]
    fn test_circuit_breaker_with_config() {
        let config = CircuitBreakerConfig {
            failure_ratio_threshold: 0.3,
            min_samples_for_evaluation: 5,
            ..Default::default()
        };
        let cb = CircuitBreaker::with_config(config);
        assert_eq!(cb.state(), CircuitState::Closed);
    }

    #[test]
    fn test_record_success() {
        let cb = CircuitBreaker::new();
        cb.record_success();
        assert_eq!(cb.total_requests(), 1);
        assert_eq!(cb.failure_ratio(), 0.0);
        assert_eq!(cb.state(), CircuitState::Closed);
    }

    #[test]
    fn test_record_failure() {
        let cb = CircuitBreaker::new();
        cb.record_failure();
        assert_eq!(cb.total_requests(), 1);
        assert_eq!(cb.total_failures(), 1);
        assert_eq!(cb.state(), CircuitState::Closed);
    }

    #[test]
    fn test_state_transition_to_open() {
        let config = CircuitBreakerConfig {
            failure_ratio_threshold: 0.5,
            min_samples_for_evaluation: 5,
            ..Default::default()
        };
        let cb = CircuitBreaker::with_config(config);

        // Record 3 failures and 2 successes (60% failure rate)
        for _ in 0..3 {
            cb.record_failure();
        }
        for _ in 0..2 {
            cb.record_success();
        }

        assert_eq!(cb.state(), CircuitState::Open);
    }

    #[test]
    fn test_operation_not_allowed_when_open() {
        let config = CircuitBreakerConfig {
            failure_ratio_threshold: 0.5,
            min_samples_for_evaluation: 5,
            recovery_timeout_ms: 10000,
            ..Default::default()
        };
        let cb = CircuitBreaker::with_config(config);

        for _ in 0..3 {
            cb.record_failure();
        }
        for _ in 0..2 {
            cb.record_success();
        }

        assert_eq!(cb.state(), CircuitState::Open);
        assert!(!cb.is_operation_allowed());
    }

    #[test]
    fn test_half_open_recovery() {
        let config = CircuitBreakerConfig {
            failure_ratio_threshold: 0.5,
            min_samples_for_evaluation: 5,
            recovery_timeout_ms: 0,
            half_open_max_requests: 3,
        };
        let cb = CircuitBreaker::with_config(config);

        // Open circuit
        for _ in 0..3 {
            cb.record_failure();
        }
        for _ in 0..2 {
            cb.record_success();
        }
        assert_eq!(cb.state(), CircuitState::Open);

        // Wait and record success to transition to half-open
        std::thread::sleep(Duration::from_millis(10));
        cb.record_success();
        assert_eq!(cb.state(), CircuitState::HalfOpen);

        // More successes close circuit
        cb.record_success();
        cb.record_success();
        assert_eq!(cb.state(), CircuitState::Closed);
    }

    #[test]
    fn test_half_open_failure_reopens() {
        let config = CircuitBreakerConfig {
            failure_ratio_threshold: 0.5,
            min_samples_for_evaluation: 5,
            recovery_timeout_ms: 0,
            ..Default::default()
        };
        let cb = CircuitBreaker::with_config(config);

        // Open circuit
        for _ in 0..3 {
            cb.record_failure();
        }
        for _ in 0..2 {
            cb.record_success();
        }
        assert_eq!(cb.state(), CircuitState::Open);

        // Transition to half-open
        std::thread::sleep(Duration::from_millis(10));
        cb.record_success();
        assert_eq!(cb.state(), CircuitState::HalfOpen);

        // Failure reopens circuit
        cb.record_failure();
        assert_eq!(cb.state(), CircuitState::Open);
    }

    #[test]
    fn test_reset() {
        let cb = CircuitBreaker::new();
        cb.record_failure();
        cb.record_failure();
        cb.record_failure();

        assert!(cb.total_failures() > 0);
        cb.reset();

        assert_eq!(cb.state(), CircuitState::Closed);
        assert_eq!(cb.failure_ratio(), 0.0);
    }

    #[test]
    fn test_swarm_status() {
        let cb = CircuitBreaker::new();
        assert_eq!(cb.swarm_status(), SwarmHealthStatus::Healthy);

        cb.set_swarm_status(SwarmHealthStatus::Degraded);
        assert_eq!(cb.swarm_status(), SwarmHealthStatus::Degraded);
    }

    #[test]
    fn test_broadcast_health_status() {
        let cb = CircuitBreaker::new();
        assert!(cb.broadcast_health_status());

        // Open circuit by forcing failures
        let config = CircuitBreakerConfig {
            failure_ratio_threshold: 0.5,
            min_samples_for_evaluation: 5,
            ..Default::default()
        };
        let cb2 = CircuitBreaker::with_config(config);
        for _ in 0..3 {
            cb2.record_failure();
        }
        for _ in 0..2 {
            cb2.record_success();
        }
        assert!(!cb2.broadcast_health_status());
    }

    #[test]
    fn test_statistics() {
        let cb = CircuitBreaker::new();
        cb.record_success();
        cb.record_failure();
        cb.record_success();

        let stats = cb.get_statistics();
        assert_eq!(stats.state, CircuitState::Closed);
        assert_eq!(stats.total_successes, 2);
        assert_eq!(stats.total_failures, 1);
    }

    #[test]
    fn test_statistics_display() {
        let cb = CircuitBreaker::new();
        cb.record_success();
        cb.record_failure();

        let stats = cb.get_statistics();
        let display = format!("{}", stats);
        assert!(display.contains("CircuitBreaker"));
        assert!(display.contains("50.00%"));
    }

    #[test]
    fn test_clone() {
        let cb1 = CircuitBreaker::new();
        cb1.record_success();
        cb1.record_failure();

        let cb2 = cb1.clone();
        assert_eq!(cb2.state(), cb1.state());
        assert_eq!(cb2.failure_ratio(), cb1.failure_ratio());
    }

    #[test]
    fn test_no_state_change_below_threshold() {
        let config = CircuitBreakerConfig {
            failure_ratio_threshold: 0.5,
            min_samples_for_evaluation: 5,
            ..Default::default()
        };
        let cb = CircuitBreaker::with_config(config);

        cb.record_failure();
        assert_eq!(cb.state(), CircuitState::Closed);

        cb.record_success();
        assert_eq!(cb.state(), CircuitState::Closed);
    }

    #[test]
    fn test_default_config() {
        let config = CircuitBreakerConfig::default();
        assert_eq!(config.failure_ratio_threshold, 0.5);
        assert_eq!(config.min_samples_for_evaluation, 10);
        assert_eq!(config.recovery_timeout_ms, 5000);
        assert_eq!(config.half_open_max_requests, 3);
    }

    #[test]
    fn test_circuit_state_display() {
        assert_eq!(format!("{}", CircuitState::Closed), "Closed");
        assert_eq!(format!("{}", CircuitState::Open), "Open");
        assert_eq!(format!("{}", CircuitState::HalfOpen), "HalfOpen");
    }

    #[test]
    fn test_swarm_health_status_display() {
        assert_eq!(format!("{}", SwarmHealthStatus::Healthy), "Healthy");
        assert_eq!(format!("{}", SwarmHealthStatus::Degraded), "Degraded");
        assert_eq!(format!("{}", SwarmHealthStatus::Critical), "Critical");
        assert_eq!(format!("{}", SwarmHealthStatus::Failed), "Failed");
    }

    #[test]
    fn test_failure_ratio_calculation() {
        let cb = CircuitBreaker::new();
        assert_eq!(cb.failure_ratio(), 0.0);

        cb.record_success();
        assert_eq!(cb.failure_ratio(), 0.0);

        cb.record_failure();
        assert!(cb.failure_ratio() > 0.4 && cb.failure_ratio() < 0.6);
    }

    #[test]
    fn test_multiple_failure_cycles() {
        let config = CircuitBreakerConfig {
            failure_ratio_threshold: 0.5,
            min_samples_for_evaluation: 5,
            recovery_timeout_ms: 0,
            ..Default::default()
        };
        let cb = CircuitBreaker::with_config(config);

        // First cycle: fail -> open
        for _ in 0..3 {
            cb.record_failure();
        }
        for _ in 0..2 {
            cb.record_success();
        }
        assert_eq!(cb.state(), CircuitState::Open);

        // Second cycle: recover -> half-open -> closed
        std::thread::sleep(Duration::from_millis(10));
        cb.record_success();
        assert_eq!(cb.state(), CircuitState::HalfOpen);

        cb.record_success();
        cb.record_success();
        assert_eq!(cb.state(), CircuitState::Closed);

        // Third cycle: fail again
        cb.record_failure();
        cb.record_failure();
        cb.record_failure();
        for _ in 0..2 {
            cb.record_success();
        }
        assert_eq!(cb.state(), CircuitState::Open);
    }
}
