## ADDED Requirements

### Requirement: IMU state propagates at sensor rate
The system SHALL propagate the nominal IMU state (rotation, velocity, position, gyro bias, accel bias) on every incoming IMU sample using midpoint integration.

#### Scenario: Nominal propagation step
- **WHEN** an IMU sample arrives with angular velocity ω and linear acceleration a
- **THEN** the nominal state updates as: R_{k+1} = R_k · Exp(ω_mid · dt), v_{k+1} = v_k + (R_k · a_mid + g) · dt, p_{k+1} = p_k + v_k · dt + 0.5 · (R_k · a_mid + g) · dt²

#### Scenario: Bias correction applied before integration
- **WHEN** propagating an IMU sample
- **THEN** gyro bias is subtracted from ω_meas and accel bias is subtracted from a_meas before integration

### Requirement: Covariance propagates with full cross-terms
The system SHALL propagate the full (15 + 6·N_clones) × (15 + 6·N_clones) covariance matrix, preserving cross-terms between the IMU block and all camera clone blocks.

#### Scenario: Cross-terms preserved during propagation
- **WHEN** IMU covariance is propagated
- **THEN** the cross-terms P_{imu,clone_i} are updated via Φ_imu (the 15×15 IMU Jacobian) while clone diagonal blocks remain unchanged

#### Scenario: Process noise added each step
- **WHEN** propagating covariance for a dt step
- **THEN** discretized process noise Q_d is added to the IMU block only (clones have no process noise)

### Requirement: Gravity vector is fixed in world frame
The system SHALL use a fixed gravity vector g = [0, 0, -9.81] m/s² in the world frame after initialization.

#### Scenario: Gravity used in propagation
- **WHEN** computing acceleration in world frame
- **THEN** gravity is added as R_k · a_body + g_world before velocity integration
