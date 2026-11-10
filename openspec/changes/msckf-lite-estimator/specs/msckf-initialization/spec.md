## ADDED Requirements

### Requirement: Gyro bias estimated from static segment
The system SHALL estimate initial gyro bias by averaging raw gyroscope measurements during a static or near-static startup segment (first 0.5s or until IMU variance drops below threshold).

#### Scenario: Gyro bias from static average
- **WHEN** the first 0.5s of IMU data is collected with angular velocity variance < 0.01 rad²/s²
- **THEN** gyro bias b_g is initialized as the mean of those measurements

#### Scenario: Gyro bias fallback to zero
- **WHEN** the startup segment is not static (variance exceeds threshold)
- **THEN** b_g is initialized to zero and initialization continues

### Requirement: Gravity direction estimated from accelerometer average
The system SHALL estimate the gravity direction (and thus initial roll and pitch) by averaging accelerometer measurements over the same static startup segment.

#### Scenario: Gravity alignment from accel
- **WHEN** a valid static segment is available
- **THEN** the world frame z-axis is aligned with −mean(a_meas), giving initial R_w_i with correct roll and pitch (yaw is unobservable and set to zero)

#### Scenario: Gravity magnitude validates sensor health
- **WHEN** the accelerometer mean magnitude deviates more than 0.5 m/s² from 9.81 m/s²
- **THEN** initialization is rejected and retried

### Requirement: Visual bootstrap provides initial velocity estimate
The system SHALL estimate initial velocity from the first pair of stereo frames with sufficient features before transitioning to NORMAL mode.

#### Scenario: Initial velocity from visual geometry
- **WHEN** the first two stereo frames with ≥20 tracked features are available
- **THEN** initial velocity is estimated from the visual displacement divided by elapsed time, constrained by IMU integration

#### Scenario: Mode transitions to NORMAL after bootstrap
- **WHEN** gyro bias, gravity alignment, and initial velocity estimate are all available
- **THEN** the filter mode transitions from INIT to NORMAL and EKF updates begin

### Requirement: Initialization can proceed without static segment
The system SHALL handle datasets where motion begins immediately (no static startup) by deferring bias estimation and using zero initial bias with larger initial uncertainty.

#### Scenario: Non-static startup
- **WHEN** the initial segment does not satisfy the static threshold
- **THEN** b_g = 0 and b_a = 0 with inflated initial covariance (σ_bg = 0.1 rad/s, σ_ba = 0.5 m/s²), and the filter proceeds to NORMAL mode after visual bootstrap
