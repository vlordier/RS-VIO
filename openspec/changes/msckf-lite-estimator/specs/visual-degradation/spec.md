## ADDED Requirements

### Requirement: Filter maintains an explicit visual quality mode
The system SHALL maintain a mode variable with three states: INIT, NORMAL, and DEGRADED. Transitions are governed by frame quality metrics with hysteresis.

#### Scenario: Initial mode is INIT
- **WHEN** the filter starts before initialization completes
- **THEN** the mode is INIT and no visual updates are applied

#### Scenario: Mode transitions NORMAL → DEGRADED on poor quality
- **WHEN** active track count drops below 25, stereo inlier ratio drops below 0.4, or median reprojection error exceeds 3.0px for a single frame
- **THEN** mode transitions to DEGRADED immediately

#### Scenario: Mode transitions DEGRADED → NORMAL on sustained recovery
- **WHEN** active track count exceeds 35, stereo inlier ratio exceeds 0.6, and median reprojection error is below 2.0px for 5 consecutive frames
- **THEN** mode transitions back to NORMAL

### Requirement: IMU-only propagation during DEGRADED mode
The system SHALL continue IMU state propagation at full rate during DEGRADED mode but SHALL NOT apply any visual EKF update.

#### Scenario: No visual update in DEGRADED mode
- **WHEN** a stereo frame arrives and mode is DEGRADED
- **THEN** feature tracking is still attempted (to maintain track continuity), clone creation is suspended, and no EKF update is performed

#### Scenario: Covariance grows during degraded segments
- **WHEN** IMU-only propagation continues for multiple frames in DEGRADED mode
- **THEN** covariance diagonal increases monotonically, reflecting growing uncertainty

### Requirement: Frame quality computed per stereo frame
The system SHALL compute per-frame quality metrics: active track count, stereo inlier ratio, and median reprojection error of currently-tracked features.

#### Scenario: Quality metrics available before mode check
- **WHEN** a stereo frame is processed
- **THEN** track count, stereo inlier ratio, and median reprojection error are computed before mode transition logic is evaluated

### Requirement: Degraded-mode updates allowed with strongest tracks only
The system SHALL optionally allow conservative updates during DEGRADED mode if at least 5 tracks pass a stricter gate (min 4 clone observations, reprojection < 1.5px).

#### Scenario: Conservative update in DEGRADED mode
- **WHEN** mode is DEGRADED and ≥5 tracks pass the strict gate
- **THEN** a conservative EKF update is applied using only those tracks with tighter noise covariance
