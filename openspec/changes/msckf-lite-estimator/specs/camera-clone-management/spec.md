## ADDED Requirements

### Requirement: Camera clone created on qualifying stereo frame
The system SHALL augment the filter state with a new camera clone when a stereo frame satisfies at least one cloning criterion: elapsed time > 0.08s, rotation > 3°, or translation > 0.08m from the last clone.

#### Scenario: Clone created on sufficient motion
- **WHEN** a stereo frame arrives and rotation since last clone exceeds 3° or translation exceeds 0.08m
- **THEN** a new clone is appended with pose T_w_c = T_w_i · T_i_c derived from current IMU nominal state

#### Scenario: Clone created on elapsed time
- **WHEN** a stereo frame arrives and more than 0.08s has elapsed since the last clone regardless of motion
- **THEN** a new clone is appended

#### Scenario: Frame rejected if no criterion met
- **WHEN** a stereo frame arrives but none of the cloning criteria are satisfied
- **THEN** no clone is created and the frame is used only for feature tracking continuity

### Requirement: Clone covariance augmented correctly
The system SHALL augment the covariance matrix when adding a clone such that the new clone's uncertainty is correctly derived from the current IMU state uncertainty and camera extrinsic uncertainty.

#### Scenario: Covariance augmented at clone creation
- **WHEN** a new clone is added
- **THEN** the covariance grows by 6 rows and 6 columns, with the new block computed as J · P_imu · Jᵀ where J is the pose composition Jacobian w.r.t. IMU state

### Requirement: Oldest clone marginalized when window is full
The system SHALL remove the oldest clone from the filter state when the number of active clones reaches the maximum (8), applying exact Schur complement marginalization.

#### Scenario: Oldest clone removed at capacity
- **WHEN** a new clone would be added and the clone count is already 8
- **THEN** the oldest clone's rows and columns are removed from state and covariance before the new clone is added

#### Scenario: Tracks with only marginalized clone observation discarded
- **WHEN** a clone is marginalized
- **THEN** any feature track whose only remaining observation is in the marginalized clone is discarded from the track manager

### Requirement: Maximum clone window is 8
The system SHALL maintain at most 8 camera clones in the filter state at any time, resulting in a maximum state dimension of 15 + 8·6 = 63.

#### Scenario: Clone count never exceeds 8
- **WHEN** clones are added continuously over many frames
- **THEN** the clone buffer never holds more than 8 entries and the state vector never exceeds 63 dimensions
