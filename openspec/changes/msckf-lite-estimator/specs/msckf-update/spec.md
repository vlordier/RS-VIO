## ADDED Requirements

### Requirement: Feature tracks mature after 3 clone observations
The system SHALL only use a feature track for an EKF update once it has been observed in at least 3 distinct camera clones.

#### Scenario: Track used only when mature
- **WHEN** building the measurement batch for an EKF update
- **THEN** only tracks with observations in ≥3 clones are included

#### Scenario: Immature tracks accumulate observations
- **WHEN** a track has fewer than 3 clone observations
- **THEN** it is kept in the track manager and grows with each new stereo frame

### Requirement: Landmark triangulated before residual formation
The system SHALL triangulate each mature feature track using multi-view DLT with stereo prior before forming residuals.

#### Scenario: Successful triangulation
- **WHEN** a track has ≥3 stereo clone observations
- **THEN** the landmark position p_f is estimated by minimizing reprojection error across all observations using DLT initialized by stereo disparity

#### Scenario: Failed triangulation rejects track
- **WHEN** triangulation produces negative depth, condition number > 100, or reprojection RMSE > 2.0px
- **THEN** the track is discarded and not included in the EKF update batch

### Requirement: Landmark eliminated via nullspace projection
The system SHALL eliminate the landmark from the residual by left-multiplying with the left nullspace of H_f before the EKF update.

#### Scenario: Nullspace projection applied
- **WHEN** forming the measurement batch for one track
- **THEN** H_f (landmark Jacobian) is computed, its left nullspace Q is computed via QR decomposition, and the projected residual r_o = Qᵀ·r and Jacobian H_o = Qᵀ·H_x are used in the update

#### Scenario: Landmark does not appear in EKF update
- **WHEN** the EKF Kalman gain is computed
- **THEN** only the 63-dim filter state (IMU + clones) is updated; no landmark state exists

### Requirement: Innovation gating rejects outlier tracks
The system SHALL apply a chi-square gate to each track's projected residual before including it in the batch.

#### Scenario: Track accepted by gate
- **WHEN** γ = r_oᵀ · (H_o · P · H_oᵀ + R_o)⁻¹ · r_o is below the χ²(4, 0.95) threshold
- **THEN** the track is included in the update batch

#### Scenario: Track rejected by gate
- **WHEN** γ exceeds the threshold
- **THEN** the track is excluded from the batch but kept in the track manager for future updates

### Requirement: Batch EKF update applied once per stereo frame
The system SHALL collect all gated track residuals into a single stacked H and r, then apply one EKF update per stereo frame using the standard Kalman equations.

#### Scenario: Batch update applied
- **WHEN** at least one gated track is available
- **THEN** state correction δx = K · r_stacked is computed and applied, and covariance is updated as P ← (I - K·H)·P

#### Scenario: No update when no gated tracks
- **WHEN** zero tracks pass gating (e.g., during DEGRADED mode)
- **THEN** no EKF update is applied and only IMU propagation advances the state
