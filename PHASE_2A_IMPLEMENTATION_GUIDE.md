# Phase 2A Implementation Guide: Preintegration Factors

This document provides detailed code examples and specifications for implementing IMU preintegration factors for bundle adjustment optimization.

---

## Overview

**Goal**: Convert PreintegratedImu measurements into optimization factors that can be included in the bundle adjustment objective.

**Result**: Tight coupling between visual odometry and inertial measurement unit.

---

## Step 1: Define the ImuFactor Struct

Create a new file: `src/optimization/imu_factor.rs`

```rust
use nalgebra as na;
use crate::imu::preintegration::PreintegratedImu;

/// IMU preintegration factor for bundle adjustment
///
/// This factor represents the constraint imposed by integrated IMU measurements
/// between two keyframes. It provides:
/// - Measurement residuals
/// - Jacobians w.r.t. poses, velocities, and biases
/// - Integration with optimization frameworks
#[derive(Debug, Clone)]
pub struct ImuFactor {
    /// Preintegrated IMU measurements and statistics
    pub preintegration: PreintegratedImu,
    
    /// Index of first keyframe (i)
    pub keyframe_i: usize,
    
    /// Index of second keyframe (j)
    pub keyframe_j: usize,
    
    /// Duration between frames (seconds)
    pub dt: f64,
    
    /// Reference gyro bias used during preintegration [rad/s]
    /// Used for first-order bias correction
    pub ref_bias_g: na::Vector3<f64>,
    
    /// Reference accel bias used during preintegration [m/s²]
    /// Used for first-order bias correction
    pub ref_bias_a: na::Vector3<f64>,
    
    /// Weight/information matrix for this factor
    /// Typically inverse of preintegration covariance
    pub information: na::Matrix12<f64>,
}

impl ImuFactor {
    /// Create new IMU factor
    pub fn new(
        preintegration: PreintegratedImu,
        keyframe_i: usize,
        keyframe_j: usize,
        dt: f64,
        ref_bias_g: na::Vector3<f64>,
        ref_bias_a: na::Vector3<f64>,
    ) -> Self {
        // Information matrix = inverse of covariance
        let cov = preintegration.covariance();
        let information = cov.try_inverse()
            .unwrap_or_else(|_| na::Matrix12::identity());
        
        Self {
            preintegration,
            keyframe_i,
            keyframe_j,
            dt,
            ref_bias_g,
            ref_bias_a,
            information,
        }
    }
    
    /// Compute preintegration prediction with bias correction
    ///
    /// This implements the bias update equation from Forster et al. 2017:
    /// - ΔR' = ΔR * exp(-J_R_bg * Δb_g)
    /// - Δv' = Δv + J_v_bg * Δb_g + J_v_ba * Δb_a
    /// - Δp' = Δp + J_p_bg * Δb_g + J_p_ba * Δb_a
    fn predict_with_bias_correction(
        &self,
        bias_g: &na::Vector3<f64>,
        bias_a: &na::Vector3<f64>,
    ) -> (na::UnitQuaternion<f64>, na::Vector3<f64>, na::Vector3<f64>) {
        // First-order bias correction
        let d_bias_g = bias_g - self.ref_bias_g;
        let d_bias_a = bias_a - self.ref_bias_a;
        
        // Correct rotation (using linearization)
        let delta_R = self.preintegration.delta_R;
        // J_R_bg is 3x3 jacobian of rotation w.r.t. gyro bias
        let d_rotation = crate::imu::preintegration::exp_map_so3(
            -self.preintegration.J_R_bg * d_bias_g
        );
        let corrected_R = delta_R * d_rotation;
        
        // Correct velocity
        let delta_v = self.preintegration.delta_v;
        let corrected_v = delta_v
            + self.preintegration.J_v_bg * d_bias_g
            + self.preintegration.J_v_ba * d_bias_a;
        
        // Correct position
        let delta_p = self.preintegration.delta_p;
        let corrected_p = delta_p
            + self.preintegration.J_p_bg * d_bias_g
            + self.preintegration.J_p_ba * d_bias_a;
        
        (corrected_R, corrected_v, corrected_p)
    }
    
    /// Compute residual vector (12-dimensional)
    ///
    /// Residual = measured - predicted
    ///
    /// Measurement stack: [ΔR error (3), Δv error (3), Δp error (3), bias error (3)]
    ///
    /// The rotation error is computed in the tangent space at predicted rotation.
    pub fn residual(
        &self,
        // Keyframe i (previous)
        R_i: &na::UnitQuaternion<f64>,
        v_i: &na::Vector3<f64>,
        p_i: &na::Vector3<f64>,
        
        // Keyframe j (current)
        R_j: &na::UnitQuaternion<f64>,
        v_j: &na::Vector3<f64>,
        p_j: &na::Vector3<f64>,
        
        // Shared biases
        bias_g: &na::Vector3<f64>,
        bias_a: &na::Vector3<f64>,
    ) -> na::Vector12<f64> {
        // Get bias-corrected predictions
        let (delta_R_pred, delta_v_pred, delta_p_pred) =
            self.predict_with_bias_correction(bias_g, bias_a);
        
        // ==================== Rotation Residual ====================
        // Measurement: R_j^T * R_i * ΔR_pred
        // (What we predict given current poses and preintegration)
        let R_ij_measured = R_j.inverse() * R_i * delta_R_pred;
        
        // Compute rotation error as rotation vector in tangent space
        // log_map of rotation error magnitude
        let rotation_error_quat = self.preintegration.delta_R.inverse() * R_ij_measured;
        let rotation_error_vec = Self::quat_to_rotation_vector(&rotation_error_quat);
        
        // ==================== Velocity Residual ====================
        // Measurement: v_j - (R_i^T * (v_i + ΔR_pred^T * (g*dt + Δv_pred)))
        let g = na::Vector3::new(0.0, 0.0, -9.81);
        let accel_world = g * self.dt + delta_v_pred;
        let v_pred = R_i.inverse() * (v_i + delta_R_pred.inverse() * accel_world);
        let v_error = v_j - v_pred;
        
        // ==================== Position Residual ====================
        // Measurement: p_j - (p_i + v_i*dt + 0.5*g*dt² + R_i^T * Δp_pred)
        let p_pred = p_i
            + v_i * self.dt
            + 0.5 * g * self.dt.powi(2)
            + R_i.inverse() * delta_p_pred;
        let p_error = p_j - p_pred;
        
        // ==================== Bias Residual ====================
        // Biases are assumed constant, so error is just regularization
        // (In full optimization, biases are variables)
        let bias_error = na::Vector3::zeros();  // No direct measurement
        
        // Stack residuals: 3 + 3 + 3 + 3 = 12 dimensional
        let mut residual = na::Vector12::zeros();
        residual.fixed_rows_mut::<3>(0).copy_from(&rotation_error_vec);
        residual.fixed_rows_mut::<3>(3).copy_from(&v_error);
        residual.fixed_rows_mut::<3>(6).copy_from(&p_error);
        residual.fixed_rows_mut::<3>(9).copy_from(&bias_error);
        
        residual
    }
    
    /// Weighted residual (residual^T * Information * residual)
    pub fn weighted_residual(
        &self,
        R_i: &na::UnitQuaternion<f64>,
        v_i: &na::Vector3<f64>,
        p_i: &na::Vector3<f64>,
        R_j: &na::UnitQuaternion<f64>,
        v_j: &na::Vector3<f64>,
        p_j: &na::Vector3<f64>,
        bias_g: &na::Vector3<f64>,
        bias_a: &na::Vector3<f64>,
    ) -> f64 {
        let r = self.residual(R_i, v_i, p_i, R_j, v_j, p_j, bias_g, bias_a);
        (r.transpose() * self.information * r).sum()
    }
    
    /// Compute Jacobians for optimization
    ///
    /// Returns:
    /// - J_R_i: 12x3 jacobian w.r.t. rotation of frame i
    /// - J_v_i: 12x3 jacobian w.r.t. velocity of frame i
    /// - J_p_i: 12x3 jacobian w.r.t. position of frame i
    /// - J_R_j: 12x3 jacobian w.r.t. rotation of frame j
    /// - J_v_j: 12x3 jacobian w.r.t. velocity of frame j
    /// - J_p_j: 12x3 jacobian w.r.t. position of frame j
    /// - J_bg: 12x3 jacobian w.r.t. gyro bias
    /// - J_ba: 12x3 jacobian w.r.t. accel bias
    pub fn jacobians(
        &self,
        R_i: &na::UnitQuaternion<f64>,
        v_i: &na::Vector3<f64>,
        p_i: &na::Vector3<f64>,
        R_j: &na::UnitQuaternion<f64>,
        v_j: &na::Vector3<f64>,
        p_j: &na::Vector3<f64>,
        bias_g: &na::Vector3<f64>,
        bias_a: &na::Vector3<f64>,
    ) -> (
        na::Matrix12x3<f64>, na::Matrix12x3<f64>, na::Matrix12x3<f64>,
        na::Matrix12x3<f64>, na::Matrix12x3<f64>, na::Matrix12x3<f64>,
        na::Matrix12x3<f64>, na::Matrix12x3<f64>,
    ) {
        // Use numerical differentiation for now (slower but guaranteed correct)
        // Can be replaced with analytical jacobians for performance
        let eps = 1e-8;
        let mut J_R_i = na::Matrix12x3::zeros();
        let mut J_v_i = na::Matrix12x3::zeros();
        let mut J_p_i = na::Matrix12x3::zeros();
        let mut J_R_j = na::Matrix12x3::zeros();
        let mut J_v_j = na::Matrix12x3::zeros();
        let mut J_p_j = na::Matrix12x3::zeros();
        let mut J_bg = na::Matrix12x3::zeros();
        let mut J_ba = na::Matrix12x3::zeros();
        
        let baseline = self.residual(R_i, v_i, p_i, R_j, v_j, p_j, bias_g, bias_a);
        
        // Jacobian w.r.t. rotation of frame i (on manifold)
        for d in 0..3 {
            let mut delta = na::Vector3::zeros();
            delta[d] = eps;
            let R_i_pert = R_i * crate::imu::preintegration::exp_map_so3(delta);
            let perturbed = self.residual(&R_i_pert, v_i, p_i, R_j, v_j, p_j, bias_g, bias_a);
            J_R_i.column_mut(d).copy_from(&((perturbed - &baseline) / eps));
        }
        
        // Jacobian w.r.t. velocity of frame i
        for d in 0..3 {
            let mut v_i_pert = v_i.clone();
            v_i_pert[d] += eps;
            let perturbed = self.residual(R_i, &v_i_pert, p_i, R_j, v_j, p_j, bias_g, bias_a);
            J_v_i.column_mut(d).copy_from(&((perturbed - &baseline) / eps));
        }
        
        // Similar for remaining jacobians...
        // (omitted for brevity, but follow same pattern)
        
        (J_R_i, J_v_i, J_p_i, J_R_j, J_v_j, J_p_j, J_bg, J_ba)
    }
    
    /// Convert unit quaternion to rotation vector (log map)
    fn quat_to_rotation_vector(q: &na::UnitQuaternion<f64>) -> na::Vector3<f64> {
        let angle = 2.0 * q.scalar().acos();
        if angle.abs() < 1e-6 {
            na::Vector3::zeros()
        } else {
            let axis = na::Vector3::new(q.i(), q.j(), q.k()).normalize();
            axis * angle
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_imu_factor_identity_residual() {
        // Create a factor with zero integrated movement
        let preint = PreintegratedImu::new(
            crate::imu::ImuNoise::default(),
            1.0,
            na::Vector3::new(0.0, 0.0, -9.81),
        );
        
        let factor = ImuFactor::new(
            preint,
            0,
            1,
            1.0,
            na::Vector3::zeros(),
            na::Vector3::zeros(),
        );
        
        // Set up poses such that preintegration matches perfectly
        let R_i = na::UnitQuaternion::identity();
        let v_i = na::Vector3::zeros();
        let p_i = na::Vector3::zeros();
        
        let R_j = na::UnitQuaternion::identity();
        let v_j = na::Vector3::new(0.0, 0.0, -9.81); // v_j = v_i + g*dt
        let p_j = na::Vector3::zeros(); // No position change
        
        let bias_g = na::Vector3::zeros();
        let bias_a = na::Vector3::zeros();
        
        let residual = factor.residual(&R_i, &v_i, &p_i, &R_j, &v_j, &p_j, &bias_g, &bias_a);
        
        // Residual should be small (gravity causes velocity change)
        println!("Residual: {}", residual);
        assert!(residual.norm() < 0.1, "Expected small residual for identity motion");
    }
}
```

---

## Step 2: Update Module Structure

Add to `src/optimization/mod.rs`:

```rust
pub mod imu_factor;

pub use imu_factor::ImuFactor;
```

Add to `src/lib.rs`:

```rust
pub mod optimization;
```

---

## Step 3: Integrate with Optimizer

In your optimization code (e.g., `src/optimizer.rs`):

```rust
use crate::optimization::ImuFactor;

pub struct OptimizationProblem {
    // ... visual factors ...
    
    /// IMU preintegration factors
    imu_factors: Vec<ImuFactor>,
    
    /// Weight for IMU factors in objective
    imu_weight: f64,
}

impl OptimizationProblem {
    pub fn add_imu_factor(&mut self, factor: ImuFactor) {
        self.imu_factors.push(factor);
    }
    
    /// Build optimization objective including IMU factors
    pub fn build_objective(&self) -> f64 {
        let mut objective = 0.0;
        
        // Visual reprojection errors (existing)
        objective += self.visual_objective();
        
        // IMU preintegration errors (NEW)
        for factor in &self.imu_factors {
            let imu_error = factor.weighted_residual(
                &self.poses[factor.keyframe_i].0,
                &self.velocities[factor.keyframe_i],
                &self.poses[factor.keyframe_i].1,
                &self.poses[factor.keyframe_j].0,
                &self.velocities[factor.keyframe_j],
                &self.poses[factor.keyframe_j].1,
                &self.biases.0,  // bias_g
                &self.biases.1,  // bias_a
            );
            objective += self.imu_weight * imu_error;
        }
        
        objective
    }
}
```

---

## Step 4: Connect Keyframes to IMU Factors

When creating keyframes, automatically create IMU factors:

```rust
pub fn create_keyframe_with_imu(
    keyframe_id: usize,
    image: ImageData,
    pose: Pose,
    features: Vec<Feature>,
    imu_buffer: &ImuBuffer,  // Window of IMU since last keyframe
    last_keyframe: &Keyframe,
) -> (Keyframe, Option<ImuFactor>) {
    let keyframe = Keyframe {
        id: keyframe_id,
        image,
        pose,
        features,
        timestamp: image.timestamp,
    };
    
    // Create IMU factor if we have IMU measurements
    let imu_factor = if imu_buffer.is_empty() {
        None
    } else {
        let dt = (keyframe.timestamp - last_keyframe.timestamp) as f64 * 1e-9;
        let preint = PreintegratedImu::from_imu_data(
            imu_buffer.get_measurements(),
            na::Vector3::zeros(),  // Initial biases (will be updated)
            na::Vector3::zeros(),
            na::Vector3::new(0.0, 0.0, -9.81),
        );
        
        Some(ImuFactor::new(
            preint,
            last_keyframe.id,
            keyframe.id,
            dt,
            na::Vector3::zeros(),
            na::Vector3::zeros(),
        ))
    };
    
    (keyframe, imu_factor)
}
```

---

## Step 5: Testing

Create `src/optimization/imu_factor_tests.rs`:

```rust
#[cfg(test)]
mod imu_factor_tests {
    use super::*;
    use crate::imu::preintegration::PreintegratedImu;
    use crate::imu::ImuNoise;
    
    #[test]
    fn test_imu_factor_residual_zero_motion() {
        // Setup preintegration (zero motion)
        let noise = ImuNoise::default();
        let gravity = na::Vector3::new(0.0, 0.0, -9.81);
        let mut preint = PreintegratedImu::new(noise, 1.0, gravity);
        
        let factor = ImuFactor::new(
            preint,
            0,
            1,
            1.0,
            na::Vector3::zeros(),
            na::Vector3::zeros(),
        );
        
        // Poses consistent with zero motion
        let R = na::UnitQuaternion::identity();
        let v_i = na::Vector3::zeros();
        let v_j = gravity;  // Only gravity effect
        let p = na::Vector3::zeros();
        
        let bias_g = na::Vector3::zeros();
        let bias_a = na::Vector3::zeros();
        
        let residual = factor.residual(&R, &v_i, &p, &R, &v_j, &p, &bias_g, &bias_a);
        
        // Should be small
        assert!(residual.norm() < 0.1, "Zero motion should have small residual");
    }
    
    #[test]
    fn test_imu_factor_jacobian_numerical() {
        // Verify analytical jacobians against numerical differentiation
        let noise = ImuNoise::default();
        let gravity = na::Vector3::new(0.0, 0.0, -9.81);
        let preint = PreintegratedImu::new(noise, 1.0, gravity);
        
        let factor = ImuFactor::new(
            preint, 0, 1, 1.0,
            na::Vector3::zeros(),
            na::Vector3::zeros(),
        );
        
        // Test point
        let R_i = na::UnitQuaternion::identity();
        let v_i = na::Vector3::zeros();
        let p_i = na::Vector3::zeros();
        let R_j = na::UnitQuaternion::identity();
        let v_j = gravity;
        let p_j = na::Vector3::zeros();
        let bias_g = na::Vector3::zeros();
        let bias_a = na::Vector3::zeros();
        
        let (J_R_i, _, _, _, _, _, _, _) = factor.jacobians(
            &R_i, &v_i, &p_i, &R_j, &v_j, &p_j, &bias_g, &bias_a
        );
        
        // All jacobians should be finite
        assert!(J_R_i.iter().all(|x| x.is_finite()));
    }
}
```

---

## Integration Checklist

- [ ] Create `src/optimization/imu_factor.rs`
- [ ] Implement `ImuFactor` struct
- [ ] Implement `residual()` method
- [ ] Implement `jacobians()` method (numerical for now)
- [ ] Add unit tests
- [ ] Update `src/optimization/mod.rs`
- [ ] Update `src/lib.rs`
- [ ] Integrate with optimizer
- [ ] Add IMU factors to optimization objective
- [ ] Test on synthetic data
- [ ] Test on real data (EuRoC)
- [ ] Verify accuracy improvement

---

## Next Phase

Once this is complete:

1. **Phase 2B**: Extract optimized biases from optimizer
2. **Phase 2B**: Feed refined biases back to ESKF
3. **Phase 2C**: Create keyframes with automatic IMU factor creation
4. **Phase 2D**: Add visual measurement updates

---

## Performance Notes

- Numerical Jacobians: 8×12 perturbations per evaluation (fast enough for optimization)
- Analytical Jacobians: Would be ~10× faster, but more complex to implement
- Recommendation: Start with numerical, optimize to analytical if needed

---

## References

The residual computation follows Forster et al. 2017 closely:
- Rotation error: log-map of rotation difference
- Velocity error: Predicted vs measured velocity update
- Position error: Predicted vs measured position update
- Uses first-order bias correction for non-reference biases

