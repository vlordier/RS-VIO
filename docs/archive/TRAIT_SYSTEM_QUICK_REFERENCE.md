# RS-VIO Unified Trait System - Quick Reference

## Overview
The RS-VIO trait system has been consolidated into 7 core traits in `src/traits.rs`. This provides a unified, maintainable interface for all major functionality.

---

## Core Traits

### 1. Convert<T> - Type Conversions
**When to use**: Converting between nalgebra and array types

```rust
use rs_vio::traits::Convert;

// Array → Matrix/Vector
let matrix: Matrix4x4 = array.convert();
let vector: Vector3 = array.convert();

// Matrix/Vector → Array
let array: Array4x4 = matrix.convert();
let array: Array3 = vector.convert();
```

**Implementations**: Array4x4 ↔ Matrix4x4, Array3x3 ↔ Matrix3x3, Array3 ↔ Vector3, Array2 ↔ Vector2

---

### 2. Strategy - Algorithm Pattern
**When to use**: Implementing swappable algorithm implementations

```rust
use rs_vio::traits::Strategy;

#[derive(Debug)]
pub struct MyAlgorithm;

impl Strategy for MyAlgorithm {
    fn name(&self) -> &str { "MyAlgorithm v1.0" }
    fn description(&self) -> &str { "Brief description" }
    fn is_available(&self) -> bool { true }
}

// Used as trait object
let algo: Box<dyn Strategy> = Box::new(MyAlgorithm);
println!("{}", algo.name());
```

**Key**: All implementations MUST be Send + Sync + Debug + 'static

**Extends**: DescriptorMatcher, GeometricVerifier, and all algorithm implementations

---

### 3. ResourcePool - Unified Pooling
**When to use**: Pooling any reusable resource (workspaces, buffers, descriptors)

```rust
use rs_vio::traits::ResourcePool;

// Using an existing pool
let workspace = workspace_pool.acquire();
do_work(&workspace);
workspace_pool.release(workspace);

// Checking metrics
println!("Utilization: {:.1}%", workspace_pool.utilization() * 100.0);
println!("Available: {}", workspace_pool.available());
println!("Capacity: {}", workspace_pool.capacity());
```

**Implementations**:
- WorkspacePool (FrameWorkspace buffers)
- OrbBinaryPool (u8 descriptor buffers)
- FloatDescriptorPool (Float descriptor buffers)
- HybridDescriptorPool (Binary or Float with fallback)

**Interface**:
```rust
pub trait ResourcePool {
    type Resource: Send + Sync;
    type Config: Clone + Debug;
    
    fn new(config: Self::Config) -> Self;
    fn acquire(&self) -> Self::Resource;
    fn try_acquire(&self) -> Option<Self::Resource>;
    fn release(&self, resource: Self::Resource);
    fn reset(&self);
    fn utilization(&self) -> f32;   // 0.0 - 1.0
    fn capacity(&self) -> usize;
    fn available(&self) -> usize;
}
```

---

### 4. StateView - Immutable State Access
**When to use**: Functions that only need to READ state

```rust
use rs_vio::traits::StateView;

fn process_state<S: StateView>(state: &S) {
    let pose = state.pose();           // Returns &Matrix4x4
    let velocity = state.velocity();    // Returns &Vector3
    let translation = state.translation();  // Returns Vector3 (computed)
    let rotation = state.rotation();     // Returns UnitQuaternion (computed)
}

// Called with State struct
let state = State::identity();
process_state(&state);
```

**Methods**:
- `pose(&self) -> &Matrix4x4`
- `velocity(&self) -> &Vector3`
- `accel_bias(&self) -> &Vector3`
- `gyro_bias(&self) -> &Vector3`
- `camera_left_extrinsics(&self) -> &Matrix4x4`
- `camera_right_extrinsics(&self) -> &Matrix4x4`
- `translation(&self) -> Vector3` (computed)
- `rotation(&self) -> UnitQuaternion` (computed)

**Benefit**: References prevent copies, clear read-only intent

---

### 5. StateTransform - State Mutations
**When to use**: Functions that COMPUTE NEW STATES

```rust
use rs_vio::traits::StateTransform;

fn transform_states<S: StateTransform>(state1: &S, state2: &S) -> S
where
    S: Sized,
{
    let composed = state1.compose(state2);
    let inverse = state1.inverse();
    let interpolated = state1.interpolate(state2, 0.5);
    interpolated
}
```

**Methods**:
- `compose(&self, other: &Self) -> Self` - Combine two states
- `inverse(&self) -> Self` - Invert all transformations
- `interpolate(&self, other: &Self, alpha: Float) -> Self` - Linear interpolation

**Note**: Extends StateView, so all read methods are available

---

### 6. Validate - Numerical Validation
**When to use**: Checking numerical validity and safety

```rust
use rs_vio::traits::Validate;

fn check_numbers<T: Validate>(value: &T) -> Result<(), ValidationError> {
    value.validate()
}

// Composite validation
let validators: [&dyn Validate; 2] = [&matrix, &vector];
T::validate_all(&validators)?;
```

**Used by**: MatrixView, VectorView for finite/NaN checks

---

### 7. CloneStrategy - Trait Object Cloning
**When to use**: Cloning boxed Strategy trait objects

```rust
use rs_vio::traits::CloneStrategy;

let algo: Box<dyn Strategy> = Box::new(MyAlgorithm);
let cloned: Box<dyn Strategy> = algo.clone_strategy();
```

---

## Migration Guide

### From StateOperations → StateView/StateTransform

**Before (Phase 2)**:
```rust
fn process<S: StateOperations>(state: &S) {
    let pose = state.pose();         // Returns owned copy
    state.compose(&other);
}
```

**After (Phase 3+)**:
```rust
// For read-only operations
fn read_state<S: StateView>(state: &S) {
    let pose_ref = state.pose();     // Returns &Matrix4x4
    let translation = state.translation(); // Computed
}

// For transformation operations
fn transform_state<S: StateTransform>(state: &S) -> S
where
    S: Sized,
{
    state.compose(&other)
}
```

**Backward Compatibility**: State struct has delegation methods for gradual migration:
- `state.pose()` - still works (returns owned copy)
- `state.velocity_vec()` - still works
- `state.accel_bias_vec()` - still works
- `state.gyro_bias_vec()` - still works
- `state.T_B_Cl()` - still works
- `state.T_B_Cr()` - still works
- `state.inverse_pose()` - still works

### From ToMatrix/ToVector/ToArray → Convert<T>

**Before (Phase 3)**:
```rust
let matrix = array.to_matrix();
let vector = array.to_vector();
let array = matrix.to_array();
```

**After (Phase 4)**:
```rust
use rs_vio::traits::Convert;

let matrix: Matrix4x4 = array.convert();
let vector: Vector3 = array.convert();
let array: Array4x4 = matrix.convert();
```

---

## Common Patterns

### Checking State Is Valid
```rust
use rs_vio::traits::StateView;

fn validate_state<S: StateView>(state: &S) -> bool {
    state.pose().is_finite() && 
    state.velocity().norm() < 50.0  // reasonable velocity check
}
```

### Creating State Variants
```rust
use rs_vio::traits::StateTransform;

fn interpolate_trajectory<S: StateTransform>(
    states: &[S],
    t: Float,
) -> S
where
    S: Sized,
{
    // Simple linear interpolation
    if states.len() < 2 {
        return states[0].clone();
    }
    
    states[0].interpolate(&states[1], t)
}
```

### Working with Pools
```rust
use rs_vio::traits::ResourcePool;

fn process_with_pool<P: ResourcePool>(pool: &P, data: &[u8]) {
    let mut buffer = pool.acquire();
    process_data(&mut buffer, data);
    pool.release(buffer);
    
    // Check pool health
    if pool.utilization() > 0.9 {
        println!("Warning: pool nearly exhausted");
    }
}
```

### Algorithm Selection
```rust
use rs_vio::traits::Strategy;

fn select_matcher(
    preferred: Box<dyn Strategy>,
    fallback: Box<dyn Strategy>,
) -> Box<dyn Strategy> {
    if preferred.is_available() {
        preferred
    } else {
        fallback
    }
}
```

---

## Testing Traits

### Testing StateView
```rust
#[test]
fn test_state_view() {
    let state = State::identity();
    
    // Can accept any StateView impl
    fn check<S: StateView>(s: &S) {
        let _pose = s.pose();
        let _vel = s.velocity();
    }
    
    check(&state);
}
```

### Testing Convert<T>
```rust
#[test]
fn test_conversions() {
    use rs_vio::traits::Convert;
    
    let array = [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ];
    
    let matrix: Matrix4x4 = array.convert();
    let back: Array4x4 = matrix.convert();
    
    assert_eq!(array, back);
}
```

---

## File Organization

```
src/
├── traits.rs (318 lines)
│   ├── Convert<T> (generic)
│   ├── Strategy (base pattern)
│   ├── ResourcePool (generic)
│   ├── Validate (chainable)
│   ├── StateView (read-only)
│   ├── StateTransform (mutations)
│   └── CloneStrategy (trait objects)
│
├── types.rs (343 lines)
│   └── Convert<T> implementations (8 types)
│
├── estimator/state.rs (165 lines)
│   ├── State struct
│   ├── StateView impl
│   ├── StateTransform impl
│   └── Backward compat delegation methods
│
└── [other modules using traits]
    ├── optimization/loop_closure/ (Strategy impls)
    ├── optimization/tight_coupling/ (State usage)
    └── viewers/rerun.rs (Convert<T> usage)
```

---

## Summary Table

| Trait | Purpose | Use When | Returns |
|-------|---------|----------|---------|
| Convert<T> | Type conversion | Converting array ↔ matrix/vector | T |
| Strategy | Algorithm pattern | Implementing pluggable algorithms | &str (name) |
| ResourcePool | Resource pooling | Managing reusable buffers | Resource |
| Validate | Numerical checks | Checking for NaN/infinite values | Result |
| StateView | Read state | Functions that read state only | References |
| StateTransform | Mutate state | Functions that compute new states | Self |
| CloneStrategy | Clone trait objects | Cloning boxed Strategy | Box<dyn Strategy> |

---

## Quick Help

**Q: How do I convert Array4x4 to Matrix4x4?**  
A: Use `array.convert()` (requires `use rs_vio::traits::Convert;`)

**Q: How do I get a reference to pose instead of a copy?**  
A: Use `StateView` trait bound: `fn f<S: StateView>(s: &S) { s.pose() }`

**Q: How do I interpolate between two states?**  
A: Use `StateTransform` trait: `state1.interpolate(&state2, 0.5)`

**Q: How do I swap pool implementations?**  
A: Use `ResourcePool` trait as generic bound, any impl works

**Q: Where are the old ToMatrix/ToVector traits?**  
A: Removed in Phase 4, use `Convert<T>` instead

**Q: Is StateOperations still available?**  
A: No (removed in Phase 3), use `StateView` or `StateTransform` instead

---

## Links to Detailed Documentation

- **PHASE_2_TRAIT_REFACTORING_COMPLETE.md** - Core trait design
- **PHASE_3_INTERNAL_MIGRATION_COMPLETE.md** - StateView/StateTransform split
- **PHASE_4_TRAIT_CLEANUP_COMPLETE.md** - Conversion trait consolidation
- **SESSION_SUMMARY_TRAIT_REFACTORING.md** - Full session summary

---

**Last Updated**: 17 January 2026  
**Status**: Complete and Production Ready ✅
