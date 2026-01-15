//! # Feature Tracker Benchmarks - TEMPORARILY DISABLED
//!
//! The FeatureTracker API has changed. Benchmarks need to be updated
//! to use StereoPatchTracker or PatchTracker instead.

#![allow(warnings)]

use criterion::{criterion_group, criterion_main, Criterion};

// All benchmarks disabled - API has changed
fn placeholder_bench(_c: &mut Criterion) {}

criterion_group!(benches, placeholder_bench);
criterion_main!(benches);
