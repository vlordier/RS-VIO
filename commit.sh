#!/bin/bash
git add .
git commit -m "feat(loop-closure): add RANSAC verifier and Hamming matcher

- RansacEpipolarVerifier with RANSAC loop (1000 iter default)
- HammingMatcher for binary descriptors
- Criterion benchmark suite for matchers/verifiers
- 19 tests passing, 170 total library tests"
