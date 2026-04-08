# VIO vs VO Comparison — TUM-VI corridor4

## Methodology

Ran the full pipeline on corridor4 (1,927 stereo frames, ~96s real-time) in two modes:
- **VO**: `--no-imu` flag — uses constant velocity model (extrapolate from last 2 keyframes)
- **VIO**: default — uses IMU pre-integration (Rodrigues' rotation + gravity-compensated integration)

The initial reprojection cost before PnP optimization measures prediction quality.
Lower = better initial pose estimate.

## Results

| Metric | VO (no IMU) | VIO (with IMU) | Improvement |
|--------|------------|----------------|-------------|
| Avg initial cost | 7.2458 | **1.9153** | **3.78× lower** |
| Max initial cost | 100.244 | **20.971** | **4.78× lower** |
| Min initial cost | 0.001 | 0.001 | equal |
| Frames tracked | 1,917 | 1,917 | equal |
| Tracking failures | 0 | 0 | equal |
| Processing speed | 40.4 fps | 38.9 fps | 3.7% overhead |
| IMU samples | 0 | 19,218 @ 200Hz | — |

## Interpretation

### Average cost: 3.78× improvement
The IMU pre-integration produces a pose estimate that is consistently much closer
to the optimal solution. Over 1,917 frames, the average reprojection error is
reduced from 7.25 → 1.92.

### Peak cost: 4.78× improvement (most important)
VO suffers large prediction errors during non-linear motion (turns, acceleration)
because it assumes constant velocity. The max cost of 100.24 means the initial
guess was far from optimal in some frames. VIO's max of 20.97 stays bounded
because IMU data captures actual rotational and translational acceleration.

### Tracking reliability: Equal (0 failures)
Both modes converge successfully on every frame. The PnP solver is robust enough
to recover even from poor initial guesses, but starting closer to the solution
means fewer LM iterations and more stable behavior.

### Speed: 3.7% overhead
VIO processes at 38.9 fps vs VO's 40.4 fps. The IMU pre-integration adds ~1.5 ms
per frame (parsing 200Hz samples between frames). This is negligible compared to
the quality improvement.

## Commands

```bash
# VO mode (no IMU)
cargo run --release --bin run_tum -- config/tum_vi.yaml data/tum-vi/dataset-corridor4_512_16 --no-imu

# VIO mode (with IMU, default)
cargo run --release --bin run_tum -- config/tum_vi.yaml data/tum-vi/dataset-corridor4_512_16
```
