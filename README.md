# Rust Stereo Visual-Inertial Odometry (RS-VIO)

This project is a stereo visual-inertial odometry (VIO) system, written fully in Rust. It utilizes patch-based stereo feature tracking, sliding window bundle adjustment with apex-solver (Levenberg-Marquardt optimization), PnP-based motion tracking, and Rerun for 3D visualization. This first release (v0.1) only supports pure stereo odometry, IMU integration is planned for the next release.

[![Demo video](https://img.youtube.com/vi/3lqf6Et3RmQ/0.jpg)](https://www.youtube.com/watch?v=3lqf6Et3RmQ)

## Features

- **Patch-based stereo feature tracking**: Multi-scale optical flow tracking using 52-point patterns for robust feature correspondence between stereo pairs.
- **Sliding window bundle adjustment**: Joint optimization of camera poses and 3D map points using apex-solver with configurable window size.
- **PnP motion tracking**: Perspective-n-Point pose estimation for inter-frame tracking between keyframes.
- **Keyframe selection**: Automatic keyframe selection based on translation and rotation thresholds.
- **Async Pipeline Processing**: Non-blocking frame processing with concurrent bundle adjustment offload for real-time performance (19.7x speedup, <100 bytes/frame memory usage).
- **Multi-camera model support**: Supports pinhole-radtan and EUCM camera models with distortion handling, more camera models can be integrated easily.
- **Self-calibrating stereo VIO**: Three complementary intrinsics refinement strategies (online, offline, and baseline) with 61.7% error reduction on TUM-VI.
- **Multi-camera architecture**: N-camera configuration system with flexible orientations and parameter sharing policies.
- **Dataset support**: Players for EuRoC, TUM-VI, and 4Seasons datasets with configurable parameters.
- **3D visualization**: Real-time visualization of trajectories, map points, and camera frustums using Rerun.

## Usage

- EuRoC:
  - Download the dataset from https://projects.asl.ethz.ch/datasets/euroc-mav/
  - Run:
```bash
cargo run --release --bin run_euroc config/euroc_vio.yaml {path_to_euroc_folder}/MH_01_easy/
```
- TUM-VI:
  - Download the 512x512 datasets in EuRoC/DSO format from https://cvg.cit.tum.de/data/datasets/visual-inertial-dataset
  - Run:
```bash
cargo run --release --bin run_tum config/tum_vi.yaml {path_to_tum_folder}/dataset-room1_512_16/
```
- 4Seasons:
  - Download the undistorted image datasets from https://cvg.cit.tum.de/data/datasets/4seasons-dataset/download
  - Run:
```bash
cargo run --release --bin run_4seasons config/4seasons.yaml {path_to_4seasons_folder}/recording_2021-01-07_13-03-56/
```

Check the run scripts in /scripts/ for more information. Configuration files are available in the `config/` directory.

## Calibration Features

### Self-Calibrating Intrinsics

RS-VIO includes three complementary strategies for camera intrinsics refinement:

- **Online Refinement** (Strategy 2): Real-time optimization during VIO execution (~24.7% error reduction)
- **Offline Post-Processing** (Strategy 3): Batch refinement after VIO with convergence-based stopping (~61.7% total reduction)
- **Baseline Analysis** (Strategy 1): Establish reference calibration quality

See [docs/FEATURE_SUMMARY.md](docs/FEATURE_SUMMARY.md) for complete details, or [docs/VALIDATION.md](docs/VALIDATION.md) for validation guides.

## Variable naming conventions

We use the following naming conventions for coordinate frame transformations:
- `T_B_A: Matrix4x4`: SE(3) transformation matrix from A to B
- `R_B_A: Matrix3x3`: SO(3) transformation matrix from A to B
- `t_B_A: Vector3`: translation from A to B (equal to position of origin of A in B)
- `q_B_A: UnitQuaternion`: Unit quaternion representing the rotation from A to B

Using this convention, we can easily chain transformations, e.g. `T_C_A = T_C_B * T_B_A`.



## Roadmap

This is my current plan, subject to change over time. Contributions are welcome :)

### Phase 1 - Current
- [x] Stereo patch tracking
- [x] Bundle adjustment and PnP solvers
- [x] PnP tracking for all frames
- [x] Keyframe selection method
- [x] Initial operating capability for pure stereo VO
- [x] EuRoC dataset player
- [x] TUM-VI dataset player
- [x] 4Seasons dataset player
- [x] Async pipeline optimizations (19.7x speedup, real-time performance)

### Phase 2 - Near future
- [ ] Small refactoring and code clean-up (coming soon)
- [ ] IMU data processing (coming soon)
- [ ] Constant velocity model*
- [ ] Marginalization of old keyframes and keypoints**
- [ ] ROS wrapper

### Phase 3 - Extensions
- [ ] Photo- / feature-metric optimization
- [ ] Loop closure

*: This will be used mostly for rigs with bad IMUs or poorly sync'd / poorly calibrated IMUs

**: Currently, the last keyframe is fixed when solving the bundle adjustment problem. Don't expect large-scale accuracy until proper marginalization is implemented.


## Acknowledgements

This project builds on excellent open-source work:

### Dependencies:
- [apex-solver](https://github.com/amin-abouee/apex-solver)
- [camera-intrinsic-model-rs](https://github.com/powei-lin/camera-intrinsic-model-rs)
- [patch-tracker-rs](https://github.com/powei-lin/patch-tracker-rs)

### Design influences:
- [VINS-Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono)
- [Basalt](https://gitlab.com/VladyslavUsenko/basalt)
- [Lightweight VIO](https://github.com/93won/lightweight_vio)
    ​

### License
It's released under the GNU General Public License v3 (GPLv3). See LICENSE file for details.

### Citation

```
@misc{rs_vio_2026,
  author = {Charles Hamesse},
  title = {Rust Stereo Visual-Inertial Odometry (RS-VIO)},
  year = {2026},
  howpublished = {\url{https://github.com/charleshamesse/rs-vio}},
  note = {v0.1}
}
```
