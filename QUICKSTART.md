# Quick Start: Running RS-VIO with Real Datasets

## Prerequisites

All three datasets require download (registration links below):
- **EuRoC**: https://projects.asl.ethz.ch/datasets/euroc-mav/ (ETH account)
- **TUM-VI**: https://vision.in.tum.de/data/datasets/visual-inertial-slam (Free)
- **4Seasons**: https://www.4seasons-dataset.com/ (Free registration)

## Setup (One-Time)

### EuRoC Dataset
```bash
# 1. Download MH_01_easy.zip from ETH website
# 2. Extract it:
mkdir -p /tmp/rs-vio-samples/euroc
unzip MH_01_easy.zip -d /tmp/rs-vio-samples/euroc/

# 3. Verify structure:
ls /tmp/rs-vio-samples/euroc/MH_01_easy/mav0/
# Should show: cam0/ cam1/ imu0/
```

### TUM-VI Dataset
```bash
# Option A: Auto-download (if mirror available)
just download-datasets

# Option B: Manual download from https://cvg.cit.tum.de/data/datasets/visual-inertial-dataset
# 1. Download dataset-room1_512_16.tar (512x512, EuRoC format)
# 2. Extract it:
mkdir -p /tmp/rs-vio-samples/tum_vi/room1
tar -xf dataset-room1_512_16.tar -C /tmp/rs-vio-samples/tum_vi/room1
```

### 4Seasons Dataset
```bash
# 1. Download recording_2021-01-07_13-03-56.zip from 4seasons website
# 2. Extract it:
mkdir -p /tmp/rs-vio-samples/4seasons
unzip recording_2021-01-07_13-03-56.zip -d /tmp/rs-vio-samples/4seasons/

# 3. Verify structure:
ls /tmp/rs-vio-samples/4seasons/recording_2021-01-07_13-03-56/
# Should show: undistorted_images/ times.txt
```

## Run the Estimators

```bash
# Build in release mode (if not already done)
cargo build --release

# Run EuRoC (stereo VIO with IMU)
cargo run --release --bin run_euroc config/euroc_vio.yaml /tmp/rs-vio-samples/euroc/MH_01_easy

# Run TUM-VI (stereo with IMU)
cargo run --release --bin run_tum config/tum_vi.yaml /tmp/rs-vio-samples/tum_vi/room1

# Run 4Seasons (stereo VIO, no IMU)
cargo run --release --bin run_4seasons config/4seasons.yaml /tmp/rs-vio-samples/4seasons/recording_2021-01-07_13-03-56
```

## Using just Targets

Once datasets are in place:

```bash
just run-euroc
just run-tum
just run-4seasons
```

## Expected Output

You should see:
```
[2026-01-10T...] [INFO] [EurocPlayer] Loaded 3682 image timestamps
[2026-01-10T...] [INFO] [Estimator] Tracking frames...
[2026-01-10T...] [INFO] [EurocPlayer] Average processing time: 45.23ms
[2026-01-10T...] [INFO] [Main] processing completed successfully!
```

Statistics saved to `{dataset_path}/statistics.txt`

## Troubleshooting

**Dataset not found error**: Verify folder structure matches above
**Out of disk space**: Need ~15 GB total for all datasets
**Slow processing**: Normal - VIO is computationally intensive

## See Also

- [DATASETS.md](DATASETS.md) - Detailed dataset guide
- [README.md](README.md) - Project documentation
