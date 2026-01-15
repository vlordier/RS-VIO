# Dataset Setup Guide

This guide explains how to download and set up the datasets supported by RS-VIO.

## Quick Start

```bash
# Auto-downloads TUM-VI (if mirror is available)
make download-datasets

# Manually place EuRoC and 4Seasons data, then run
make run-euroc
make run-tum
make run-4seasons
```

## EuRoC MAV Dataset

**Status**: Requires registration & manual download  
**Size**: ~2-3 GB per sequence  
**Sequences**: MH_01_easy, MH_02_easy, MH_03_medium, V1_01_easy, V2_01_easy, etc.

### Download Instructions

1. Visit https://projects.asl.ethz.ch/datasets/euroc-mav/
2. Register for free account
3. Download `MH_01_easy.zip` (or any other sequence)
4. Extract to `/tmp/rs-vio-samples/euroc/`:
   ```bash
   mkdir -p /tmp/rs-vio-samples/euroc
   unzip MH_01_easy.zip -d /tmp/rs-vio-samples/euroc/
   ```

### Run

```bash
cargo run --release --bin run_euroc config/euroc_vio.yaml /tmp/rs-vio-samples/euroc/MH_01_easy
```

Expected output:
- Loading 3682 stereo image pairs
- Real-time feature tracking and pose estimation
- Statistics saved to `statistics.txt`

## TUM-VI Dataset

**Status**: Auto-downloads (if mirror available), otherwise manual setup  
**Size**: ~2 GB per sequence  
**Sequences**: room1, room2, room3, room4, room5, room6

### Auto-Download

```bash
make download-datasets
```

This will download and extract the TUM RGB-D walking sequence to `/tmp/rs-vio-samples/tum_vi/`.

### Manual Download

If auto-download fails:

1. Visit https://vision.in.tum.de/data/datasets/visual-inertial-slam
2. Download the RGB-D sequence (e.g., `rgbd-dataset_freiburg3_walking_xyz.tgz`)
3. Extract to `/tmp/rs-vio-samples/tum_vi/`:
   ```bash
   mkdir -p /tmp/rs-vio-samples/tum_vi
   tar -xzf rgbd-dataset_freiburg3_walking_xyz.tgz -C /tmp/rs-vio-samples/tum_vi/ --strip-components=1
   ```

### Run

```bash
cargo run --release --bin run_tum config/tum_vi.yaml /tmp/rs-vio-samples/tum_vi
```

## 4Seasons Dataset

**Status**: Requires free registration & manual download  
**Size**: ~5 GB per recording  
**Recordings**: 4 seasonal recordings with 600+ minutes of video

### Download Instructions

1. Visit https://www.4seasons-dataset.com/
2. Register for free account
3. Download one or more recording ZIPs (e.g., `recording_2021-01-07_13-03-56.zip`)
4. Extract to `/tmp/rs-vio-samples/4seasons/`:
   ```bash
   mkdir -p /tmp/rs-vio-samples/4seasons
   unzip recording_2021-01-07_13-03-56.zip -d /tmp/rs-vio-samples/4seasons/
   ```

### Run

```bash
cargo run --release --bin run_4seasons config/4seasons.yaml /tmp/rs-vio-samples/4seasons/recording_2021-01-07_13-03-56
```

## Docker with Datasets

Mount datasets as volumes:

```bash
# Build image
docker build -t rs-vio .

# Run with EuRoC
docker run --rm \
  -v /tmp/rs-vio-samples:/data:ro \
  rs-vio:latest \
  /usr/local/bin/run_euroc config/euroc_vio.yaml /data/euroc/MH_01_easy

# Run with TUM-VI
docker run --rm \
  -v /tmp/rs-vio-samples:/data:ro \
  --entrypoint /usr/local/bin/run_tum \
  rs-vio:latest \
  config/tum_vi.yaml /data/tum_vi

# Run with 4Seasons
docker run --rm \
  -v /tmp/rs-vio-samples:/data:ro \
  --entrypoint /usr/local/bin/run_4seasons \
  rs-vio:latest \
  config/4seasons.yaml /data/4seasons/recording_2021-01-07_13-03-56
```

## Dataset Compatibility

| Dataset | Format | Cameras | IMU | Distortion Model | Status |
|---------|--------|---------|-----|------------------|--------|
| EuRoC | mav0/ CSV | Stereo | Yes | Radtan | ✓ Supported |
| TUM-VI | RGB-D + CSV | Mono/Stereo | Yes | Various | ✓ Supported |
| 4Seasons | undistorted_images/ | Stereo | No | None | ✓ Supported |

## Troubleshooting

### "Dataset not found" error

```bash
# Verify dataset structure
ls -la /tmp/rs-vio-samples/euroc/MH_01_easy/mav0/
# Should show: cam0/, cam1/, imu0/ directories

ls -la /tmp/rs-vio-samples/4seasons/recording_2021-01-07_13-03-56/
# Should show: undistorted_images/, times.txt files
```

### Slow download speed

- Try different mirror (if available)
- Download in off-peak hours
- Check network connectivity

### Disk space issues

- EuRoC: ~2-3 GB per sequence
- TUM-VI: ~2 GB per sequence
- 4Seasons: ~5 GB per recording
- Total for all test: ~15 GB

Free up space before downloading large sequences.

## Citation

If using these datasets, please cite:

**EuRoC MAV Dataset**:
```
@article{burri2016euroc,
  title={The {EuRoC} micro aerial vehicle datasets},
  author={Burri, Michael and others},
  journal={IJRR},
  year={2016}
}
```

**TUM RGB-D Dataset**:
```
@article{sturm2012benchmark,
  title={A benchmark for the evaluation of {RGB-D} SLAM systems},
  author={Sturm, J. and others},
  journal={IROS},
  year={2012}
}
```

**4Seasons Dataset**:
```
@article{buchner20224seasons,
  title={The {4Seasons} dataset},
  author={B\"uchner, V. and others},
  journal={IJRR},
  year={2022}
}
```
