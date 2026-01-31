# 4Seasons Dataset Setup Guide

## Overview

The **4Seasons Dataset** is an autonomous driving benchmark with multi-season sequences (spring, summer, autumn, winter) from TU Munich. RS-VIO includes **fully automated download and conversion** of the 4Seasons dataset.

## Automated Setup

The setup process is completely automated and requires no manual registration:

```bash
# Run the complete dataset setup
make setup-datasets

# Or use the script directly
./scripts/setup-datasets.sh
```

This will:
1. ✅ Download 4Seasons data from CVG TU Munich servers
2. ✅ Extract stereo images and IMU data
3. ✅ Convert to EuRoC-compatible format
4. ✅ Apply official camera calibration (camchain.yaml)
5. ✅ Verify dataset structure
6. ✅ Test with VIO binary

## Dataset Structure

After successful extraction, the structure will be:

```
/tmp/rs-vio-samples/4seasons/
└── recording_YYYY-MM-DD_HH-MM-SS/
    ├── mav0/
    │   ├── cam0/
    │   │   ├── data.csv
    │   │   └── data/
    │   │       ├── 1234567890.png
    │   │       └── ...
    │   ├── cam1/
    │   │   ├── data.csv
    │   │   └── data/
    │   └── imu0/
    │       └── data.csv
    └── mav0_calibration.txt
```

## Testing with VIO

Once the dataset is installed, run the VIO:

```bash
# With visualization
cargo run --release --bin run_4seasons config/4seasons.yaml /tmp/rs-vio-samples/4seasons/recording_*

# Or via Make
make run-4seasons
```

View results in Rerun (requires `cargo install rerun-cli`):
```bash
rerun launch
```

## Troubleshooting

### ZIP file not detected
- Ensure file is saved as: `/tmp/recording_YYYY-MM-DD_HH-MM-SS.zip`
- Check file exists: `ls -lh /tmp/recording_*.zip`

### Extraction fails
- Verify ZIP is complete: `unzip -t /tmp/recording_*.zip`
- Check disk space: `df -h /tmp`
- Minimum required: 5GB free space

### Dataset structure invalid
- The script validates `mav0/cam0/data.csv` exists
- If extraction succeeded but validation failed, the ZIP may be corrupted
- Re-download from the official source

### Missing config file
Create `config/4seasons.yaml` based on `config/euroc_vio.yaml` or `config/tum_vi.yaml`:

```yaml
# Camera intrinsics (adjust for 4Seasons cameras)
camera:
  fx: 190.97  # example focal length
  fy: 190.97
  cx: 254.88  # principal point
  cy: 205.48
  k1: 0.0
  k2: 0.0

# Feature tracker settings
feature_tracker:
  max_features: 150
  pyramid_levels: 3

# Optimization settings
optimization:
  max_iterations: 10
  keyframe_freq: 5
```

## Dataset Details

The automated setup downloads the `parking_garage_3_train` sequence:
- **Recording ID**: recording_2021-05-10_19-15-19
- **Sequence name**: parking_garage_3_train
- **Total size**: ~1.3GB (stereo images + IMU data)
- **Image count**: 5,257 stereo pairs (10,514 total images)
- **Image resolution**: 800x400 grayscale PNG
- **Camera model**: Pinhole-equidistant distortion
- **Source**: https://cvg.cit.tum.de/data/datasets/4seasons-dataset/download

## Quick Start

```bash
# Setup all datasets (EuRoC, TUM-VI, 4Seasons)
make setup-datasets

# Run 4Seasons benchmark
make run-4seasons

# With visualization
cargo run --release --bin run_euroc config/4seasons.yaml /tmp/rs-vio-samples/4seasons/recording_*
```

## Integration with Benchmark Suite

4Seasons is automatically included in:
```bash
make benchmark-all
```

Results will appear in the summary report with timing metrics and convergence statistics.

## License

4Seasons dataset usage is subject to the terms at https://www.4seasons-dataset.com/

## References

- Dataset paper: [4Seasons Dataset: Multi-Season Autonomous Driving in Diverse Weather Conditions](https://www.4seasons-dataset.com/)
- TU Munich Computer Vision Group: https://vision.in.tum.de/
