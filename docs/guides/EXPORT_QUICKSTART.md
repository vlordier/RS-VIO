# Quick Start: Teacher Data Export

## 1. Build the Export Binary

```bash
cd /Users/vincent/Work/RS-VIO

# Build in release mode (optimized)
cargo build --bin export_teacher --features export-teacher --release
```

**Output**: `target/release/export_teacher`

## 2. Prepare TUM-VI Dataset

Make sure you have TUM-VI dataset downloaded. Structure should be:
```
/path/to/tum-vi/
├── room1/
│   ├── cam0/
│   ├── cam1/
│   ├── imu0/
│   └── groundtruth.txt
├── room2/
├── room3/
├── room4/
├── room5/
└── room6/
```

## 3. Test on Single Sequence

```bash
# Create test output directory
mkdir -p test_export

# Run export on room1 (smaller sequence for testing)
./target/release/export_teacher \
  --config config/teacher_tumvi_export.yaml \
  --dataset-path /path/to/tum-vi/room1 \
  --output-dir test_export \
  --sequence-name room1_test

# Expected output:
# - test_export/room1_test.h5 (150-250 MB)
# - test_export/export_stats.json
```

## 4. Validate Export

```bash
# Install Python dependencies (first time only)
pip install h5py numpy

# Basic validation
python tools/validate_export.py test_export/room1_test.h5

# Detailed validation with statistics
python tools/validate_export.py test_export/room1_test.h5 --verbose

# With visualization (requires matplotlib)
pip install matplotlib
python tools/validate_export.py test_export/room1_test.h5 --visualize --samples 5
```

## 5. Generate Full Dataset

```bash
# Make script executable
chmod +x tools/generate_teacher_dataset.sh

# Generate all sequences
./tools/generate_teacher_dataset.sh /path/to/tum-vi ./teacher_data

# Expected output:
# teacher_data/
# ├── room1/
# │   ├── room1.h5
# │   ├── export.log
# │   └── export_stats.json
# ├── room2/
# └── ...

# Check results
ls -lh teacher_data/*/
du -sh teacher_data/
```

## Typical Timeline

| Step | Duration | Notes |
|------|----------|-------|
| Build binary | 5-10 sec | First time: 30-60 sec |
| Single sequence | 10-30 min | Depends on sequence length |
| Full dataset (6 sequences) | 3-6 hours | Can run in parallel with tmux |
| Validation | 1-2 min | Per sequence |

## Troubleshooting

### Binary not found
```bash
# Check if build succeeded
ls -l target/release/export_teacher

# Rebuild
cargo clean
cargo build --bin export_teacher --features export-teacher --release
```

### Dataset path issues
```bash
# Verify TUM-VI structure
ls -la /path/to/tum-vi/room1/cam0/ | head

# Should show image files like:
# 1305031102916175872.png
# 1305031102916175873.png
# ...
```

### Export hangs or crashes
```bash
# Check logs for errors
tail -f test_export/export.log  # If available

# Try with verbose logging
RUST_LOG=debug ./target/release/export_teacher ...
```

### Validation failures
```bash
# If h5py not installed
pip install h5py

# If matplotlib missing (not required)
pip install matplotlib  # Optional, for visualizations
```

## Output Structure

Each sequence produces:

```
room1/
├── room1.h5                 # Main HDF5 file (150-250 MB)
├── export.log              # Execution log
└── export_stats.json       # Statistics
    {
      "num_frames": 612,
      "timestamp_duration": 613.45,
      "depth_coverage_avg": 78.5,
      "image_mean_avg": 127.3,
      "imu_norm_avg": 0.234
    }
```

## Next: Student Network Training

Once dataset is generated:

```bash
# List all exported sequences
ls -1 teacher_data/*/room*.h5

# Ready to use for training
# - Load with h5py
# - Create PyTorch DataLoader
# - Train student network
# - Benchmark against RANSAC
```

## Commands for Quick Testing

```bash
# All-in-one test (requires TUM-VI at /data/tum-vi)
mkdir -p quick_test && \
./target/release/export_teacher \
  --dataset-path /data/tum-vi/room1 \
  --output-dir quick_test \
  --sequence-name test && \
python tools/validate_export.py quick_test/test.h5 --verbose
```

## Further Help

```bash
# Export binary help
./target/release/export_teacher --help

# Validation tool help
python tools/validate_export.py --help
```

---

**Note**: First export of a sequence takes ~10-30 minutes depending on CPU and disk speed. Subsequent exports cache some data, so may be faster if re-running with same parameters.
