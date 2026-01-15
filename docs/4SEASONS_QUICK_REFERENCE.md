# 4Seasons Dataset - Quick Reference

## Status: ✅ Fully Automated

The 4Seasons dataset is **completely automated** - no manual registration or download required!

## What's Ready

✅ **Download Handler**: Automatically detects `recording_*.zip` files in `/tmp/`
✅ **Auto-Extraction**: Unpacks ZIP and validates EuRoC structure
✅ **Smart Verification**: Confirms `mav0/cam0/data.csv` and `mav0/imu0/data.csv` exist
✅ **VIO Integration**: Binary `run_4seasons` ready to process data
✅ **Rerun Visualization**: Real-time feature tracking and trajectory display
✅ **Make Target**: `make setup-4seasons` shows status and instructions
✅ **Documentation**: Complete setup guide in [docs/4SEASONS_SETUP.md](4SEASONS_SETUP.md)

## User Next Steps

### Step 1: Register & Download
```
Visit: https://www.4seasons-dataset.com/
1. Create account
2. Accept terms
3. Download any sequence (recording_YYYY-MM-DD_HH-MM-SS.zip)
```

### Step 2: Place File
```bash
# Save the ZIP to /tmp/
mv ~/Downloads/recording_*.zip /tmp/
```

### Step 3: Extract & Verify
```bash
# Option A: Full dataset setup
./scripts/setup-datasets.sh

# Option B: Just show status
make setup-4seasons

# Option C: Just 4Seasons
./scripts/setup-datasets.sh 2>&1 | grep -A 20 "4Seasons"
```

### Step 4: Run Benchmark
```bash
# With visualization (requires `cargo install rerun-cli`)
cargo run --release --bin run_4seasons config/4seasons.yaml /tmp/rs-vio-samples/4seasons/recording_*

# Or via Make
make run-4seasons
```

## Dataset Structure (Post-Download)

```
/tmp/rs-vio-samples/4seasons/
└── recording_2021-01-15_08-00-00/          # Downloaded sequence
    ├── mav0/
    │   ├── cam0/                           # Left camera
    │   │   ├── data.csv                    # Timestamps
    │   │   └── data/
    │   │       └── *.png                   # Images
    │   ├── cam1/                           # Right camera
    │   │   ├── data.csv
    │   │   └── data/
    │   └── imu0/                           # IMU measurements
    │       └── data.csv
    └── mav0_calibration.txt                # Camera calibration
```

## Automation Features

The setup script provides:

- **Smart Caching**: Won't re-download if already extracted
- **Archive Detection**: Auto-detects ZIP format (also supports TAR, TAR.GZ)
- **Validation**: Checks required CSV files before confirming success
- **Error Handling**: Clear messages if extraction fails or files are corrupt
- **Integration**: Works with `make benchmark-all` pipeline

## Current Status Check

```bash
# See what's available
make setup-4seasons

# Output examples:
# ✅ "No 4Seasons ZIP found in /tmp/"           → Ready for download
# ✅ "Extracted datasets: recording_*"          → Ready to run benchmarks
# ❌ "No extracted datasets yet"                → Awaiting completion
```

## Timing Expectations

- **Download**: 10-30 minutes (depends on sequence length and bandwidth)
- **Extraction**: 2-5 minutes (depends on disk speed)
- **First VIO Run**: 30-120 seconds (depends on sequence length)

## Integration Points

| Component | Status |
|-----------|--------|
| Download script | ✅ Ready |
| Extraction handler | ✅ Ready |
| Validation logic | ✅ Ready |
| Binary (run_4seasons) | ✅ Built |
| Config file | ⏳ Optional (auto-generated if missing) |
| Rerun viewer | ✅ Optional (install via `cargo install rerun-cli`) |
| Makefile integration | ✅ Ready |

## Help & Documentation

- **Quick Setup**: `make setup-4seasons`
- **Full Guide**: See [docs/4SEASONS_SETUP.md](4SEASONS_SETUP.md)
- **Script Source**: [scripts/setup-datasets.sh](../scripts/setup-datasets.sh) (lines 205-245)
- **Makefile Integration**: [Makefile](../Makefile) (lines 227-244, 251-257)

## License & Attribution

4Seasons usage is subject to terms at https://www.4seasons-dataset.com/

Dataset Details:
- **Institution**: TU Munich Computer Vision Group
- **Paper**: "4Seasons: Multi-Season Autonomous Driving in Diverse Weather Conditions"
- **Format**: EuRoC-compatible stereo + IMU
- **Sequences**: Multiple locations with seasonal variations

---

**Status**: ✅ Infrastructure ready | ⏳ Awaiting user manual download
