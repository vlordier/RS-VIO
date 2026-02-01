# RS-VIO Dataset Management Scripts

Python utilities for managing and setting up RS-VIO datasets.

## Scripts

### `download_datasets.py`

Download datasets for testing and benchmarking.

**Supported Datasets:**
- **EuRoC**: Requires manual download from https://projects.asl.ethz.ch/datasets/euroc-mav/
- **TUM-VI**: Automatically downloaded from https://vision.in.tum.de/data/datasets/visual-inertial-dataset
- **4Seasons**: Requires manual download from https://www.4seasons-dataset.com/

**Usage:**

```bash
# Download all datasets
python scripts/download_datasets.py --target /tmp/datasets --datasets all

# Download specific datasets
python scripts/download_datasets.py --target /tmp/datasets --datasets euroc,tum

# Download just TUM-VI (automatic)
python scripts/download_datasets.py --target /tmp/datasets --datasets tum
```

**Options:**
- `--target DIR`: Target directory for datasets (required)
- `--datasets LIST`: Comma-separated list of datasets to download (default: all)

**Manual Downloads:**

For EuRoC and 4Seasons datasets that require manual download:

1. **EuRoC:**
   - Register at https://projects.asl.ethz.ch/datasets/euroc-mav/
   - Download `MH_01_easy.zip`
   - Place in `/tmp/MH_01_easy.zip`
   - Script will extract it automatically

2. **4Seasons:**
   - Download from https://www.4seasons-dataset.com/
   - Extract to `<target-dir>/4seasons/`

### `setup_datasets.py`

Setup and verify downloaded datasets.

**Usage:**

```bash
# Verify available datasets
python scripts/setup_datasets.py --datasets-dir /tmp/datasets --verify

# Setup (extract) datasets
python scripts/setup_datasets.py --datasets-dir /tmp/datasets

# Setup with custom checksum file
python scripts/setup_datasets.py --checksum custom_checksums.sha256
```

**Options:**
- `--datasets-dir DIR`: Path to datasets directory (default: ./datasets)
- `--checksum FILE`: Path to checksums file for verification
- `--verify`: Verify datasets and exit (don't setup)

**Features:**
- Automatic archive extraction
- SHA256 checksum verification (optional)
- Dataset readiness validation
- Detailed status reporting

## Quick Start

### Download and Setup All Datasets

```bash
# 1. Download datasets to /tmp
python scripts/download_datasets.py --target /tmp/datasets --datasets all

# 2. For EuRoC: manually download and place MH_01_easy.zip in /tmp/
cp /path/to/MH_01_easy.zip /tmp/MH_01_easy.zip

# 3. Re-run download to extract EuRoC
python scripts/download_datasets.py --target /tmp/datasets --datasets euroc

# 4. Verify all datasets are ready
python scripts/setup_datasets.py --datasets-dir /tmp/datasets --verify
```

### Using with `just`

The `justfile` provides shortcuts:

```bash
# Download all datasets
just download-datasets

# Setup datasets
just setup-datasets
```

## Dataset Sizes

- **TUM-VI**: ~1.5 GB (automatic download)
- **EuRoC MH_01_easy**: ~260 MB (manual download)
- **4Seasons**: 10-50 GB per recording (manual download)

## Requirements

- Python 3.8+
- `curl`: For downloading files
- `tar`: For extracting tar.gz archives
- `unzip`: For extracting ZIP archives

## Checksums

Optional SHA256 verification using `dataset_checksums.sha256`:

```
abc123def456...  tum_vi.tgz
fed456abc123...  MH_01_easy.zip
```

One checksum per line in format: `<sha256>  <filename>`

## Troubleshooting

### Download Failures

- Check internet connection
- Verify URLs are accessible
- Run with `--verbose` flag for detailed output

### Extraction Issues

- Ensure sufficient disk space
- Check file permissions
- Verify archives aren't corrupted (use `--checksum` option)

### Missing Tools

Install required tools:

```bash
# macOS
brew install curl tar unzip

# Ubuntu/Debian
sudo apt-get install curl tar unzip

# CentOS/RHEL
sudo yum install curl tar unzip
```

## See Also

- [RS-VIO README](../README.md)
- [Benchmarking Guide](../BENCHMARKING_QUICKSTART.sh)
- [Dataset Documentation](../DATASETS.md)
