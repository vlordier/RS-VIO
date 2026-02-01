"""
Configuration and constants for dataset management.

This module centralizes all constants, URLs, paths, and configuration
parameters used by dataset download and setup scripts.
"""

import logging
import tempfile
from pathlib import Path
from typing import List

# ============================================================================
# Logging Configuration
# ============================================================================

LOG_LEVEL = logging.INFO
LOG_FORMAT = "%(asctime)s - %(name)s - %(levelname)s - %(message)s"
LOG_FILE = Path(__file__).parent / "dataset_management.log"

# Setup logger
logger = logging.getLogger("dataset_management")
logger.setLevel(LOG_LEVEL)

# File handler
file_handler = logging.FileHandler(LOG_FILE)
file_handler.setFormatter(logging.Formatter(LOG_FORMAT))
logger.addHandler(file_handler)

# Console handler
console_handler = logging.StreamHandler()
console_handler.setFormatter(logging.Formatter(LOG_FORMAT))
logger.addHandler(console_handler)


# ============================================================================
# Dataset URLs and Sources
# ============================================================================

DATASETS = {
    "euroc": {
        "name": "EuRoC MH_01_easy",
        "url": "https://projects.asl.ethz.ch/datasets/euroc-mav/",
        "manual": True,
        "local_path": str(Path(tempfile.gettempdir()) / "MH_01_easy.zip"),
        "description": "EuRoC Micro Aerial Vehicle dataset",
    },
    "tum": {
        "name": "TUM-VI freiburg3_walking_xyz",
        "url": "http://download.tum.de/rgbd/dataset/freiburg3/rgbd-dataset_freiburg3_walking_xyz.tgz",
        "manual": False,
        "local_path": None,
        "description": "TUM Visual-Inertial dataset",
    },
    "4seasons": {
        "name": "4Seasons Dataset",
        "url": "https://cvg.cit.tum.de/data/datasets/4seasons-dataset/download",
        "manual": True,
        "local_path": None,
        "description": "4Seasons cross-season dataset",
    },
}


# ============================================================================
# Download Configuration
# ============================================================================

DOWNLOAD_CONFIG = {
    "max_retries": 3,
    "retry_delays": [2, 4, 8],  # Exponential backoff: 2s, 4s, 8s
    "chunk_size": 8192,  # 8KB chunks for downloads
    "timeout": 30,  # Connection timeout in seconds
    "connection_retries": 3,
}


# ============================================================================
# System Requirements
# ============================================================================

REQUIRED_TOOLS: List[str] = ["curl", "tar", "unzip"]
MIN_DISK_SPACE_MB = 2000  # Minimum 2GB free space recommended


# ============================================================================
# Dataset Directory Structure Patterns
# ============================================================================

DATASET_PATTERNS = {
    "euroc": {
        "extracted_marker": "MH_01_easy",
        "subdirs": ["MH_01_easy"],
        "files": ["MH_01_easy/imu0.csv"],
    },
    "tum": {
        "extracted_marker": "rgb",
        "subdirs": ["rgb", "depth", "groundtruth.txt"],
        "files": ["rgb/1311243649.357847.png"],
    },
    "4seasons": {
        "extracted_marker": "recordings",
        "subdirs": ["Recordings"],
        "files": [],
    },
}


# ============================================================================
# Checksum Configuration
# ============================================================================

CHECKSUM_FILE = Path(__file__).parent / "dataset_checksums.sha256"
CHECKSUM_ALGORITHM = "sha256"
CHECKSUM_CHUNK_SIZE = 4096


# ============================================================================
# Error Messages
# ============================================================================

ERROR_MESSAGES = {
    "missing_tools": "Missing required tools: {tools}. Please install them and try again.",
    "download_failed": "Failed to download {name} after {retries} attempts: {error}",
    "extraction_failed": "Failed to extract {file}: {error}",
    "checksum_mismatch": "Checksum mismatch for {file}. Expected {expected}, got {actual}",
    "checksum_missing": "No checksum found for {file} in {checksum_file}",
    "invalid_path": "Invalid path: {path}",
    "directory_creation_failed": "Failed to create directory {path}: {error}",
    "disk_space_low": "Low disk space. Available: {available}MB, Recommended: {recommended}MB",
    "no_datasets_found": "No datasets found in {path}",
}


# ============================================================================
# Success Messages
# ============================================================================

SUCCESS_MESSAGES = {
    "download_complete": "Successfully downloaded {name}",
    "extraction_complete": "Successfully extracted {file}",
    "checksum_verified": "Checksum verified for {file}",
    "setup_complete": "Dataset setup complete for {dataset}",
    "already_downloaded": "Already downloaded: {name} ({size}MB)",
    "already_extracted": "Already extracted: {name}",
}


# ============================================================================
# Info Messages
# ============================================================================

INFO_MESSAGES = {
    "starting_download": "Starting download of {name}...",
    "attempt": "Attempt {attempt}/{max_attempts}",
    "retrying": "Retrying in {delay}s...",
    "extracting": "Extracting {file}...",
    "verifying": "Verifying {file}...",
    "manual_download_required": "Manual download required for {name}",
    "steps": "Please follow these steps:",
}
