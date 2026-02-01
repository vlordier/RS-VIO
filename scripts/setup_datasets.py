#!/usr/bin/env python3
"""
Setup and verify datasets for RS-VIO testing.

Performs:
- Directory structure validation
- Checksum verification (optional, SHA256)
- Archive extraction
- Dataset readiness checks

Usage:
    python scripts/setup_datasets.py --datasets-dir /path/to/datasets [--verify]
    python scripts/setup_datasets.py --datasets-dir /tmp/rs-vio-datasets --verify
    python scripts/setup_datasets.py  # Uses default: ./datasets
"""

import argparse
import hashlib
import json
import shutil
import subprocess
import sys
import tempfile
import zipfile
from pathlib import Path
from typing import Dict, List, Optional, Tuple


class DatasetSetup:
    """Handle dataset setup, extraction, and verification."""

    def __init__(self, datasets_dir: Path, checksum_file: Optional[Path] = None):
        """Initialize dataset setup."""
        self.datasets_dir = Path(datasets_dir)
        self.datasets_dir.mkdir(parents=True, exist_ok=True)

        self.checksum_file = checksum_file or Path(__file__).parent / "dataset_checksums.sha256"
        self.checksums = self._load_checksums()

    def _load_checksums(self) -> Dict[str, str]:
        """Load checksums from file."""
        checksums = {}

        if not self.checksum_file.exists():
            return checksums

        try:
            with open(self.checksum_file) as f:
                for line in f:
                    line = line.strip()
                    if not line or line.startswith("#"):
                        continue

                    parts = line.split()
                    if len(parts) >= 2:
                        sha256, filename = parts[0], parts[-1]
                        checksums[filename] = sha256
        except Exception as e:
            print(f"⚠️  Failed to load checksums: {e}")

        return checksums

    def check_tools(self) -> bool:
        """Check if required tools are available."""
        required = ["curl", "tar", "unzip"]
        missing = []

        for tool in required:
            if shutil.which(tool) is None:
                missing.append(tool)

        if missing:
            print(f"❌ Missing required tools: {', '.join(missing)}")
            return False

        return True

    def compute_checksum(self, filepath: Path) -> str:
        """Compute SHA256 checksum of a file."""
        sha256 = hashlib.sha256()

        try:
            with open(filepath, "rb") as f:
                for chunk in iter(lambda: f.read(4096), b""):
                    sha256.update(chunk)
            return sha256.hexdigest()
        except Exception as e:
            print(f"❌ Failed to compute checksum: {e}")
            return ""

    def verify_checksum(self, filepath: Path) -> bool:
        """Verify checksum of a file."""
        filename = filepath.name

        if filename not in self.checksums:
            print(f"⚠️  No checksum for {filename} (skipping verification)")
            return True

        expected = self.checksums[filename]
        actual = self.compute_checksum(filepath)

        if expected != actual:
            print(f"❌ Checksum mismatch for {filename}")
            print(f"   Expected: {expected}")
            print(f"   Actual:   {actual}")
            return False

        print(f"✅ Checksum verified: {filename}")
        return True

    def extract_zip(self, archive_path: Path, extract_to: Path) -> bool:
        """Extract a ZIP archive."""
        extract_to.mkdir(parents=True, exist_ok=True)

        try:
            print(f"📦 Extracting {archive_path.name} -> {extract_to.name}")
            with zipfile.ZipFile(archive_path, "r") as zip_ref:
                zip_ref.extractall(extract_to)
            print(f"✅ Extracted: {archive_path.name}")
            return True
        except Exception as e:
            print(f"❌ Extraction failed: {e}")
            return False

    def extract_tar_gz(self, archive_path: Path, extract_to: Path) -> bool:
        """Extract a tar.gz archive."""
        extract_to.mkdir(parents=True, exist_ok=True)

        try:
            print(f"📦 Extracting {archive_path.name} -> {extract_to.name}")
            subprocess.run(
                ["tar", "-xzf", str(archive_path), "-C", str(extract_to), "--strip-components=1"],
                check=True,
                capture_output=True,
            )
            print(f"✅ Extracted: {archive_path.name}")
            return True
        except subprocess.CalledProcessError as e:
            print(f"❌ Extraction failed: {e}")
            return False

    def check_euroc(self) -> Tuple[bool, str]:
        """Check EuRoC dataset status."""
        euroc_dir = self.datasets_dir / "euroc"

        if euroc_dir.exists() and list(euroc_dir.glob("**/MH_01_easy")):
            frame_count = len(list(euroc_dir.glob("**/cam0/data/*")))
            return True, f"✅ EuRoC ready ({frame_count} frames)"

        return False, "⏭️  EuRoC not found (manual download required)"

    def check_tum(self) -> Tuple[bool, str]:
        """Check TUM-VI dataset status."""
        tum_dir = self.datasets_dir / "tum_vi"

        if tum_dir.exists() and list(tum_dir.glob("**/rgb/*")):
            frame_count = len(list(tum_dir.glob("**/rgb/*")))
            return True, f"✅ TUM-VI ready ({frame_count} frames)"

        return False, "⏭️  TUM-VI not found"

    def check_4seasons(self) -> Tuple[bool, str]:
        """Check 4Seasons dataset status."""
        seasons_dir = self.datasets_dir / "4seasons"

        if seasons_dir.exists() and list(seasons_dir.iterdir()):
            recordings = [d.name for d in seasons_dir.iterdir() if d.is_dir()]
            if recordings:
                return True, f"✅ 4Seasons ready ({len(recordings)} recordings)"

        return False, "⏭️  4Seasons not found (manual download required)"

    def verify_all(self) -> bool:
        """Verify all datasets."""
        print("=" * 70)
        print("📊 DATASET VERIFICATION")
        print("=" * 70)
        print()

        results = []

        # Check each dataset
        euroc_ok, euroc_msg = self.check_euroc()
        results.append(("EuRoC", euroc_ok))
        print(euroc_msg)

        tum_ok, tum_msg = self.check_tum()
        results.append(("TUM-VI", tum_ok))
        print(tum_msg)

        seasons_ok, seasons_msg = self.check_4seasons()
        results.append(("4Seasons", seasons_ok))
        print(seasons_msg)

        print()
        print("=" * 70)

        available = sum(1 for _, ok in results if ok)
        print(f"📊 Available datasets: {available}/{len(results)}")
        print("=" * 70)
        print()

        return available > 0

    def setup_all(self) -> bool:
        """Setup all available datasets."""
        print("=" * 70)
        print("🔧 DATASET SETUP")
        print("=" * 70)
        print(f"Datasets directory: {self.datasets_dir}")
        print()

        # Check for archives and extract them
        archives_found = False

        # Check for EuRoC ZIP
        euroc_zip = self.datasets_dir / "MH_01_easy.zip"
        if euroc_zip.exists():
            archives_found = True
            if self.verify_checksum(euroc_zip):
                self.extract_zip(euroc_zip, self.datasets_dir / "euroc")

        # Check for TUM tgz
        tum_tgz = self.datasets_dir / "tum_vi.tgz"
        if tum_tgz.exists():
            archives_found = True
            if self.verify_checksum(tum_tgz):
                self.extract_tar_gz(tum_tgz, self.datasets_dir / "tum_vi")

        if not archives_found:
            print("ℹ️  No archives found to extract")

        print()

        # Verify datasets
        self.verify_all()

        return True


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="Setup and verify RS-VIO datasets",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python scripts/setup_datasets.py
  python scripts/setup_datasets.py --datasets-dir /tmp/datasets
  python scripts/setup_datasets.py --datasets-dir /tmp/datasets --verify
  python scripts/setup_datasets.py --checksum scripts/dataset_checksums.sha256
        """,
    )

    parser.add_argument(
        "--datasets-dir",
        default="./datasets",
        help="Path to datasets directory (default: ./datasets)",
    )
    parser.add_argument(
        "--checksum",
        help="Path to checksums file (default: scripts/dataset_checksums.sha256)",
    )
    parser.add_argument(
        "--verify",
        action="store_true",
        help="Verify datasets and exit (don't setup)",
    )

    args = parser.parse_args()

    # Check dependencies
    setup = DatasetSetup(Path(args.datasets_dir), Path(args.checksum) if args.checksum else None)

    if not setup.check_tools():
        print("❌ Please install missing dependencies")
        return 1

    # Run setup or verify
    if args.verify:
        setup.verify_all()
    else:
        setup.setup_all()

    return 0


if __name__ == "__main__":
    sys.exit(main())
