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
import shutil
import subprocess
import sys
import zipfile
from pathlib import Path
from typing import Dict, Optional, Tuple

try:
    from rich.console import Console
    from rich.table import Table
    HAS_RICH = True
except ImportError:
    HAS_RICH = False

# Import configuration
try:
    from dataset_config import (
        CHECKSUM_FILE,
        DATASET_PATTERNS,
        ERROR_MESSAGES,
        INFO_MESSAGES,
        REQUIRED_TOOLS,
        SUCCESS_MESSAGES,
        logger,
    )
except ImportError:
    # Fallback if config not available
    CHECKSUM_FILE = Path(__file__).parent / "dataset_checksums.sha256"
    DATASET_PATTERNS = {}
    ERROR_MESSAGES = {}
    SUCCESS_MESSAGES = {}
    INFO_MESSAGES = {}
    REQUIRED_TOOLS = ["curl", "tar", "unzip"]
    logger = None

console = Console() if HAS_RICH else None


class DatasetSetup:
    """Handle dataset setup, extraction, and verification."""

    def __init__(self, datasets_dir: Path, checksum_file: Optional[Path] = None):
        """Initialize dataset setup.

        Args:
            datasets_dir: Path to datasets directory
            checksum_file: Optional path to checksum file

        Raises:
            OSError: If datasets directory cannot be created
        """
        try:
            self.datasets_dir = Path(datasets_dir)
            self.datasets_dir.mkdir(parents=True, exist_ok=True)

            self.checksum_file = checksum_file or CHECKSUM_FILE
            self.checksums = self._load_checksums()

            if logger:
                logger.info(f"Initialized setup with datasets_dir: {self.datasets_dir}")
        except OSError as e:
            error_msg = f"Failed to initialize dataset setup: {e}"
            if logger:
                logger.error(error_msg)
            raise

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
        """Check if required tools are available.

        Returns:
            True if all tools available, False otherwise
        """
        missing = []

        for tool in REQUIRED_TOOLS:
            if shutil.which(tool) is None:
                missing.append(tool)

        if missing:
            error_msg = f"Missing required tools: {', '.join(missing)}"
            if logger:
                logger.error(error_msg)
            self._print_error(error_msg)
            return False

        if logger:
            logger.info(f"All required tools available: {', '.join(REQUIRED_TOOLS)}")
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
            msg = f"⚠️  No checksum for {filename} (skipping verification)"
            if HAS_RICH:
                console.print(msg, style="yellow")
            else:
                print(msg)
            return True

        expected = self.checksums[filename]
        actual = self.compute_checksum(filepath)

        if expected != actual:
            msg = f"❌ Checksum mismatch for {filename}"
            if HAS_RICH:
                console.print(msg, style="red")
                console.print(f"   Expected: {expected}", style="red")
                console.print(f"   Actual:   {actual}", style="red")
            else:
                print(msg)
                print(f"   Expected: {expected}")
                print(f"   Actual:   {actual}")
            return False

        msg = f"✅ Checksum verified: {filename}"
        if HAS_RICH:
            console.print(msg, style="green")
        else:
            print(msg)
        return True

    def extract_zip(self, archive_path: Path, extract_to: Path) -> bool:
        """Extract a ZIP archive."""
        extract_to.mkdir(parents=True, exist_ok=True)

        try:
            msg = f"📦 Extracting {archive_path.name} -> {extract_to.name}"
            if HAS_RICH:
                console.print(msg, style="cyan")
            else:
                print(msg)

            with zipfile.ZipFile(archive_path, "r") as zip_ref:
                zip_ref.extractall(extract_to)

            msg = f"✅ Extracted: {archive_path.name}"
            if HAS_RICH:
                console.print(msg, style="green")
            else:
                print(msg)
            return True
        except Exception as e:
            msg = f"❌ Extraction failed: {e}"
            if HAS_RICH:
                console.print(msg, style="red")
            else:
                print(msg)
            return False

    def extract_tar_gz(self, archive_path: Path, extract_to: Path) -> bool:
        """Extract a tar.gz archive."""
        extract_to.mkdir(parents=True, exist_ok=True)

        try:
            msg = f"📦 Extracting {archive_path.name} -> {extract_to.name}"
            if HAS_RICH:
                console.print(msg, style="cyan")
            else:
                print(msg)

            subprocess.run(
                ["tar", "-xzf", str(archive_path), "-C", str(extract_to), "--strip-components=1"],
                check=True,
                capture_output=True,
            )

            msg = f"✅ Extracted: {archive_path.name}"
            if HAS_RICH:
                console.print(msg, style="green")
            else:
                print(msg)
            return True
        except subprocess.CalledProcessError as e:
            msg = f"❌ Extraction failed: {e}"
            if HAS_RICH:
                console.print(msg, style="red")
            else:
                print(msg)
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

        if not tum_dir.exists():
            return False, "⏭️  TUM-VI not found"

        # Check for RGB-D format (rgb/ and depth/ directories)
        rgb_frames = list(tum_dir.glob("**/rgb/*"))
        if rgb_frames:
            frame_count = len(rgb_frames)
            return True, f"✅ TUM-VI ready ({frame_count} frames, RGB-D format)"

        # Check for MAV0 format (like EuRoC: room1/mav0/cam0/)
        mav0_dirs = list(tum_dir.glob("**/mav0/cam0/data/*"))
        if mav0_dirs:
            frame_count = len(mav0_dirs)
            sequences = len(list(tum_dir.glob("*/mav0")))
            return True, f"✅ TUM-VI ready ({sequences} sequences, {frame_count} frames, MAV0 format)"

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
        if HAS_RICH:
            console.rule("📊 DATASET VERIFICATION", style="blue")
        else:
            print("=" * 70)
            print("📊 DATASET VERIFICATION")
            print("=" * 70)

        results = []

        # Check each dataset
        euroc_ok, euroc_msg = self.check_euroc()
        results.append(("EuRoC", euroc_ok))
        if HAS_RICH:
            console.print(euroc_msg, style="green" if euroc_ok else "cyan")
        else:
            print(euroc_msg)

        tum_ok, tum_msg = self.check_tum()
        results.append(("TUM-VI", tum_ok))
        if HAS_RICH:
            console.print(tum_msg, style="green" if tum_ok else "cyan")
        else:
            print(tum_msg)

        seasons_ok, seasons_msg = self.check_4seasons()
        results.append(("4Seasons", seasons_ok))
        if HAS_RICH:
            console.print(seasons_msg, style="green" if seasons_ok else "cyan")
        else:
            print(seasons_msg)

        if HAS_RICH:
            console.print()
            table = Table(show_header=True, header_style="bold magenta")
            table.add_column("Dataset", style="cyan")
            table.add_column("Status", style="green")

            for dataset, ok in results:
                status = "✅ Available" if ok else "⏭️  Missing"
                table.add_row(dataset, status)

            console.print(table)

            available = sum(1 for _, ok in results if ok)
            console.rule(f"Available: {available}/{len(results)} datasets", style="green")
            console.print()
        else:
            print()
            print("=" * 70)
            available = sum(1 for _, ok in results if ok)
            print(f"📊 Available datasets: {available}/{len(results)}")
            print("=" * 70)
            print()

        return available > 0

    def setup_all(self) -> bool:
        """Setup all available datasets."""
        if HAS_RICH:
            console.rule("🔧 DATASET SETUP", style="yellow")
            console.print(f"Datasets directory: {self.datasets_dir}", style="bold")
            console.print()
        else:
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
            msg = "ℹ️  No archives found to extract"
            if HAS_RICH:
                console.print(msg, style="yellow")
            else:
                print(msg)

        if HAS_RICH:
            console.print()
        else:
            print()

        # Verify datasets
        self.verify_all()

        return True

    def _print_success(self, text: str) -> None:
        """Print a success message."""
        if HAS_RICH:
            console.print(text, style="green")
        else:
            print(text)

    def _print_error(self, text: str) -> None:
        """Print an error message."""
        if HAS_RICH:
            console.print(text, style="red")
        else:
            print(text)

    def _print_warning(self, text: str) -> None:
        """Print a warning message."""
        if HAS_RICH:
            console.print(text, style="yellow")
        else:
            print(text)

    def _print_info(self, text: str) -> None:
        """Print an info message."""
        if HAS_RICH:
            console.print(text, style="blue")
        else:
            print(text)


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

    if HAS_RICH:
        console.rule("[bold blue]🎬 RS-VIO Dataset Setup[/bold blue]")
        console.print()
    else:
        print("\n" + "=" * 70)
        print("🎬 RS-VIO Dataset Setup")
        print("=" * 70)
        print()

    if not setup.check_tools():
        msg = "❌ Please install missing dependencies"
        if HAS_RICH:
            console.print(msg, style="bold red")
        else:
            print(msg)
        return 1

    # Run setup or verify
    if args.verify:
        setup.verify_all()
    else:
        setup.setup_all()

    return 0


if __name__ == "__main__":
    sys.exit(main())
