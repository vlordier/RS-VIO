#!/usr/bin/env python3
"""
Download datasets for RS-VIO testing and benchmarking.

Supports:
- EuRoC: Requires manual download from https://projects.asl.ethz.ch/datasets/euroc-mav/
- TUM-VI: Automatic download from https://vision.in.tum.de/data/datasets/visual-inertial-dataset
- 4Seasons: Manual download from https://cvg.cit.tum.de/data/datasets/4seasons-dataset/download

Usage:
    python scripts/download_datasets.py --target /path/to/datasets [--datasets euroc,tum,4seasons]
    python scripts/download_datasets.py --target /tmp/rs-vio-datasets --datasets all
"""

import argparse
import shutil
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path
from typing import Optional

from dataset_utils import extract_tar, extract_zip

try:
    from rich.console import Console
    from rich.table import Table
    HAS_RICH = True
except ImportError:
    HAS_RICH = False
    Console = None  # type: ignore
    Table = None    # type: ignore

# Import configuration
try:
    from dataset_config import (
        DATASETS,
        DOWNLOAD_CONFIG,
        ERROR_MESSAGES,
        INFO_MESSAGES,
        REQUIRED_TOOLS,
        SUCCESS_MESSAGES,
        logger,
    )
except ImportError:
    # Fallback if config not available
    DATASETS = {}
    DOWNLOAD_CONFIG = {"max_retries": 3, "retry_delays": [2, 4, 8]}
    REQUIRED_TOOLS = ["tar", "unzip"]
    ERROR_MESSAGES = {}
    SUCCESS_MESSAGES = {}
    INFO_MESSAGES = {}
    logger = None  # type: ignore

console: Optional[Console] = Console() if HAS_RICH else None


class DatasetDownloader:
    """Handle downloading and extracting datasets."""

    def __init__(self, target_dir: Path):
        """Initialize downloader with target directory.

        Args:
            target_dir: Target directory for downloaded datasets

        Raises:
            OSError: If target directory cannot be created
        """
        try:
            self.target_dir = Path(target_dir)
            self.target_dir.mkdir(parents=True, exist_ok=True)

            if logger:
                logger.info(f"Initialized downloader with target: {self.target_dir}")
        except OSError as e:
            error_msg = f"Failed to create target directory {target_dir}: {e}"
            if logger:
                logger.error(error_msg)
            raise

    def check_dependencies(self) -> bool:
        """Check if required tools are available.

        Returns:
            True if all tools are available, False otherwise
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

    def download_file(self, url: str, output_path: Path, max_retries: int = 3) -> bool:
        """Download a file with retry logic and skip if exists."""
        output_path.parent.mkdir(parents=True, exist_ok=True)

        # Skip if already downloaded
        if output_path.exists():
            size = output_path.stat().st_size / (1024 * 1024)
            msg = f"✅ Already downloaded: {output_path.name} ({size:.1f} MB)"
            if HAS_RICH and console:
                console.print(msg, style="green")
            else:
                print(msg)
            return True

        for attempt in range(1, max_retries + 1):
            try:
                msg = f"⬇️  Downloading (attempt {attempt}/{max_retries}): {output_path.name}"
                if HAS_RICH and console:
                    console.print(msg, style="blue")
                else:
                    print(msg)

                urllib.request.urlretrieve(url, output_path)

                size = output_path.stat().st_size / (1024 * 1024)
                msg = f"✅ Downloaded: {output_path.name} ({size:.1f} MB)"
                if HAS_RICH and console:
                    console.print(msg, style="green")
                else:
                    print(msg)
                return True

            except Exception as e:
                msg = f"⚠️  Attempt {attempt} failed: {e}"
                if HAS_RICH and console:
                    console.print(msg, style="yellow")
                else:
                    print(msg)

                if attempt < max_retries:
                    wait = 2 ** attempt  # Exponential backoff: 2, 4, 8 seconds
                    if HAS_RICH and console:
                        console.print(f"⏳ Retrying in {wait}s...", style="dim")
                    else:
                        print(f"Retrying in {wait}s...")
                    time.sleep(wait)

        msg = f"❌ Failed to download after {max_retries} attempts: {url}"
        if HAS_RICH and console:
            console.print(msg, style="red")
        else:
            print(msg)
        return False

    def download_euroc(self) -> bool:
        """Download EuRoC dataset.

        Returns:
            True if dataset is available or extracted, False otherwise
        """
        dataset = DATASETS.get("euroc", {})
        name = dataset.get("name", "EuRoC")
        url = dataset.get("url", "https://projects.asl.ethz.ch/datasets/euroc-mav/")

        self._print_header("📊 EuRoC MH_01_easy Dataset")

        euroc_dir = self.target_dir / "euroc"

        # Check if already extracted
        extracted_dirs = list(euroc_dir.glob("**/MH_01_easy")) if euroc_dir.exists() else []
        if extracted_dirs:
            success_msg = f"✅ {name} already extracted"
            if logger:
                logger.info(f"EuRoC already extracted at {euroc_dir}")
            self._print_success(success_msg)
            return True

        # Look for manual download in target directory
        manual_archive_path = self.target_dir / "MH_01_easy.zip"

        if manual_archive_path.exists():
            if logger:
                logger.info(f"Found local EuRoC file: {manual_archive_path}")
            if extract_zip(manual_archive_path, euroc_dir):
                try:
                    manual_archive_path.unlink()
                    if logger:
                        logger.info(f"Deleted archive file: {manual_archive_path}")
                except OSError as e:
                    if logger:
                        logger.warning(f"Failed to delete {manual_archive_path}: {e}")

                success_msg = f"✅ {name} extracted"
                if logger:
                    logger.info("EuRoC extraction and setup complete")
                self._print_success(success_msg)
                return True
        else:
            info_msg = f"ℹ️ {name} requires manual download"
            if logger:
                logger.info("EuRoC manual download required")
            self._print_info(info_msg)
            if HAS_RICH and console:
                console.print("[bold cyan]Steps:[/bold cyan]")
                console.print(f"  1. Register at {url}", style="dim")
                console.print("  2. Download MH_01_easy.zip", style="dim")
                console.print(f"  3. Place it in {self.target_dir}", style="dim")
                console.print("  4. Re-run this script", style="dim")
            else:
                print("Steps:")
                print(f"  1. Register at {url}")
                print("  2. Download MH_01_easy.zip")
                print(f"  3. Place it in {self.target_dir}")
                print("  4. Re-run this script")

        return False

    def download_tum(self) -> bool:
        """Download TUM-VI dataset.

        Returns:
            True if download/extraction successful, False otherwise
        """
        dataset = DATASETS.get("tum", {})
        name = dataset.get("name", "TUM-VI")
        sequences = dataset.get("sequences", {})

        if not sequences:
            url = str(dataset.get("url", ""))
            sequences = {"default": url}

        self._print_header("📊 TUM-VI Dataset Download")

        if logger:
            logger.info("Starting TUM-VI dataset download")

        tum_dir = self.target_dir / "tum_vi"

        # Check if already extracted (MAV0 format)
        if tum_dir.exists() and list(tum_dir.glob("**/mav0/cam0/data/*")):
            success_msg = f"✅ {name} already extracted"
            if logger:
                logger.info(f"TUM-VI already extracted at {tum_dir}")
            self._print_success(success_msg)
            return True

        # Download each sequence
        all_success = True
        for seq_name, seq_url in sequences.items():
            archive_path = self.target_dir / f"tum_vi_{seq_name}.tar"
            seq_dir = tum_dir / seq_name

            if seq_dir.exists() and list(seq_dir.glob("**/mav0/cam0/data/*")):
                self._print_success(f"✅ {seq_name} already extracted")
                continue

            if self.download_file(seq_url, archive_path):
                extracted = extract_tar(archive_path, seq_dir)
                # Always attempt to remove archive after extraction attempt
                try:
                    if archive_path.exists():
                        archive_path.unlink()
                except OSError as e:
                    if logger:
                        logger.warning(f"Could not remove archive {archive_path}: {e}")

                if extracted:
                    self._print_success(f"✅ {seq_name} extracted")
                else:
                    self._print_error(f"❌ {seq_name} extraction failed")
                    all_success = False
            else:
                all_success = False

        if all_success:
            if logger:
                logger.info("TUM-VI download and extraction complete")

        return all_success

    def download_4seasons(self) -> bool:
        """Download 4Seasons dataset.

        Returns:
            True if dataset found locally, False otherwise
        """
        dataset = DATASETS.get("4seasons", {})
        name = dataset.get("name", "4Seasons")
        url = dataset.get("url", "https://cvg.cit.tum.de/data/datasets/4seasons-dataset/download")

        self._print_header("📊 4Seasons Dataset")

        if logger:
            logger.info("Checking for 4Seasons dataset")

        seasons_dir = self.target_dir / "4seasons"

        if seasons_dir.exists() and list(seasons_dir.iterdir()):
            success_msg = f"✅ {name} found"
            if logger:
                logger.info(f"4Seasons dataset found at {seasons_dir}")
            self._print_success(success_msg)
            return True
        else:
            info_msg = f"ℹ️ {name} requires manual download"
            if logger:
                logger.info("4Seasons manual download required")
            self._print_info(info_msg)
            if HAS_RICH and console:
                console.print("[bold cyan]Steps:[/bold cyan]")
                console.print(f"  1. Visit {url}", style="dim")
                console.print("  2. Download one or more recording ZIPs (undistorted recommended)", style="dim")
                console.print(f"  3. Extract to {seasons_dir}", style="dim")
                console.print("  4. Re-run this script", style="dim")
            else:
                print("Steps:")
                print(f"  1. Visit {url}")
                print("  2. Download one or more recording ZIPs (undistorted recommended)")
                print(f"  3. Extract to {seasons_dir}")
                print("  4. Re-run this script")

        return False

    def download_all(self) -> bool:
        """Download all datasets."""
        results = {
            "euroc": self.download_euroc(),
            "tum": self.download_tum(),
            "4seasons": self.download_4seasons(),
        }

        self._print_summary(results)
        return True

    def download_specific(self, datasets: list) -> bool:
        """Download specific datasets."""
        results = {}

        for dataset in datasets:
            if dataset.lower() == "euroc":
                results["euroc"] = self.download_euroc()
            elif dataset.lower() == "tum":
                results["tum"] = self.download_tum()
            elif dataset.lower() == "4seasons":
                results["4seasons"] = self.download_4seasons()

        self._print_summary(results)
        return True

    def _print_summary(self, results: dict) -> None:
        """Print download summary using rich or plain text."""
        if HAS_RICH and console:
            console.rule("📊 DOWNLOAD SUMMARY", style="green")
            table = Table(show_header=True, header_style="bold magenta")
            table.add_column("Dataset", style="cyan")
            table.add_column("Status", style="green")

            for dataset, success in results.items():
                status = "✅ Downloaded" if success else "⏭️  Skipped"
                table.add_row(dataset.upper(), status)

            console.print(table)
            console.print(f"\n📁 Datasets available at: {self.target_dir}\n", style="bold blue")
        else:
            print("\n" + "=" * 70)
            print("📊 DOWNLOAD SUMMARY")
            print("=" * 70)
            for dataset, success in results.items():
                status = "✅" if success else "⏭️"
                print(f"{status} {dataset.upper()}")
            print("=" * 70)

        if logger:
            logger.info(f"Download summary: {results}")


    def _print_header(self, text: str) -> None:
        """Print a formatted header."""
        if HAS_RICH and console:
            console.rule(text, style="blue")
        else:
            print("\n" + "=" * 70)
            print(text)
            print("=" * 70)

    def _print_success(self, text: str) -> None:
        """Print a success message."""
        if HAS_RICH and console:
            console.print(text, style="green")
        else:
            print(text)

    def _print_error(self, text: str) -> None:
        """Print an error message."""
        if HAS_RICH and console:
            console.print(text, style="red")
        else:
            print(text)

    def _print_warning(self, text: str) -> None:
        """Print a warning message."""
        if HAS_RICH and console:
            console.print(text, style="yellow")
        else:
            print(text)

    def _print_info(self, text: str) -> None:
        """Print an info message."""
        if HAS_RICH and console:
            console.print(text, style="blue")
        else:
            print(text)


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="Download RS-VIO datasets",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python scripts/download_datasets.py --target /tmp/datasets
  python scripts/download_datasets.py --target /tmp/datasets --datasets euroc,tum
  python scripts/download_datasets.py --target /tmp/datasets --datasets all
        """,
    )

    parser.add_argument(
        "--target",
        required=True,
        help="Target directory for datasets",
    )
    parser.add_argument(
        "--datasets",
        default="all",
        help="Comma-separated list of datasets to download (euroc, tum, 4seasons, all)",
    )

    args = parser.parse_args()

    downloader = DatasetDownloader(args.target)

    # Check dependencies
    if not downloader.check_dependencies():
        msg = "❌ Please install missing dependencies"
        if HAS_RICH:
            console.print(msg, style="bold red")
        else:
            print(msg)
        return 1

    if HAS_RICH:
        console.rule("[bold blue]🎬 RS-VIO Dataset Downloader[/bold blue]")
        console.print(f"Target: {downloader.target_dir}\n", style="dim")
    else:
        print("\n" + "=" * 70)
        print("🎬 RS-VIO Dataset Downloader")
        print("=" * 70)
        print(f"Target: {downloader.target_dir}\n")

    # Download datasets
    if args.datasets.lower() == "all":
        downloader.download_all()
    else:
        datasets = [d.strip() for d in args.datasets.split(",")]
        downloader.download_specific(datasets)

    msg = f"✅ Datasets available in: {downloader.target_dir}\n"
    if HAS_RICH:
        console.print(msg, style="bold green")
    else:
        print(msg)

    return 0


if __name__ == "__main__":
    sys.exit(main())
