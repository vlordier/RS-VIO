#!/usr/bin/env python3
"""
Download datasets for RS-VIO testing and benchmarking.

Supports:
- EuRoC: Requires manual download from https://projects.asl.ethz.ch/datasets/euroc-mav/
- TUM-VI: Automatic download from https://vision.in.tum.de/data/datasets/visual-inertial-dataset
- 4Seasons: Free download from https://www.4seasons-dataset.com/

Usage:
    python scripts/download_datasets.py --target /path/to/datasets [--datasets euroc,tum,4seasons]
    python scripts/download_datasets.py --target /tmp/rs-vio-datasets --datasets all
"""

import argparse
import shutil
import subprocess
import sys
import time
import urllib.request
import zipfile
from pathlib import Path
from typing import Optional

try:
    from rich.console import Console
    from rich.table import Table
    from rich.progress import Progress, SpinnerColumn, BarColumn, TextColumn
    HAS_RICH = True
except ImportError:
    HAS_RICH = False

console = Console() if HAS_RICH else None


class DatasetDownloader:
    """Handle downloading and extracting datasets."""

    EUROC_MANUAL_URL = "https://projects.asl.ethz.ch/datasets/euroc-mav/"
    TUM_VI_URL = "http://download.tum.de/rgbd/dataset/freiburg3/rgbd-dataset_freiburg3_walking_xyz.tgz"
    EUROC_MANUAL_FILE = "/tmp/MH_01_easy.zip"

    def __init__(self, target_dir: Path):
        """Initialize downloader with target directory."""
        self.target_dir = Path(target_dir)
        self.target_dir.mkdir(parents=True, exist_ok=True)

    def check_dependencies(self) -> bool:
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

    def download_file(self, url: str, output_path: Path, max_retries: int = 3) -> bool:
        """Download a file with retry logic and skip if exists."""
        output_path.parent.mkdir(parents=True, exist_ok=True)

        # Skip if already downloaded
        if output_path.exists():
            size = output_path.stat().st_size / (1024 * 1024)
            msg = f"✅ Already downloaded: {output_path.name} ({size:.1f} MB)"
            if HAS_RICH:
                console.print(msg, style="green")
            else:
                print(msg)
            return True

        for attempt in range(1, max_retries + 1):
            try:
                msg = f"⬇️  Downloading (attempt {attempt}/{max_retries}): {output_path.name}"
                if HAS_RICH:
                    console.print(msg, style="blue")
                else:
                    print(msg)

                urllib.request.urlretrieve(url, output_path)

                size = output_path.stat().st_size / (1024 * 1024)
                msg = f"✅ Downloaded: {output_path.name} ({size:.1f} MB)"
                if HAS_RICH:
                    console.print(msg, style="green")
                else:
                    print(msg)
                return True

            except Exception as e:
                msg = f"⚠️  Attempt {attempt} failed: {e}"
                if HAS_RICH:
                    console.print(msg, style="yellow")
                else:
                    print(msg)

                if attempt < max_retries:
                    wait = 2 ** attempt  # Exponential backoff: 2, 4, 8 seconds
                    if HAS_RICH:
                        console.print(f"⏳ Retrying in {wait}s...", style="dim")
                    else:
                        print(f"Retrying in {wait}s...")
                    time.sleep(wait)

        msg = f"❌ Failed to download after {max_retries} attempts: {url}"
        if HAS_RICH:
            console.print(msg, style="red")
        else:
            print(msg)
        return False

    def extract_zip(self, archive_path: Path, extract_to: Path) -> bool:
        """Extract a ZIP archive."""
        extract_to.mkdir(parents=True, exist_ok=True)

        try:
            print(f"📦 Extracting {archive_path.name} -> {extract_to}")
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
            print(f"📦 Extracting {archive_path.name} -> {extract_to}")
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

    def download_euroc(self) -> bool:
        """Download EuRoC dataset."""
        header = "📊 EuRoC MH_01_easy Dataset"
        if HAS_RICH:
            console.rule(header, style="blue")
        else:
            print("\n" + "=" * 70)
            print(header)
            print("=" * 70)

        msg = f"Registration required: {self.EUROC_MANUAL_URL}"
        if HAS_RICH:
            console.print(msg, style="yellow")
            console.print("Please download MH_01_easy.zip and place in /tmp/", style="dim")
        else:
            print(msg)
            print("Please download MH_01_easy.zip and place in /tmp/")

        euroc_dir = self.target_dir / "euroc"

        # Check if already extracted
        extracted_dirs = list(euroc_dir.glob("**/MH_01_easy")) if euroc_dir.exists() else []
        if extracted_dirs:
            msg = f"✅ EuROC already extracted to {euroc_dir}"
            if HAS_RICH:
                console.print(msg, style="green")
            else:
                print(msg)
            return True

        if Path(self.EUROC_MANUAL_FILE).exists():
            if self.extract_zip(Path(self.EUROC_MANUAL_FILE), euroc_dir):
                Path(self.EUROC_MANUAL_FILE).unlink()
                msg = f"✅ EuRoC extracted to {euroc_dir}"
                if HAS_RICH:
                    console.print(msg, style="green")
                else:
                    print(msg)
                return True
        else:
            msg = "⏭️  EuRoC requires manual download (skipped)"
            if HAS_RICH:
                console.print(msg, style="cyan")
                console.print("Steps:", style="bold")
                console.print(f"  1. Register at {self.EUROC_MANUAL_URL}", style="dim")
                console.print("  2. Download MH_01_easy.zip", style="dim")
                console.print("  3. Place it in /tmp/MH_01_easy.zip", style="dim")
                console.print("  4. Re-run this script", style="dim")
            else:
                print("⏭️  EuRoC requires manual download (skipped)")
                print(f"Steps:")
                print(f"  1. Register at {self.EUROC_MANUAL_URL}")
                print(f"  2. Download MH_01_easy.zip")
                print(f"  3. Place it in /tmp/MH_01_easy.zip")
                print(f"  4. Re-run this script")

        return False

    def download_tum(self) -> bool:
        """Download TUM-VI dataset."""
        header = "📊 TUM-VI freiburg3_walking_xyz Dataset"
        if HAS_RICH:
            console.rule(header, style="blue")
        else:
            print("\n" + "=" * 70)
            print(header)
            print("=" * 70)

        tum_dir = self.target_dir / "tum_vi"
        archive_path = self.target_dir / "tum_vi.tgz"

        # Check if already extracted
        if tum_dir.exists() and list(tum_dir.glob("**/rgb/*")):
            msg = f"✅ TUM-VI already extracted to {tum_dir}"
            if HAS_RICH:
                console.print(msg, style="green")
            else:
                print(msg)
            return True

        if self.download_file(self.TUM_VI_URL, archive_path):
            if self.extract_tar_gz(archive_path, tum_dir):
                archive_path.unlink()
                msg = f"✅ TUM-VI extracted to {tum_dir}"
                if HAS_RICH:
                    console.print(msg, style="green")
                else:
                    print(msg)
                return True

        return False

    def download_4seasons(self) -> bool:
        """Download 4Seasons dataset (manual)."""
        header = "📊 4Seasons Dataset"
        if HAS_RICH:
            console.rule(header, style="blue")
        else:
            print("\n" + "=" * 70)
            print(header)
            print("=" * 70)

        msg = "Download available at: https://www.4seasons-dataset.com/"
        if HAS_RICH:
            console.print(msg, style="yellow")
            console.print("Please download one or more recording ZIPs manually.", style="dim")
            console.print(f"Extract to: {self.target_dir / '4seasons'}", style="dim")
        else:
            print(msg)
            print("Please download one or more recording ZIPs manually.")
            print(f"Extract to: {self.target_dir / '4seasons'}")

        seasons_dir = self.target_dir / "4seasons"
        if seasons_dir.exists() and list(seasons_dir.iterdir()):
            msg = f"✅ 4Seasons found at {seasons_dir}"
            if HAS_RICH:
                console.print(msg, style="green")
            else:
                print(msg)
            return True
        else:
            msg = "⏭️  4Seasons requires manual download (skipped)"
            if HAS_RICH:
                console.print(msg, style="cyan")
            else:
                print(msg)

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
        if HAS_RICH:
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
