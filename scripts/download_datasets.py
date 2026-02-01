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
import urllib.request
import zipfile
from pathlib import Path
from typing import Optional


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
        """Download a file with retry logic."""
        output_path.parent.mkdir(parents=True, exist_ok=True)

        for attempt in range(1, max_retries + 1):
            print(f"⬇️  Downloading (attempt {attempt}/{max_retries}): {url}")
            try:
                urllib.request.urlretrieve(url, output_path)
                print(f"✅ Downloaded: {output_path.name}")
                return True
            except Exception as e:
                print(f"⚠️  Download failed: {e}")
                if attempt < max_retries:
                    import time
                    time.sleep(5)

        print(f"❌ Failed to download after {max_retries} attempts: {url}")
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
        print("\n" + "=" * 70)
        print("📊 EuRoC MH_01_easy Dataset")
        print("=" * 70)
        print(f"Registration required: {self.EUROC_MANUAL_URL}")
        print("Please download MH_01_easy.zip manually and place in /tmp/")
        print()

        euroc_dir = self.target_dir / "euroc"

        if Path(self.EUROC_MANUAL_FILE).exists():
            if self.extract_zip(Path(self.EUROC_MANUAL_FILE), euroc_dir):
                Path(self.EUROC_MANUAL_FILE).unlink()
                print(f"✅ EuRoC extracted to {euroc_dir}")
                return True
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
        print("\n" + "=" * 70)
        print("📊 TUM-VI freiburg3_walking_xyz Dataset")
        print("=" * 70)

        tum_dir = self.target_dir / "tum_vi"
        archive_path = self.target_dir / "tum_vi.tgz"

        if self.download_file(self.TUM_VI_URL, archive_path):
            if self.extract_tar_gz(archive_path, tum_dir):
                archive_path.unlink()
                print(f"✅ TUM-VI extracted to {tum_dir}")
                return True

        return False

    def download_4seasons(self) -> bool:
        """Download 4Seasons dataset (manual)."""
        print("\n" + "=" * 70)
        print("📊 4Seasons Dataset")
        print("=" * 70)
        print("Download available at: https://www.4seasons-dataset.com/")
        print("Please download one or more recording ZIPs manually.")
        print(f"Extract to: {self.target_dir / '4seasons'}")
        print()

        seasons_dir = self.target_dir / "4seasons"
        if seasons_dir.exists() and list(seasons_dir.iterdir()):
            print(f"✅ 4Seasons found at {seasons_dir}")
            return True
        else:
            print("⏭️  4Seasons requires manual download (skipped)")

        return False

    def download_all(self) -> bool:
        """Download all datasets."""
        results = {
            "euroc": self.download_euroc(),
            "tum": self.download_tum(),
            "4seasons": self.download_4seasons(),
        }

        print("\n" + "=" * 70)
        print("📊 DOWNLOAD SUMMARY")
        print("=" * 70)
        for dataset, success in results.items():
            status = "✅" if success else "⏭️"
            print(f"{status} {dataset.upper()}")

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

        print("\n" + "=" * 70)
        print("📊 DOWNLOAD SUMMARY")
        print("=" * 70)
        for dataset, success in results.items():
            status = "✅" if success else "⏭️"
            print(f"{status} {dataset.upper()}")

        return True


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
        print("❌ Please install missing dependencies")
        return 1

    # Download datasets
    if args.datasets.lower() == "all":
        downloader.download_all()
    else:
        datasets = [d.strip() for d in args.datasets.split(",")]
        downloader.download_specific(datasets)

    print(f"\n✅ Datasets available in: {downloader.target_dir}\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
