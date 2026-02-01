#!/usr/bin/env python3
"""Shared utilities for dataset management scripts."""

import subprocess
import zipfile
from pathlib import Path
from typing import Optional

try:
    from rich.console import Console
    HAS_RICH = True
except ImportError:
    HAS_RICH = False
    Console = None  # type: ignore


console: Optional[Console] = Console() if HAS_RICH else None


def extract_zip(archive_path: Path, extract_to: Path, verbose: bool = False) -> bool:
    """Extract a ZIP archive.

    Args:
        archive_path: Path to ZIP archive
        extract_to: Directory to extract to
        verbose: Print detailed output

    Returns:
        True if successful, False otherwise
    """
    extract_to.mkdir(parents=True, exist_ok=True)

    try:
        if verbose:
            msg = f"📦 Extracting {archive_path.name} -> {extract_to.name}"
            if HAS_RICH and console:
                console.print(msg, style="cyan")
            else:
                print(msg)

        with zipfile.ZipFile(archive_path, "r") as zip_ref:
            zip_ref.extractall(extract_to)

        if verbose:
            msg = f"✅ Extracted: {archive_path.name}"
            if HAS_RICH and console:
                console.print(msg, style="green")
            else:
                print(msg)
        return True
    except zipfile.BadZipFile as e:
        msg = f"❌ Invalid ZIP file: {e}"
        if HAS_RICH and console:
            console.print(msg, style="red")
        else:
            print(msg)
        return False
    except OSError as e:
        msg = f"❌ Extraction failed: {e}"
        if HAS_RICH and console:
            console.print(msg, style="red")
        else:
            print(msg)
        return False


def extract_tar_gz(archive_path: Path, extract_to: Path, verbose: bool = False) -> bool:
    """Extract a tar.gz archive.

    Args:
        archive_path: Path to tar.gz archive
        extract_to: Directory to extract to
        verbose: Print detailed output

    Returns:
        True if successful, False otherwise
    """
    extract_to.mkdir(parents=True, exist_ok=True)

    try:
        if verbose:
            msg = f"📦 Extracting {archive_path.name} -> {extract_to.name}"
            if HAS_RICH and console:
                console.print(msg, style="cyan")
            else:
                print(msg)

        subprocess.run(
            ["tar", "-xzf", str(archive_path), "-C", str(extract_to), "--strip-components=1"],
            check=True,
            capture_output=True,
            text=True,
        )

        if verbose:
            msg = f"✅ Extracted: {archive_path.name}"
            if HAS_RICH and console:
                console.print(msg, style="green")
            else:
                print(msg)
        return True
    except subprocess.CalledProcessError as e:
        msg = f"❌ Extraction failed: {e}"
        if e.stderr:
            msg += f"\nStderr: {e.stderr}"
        if HAS_RICH and console:
            console.print(msg, style="red")
        else:
            print(msg)
        return False


def extract_tar(archive_path: Path, extract_to: Path, verbose: bool = False) -> bool:
    """Extract a tar archive (uncompressed).

    Args:
        archive_path: Path to tar archive
        extract_to: Directory to extract to
        verbose: Print detailed output

    Returns:
        True if successful, False otherwise
    """
    extract_to.mkdir(parents=True, exist_ok=True)

    try:
        if verbose:
            msg = f"📦 Extracting {archive_path.name} -> {extract_to.name}"
            if HAS_RICH and console:
                console.print(msg, style="cyan")
            else:
                print(msg)

        subprocess.run(
            ["tar", "-xf", str(archive_path), "-C", str(extract_to), "--strip-components=1"],
            check=True,
            capture_output=True,
            text=True,
        )

        if verbose:
            msg = f"✅ Extracted: {archive_path.name}"
            if HAS_RICH and console:
                console.print(msg, style="green")
            else:
                print(msg)
        return True
    except subprocess.CalledProcessError as e:
        msg = f"❌ Extraction failed: {e}"
        if e.stderr:
            msg += f"\nStderr: {e.stderr}"
        if HAS_RICH and console:
            console.print(msg, style="red")
        else:
            print(msg)
        return False
