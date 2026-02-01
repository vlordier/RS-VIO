#!/usr/bin/env python3
"""
Tests for setup_datasets.py module.

Tests the DatasetSetup class functionality including:
- Checksum loading and verification
- Archive extraction
- Dataset validation
- Directory structure checks
"""

import hashlib
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
import sys

# Add scripts directory to path
sys.path.insert(0, str(Path(__file__).parent.parent / "scripts"))

from setup_datasets import DatasetSetup


class TestDatasetSetupInit(unittest.TestCase):
    """Test DatasetSetup initialization."""

    def test_init_creates_dir(self):
        """Test that __init__ creates datasets directory."""
        with tempfile.TemporaryDirectory() as tmpdir:
            datasets_dir = Path(tmpdir) / "datasets"
            setup = DatasetSetup(datasets_dir)

            self.assertTrue(datasets_dir.exists())
            self.assertEqual(setup.datasets_dir, datasets_dir)

    def test_init_loads_checksums(self):
        """Test that __init__ loads checksums file."""
        with tempfile.TemporaryDirectory() as tmpdir:
            tmppath = Path(tmpdir)
            checksum_file = tmppath / "checksums.sha256"

            # Create checksum file
            checksum_file.write_text("abc123  file1.zip\ndef456  file2.tgz\n")

            setup = DatasetSetup(tmppath / "datasets", checksum_file)

            self.assertEqual(setup.checksums["file1.zip"], "abc123")
            self.assertEqual(setup.checksums["file2.tgz"], "def456")

    def test_init_missing_checksum_file(self):
        """Test initialization with missing checksum file."""
        with tempfile.TemporaryDirectory() as tmpdir:
            tmppath = Path(tmpdir)
            setup = DatasetSetup(tmppath / "datasets")

            self.assertEqual(setup.checksums, {})


class TestDatasetSetupChecksums(unittest.TestCase):
    """Test checksum functionality."""

    def test_compute_checksum(self):
        """Test SHA256 checksum computation."""
        with tempfile.TemporaryDirectory() as tmpdir:
            tmppath = Path(tmpdir)
            setup = DatasetSetup(tmppath / "datasets")

            # Create a test file
            test_file = tmppath / "test.txt"
            test_file.write_text("test content")

            # Compute checksum
            checksum = setup.compute_checksum(test_file)

            # Verify it matches expected SHA256
            expected = hashlib.sha256(b"test content").hexdigest()
            self.assertEqual(checksum, expected)

    def test_verify_checksum_success(self):
        """Test successful checksum verification."""
        with tempfile.TemporaryDirectory() as tmpdir:
            tmppath = Path(tmpdir)
            test_file = tmppath / "test.txt"
            test_file.write_text("test content")

            expected_sha256 = hashlib.sha256(b"test content").hexdigest()

            checksum_file = tmppath / "checksums.sha256"
            checksum_file.write_text(f"{expected_sha256}  test.txt\n")

            setup = DatasetSetup(tmppath / "datasets", checksum_file)
            result = setup.verify_checksum(test_file)

            self.assertTrue(result)

    def test_verify_checksum_mismatch(self):
        """Test checksum mismatch."""
        with tempfile.TemporaryDirectory() as tmpdir:
            tmppath = Path(tmpdir)
            test_file = tmppath / "test.txt"
            test_file.write_text("test content")

            checksum_file = tmppath / "checksums.sha256"
            checksum_file.write_text("wronghash123  test.txt\n")

            setup = DatasetSetup(tmppath / "datasets", checksum_file)
            result = setup.verify_checksum(test_file)

            self.assertFalse(result)

    def test_verify_checksum_missing_entry(self):
        """Test checksum verification with missing file entry."""
        with tempfile.TemporaryDirectory() as tmpdir:
            tmppath = Path(tmpdir)
            test_file = tmppath / "test.txt"
            test_file.write_text("test content")

            checksum_file = tmppath / "checksums.sha256"
            checksum_file.write_text("abc123  other.zip\n")

            setup = DatasetSetup(tmppath / "datasets", checksum_file)
            result = setup.verify_checksum(test_file)

            # Should return True (skip verification) when entry not found
            self.assertTrue(result)


class TestDatasetSetupTools(unittest.TestCase):
    """Test tool availability checking."""

    @patch("shutil.which")
    def test_check_tools_success(self, mock_which):
        """Test successful tool check."""
        mock_which.return_value = "/usr/bin/curl"

        with tempfile.TemporaryDirectory() as tmpdir:
            setup = DatasetSetup(Path(tmpdir))
            result = setup.check_tools()

            self.assertTrue(result)

    @patch("shutil.which")
    def test_check_tools_missing(self, mock_which):
        """Test missing tools."""
        mock_which.return_value = None

        with tempfile.TemporaryDirectory() as tmpdir:
            setup = DatasetSetup(Path(tmpdir))
            result = setup.check_tools()

            self.assertFalse(result)


class TestDatasetSetupExtraction(unittest.TestCase):
    """Test archive extraction."""

    def test_extract_zip(self):
        """Test ZIP extraction."""
        import zipfile

        with tempfile.TemporaryDirectory() as tmpdir:
            tmppath = Path(tmpdir)
            zip_file = tmppath / "test.zip"
            extract_dir = tmppath / "extracted"

            # Create test ZIP
            with zipfile.ZipFile(zip_file, "w") as zf:
                zf.writestr("file.txt", "content")

            setup = DatasetSetup(tmppath / "datasets")
            result = setup.extract_zip(zip_file, extract_dir)

            self.assertTrue(result)
            self.assertTrue((extract_dir / "file.txt").exists())

    @patch("subprocess.run")
    def test_extract_tar_gz(self, mock_run):
        """Test tar.gz extraction."""
        mock_run.return_value = None

        with tempfile.TemporaryDirectory() as tmpdir:
            tmppath = Path(tmpdir)
            archive = tmppath / "test.tar.gz"
            extract_dir = tmppath / "extracted"

            archive.touch()

            setup = DatasetSetup(tmppath / "datasets")
            result = setup.extract_tar_gz(archive, extract_dir)

            self.assertTrue(result)
            mock_run.assert_called_once()


class TestDatasetSetupValidation(unittest.TestCase):
    """Test dataset validation."""

    def test_check_euroc_found(self):
        """Test EuROC dataset detection."""
        with tempfile.TemporaryDirectory() as tmpdir:
            tmppath = Path(tmpdir)
            datasets_dir = tmppath / "datasets"
            datasets_dir.mkdir()

            # Create EuROC structure
            euroc_dir = datasets_dir / "euroc"
            euroc_dir.mkdir()
            cam0_data = euroc_dir / "MH_01_easy" / "cam0" / "data"
            cam0_data.mkdir(parents=True)
            (cam0_data / "frame1.png").touch()
            (cam0_data / "frame2.png").touch()

            setup = DatasetSetup(datasets_dir)
            ok, msg = setup.check_euroc()

            self.assertTrue(ok)
            self.assertIn("✅", msg)

    def test_check_euroc_not_found(self):
        """Test EuROC dataset not found."""
        with tempfile.TemporaryDirectory() as tmpdir:
            tmppath = Path(tmpdir)
            datasets_dir = tmppath / "datasets"
            datasets_dir.mkdir()

            setup = DatasetSetup(datasets_dir)
            ok, msg = setup.check_euroc()

            self.assertFalse(ok)
            self.assertIn("⏭️", msg)

    def test_check_tum_found(self):
        """Test TUM-VI dataset detection."""
        with tempfile.TemporaryDirectory() as tmpdir:
            tmppath = Path(tmpdir)
            datasets_dir = tmppath / "datasets"
            datasets_dir.mkdir()

            # Create TUM structure
            tum_dir = datasets_dir / "tum_vi"
            rgb_dir = tum_dir / "rgb"
            rgb_dir.mkdir(parents=True)
            (rgb_dir / "frame1.png").touch()
            (rgb_dir / "frame2.png").touch()

            setup = DatasetSetup(datasets_dir)
            ok, msg = setup.check_tum()

            self.assertTrue(ok)
            self.assertIn("✅", msg)

    def test_check_4seasons_found(self):
        """Test 4Seasons dataset detection."""
        with tempfile.TemporaryDirectory() as tmpdir:
            tmppath = Path(tmpdir)
            datasets_dir = tmppath / "datasets"
            datasets_dir.mkdir()

            # Create 4Seasons structure
            seasons_dir = datasets_dir / "4seasons"
            seasons_dir.mkdir()
            (seasons_dir / "recording1").mkdir()

            setup = DatasetSetup(datasets_dir)
            ok, msg = setup.check_4seasons()

            self.assertTrue(ok)
            self.assertIn("✅", msg)

    def test_check_4seasons_not_found(self):
        """Test 4Seasons dataset not found."""
        with tempfile.TemporaryDirectory() as tmpdir:
            tmppath = Path(tmpdir)
            datasets_dir = tmppath / "datasets"
            datasets_dir.mkdir()

            setup = DatasetSetup(datasets_dir)
            ok, msg = setup.check_4seasons()

            self.assertFalse(ok)
            self.assertIn("⏭️", msg)


class TestDatasetSetupVerification(unittest.TestCase):
    """Test dataset verification."""

    def test_verify_all_none_available(self):
        """Test verification when no datasets available."""
        with tempfile.TemporaryDirectory() as tmpdir:
            tmppath = Path(tmpdir)
            datasets_dir = tmppath / "datasets"
            datasets_dir.mkdir()

            setup = DatasetSetup(datasets_dir)
            result = setup.verify_all()

            # Should return False when no datasets found
            self.assertFalse(result)

    def test_verify_all_some_available(self):
        """Test verification when some datasets available."""
        with tempfile.TemporaryDirectory() as tmpdir:
            tmppath = Path(tmpdir)
            datasets_dir = tmppath / "datasets"
            datasets_dir.mkdir()

            # Create TUM structure
            tum_dir = datasets_dir / "tum_vi"
            rgb_dir = tum_dir / "rgb"
            rgb_dir.mkdir(parents=True)
            (rgb_dir / "frame.png").touch()

            setup = DatasetSetup(datasets_dir)
            result = setup.verify_all()

            # Should return True when at least one dataset found
            self.assertTrue(result)


if __name__ == "__main__":
    unittest.main()
