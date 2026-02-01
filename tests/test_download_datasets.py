#!/usr/bin/env python3
"""
Tests for download_datasets.py module.

Tests the DatasetDownloader class functionality including:
- File downloading with retry logic
- Archive extraction
- Dataset availability checks
"""

import tempfile
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch, mock_open
import sys

# Add scripts directory to path
sys.path.insert(0, str(Path(__file__).parent.parent / "scripts"))

from download_datasets import DatasetDownloader


class TestDatasetDownloaderInit(unittest.TestCase):
    """Test DatasetDownloader initialization."""

    def test_init_creates_target_dir(self):
        """Test that __init__ creates target directory."""
        with tempfile.TemporaryDirectory() as tmpdir:
            target = Path(tmpdir) / "datasets"
            downloader = DatasetDownloader(target)

            self.assertTrue(target.exists())
            self.assertEqual(downloader.target_dir, target)

    def test_init_with_existing_dir(self):
        """Test initialization with existing directory."""
        with tempfile.TemporaryDirectory() as tmpdir:
            downloader = DatasetDownloader(Path(tmpdir))
            self.assertTrue(Path(tmpdir).exists())


class TestDatasetDownloaderDependencies(unittest.TestCase):
    """Test dependency checking."""

    @patch("shutil.which")
    def test_check_dependencies_success(self, mock_which):
        """Test successful dependency check."""
        mock_which.return_value = "/usr/bin/curl"

        with tempfile.TemporaryDirectory() as tmpdir:
            downloader = DatasetDownloader(Path(tmpdir))
            result = downloader.check_dependencies()

            self.assertTrue(result)

    @patch("shutil.which")
    def test_check_dependencies_missing(self, mock_which):
        """Test missing dependencies."""
        mock_which.return_value = None

        with tempfile.TemporaryDirectory() as tmpdir:
            downloader = DatasetDownloader(Path(tmpdir))
            result = downloader.check_dependencies()

            self.assertFalse(result)


class TestDatasetDownloaderDownload(unittest.TestCase):
    """Test file downloading."""

    @patch("urllib.request.urlretrieve")
    def test_download_file_success(self, mock_urlretrieve):
        """Test successful file download."""
        mock_urlretrieve.return_value = None

        with tempfile.TemporaryDirectory() as tmpdir:
            downloader = DatasetDownloader(Path(tmpdir))
            output_path = Path(tmpdir) / "test.zip"
            output_path.touch()  # Create the file so it passes

            result = downloader.download_file("http://example.com/test.zip", output_path)

            self.assertTrue(result)

    @patch("time.sleep")
    @patch("urllib.request.urlretrieve")
    def test_download_file_retry(self, mock_urlretrieve, mock_sleep):
        """Test download with retries on failure."""
        # First two calls fail, third succeeds
        def side_effect(url, path):
            if not Path(path).exists():
                raise Exception("Network error")

        mock_urlretrieve.side_effect = side_effect

        with tempfile.TemporaryDirectory() as tmpdir:
            downloader = DatasetDownloader(Path(tmpdir))
            output_path = Path(tmpdir) / "test.zip"

            # Mock to create file on last attempt
            call_count = [0]
            def create_file_on_third(url, path):
                call_count[0] += 1
                if call_count[0] == 3:
                    Path(path).touch()
                else:
                    raise Exception("Network error")

            mock_urlretrieve.side_effect = create_file_on_third

            result = downloader.download_file("http://example.com/test.zip", output_path, max_retries=3)

            self.assertTrue(result)
            self.assertEqual(call_count[0], 3)

    @patch("urllib.request.urlretrieve")
    def test_download_file_failure(self, mock_urlretrieve):
        """Test download failure after retries."""
        mock_urlretrieve.side_effect = Exception("Network error")

        with tempfile.TemporaryDirectory() as tmpdir:
            downloader = DatasetDownloader(Path(tmpdir))
            output_path = Path(tmpdir) / "test.zip"

            result = downloader.download_file("http://example.com/test.zip", output_path, max_retries=2)

            self.assertFalse(result)
            self.assertEqual(mock_urlretrieve.call_count, 2)


class TestDatasetDownloaderExtraction(unittest.TestCase):
    """Test archive extraction."""

    def test_extract_zip(self):
        """Test ZIP extraction."""
        with tempfile.TemporaryDirectory() as tmpdir:
            # Create a test ZIP file
            import zipfile
            tmppath = Path(tmpdir)
            zip_file = tmppath / "test.zip"
            extract_dir = tmppath / "extracted"

            with zipfile.ZipFile(zip_file, "w") as zf:
                zf.writestr("test.txt", "test content")

            downloader = DatasetDownloader(tmppath)
            result = downloader.extract_zip(zip_file, extract_dir)

            self.assertTrue(result)
            self.assertTrue((extract_dir / "test.txt").exists())

    @patch("subprocess.run")
    def test_extract_tar_gz(self, mock_run):
        """Test tar.gz extraction."""
        mock_run.return_value = None

        with tempfile.TemporaryDirectory() as tmpdir:
            tmppath = Path(tmpdir)
            archive = tmppath / "test.tar.gz"
            extract_dir = tmppath / "extracted"

            # Create dummy archive file
            archive.touch()

            downloader = DatasetDownloader(tmppath)
            result = downloader.extract_tar_gz(archive, extract_dir)

            self.assertTrue(result)
            mock_run.assert_called_once()


class TestDatasetDownloaderEuROC(unittest.TestCase):
    """Test EuROC dataset download."""

    def test_euroc_manual_file_exists(self):
        """Test EuROC when manual file exists."""
        with tempfile.TemporaryDirectory() as tmpdir:
            tmppath = Path(tmpdir)
            manual_file = tmppath / "MH_01_easy.zip"
            manual_file.touch()

            downloader = DatasetDownloader(tmppath)
            
            # Mock the config to use our temp file
            with patch("download_datasets.DATASETS", {
                "euroc": {"name": "EuRoC", "url": "http://example.com", "local_path": str(manual_file)}
            }):
                with patch.object(downloader, "extract_zip", return_value=True):
                    result = downloader.download_euroc()

            self.assertTrue(result)

    def test_euroc_manual_file_missing(self):
        """Test EuROC when manual file is missing."""
        with tempfile.TemporaryDirectory() as tmpdir:
            tmppath = Path(tmpdir)
            downloader = DatasetDownloader(tmppath)

            with patch("download_datasets.DATASETS", {
                "euroc": {"name": "EuRoC", "url": "http://example.com", "local_path": "/nonexistent/MH_01_easy.zip"}
            }):
                result = downloader.download_euroc()

            # Should return False but not crash
            self.assertFalse(result)


class TestDatasetDownloaderTUM(unittest.TestCase):
    """Test TUM-VI dataset download."""

    @patch("pathlib.Path.unlink")
    @patch.object(DatasetDownloader, "extract_tar", return_value=True)
    @patch.object(DatasetDownloader, "download_file", return_value=True)
    def test_tum_download_success(self, mock_download, mock_extract, mock_unlink):
        """Test successful TUM download."""
        with tempfile.TemporaryDirectory() as tmpdir:
            downloader = DatasetDownloader(Path(tmpdir))
            result = downloader.download_tum()

            self.assertTrue(result)
            mock_download.assert_called()
            mock_extract.assert_called()

    @patch.object(DatasetDownloader, "download_file", return_value=False)
    def test_tum_download_failure(self, mock_download):
        """Test TUM download failure."""
        with tempfile.TemporaryDirectory() as tmpdir:
            downloader = DatasetDownloader(Path(tmpdir))
            result = downloader.download_tum()

            self.assertFalse(result)


class TestDatasetDownloader4Seasons(unittest.TestCase):
    """Test 4Seasons dataset check."""

    def test_4seasons_not_found(self):
        """Test 4Seasons when directory doesn't exist."""
        with tempfile.TemporaryDirectory() as tmpdir:
            downloader = DatasetDownloader(Path(tmpdir))
            result = downloader.download_4seasons()

            self.assertFalse(result)

    def test_4seasons_found(self):
        """Test 4Seasons when directory exists."""
        with tempfile.TemporaryDirectory() as tmpdir:
            tmppath = Path(tmpdir)
            seasons_dir = tmppath / "4seasons"
            seasons_dir.mkdir()
            (seasons_dir / "recording1").mkdir()

            downloader = DatasetDownloader(tmppath)
            result = downloader.download_4seasons()

            self.assertTrue(result)


if __name__ == "__main__":
    unittest.main()
