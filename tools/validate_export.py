#!/usr/bin/env python3
"""
Validate and visualize exported teacher data from HDF5 files.

Usage:
    python tools/validate_export.py /path/to/teacher_data/room1.h5
    python tools/validate_export.py /path/to/teacher_data/ --all
"""

import argparse
import sys
from pathlib import Path
from typing import Optional

import h5py
import numpy as np

try:
    import matplotlib.pyplot as plt
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False


class ExportValidator:
    """Validates teacher export HDF5 files."""

    REQUIRED_FIELDS = {
        'frame_id': (),
        'timestamp': (),
        'left_image': (512, 512),
        'right_image': (512, 512),
        'left_downscaled': (256, 256),
        'right_downscaled': (256, 256),
        'imu_preintegration': (15,),
        'flow_grid': None,  # Variable length
        'pose_world_cam': (4, 4),
        'pose_covariance': (6, 6),
        'depth_map': (512, 512),
        'depth_confidence': (512, 512),
        'reprojection_errors': None,  # Variable length
    }

    def __init__(self, hdf5_path: str, verbose: bool = False):
        self.hdf5_path = Path(hdf5_path)
        self.verbose = verbose
        self.stats = {}

    def validate(self) -> bool:
        """Validate HDF5 file structure and contents."""
        if not self.hdf5_path.exists():
            print(f"❌ File not found: {self.hdf5_path}")
            return False

        try:
            with h5py.File(self.hdf5_path, 'r') as f:
                if 'frames' not in f:
                    print("❌ Missing 'frames' group in HDF5 file")
                    return False

                frames_group = f['frames']
                num_frames = len(frames_group)

                if num_frames == 0:
                    print("⚠️  No frames found in HDF5 file")
                    return False

                print(f"✅ Found {num_frames} frames")

                # Check first frame structure
                first_frame_key = sorted(frames_group.keys())[0]
                first_frame = frames_group[first_frame_key]

                missing_fields = []
                for field, expected_shape in self.REQUIRED_FIELDS.items():
                    if field not in first_frame:
                        missing_fields.append(field)
                        print(f"❌ Missing field: {field}")
                    elif expected_shape is not None:
                        actual_shape = first_frame[field].shape
                        if actual_shape != expected_shape:
                            print(f"⚠️  Field '{field}' has shape {actual_shape}, expected {expected_shape}")

                if missing_fields:
                    return False

                print("✅ All required fields present in first frame")

                # Collect statistics
                self._collect_statistics(frames_group)

                return True

        except Exception as e:
            print(f"❌ Error reading HDF5 file: {e}")
            return False

    def _collect_statistics(self, frames_group) -> None:
        """Collect statistics from all frames."""
        frame_ids = []
        timestamps = []
        image_means = []
        depth_valid_percentages = []
        imu_norms = []

        for frame_key in sorted(frames_group.keys()):
            frame = frames_group[frame_key]

            frame_ids.append(frame['frame_id'][()])
            timestamps.append(frame['timestamp'][()])

            # Image statistics
            left_img = frame['left_image'][:]
            image_means.append(np.mean(left_img))

            # Depth statistics
            depth = frame['depth_map'][:]
            valid_depth = np.sum(depth > 0)
            total_pixels = depth.size
            valid_percentage = (valid_depth / total_pixels) * 100
            depth_valid_percentages.append(valid_percentage)

            # IMU statistics
            imu = frame['imu_preintegration'][:]
            imu_norms.append(np.linalg.norm(imu))

        self.stats = {
            'num_frames': len(frame_ids),
            'frame_ids': frame_ids,
            'timestamps': timestamps,
            'timestamp_range': (timestamps[0], timestamps[-1]),
            'timestamp_duration': timestamps[-1] - timestamps[0],
            'image_mean_avg': np.mean(image_means),
            'depth_coverage_avg': np.mean(depth_valid_percentages),
            'imu_norm_avg': np.mean(imu_norms),
        }

        if self.verbose:
            self._print_statistics()

    def _print_statistics(self) -> None:
        """Print collected statistics."""
        stats = self.stats

        print("\n" + "="*60)
        print("DATASET STATISTICS")
        print("="*60)
        print(f"Total frames:              {stats['num_frames']}")
        print(f"Frame ID range:            {stats['frame_ids'][0]} - {stats['frame_ids'][-1]}")
        print(f"Timestamp range:           {stats['timestamp_range'][0]:.3f} - {stats['timestamp_range'][1]:.3f}s")
        print(f"Duration:                  {stats['timestamp_duration']:.2f}s")
        print(f"Mean image intensity:      {stats['image_mean_avg']:.1f}")
        print(f"Avg depth coverage:        {stats['depth_coverage_avg']:.1f}%")
        print(f"Avg IMU norm:              {stats['imu_norm_avg']:.3f}")
        print("="*60 + "\n")

    def visualize_samples(self, num_samples: int = 3, output_dir: Optional[str] = None) -> None:
        """Visualize sample frames from the dataset."""
        if not MATPLOTLIB_AVAILABLE:
            print("⚠️  matplotlib not available, skipping visualization")
            return

        with h5py.File(self.hdf5_path, 'r') as f:
            frames_group = f['frames']
            frame_keys = sorted(frames_group.keys())

            # Sample evenly distributed frames
            indices = np.linspace(0, len(frame_keys) - 1, num_samples, dtype=int)

            fig, axes = plt.subplots(num_samples, 4, figsize=(14, 3*num_samples))
            if num_samples == 1:
                axes = axes.reshape(1, -1)

            for row, idx in enumerate(indices):
                frame_key = frame_keys[idx]
                frame = frames_group[frame_key]

                # Left image
                left_img = frame['left_image'][:]
                axes[row, 0].imshow(left_img, cmap='gray')
                axes[row, 0].set_title(f"Left (Frame {frame['frame_id'][()]})")
                axes[row, 0].axis('off')

                # Right image
                right_img = frame['right_image'][:]
                axes[row, 1].imshow(right_img, cmap='gray')
                axes[row, 1].set_title("Right")
                axes[row, 1].axis('off')

                # Depth map
                depth = frame['depth_map'][:]
                depth_valid = np.where(depth > 0, depth, np.nan)
                im = axes[row, 2].imshow(depth_valid, cmap='viridis')
                axes[row, 2].set_title("Depth Map")
                axes[row, 2].axis('off')
                plt.colorbar(im, ax=axes[row, 2])

                # Depth confidence
                confidence = frame['depth_confidence'][:]
                axes[row, 3].imshow(confidence, cmap='hot')
                axes[row, 3].set_title("Depth Confidence")
                axes[row, 3].axis('off')

            plt.tight_layout()

            if output_dir:
                output_path = Path(output_dir) / f"{self.hdf5_path.stem}_samples.png"
                plt.savefig(output_path, dpi=100, bbox_inches='tight')
                print(f"✅ Saved visualization to {output_path}")
            else:
                plt.show()

    def report(self) -> str:
        """Generate a validation report."""
        report = f"""
================================================================================
TEACHER DATA VALIDATION REPORT
================================================================================
File: {self.hdf5_path}

Dataset Statistics:
  Total Frames:       {self.stats.get('num_frames', 'N/A')}
  Duration:           {self.stats.get('timestamp_duration', 'N/A'):.2f}s
  Avg Depth Coverage: {self.stats.get('depth_coverage_avg', 'N/A'):.1f}%
  Avg IMU Norm:       {self.stats.get('imu_norm_avg', 'N/A'):.3f}

Status: ✅ VALID
================================================================================
"""
        return report


def main():
    parser = argparse.ArgumentParser(description="Validate teacher export data")
    parser.add_argument('path', help="Path to HDF5 file or directory")
    parser.add_argument('--all', action='store_true', help="Validate all files in directory")
    parser.add_argument('--visualize', action='store_true', help="Visualize sample frames")
    parser.add_argument('--samples', type=int, default=3, help="Number of samples to visualize")
    parser.add_argument('--output-dir', help="Directory to save visualizations")
    parser.add_argument('-v', '--verbose', action='store_true', help="Verbose output")

    args = parser.parse_args()

    path = Path(args.path)

    if path.is_dir() and args.all:
        hdf5_files = list(path.glob("*.h5"))
        if not hdf5_files:
            print(f"No HDF5 files found in {path}")
            return 1

        print(f"Found {len(hdf5_files)} HDF5 files\n")

        valid_count = 0
        for hdf5_file in sorted(hdf5_files):
            print(f"Validating {hdf5_file.name}...", end=" ")
            validator = ExportValidator(str(hdf5_file), verbose=args.verbose)
            if validator.validate():
                print("✅")
                valid_count += 1
                if args.visualize:
                    validator.visualize_samples(args.samples, args.output_dir)
            else:
                print("❌")

        print(f"\n{valid_count}/{len(hdf5_files)} files valid")
        return 0 if valid_count == len(hdf5_files) else 1

    elif path.is_file() or path.suffix == '.h5':
        validator = ExportValidator(str(path), verbose=args.verbose)
        if validator.validate():
            print(validator.report())
            if args.visualize:
                validator.visualize_samples(args.samples, args.output_dir)
            return 0
        else:
            return 1
    else:
        print(f"Invalid path: {path}")
        return 1


if __name__ == '__main__':
    sys.exit(main())
