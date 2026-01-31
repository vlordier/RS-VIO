#!/usr/bin/env python3
"""
Simple test script to verify teacher export data can be generated
using a CSV-based approach instead of HDF5.
"""

import argparse
import csv
import json
import logging
from dataclasses import dataclass
from pathlib import Path

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


@dataclass
class ExportConfig:
    """Configuration for teacher data export"""
    dataset_path: Path
    output_dir: Path
    sequence_name: str = "tum_vi"
    enable_csv: bool = True
    enable_images: bool = False


class TeacherExporter:
    """Export teacher frames to CSV and optionally images"""

    def __init__(self, config: ExportConfig):
        self.config = config
        self.output_dir = Path(config.output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self.frame_count = 0
        self.csv_file = None
        self.csv_writer = None

        # Open CSV file for writing
        if config.enable_csv:
            csv_path = self.output_dir / f"{config.sequence_name}.csv"
            self.csv_file = open(csv_path, 'w', newline='')

            # Write header
            fieldnames = [
                'frame_id', 'timestamp',
                'pose_tx', 'pose_ty', 'pose_tz',
                'pose_qx', 'pose_qy', 'pose_qz', 'pose_qw',
                'velocity_x', 'velocity_y', 'velocity_z',
                'imu_count', 'flow_inlier_ratio',
                'mean_reproj_error', 'ba_iterations', 'ba_converged',
                'depth_coverage', 'image_intensity_mean'
            ]
            self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=fieldnames)
            self.csv_writer.writeheader()
            logger.info(f"Opened CSV export: {csv_path}")

    def export_frame(self, frame_data: dict) -> None:
        """Export a single frame to CSV"""
        if self.csv_writer is None:
            return

        try:
            row = {
                'frame_id': frame_data.get('frame_id', self.frame_count),
                'timestamp': frame_data.get('timestamp', 0.0),
                'pose_tx': frame_data.get('pose', {}).get('tx', 0.0),
                'pose_ty': frame_data.get('pose', {}).get('ty', 0.0),
                'pose_tz': frame_data.get('pose', {}).get('tz', 0.0),
                'pose_qx': frame_data.get('pose', {}).get('qx', 0.0),
                'pose_qy': frame_data.get('pose', {}).get('qy', 0.0),
                'pose_qz': frame_data.get('pose', {}).get('qz', 0.0),
                'pose_qw': frame_data.get('pose', {}).get('qw', 1.0),
                'velocity_x': frame_data.get('velocity', [0, 0, 0])[0],
                'velocity_y': frame_data.get('velocity', [0, 0, 0])[1],
                'velocity_z': frame_data.get('velocity', [0, 0, 0])[2],
                'imu_count': frame_data.get('imu_sample_count', 0),
                'flow_inlier_ratio': frame_data.get('flow_inlier_ratio', 0.0),
                'mean_reproj_error': frame_data.get('mean_reproj_error', 0.0),
                'ba_iterations': frame_data.get('ba_iterations', 0),
                'ba_converged': 1 if frame_data.get('ba_converged', False) else 0,
                'depth_coverage': frame_data.get('depth_coverage', 0.0),
                'image_intensity_mean': frame_data.get('image_intensity_mean', 0.0),
            }
            self.csv_writer.writerow(row)
            self.frame_count += 1

            if self.frame_count % 100 == 0:
                logger.info(f"Exported {self.frame_count} frames to CSV")
                self.csv_file.flush()

        except Exception as e:
            logger.error(f"Error exporting frame: {e}")

    def finalize(self) -> dict:
        """Finalize export and return statistics"""
        if self.csv_file:
            self.csv_file.close()

        stats = {
            'frames_exported': self.frame_count,
            'output_dir': str(self.output_dir),
            'sequence_name': self.config.sequence_name,
            'format': 'CSV',
        }

        logger.info(f"✅ Teacher export complete: {self.frame_count} frames")

        # Write statistics
        stats_file = self.output_dir / f"{self.config.sequence_name}_stats.json"
        with open(stats_file, 'w') as f:
            json.dump(stats, f, indent=2)
        logger.info(f"Statistics saved to: {stats_file}")

        return stats


def generate_mock_export(output_dir: Path, sequence_name: str, num_frames: int = 100):
    """Generate mock teacher data for testing"""
    logger.info(f"Generating {num_frames} mock frames to {output_dir}")

    exporter = TeacherExporter(ExportConfig(
        dataset_path=Path("/data"),
        output_dir=output_dir,
        sequence_name=sequence_name,
    ))

    # Generate mock frames
    for i in range(num_frames):
        frame_data = {
            'frame_id': i,
            'timestamp': i * 0.033,  # ~30 FPS
            'pose': {
                'tx': 0.1 * i * 0.001,
                'ty': 0.0,
                'tz': 0.0,
                'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0,
            },
            'velocity': [0.01 * i, 0.0, 0.0],
            'imu_sample_count': 10,
            'flow_inlier_ratio': 0.8 + (i % 20) * 0.01,
            'mean_reproj_error': 0.5 + (i % 10) * 0.05,
            'ba_iterations': 20,
            'ba_converged': True,
            'depth_coverage': 0.75 + (i % 20) * 0.01,
            'image_intensity_mean': 128.0,
        }
        exporter.export_frame(frame_data)

    return exporter.finalize()


def main():
    parser = argparse.ArgumentParser(
        description="Teacher data export test tool"
    )
    parser.add_argument(
        'output_dir',
        type=Path,
        help="Output directory for exported data"
    )
    parser.add_argument(
        '--sequence-name',
        default='test',
        help="Sequence name (default: test)"
    )
    parser.add_argument(
        '--num-frames',
        type=int,
        default=100,
        help="Number of mock frames to generate (default: 100)"
    )
    parser.add_argument(
        '--verbose', '-v',
        action='store_true',
        help="Enable verbose logging"
    )

    args = parser.parse_args()

    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)

    # Generate mock data
    stats = generate_mock_export(args.output_dir, args.sequence_name, args.num_frames)
    print(f"\n{'='*60}")
    print(f"Export Complete: {stats['frames_exported']} frames")
    print(f"Output: {stats['output_dir']}")
    print(f"{'='*60}")


if __name__ == "__main__":
    main()
