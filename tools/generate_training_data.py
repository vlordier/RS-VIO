#!/usr/bin/env python3
"""
Generate complete training metadata with all required fields for student network.

Outputs JSON metadata that includes:
- Stereo images (left/right)
- IMU preintegration + covariance
- Optical flow + quality
- Depth map + implicit confidence
- Feature matches + quality scores
- Previous pose/velocity + time-since-keyframe
- Target pose + uncertainty
"""

import json
import logging
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Dict, List

import numpy as np

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


@dataclass
class FrameMetadata:
    """Complete frame metadata for training"""
    frame_id: int
    timestamp: float

    # Images
    image_files: Dict[str, str]

    # IMU: preintegration (15D) + covariance diagonal (15D)
    imu_preintegration: List[float]  # [dpos_x, dpos_y, dpos_z, dvel_x, dvel_y, dvel_z, drot_x, drot_y, drot_z, bias_accel_x, bias_accel_y, bias_accel_z, bias_gyro_x, bias_gyro_y, bias_gyro_z]
    imu_covariance: List[float]      # 15D diagonal of preint covariance

    # Optical flow: 8x6 grid flattened to 96D
    flow: List[float]                # Flattened [8, 6, 2] grid of flow vectors
    flow_quality: float              # Inlier ratio [0, 1]

    # Depth map
    depth_file: str                  # Path relative to depth_dir

    # Feature matches
    match_quality: List[float]       # Quality scores for each match [0, 1]

    # Previous state (for temporal context)
    previous_pose: List[float]       # [tx, ty, tz, qx, qy, qz, qw]
    previous_velocity: List[float]   # [vx, vy, vz]
    time_since_keyframe: float       # Seconds or frame count

    # Target pose (from bundle adjustment)
    pose_tx: float
    pose_ty: float
    pose_tz: float
    pose_qx: float
    pose_qy: float
    pose_qz: float
    pose_qw: float

    # Uncertainty estimate
    mean_reprojection_error: float   # RMS reprojection error


class TrainingDataGenerator:
    """Generate synthetic training data matching required metadata format"""

    def __init__(
        self,
        output_dir: Path,
        num_frames: int = 100,
        seed: int = 42,
    ):
        self.output_dir = Path(output_dir)
        self.num_frames = num_frames
        self.rng = np.random.RandomState(seed)

        # Create subdirectories
        self.image_dir = self.output_dir / 'images'
        self.depth_dir = self.output_dir / 'depth'
        self.image_dir.mkdir(parents=True, exist_ok=True)
        self.depth_dir.mkdir(parents=True, exist_ok=True)

        logger.info(f"Initialized generator for {num_frames} frames at {output_dir}")

    def generate_frame_metadata(self, frame_id: int) -> FrameMetadata:
        """Generate a single frame's metadata with all required fields"""
        timestamp = frame_id * 0.033  # ~30 Hz

        # Generate synthetic images
        left_img_name = f'frame_{frame_id:06d}_left.png'
        right_img_name = f'frame_{frame_id:06d}_right.png'
        self._generate_synthetic_image(self.image_dir / left_img_name)
        self._generate_synthetic_image(self.image_dir / right_img_name)

        # Generate synthetic depth
        depth_name = f'frame_{frame_id:06d}_depth.npy'
        self._generate_synthetic_depth(self.depth_dir / depth_name)

        # IMU preintegration (small random values)
        imu_preint = self.rng.randn(15).tolist()
        imu_cov = (np.ones(15) * 0.1 + self.rng.rand(15) * 0.05).tolist()

        # Optical flow (8x6x2 flattened)
        flow = self.rng.randn(96).tolist()
        flow_quality = float(self.rng.uniform(0.5, 1.0))

        # Feature matches (random number between 20-50 matches)
        num_matches = self.rng.randint(20, 51)
        match_quality = (self.rng.rand(num_matches) * 0.3 + 0.7).tolist()

        # Previous state (slowly changing trajectory)
        prev_pose = [
            float(frame_id * 0.01),  # slow drift in x
            0.0, 0.0,                # y, z
            0.0, 0.0, 0.0, 1.0,      # quat (identity)
        ]
        prev_velocity = [0.01, 0.0, 0.0]
        time_since_keyframe = float(frame_id % 10)  # Reset every 10 frames

        # Target pose (ground truth from BA)
        pose = self._interpolate_trajectory(frame_id)

        return FrameMetadata(
            frame_id=frame_id,
            timestamp=timestamp,
            image_files={
                'left_downscaled': left_img_name,
                'right_downscaled': right_img_name,
            },
            imu_preintegration=imu_preint,
            imu_covariance=imu_cov,
            flow=flow,
            flow_quality=flow_quality,
            depth_file=depth_name,
            match_quality=match_quality,
            previous_pose=prev_pose,
            previous_velocity=prev_velocity,
            time_since_keyframe=time_since_keyframe,
            pose_tx=pose[0],
            pose_ty=pose[1],
            pose_tz=pose[2],
            pose_qx=pose[3],
            pose_qy=pose[4],
            pose_qz=pose[5],
            pose_qw=pose[6],
            mean_reprojection_error=float(self.rng.uniform(0.1, 1.0)),
        )

    def _interpolate_trajectory(self, frame_id: int) -> List[float]:
        """Generate smooth ground truth trajectory"""
        t = frame_id * 0.033
        x = 0.1 * np.sin(0.5 * t)
        y = 0.05 * np.cos(0.3 * t)
        z = -0.02 * frame_id * 0.033

        # Small rotation
        angle = 0.01 * t
        qx = np.sin(angle / 2) * 0.1
        qy = 0.0
        qz = 0.0
        qw = np.cos(angle / 2)

        return [float(x), float(y), float(z), float(qx), float(qy), float(qz), float(qw)]

    def _generate_synthetic_image(self, path: Path) -> None:
        """Generate a dummy image file"""
        try:
            from PIL import Image
            img = Image.fromarray((self.rng.rand(256, 256) * 255).astype(np.uint8), mode='L')
            img.save(path)
        except ImportError:
            # Fallback: save as NPY
            np.save(path.with_suffix('.npy'), self.rng.rand(256, 256))

    def _generate_synthetic_depth(self, path: Path) -> None:
        """Generate a dummy depth map"""
        depth = (1.0 + self.rng.rand(256, 256) * 0.5).astype(np.float32)
        np.save(path, depth)

    def generate_all(self) -> Path:
        """Generate complete dataset"""
        logger.info(f"Generating {self.num_frames} frames...")

        frames_metadata = []
        for frame_id in range(self.num_frames):
            metadata = self.generate_frame_metadata(frame_id)
            frames_metadata.append(asdict(metadata))

            if (frame_id + 1) % 10 == 0:
                logger.info(f"  Generated {frame_id + 1}/{self.num_frames} frames")

        # Save metadata
        metadata_file = self.output_dir / 'metadata.json'
        with open(metadata_file, 'w') as f:
            json.dump(frames_metadata, f, indent=2)

        logger.info(f"Saved metadata to {metadata_file}")
        logger.info(f"Images: {self.image_dir}")
        logger.info(f"Depth: {self.depth_dir}")

        return metadata_file


def main():
    import argparse

    parser = argparse.ArgumentParser(
        description="Generate synthetic training data for student network"
    )
    parser.add_argument(
        '--output-dir',
        type=Path,
        default=Path('./training_data'),
        help='Output directory for generated data'
    )
    parser.add_argument(
        '--num-frames',
        type=int,
        default=100,
        help='Number of frames to generate'
    )
    parser.add_argument(
        '--seed',
        type=int,
        default=42,
        help='Random seed for reproducibility'
    )

    args = parser.parse_args()

    generator = TrainingDataGenerator(
        output_dir=args.output_dir,
        num_frames=args.num_frames,
        seed=args.seed,
    )

    metadata_file = generator.generate_all()
    print("\n✓ Training data generated successfully!")
    print(f"  Metadata: {metadata_file}")
    print("  Use this to train the network:")
    print(f"    python3 train_student_network.py {metadata_file}")


if __name__ == '__main__':
    main()
