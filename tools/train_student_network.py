#!/usr/bin/env python3
"""
Training pipeline for student pose estimation network

Trains the student network using teacher labels from offline VIO processing.
"""

import argparse
import json
import logging
from pathlib import Path
from typing import Dict, Optional

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from torch.utils.data import DataLoader, Dataset

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


def validate_metadata(metadata_file: Path, depth_dir: Path) -> None:
    """Fail fast if metadata is missing required keys or files"""
    with open(metadata_file, 'r') as f:
        frames = json.load(f)
    required_keys = [
        'imu_preintegration', 'imu_covariance', 'flow', 'flow_quality',
        'depth_file', 'match_quality', 'previous_pose', 'previous_velocity',
        'time_since_keyframe', 'pose_tx', 'pose_ty', 'pose_tz',
        'pose_qx', 'pose_qy', 'pose_qz', 'pose_qw', 'mean_reprojection_error',
    ]
    for i, frame in enumerate(frames):
        missing = [k for k in required_keys if k not in frame]
        if missing:
            raise ValueError(f"Frame {i} missing keys: {missing}")
        if len(frame['imu_preintegration']) != 15:
            raise ValueError(f"Frame {i} imu_preintegration must have length 15")
        if len(frame['imu_covariance']) != 15:
            raise ValueError(f"Frame {i} imu_covariance must have length 15")
        if len(frame['flow']) != 96:
            raise ValueError(f"Frame {i} flow must have length 96 (8x6x2)")
        if len(frame['previous_pose']) != 7:
            raise ValueError(f"Frame {i} previous_pose must have length 7")
        if len(frame['previous_velocity']) != 3:
            raise ValueError(f"Frame {i} previous_velocity must have length 3")
        if len(frame['match_quality']) == 0:
            raise ValueError(f"Frame {i} match_quality must be non-empty")
        depth_path = depth_dir / frame['depth_file']
        if not depth_path.exists():
            raise FileNotFoundError(f"Frame {i} depth file not found: {depth_path}")
    logger.info(f"Metadata validation passed for {len(frames)} frames")


class TeacherDataset(Dataset):
    """
    Load teacher training data from JSON metadata files
    """

    def __init__(
        self,
        metadata_file: Path,
        image_dir: Path,
        depth_dir: Path,
        transform=None,
    ):
        self.metadata_file = Path(metadata_file)
        self.image_dir = Path(image_dir)
        self.depth_dir = Path(depth_dir)
        self.transform = transform

        # Load metadata
        with open(self.metadata_file, 'r') as f:
            self.frames = json.load(f)

        logger.info(f"Loaded {len(self.frames)} frames from {metadata_file}")

    def __len__(self) -> int:
        return len(self.frames)

    def __getitem__(self, idx: int) -> Dict[str, torch.Tensor]:
        """
        Returns:
            dict with keys:
                - 'images': [2, 256, 256] - stereo pair
                - 'imu': [15] - IMU preintegration (required)
                - 'imu_covariance': [15] - IMU covariance (required)
                - 'flow': [96] - optical flow grid (required)
                - 'flow_quality': [1] - flow reliability (required)
                - 'depth_map': [1, 256, 256] - depth (required)
                - 'match_quality': [M] - match scores (required)
                - 'previous_pose': [7] - previous pose (required)
                - 'previous_velocity': [3] - previous velocity (required)
                - 'time_since_keyframe': [1] - scalar interval (required)
                - 'pose': [7] - target pose
                - 'uncertainty': [6] - uncertainty estimates
        """
        frame = self.frames[idx]

        try:
            required_keys = [
                'imu_preintegration', 'imu_covariance', 'flow', 'flow_quality',
                'depth_file', 'match_quality', 'previous_pose', 'previous_velocity',
                'time_since_keyframe', 'pose_tx', 'pose_ty', 'pose_tz',
                'pose_qx', 'pose_qy', 'pose_qz', 'pose_qw', 'mean_reprojection_error',
            ]
            missing = [k for k in required_keys if k not in frame]
            if missing:
                raise KeyError(f"Missing required keys in metadata: {missing}")

            # Load images
            from PIL import Image

            left_img = Image.open(self.image_dir / frame['image_files']['left_downscaled'])
            right_img = Image.open(self.image_dir / frame['image_files']['right_downscaled'])

            left_arr = np.array(left_img, dtype=np.float32) / 255.0
            right_arr = np.array(right_img, dtype=np.float32) / 255.0

            # Stack stereo pair
            images = np.stack([left_arr, right_arr], axis=0)
            images = torch.from_numpy(images)

            # IMU preintegration
            imu = torch.tensor(frame['imu_preintegration'], dtype=torch.float32)
            imu_covariance = torch.tensor(frame['imu_covariance'], dtype=torch.float32)

            # Optical flow
            flow = torch.tensor(frame['flow'], dtype=torch.float32)
            if flow.numel() != 96:
                raise ValueError("flow must have 96 elements (8x6x2)")
            flow_quality = torch.tensor(frame['flow_quality'], dtype=torch.float32)

            # Depth map if present
            depth_file = frame['depth_file']
            depth_path = self.depth_dir / depth_file
            if not depth_path.exists():
                raise FileNotFoundError(f"Depth file not found: {depth_path}")
            if depth_path.suffix in {'.npy', '.npz'}:
                depth_arr = np.load(depth_path)
            else:
                depth_img = Image.open(depth_path)
                depth_arr = np.array(depth_img, dtype=np.float32)
            depth_arr = depth_arr / (np.max(depth_arr) + 1e-6)
            if depth_arr.ndim == 2:
                depth_arr = depth_arr[None, ...]
            depth_map = torch.from_numpy(depth_arr.astype(np.float32))

            # Match quality
            match_quality = torch.tensor(frame['match_quality'], dtype=torch.float32)
            if match_quality.numel() == 0:
                raise ValueError("match_quality must be non-empty")
            # Pad/truncate to fixed length for batching
            target_len = 32
            if match_quality.numel() < target_len:
                pad = torch.zeros(target_len - match_quality.numel())
                match_quality = torch.cat([match_quality, pad], dim=0)
            elif match_quality.numel() > target_len:
                match_quality = match_quality[:target_len]

            # Previous state context
            previous_pose = torch.tensor(frame['previous_pose'], dtype=torch.float32)
            previous_velocity = torch.tensor(frame['previous_velocity'], dtype=torch.float32)
            time_since_keyframe = torch.tensor(frame['time_since_keyframe'], dtype=torch.float32)

            # Target pose
            pose = torch.tensor([
                frame['pose_tx'], frame['pose_ty'], frame['pose_tz'],
                frame['pose_qx'], frame['pose_qy'], frame['pose_qz'], frame['pose_qw'],
            ], dtype=torch.float32)

            # Normalize quaternion
            quat = pose[3:]
            pose[3:] = F.normalize(quat, p=2, dim=0)

            # Uncertainty
            uncertainty = torch.tensor([
                frame['mean_reprojection_error'],
                frame['mean_reprojection_error'],
                frame['mean_reprojection_error'],
                frame['mean_reprojection_error'],
                frame['mean_reprojection_error'],
                frame['mean_reprojection_error'],
            ], dtype=torch.float32)

            return {
                'images': images,
                'imu': imu,
                'imu_covariance': imu_covariance,
                'flow': flow,
                'flow_quality': flow_quality,
                'depth_map': depth_map,
                'match_quality': match_quality,
                'previous_pose': previous_pose,
                'previous_velocity': previous_velocity,
                'time_since_keyframe': time_since_keyframe,
                'pose': pose,
                'uncertainty': uncertainty,
                'frame_id': frame['frame_id'],
            }

        except Exception as e:
            logger.error(f"Error loading frame {idx}: {e}")
            raise


class StudentNetworkTrainer:
    """Manages training of student pose estimation network"""

    def __init__(
        self,
        model: nn.Module,
        device: torch.device,
        learning_rate: float = 1e-4,
        weight_decay: float = 1e-5,
    ):
        self.model = model.to(device)
        self.device = device

        # Optimizer
        self.optimizer = optim.Adam(
            self.model.parameters(),
            lr=learning_rate,
            weight_decay=weight_decay,
        )

        # Learning rate scheduler
        self.scheduler = optim.lr_scheduler.CosineAnnealingLR(
            self.optimizer,
            T_max=100,
            eta_min=1e-6,
        )

        # Loss function
        self.loss_fn = PoseLoss(use_uncertainty_weighting=True)

        # Tracking
        self.train_losses = []
        self.val_losses = []
        self.best_val_loss = float('inf')

    def train_epoch(self, train_loader: DataLoader) -> float:
        """Train one epoch"""
        self.model.train()
        total_loss = 0.0
        num_batches = 0

        for batch in train_loader:
            # Move to device
            images = batch['images'].to(self.device)
            imu = batch['imu'].to(self.device)
            flow = batch['flow'].to(self.device)
            imu_cov = batch['imu_covariance'].to(self.device)
            flow_quality = batch['flow_quality'].to(self.device).view(images.size(0), -1).squeeze(-1)
            depth_map = batch['depth_map'].to(self.device)
            match_quality = batch['match_quality'].to(self.device)
            previous_pose = batch['previous_pose'].to(self.device)
            previous_velocity = batch['previous_velocity'].to(self.device)
            time_since_keyframe = batch['time_since_keyframe'].to(self.device).view(images.size(0))
            pose_target = batch['pose'].to(self.device)
            uncertainty = batch['uncertainty'].to(self.device)

            # Forward pass
            outputs = self.model(
                images,
                imu,
                flow,
                depth_map=depth_map,
                match_quality=match_quality,
                flow_quality=flow_quality,
                imu_covariance=imu_cov,
                previous_pose=previous_pose,
                previous_velocity=previous_velocity,
                time_since_keyframe=time_since_keyframe,
            )
            pose_pred = outputs['pose']

            # Compute loss
            loss = self.loss_fn(pose_pred, pose_target, uncertainty)

            # Backward pass
            self.optimizer.zero_grad()
            loss.backward()
            torch.nn.utils.clip_grad_norm_(self.model.parameters(), max_norm=1.0)
            self.optimizer.step()

            total_loss += loss.item()
            num_batches += 1

        avg_loss = total_loss / num_batches
        self.train_losses.append(avg_loss)
        return avg_loss

    @torch.no_grad()
    def validate(self, val_loader: DataLoader) -> float:
        """Validate on validation set"""
        self.model.eval()
        total_loss = 0.0
        num_batches = 0

        for batch in val_loader:
            # Move to device
            images = batch['images'].to(self.device)
            imu = batch['imu'].to(self.device)
            flow = batch['flow'].to(self.device)
            imu_cov = batch['imu_covariance'].to(self.device)
            flow_quality = batch['flow_quality'].to(self.device).view(images.size(0), -1).squeeze(-1)
            depth_map = batch['depth_map'].to(self.device)
            match_quality = batch['match_quality'].to(self.device)
            previous_pose = batch['previous_pose'].to(self.device)
            previous_velocity = batch['previous_velocity'].to(self.device)
            time_since_keyframe = batch['time_since_keyframe'].to(self.device).view(images.size(0))
            pose_target = batch['pose'].to(self.device)
            uncertainty = batch['uncertainty'].to(self.device)

            # Forward pass
            outputs = self.model(
                images,
                imu,
                flow,
                depth_map=depth_map,
                match_quality=match_quality,
                flow_quality=flow_quality,
                imu_covariance=imu_cov,
                previous_pose=previous_pose,
                previous_velocity=previous_velocity,
                time_since_keyframe=time_since_keyframe,
            )
            pose_pred = outputs['pose']

            # Compute loss
            loss = self.loss_fn(pose_pred, pose_target, uncertainty)

            total_loss += loss.item()
            num_batches += 1

        avg_loss = total_loss / num_batches
        self.val_losses.append(avg_loss)
        return avg_loss

    def save_checkpoint(self, path: Path, epoch: int, is_best: bool = False):
        """Save model checkpoint"""
        checkpoint = {
            'epoch': epoch,
            'model_state_dict': self.model.state_dict(),
            'optimizer_state_dict': self.optimizer.state_dict(),
            'scheduler_state_dict': self.scheduler.state_dict(),
            'train_losses': self.train_losses,
            'val_losses': self.val_losses,
        }

        path.parent.mkdir(parents=True, exist_ok=True)
        torch.save(checkpoint, path)

        if is_best:
            best_path = path.parent / 'model_best.pth'
            torch.save(checkpoint, best_path)
            logger.info(f"Saved best model checkpoint to {best_path}")


class PoseLoss(nn.Module):
    """Loss function for pose prediction"""

    def __init__(self, use_uncertainty_weighting: bool = True):
        super().__init__()
        self.use_uncertainty_weighting = use_uncertainty_weighting

    def forward(
        self,
        predicted_pose: torch.Tensor,
        target_pose: torch.Tensor,
        uncertainty: Optional[torch.Tensor] = None,
    ) -> torch.Tensor:
        """
        Args:
            predicted_pose: [B, 7] (tx, ty, tz, qx, qy, qz, qw)
            target_pose: [B, 7]
            uncertainty: [B, 6] optional

        Returns:
            loss: scalar
        """
        # Translation loss
        trans_pred = predicted_pose[:, :3]
        trans_target = target_pose[:, :3]
        trans_loss = F.mse_loss(trans_pred, trans_target)

        # Rotation loss (quaternion distance)
        quat_pred = predicted_pose[:, 3:]
        quat_target = target_pose[:, 3:]

        # Normalize quaternions
        quat_pred = F.normalize(quat_pred, p=2, dim=1)
        quat_target = F.normalize(quat_target, p=2, dim=1)

        # Quaternion distance
        dot_prod = (quat_pred * quat_target).sum(dim=1).abs()
        quat_loss = (1.0 - dot_prod).mean()

        # Combined
        loss = trans_loss + quat_loss

        return loss


def train_student_network(
    metadata_file: Path,
    image_dir: Path,
    depth_dir: Path,
    output_dir: Path,
    num_epochs: int = 50,
    batch_size: int = 32,
    learning_rate: float = 1e-4,
    val_split: float = 0.1,
):
    """Main training function"""

    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    logger.info(f"Using device: {device}")

    # Validate metadata before dataset construction
    logger.info(f"Validating metadata {metadata_file}")
    validate_metadata(metadata_file, depth_dir)

    # Load dataset
    logger.info(f"Loading dataset from {metadata_file}")
    dataset = TeacherDataset(
        metadata_file=metadata_file,
        image_dir=image_dir,
        depth_dir=depth_dir,
    )

    # Split into train/val
    val_size = int(len(dataset) * val_split)
    train_size = len(dataset) - val_size
    train_set, val_set = torch.utils.data.random_split(
        dataset,
        [train_size, val_size],
        generator=torch.Generator().manual_seed(42),
    )

    # Create dataloaders
    train_loader = DataLoader(
        train_set,
        batch_size=batch_size,
        shuffle=True,
        num_workers=4,
        pin_memory=True,
    )
    val_loader = DataLoader(
        val_set,
        batch_size=batch_size,
        shuffle=False,
        num_workers=4,
    )

    logger.info(f"Training set: {len(train_set)}, Validation set: {len(val_set)}")

    # Create model
    from student_network import create_student_network
    model = create_student_network().to(device)
    logger.info(f"Model parameters: {sum(p.numel() for p in model.parameters())}")

    # Create trainer
    trainer = StudentNetworkTrainer(
        model=model,
        device=device,
        learning_rate=learning_rate,
    )

    # Training loop
    for epoch in range(num_epochs):
        train_loss = trainer.train_epoch(train_loader)
        val_loss = trainer.validate(val_loader)

        trainer.scheduler.step()

        logger.info(
            f"Epoch {epoch+1}/{num_epochs} - "
            f"Train Loss: {train_loss:.4f}, Val Loss: {val_loss:.4f}"
        )

        # Save checkpoint
        is_best = val_loss < trainer.best_val_loss
        if is_best:
            trainer.best_val_loss = val_loss

        if (epoch + 1) % 5 == 0 or is_best:
            checkpoint_path = output_dir / f"checkpoint_epoch_{epoch+1:03d}.pth"
            trainer.save_checkpoint(checkpoint_path, epoch + 1, is_best)

    # Save training history
    history = {
        'train_losses': trainer.train_losses,
        'val_losses': trainer.val_losses,
    }
    with open(output_dir / 'training_history.json', 'w') as f:
        json.dump(history, f, indent=2)

    logger.info(f"Training complete! Results saved to {output_dir}")

    return model, trainer


def main():
    parser = argparse.ArgumentParser(
        description="Train student pose estimation network"
    )
    parser.add_argument(
        'metadata_file',
        type=Path,
        help="Path to teacher metadata JSON file"
    )
    parser.add_argument(
        '--image-dir',
        type=Path,
        help="Directory containing images (default: parent of metadata file)"
    )
    parser.add_argument(
        '--depth-dir',
        type=Path,
        help="Directory containing depth maps (default: parent of metadata file)"
    )
    parser.add_argument(
        '--output-dir',
        type=Path,
        default=Path('./model_outputs'),
        help="Output directory for model checkpoints"
    )
    parser.add_argument(
        '--num-epochs',
        type=int,
        default=50,
        help="Number of training epochs"
    )
    parser.add_argument(
        '--batch-size',
        type=int,
        default=32,
        help="Training batch size"
    )
    parser.add_argument(
        '--learning-rate',
        type=float,
        default=1e-4,
        help="Initial learning rate"
    )
    parser.add_argument(
        '--val-split',
        type=float,
        default=0.1,
        help="Validation split ratio"
    )

    args = parser.parse_args()

    # Set default image/depth dirs
    image_dir = args.image_dir or args.metadata_file.parent / 'images'
    depth_dir = args.depth_dir or args.metadata_file.parent / 'depth'

    train_student_network(
        metadata_file=args.metadata_file,
        image_dir=image_dir,
        depth_dir=depth_dir,
        output_dir=args.output_dir,
        num_epochs=args.num_epochs,
        batch_size=args.batch_size,
        learning_rate=args.learning_rate,
        val_split=args.val_split,
    )


if __name__ == '__main__':
    main()
