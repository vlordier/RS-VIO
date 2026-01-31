#!/usr/bin/env python3
"""
Student Network for Pose Initialization in VIO

This neural network learns to predict 6-DOF camera poses from:
- Stereo images (256x256)
- IMU preintegration (15D vector)
- Optical flow features (sparse grid)

The network is trained as a "student" network using teacher labels
from offline VIO processing with bundle adjustment.
"""

from typing import Dict, Optional, Tuple

import torch
import torch.nn as nn
import torch.nn.functional as F


class StereoImageEncoder(nn.Module):
    """Encodes stereo image pair into feature vectors"""

    def __init__(self, in_channels: int = 1, feat_dim: int = 128):
        super().__init__()
        self.feat_dim = feat_dim

        # Shared CNN backbone for both images
        self.backbone = nn.Sequential(
            nn.Conv2d(in_channels, 32, 7, stride=2, padding=3),  # 256 -> 128
            nn.BatchNorm2d(32),
            nn.ReLU(inplace=True),

            nn.Conv2d(32, 64, 5, stride=2, padding=2),  # 128 -> 64
            nn.BatchNorm2d(64),
            nn.ReLU(inplace=True),

            nn.Conv2d(64, 128, 3, stride=2, padding=1),  # 64 -> 32
            nn.BatchNorm2d(128),
            nn.ReLU(inplace=True),

            nn.Conv2d(128, 256, 3, stride=2, padding=1),  # 32 -> 16
            nn.BatchNorm2d(256),
            nn.ReLU(inplace=True),
        )

        # Spatial attention to weight different regions
        self.attention = nn.Sequential(
            nn.Conv2d(256, 64, 1),
            nn.BatchNorm2d(64),
            nn.ReLU(inplace=True),
            nn.Conv2d(64, 1, 1),
            nn.Sigmoid(),
        )

        # Global average pooling + FC
        self.fc = nn.Sequential(
            nn.Linear(512, feat_dim * 2),
            nn.ReLU(inplace=True),
            nn.Dropout(0.2),
            nn.Linear(feat_dim * 2, feat_dim),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """
        Args:
            x: Stereo images [B, 2, 256, 256]

        Returns:
            features: [B, feat_dim]
        """
        batch_size = x.size(0)

        # Process left and right separately then concat
        left = x[:, 0:1, :, :]  # [B, 1, 256, 256]
        right = x[:, 1:2, :, :]

        left_feat = self.backbone(left)  # [B, 256, H, W]
        right_feat = self.backbone(right)

        # Apply attention and concatenate
        left_attn = self.attention(left_feat)
        right_attn = self.attention(right_feat)

        left_feat = left_feat * left_attn
        right_feat = right_feat * right_attn

        feat = torch.cat([left_feat, right_feat], dim=1)  # [B, 512, H, W]

        # Adaptive global average pooling
        feat = torch.nn.functional.adaptive_avg_pool2d(feat, 1)  # [B, 512, 1, 1]
        feat = feat.view(batch_size, -1)  # [B, 512]

        feat = self.fc(feat)
        return feat


class IMUPreintegrationEncoder(nn.Module):
    """Encodes IMU preintegration (15D) into feature vector"""

    def __init__(self, input_dim: int = 15, feat_dim: int = 64):
        super().__init__()
        self.input_dim = input_dim
        self.encoder = nn.Sequential(
            nn.Linear(input_dim, feat_dim),
            nn.BatchNorm1d(feat_dim),
            nn.ReLU(inplace=True),
            nn.Dropout(0.1),

            nn.Linear(feat_dim, feat_dim),
            nn.BatchNorm1d(feat_dim),
            nn.ReLU(inplace=True),

            nn.Linear(feat_dim, feat_dim),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """
        Args:
            x: IMU preintegration [B, 15]

        Returns:
            features: [B, feat_dim]
        """
        # Pad with zeros if covariance is missing so input dimension stays consistent
        if x.size(1) < self.input_dim:
            pad = torch.zeros(x.size(0), self.input_dim - x.size(1), device=x.device, dtype=x.dtype)
            x = torch.cat([x, pad], dim=1)
        return self.encoder(x)


class OpticalFlowEncoder(nn.Module):
    """Encodes sparse optical flow grid into feature vector with quality weighting"""

    def __init__(self, grid_size: Tuple[int, int] = (8, 6), feat_dim: int = 64):
        super().__init__()
        self.grid_size = grid_size
        grid_dim = grid_size[0] * grid_size[1] * 2  # Each cell has (dx, dy)

        self.encoder = nn.Sequential(
            nn.Linear(grid_dim, feat_dim),
            nn.ReLU(inplace=True),
            nn.Dropout(0.1),

            nn.Linear(feat_dim, feat_dim),
        )

    def forward(
        self,
        x: torch.Tensor,
        flow_quality: torch.Tensor,
    ) -> torch.Tensor:
        """
        Args:
            x: Flow grid [B, 8, 6, 2] or flattened [B, 96]
            flow_quality: Scalar quality per frame [B]
        """
        if x.dim() > 2:
            x = x.view(x.size(0), -1)
        # Down-weight unreliable flow
        x = x * flow_quality.unsqueeze(-1)
        return self.encoder(x)


class DepthEncoder(nn.Module):
    """Lightweight encoder for depth maps"""

    def __init__(self, feat_dim: int = 64):
        super().__init__()
        self.feat_dim = feat_dim
        self.cnn = nn.Sequential(
            nn.Conv2d(1, 16, 5, stride=2, padding=2),  # 256 -> 128
            nn.BatchNorm2d(16),
            nn.ReLU(inplace=True),
            nn.Conv2d(16, 32, 5, stride=2, padding=2),  # 128 -> 64
            nn.BatchNorm2d(32),
            nn.ReLU(inplace=True),
        )
        self.fc = nn.Sequential(
            nn.Linear(32 * 64 * 64, feat_dim * 2),
            nn.ReLU(inplace=True),
            nn.Dropout(0.1),
            nn.Linear(feat_dim * 2, feat_dim),
        )

    def forward(self, depth_map: torch.Tensor, batch_size: int, device: torch.device) -> torch.Tensor:
        """
        Args:
            depth_map: [B, 1, 256, 256]
        """
        if depth_map.dim() == 3:
            depth_map = depth_map.unsqueeze(1)
        feat = self.cnn(depth_map)
        feat = feat.view(feat.size(0), -1)
        return self.fc(feat)


class ContextEncoder(nn.Module):
    """Encodes quality/context signals into a compact feature"""

    def __init__(self, feat_dim: int = 32):
        super().__init__()
        self.encoder = nn.Sequential(
            nn.Linear(16, feat_dim),
            nn.ReLU(inplace=True),
            nn.Linear(feat_dim, feat_dim),
        )

    def forward(
        self,
        batch_size: int,
        device: torch.device,
        match_quality: torch.Tensor,  # [B, M]
        flow_quality: torch.Tensor,   # [B]
        previous_pose: torch.Tensor,  # [B, 7]
        previous_velocity: torch.Tensor,  # [B, 3]
        time_since_keyframe: torch.Tensor,  # [B]
    ) -> torch.Tensor:
        device = match_quality.device
        # Match quality stats
        mean_q = match_quality.mean(dim=1, keepdim=True)
        std_q = match_quality.std(dim=1, keepdim=True)
        max_q = match_quality.max(dim=1, keepdim=True)[0]
        count_q = torch.ones_like(mean_q) * match_quality.size(1)

        # Flow quality scalar
        flow_quality = flow_quality.view(batch_size, 1)
        time_since_keyframe = time_since_keyframe.view(batch_size, 1)

        parts = [
            mean_q, std_q, max_q, count_q,
            flow_quality,
            previous_pose,
            previous_velocity,
            time_since_keyframe,
        ]
        fused = torch.cat(parts, dim=1)  # [B, 16]
        return self.encoder(fused)


class PoseHead(nn.Module):
    """Outputs 6-DOF pose (translation + rotation) with uncertainty"""

    def __init__(self, input_dim: int = 256, output_uncertainty: bool = True):
        super().__init__()
        self.output_uncertainty = output_uncertainty

        # Pose estimation head
        self.pose_layer = nn.Sequential(
            nn.Linear(input_dim, 128),
            nn.ReLU(inplace=True),
            nn.Dropout(0.2),
            nn.Linear(128, 7),  # tx, ty, tz, qx, qy, qz, qw
        )

        # Uncertainty estimation head
        if output_uncertainty:
            self.uncertainty_layer = nn.Sequential(
                nn.Linear(input_dim, 64),
                nn.ReLU(inplace=True),
                nn.Dropout(0.1),
                nn.Linear(64, 6),
                nn.Softplus(),  # Ensure positive uncertainty
            )

    def forward(self, x: torch.Tensor) -> Dict[str, torch.Tensor]:
        """
        Args:
            x: Features [B, input_dim]

        Returns:
            outputs: {
                'pose': [B, 7],  # 3D position + quaternion
                'uncertainty': [B, 6],  # 6 uncertainty values if enabled
            }
        """
        pose = self.pose_layer(x)

        outputs = {'pose': pose}
        if self.output_uncertainty:
            uncertainty = self.uncertainty_layer(x)
            outputs['uncertainty'] = uncertainty

        return outputs


class StudentPoseNetwork(nn.Module):
    """
    Complete student network for pose initialization

    Input:
        - Stereo images: [B, 2, 256, 256]
        - IMU preintegration: [B, 15]
        - IMU covariance: [B, 15]
        - Optical flow: [B, 8, 6, 2] or [B, 96]
        - Flow quality: [B]
        - Depth map: [B, 1, 256, 256]
        - Match quality: [B, M]
        - Previous pose / velocity: [B, 7], [B, 3]
        - Time since keyframe: [B]

    Output:
        - Pose: [B, 7] (translation + quaternion)
        - Uncertainty (optional): [B, 6]
    """

    def __init__(
        self,
        image_feat_dim: int = 128,
        imu_feat_dim: int = 64,
        flow_feat_dim: int = 64,
        output_uncertainty: bool = True,
    ):
        super().__init__()

        # Encoders for different modalities
        self.image_encoder = StereoImageEncoder(feat_dim=image_feat_dim)
        self.imu_encoder = IMUPreintegrationEncoder(input_dim=30, feat_dim=imu_feat_dim)
        self.flow_encoder = OpticalFlowEncoder(feat_dim=flow_feat_dim)
        self.depth_encoder = DepthEncoder(feat_dim=64)
        self.context_encoder = ContextEncoder(feat_dim=32)

        # Fusion layers
        fusion_dim = image_feat_dim + imu_feat_dim + flow_feat_dim + 64 + 32
        self.fusion = nn.Sequential(
            nn.Linear(fusion_dim, 256),
            nn.BatchNorm1d(256),
            nn.ReLU(inplace=True),
            nn.Dropout(0.2),

            nn.Linear(256, 256),
            nn.BatchNorm1d(256),
            nn.ReLU(inplace=True),
            nn.Dropout(0.2),
        )

        # Pose output head
        self.pose_head = PoseHead(input_dim=256, output_uncertainty=output_uncertainty)

    def forward(
        self,
        images: torch.Tensor,
        imu: torch.Tensor,
        imu_covariance: torch.Tensor,
        flow: torch.Tensor,
        flow_quality: torch.Tensor,
        depth_map: torch.Tensor,
        match_quality: torch.Tensor,
        previous_pose: torch.Tensor,
        previous_velocity: torch.Tensor,
        time_since_keyframe: torch.Tensor,
    ) -> Dict[str, torch.Tensor]:
        """
        Args:
            images: Stereo images [B, 2, 256, 256]
            imu: IMU preintegration [B, 15]
            flow: Optical flow [B, 8, 6, 2] or [B, 96]
            depth_map: Depth map [B, 1, 256, 256] or None
            match_quality: Match quality scores [B, M] or None
            flow_quality: Flow inlier ratio [B] or None
            imu_covariance: IMU covariance diag [B, 15] or None
            previous_pose: Previous pose [B, 7] or None
            previous_velocity: Previous velocity [B, 3] or None
            time_since_keyframe: Scalar time [B] or None

        Returns:
            outputs: {
                'pose': [B, 7],
                'uncertainty': [B, 6] if enabled
            }
        """
        batch_size = images.size(0)
        device = images.device
        # Encode each modality
        image_feat = self.image_encoder(images)
        imu_input = torch.cat([imu, imu_covariance], dim=1)
        imu_feat = self.imu_encoder(imu_input)
        flow_feat = self.flow_encoder(flow, flow_quality=flow_quality)
        depth_feat = self.depth_encoder(depth_map, batch_size=batch_size, device=device)
        context_feat = self.context_encoder(
            batch_size=batch_size,
            device=device,
            match_quality=match_quality,
            flow_quality=flow_quality,
            previous_pose=previous_pose,
            previous_velocity=previous_velocity,
            time_since_keyframe=time_since_keyframe,
        )

        # Fuse features
        fused = torch.cat([image_feat, imu_feat, flow_feat, depth_feat, context_feat], dim=1)
        fused = self.fusion(fused)

        # Output pose and uncertainty
        outputs = self.pose_head(fused)

        return outputs

    def get_pose(self, outputs: Dict[str, torch.Tensor]) -> torch.Tensor:
        """Extract just the pose tensor"""
        return outputs['pose']

    def get_uncertainty(self, outputs: Dict[str, torch.Tensor]) -> Optional[torch.Tensor]:
        """Extract uncertainty if available"""
        return outputs.get('uncertainty', None)


class PoseLoss(nn.Module):
    """
    Loss function for pose prediction

    Combines:
    - Translation loss (L2)
    - Rotation loss (geodesic distance on SO(3))
    - Uncertainty weighting
    """

    def __init__(self, use_uncertainty_weighting: bool = True):
        super().__init__()
        self.use_uncertainty_weighting = use_uncertainty_weighting

    def forward(
        self,
        predicted_pose: torch.Tensor,  # [B, 7]
        target_pose: torch.Tensor,      # [B, 7]
        uncertainty: Optional[torch.Tensor] = None,  # [B, 6] optional
    ) -> torch.Tensor:
        """
        Args:
            predicted_pose: [B, 7] (tx, ty, tz, qx, qy, qz, qw)
            target_pose: [B, 7]
            uncertainty: [B, 6] optional

        Returns:
            loss: scalar
        """
        # Translation loss (L2)
        trans_pred = predicted_pose[:, :3]
        trans_target = target_pose[:, :3]
        trans_loss = F.mse_loss(trans_pred, trans_target)

        # Rotation loss (quaternion, with proper normalization)
        quat_pred = predicted_pose[:, 3:]
        quat_target = target_pose[:, 3:]

        # Normalize quaternions
        quat_pred = F.normalize(quat_pred, p=2, dim=1)
        quat_target = F.normalize(quat_target, p=2, dim=1)

        # Quaternion distance: 1 - |<q1, q2>|
        dot_prod = (quat_pred * quat_target).sum(dim=1).abs()
        quat_loss = (1.0 - dot_prod).mean()

        # Combined loss
        loss = trans_loss + quat_loss

        # Weighted by uncertainty if available
        if self.use_uncertainty_weighting and uncertainty is not None:
            # Uncertainty on [3 trans + 3 rot]
            weights = 1.0 / (uncertainty + 1e-6)
            trans_weights = weights[:, :3].mean(dim=1)
            rot_weights = weights[:, 3:].mean(dim=1)

            loss = (trans_loss * trans_weights.mean() +
                   quat_loss * rot_weights.mean())

        return loss


def create_student_network(pretrained: bool = False) -> StudentPoseNetwork:
    """Factory function to create student network"""
    network = StudentPoseNetwork(
        image_feat_dim=128,
        imu_feat_dim=64,
        flow_feat_dim=64,
        output_uncertainty=True,
    )

    if pretrained:
        # Load pretrained weights if available
        pass

    return network


if __name__ == "__main__":
    # Test network
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    model = create_student_network().to(device)

    # Create dummy inputs
    images = torch.randn(4, 2, 256, 256).to(device)
    imu = torch.randn(4, 15).to(device)
    flow = torch.randn(4, 96).to(device)

    # Forward pass
    outputs = model(images, imu, flow)

    print(f"Pose output shape: {outputs['pose'].shape}")
    print(f"Uncertainty shape: {outputs.get('uncertainty', 'N/A')}")
    print(f"Total parameters: {sum(p.numel() for p in model.parameters())}")

    # Test loss
    loss_fn = PoseLoss()
    target_pose = torch.randn(4, 7).to(device)
    loss = loss_fn(outputs['pose'], target_pose)
    print(f"Loss: {loss.item():.4f}")
