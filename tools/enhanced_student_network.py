#!/usr/bin/env python3
"""
Enhanced Student Network for Pose Initialization

Uses all available VIO information:
- Stereo images + feature matches
- IMU preintegration with covariance
- Optical flow with quality metrics
- Semi-dense depth with confidence
- Prior state information

Multi-stream architecture with attention-based fusion.
"""

from typing import Dict, Optional

import torch
import torch.nn as nn
import torch.nn.functional as F


class VisualFeatureEncoder(nn.Module):
    """
    Encodes stereo images and feature matches into visual features

    Inputs:
    - Stereo pair: [B, 2, 512, 512]
    - Feature matches: [B, M, 4] (left_x, left_y, right_x, right_y)
    - Match quality: [B, M] (scores 0-1)
    """

    def __init__(self, output_dim: int = 256):
        super().__init__()
        self.output_dim = output_dim

        # Stereo image backbone
        self.stereo_backbone = nn.Sequential(
            nn.Conv2d(2, 64, 7, stride=2, padding=3),  # 512 -> 256
            nn.BatchNorm2d(64),
            nn.ReLU(inplace=True),

            nn.Conv2d(64, 128, 5, stride=2, padding=2),  # 256 -> 128
            nn.BatchNorm2d(128),
            nn.ReLU(inplace=True),

            nn.Conv2d(128, 256, 3, stride=2, padding=1),  # 128 -> 64
            nn.BatchNorm2d(256),
            nn.ReLU(inplace=True),
        )

        # Feature match encoder
        self.match_encoder = nn.Sequential(
            nn.Linear(4, 32),
            nn.ReLU(inplace=True),
            nn.Linear(32, 32),
        )

        # Attention for matches
        self.match_attention = nn.Sequential(
            nn.Linear(32, 16),
            nn.ReLU(inplace=True),
            nn.Linear(16, 1),
            nn.Sigmoid(),
        )

        # Feature fusion
        self.fusion = nn.Sequential(
            nn.Linear(256 * 64 * 64 // (64*64) + 32, output_dim),  # Image feat + match feat
            nn.ReLU(inplace=True),
            nn.Dropout(0.2),
            nn.Linear(output_dim, output_dim),
        )

    def forward(
        self,
        stereo_images: torch.Tensor,  # [B, 2, 512, 512]
        feature_matches: Optional[torch.Tensor] = None,  # [B, M, 4]
        match_quality: Optional[torch.Tensor] = None,  # [B, M]
    ) -> torch.Tensor:
        batch_size = stereo_images.size(0)

        # Process stereo images
        image_feat = self.stereo_backbone(stereo_images)  # [B, 256, 64, 64]
        image_feat = F.adaptive_avg_pool2d(image_feat, 1).view(batch_size, -1)  # [B, 256]

        # Process feature matches if available
        if feature_matches is not None:
            match_feat = self.match_encoder(feature_matches)  # [B, M, 32]

            if match_quality is not None:
                # Weight matches by quality
                weights = self.match_attention(match_feat)  # [B, M, 1]
                weights = weights * match_quality.unsqueeze(-1)  # Apply quality
                match_feat = (match_feat * weights).sum(dim=1) / (weights.sum(dim=1) + 1e-6)
            else:
                match_feat = match_feat.mean(dim=1)  # [B, 32]
        else:
            match_feat = torch.zeros(batch_size, 32, device=stereo_images.device)

        # Fuse image and match features
        combined = torch.cat([image_feat, match_feat], dim=1)
        feat = self.fusion(combined)

        return feat


class MotionEncoder(nn.Module):
    """
    Encodes IMU preintegration, optical flow, and motion quality

    Inputs:
    - IMU preintegration: [B, 15]
    - IMU covariance diagonal: [B, 15]
    - Optical flow grid: [B, 8, 6, 2] or [B, 96]
    - Flow inlier ratio: [B] scalar
    """

    def __init__(self, output_dim: int = 128):
        super().__init__()
        self.output_dim = output_dim

        # IMU encoder with uncertainty
        self.imu_encoder = nn.Sequential(
            nn.Linear(15 + 15, 64),  # preint + covariance
            nn.BatchNorm1d(64),
            nn.ReLU(inplace=True),
            nn.Dropout(0.1),
            nn.Linear(64, 64),
        )

        # Optical flow encoder
        self.flow_encoder = nn.Sequential(
            nn.Linear(96, 48),
            nn.ReLU(inplace=True),
            nn.Dropout(0.1),
            nn.Linear(48, 48),
        )

        # Fusion
        self.fusion = nn.Sequential(
            nn.Linear(64 + 48, output_dim),
            nn.ReLU(inplace=True),
            nn.Linear(output_dim, output_dim),
        )

    def forward(
        self,
        imu_preint: torch.Tensor,  # [B, 15]
        imu_covariance: Optional[torch.Tensor] = None,  # [B, 15]
        optical_flow: torch.Tensor = None,  # [B, 96] or [B, 8, 6, 2]
        flow_quality: Optional[torch.Tensor] = None,  # [B] scalar
    ) -> torch.Tensor:
        batch_size = imu_preint.size(0)
        device = imu_preint.device

        # IMU: combine preintegration with covariance
        if imu_covariance is not None:
            imu_input = torch.cat([imu_preint, imu_covariance], dim=1)
        else:
            imu_input = torch.cat([imu_preint, torch.ones_like(imu_preint) * 0.1], dim=1)

        imu_feat = self.imu_encoder(imu_input)  # [B, 64]

        # Optical flow
        if optical_flow is not None:
            if optical_flow.dim() > 2:
                optical_flow = optical_flow.view(batch_size, -1)  # Flatten to [B, 96]

            if flow_quality is not None:
                # Weight by quality
                optical_flow = optical_flow * flow_quality.unsqueeze(-1)

            flow_feat = self.flow_encoder(optical_flow)  # [B, 48]
        else:
            flow_feat = torch.zeros(batch_size, 48, device=device)

        # Fuse motion features
        combined = torch.cat([imu_feat, flow_feat], dim=1)
        feat = self.fusion(combined)

        return feat


class DepthEncoder(nn.Module):
    """
    Encodes semi-dense depth map and confidence

    Inputs:
    - Depth map: [B, 512, 512] or [B, 1, 512, 512]
    - Depth confidence: [B, 512, 512] or [B, 1, 512, 512]
    - Depth coverage %: [B] scalar
    """

    def __init__(self, output_dim: int = 96):
        super().__init__()
        self.output_dim = output_dim

        # Depth map processing (lightweight CNN)
        self.depth_cnn = nn.Sequential(
            nn.Conv2d(2, 32, 7, stride=4, padding=3),  # 512 -> 128
            nn.BatchNorm2d(32),
            nn.ReLU(inplace=True),

            nn.Conv2d(32, 64, 5, stride=2, padding=2),  # 128 -> 64
            nn.BatchNorm2d(64),
            nn.ReLU(inplace=True),
        )

        # Global pooling + FC
        self.fc = nn.Sequential(
            nn.Linear(64 * 64, output_dim * 2),
            nn.ReLU(inplace=True),
            nn.Dropout(0.15),
            nn.Linear(output_dim * 2, output_dim),
        )

    def forward(
        self,
        depth_map: Optional[torch.Tensor] = None,  # [B, 512, 512] or [B, 1, 512, 512]
        depth_confidence: Optional[torch.Tensor] = None,  # [B, 512, 512]
        depth_coverage: Optional[torch.Tensor] = None,  # [B] scalar
    ) -> torch.Tensor:
        batch_size = depth_map.size(0) if depth_map is not None else 1
        device = depth_map.device if depth_map is not None else torch.device('cpu')

        if depth_map is None:
            return torch.zeros(batch_size, self.output_dim, device=device)

        # Ensure proper dimensions
        if depth_map.dim() == 3:
            depth_map = depth_map.unsqueeze(1)  # [B, 1, 512, 512]
        if depth_confidence is not None and depth_confidence.dim() == 3:
            depth_confidence = depth_confidence.unsqueeze(1)

        # Combine depth and confidence
        if depth_confidence is not None:
            combined = torch.cat([depth_map, depth_confidence], dim=1)  # [B, 2, 512, 512]
        else:
            combined = torch.cat([depth_map, torch.ones_like(depth_map) * 0.5], dim=1)

        # Process through CNN
        feat = self.depth_cnn(combined)  # [B, 64, 64, 64]
        feat = F.adaptive_avg_pool2d(feat, 1).view(batch_size, -1)  # [B, 64]

        # FC layers
        feat = self.fc(feat)  # [B, output_dim]

        # Apply coverage weighting if available
        if depth_coverage is not None:
            feat = feat * (depth_coverage.unsqueeze(-1) + 0.5)  # Weight by coverage

        return feat


class ContextEncoder(nn.Module):
    """
    Encodes prior state context

    Inputs:
    - Previous pose: [B, 7] (if available)
    - Previous velocity: [B, 3]
    - Time since last keyframe: [B] scalar
    - Feature track histogram: [B, 10] (track length distribution)
    """

    def __init__(self, output_dim: int = 64):
        super().__init__()
        self.output_dim = output_dim

        self.encoder = nn.Sequential(
            nn.Linear(7 + 3 + 1 + 10, output_dim),
            nn.ReLU(inplace=True),
            nn.Dropout(0.1),
            nn.Linear(output_dim, output_dim),
        )

    def forward(
        self,
        previous_pose: Optional[torch.Tensor] = None,  # [B, 7]
        previous_velocity: Optional[torch.Tensor] = None,  # [B, 3]
        time_since_keyframe: Optional[torch.Tensor] = None,  # [B]
        track_lengths: Optional[torch.Tensor] = None,  # [B, 10]
    ) -> torch.Tensor:
        batch_size = 1
        device = torch.device('cpu')

        # Determine batch size and device
        if previous_pose is not None:
            batch_size = previous_pose.size(0)
            device = previous_pose.device
        elif previous_velocity is not None:
            batch_size = previous_velocity.size(0)
            device = previous_velocity.device

        # Build input vector
        inputs = []

        if previous_pose is not None:
            inputs.append(previous_pose)
        else:
            inputs.append(torch.zeros(batch_size, 7, device=device))

        if previous_velocity is not None:
            inputs.append(previous_velocity)
        else:
            inputs.append(torch.zeros(batch_size, 3, device=device))

        if time_since_keyframe is not None:
            inputs.append(time_since_keyframe.unsqueeze(-1))
        else:
            inputs.append(torch.zeros(batch_size, 1, device=device))

        if track_lengths is not None:
            inputs.append(track_lengths)
        else:
            inputs.append(torch.zeros(batch_size, 10, device=device))

        combined = torch.cat(inputs, dim=1)
        feat = self.encoder(combined)

        return feat


class EnhancedStudentPoseNetwork(nn.Module):
    """
    Enhanced student network using all available VIO information

    Inputs:
        Visual:
        - Stereo images: [B, 2, 512, 512]
        - Feature matches: [B, M, 4] optional
        - Match quality: [B, M] optional

        Motion:
        - IMU preintegration: [B, 15]
        - IMU covariance: [B, 15] optional
        - Optical flow: [B, 96] or [B, 8, 6, 2]
        - Flow quality: [B] optional

        Depth:
        - Depth map: [B, 512, 512]
        - Depth confidence: [B, 512, 512]
        - Depth coverage %: [B] optional

        Context:
        - Previous pose: [B, 7] optional
        - Previous velocity: [B, 3] optional
        - Time since keyframe: [B] optional
        - Track lengths: [B, 10] optional

    Output:
        - Pose: [B, 7] (tx, ty, tz, qx, qy, qz, qw)
        - Uncertainty: [B, 6] optional
    """

    def __init__(
        self,
        visual_dim: int = 256,
        motion_dim: int = 128,
        depth_dim: int = 96,
        context_dim: int = 64,
        fusion_dim: int = 256,
        output_uncertainty: bool = True,
    ):
        super().__init__()

        # Multi-stream encoders
        self.visual_encoder = VisualFeatureEncoder(visual_dim)
        self.motion_encoder = MotionEncoder(motion_dim)
        self.depth_encoder = DepthEncoder(depth_dim)
        self.context_encoder = ContextEncoder(context_dim)

        # Fusion with attention
        total_dim = visual_dim + motion_dim + depth_dim + context_dim

        self.attention = nn.Sequential(
            nn.Linear(total_dim, total_dim // 2),
            nn.ReLU(inplace=True),
            nn.Linear(total_dim // 2, 4),  # Attention weights for 4 streams
            nn.Softmax(dim=1),
        )

        self.fusion = nn.Sequential(
            nn.Linear(total_dim, fusion_dim),
            nn.BatchNorm1d(fusion_dim),
            nn.ReLU(inplace=True),
            nn.Dropout(0.2),

            nn.Linear(fusion_dim, fusion_dim),
            nn.BatchNorm1d(fusion_dim),
            nn.ReLU(inplace=True),
            nn.Dropout(0.2),
        )

        # Pose output
        self.pose_layer = nn.Sequential(
            nn.Linear(fusion_dim, 128),
            nn.ReLU(inplace=True),
            nn.Linear(128, 7),
        )

        # Uncertainty (optional)
        self.output_uncertainty = output_uncertainty
        if output_uncertainty:
            self.uncertainty_layer = nn.Sequential(
                nn.Linear(fusion_dim, 64),
                nn.ReLU(inplace=True),
                nn.Linear(64, 6),
                nn.Softplus(),
            )

    def forward(
        self,
        stereo_images: torch.Tensor,
        imu_preint: torch.Tensor,
        optical_flow: torch.Tensor,
        # Optional visual inputs
        feature_matches: Optional[torch.Tensor] = None,
        match_quality: Optional[torch.Tensor] = None,
        # Optional motion inputs
        imu_covariance: Optional[torch.Tensor] = None,
        flow_quality: Optional[torch.Tensor] = None,
        # Optional depth inputs
        depth_map: Optional[torch.Tensor] = None,
        depth_confidence: Optional[torch.Tensor] = None,
        depth_coverage: Optional[torch.Tensor] = None,
        # Optional context inputs
        previous_pose: Optional[torch.Tensor] = None,
        previous_velocity: Optional[torch.Tensor] = None,
        time_since_keyframe: Optional[torch.Tensor] = None,
        track_lengths: Optional[torch.Tensor] = None,
    ) -> Dict[str, torch.Tensor]:
        # Encode each stream
        visual_feat = self.visual_encoder(
            stereo_images, feature_matches, match_quality
        )

        motion_feat = self.motion_encoder(
            imu_preint, imu_covariance, optical_flow, flow_quality
        )

        depth_feat = self.depth_encoder(
            depth_map, depth_confidence, depth_coverage
        )

        context_feat = self.context_encoder(
            previous_pose, previous_velocity, time_since_keyframe, track_lengths
        )

        # Concatenate all features
        all_feat = torch.cat(
            [visual_feat, motion_feat, depth_feat, context_feat],
            dim=1
        )

        # Attention-weighted combination
        attention_weights = self.attention(all_feat)  # [B, 4]
        batch_size = all_feat.size(0)

        # Apply attention weights
        visual_feat_weighted = visual_feat * attention_weights[:, 0:1]
        motion_feat_weighted = motion_feat * attention_weights[:, 1:2]
        depth_feat_weighted = depth_feat * attention_weights[:, 2:3]
        context_feat_weighted = context_feat * attention_weights[:, 3:4]

        weighted_feat = torch.cat(
            [visual_feat_weighted, motion_feat_weighted,
             depth_feat_weighted, context_feat_weighted],
            dim=1
        )

        # Fusion
        fused = self.fusion(weighted_feat)

        # Pose output
        pose = self.pose_layer(fused)

        # Normalize quaternion
        pose_norm = pose.clone()
        quat = F.normalize(pose[:, 3:], p=2, dim=1)
        pose_norm[:, 3:] = quat

        outputs = {'pose': pose_norm}

        # Uncertainty
        if self.output_uncertainty:
            uncertainty = self.uncertainty_layer(fused)
            outputs['uncertainty'] = uncertainty

        # Return attention weights for interpretability
        outputs['attention_weights'] = attention_weights

        return outputs


def create_enhanced_student_network(
    pretrained: bool = False
) -> EnhancedStudentPoseNetwork:
    """Factory function"""
    network = EnhancedStudentPoseNetwork(
        visual_dim=256,
        motion_dim=128,
        depth_dim=96,
        context_dim=64,
        fusion_dim=256,
        output_uncertainty=True,
    )

    return network


if __name__ == "__main__":
    # Test enhanced network
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    model = create_enhanced_student_network().to(device)

    # Create dummy inputs
    stereo = torch.randn(2, 2, 512, 512).to(device)
    imu = torch.randn(2, 15).to(device)
    flow = torch.randn(2, 96).to(device)

    # Optional inputs
    matches = torch.randn(2, 50, 4).to(device)  # 50 matches
    match_qual = torch.rand(2, 50).to(device)

    imu_cov = torch.ones(2, 15).to(device) * 0.1
    flow_qual = torch.rand(2).to(device)

    depth = torch.randn(2, 512, 512).to(device).clamp(min=0)
    depth_conf = torch.rand(2, 512, 512).to(device)
    depth_cov = torch.rand(2).to(device)

    prev_pose = torch.randn(2, 7).to(device)
    prev_vel = torch.randn(2, 3).to(device)
    time_kf = torch.rand(2).to(device)
    tracks = torch.randn(2, 10).to(device).softmax(dim=1)

    # Forward pass
    output = model(
        stereo, imu, flow,
        feature_matches=matches,
        match_quality=match_qual,
        imu_covariance=imu_cov,
        flow_quality=flow_qual,
        depth_map=depth,
        depth_confidence=depth_conf,
        depth_coverage=depth_cov,
        previous_pose=prev_pose,
        previous_velocity=prev_vel,
        time_since_keyframe=time_kf,
        track_lengths=tracks,
    )

    print(f"Pose shape: {output['pose'].shape}")
    print(f"Uncertainty shape: {output['uncertainty'].shape}")
    print(f"Attention weights: {output['attention_weights'][0]}")
    print(f"Total parameters: {sum(p.numel() for p in model.parameters()):,}")
