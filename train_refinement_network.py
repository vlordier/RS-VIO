#!/usr/bin/env python3
"""
Training script for position refinement network.

Key differences from absolute prediction:
1. Input includes VIO estimate (baseline position)
2. Output is a CORRECTION/DELTA to apply to VIO estimate
3. Loss function measures correction accuracy
4. Network learns to refine, not replace, VIO estimates

This script:
1. Loads metadata from exported_data/metadata.json
2. Simulates VIO baseline estimates (later replaced with real Rust VIO)
3. Trains network to predict position corrections
4. Exports model to ONNX for Rust integration
5. Saves checkpoint to results/refinement_checkpoint.pt

Usage:
    python3 train_refinement_network.py
"""

import json
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader
from pathlib import Path
import time
from datetime import datetime

# ============================================================================
# Configuration
# ============================================================================

CONFIG = {
    "metadata_path": "/Users/vincent/Work/RS-VIO/exported_data/metadata.json",
    "image_root": "/Users/vincent/Work/RS-VIO/exported_data/images",
    "depth_root": "/Users/vincent/Work/RS-VIO/exported_data/depth_maps",
    "num_epochs": 10,
    "batch_size": 8,
    "learning_rate": 1e-3,
    "weight_decay": 1e-5,
    "train_split": 0.8,
    "val_split": 0.2,
    "device": "cuda" if torch.cuda.is_available() else "cpu",
    "output_dir": Path("/Users/vincent/Work/RS-VIO/results"),
    # Simulated VIO noise parameters (replaced with real VIO later)
    "vio_noise_std": 0.05,  # 5cm std dev for simulated VIO
    "vio_bias_std": 0.02,   # 2cm systematic bias
    # Data augmentation parameters
    "augment_train": True,
    "aug_vio_noise_range": (0.02, 0.10),  # Vary VIO noise 2-10cm
    "aug_imu_noise_std": 0.05,  # Add 5% noise to IMU
    "aug_flow_noise_std": 0.5,  # Add noise to flow
}

# Ensure output directory exists
CONFIG["output_dir"].mkdir(exist_ok=True)


# ============================================================================
# Dataset for Refinement Learning
# ============================================================================

class RefinementDataset(Dataset):
    """
    Dataset for learning position refinement.
    
    Returns:
        - Left image (640x480 grayscale) -> Will use features later
        - Right image (640x480 grayscale)
        - Optical flow (8x6 grid = 96 values)
        - IMU preintegration (15 values)
        - VIO estimate (simulated baseline, tx, ty, tz)
        - Ground truth correction (gt_position - vio_estimate)
        - Ground truth position (tx, ty, tz)
        - Timestamp
    """
    
    def __init__(self, metadata_list, image_root, depth_root, vio_noise_std=0.05, vio_bias_std=0.02, 
                 augment=False, aug_vio_range=(0.02, 0.10), aug_imu_std=0.05, aug_flow_std=0.5, transform=None):
        self.metadata_list = metadata_list
        self.image_root = Path(image_root)
        self.depth_root = Path(depth_root)
        self.vio_noise_std = vio_noise_std
        self.vio_bias_std = vio_bias_std
        self.augment = augment
        self.aug_vio_range = aug_vio_range
        self.aug_imu_std = aug_imu_std
        self.aug_flow_std = aug_flow_std
        self.transform = transform
        
        # Generate consistent bias per sequence (simulates VIO drift)
        np.random.seed(42)
        self.sequence_bias = np.random.randn(3) * vio_bias_std
        
    def __len__(self):
        return len(self.metadata_list)
    
    def __getitem__(self, idx):
        frame = self.metadata_list[idx]
        
        # Load images (placeholder for now, will use features later)
        left_path = self.image_root / "cam0" / (frame["image_files"]["left"].split("/")[-1])
        right_path = self.image_root / "cam1" / (frame["image_files"]["right"].split("/")[-1])
        
        left_img = self._load_image(left_path)
        right_img = self._load_image(right_path)
        
        # Extract sensor data
        imu_preint = np.array(frame["imu_preintegration"], dtype=np.float32)
        flow = np.array(frame["flow"], dtype=np.float32)
        flow_quality = frame["flow_quality"]
        
        # Ground truth position
        gt_position = np.array([
            frame["pose_tx"],
            frame["pose_ty"],
            frame["pose_tz"]
        ], dtype=np.float32)
        
        # Simulate VIO baseline estimate (noisy version of ground truth)
        # This will be replaced with real Rust VIO estimates in production
        if self.augment:
            # Augment: Vary VIO noise level per sample
            vio_noise_std = np.random.uniform(self.aug_vio_range[0], self.aug_vio_range[1])
            noise = np.random.randn(3).astype(np.float32) * vio_noise_std
        else:
            noise = np.random.randn(3).astype(np.float32) * self.vio_noise_std
        vio_estimate = gt_position + noise + self.sequence_bias.astype(np.float32)
        
        # Apply augmentation to sensor inputs if enabled
        if self.augment:
            # Add noise to IMU measurements
            imu_noise = np.random.randn(*imu_preint.shape).astype(np.float32) * self.aug_imu_std * np.abs(imu_preint)
            imu_preint = imu_preint + imu_noise
            
            # Add noise to optical flow
            flow_noise = np.random.randn(*flow.shape).astype(np.float32) * self.aug_flow_std
            flow = flow + flow_noise
        
        # Compute correction (what the network should learn to predict)
        correction = gt_position - vio_estimate
        
        # Timestamp
        timestamp = frame["timestamp_ns"]
        
        return {
            "left_image": left_img,
            "right_image": right_img,
            "flow": torch.from_numpy(flow),
            "flow_quality": torch.tensor(flow_quality, dtype=torch.float32),
            "imu": torch.from_numpy(imu_preint),
            "vio_estimate": torch.from_numpy(vio_estimate),
            "correction": torch.from_numpy(correction),  # Target for training
            "gt_position": torch.from_numpy(gt_position),
            "timestamp": timestamp,
        }
    
    def _load_image(self, path):
        """Load grayscale image as torch tensor (1, H, W)"""
        from PIL import Image
        img = Image.open(path).convert("L")
        img_array = np.array(img, dtype=np.float32) / 255.0
        return torch.from_numpy(img_array).unsqueeze(0)


# ============================================================================
# Refinement Network Architecture
# ============================================================================

class RefinementNetwork(nn.Module):
    """
    Network for learning position corrections (OPTIMIZED FOR LOW COMPUTE).
    
    Key design principles for embedded/real-time deployment:
    1. Simple concatenation fusion (no attention overhead)
    2. Batch normalization for stability
    3. Direct VIO skip connection (preserve good estimates)
    4. Learnable correction scale (encourage small corrections)
    5. Minimal depth (3-layer MLP for fusion)
    
    Input:
        - Image features (from Conv encoder)
        - Optical flow (96 values)
        - IMU preintegration (15 values)
        - VIO estimate (3 values)
    
    Output:
        - Position correction (delta_x, delta_y, delta_z)
    """
    
    def __init__(self):
        super().__init__()
        
        # Image encoder (lightweight CNN with batch norm)
        self.img_encoder = nn.Sequential(
            nn.Conv2d(1, 32, kernel_size=5, stride=2, padding=2),
            nn.BatchNorm2d(32),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(2),
            nn.Conv2d(32, 64, kernel_size=3, stride=2, padding=1),
            nn.BatchNorm2d(64),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(2),
            nn.Conv2d(64, 128, kernel_size=3, stride=2, padding=1),
            nn.BatchNorm2d(128),
            nn.ReLU(inplace=True),
            nn.AdaptiveAvgPool2d((1, 1))
        )
        
        # IMU processor (single layer for efficiency)
        self.imu_processor = nn.Sequential(
            nn.Linear(15, 64),
            nn.BatchNorm1d(64),
            nn.ReLU(inplace=True),
        )
        
        # Flow processor (single layer for efficiency)
        self.flow_processor = nn.Sequential(
            nn.Linear(96, 64),
            nn.BatchNorm1d(64),
            nn.ReLU(inplace=True),
        )
        
        # Fusion layer: Simple concatenation + small MLP
        # 128 (img) + 64 (imu) + 64 (flow) + 3 (vio) = 259
        self.fusion_mlp = nn.Sequential(
            nn.Linear(128 + 64 + 64 + 3, 128),
            nn.BatchNorm1d(128),
            nn.ReLU(inplace=True),
            nn.Dropout(0.1),
            nn.Linear(128, 64),
            nn.BatchNorm1d(64),
            nn.ReLU(inplace=True),
            nn.Linear(64, 3),  # Output: (delta_x, delta_y, delta_z)
        )
        
        # Learnable correction scale (initialized small)
        # Encourages small corrections, prevents large jumps
        self.correction_scale = nn.Parameter(torch.tensor(0.1))
        
    def forward(self, left_img, flow, imu, vio_estimate):
        # Encode image
        img_features = self.img_encoder(left_img)
        img_features = img_features.view(img_features.size(0), -1)
        
        # Process sensors (single-layer for speed)
        imu_features = self.imu_processor(imu)
        flow_features = self.flow_processor(flow)
        
        # Simple concatenation fusion (fast, no attention overhead)
        # Direct VIO skip connection preserves good estimates
        combined = torch.cat([img_features, imu_features, flow_features, vio_estimate], dim=1)
        
        # Predict correction with learnable scale
        correction = self.fusion_mlp(combined)
        correction = correction * self.correction_scale
        
        return correction


# ============================================================================
# Training Loop
# ============================================================================

def train_refinement_network():
    """Main training function for refinement network."""
    
    print("\n" + "="*70)
    print("TRAINING REFINEMENT NETWORK")
    print("="*70)
    print(f"Task: Learn position CORRECTIONS (not absolute positions)")
    print(f"Input: Visual features + IMU + Flow + VIO estimate")
    print(f"Output: Position delta (delta_x, delta_y, delta_z)")
    print("="*70 + "\n")
    
    # Load metadata
    print(f"Loading metadata from {CONFIG['metadata_path']}...")
    with open(CONFIG['metadata_path'], 'r') as f:
        metadata = json.load(f)
    
    print(f"Total frames: {len(metadata)}")
    
    # Split into train/val
    n_train = int(len(metadata) * CONFIG['train_split'])
    train_meta = metadata[:n_train]
    val_meta = metadata[n_train:]
    
    print(f"Train: {len(train_meta)}, Val: {len(val_meta)}")
    
    # Create datasets
    train_dataset = RefinementDataset(
        train_meta, 
        CONFIG['image_root'], 
        CONFIG['depth_root'],
        vio_noise_std=CONFIG['vio_noise_std'],
        vio_bias_std=CONFIG['vio_bias_std'],
        augment=CONFIG['augment_train'],
        aug_vio_range=CONFIG['aug_vio_noise_range'],
        aug_imu_std=CONFIG['aug_imu_noise_std'],
        aug_flow_std=CONFIG['aug_flow_noise_std']
    )
    val_dataset = RefinementDataset(
        val_meta, 
        CONFIG['image_root'], 
        CONFIG['depth_root'],
        vio_noise_std=CONFIG['vio_noise_std'],
        vio_bias_std=CONFIG['vio_bias_std'],
        augment=False  # No augmentation for validation
    )
    
    train_loader = DataLoader(train_dataset, batch_size=CONFIG['batch_size'], shuffle=True, num_workers=4)
    val_loader = DataLoader(val_dataset, batch_size=CONFIG['batch_size'], shuffle=False, num_workers=4)
    
    # Create model
    model = RefinementNetwork().to(CONFIG['device'])
    
    # Count parameters
    total_params = sum(p.numel() for p in model.parameters())
    trainable_params = sum(p.numel() for p in model.parameters() if p.requires_grad)
    print(f"\nModel parameters: {total_params:,} (trainable: {trainable_params:,})")
    
    # Loss function: L2 norm (Euclidean distance) for position correction
    # More interpretable than MSE - directly measures correction error in meters
    def l2_loss(pred, target):
        """L2 norm loss: mean Euclidean distance between predictions and targets"""
        return torch.norm(pred - target, dim=1).mean()
    
    criterion = l2_loss
    
    # Optimizer
    optimizer = optim.Adam(model.parameters(), lr=CONFIG['learning_rate'], weight_decay=CONFIG['weight_decay'])
    
    # Learning rate scheduler
    scheduler = optim.lr_scheduler.ReduceLROnPlateau(optimizer, mode='min', factor=0.5, patience=2)
    
    # Training loop
    best_val_loss = float('inf')
    training_start = time.time()
    
    for epoch in range(CONFIG['num_epochs']):
        epoch_start = time.time()
        
        # Training
        model.train()
        train_loss = 0.0
        train_correction_error = 0.0
        
        for batch_idx, batch in enumerate(train_loader):
            left_img = batch['left_image'].to(CONFIG['device'])
            flow = batch['flow'].to(CONFIG['device'])
            imu = batch['imu'].to(CONFIG['device'])
            vio_estimate = batch['vio_estimate'].to(CONFIG['device'])
            correction_gt = batch['correction'].to(CONFIG['device'])
            
            optimizer.zero_grad()
            
            # Forward pass
            correction_pred = model(left_img, flow, imu, vio_estimate)
            
            # Loss: How well does the network predict the correction?
            loss = criterion(correction_pred, correction_gt)
            
            # Backward pass
            loss.backward()
            optimizer.step()
            
            train_loss += loss.item()
            
            # Measure correction error (L2 norm)
            with torch.no_grad():
                correction_error = torch.norm(correction_pred - correction_gt, dim=1).mean()
                train_correction_error += correction_error.item()
        
        train_loss /= len(train_loader)
        train_correction_error /= len(train_loader)
        
        # Validation
        model.eval()
        val_loss = 0.0
        val_correction_error = 0.0
        val_refined_error = 0.0  # Error after applying correction
        
        with torch.no_grad():
            for batch in val_loader:
                left_img = batch['left_image'].to(CONFIG['device'])
                flow = batch['flow'].to(CONFIG['device'])
                imu = batch['imu'].to(CONFIG['device'])
                vio_estimate = batch['vio_estimate'].to(CONFIG['device'])
                correction_gt = batch['correction'].to(CONFIG['device'])
                gt_position = batch['gt_position'].to(CONFIG['device'])
                
                # Forward pass
                correction_pred = model(left_img, flow, imu, vio_estimate)
                
                # Loss
                loss = criterion(correction_pred, correction_gt)
                val_loss += loss.item()
                
                # Measure correction error
                correction_error = torch.norm(correction_pred - correction_gt, dim=1).mean()
                val_correction_error += correction_error.item()
                
                # Measure final position error (VIO + NN correction vs ground truth)
                refined_position = vio_estimate + correction_pred
                refined_error = torch.norm(refined_position - gt_position, dim=1).mean()
                val_refined_error += refined_error.item()
        
        val_loss /= len(val_loader)
        val_correction_error /= len(val_loader)
        val_refined_error /= len(val_loader)
        
        # Scheduler step
        scheduler.step(val_loss)
        
        # Print progress
        epoch_time = time.time() - epoch_start
        print(f"\nEpoch [{epoch+1}/{CONFIG['num_epochs']}] ({epoch_time:.1f}s)")
        print(f"  Train: L2 Loss={train_loss:.4f}m")
        print(f"  Val:   L2 Loss={val_loss:.4f}m, Refined Error={val_refined_error:.4f}m")
        
        # Save best model
        if val_loss < best_val_loss:
            best_val_loss = val_loss
            checkpoint_path = CONFIG['output_dir'] / 'refinement_checkpoint.pt'
            torch.save({
                'epoch': epoch,
                'model_state_dict': model.state_dict(),
                'optimizer_state_dict': optimizer.state_dict(),
                'val_loss': val_loss,
                'val_correction_error': val_correction_error,
                'val_refined_error': val_refined_error,
                'config': CONFIG,
            }, checkpoint_path)
            print(f"  ✓ Saved checkpoint (L2_loss={val_loss:.4f}m)")
    
    training_time = time.time() - training_start
    print(f"\n{'='*70}")
    print(f"Training completed in {training_time/60:.1f} minutes")
    print(f"Best validation correction error: {best_val_loss:.6f}")
    print(f"{'='*70}\n")
    
    return model


# ============================================================================
# ONNX Export
# ============================================================================

def export_to_onnx(model, output_path):
    """Export trained model to ONNX for Rust integration."""
    
    print("\n" + "="*70)
    print("EXPORTING TO ONNX")
    print("="*70)
    
    model.eval()
    
    # Dummy inputs (batch_size=1)
    dummy_left_img = torch.randn(1, 1, 480, 640).to(CONFIG['device'])
    dummy_flow = torch.randn(1, 96).to(CONFIG['device'])
    dummy_imu = torch.randn(1, 15).to(CONFIG['device'])
    dummy_vio = torch.randn(1, 3).to(CONFIG['device'])
    
    # Export
    torch.onnx.export(
        model,
        (dummy_left_img, dummy_flow, dummy_imu, dummy_vio),
        output_path,
        export_params=True,
        opset_version=14,
        do_constant_folding=True,
        input_names=['left_image', 'flow', 'imu', 'vio_estimate'],
        output_names=['correction'],
        dynamic_axes={
            'left_image': {0: 'batch_size'},
            'flow': {0: 'batch_size'},
            'imu': {0: 'batch_size'},
            'vio_estimate': {0: 'batch_size'},
            'correction': {0: 'batch_size'}
        }
    )
    
    print(f"✓ ONNX model saved to {output_path}")
    print(f"  Input names: left_image (1,1,480,640), flow (1,96), imu (1,15), vio_estimate (1,3)")
    print(f"  Output names: correction (1,3)")
    print("="*70 + "\n")


# ============================================================================
# Main
# ============================================================================

if __name__ == "__main__":
    print(f"Device: {CONFIG['device']}")
    print(f"Using PyTorch {torch.__version__}")
    print()
    
    # Train network
    model = train_refinement_network()
    
    # Export to ONNX
    onnx_path = CONFIG['output_dir'] / 'refinement_model.onnx'
    export_to_onnx(model, onnx_path)
    
    print("\n✓ Training and export complete!")
    print(f"  PyTorch checkpoint: {CONFIG['output_dir'] / 'refinement_checkpoint.pt'}")
    print(f"  ONNX model: {onnx_path}")
    print("\nNext steps:")
    print("  1. Integrate ONNX model into Rust VIO pipeline")
    print("  2. Replace simulated VIO estimates with real Rust VIO")
    print("  3. Validate on complete sequences")
