#!/usr/bin/env python3
"""
Training script for student network on Week 6 exported data (2,821 TUM-VI frames).

This script:
1. Loads metadata from exported_data/metadata.json
2. Creates PyTorch DataLoader for stereo pairs with optical flow, IMU, and depth
3. Trains student network for 5 epochs
4. Targets validation error < 0.02m
5. Saves checkpoint to results/student_checkpoint.pt

Usage:
    python3 train_student_network.py
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
    "num_epochs": 5,
    "batch_size": 8,
    "learning_rate": 1e-3,
    "weight_decay": 1e-5,
    "train_split": 0.8,
    "val_split": 0.2,
    "device": "cuda" if torch.cuda.is_available() else "cpu",
    "output_dir": Path("/Users/vincent/Work/RS-VIO/results"),
}

# Ensure output directory exists
CONFIG["output_dir"].mkdir(exist_ok=True)

print(f"Device: {CONFIG['device']}")
print(f"Using PyTorch {torch.__version__}")


# ============================================================================
# Dataset
# ============================================================================

class TUMVIDataset(Dataset):
    """
    Dataset for TUM-VI room1 exported data.
    
    Returns:
        - Left image (640x480 grayscale)
        - Right image (640x480 grayscale)
        - Optical flow (8x6 grid = 96 values)
        - Flow quality (inlier ratio)
        - IMU preintegration (15 values)
        - Ground truth pose (tx, ty, tz, qx, qy, qz, qw)
        - Timestamp
    """
    
    def __init__(self, metadata_list, image_root, depth_root, transform=None):
        self.metadata_list = metadata_list
        self.image_root = Path(image_root)
        self.depth_root = Path(depth_root)
        self.transform = transform
        
    def __len__(self):
        return len(self.metadata_list)
    
    def __getitem__(self, idx):
        frame = self.metadata_list[idx]
        
        # Load images
        left_path = self.image_root / "cam0" / (frame["image_files"]["left"].split("/")[-1])
        right_path = self.image_root / "cam1" / (frame["image_files"]["right"].split("/")[-1])
        
        left_img = self._load_image(left_path)
        right_img = self._load_image(right_path)
        
        # Extract sensor data
        imu_preint = np.array(frame["imu_preintegration"], dtype=np.float32)
        flow = np.array(frame["flow"], dtype=np.float32)
        flow_quality = frame["flow_quality"]
        
        # Ground truth pose (position only, regression task)
        gt_position = np.array([
            frame["pose_tx"],
            frame["pose_ty"],
            frame["pose_tz"]
        ], dtype=np.float32)
        
        # Timestamp
        timestamp = frame["timestamp_ns"]
        
        return {
            "left_image": torch.from_numpy(left_img).float().unsqueeze(0),
            "right_image": torch.from_numpy(right_img).float().unsqueeze(0),
            "imu_preint": torch.from_numpy(imu_preint).float(),
            "flow": torch.from_numpy(flow).float(),
            "flow_quality": torch.tensor(flow_quality, dtype=torch.float32),
            "gt_position": torch.from_numpy(gt_position).float(),
            "timestamp": timestamp,
        }
    
    def _load_image(self, path):
        """Load grayscale image, normalize to [0, 1]."""
        try:
            from PIL import Image
            img = Image.open(path).convert('L')
            return np.array(img, dtype=np.float32) / 255.0
        except Exception as e:
            print(f"Error loading {path}: {e}")
            # Return blank image on error
            return np.zeros((480, 640), dtype=np.float32)


# ============================================================================
# Student Network
# ============================================================================

class StudentNetwork(nn.Module):
    """
    Lightweight student network for depth refinement.
    
    Architecture:
    - Encoder: Conv layers on left image (128 -> 256 -> 512 features)
    - IMU branch: MLP on preintegration (15 -> 64 -> 128)
    - Optical flow branch: MLP on flow (96 -> 128 -> 128)
    - Fusion: Concatenate all branches + global average pooling
    - Decoder: MLP to predict position delta (tx, ty, tz)
    
    Output: 3D position prediction (tx, ty, tz)
    """
    
    def __init__(self):
        super().__init__()
        
        # Image encoder (lightweight)
        self.img_encoder = nn.Sequential(
            nn.Conv2d(1, 32, kernel_size=3, stride=2, padding=1),  # 320x240
            nn.ReLU(inplace=True),
            nn.Conv2d(32, 64, kernel_size=3, stride=2, padding=1),  # 160x120
            nn.ReLU(inplace=True),
            nn.Conv2d(64, 128, kernel_size=3, stride=2, padding=1),  # 80x60
            nn.ReLU(inplace=True),
            nn.AdaptiveAvgPool2d((1, 1)),
        )
        
        # IMU processor (preintegration)
        self.imu_processor = nn.Sequential(
            nn.Linear(15, 64),
            nn.ReLU(inplace=True),
            nn.Linear(64, 128),
            nn.ReLU(inplace=True),
        )
        
        # Optical flow processor
        self.flow_processor = nn.Sequential(
            nn.Linear(96, 128),
            nn.ReLU(inplace=True),
            nn.Linear(128, 128),
            nn.ReLU(inplace=True),
        )
        
        # Fusion MLP (128 + 128 + 128 = 384)
        self.fusion_mlp = nn.Sequential(
            nn.Linear(128 + 128 + 128, 256),
            nn.ReLU(inplace=True),
            nn.Dropout(0.2),
            nn.Linear(256, 128),
            nn.ReLU(inplace=True),
            nn.Dropout(0.2),
            nn.Linear(128, 3),  # Output: tx, ty, tz
        )
        
    def forward(self, left_img, imu_preint, flow):
        """
        Args:
            left_img: (B, 1, 640, 480)
            imu_preint: (B, 15)
            flow: (B, 96)
        
        Returns:
            position: (B, 3) - predicted tx, ty, tz
        """
        # Encode image
        img_feat = self.img_encoder(left_img)  # (B, 128, 1, 1)
        img_feat = img_feat.view(img_feat.size(0), -1)  # (B, 128)
        
        # Process IMU
        imu_feat = self.imu_processor(imu_preint)  # (B, 128)
        
        # Process flow
        flow_feat = self.flow_processor(flow)  # (B, 128)
        
        # Fuse all features
        fused = torch.cat([img_feat, imu_feat, flow_feat], dim=1)  # (B, 384)
        position = self.fusion_mlp(fused)  # (B, 3)
        
        return position


# ============================================================================
# Training
# ============================================================================

def load_metadata(path):
    """Load metadata from JSON file."""
    with open(path, 'r') as f:
        return json.load(f)


def train_epoch(model, dataloader, optimizer, criterion, device):
    """Train for one epoch."""
    model.train()
    total_loss = 0.0
    num_batches = 0
    
    for batch in dataloader:
        left_img = batch["left_image"].to(device)
        imu_preint = batch["imu_preint"].to(device)
        flow = batch["flow"].to(device)
        gt_position = batch["gt_position"].to(device)
        
        # Forward pass
        pred_position = model(left_img, imu_preint, flow)
        
        # Loss
        loss = criterion(pred_position, gt_position)
        
        # Backward pass
        optimizer.zero_grad()
        loss.backward()
        torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
        optimizer.step()
        
        total_loss += loss.item()
        num_batches += 1
    
    return total_loss / num_batches


def validate(model, dataloader, criterion, device):
    """Validation pass."""
    model.eval()
    total_loss = 0.0
    total_error = 0.0
    num_batches = 0
    
    with torch.no_grad():
        for batch in dataloader:
            left_img = batch["left_image"].to(device)
            imu_preint = batch["imu_preint"].to(device)
            flow = batch["flow"].to(device)
            gt_position = batch["gt_position"].to(device)
            
            # Forward pass
            pred_position = model(left_img, imu_preint, flow)
            
            # Loss
            loss = criterion(pred_position, gt_position)
            
            # Error (L2 distance in meters)
            error = torch.norm(pred_position - gt_position, dim=1).mean().item()
            
            total_loss += loss.item()
            total_error += error
            num_batches += 1
    
    avg_loss = total_loss / num_batches
    avg_error = total_error / num_batches
    
    return avg_loss, avg_error


def main():
    """Main training loop."""
    
    print("\n" + "="*80)
    print("STUDENT NETWORK TRAINING - WEEK 6")
    print("="*80)
    
    # Load metadata
    print("\nLoading metadata...")
    metadata = load_metadata(CONFIG["metadata_path"])
    print(f"Loaded {len(metadata)} frames")
    
    # Create dataset
    print("Creating dataset...")
    dataset = TUMVIDataset(
        metadata,
        CONFIG["image_root"],
        CONFIG["depth_root"]
    )
    
    # Split into train/val
    num_frames = len(dataset)
    num_train = int(num_frames * CONFIG["train_split"])
    num_val = num_frames - num_train
    
    train_dataset, val_dataset = torch.utils.data.random_split(
        dataset, [num_train, num_val]
    )
    
    print(f"Train set: {len(train_dataset)} frames")
    print(f"Val set: {len(val_dataset)} frames")
    
    # Create dataloaders
    train_loader = DataLoader(
        train_dataset,
        batch_size=CONFIG["batch_size"],
        shuffle=True,
        num_workers=0,
    )
    
    val_loader = DataLoader(
        val_dataset,
        batch_size=CONFIG["batch_size"],
        shuffle=False,
        num_workers=0,
    )
    
    # Create model
    print("\nInitializing student network...")
    model = StudentNetwork().to(CONFIG["device"])
    
    # Count parameters
    total_params = sum(p.numel() for p in model.parameters())
    print(f"Total parameters: {total_params:,}")
    
    # Optimizer and loss
    optimizer = optim.Adam(
        model.parameters(),
        lr=CONFIG["learning_rate"],
        weight_decay=CONFIG["weight_decay"]
    )
    criterion = nn.MSELoss()
    
    # Training loop
    print("\n" + "="*80)
    print("TRAINING")
    print("="*80)
    
    best_val_error = float('inf')
    start_time = time.time()
    
    for epoch in range(CONFIG["num_epochs"]):
        epoch_start = time.time()
        
        # Train
        train_loss = train_epoch(model, train_loader, optimizer, criterion, CONFIG["device"])
        
        # Validate
        val_loss, val_error = validate(model, val_loader, criterion, CONFIG["device"])
        
        epoch_time = time.time() - epoch_start
        
        print(f"Epoch {epoch+1}/{CONFIG['num_epochs']} | "
              f"Time: {epoch_time:.1f}s | "
              f"Train Loss: {train_loss:.6f} | "
              f"Val Loss: {val_loss:.6f} | "
              f"Val Error: {val_error:.6f}m")
        
        # Save checkpoint if best
        if val_error < best_val_error:
            best_val_error = val_error
            checkpoint_path = CONFIG["output_dir"] / "student_checkpoint.pt"
            torch.save({
                "epoch": epoch,
                "model_state_dict": model.state_dict(),
                "optimizer_state_dict": optimizer.state_dict(),
                "val_error": val_error,
                "config": CONFIG,
            }, checkpoint_path)
            print(f"  -> Saved checkpoint to {checkpoint_path}")
    
    total_time = time.time() - start_time
    
    # Final summary
    print("\n" + "="*80)
    print("TRAINING COMPLETE")
    print("="*80)
    print(f"Total training time: {total_time/3600:.2f} hours ({total_time/60:.1f} minutes)")
    print(f"Best validation error: {best_val_error:.6f}m")
    
    if best_val_error < 0.02:
        print("✓ SUCCESS: Validation error < 0.02m")
    else:
        print(f"⚠ WARNING: Validation error {best_val_error:.6f}m > 0.02m target")
    
    # Save training summary
    summary = {
        "timestamp": datetime.now().isoformat(),
        "num_frames": num_frames,
        "num_epochs": CONFIG["num_epochs"],
        "batch_size": CONFIG["batch_size"],
        "learning_rate": CONFIG["learning_rate"],
        "total_training_time_seconds": total_time,
        "best_validation_error": float(best_val_error),
        "device": CONFIG["device"],
        "model_parameters": int(total_params),
    }
    
    summary_path = CONFIG["output_dir"] / "training_summary.json"
    with open(summary_path, 'w') as f:
        json.dump(summary, f, indent=2)
    print(f"\nSummary saved to {summary_path}")


if __name__ == "__main__":
    main()
