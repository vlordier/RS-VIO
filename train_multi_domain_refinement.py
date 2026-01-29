#!/usr/bin/env python3
"""
Multi-domain training for robust VIO refinement across diverse missions.

Strategies for generalization:
1. Multi-dataset training (indoor, outdoor, different lighting)
2. Domain-specific augmentation (brightness, contrast, weather)
3. Curriculum learning (easy → hard scenarios)
4. Uncertainty estimation (network confidence)
5. Online adaptation support (fine-tuning during deployment)

Datasets to use:
- TUM-VI: Indoor lab environments
- EuRoC MAV: Industrial + outdoor
- UZH-FPV: Aggressive drone flight
- KITTI: Outdoor driving (if available)
- Your custom drone missions

Usage:
    python3 train_multi_domain_refinement.py --datasets tumvi euroc --epochs 20
"""

import json
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader, ConcatDataset
from pathlib import Path
import time
from datetime import datetime
import argparse
from typing import List, Dict, Tuple

# ============================================================================
# Enhanced Configuration
# ============================================================================

BASE_CONFIG = {
    "num_epochs": 20,
    "batch_size": 8,
    "learning_rate": 1e-3,
    "weight_decay": 1e-5,
    "train_split": 0.8,
    "device": "cuda" if torch.cuda.is_available() else "cpu",
    "output_dir": Path("/Users/vincent/Work/RS-VIO/results"),
    
    # VIO noise simulation (will be replaced with real VIO)
    "vio_noise_std": 0.05,
    "vio_bias_std": 0.02,
    
    # Enhanced augmentation for domain robustness
    "augment_train": True,
    "aug_vio_noise_range": (0.01, 0.15),  # Wider range: 1-15cm
    "aug_imu_noise_std": 0.10,  # 10% IMU noise
    "aug_flow_noise_std": 1.0,  # Higher flow noise
    
    # Domain-specific augmentation
    "aug_brightness_range": (0.7, 1.3),  # ±30% brightness
    "aug_contrast_range": (0.8, 1.2),  # ±20% contrast
    "aug_gaussian_noise_std": 0.02,  # Image noise for low-light
    "aug_motion_blur_prob": 0.1,  # Simulate fast motion
    
    # Curriculum learning
    "curriculum_learning": True,
    "curriculum_start_noise": 0.02,  # Start with 2cm VIO error
    "curriculum_end_noise": 0.15,  # End with 15cm VIO error
    
    # Uncertainty estimation
    "estimate_uncertainty": True,
    "mc_dropout_samples": 10,  # Monte Carlo dropout for uncertainty
}

# Dataset configurations - Auto-discover from exported_data_multi/
def discover_datasets(base_path="/Users/vincent/Work/RS-VIO/exported_data_multi"):
    """Auto-discover exported datasets"""
    from pathlib import Path
    configs = {}
    
    base = Path(base_path)
    if not base.exists():
        # Fallback to single dataset
        return {
            "tumvi_room1": {
                "metadata_path": "/Users/vincent/Work/RS-VIO/exported_data/metadata.json",
                "image_root": "/Users/vincent/Work/RS-VIO/exported_data/images",
                "depth_root": "/Users/vincent/Work/RS-VIO/exported_data/depth_maps",
                "environment": "indoor",
                "lighting": "controlled",
                "motion_type": "slow_walking",
            }
        }
    
    # Scan for exported datasets
    for dataset_dir in base.iterdir():
        if dataset_dir.is_dir():
            metadata_path = dataset_dir / "metadata.json"
            if metadata_path.exists():
                dataset_name = dataset_dir.name
                
                # Infer environment from name
                if "outdoor" in dataset_name or "4seasons" in dataset_name or "magistrale" in dataset_name:
                    env = "outdoor"
                    lighting = "variable"
                    motion = "variable"
                elif "euroc" in dataset_name:
                    env = "industrial"
                    lighting = "indoor_industrial"
                    motion = "slow_to_medium"
                    if "difficult" in dataset_name:
                        motion = "aggressive"
                else:
                    env = "indoor"
                    lighting = "controlled"
                    motion = "slow_walking"
                
                configs[dataset_name] = {
                    "metadata_path": str(metadata_path),
                    "image_root": str(dataset_dir / "images"),
                    "depth_root": str(dataset_dir / "depth_maps"),
                    "environment": env,
                    "lighting": lighting,
                    "motion_type": motion,
                }
    
    return configs

DATASET_CONFIGS = discover_datasets()


# ============================================================================
# Enhanced Dataset with Domain Adaptation
# ============================================================================

class MultiDomainRefinementDataset(Dataset):
    """
    Enhanced dataset with domain-specific augmentation.
    
    Supports:
    - Multiple datasets (indoor, outdoor, etc.)
    - Brightness/contrast variation (lighting conditions)
    - Motion blur (fast flight)
    - Gaussian noise (low-light)
    - Curriculum learning (progressive difficulty)
    """
    
    def __init__(self, 
                 metadata_list, 
                 image_root, 
                 depth_root,
                 dataset_name: str,
                 vio_noise_std=0.05,
                 vio_bias_std=0.02,
                 augment=False,
                 aug_config=None,
                 curriculum_epoch=None,
                 max_epochs=20):
        
        self.metadata_list = metadata_list
        self.image_root = Path(image_root)
        self.depth_root = Path(depth_root)
        self.dataset_name = dataset_name
        self.vio_noise_std = vio_noise_std
        self.vio_bias_std = vio_bias_std
        self.augment = augment
        self.aug_config = aug_config or {}
        
        # Curriculum learning: progressively increase difficulty
        if curriculum_epoch is not None and self.aug_config.get('curriculum_learning', False):
            progress = curriculum_epoch / max_epochs
            start_noise = self.aug_config.get('curriculum_start_noise', 0.02)
            end_noise = self.aug_config.get('curriculum_end_noise', 0.15)
            self.vio_noise_std = start_noise + progress * (end_noise - start_noise)
            print(f"[Curriculum] Epoch {curriculum_epoch}/{max_epochs}: VIO noise = {self.vio_noise_std:.3f}m")
        
        # Generate consistent bias per sequence
        np.random.seed(42)
        self.sequence_bias = np.random.randn(3) * vio_bias_std
        
    def __len__(self):
        return len(self.metadata_list)
    
    def _augment_image(self, img_array: np.ndarray) -> np.ndarray:
        """Apply domain-specific image augmentation"""
        if not self.augment:
            return img_array
        
        # Brightness augmentation (lighting variation)
        if 'aug_brightness_range' in self.aug_config:
            brightness_factor = np.random.uniform(*self.aug_config['aug_brightness_range'])
            img_array = np.clip(img_array * brightness_factor, 0, 255)
        
        # Contrast augmentation
        if 'aug_contrast_range' in self.aug_config:
            contrast_factor = np.random.uniform(*self.aug_config['aug_contrast_range'])
            mean = img_array.mean()
            img_array = np.clip((img_array - mean) * contrast_factor + mean, 0, 255)
        
        # Gaussian noise (low-light conditions)
        if 'aug_gaussian_noise_std' in self.aug_config:
            if np.random.rand() < 0.3:  # 30% chance
                noise = np.random.randn(*img_array.shape) * self.aug_config['aug_gaussian_noise_std'] * 255
                img_array = np.clip(img_array + noise, 0, 255)
        
        # Motion blur (fast flight)
        if self.aug_config.get('aug_motion_blur_prob', 0) > 0:
            if np.random.rand() < self.aug_config['aug_motion_blur_prob']:
                kernel_size = np.random.randint(3, 8)
                kernel = np.zeros((kernel_size, kernel_size))
                kernel[kernel_size // 2, :] = 1.0 / kernel_size
                # Simple horizontal motion blur (could use cv2 for better quality)
                from scipy import ndimage
                img_array = ndimage.convolve(img_array, kernel, mode='reflect')
        
        return img_array
    
    def __getitem__(self, idx):
        frame = self.metadata_list[idx]
        
        # Load images
        left_path = self.image_root / "cam0" / (frame["image_files"]["left"].split("/")[-1])
        left_img = self._load_image(left_path)
        
        # Extract sensor data
        imu_preint = np.array(frame["imu_preintegration"], dtype=np.float32)
        flow = np.array(frame["flow"], dtype=np.float32)
        
        # Ground truth position
        gt_position = np.array([
            frame["pose_tx"],
            frame["pose_ty"],
            frame["pose_tz"]
        ], dtype=np.float32)
        
        # Simulate VIO with variable noise
        if self.augment and 'aug_vio_noise_range' in self.aug_config:
            vio_noise_std = np.random.uniform(*self.aug_config['aug_vio_noise_range'])
        else:
            vio_noise_std = self.vio_noise_std
        
        noise = np.random.randn(3).astype(np.float32) * vio_noise_std
        vio_estimate = gt_position + noise + self.sequence_bias.astype(np.float32)
        
        # Augment sensor inputs
        if self.augment:
            # IMU noise
            if 'aug_imu_noise_std' in self.aug_config:
                imu_noise = np.random.randn(*imu_preint.shape).astype(np.float32)
                imu_noise *= self.aug_config['aug_imu_noise_std'] * np.abs(imu_preint)
                imu_preint = imu_preint + imu_noise
            
            # Flow noise
            if 'aug_flow_noise_std' in self.aug_config:
                flow_noise = np.random.randn(*flow.shape).astype(np.float32)
                flow_noise *= self.aug_config['aug_flow_noise_std']
                flow = flow + flow_noise
        
        correction = gt_position - vio_estimate
        
        return {
            "left_image": left_img,
            "flow": torch.from_numpy(flow),
            "imu": torch.from_numpy(imu_preint),
            "vio_estimate": torch.from_numpy(vio_estimate),
            "correction": torch.from_numpy(correction),
            "gt_position": torch.from_numpy(gt_position),
            "timestamp": frame["timestamp_ns"],
            "dataset": self.dataset_name,
        }
    
    def _load_image(self, path):
        """Load and augment grayscale image"""
        from PIL import Image
        img = Image.open(path).convert("L")
        img_array = np.array(img, dtype=np.float32)
        
        # Apply domain-specific augmentation
        img_array = self._augment_image(img_array)
        
        # Normalize to [0, 1]
        img_array = img_array / 255.0
        return torch.from_numpy(img_array).unsqueeze(0)


# ============================================================================
# Network with Uncertainty Estimation
# ============================================================================

class RefinementNetworkWithUncertainty(nn.Module):
    """
    Refinement network with MC Dropout for uncertainty estimation.
    
    During training: Regular dropout
    During inference: Can enable dropout to estimate uncertainty via MC sampling
    """
    
    def __init__(self, dropout_rate=0.1):
        super().__init__()
        
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
        
        self.imu_processor = nn.Sequential(
            nn.Linear(15, 64),
            nn.BatchNorm1d(64),
            nn.ReLU(inplace=True),
            nn.Dropout(dropout_rate),
        )
        
        self.flow_processor = nn.Sequential(
            nn.Linear(96, 64),
            nn.BatchNorm1d(64),
            nn.ReLU(inplace=True),
            nn.Dropout(dropout_rate),
        )
        
        self.fusion_mlp = nn.Sequential(
            nn.Linear(128 + 64 + 64 + 3, 128),
            nn.BatchNorm1d(128),
            nn.ReLU(inplace=True),
            nn.Dropout(dropout_rate),
            nn.Linear(128, 64),
            nn.BatchNorm1d(64),
            nn.ReLU(inplace=True),
            nn.Dropout(dropout_rate),  # MC Dropout for uncertainty
            nn.Linear(64, 3),
        )
        
        self.correction_scale = nn.Parameter(torch.tensor(0.1))
        
    def forward(self, left_img, flow, imu, vio_estimate):
        img_features = self.img_encoder(left_img).view(left_img.size(0), -1)
        imu_features = self.imu_processor(imu)
        flow_features = self.flow_processor(flow)
        
        combined = torch.cat([img_features, imu_features, flow_features, vio_estimate], dim=1)
        correction = self.fusion_mlp(combined)
        correction = correction * self.correction_scale
        
        return correction
    
    def predict_with_uncertainty(self, left_img, flow, imu, vio_estimate, n_samples=10):
        """
        Predict with uncertainty estimation using MC Dropout.
        
        Returns:
            mean_correction: Mean prediction
            std_correction: Standard deviation (uncertainty)
        """
        self.train()  # Enable dropout
        
        predictions = []
        for _ in range(n_samples):
            with torch.no_grad():
                pred = self.forward(left_img, flow, imu, vio_estimate)
                predictions.append(pred.cpu().numpy())
        
        predictions = np.array(predictions)
        mean_correction = predictions.mean(axis=0)
        std_correction = predictions.std(axis=0)
        
        return mean_correction, std_correction


# ============================================================================
# Training Function
# ============================================================================

def train_multi_domain(datasets: List[str], config: Dict):
    """Train on multiple datasets for robust generalization"""
    
    print("\n" + "="*70)
    print("MULTI-DOMAIN REFINEMENT NETWORK TRAINING")
    print("="*70)
    print(f"Datasets: {', '.join(datasets)}")
    print(f"Epochs: {config['num_epochs']}")
    print(f"Batch size: {config['batch_size']}")
    print(f"Device: {config['device']}")
    print(f"Augmentation: {config['augment_train']}")
    print(f"Curriculum learning: {config.get('curriculum_learning', False)}")
    print("="*70 + "\n")
    
    # Load all datasets
    all_train_datasets = []
    all_val_datasets = []
    
    for dataset_name in datasets:
        if dataset_name not in DATASET_CONFIGS:
            print(f"Warning: Dataset '{dataset_name}' not configured, skipping")
            continue
        
        ds_config = DATASET_CONFIGS[dataset_name]
        print(f"Loading {dataset_name}...")
        
        with open(ds_config['metadata_path'], 'r') as f:
            metadata = json.load(f)
        
        split_idx = int(len(metadata) * config['train_split'])
        train_meta = metadata[:split_idx]
        val_meta = metadata[split_idx:]
        
        # Create datasets (curriculum_epoch will be updated during training)
        train_ds = MultiDomainRefinementDataset(
            train_meta,
            ds_config['image_root'],
            ds_config['depth_root'],
            dataset_name=dataset_name,
            vio_noise_std=config['vio_noise_std'],
            vio_bias_std=config['vio_bias_std'],
            augment=config['augment_train'],
            aug_config=config,
            curriculum_epoch=0,
            max_epochs=config['num_epochs']
        )
        
        val_ds = MultiDomainRefinementDataset(
            val_meta,
            ds_config['image_root'],
            ds_config['depth_root'],
            dataset_name=dataset_name,
            vio_noise_std=config['vio_noise_std'],
            vio_bias_std=config['vio_bias_std'],
            augment=False
        )
        
        all_train_datasets.append(train_ds)
        all_val_datasets.append(val_ds)
        
        print(f"  {dataset_name}: {len(train_meta)} train, {len(val_meta)} val")
    
    # Combine all datasets
    if len(all_train_datasets) > 1:
        train_dataset = ConcatDataset(all_train_datasets)
        val_dataset = ConcatDataset(all_val_datasets)
    else:
        train_dataset = all_train_datasets[0]
        val_dataset = all_val_datasets[0]
    
    print(f"\nTotal: {len(train_dataset)} train, {len(val_dataset)} val\n")
    
    train_loader = DataLoader(train_dataset, batch_size=config['batch_size'], 
                              shuffle=True, num_workers=4)
    val_loader = DataLoader(val_dataset, batch_size=config['batch_size'], 
                            shuffle=False, num_workers=4)
    
    # Create model
    model = RefinementNetworkWithUncertainty().to(config['device'])
    
    # Count parameters
    total_params = sum(p.numel() for p in model.parameters())
    print(f"Model parameters: {total_params:,}\n")
    
    # Loss and optimizer
    def l2_loss(pred, target):
        return torch.norm(pred - target, dim=1).mean()
    
    optimizer = optim.Adam(model.parameters(), lr=config['learning_rate'], 
                          weight_decay=config['weight_decay'])
    scheduler = optim.lr_scheduler.ReduceLROnPlateau(optimizer, mode='min', 
                                                     factor=0.5, patience=3)
    
    # Training loop
    best_val_loss = float('inf')
    
    for epoch in range(config['num_epochs']):
        epoch_start = time.time()
        
        # Update curriculum difficulty
        if config.get('curriculum_learning', False):
            for ds in all_train_datasets:
                ds.curriculum_epoch = epoch
                ds.max_epochs = config['num_epochs']
        
        # Training
        model.train()
        train_loss = 0.0
        
        for batch in train_loader:
            left_img = batch['left_image'].to(config['device'])
            flow = batch['flow'].to(config['device'])
            imu = batch['imu'].to(config['device'])
            vio_estimate = batch['vio_estimate'].to(config['device'])
            correction_gt = batch['correction'].to(config['device'])
            
            optimizer.zero_grad()
            correction_pred = model(left_img, flow, imu, vio_estimate)
            loss = l2_loss(correction_pred, correction_gt)
            loss.backward()
            optimizer.step()
            
            train_loss += loss.item()
        
        train_loss /= len(train_loader)
        
        # Validation
        model.eval()
        val_loss = 0.0
        val_refined_error = 0.0
        
        with torch.no_grad():
            for batch in val_loader:
                left_img = batch['left_image'].to(config['device'])
                flow = batch['flow'].to(config['device'])
                imu = batch['imu'].to(config['device'])
                vio_estimate = batch['vio_estimate'].to(config['device'])
                correction_gt = batch['correction'].to(config['device'])
                gt_position = batch['gt_position'].to(config['device'])
                
                correction_pred = model(left_img, flow, imu, vio_estimate)
                loss = l2_loss(correction_pred, correction_gt)
                val_loss += loss.item()
                
                refined_position = vio_estimate + correction_pred
                refined_error = torch.norm(refined_position - gt_position, dim=1).mean()
                val_refined_error += refined_error.item()
        
        val_loss /= len(val_loader)
        val_refined_error /= len(val_loader)
        
        scheduler.step(val_loss)
        
        # Print progress
        epoch_time = time.time() - epoch_start
        print(f"Epoch [{epoch+1}/{config['num_epochs']}] ({epoch_time:.1f}s)")
        print(f"  Train: L2 Loss={train_loss:.4f}m")
        print(f"  Val:   L2 Loss={val_loss:.4f}m, Refined Error={val_refined_error:.4f}m")
        
        # Save best model
        if val_loss < best_val_loss:
            best_val_loss = val_loss
            checkpoint_path = config['output_dir'] / 'multi_domain_refinement_checkpoint.pt'
            torch.save({
                'epoch': epoch,
                'model_state_dict': model.state_dict(),
                'optimizer_state_dict': optimizer.state_dict(),
                'val_loss': val_loss,
                'val_refined_error': val_refined_error,
                'config': config,
                'datasets': datasets,
            }, checkpoint_path)
            print(f"  ✓ Saved checkpoint (L2_loss={val_loss:.4f}m)")
    
    print(f"\n✓ Training complete! Best val loss: {best_val_loss:.4f}m")
    return model


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--datasets', nargs='+', default=['tumvi'],
                       help='Datasets to train on (tumvi, euroc, custom)')
    parser.add_argument('--epochs', type=int, default=20)
    parser.add_argument('--batch-size', type=int, default=8)
    parser.add_argument('--lr', type=float, default=1e-3)
    parser.add_argument('--no-curriculum', action='store_true',
                       help='Disable curriculum learning')
    
    args = parser.parse_args()
    
    config = BASE_CONFIG.copy()
    config['num_epochs'] = args.epochs
    config['batch_size'] = args.batch_size
    config['learning_rate'] = args.lr
    if args.no_curriculum:
        config['curriculum_learning'] = False
    
    model = train_multi_domain(args.datasets, config)
    
    # Export to ONNX
    print("\nExporting to ONNX...")
    model.eval()
    dummy_left = torch.randn(1, 1, 480, 640).to(config['device'])
    dummy_flow = torch.randn(1, 96).to(config['device'])
    dummy_imu = torch.randn(1, 15).to(config['device'])
    dummy_vio = torch.randn(1, 3).to(config['device'])
    
    onnx_path = config['output_dir'] / 'multi_domain_refinement.onnx'
    torch.onnx.export(
        model,
        (dummy_left, dummy_flow, dummy_imu, dummy_vio),
        onnx_path,
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
    print(f"✓ ONNX model saved to {onnx_path}")


if __name__ == "__main__":
    main()
