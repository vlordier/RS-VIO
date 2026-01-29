#!/usr/bin/env python3
"""
VIO Integration script for Friday - integrates trained student network into full pipeline.

This script:
1. Loads trained checkpoint from Wednesday training
2. Integrates into VIO pipeline
3. Validates on test sequence
4. Measures real-time performance (FPS)
5. Compares with ground truth
6. Generates integration report
"""

import json
import torch
import numpy as np
from pathlib import Path
import time
from datetime import datetime

# ============================================================================
# Configuration
# ============================================================================

CONFIG = {
    "checkpoint_path": "/Users/vincent/Work/RS-VIO/results/student_checkpoint.pt",
    "metadata_path": "/Users/vincent/Work/RS-VIO/exported_data/metadata.json",
    "image_root": "/Users/vincent/Work/RS-VIO/exported_data/images",
    "num_test_frames": 200,
    "target_fps": 30.0,
    "output_dir": Path("/Users/vincent/Work/RS-VIO/results"),
}

CONFIG["output_dir"].mkdir(exist_ok=True)


# ============================================================================
# Student Network (Copied from training)
# ============================================================================

class StudentNetwork(torch.nn.Module):
    """Same architecture as training for checkpoint loading."""
    
    def __init__(self):
        super().__init__()
        
        self.img_encoder = torch.nn.Sequential(
            torch.nn.Conv2d(1, 32, kernel_size=3, stride=2, padding=1),
            torch.nn.ReLU(inplace=True),
            torch.nn.Conv2d(32, 64, kernel_size=3, stride=2, padding=1),
            torch.nn.ReLU(inplace=True),
            torch.nn.Conv2d(64, 128, kernel_size=3, stride=2, padding=1),
            torch.nn.ReLU(inplace=True),
            torch.nn.AdaptiveAvgPool2d((1, 1)),
        )
        
        self.imu_processor = torch.nn.Sequential(
            torch.nn.Linear(15, 64),
            torch.nn.ReLU(inplace=True),
            torch.nn.Linear(64, 128),
            torch.nn.ReLU(inplace=True),
        )
        
        self.flow_processor = torch.nn.Sequential(
            torch.nn.Linear(96, 128),
            torch.nn.ReLU(inplace=True),
            torch.nn.Linear(128, 128),
            torch.nn.ReLU(inplace=True),
        )
        
        self.fusion_mlp = torch.nn.Sequential(
            torch.nn.Linear(128 + 128 + 128, 256),
            torch.nn.ReLU(inplace=True),
            torch.nn.Dropout(0.2),
            torch.nn.Linear(256, 128),
            torch.nn.ReLU(inplace=True),
            torch.nn.Dropout(0.2),
            torch.nn.Linear(128, 3),
        )
        
    def forward(self, left_img, imu_preint, flow):
        img_feat = self.img_encoder(left_img).view(left_img.size(0), -1)
        imu_feat = self.imu_processor(imu_preint)
        flow_feat = self.flow_processor(flow)
        fused = torch.cat([img_feat, imu_feat, flow_feat], dim=1)
        position = self.fusion_mlp(fused)
        return position


# ============================================================================
# Integration
# ============================================================================

def load_image(path):
    """Load grayscale image, normalize to [0, 1]."""
    try:
        from PIL import Image
        img = Image.open(path).convert('L')
        return np.array(img, dtype=np.float32) / 255.0
    except Exception as e:
        print(f"Error loading {path}: {e}")
        return np.zeros((480, 640), dtype=np.float32)


def integrate_vio():
    """Main VIO integration test."""
    
    print("\n" + "="*80)
    print("VIO INTEGRATION TEST - WEEK 6 FRIDAY")
    print("="*80)
    
    # Load checkpoint
    print("\nLoading trained checkpoint...")
    if not Path(CONFIG["checkpoint_path"]).exists():
        print(f"ERROR: Checkpoint not found at {CONFIG['checkpoint_path']}")
        print("Ensure training completed successfully on Wednesday")
        return None
    
    checkpoint = torch.load(CONFIG["checkpoint_path"], map_location="cpu", weights_only=False)
    model = StudentNetwork()
    model.load_state_dict(checkpoint["model_state_dict"])
    model.eval()
    
    print(f"✓ Checkpoint loaded")
    print(f"  Validation error from training: {checkpoint.get('val_error', 'N/A'):.6f}m")
    
    # Load metadata
    print("\nLoading metadata...")
    with open(CONFIG["metadata_path"]) as f:
        metadata = json.load(f)
    
    num_frames = min(CONFIG["num_test_frames"], len(metadata))
    print(f"✓ Metadata loaded: {len(metadata)} total, using {num_frames} for test")
    
    # Run integration test
    print("\n" + "="*80)
    print("VIO INTEGRATION TEST")
    print("="*80)
    
    results = {
        "frames_tested": 0,
        "successful": 0,
        "failed": 0,
        "total_time_s": 0.0,
        "avg_fps": 0.0,
        "position_errors": [],
        "frame_results": [],
    }
    
    test_start = time.perf_counter()
    
    for i in range(num_frames):
        frame = metadata[i]
        frame_start = time.perf_counter()
        
        try:
            # Load image
            left_path = Path(CONFIG["image_root"]) / "cam0" / (frame["image_files"]["left"].split("/")[-1])
            left_img = load_image(left_path)
            
            # Prepare inputs
            left_tensor = torch.from_numpy(left_img).float().unsqueeze(0).unsqueeze(0)
            imu_tensor = torch.tensor(frame["imu_preintegration"], dtype=torch.float32).unsqueeze(0)
            flow_tensor = torch.tensor(frame["flow"], dtype=torch.float32).unsqueeze(0)
            
            # Predict position
            with torch.no_grad():
                pred_pos = model(left_tensor, imu_tensor, flow_tensor).numpy()[0]
            
            # Ground truth
            gt_pos = np.array([frame["pose_tx"], frame["pose_ty"], frame["pose_tz"]], dtype=np.float32)
            
            # Compute error
            error = np.linalg.norm(pred_pos - gt_pos)
            
            frame_time = (time.perf_counter() - frame_start) * 1000  # ms
            
            results["frame_results"].append({
                "frame_idx": i,
                "error_m": float(error),
                "time_ms": float(frame_time),
            })
            
            results["position_errors"].append(float(error))
            results["successful"] += 1
            
            if (i + 1) % 50 == 0:
                print(f"  Frame {i+1}/{num_frames}: Error {error:.6f}m, Time {frame_time:.2f}ms")
        
        except Exception as e:
            print(f"  Frame {i}: Failed - {e}")
            results["failed"] += 1
        
        results["frames_tested"] = i + 1
    
    total_time = time.perf_counter() - test_start
    results["total_time_s"] = float(total_time)
    results["avg_fps"] = float(results["frames_tested"] / total_time)
    
    # Statistics
    print("\n" + "="*80)
    print("RESULTS")
    print("="*80)
    print(f"\nFrames tested: {results['frames_tested']}")
    print(f"Successful: {results['successful']}")
    print(f"Failed: {results['failed']}")
    
    if results["position_errors"]:
        errors = np.array(results["position_errors"])
        print(f"\nPosition Error Statistics:")
        print(f"  Mean: {np.mean(errors):.6f}m")
        print(f"  Std Dev: {np.std(errors):.6f}m")
        print(f"  Min: {np.min(errors):.6f}m")
        print(f"  Max: {np.max(errors):.6f}m")
        print(f"  Median: {np.median(errors):.6f}m")
    
    print(f"\nTiming:")
    print(f"  Total time: {total_time:.1f}s")
    print(f"  Average FPS: {results['avg_fps']:.1f}")
    print(f"  Target FPS: {CONFIG['target_fps']}")
    
    if results["avg_fps"] >= CONFIG["target_fps"]:
        print(f"✓ Real-time performance ACHIEVED")
    else:
        print(f"⚠ Real-time performance NOT achieved ({results['avg_fps']:.1f} < {CONFIG['target_fps']} FPS)")
    
    # Save results
    results["timestamp"] = datetime.now().isoformat()
    results["checkpoint_val_error"] = float(checkpoint.get("val_error", -1))
    results["success"] = results["avg_fps"] >= CONFIG["target_fps"]
    
    report_path = CONFIG["output_dir"] / "vio_integration_report.json"
    with open(report_path, 'w') as f:
        json.dump(results, f, indent=2)
    
    print(f"\nReport saved to {report_path}")
    
    return results


if __name__ == "__main__":
    integrate_vio()
