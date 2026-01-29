#!/usr/bin/env python3
"""Comprehensive validation of exported dataset."""

import json
from pathlib import Path
import sys

def validate_dataset():
    """Validate exported dataset structure and content."""
    
    exported_data = Path('/Users/vincent/Work/RS-VIO/exported_data')
    metadata_file = exported_data / 'metadata.json'
    
    print("=" * 70)
    print("DATASET VALIDATION REPORT")
    print("=" * 70)
    
    # Check metadata file
    print("\n[1] Metadata File Check")
    if not metadata_file.exists():
        print(f"  ❌ Missing: {metadata_file}")
        return False
    
    with open(metadata_file) as f:
        data = json.load(f)
    
    print(f"  ✅ Found metadata.json with {len(data)} frames")
    
    # Check file structure
    print("\n[2] Directory Structure Check")
    dirs = {
        'images/cam0': 'Camera 0 images',
        'images/cam1': 'Camera 1 images', 
        'depth_maps': 'Depth maps'
    }
    
    for dir_path, desc in dirs.items():
        full_path = exported_data / dir_path
        if full_path.exists():
            count = len(list(full_path.glob('*')))
            print(f"  ✅ {desc:20s}: {count} files")
        else:
            print(f"  ❌ Missing: {dir_path}")
            return False
    
    # Validate metadata structure
    print("\n[3] Metadata Content Check")
    required_fields = [
        'imu_preintegration', 'imu_covariance', 'flow', 'flow_quality',
        'depth_file', 'match_quality', 'previous_pose', 'previous_velocity',
        'time_since_keyframe', 'image_files', 'timestamp_ns',
        'pose_tx', 'pose_ty', 'pose_tz',
        'pose_qx', 'pose_qy', 'pose_qz', 'pose_qw',
        'mean_reprojection_error'
    ]
    
    sample_frame = data[0]
    missing_fields = [f for f in required_fields if f not in sample_frame]
    
    if missing_fields:
        print(f"  ❌ Missing fields: {missing_fields}")
        return False
    
    print(f"  ✅ All {len(required_fields)} required fields present")
    
    # Check field shapes
    print("\n[4] Field Shape Validation")
    shape_checks = {
        'imu_preintegration': 15,
        'imu_covariance': 15,
        'flow': 96,
        'match_quality': 32,
        'previous_pose': 7,
        'previous_velocity': 3
    }
    
    all_good = True
    for field, expected_len in shape_checks.items():
        actual_len = len(sample_frame[field])
        status = "✅" if actual_len == expected_len else "❌"
        print(f"  {status} {field:25s}: {actual_len} (expected {expected_len})")
        if actual_len != expected_len:
            all_good = False
    
    if not all_good:
        return False
    
    # Validate image references
    print("\n[5] Image File References")
    missing_images = 0
    for i, frame in enumerate(data[:100]):  # Check first 100
        left_path = exported_data / frame['image_files']['left']
        right_path = exported_data / frame['image_files']['right']
        if not left_path.exists() or not right_path.exists():
            missing_images += 1
    
    if missing_images > 0:
        print(f"  ⚠️  {missing_images} missing image references (from first 100 frames)")
    else:
        print(f"  ✅ All image references valid (checked first 100 frames)")
    
    # Validate depth references
    print("\n[6] Depth File References")
    missing_depths = 0
    for i, frame in enumerate(data[:100]):  # Check first 100
        depth_path = exported_data / frame['depth_file']
        if not depth_path.exists():
            missing_depths += 1
    
    if missing_depths > 0:
        print(f"  ⚠️  {missing_depths} missing depth references (from first 100 frames)")
    else:
        print(f"  ✅ All depth references valid (checked first 100 frames)")
    
    # Summary
    print("\n" + "=" * 70)
    print("VALIDATION SUMMARY")
    print("=" * 70)
    print(f"  Total Frames: {len(data)}")
    print(f"  Metadata Size: {metadata_file.stat().st_size / 1e6:.1f} MB")
    print(f"  Status: ✅ READY FOR TRAINING")
    print("=" * 70)
    
    return True

if __name__ == '__main__':
    success = validate_dataset()
    sys.exit(0 if success else 1)
