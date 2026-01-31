#!/usr/bin/env python3
"""
Quick test utility for real datasets before full training
Usage: python3 test_dataset.py /path/to/metadata.json
"""

import json
import sys
from pathlib import Path


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 test_dataset.py <metadata_file>")
        sys.exit(1)

    metadata_file = Path(sys.argv[1])
    image_dir = metadata_file.parent / 'images'
    depth_dir = metadata_file.parent / 'depth'

    print(f"Testing dataset: {metadata_file}")
    print(f"  Images: {image_dir}")
    print(f"  Depth: {depth_dir}\n")

    # Step 1: Basic metadata check
    print("[1] Loading metadata...")
    try:
        with open(metadata_file, 'r') as f:
            frames = json.load(f)
        print(f"  ✓ Loaded {len(frames)} frames")
    except Exception as e:
        print(f"  ✗ Error: {e}")
        return 1

    # Step 2: Validate with training validator
    print("\n[2] Validating metadata...")
    try:
        from train_student_network import validate_metadata
        validate_metadata(metadata_file, depth_dir)
        print("  ✓ All fields valid")
    except Exception as e:
        print(f"  ✗ Error: {e}")
        return 1

    # Step 3: Load dataset
    print("\n[3] Loading dataset...")
    try:
        from train_student_network import TeacherDataset
        dataset = TeacherDataset(metadata_file, image_dir, depth_dir)
        print(f"  ✓ Dataset ready ({len(dataset)} frames)")
    except Exception as e:
        print(f"  ✗ Error: {e}")
        return 1

    # Step 4: Test sample loading
    print("\n[4] Testing sample loading...")
    try:
        for i in range(min(3, len(dataset))):
            sample = dataset[i]
            print(f"  ✓ Frame {i}:")
            print(f"      images: {sample['images'].shape}")
            print(f"      pose: {sample['pose'].shape}")
            print(f"      match_quality: {sample['match_quality'].shape}")
    except Exception as e:
        print(f"  ✗ Error on frame {i}: {e}")
        return 1

    # Step 5: Test batching
    print("\n[5] Testing batching...")
    try:
        from torch.utils.data import DataLoader
        loader = DataLoader(dataset, batch_size=4, num_workers=0)
        batch = next(iter(loader))
        print(f"  ✓ Batch created (size={batch['images'].size(0)})")
        print(f"      images: {batch['images'].shape}")
        print(f"      imu: {batch['imu'].shape}")
        print(f"      depth_map: {batch['depth_map'].shape}")
        print(f"      match_quality: {batch['match_quality'].shape}")
    except Exception as e:
        print(f"  ✗ Error: {e}")
        import traceback
        traceback.print_exc()
        return 1

    print("\n✓ All tests passed! Dataset is ready for training.\n")
    print("To train the network, run:")
    print(f"  python3 train_student_network.py {metadata_file}")
    print(f"    --image-dir {image_dir}")
    print(f"    --depth-dir {depth_dir}")
    print("    --output-dir ./model_outputs")
    print("    --num-epochs 50")

    return 0

if __name__ == '__main__':
    sys.exit(main())
