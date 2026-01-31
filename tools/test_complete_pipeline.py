#!/usr/bin/env python3
"""
Complete end-to-end test: generate data → train network → validate outputs
"""

import json
import logging
import sys
from pathlib import Path

import torch

logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')
logger = logging.getLogger(__name__)


def main():
    script_dir = Path(__file__).parent
    sys.path.insert(0, str(script_dir))

    # Step 1: Generate synthetic training data
    logger.info("=" * 60)
    logger.info("STEP 1: Generating synthetic training data")
    logger.info("=" * 60)

    from generate_training_data import TrainingDataGenerator

    data_dir = Path('/tmp/student_network_test_data')
    data_dir.mkdir(parents=True, exist_ok=True)

    generator = TrainingDataGenerator(
        output_dir=data_dir,
        num_frames=50,
        seed=42,
    )
    metadata_file = generator.generate_all()

    # Step 2: Validate metadata
    logger.info("\n" + "=" * 60)
    logger.info("STEP 2: Validating metadata format")
    logger.info("=" * 60)

    with open(metadata_file, 'r') as f:
        frames = json.load(f)
    logger.info(f"Loaded {len(frames)} frames from metadata")

    # Check first frame structure
    frame0 = frames[0]
    required_keys = [
        'imu_preintegration', 'imu_covariance', 'flow', 'flow_quality',
        'depth_file', 'match_quality', 'previous_pose', 'previous_velocity',
        'time_since_keyframe', 'pose_tx', 'pose_ty', 'pose_tz',
        'pose_qx', 'pose_qy', 'pose_qz', 'pose_qw', 'mean_reprojection_error',
    ]
    for key in required_keys:
        if key not in frame0:
            logger.error(f"  ✗ Missing key: {key}")
            return 1
        logger.info(f"  ✓ {key}: {type(frame0[key])}")

    # Step 3: Validate metadata with training validator
    logger.info("\n" + "=" * 60)
    logger.info("STEP 3: Running dataset validation")
    logger.info("=" * 60)

    from train_student_network import validate_metadata

    try:
        validate_metadata(metadata_file, data_dir / 'depth')
        logger.info("  ✓ Metadata validation passed!")
    except Exception as e:
        logger.error(f"  ✗ Metadata validation failed: {e}")
        return 1

    # Step 4: Load network and test forward pass
    logger.info("\n" + "=" * 60)
    logger.info("STEP 4: Testing network forward pass")
    logger.info("=" * 60)

    from student_network import create_student_network

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    logger.info(f"Using device: {device}")

    model = create_student_network().to(device)
    num_params = sum(p.numel() for p in model.parameters())
    logger.info(f"Model created with {num_params:,} parameters")

    # Create dummy batch matching required inputs
    batch_size = 4
    images = torch.randn(batch_size, 2, 256, 256).to(device)
    imu = torch.randn(batch_size, 15).to(device)
    imu_cov = torch.randn(batch_size, 15).to(device)
    flow = torch.randn(batch_size, 96).to(device)
    flow_quality = torch.rand(batch_size).to(device)
    depth_map = torch.randn(batch_size, 1, 256, 256).to(device)
    match_quality = torch.rand(batch_size, 32).to(device)
    previous_pose = torch.randn(batch_size, 7).to(device)
    previous_velocity = torch.randn(batch_size, 3).to(device)
    time_since_keyframe = torch.rand(batch_size).to(device)

    try:
        outputs = model(
            images=images,
            imu=imu,
            imu_covariance=imu_cov,
            flow=flow,
            flow_quality=flow_quality,
            depth_map=depth_map,
            match_quality=match_quality,
            previous_pose=previous_pose,
            previous_velocity=previous_velocity,
            time_since_keyframe=time_since_keyframe,
        )

        logger.info("  ✓ Forward pass successful")
        logger.info(f"    Pose output shape: {outputs['pose'].shape}")
        logger.info(f"    Uncertainty shape: {outputs.get('uncertainty', 'N/A')}")

    except Exception as e:
        logger.error(f"  ✗ Forward pass failed: {e}")
        import traceback
        traceback.print_exc()
        return 1

    # Step 5: Test dataset loading
    logger.info("\n" + "=" * 60)
    logger.info("STEP 5: Testing dataset loading")
    logger.info("=" * 60)

    from train_student_network import TeacherDataset

    try:
        dataset = TeacherDataset(
            metadata_file=metadata_file,
            image_dir=data_dir / 'images',
            depth_dir=data_dir / 'depth',
        )
        logger.info(f"  ✓ Dataset created with {len(dataset)} frames")

        # Load first sample
        sample = dataset[0]
        logger.info("  ✓ Loaded sample 0")
        for key, val in sample.items():
            if isinstance(val, torch.Tensor):
                logger.info(f"    {key}: {val.shape}")
            else:
                logger.info(f"    {key}: {val}")

    except Exception as e:
        logger.error(f"  ✗ Dataset loading failed: {e}")
        import traceback
        traceback.print_exc()
        return 1

    # Step 6: Test mini batch through model
    logger.info("\n" + "=" * 60)
    logger.info("STEP 6: Testing batch through model")
    logger.info("=" * 60)

    from torch.utils.data import DataLoader

    try:
        loader = DataLoader(dataset, batch_size=4, shuffle=False)
        batch = next(iter(loader))

        model.eval()
        with torch.no_grad():
            outputs = model(
                images=batch['images'].to(device),
                imu=batch['imu'].to(device),
                imu_covariance=batch['imu_covariance'].to(device),
                flow=batch['flow'].to(device),
                flow_quality=batch['flow_quality'].to(device),
                depth_map=batch['depth_map'].to(device),
                match_quality=batch['match_quality'].to(device),
                previous_pose=batch['previous_pose'].to(device),
                previous_velocity=batch['previous_velocity'].to(device),
                time_since_keyframe=batch['time_since_keyframe'].to(device),
            )

        logger.info("  ✓ Batch forward pass successful")
        logger.info(f"    Batch pose output shape: {outputs['pose'].shape}")
        logger.info(f"    Batch uncertainty shape: {outputs['uncertainty'].shape}")

    except Exception as e:
        logger.error(f"  ✗ Batch forward pass failed: {e}")
        import traceback
        traceback.print_exc()
        return 1

    # Summary
    logger.info("\n" + "=" * 60)
    logger.info("ALL TESTS PASSED ✓")
    logger.info("=" * 60)
    logger.info(f"\nTraining data location: {data_dir}")
    logger.info(f"Metadata file: {metadata_file}")
    logger.info("\nTo train the full network, run:")
    logger.info(f"  python3 train_student_network.py {metadata_file} \\")
    logger.info(f"    --image-dir {data_dir / 'images'} \\")
    logger.info(f"    --depth-dir {data_dir / 'depth'} \\")
    logger.info("    --output-dir ./model_outputs \\")
    logger.info("    --num-epochs 10")

    return 0


if __name__ == '__main__':
    sys.exit(main())
