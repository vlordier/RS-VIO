#!/usr/bin/env python3
import json
from pathlib import Path

import numpy as np
import torch

# Load metadata to verify
with open('exported_data/metadata.json', 'r') as f:
    metadata = json.load(f)
    print(f'Loaded metadata: {len(metadata)} frames')

# Check first frame
frame = metadata[0]
print(f'Frame 0 has {len(frame)} fields')
print(f'  imu_preint: {len(frame["imu_preintegration"])} values')
print(f'  flow: {len(frame["flow"])} values')

# Check depth map
depth_path = Path('exported_data/depth_maps/frame_000000.npy')
if depth_path.exists():
    depth = np.load(depth_path)
    print(f'Depth map shape: {depth.shape}')

print(f'PyTorch: {torch.__version__}')
print('Data ready for training')
