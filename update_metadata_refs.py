#!/usr/bin/env python3
"""Update metadata references to match actual image and depth files."""

import json
from pathlib import Path

# Load metadata
with open('/Users/vincent/Work/RS-VIO/exported_data/metadata.json') as f:
    data = json.load(f)

# Get list of camera images
cam0_files = sorted(Path('/Users/vincent/Work/RS-VIO/exported_data/images/cam0').glob('*.png'))
cam1_files = sorted(Path('/Users/vincent/Work/RS-VIO/exported_data/images/cam1').glob('*.png'))

print(f"Found {len(cam0_files)} cam0 files and {len(cam1_files)} cam1 files")

# Update image references
for i, frame in enumerate(data):
    if i < len(cam0_files) and i < len(cam1_files):
        left_filename = cam0_files[i].name
        right_filename = cam1_files[i].name
        frame['image_files'] = {
            'left': f'images/cam0/{left_filename}',
            'right': f'images/cam1/{right_filename}'
        }
        
        # Update depth file reference
        frame['depth_file'] = f'depth_maps/frame_{i:06d}.npy'

# Save updated metadata
with open('/Users/vincent/Work/RS-VIO/exported_data/metadata.json', 'w') as f:
    json.dump(data, f)

print(f"\n✅ Updated metadata for {len(data)} frames")
print(f"  Sample image_files: {data[0]['image_files']}")
print(f"  Sample depth_file: {data[0]['depth_file']}")

# Verify image files exist
missing = 0
for frame in data:
    left = Path('/Users/vincent/Work/RS-VIO/exported_data') / frame['image_files']['left']
    right = Path('/Users/vincent/Work/RS-VIO/exported_data') / frame['image_files']['right']
    if not left.exists() or not right.exists():
        missing += 1

if missing == 0:
    print(f"✅ All image file references verified")
else:
    print(f"⚠️  {missing} missing image files")
