#!/usr/bin/env python3
import torch
from train_refinement_network import RefinementNetwork

# Load trained model
checkpoint = torch.load('results/refinement_checkpoint.pt', map_location='cpu', weights_only=False)
model = RefinementNetwork()
model.load_state_dict(checkpoint['model_state_dict'])
model.eval()

print('✓ Loaded checkpoint from epoch', checkpoint['epoch'])
print(f'  Validation loss: {checkpoint["val_loss"]:.4f}m')
print(f'  Refined error: {checkpoint["val_refined_error"]:.4f}m')

# Export to ONNX
print('\nExporting to ONNX...')
dummy_left = torch.randn(1, 1, 480, 640)
dummy_flow = torch.randn(1, 96)
dummy_imu = torch.randn(1, 15)
dummy_vio = torch.randn(1, 3)

torch.onnx.export(
    model,
    (dummy_left, dummy_flow, dummy_imu, dummy_vio),
    'results/refinement_model.onnx',
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

print('✓ ONNX model saved to results/refinement_model.onnx')

# Check file size
import os
size_mb = os.path.getsize('results/refinement_model.onnx') / (1024 * 1024)
print(f'  Model size: {size_mb:.2f} MB')
