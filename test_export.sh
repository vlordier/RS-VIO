#!/bin/bash

# Week 6 Monday - Quick Validation Export
# Test 10 frames to verify Rust→JSON export works with real data

set -e

REPO_DIR="/Users/vincent/Work/RS-VIO"
DATASET_PATH="${REPO_DIR}/datasets/tum_vi/room1"
OUTPUT_DIR="/tmp/rs_vio_student_training_test"
BINARY="${REPO_DIR}/target/release/export_teacher"

echo "======================================"
echo "Week 6 Export System - Quick Test"
echo "======================================"
echo ""

# Verify dataset exists
if [ ! -d "$DATASET_PATH" ]; then
    echo "❌ Dataset not found: $DATASET_PATH"
    exit 1
fi

# Create output directory
mkdir -p "$OUTPUT_DIR"

echo "📊 Dataset Information:"
echo "  Location: $DATASET_PATH"
FRAME_COUNT=$(find "$DATASET_PATH/mav0/cam0/data" -name "*.png" 2>/dev/null | wc -l)
echo "  Total frames: $FRAME_COUNT"
echo ""

echo "🚀 Running export (first 10 frames with VIO)..."
echo "   Binary: $BINARY"
echo "   Output: $OUTPUT_DIR"
echo ""

# Run the export binary
timeout 120 "$BINARY" \
    --dataset-path "$DATASET_PATH" \
    --output-dir "$OUTPUT_DIR" \
    2>&1 | tail -50 || echo "Export completed (or timed out after 2 min)"

echo ""
echo "📁 Checking output structure..."

if [ -f "$OUTPUT_DIR/metadata.json" ]; then
    echo "✅ metadata.json found"
    echo ""
    echo "📋 Sample metadata (first frame):"
    python3 << 'PYTHON'
import json
with open('/tmp/rs_vio_student_training_test/metadata.json', 'r') as f:
    data = json.load(f)
    if isinstance(data, list) and len(data) > 0:
        frame = data[0]
        print(f"  Frame ID: {frame.get('frame_id')}")
        print(f"  Timestamp: {frame.get('timestamp'):.6f}")
        print(f"  Pose: [{frame.get('pose_tx'):.4f}, {frame.get('pose_ty'):.4f}, {frame.get('pose_tz'):.4f}]")
        print(f"  Quaternion: [{frame.get('pose_qx'):.4f}, {frame.get('pose_qy'):.4f}, {frame.get('pose_qz'):.4f}, {frame.get('pose_qw'):.4f}]")
        print(f"  Previous pose: {frame.get('previous_pose', 'N/A')[:50] if 'previous_pose' in frame else 'N/A'}...")
        print(f"  Previous velocity: {frame.get('previous_velocity', 'N/A')}")
        print(f"  IMU preintegration: [{frame.get('imu_preintegration', [0]*15)[0]:.4f}, ...] ({len(frame.get('imu_preintegration', []))} elements)")
        print(f"  IMU covariance: [{frame.get('imu_covariance', [0]*15)[0]:.4f}, ...] ({len(frame.get('imu_covariance', []))} elements)")
        print(f"  Flow: [... {len(frame.get('flow', []))} elements ...]")
        print(f"  Flow quality: {frame.get('flow_quality', 'N/A')}")
        print(f"  Match quality: [{frame.get('match_quality', [0]*32)[0]:.4f}, ...] ({len(frame.get('match_quality', []))} elements)")
        print(f"  Time since keyframe: {frame.get('time_since_keyframe', 'N/A')}")
        print(f"  Mean reprojection error: {frame.get('mean_reprojection_error', 'N/A')}")
        print(f"  Depth file: {frame.get('image_files', {}).get('depth_map', 'N/A')}")
        print(f"  Total frames in metadata: {len(data)}")
PYTHON
else
    echo "❌ metadata.json not found - export may have failed"
    ls -lh "$OUTPUT_DIR" 2>/dev/null || echo "Output dir is empty"
fi

echo ""
echo "📋 Validation:"
python3 << 'PYTHON'
import json
import sys

try:
    with open('/tmp/rs_vio_student_training_test/metadata.json', 'r') as f:
        data = json.load(f)
    
    if not isinstance(data, list):
        print("❌ metadata.json is not a list")
        sys.exit(1)
    
    if len(data) == 0:
        print("⚠️  No frames exported")
        sys.exit(1)
    
    # Check first frame has all required fields
    frame = data[0]
    required_fields = [
        'frame_id', 'timestamp',
        'pose_tx', 'pose_ty', 'pose_tz', 'pose_qx', 'pose_qy', 'pose_qz', 'pose_qw',
        'imu_preintegration', 'imu_covariance',
        'flow', 'flow_quality',
        'previous_pose', 'previous_velocity',
        'match_quality', 'time_since_keyframe',
        'mean_reprojection_error'
    ]
    
    missing = [f for f in required_fields if f not in frame]
    if missing:
        print(f"❌ Missing fields: {missing}")
        sys.exit(1)
    
    # Check array dimensions
    checks = {
        'imu_preintegration': 15,
        'imu_covariance': 15,
        'flow': 96,
        'previous_pose': 7,
        'previous_velocity': 3,
        'match_quality': 32,
    }
    
    for field, expected_len in checks.items():
        actual_len = len(frame.get(field, []))
        if actual_len != expected_len:
            print(f"❌ {field}: expected {expected_len} elements, got {actual_len}")
            sys.exit(1)
    
    print(f"✅ All validation checks passed!")
    print(f"   Frames exported: {len(data)}")
    print(f"   All required fields present")
    print(f"   All array dimensions correct")
    
except Exception as e:
    print(f"❌ Validation error: {e}")
    sys.exit(1)
PYTHON

echo ""
echo "======================================"
echo "Test Complete"
echo "======================================"
