#!/bin/bash
# Quick benchmark runner for corridor1

set -e

cd /Users/vincent/Work/RS-VIO

DATASET="/Users/vincent/data/tum-vi/dataset-corridor1_512_16"
BINARY="target/release/run_tum"
CONFIG="config/tum_vi.yaml"

echo "========================================================================"
echo "RS-VIO Benchmark: Corridor1 (VO vs VIO)"
echo "========================================================================"
echo ""

# Run VO (no IMU)
echo "Running VO (no IMU)..."
rm -f $DATASET/estimated_trajectory.txt
$BINARY --no-imu $CONFIG $DATASET &
VO_PID=$!

# Monitor progress
echo "VO PID: $VO_PID"
while kill -0 $VO_PID 2>/dev/null; do
    sleep 30
    echo "  VO still running... (PID: $VO_PID)"
done
wait $VO_PID
VO_STATUS=$?

if [ $VO_STATUS -eq 0 ] && [ -f $DATASET/estimated_trajectory.txt ]; then
    echo "✓ VO completed successfully"
    mv $DATASET/estimated_trajectory.txt $DATASET/estimated_trajectory_vo.txt
else
    echo "✗ VO failed or timed out"
fi

echo ""

# Run VIO (with IMU)
echo "Running VIO (with IMU)..."
rm -f $DATASET/estimated_trajectory.txt
$BINARY $CONFIG $DATASET &
VIO_PID=$!

# Monitor progress
echo "VIO PID: $VIO_PID"
while kill -0 $VIO_PID 2>/dev/null; do
    sleep 30
    echo "  VIO still running... (PID: $VIO_PID)"
done
wait $VIO_PID
VIO_STATUS=$?

if [ $VIO_STATUS -eq 0 ] && [ -f $DATASET/estimated_trajectory.txt ]; then
    echo "✓ VIO completed successfully"
    mv $DATASET/estimated_trajectory.txt $DATASET/estimated_trajectory_vio.txt
else
    echo "✗ VIO failed or timed out"
fi

echo ""
echo "========================================================================"
echo "Evaluation"
echo "========================================================================"

# Evaluate VO
if [ -f $DATASET/estimated_trajectory_vo.txt ]; then
    echo ""
    echo "VO Results:"
    python3 scripts/eval_ate_rpe.py \
        --est $DATASET/estimated_trajectory_vo.txt \
        --gt $DATASET/mav0/mocap0/data.csv 2>&1 | grep -E "(Sim|Path|ATE|RMSE)" | head -10
fi

# Evaluate VIO
if [ -f $DATASET/estimated_trajectory_vio.txt ]; then
    echo ""
    echo "VIO Results:"
    python3 scripts/eval_ate_rpe.py \
        --est $DATASET/estimated_trajectory_vio.txt \
        --gt $DATASET/mav0/mocap0/data.csv 2>&1 | grep -E "(Sim|Path|ATE|RMSE)" | head -10
fi

echo ""
echo "========================================================================"
echo "Benchmark Complete"
echo "========================================================================"
