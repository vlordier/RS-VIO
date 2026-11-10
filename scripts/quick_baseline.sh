#!/bin/bash
# Quick baseline evaluation on all 4 TUM-VI sequences

set -e

cd /Users/vincent/Work/RS-VIO

BINARY="target/release/run_tum"
CONFIG="config/tum_vi.yaml"
DATA_BASE="/Users/vincent/data/tum-vi"

echo "================================================================================"
echo "RS-VIO Baseline Evaluation (commit 70bafec - velocity harness only)"
echo "================================================================================"
echo ""

SEQUENCES=("slides1" "corridor1" "room1" "corridor2")

for seq in "${SEQUENCES[@]}"; do
    DATASET="$DATA_BASE/dataset-${seq}_512_16"
    GT="$DATASET/mav0/mocap0/data.csv"

    if [ ! -d "$DATASET" ]; then
        echo "⚠ $seq: Dataset not found"
        continue
    fi

    echo "$seq:"

    # Run VIO
    rm -f "$DATASET/estimated_trajectory.txt" 2>/dev/null
    timeout 600 "$BINARY" "$CONFIG" "$DATASET" > /tmp/${seq}.log 2>&1 || {
        echo "  ✗ VIO failed"
        continue
    }

    if [ ! -f "$DATASET/estimated_trajectory.txt" ]; then
        echo "  ✗ No trajectory generated"
        continue
    fi

    # Evaluate
    python3 scripts/eval_ate_rpe.py \
        --est "$DATASET/estimated_trajectory.txt" \
        --gt "$GT" 2>&1 | {

        read_metric() {
            grep "$1" | head -1 | sed 's/.*: //' | sed 's/ .*//' | sed "s/'//g"
        }

        local_output=$(cat)
        echo "$local_output" | grep -E "Sim\(3\) scale|RMSE   =" | head -2 | \
            awk 'NR==1 {gsub(/.*scale: /,""); gsub(/ .*/,""); scale=$0}
                 NR==2 {gsub(/.*=/,""); gsub(/ .*/,""); ate=$0}
                 END {printf "  ATE=%.3fm, scale=%s\n", ate, scale}'
    }

done

echo ""
echo "================================================================================"
