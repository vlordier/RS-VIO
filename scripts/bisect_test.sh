#!/usr/bin/env bash
# Git bisect test script: returns 0 (good) if room1 ATE < threshold, 1 (bad) otherwise.
set -e

DATASET="/Users/vincent/data/tum-vi/dataset-room1_512_16"
GT="$DATASET/mav0/mocap0/data.csv"
THRESHOLD="1.0"  # ATE threshold in meters

# Ensure data.csv exists for room1
for cam in cam0 cam1; do
  dir="$DATASET/mav0/$cam"
  if [ ! -f "$dir/data.csv" ]; then
    echo "#timestamp [ns],filename" > "$dir/data.csv"
    ls "$dir/data/" | sort | sed 's/\.png$//' | while read ts; do echo "$ts,$ts.png"; done >> "$dir/data.csv"
  fi
done

# Apply pose_log patch if not present
if ! grep -q "pose_log" src/estimator/estimator.rs 2>/dev/null; then
  echo "Adding pose_log..."
  # Add field to struct
  sed -i '' '/trajectory: Vec<Matrix4x4>,/a\
    pub pose_log: Vec<(i64, Matrix4x4)>,
' src/estimator/estimator.rs
  # Add init to constructor (find "trajectory: Vec::new()," and add after)
  sed -i '' '/trajectory: Vec::new(),/a\
            pose_log: Vec::new(),
' src/estimator/estimator.rs
  # Add push after view_motion_tracking_results
  sed -i '' '/self\.view_motion_tracking_results/i\
                    self.pose_log.push((timestamp_ns, T_W_B));
' src/estimator/estimator.rs
fi

# Build
if ! cargo build --release 2>&1 | tail -1 | grep -q "Finished"; then
  echo "BUILD FAILED — skip"
  exit 125  # git bisect skip
fi

# Run
./target/release/run_tum config/tum_vi.yaml "$DATASET" 2>&1 > /dev/null

# Evaluate
ATE=$(python3 scripts/eval_ate_rpe.py --est "$DATASET/estimated_trajectory.txt" --gt "$GT" 2>&1 | grep "ATE RMSE:" | head -1 | awk '{print $NF}')
echo "ATE RMSE = $ATE (threshold = $THRESHOLD)"

if [ -z "$ATE" ]; then
  echo "No ATE computed — skip"
  exit 125
fi

# Compare (use python for float comparison)
RESULT=$(python3 -c "print('good' if float('$ATE') < float('$THRESHOLD') else 'bad')")
echo "Result: $RESULT"

# Reset any patches
git checkout -- src/estimator/estimator.rs 2>/dev/null || true

if [ "$RESULT" = "good" ]; then
  exit 0
else
  exit 1
fi
