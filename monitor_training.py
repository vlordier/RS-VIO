#!/usr/bin/env python3
"""Monitor training progress by checking checkpoint file."""

import time
import json
from pathlib import Path

results_dir = Path("/Users/vincent/Work/RS-VIO/results")

while True:
    print("\n" + "="*80)
    print("TRAINING MONITOR")
    print("="*80)
    
    # Check for checkpoint
    checkpoint_path = results_dir / "student_checkpoint.pt"
    if checkpoint_path.exists():
        size_mb = checkpoint_path.stat().st_size / 1024 / 1024
        print(f"✓ Checkpoint exists: {size_mb:.1f} MB")
    else:
        print("⏳ Checkpoint not yet saved (training in progress)")
    
    # Check for training summary
    summary_path = results_dir / "training_summary.json"
    if summary_path.exists():
        with open(summary_path) as f:
            summary = json.load(f)
            print(f"✓ Training complete!")
            print(f"  Best validation error: {summary.get('best_validation_error', 'N/A'):.6f}m")
            print(f"  Training time: {summary.get('total_training_time_seconds', 0)/60:.1f} minutes")
            print(f"  Device: {summary.get('device', 'N/A')}")
        break
    
    # Check log file
    log_path = results_dir / "training_log.txt"
    if log_path.exists() and log_path.stat().st_size > 100:
        with open(log_path) as f:
            lines = f.readlines()
            if lines:
                last_line = lines[-1].strip()
                if last_line:
                    print(f"Last update: {last_line}")
    
    print("\nWaiting for training to complete...")
    time.sleep(30)
