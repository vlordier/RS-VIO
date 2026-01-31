#!/usr/bin/env python3
"""
Week 6 Master Control Script
Orchestrates Tuesday-Friday execution with automated monitoring and checkpoint management.

This script:
1. Verifies data is prepared (Tuesday)
2. Monitors training completion (Wednesday)
3. Runs benchmarking (Thursday)
4. Executes integration (Friday)
5. Generates comprehensive report

Usage:
    python3 execute_week6.py --phase [verify|train|benchmark|integrate|all]
"""

import argparse
import json
import subprocess
import sys
from datetime import datetime
from pathlib import Path

# ============================================================================
# Configuration
# ============================================================================

PATHS = {
    "root": Path("/Users/vincent/Work/RS-VIO"),
    "exported_data": Path("/Users/vincent/Work/RS-VIO/exported_data"),
    "results": Path("/Users/vincent/Work/RS-VIO/results"),
    "metadata": Path("/Users/vincent/Work/RS-VIO/exported_data/metadata.json"),
    "checkpoint": Path("/Users/vincent/Work/RS-VIO/results/student_checkpoint.pt"),
    "training_summary": Path("/Users/vincent/Work/RS-VIO/results/training_summary.json"),
    "benchmark_results": Path("/Users/vincent/Work/RS-VIO/results/benchmark_results.json"),
    "integration_report": Path("/Users/vincent/Work/RS-VIO/results/vio_integration_report.json"),
}

# ============================================================================
# Phase 1: Verify (Tuesday - Completed)
# ============================================================================

def verify_data():
    """Verify Tuesday's data preparation."""
    print("\n" + "="*80)
    print("PHASE 1: DATA VERIFICATION (Tuesday)")
    print("="*80)

    checks = {
        "metadata_exists": PATHS["metadata"].exists(),
        "metadata_readable": False,
        "metadata_frames": 0,
        "images_cam0": 0,
        "images_cam1": 0,
        "depth_maps": 0,
    }

    # Check metadata
    if checks["metadata_exists"]:
        try:
            with open(PATHS["metadata"]) as f:
                metadata = json.load(f)
                checks["metadata_readable"] = True
                checks["metadata_frames"] = len(metadata)

                if metadata:
                    frame = metadata[0]
                    checks["metadata_fields"] = len(frame)
        except Exception as e:
            print(f"ERROR reading metadata: {e}")
            return False

    # Check images
    cam0_dir = PATHS["exported_data"] / "images" / "cam0"
    cam1_dir = PATHS["exported_data"] / "images" / "cam1"
    depth_dir = PATHS["exported_data"] / "depth_maps"

    if cam0_dir.exists():
        checks["images_cam0"] = len(list(cam0_dir.glob("*.png")))
    if cam1_dir.exists():
        checks["images_cam1"] = len(list(cam1_dir.glob("*.png")))
    if depth_dir.exists():
        checks["depth_maps"] = len(list(depth_dir.glob("*.npy")))

    # Print results
    print(f"\n✓ Metadata: {checks['metadata_frames']} frames, {checks.get('metadata_fields', 'N/A')} fields")
    print(f"✓ Images cam0: {checks['images_cam0']} symlinks")
    print(f"✓ Images cam1: {checks['images_cam1']} symlinks")
    print(f"✓ Depth maps: {checks['depth_maps']} files")

    # Verify all present
    success = all([
        checks["metadata_exists"],
        checks["metadata_readable"],
        checks["metadata_frames"] == 2821,
        checks["images_cam0"] == 2821,
        checks["images_cam1"] == 2821,
        checks["depth_maps"] == 2821,
    ])

    if success:
        print("\n✅ DATA VERIFICATION COMPLETE")
        print("All 2,821 frames ready for training")
    else:
        print("\n❌ DATA VERIFICATION FAILED")
        print("Some files are missing or invalid")

    return success


# ============================================================================
# Phase 2: Training (Wednesday)
# ============================================================================

def run_training(wait=False):
    """Execute training script."""
    print("\n" + "="*80)
    print("PHASE 2: NETWORK TRAINING (Wednesday)")
    print("="*80)
    print("\nStarting training on 2,256 frames (80% of 2,821)")
    print("Expected time: 2-3 hours on Apple M4 Pro")
    print("Target: Validation error < 0.02m")

    script = PATHS["root"] / "train_student_network.py"
    log_file = PATHS["results"] / "training_log.txt"

    PATHS["results"].mkdir(exist_ok=True)

    # Start training
    with open(log_file, 'w') as log:
        process = subprocess.Popen(
            ["python3", str(script)],
            stdout=log,
            stderr=subprocess.STDOUT,
            cwd=str(PATHS["root"])
        )

    print(f"\n✓ Training started (PID: {process.pid})")
    print(f"  Log: {log_file}")

    if not wait:
        print("\n(Training running in background. Check results/ for checkpoint when complete)")
        return True

    # Wait for completion
    print("\nWaiting for training to complete...")
    process.wait()

    # Check for checkpoint
    if PATHS["checkpoint"].exists():
        size_mb = PATHS["checkpoint"].stat().st_size / 1024 / 1024
        print("\n✅ TRAINING COMPLETE")
        print(f"Checkpoint saved: {size_mb:.1f} MB")
        return True
    else:
        print("\n❌ TRAINING FAILED")
        print(f"Checkpoint not found at {PATHS['checkpoint']}")
        return False


# ============================================================================
# Phase 3: Benchmarking (Thursday)
# ============================================================================

def run_benchmarking():
    """Execute benchmarking script."""
    print("\n" + "="*80)
    print("PHASE 3: CPU BENCHMARKING (Thursday)")
    print("="*80)
    print("\nProfiling feature tracking on 100 frames")
    print("Measuring: Image pyramid + gradient computation time")
    print("Decision: GPU justified if > 60% of frame time")

    script = PATHS["root"] / "benchmark_cpu_pipeline.py"

    # Run benchmarking
    result = subprocess.run(
        ["python3", str(script)],
        cwd=str(PATHS["root"]),
        capture_output=True,
        text=True
    )

    if result.returncode == 0:
        print("\n✅ BENCHMARKING COMPLETE")
        print("GPU acceleration JUSTIFIED")
        return True
    elif result.returncode == 1:
        print("\n✅ BENCHMARKING COMPLETE")
        print("CPU sufficient, GPU NOT justified")
        return True
    else:
        print("\n❌ BENCHMARKING FAILED")
        print(f"Error: {result.stderr}")
        return False


# ============================================================================
# Phase 4: Integration (Friday)
# ============================================================================

def run_integration():
    """Execute VIO integration."""
    print("\n" + "="*80)
    print("PHASE 4: VIO INTEGRATION (Friday)")
    print("="*80)
    print("\nIntegrating trained checkpoint into VIO pipeline")
    print("Testing on 200 frames from test sequence")
    print("Measuring: Real-time performance (target: >30 FPS)")

    script = PATHS["root"] / "integrate_vio_friday.py"

    if not PATHS["checkpoint"].exists():
        print("\n❌ INTEGRATION FAILED")
        print(f"Checkpoint not found at {PATHS['checkpoint']}")
        print("Ensure training completed successfully on Wednesday")
        return False

    # Run integration
    result = subprocess.run(
        ["python3", str(script)],
        cwd=str(PATHS["root"]),
        capture_output=True,
        text=True
    )

    if result.returncode == 0 or PATHS["integration_report"].exists():
        print("\n✅ INTEGRATION COMPLETE")

        # Load and display results
        try:
            with open(PATHS["integration_report"]) as f:
                report = json.load(f)
                print(f"Average FPS: {report.get('avg_fps', 'N/A'):.1f}")
                errors = report.get('position_errors', [])
                if errors:
                    import numpy as np
                    errors = np.array(errors)
                    print(f"Mean position error: {np.mean(errors):.6f}m")
        except Exception as e:
            print(f"(Report generated, details: {e})")

        return True
    else:
        print("\n❌ INTEGRATION FAILED")
        print(f"Error: {result.stderr}")
        return False


# ============================================================================
# Generate Final Report
# ============================================================================

def generate_final_report():
    """Generate comprehensive Week 6 final report."""
    print("\n" + "="*80)
    print("WEEK 6 FINAL REPORT")
    print("="*80)

    report = {
        "timestamp": datetime.now().isoformat(),
        "phases": {
            "data_verification": PATHS["metadata"].exists(),
            "training": PATHS["checkpoint"].exists(),
            "benchmarking": PATHS["benchmark_results"].exists(),
            "integration": PATHS["integration_report"].exists(),
        },
        "results": {}
    }

    # Training results
    if PATHS["training_summary"].exists():
        try:
            with open(PATHS["training_summary"]) as f:
                summary = json.load(f)
                report["results"]["training"] = summary
        except Exception:
            pass

    # Benchmarking results
    if PATHS["benchmark_results"].exists():
        try:
            with open(PATHS["benchmark_results"]) as f:
                bench = json.load(f)
                report["results"]["benchmarking"] = {
                    "gpu_justified": bench.get("summary", {}).get("gpu_justified"),
                    "gpu_percentage": bench.get("summary", {}).get("gpu_percentage"),
                }
        except Exception:
            pass

    # Integration results
    if PATHS["integration_report"].exists():
        try:
            with open(PATHS["integration_report"]) as f:
                integration = json.load(f)
                report["results"]["integration"] = {
                    "avg_fps": integration.get("avg_fps"),
                    "real_time": integration.get("avg_fps", 0) >= 30,
                }
        except Exception:
            pass

    # Print summary
    print("\nPhase Completion:")
    for phase, complete in report["phases"].items():
        status = "✅" if complete else "⏳"
        print(f"  {status} {phase}")

    # Save full report
    report_path = PATHS["results"] / "WEEK6_FINAL_REPORT.json"
    with open(report_path, 'w') as f:
        json.dump(report, f, indent=2)

    print(f"\nFinal report saved to {report_path}")

    # Check success
    all_complete = all(report["phases"].values())
    if all_complete:
        print("\n🎉 WEEK 6 SUCCESSFULLY COMPLETED")
    else:
        print("\n📋 Week 6 in progress (some phases pending)")

    return report


# ============================================================================
# Main
# ============================================================================

def main():
    parser = argparse.ArgumentParser(description="Week 6 execution controller")
    parser.add_argument(
        "--phase",
        choices=["verify", "train", "benchmark", "integrate", "report", "all"],
        default="all",
        help="Which phase to execute"
    )
    parser.add_argument("--wait", action="store_true", help="Wait for training to complete")

    args = parser.parse_args()

    success = True

    if args.phase in ["verify", "all"]:
        if not verify_data():
            success = False

    if args.phase in ["train", "all"] and success:
        if not run_training(wait=args.wait):
            if args.phase == "train":
                success = False

    if args.phase in ["benchmark", "all"]:
        if PATHS["checkpoint"].exists() or args.phase != "all":
            if not run_benchmarking():
                if args.phase == "benchmark":
                    success = False

    if args.phase in ["integrate", "all"]:
        if PATHS["checkpoint"].exists() or args.phase != "all":
            if not run_integration():
                if args.phase == "integrate":
                    success = False

    if args.phase in ["report", "all"]:
        generate_final_report()

    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
