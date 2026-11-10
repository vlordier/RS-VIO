#!/usr/bin/env python3
"""
Complete VO vs VIO Benchmark Suite for RS-VIO.

This script:
1. Runs VO (no IMU) and VIO (with IMU) on specified datasets
2. Evaluates accuracy (ATE/RPE) for both modes
3. Generates comprehensive comparison report
4. Saves results to JSON for tracking over time

Usage:
    python3 scripts/benchmark_vo_vio.py \
        --config config/tum_vi.yaml \
        --datasets /path/to/dataset1 /path/to/dataset2 \
        --output results/benchmark_report.json
"""

import argparse
import json
import os
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

# Add scripts directory to path for imports
sys.path.insert(0, os.path.dirname(__file__))
from eval_ate_rpe import read_csv_gt, read_est, compute_ate, compute_rpe, umeyama_align, associate

import numpy as np


def run_vio(binary: str, dataset_path: str, config_path: str, use_imu: bool, timeout_sec: int = 600):
    """Run VIO/VO on dataset."""
    cmd = [binary, config_path, dataset_path]
    if not use_imu:
        cmd.insert(1, '--no-imu')
    
    print(f"\n{'='*70}")
    print(f"Running {'VIO' if use_imu else 'VO'} on {os.path.basename(dataset_path)}")
    print(f"{'='*70}")
    print(f"Command: {' '.join(cmd)}")
    
    env = os.environ.copy()
    env['RUST_LOG'] = 'warn'
    
    start_time = time.time()
    try:
        result = subprocess.run(
            cmd,
            env=env,
            capture_output=True,
            text=True,
            timeout=timeout_sec,
            cwd=os.path.dirname(os.path.dirname(__file__))
        )
        elapsed = time.time() - start_time
        
        if result.returncode != 0:
            print(f"ERROR: {' '.join(cmd)}")
            print(result.stderr[-1000:] if result.stderr else "No error output")
            return None
        
        # Find output trajectory
        default_output = os.path.join(dataset_path, 'estimated_trajectory.txt')
        if os.path.exists(default_output):
            return {'elapsed_s': elapsed, 'output_traj': default_output}
        
        print(f"Warning: Trajectory file not found at {default_output}")
        return None
        
    except subprocess.TimeoutExpired:
        print(f"ERROR: Timeout after {timeout_sec}s")
        return None


def evaluate_accuracy(traj_path: str, gt_path: str):
    """Evaluate trajectory accuracy against ground truth."""
    if not os.path.exists(traj_path):
        return {'error': f'Trajectory not found: {traj_path}'}
    
    if not os.path.exists(gt_path):
        return {'error': f'Ground truth not found: {gt_path}'}
    
    try:
        gt_ts, gt_pos, gt_quat = read_csv_gt(gt_path)
        est_ts, est_pos, est_quat = read_est(traj_path)
        
        if len(est_pos) < 10:
            return {'error': 'Too few estimated poses'}
        
        # Associate timestamps
        pairs = associate(est_ts, gt_ts)
        if len(pairs) < 10:
            return {'error': f'Too few associations: {len(pairs)}'}
        
        # Umeyama alignment
        pos_est_aligned = [est_pos[i] for i, _ in pairs]
        pos_gt = [gt_pos[j] for _, j in pairs]
        T = umeyama_align(pos_est_aligned, pos_gt)
        pos_est_aligned = [T @ np.array([*p, 1.0])[:3] for p in pos_est_aligned]
        
        # Compute ATE
        ate_errors = [np.linalg.norm(p1 - p2) for p1, p2 in zip(pos_est_aligned, pos_gt)]
        ate_rmse = float(np.sqrt(np.mean(np.square(ate_errors))))
        ate_median = float(np.median(ate_errors))
        ate_std = float(np.std(ate_errors))
        ate_max = float(np.max(ate_errors))
        
        # Compute path length
        est_path_length = sum(np.linalg.norm(np.array(pos_est_aligned[i+1]) - np.array(pos_est_aligned[i])) 
                             for i in range(len(pos_est_aligned)-1))
        gt_path_length = sum(np.linalg.norm(np.array(pos_gt[i+1]) - np.array(pos_gt[i])) 
                            for i in range(len(pos_gt)-1))
        
        # Sim(3) scale
        sim3_scale = est_path_length / gt_path_length if gt_path_length > 0 else 0
        
        ate_stats = {
            'rmse': ate_rmse,
            'median': ate_median,
            'std': ate_std,
            'max': ate_max,
            'n_pairs': len(pairs),
            'est_path_length': est_path_length,
            'gt_path_length': gt_path_length,
            'sim3_scale': sim3_scale,
        }
        
        # Compute RPE at multiple intervals
        rpe_results = {}
        for delta in [0.1, 1.0, 5.0, 10.0]:
            trans_err, rot_err = compute_rpe(est_ts, est_pos, est_quat, gt_pos, gt_quat, pairs, delta)
            if len(trans_err) > 0 and len(rot_err) > 0:
                rpe_results[f'delta_{delta}s'] = {
                    'trans_rmse': float(np.sqrt(np.mean(np.square(trans_err)))),
                    'rot_rmse': float(np.sqrt(np.mean(np.square(rot_err)))),
                    'n_pairs': len(trans_err)
                }
        
        return {
            'ate': ate_stats,
            'rpe': rpe_results,
            'n_poses': len(est_pos),
            'duration_s': (est_ts[-1] - est_ts[0]) / 1e9 if len(est_ts) > 1 else 0
        }
        
    except Exception as e:
        return {'error': str(e)}


def benchmark_dataset(binary: str, dataset_path: str, config_path: str, gt_path: str, timeout_sec: int = 600):
    """Run complete VO vs VIO benchmark on single dataset."""
    print(f"\n{'='*80}")
    print(f"BENCHMARKING: {os.path.basename(dataset_path)}")
    print(f"{'='*80}")
    
    results = {
        'dataset': os.path.basename(dataset_path),
        'dataset_path': dataset_path,
        'gt_path': gt_path,
    }
    
    # Run VO (no IMU)
    vo_result = run_vio(binary, dataset_path, config_path, use_imu=False, timeout_sec=timeout_sec)
    if vo_result:
        vo_accuracy = evaluate_accuracy(vo_result['output_traj'], gt_path)
        results['vo'] = {
            'runtime_s': vo_result['elapsed_s'],
            'accuracy': vo_accuracy,
        }
        print(f"\nVO Results:")
        if 'error' not in vo_accuracy:
            print(f"  ATE RMSE: {vo_accuracy['ate']['rmse']:.3f} m")
            print(f"  Sim(3) scale: {vo_accuracy['ate']['sim3_scale']:.3f}")
            if 'delta_1.0s' in vo_accuracy['rpe']:
                rpe = vo_accuracy['rpe']['delta_1.0s']
                print(f"  RPE (1s): trans={rpe['trans_rmse']:.3f} m/s, rot={rpe['rot_rmse']:.3f} deg/s")
        else:
            print(f"  ERROR: {vo_accuracy['error']}")
    
    # Run VIO (with IMU)
    vio_result = run_vio(binary, dataset_path, config_path, use_imu=True, timeout_sec=timeout_sec)
    if vio_result:
        vio_accuracy = evaluate_accuracy(vio_result['output_traj'], gt_path)
        results['vio'] = {
            'runtime_s': vio_result['elapsed_s'],
            'accuracy': vio_accuracy,
        }
        print(f"\nVIO Results:")
        if 'error' not in vio_accuracy:
            print(f"  ATE RMSE: {vio_accuracy['ate']['rmse']:.3f} m")
            print(f"  Sim(3) scale: {vio_accuracy['ate']['sim3_scale']:.3f}")
            if 'delta_1.0s' in vio_accuracy['rpe']:
                rpe = vio_accuracy['rpe']['delta_1.0s']
                print(f"  RPE (1s): trans={rpe['trans_rmse']:.3f} m/s, rot={rpe['rot_rmse']:.3f} deg/s")
        else:
            print(f"  ERROR: {vio_accuracy['error']}")
    
    # Compute improvement
    if 'vo' in results and 'vio' in results:
        vo_ate = results['vo']['accuracy'].get('ate', {})
        vio_ate = results['vio']['accuracy'].get('ate', {})
        
        if 'rmse' in vo_ate and 'rmse' in vio_ate and vo_ate['rmse'] > 0:
            improvement = (vo_ate['rmse'] - vio_ate['rmse']) / vo_ate['rmse'] * 100
            results['improvement'] = {
                'vo_ate_rmse': vo_ate['rmse'],
                'vio_ate_rmse': vio_ate['rmse'],
                'improvement_pct': improvement,
                'improvement_factor': vo_ate['rmse'] / vio_ate['rmse'] if vio_ate['rmse'] > 0 else float('inf'),
            }
    
    return results


def main():
    parser = argparse.ArgumentParser(description='VO vs VIO Benchmark Suite')
    parser.add_argument('--binary', default='target/release/run_tum',
                       help='Path to VIO binary (default: target/release/run_tum)')
    parser.add_argument('--config', default='config/tum_vi.yaml',
                       help='Config file path')
    parser.add_argument('--datasets', nargs='+', required=True,
                       help='Dataset paths to benchmark')
    parser.add_argument('--gt-suffix', default='mav0/mocap0/data.csv',
                       help='Ground truth path suffix (default: mav0/mocap0/data.csv)')
    parser.add_argument('--output', default='benchmark_results.json',
                       help='Output JSON file')
    parser.add_argument('--timeout', type=int, default=600,
                       help='Timeout per dataset in seconds')
    parser.add_argument('--skip-run', action='store_true',
                       help='Skip running, just evaluate existing trajectories')
    args = parser.parse_args()
    
    # Resolve paths
    binary = os.path.abspath(args.binary)
    config = os.path.abspath(args.config)
    
    if not os.path.exists(binary):
        print(f"ERROR: Binary not found: {binary}")
        print("Run: cargo build --release")
        sys.exit(1)
    
    if not os.path.exists(config):
        print(f"ERROR: Config not found: {config}")
        sys.exit(1)
    
    # Run benchmarks
    all_results = {
        'timestamp': datetime.now().isoformat(),
        'binary': binary,
        'config': config,
        'datasets': [],
    }
    
    for dataset_path in args.datasets:
        dataset_path = os.path.abspath(dataset_path)
        if not os.path.exists(dataset_path):
            print(f"WARNING: Dataset not found: {dataset_path}")
            continue
        
        gt_path = os.path.join(dataset_path, args.gt_suffix)
        if not os.path.exists(gt_path):
            print(f"WARNING: Ground truth not found: {gt_path}")
            continue
        
        results = benchmark_dataset(binary, dataset_path, config, gt_path, args.timeout)
        all_results['datasets'].append(results)
    
    # Summary
    print(f"\n{'='*80}")
    print("BENCHMARK SUMMARY")
    print(f"{'='*80}")
    
    for ds in all_results['datasets']:
        print(f"\n{ds['dataset']}:")
        if 'improvement' in ds:
            imp = ds['improvement']
            print(f"  VO ATE:  {imp['vo_ate_rmse']:.3f} m")
            print(f"  VIO ATE: {imp['vio_ate_rmse']:.3f} m")
            print(f"  Improvement: {imp['improvement_pct']:.1f}% ({imp['improvement_factor']:.1f}× better)")
        else:
            print(f"  Incomplete results")
    
    # Save JSON
    with open(args.output, 'w') as f:
        json.dump(all_results, f, indent=2)
    print(f"\nResults saved to: {args.output}")
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
