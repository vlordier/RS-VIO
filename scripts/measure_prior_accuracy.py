#!/usr/bin/env python3
"""
Measure actual accuracy improvements from motion priors.

This script:
1. Runs VIO on a dataset with priors DISABLED
2. Runs VIO on the same dataset with priors ENABLED
3. Compares ATE/RPE metrics
4. Reports actual accuracy improvement
"""

import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

# Import eval_ate_rpe functions
sys.path.insert(0, os.path.dirname(__file__))
from eval_ate_rpe import read_csv_gt, read_est, compute_ate, compute_rpe, umeyama_align, associate


def run_vio(dataset_path: str, config_path: str, use_priors: bool, output_label: str):
    """Run VIO on dataset with or without priors."""
    binary = os.path.join(os.path.dirname(__file__), 'target', 'release', 'run_tum')
    if not os.path.exists(binary):
        # Try alternate path
        binary = os.path.join(os.path.dirname(__file__), '..', 'target', 'release', 'run_tum')
    
    if not os.path.exists(binary):
        print(f"Error: Binary not found at {binary}")
        print("Run: cargo build --release")
        sys.exit(1)
    
    output_traj = os.path.join(dataset_path, f"estimated_trajectory_{output_label}.txt")
    
    # Build command
    cmd = [binary, config_path, dataset_path]
    
    # Set environment to disable viewer (headless)
    env = os.environ.copy()
    env['RUST_LOG'] = 'warn'
    
    print(f"\n{'='*70}")
    print(f"Running VIO {'WITH' if use_priors else 'WITHOUT'} motion priors")
    print(f"{'='*70}")
    print(f"Command: {' '.join(cmd)}")
    print(f"Output: {output_traj}")
    
    start_time = time.time()
    
    try:
        result = subprocess.run(
            cmd,
            env=env,
            capture_output=True,
            text=True,
            timeout=600,  # 10 min timeout
            cwd=os.path.dirname(__file__)
        )
        elapsed = time.elapsed() if hasattr(time, 'elapsed') else time.time() - start_time
        
        if result.returncode != 0:
            print(f"Error running VIO:")
            print(result.stderr[-2000:] if result.stderr else "No stderr output")
            return None
        
        # Rename output trajectory
        default_output = os.path.join(dataset_path, 'estimated_trajectory.txt')
        if os.path.exists(default_output):
            os.rename(default_output, output_traj)
        
        print(f"Completed in {elapsed:.1f}s")
        return {'elapsed_s': elapsed, 'output_traj': output_traj}
        
    except subprocess.TimeoutExpired:
        print(f"Timeout after 10 minutes")
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
        
        ate_stats = {
            'rmse': ate_rmse,
            'median': ate_median,
            'std': ate_std,
            'max': ate_max,
            'n_pairs': len(pairs)
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


def main():
    parser = argparse.ArgumentParser(description='Measure motion prior accuracy improvements')
    parser.add_argument('dataset_path', help='Path to dataset (e.g., /path/to/tum-vi/dataset-corridor1_512_16)')
    parser.add_argument('--config', default='config/tum_vi.yaml', help='Config file path')
    parser.add_argument('--skip-run', action='store_true', help='Skip running VIO, just evaluate existing trajectories')
    parser.add_argument('--json', help='Output JSON results file')
    args = parser.parse_args()
    
    dataset_path = os.path.abspath(args.dataset_path)
    config_path = os.path.abspath(args.config)
    
    if not os.path.exists(dataset_path):
        print(f"Error: Dataset not found: {dataset_path}")
        sys.exit(1)
    
    gt_path = os.path.join(dataset_path, 'mav0', 'mocap0', 'data.csv')
    
    results = {
        'dataset': dataset_path,
        'config': config_path,
        'gt_path': gt_path,
    }
    
    if not args.skip_run:
        # Run without priors (baseline)
        baseline_result = run_vio(dataset_path, config_path, use_priors=False, output_label='baseline')
        if baseline_result:
            results['baseline'] = baseline_result
        
        # Run with priors
        prior_result = run_vio(dataset_path, config_path, use_priors=True, output_label='with_priors')
        if prior_result:
            results['with_priors'] = prior_result
    else:
        # Use existing trajectories
        baseline_traj = os.path.join(dataset_path, 'estimated_trajectory_baseline.txt')
        prior_traj = os.path.join(dataset_path, 'estimated_trajectory_with_priors.txt')
        
        if os.path.exists(baseline_traj):
            results['baseline'] = {'output_traj': baseline_traj}
        if os.path.exists(prior_traj):
            results['with_priors'] = {'output_traj': prior_traj}
    
    # Evaluate accuracy
    print(f"\n{'='*70}")
    print("EVALUATING ACCURACY")
    print(f"{'='*70}")
    
    if 'baseline' in results and 'output_traj' in results['baseline']:
        print("\n--- Baseline (no priors) ---")
        baseline_acc = evaluate_accuracy(results['baseline']['output_traj'], gt_path)
        if 'error' not in baseline_acc:
            print(f"ATE RMSE: {baseline_acc['ate']['rmse']:.3f} m")
            print(f"ATE median: {baseline_acc['ate']['median']:.3f} m")
            if 'delta_1.0s' in baseline_acc['rpe']:
                rpe = baseline_acc['rpe']['delta_1.0s']
                print(f"RPE (1s): trans={rpe['trans_rmse']:.3f} m/s, rot={rpe['rot_rmse']:.3f} deg/s")
        else:
            print(f"Error: {baseline_acc['error']}")
        results['baseline_accuracy'] = baseline_acc
    
    if 'with_priors' in results and 'output_traj' in results['with_priors']:
        print("\n--- With Motion Priors ---")
        prior_acc = evaluate_accuracy(results['with_priors']['output_traj'], gt_path)
        if 'error' not in prior_acc:
            print(f"ATE RMSE: {prior_acc['ate']['rmse']:.3f} m")
            print(f"ATE median: {prior_acc['ate']['median']:.3f} m")
            if 'delta_1.0s' in prior_acc['rpe']:
                rpe = prior_acc['rpe']['delta_1.0s']
                print(f"RPE (1s): trans={rpe['trans_rmse']:.3f} m/s, rot={rpe['rot_rmse']:.3f} deg/s")
        else:
            print(f"Error: {prior_acc['error']}")
        results['with_priors_accuracy'] = prior_acc
    
    # Compute improvement
    if 'baseline_accuracy' in results and 'with_priors_accuracy' in results:
        if 'error' not in results['baseline_accuracy'] and 'error' not in results['with_priors_accuracy']:
            baseline_ate = results['baseline_accuracy']['ate']['rmse']
            prior_ate = results['with_priors_accuracy']['ate']['rmse']
            
            if baseline_ate > 0:
                improvement = (baseline_ate - prior_ate) / baseline_ate * 100
                results['improvement'] = {
                    'ate_baseline': baseline_ate,
                    'ate_with_priors': prior_ate,
                    'improvement_pct': improvement,
                    'improvement_factor': baseline_ate / prior_ate if prior_ate > 0 else float('inf')
                }
                
                print(f"\n{'='*70}")
                print("IMPROVEMENT SUMMARY")
                print(f"{'='*70}")
                print(f"ATE RMSE: {baseline_ate:.3f}m → {prior_ate:.3f}m")
                print(f"Improvement: {improvement:.1f}% ({baseline_ate/prior_ate:.1f}× better)")
    
    # Save JSON
    if args.json:
        # Convert numpy types for JSON
        def convert(obj):
            import numpy as np
            if isinstance(obj, np.floating):
                return float(obj)
            if isinstance(obj, np.integer):
                return int(obj)
            if isinstance(obj, np.ndarray):
                return obj.tolist()
            return obj
        
        results_json = {k: convert(v) if not isinstance(v, dict) else 
                       {k2: convert(v2) for k2, v2 in v.items()} 
                       for k, v in results.items()}
        
        with open(args.json, 'w') as f:
            json.dump(results_json, f, indent=2)
        print(f"\nJSON results saved to: {args.json}")
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
