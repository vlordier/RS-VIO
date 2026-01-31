#!/usr/bin/env python3
"""
Benchmark script comparing VIO with and without NN refinement.

This script demonstrates proper integration:
1. Loads trained refinement network (ONNX)
2. Runs VIO baseline (simulated for now, will use real Rust VIO later)
3. Applies NN refinement to VIO estimates
4. Compares refined vs baseline trajectories
5. Generates performance metrics and plots

Usage:
    python3 benchmark_refinement.py --model results/refinement_model.onnx --max-frames 500
"""

import argparse
import json
import time
from pathlib import Path
from typing import Dict, Tuple

import matplotlib.pyplot as plt
import numpy as np

try:
    import onnxruntime as ort
    ONNX_AVAILABLE = True
except ImportError:
    print("Warning: onnxruntime not installed. Install with: pip install onnxruntime")
    ONNX_AVAILABLE = False


class RefinementBenchmark:
    """Benchmark VIO with NN refinement"""

    def __init__(self, metadata_path: str, onnx_model_path: str = None):
        self.metadata_path = Path(metadata_path)
        self.onnx_model_path = Path(onnx_model_path) if onnx_model_path else None

        # Load metadata
        with open(self.metadata_path, 'r') as f:
            self.metadata = json.load(f)

        print(f"Loaded {len(self.metadata)} frames from {self.metadata_path}")

        # Load ONNX model if provided
        self.onnx_session = None
        if self.onnx_model_path and ONNX_AVAILABLE:
            if self.onnx_model_path.exists():
                self.onnx_session = ort.InferenceSession(
                    str(self.onnx_model_path),
                    providers=['CPUExecutionProvider']
                )
                print(f"Loaded ONNX model from {self.onnx_model_path}")
                print(f"  Inputs: {[i.name for i in self.onnx_session.get_inputs()]}")
                print(f"  Outputs: {[o.name for o in self.onnx_session.get_outputs()]}")
            else:
                print(f"Warning: ONNX model not found at {self.onnx_model_path}")

    def simulate_vio_estimate(self, gt_position: np.ndarray, noise_std: float = 0.05) -> np.ndarray:
        """Simulate VIO estimate with added noise"""
        noise = np.random.randn(3) * noise_std
        return gt_position + noise

    def run_refinement(self,
                      image: np.ndarray,
                      flow: np.ndarray,
                      imu: np.ndarray,
                      vio_estimate: np.ndarray) -> Tuple[np.ndarray, float]:
        """Run NN refinement on VIO estimate

        Returns:
            correction: Position correction (3D vector)
            latency_ms: Inference latency in milliseconds
        """
        if self.onnx_session is None:
            return np.zeros(3), 0.0

        start = time.time()

        # Prepare inputs (batch size = 1)
        left_image = image.reshape(1, 1, 480, 640).astype(np.float32) / 255.0
        flow_input = flow.reshape(1, 96).astype(np.float32)
        imu_input = imu.reshape(1, 15).astype(np.float32)
        vio_input = vio_estimate.reshape(1, 3).astype(np.float32)

        # Run inference
        outputs = self.onnx_session.run(
            None,
            {
                'left_image': left_image,
                'flow': flow_input,
                'imu': imu_input,
                'vio_estimate': vio_input,
            }
        )

        correction = outputs[0][0]  # Extract first (and only) batch element
        latency_ms = (time.time() - start) * 1000

        return correction, latency_ms

    def benchmark(self, max_frames: int = None) -> Dict:
        """Run benchmark comparing baseline vs refined VIO

        Returns:
            Dictionary with metrics for both approaches
        """
        frames_to_process = self.metadata[:max_frames] if max_frames else self.metadata
        n_frames = len(frames_to_process)

        print(f"\nBenchmarking on {n_frames} frames...")
        print("="*70)

        # Results storage
        results = {
            'baseline': {
                'positions': [],
                'errors': [],
                'latencies_ms': [],
            },
            'refined': {
                'positions': [],
                'errors': [],
                'corrections': [],
                'latencies_ms': [],
            },
            'ground_truth': []
        }

        for i, frame in enumerate(frames_to_process):
            if i % 100 == 0:
                print(f"Processing frame {i}/{n_frames}...")

            # Ground truth position
            gt_position = np.array([
                frame['pose_tx'],
                frame['pose_ty'],
                frame['pose_tz']
            ])
            results['ground_truth'].append(gt_position)

            # Simulate VIO baseline estimate
            vio_estimate = self.simulate_vio_estimate(gt_position, noise_std=0.05)
            baseline_error = np.linalg.norm(vio_estimate - gt_position)
            results['baseline']['positions'].append(vio_estimate)
            results['baseline']['errors'].append(baseline_error)
            results['baseline']['latencies_ms'].append(0.003)  # Simulated VIO latency

            # Apply NN refinement if model is available
            if self.onnx_session:
                # Prepare inputs (dummy image for now)
                image = np.random.randint(0, 256, (480, 640), dtype=np.uint8)
                flow = np.array(frame['flow'], dtype=np.float32)
                imu = np.array(frame['imu_preintegration'], dtype=np.float32)

                # Get correction from NN
                correction, latency_ms = self.run_refinement(image, flow, imu, vio_estimate)
                refined_position = vio_estimate + correction
                refined_error = np.linalg.norm(refined_position - gt_position)

                results['refined']['positions'].append(refined_position)
                results['refined']['errors'].append(refined_error)
                results['refined']['corrections'].append(correction)
                results['refined']['latencies_ms'].append(latency_ms)
            else:
                # No refinement available
                results['refined']['positions'].append(vio_estimate)
                results['refined']['errors'].append(baseline_error)
                results['refined']['corrections'].append(np.zeros(3))
                results['refined']['latencies_ms'].append(0.0)

        # Compute statistics
        stats = self._compute_statistics(results)
        return stats

    def _compute_statistics(self, results: Dict) -> Dict:
        """Compute summary statistics"""
        baseline_errors = np.array(results['baseline']['errors'])
        refined_errors = np.array(results['refined']['errors'])
        corrections = np.array(results['refined']['corrections'])

        baseline_latencies = np.array(results['baseline']['latencies_ms'])
        refined_latencies = np.array(results['refined']['latencies_ms'])

        stats = {
            'baseline': {
                'mean_error_m': float(np.mean(baseline_errors)),
                'std_error_m': float(np.std(baseline_errors)),
                'median_error_m': float(np.median(baseline_errors)),
                'max_error_m': float(np.max(baseline_errors)),
                'mean_latency_ms': float(np.mean(baseline_latencies)),
            },
            'refined': {
                'mean_error_m': float(np.mean(refined_errors)),
                'std_error_m': float(np.std(refined_errors)),
                'median_error_m': float(np.median(refined_errors)),
                'max_error_m': float(np.max(refined_errors)),
                'mean_latency_ms': float(np.mean(refined_latencies)),
                'mean_correction_norm_m': float(np.mean(np.linalg.norm(corrections, axis=1))),
            },
            'improvement': {
                'error_reduction_percent': float((1 - np.mean(refined_errors) / np.mean(baseline_errors)) * 100),
                'latency_overhead_ms': float(np.mean(refined_latencies) - np.mean(baseline_latencies)),
            },
            'raw_data': results
        }

        return stats

    def print_results(self, stats: Dict):
        """Print benchmark results"""
        print("\n" + "="*70)
        print("BENCHMARK RESULTS")
        print("="*70)

        print("\nBaseline VIO:")
        print(f"  Mean Error:   {stats['baseline']['mean_error_m']:.4f} ± {stats['baseline']['std_error_m']:.4f} m")
        print(f"  Median Error: {stats['baseline']['median_error_m']:.4f} m")
        print(f"  Max Error:    {stats['baseline']['max_error_m']:.4f} m")
        print(f"  Latency:      {stats['baseline']['mean_latency_ms']:.3f} ms")

        print("\nRefined VIO (with NN):")
        print(f"  Mean Error:   {stats['refined']['mean_error_m']:.4f} ± {stats['refined']['std_error_m']:.4f} m")
        print(f"  Median Error: {stats['refined']['median_error_m']:.4f} m")
        print(f"  Max Error:    {stats['refined']['max_error_m']:.4f} m")
        print(f"  Latency:      {stats['refined']['mean_latency_ms']:.3f} ms")
        print(f"  Mean Correction: {stats['refined']['mean_correction_norm_m']:.4f} m")

        print("\nImprovement:")
        error_reduction = stats['improvement']['error_reduction_percent']
        if error_reduction > 0:
            print(f"  ✓ Error Reduction: {error_reduction:.1f}%")
        else:
            print(f"  ✗ Error Increase: {-error_reduction:.1f}%")
        print(f"  Latency Overhead: {stats['improvement']['latency_overhead_ms']:.3f} ms")

        print("\n" + "="*70)

    def plot_results(self, stats: Dict, output_path: str = "refinement_benchmark.png"):
        """Generate visualization plots"""
        results = stats['raw_data']

        fig, axes = plt.subplots(2, 2, figsize=(14, 10))

        # Plot 1: Error comparison
        ax = axes[0, 0]
        baseline_errors = results['baseline']['errors']
        refined_errors = results['refined']['errors']
        x = range(len(baseline_errors))
        ax.plot(x, baseline_errors, 'r-', alpha=0.6, label='Baseline VIO')
        ax.plot(x, refined_errors, 'g-', alpha=0.6, label='Refined VIO')
        ax.set_xlabel('Frame')
        ax.set_ylabel('Position Error (m)')
        ax.set_title('Position Error Over Time')
        ax.legend()
        ax.grid(True, alpha=0.3)

        # Plot 2: Error histogram
        ax = axes[0, 1]
        ax.hist(baseline_errors, bins=50, alpha=0.6, label='Baseline', color='red')
        ax.hist(refined_errors, bins=50, alpha=0.6, label='Refined', color='green')
        ax.set_xlabel('Position Error (m)')
        ax.set_ylabel('Frequency')
        ax.set_title('Error Distribution')
        ax.legend()
        ax.grid(True, alpha=0.3)

        # Plot 3: Correction magnitude
        ax = axes[1, 0]
        corrections = np.array(results['refined']['corrections'])
        correction_norms = np.linalg.norm(corrections, axis=1)
        ax.plot(correction_norms, 'b-', alpha=0.6)
        ax.axhline(y=np.mean(correction_norms), color='r', linestyle='--',
                   label=f'Mean: {np.mean(correction_norms):.4f}m')
        ax.set_xlabel('Frame')
        ax.set_ylabel('Correction Magnitude (m)')
        ax.set_title('NN Correction Magnitude')
        ax.legend()
        ax.grid(True, alpha=0.3)

        # Plot 4: Latency
        ax = axes[1, 1]
        refined_latencies = results['refined']['latencies_ms']
        ax.plot(refined_latencies, 'purple', alpha=0.6)
        ax.axhline(y=np.mean(refined_latencies), color='r', linestyle='--',
                   label=f'Mean: {np.mean(refined_latencies):.3f}ms')
        ax.set_xlabel('Frame')
        ax.set_ylabel('Latency (ms)')
        ax.set_title('NN Inference Latency')
        ax.legend()
        ax.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.savefig(output_path, dpi=150)
        print(f"\n✓ Saved plots to {output_path}")


def main():
    parser = argparse.ArgumentParser(description='Benchmark VIO refinement network')
    parser.add_argument('--metadata', type=str,
                        default='/Users/vincent/Work/RS-VIO/exported_data/metadata.json',
                        help='Path to metadata JSON file')
    parser.add_argument('--model', type=str,
                        default='/Users/vincent/Work/RS-VIO/results/refinement_model.onnx',
                        help='Path to ONNX refinement model')
    parser.add_argument('--max-frames', type=int, default=500,
                        help='Maximum number of frames to process')
    parser.add_argument('--output', type=str, default='refinement_benchmark.png',
                        help='Output plot filename')

    args = parser.parse_args()

    # Create benchmark
    benchmark = RefinementBenchmark(args.metadata, args.model)

    # Run benchmark
    stats = benchmark.benchmark(max_frames=args.max_frames)

    # Print results
    benchmark.print_results(stats)

    # Generate plots
    benchmark.plot_results(stats, args.output)

    # Save statistics to JSON
    output_json = Path(args.output).with_suffix('.json')
    # Remove raw_data before saving (too large)
    stats_copy = stats.copy()
    stats_copy.pop('raw_data', None)
    with open(output_json, 'w') as f:
        json.dump(stats_copy, f, indent=2)
    print(f"✓ Saved statistics to {output_json}")


if __name__ == "__main__":
    main()
