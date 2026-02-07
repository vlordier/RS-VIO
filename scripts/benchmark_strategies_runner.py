#!/usr/bin/env python3

"""
Multi-Strategy Benchmarking Suite for RS-VIO

Orchestrates comprehensive testing of all stereo matching strategies
across all available datasets (EuRoC, TUM-VI, 4Seasons).

Usage:
    python3 scripts/benchmark_strategies_runner.py --all
    python3 scripts/benchmark_strategies_runner.py --euroc --fast
    python3 scripts/benchmark_strategies_runner.py --datasets euroc,tum
"""

import argparse
import subprocess
import json
import os
import sys
import time
from pathlib import Path
from datetime import datetime
from typing import List, Dict, Optional

from logging_utils import Colors, log_info, log_warn, log_error, log_header, log_section

# ============================================================================
# Configuration
# ============================================================================

SCRIPT_DIR = Path(__file__).parent.absolute()
PROJECT_ROOT = SCRIPT_DIR.parent
DATASET_BASE = Path(os.environ.get("DATASET_DIR", "/tmp/rs-vio-samples"))
RELEASE_DIR = PROJECT_ROOT / "target" / "release"
RESULTS_DIR = PROJECT_ROOT / "benchmark_results"
CONFIG_DIR = PROJECT_ROOT / "config"

# ============================================================================
# Dataset Detection
# ============================================================================

def find_available_datasets() -> Dict[str, Path]:
    """Discover available datasets on the system."""
    datasets = {}

    euroc_path = DATASET_BASE / "euroc" / "MH_01_easy"
    if euroc_path.exists():
        datasets["euroc"] = euroc_path
        log_info(f"Found EuRoC dataset: {euroc_path}")

    tum_path = DATASET_BASE / "tum_vi"
    if tum_path.exists():
        datasets["tum"] = tum_path
        log_info(f"Found TUM-VI dataset: {tum_path}")

    # Check for 4Seasons (any recording_*)
    seasons_base = DATASET_BASE / "4seasons"
    if seasons_base.exists():
        recordings = list(seasons_base.glob("recording_*"))
        if recordings:
            datasets["4seasons"] = recordings[0]
            log_info(f"Found 4Seasons dataset: {recordings[0]}")

    if not datasets:
        log_error("No datasets found!")
        log_error(f"Expected location: {DATASET_BASE}")
        log_error("Download with: just setup-datasets")
        sys.exit(1)

    return datasets

# ============================================================================
# Benchmarking
# ============================================================================

class BenchmarkRunner:
    def __init__(self, strategies: List[str], datasets: Dict[str, Path],
                 skip_build: bool = False):
        self.strategies = strategies
        self.datasets = datasets
        self.skip_build = skip_build
        self.results: List[Dict] = []
        self.timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.results_file = RESULTS_DIR / f"strategies_{self.timestamp}.json"
        self.csv_file = RESULTS_DIR / f"strategies_{self.timestamp}.csv"

    def build_release(self):
        """Compile release binaries."""
        if self.skip_build:
            log_info("Skipping build (--no-build specified)")
            return

        log_section("Building Release Binaries")
        try:
            result = subprocess.run(
                ["cargo", "build", "--release", "--quiet"],
                cwd=PROJECT_ROOT,
                capture_output=True,
                text=True,
                timeout=600  # 10 minutes
            )
            if result.returncode == 0:
                log_info("Release binaries built successfully")
            else:
                log_error("Build failed!")
                print(result.stderr)
                sys.exit(1)
        except subprocess.TimeoutExpired:
            log_error("Build timed out after 10 minutes")
            sys.exit(1)

    def run_benchmark(self, strategy: str, dataset_name: str,
                      dataset_path: Path) -> Optional[Dict]:
        """Run a single benchmark."""

        # Get config file
        config_map = {
            "euroc": CONFIG_DIR / "euroc_vio.yaml",
            "tum": CONFIG_DIR / "tum_vi.yaml",
            "4seasons": CONFIG_DIR / "4seasons.yaml"
        }
        config_file = config_map.get(dataset_name)
        if not config_file or not config_file.exists():
            log_warn(f"Config not found: {config_file}")
            return None

        # Get binary
        binary_map = {
            "euroc": RELEASE_DIR / "run_euroc",
            "tum": RELEASE_DIR / "run_tum",
            "4seasons": RELEASE_DIR / "run_4seasons"
        }
        binary = binary_map.get(dataset_name)
        if not binary or not binary.exists():
            log_warn(f"Binary not found: {binary}")
            return None

        # Run benchmark
        print(f"  {strategy:25s} on {dataset_name:10s} ... ", end="", flush=True)
        start_time = time.time()

        try:
            result = subprocess.run(
                [str(binary), str(config_file), str(dataset_path)],
                cwd=PROJECT_ROOT,
                capture_output=True,
                text=True,
                timeout=120
            )
            elapsed = time.time() - start_time

            # Parse output
            frames = 0
            avg_time = 0.0
            output = result.stdout + result.stderr

            # Extract metrics from output
            import re
            frames_match = re.search(r'Processed (\d+) frames', output)
            if frames_match:
                frames = int(frames_match.group(1))

            time_match = re.search(r'average ([0-9.]+)ms per frame', output)
            if time_match:
                avg_time = float(time_match.group(1))

            status = f"{Colors.GREEN}{elapsed:.1f}s{Colors.RESET}"
            print(f"{status} (frames: {frames}, avg: {avg_time:.2f}ms)")

            return {
                "timestamp": datetime.now().isoformat(),
                "strategy": strategy,
                "dataset": dataset_name,
                "dataset_path": str(dataset_path),
                "elapsed_seconds": elapsed,
                "frames_processed": frames,
                "avg_processing_ms": avg_time,
                "fps": frames / elapsed if elapsed > 0 else 0
            }

        except subprocess.TimeoutExpired:
            print(f"{Colors.RED}TIMEOUT{Colors.RESET}")
            log_warn(f"Benchmark timed out for {strategy} on {dataset_name}")
            return None
        except Exception as e:
            print(f"{Colors.RED}ERROR{Colors.RESET}")
            log_error(f"Benchmark failed: {e}")
            return None

    def run_all(self) -> bool:
        """Run all benchmarks."""
        log_header("RS-VIO Multi-Strategy Benchmarking")
        print(f"Timestamp: {self.timestamp}")
        print(f"Project: {PROJECT_ROOT}")
        print(f"Datasets: {DATASET_BASE}")
        print()

        # Build
        self.build_release()

        # Run benchmarks
        log_section("Running Benchmarks")
        total = len(self.strategies) * len(self.datasets)
        current = 0

        for strategy in self.strategies:
            for dataset_name, dataset_path in self.datasets.items():
                current += 1
                print()
                print(f"{Colors.YELLOW}[{current}/{total}]{Colors.RESET} "
                      f"Strategy: {Colors.CYAN}{strategy}{Colors.RESET}")

                result = self.run_benchmark(strategy, dataset_name, dataset_path)
                if result:
                    self.results.append(result)

        # Save results
        self.save_results()
        self.analyze_results()
        return True

    def save_results(self):
        """Save results to files."""
        RESULTS_DIR.mkdir(parents=True, exist_ok=True)

        # JSON format
        with open(self.results_file, 'w') as f:
            json.dump(self.results, f, indent=2)
        log_info(f"Results saved to: {self.results_file}")

        # CSV format
        with open(self.csv_file, 'w') as f:
            f.write("timestamp,strategy,dataset,elapsed_seconds,frames_processed,avg_processing_ms,fps\n")
            for result in self.results:
                f.write(
                    f"{result['timestamp']},"
                    f"{result['strategy']},"
                    f"{result['dataset']},"
                    f"{result['elapsed_seconds']:.2f},"
                    f"{result['frames_processed']},"
                    f"{result['avg_processing_ms']:.2f},"
                    f"{result['fps']:.2f}\n"
                )
        log_info(f"CSV saved to: {self.csv_file}")

    def analyze_results(self):
        """Display analysis of results."""
        log_header("Benchmark Results Summary")

        if not self.results:
            log_error("No results to analyze")
            return

        # Group by dataset
        by_dataset = {}
        for result in self.results:
            dataset = result['dataset']
            if dataset not in by_dataset:
                by_dataset[dataset] = []
            by_dataset[dataset].append(result)

        # Display table per dataset
        for dataset in sorted(by_dataset.keys()):
            print(f"\n{Colors.CYAN}Dataset: {dataset.upper()}{Colors.RESET}")
            print("┌" + "─" * 58 + "┐")
            print("│ Strategy                 │ Frames │ Avg Time │  FPS    │")
            print("├" + "─" * 58 + "┤")

            for result in sorted(by_dataset[dataset],
                                key=lambda r: r['fps'], reverse=True):
                strategy = result['strategy']
                frames = result['frames_processed']
                avg_time = result['avg_processing_ms']
                fps = result['fps']
                print(f"│ {strategy:24s} │ {frames:6d} │ {avg_time:7.2f}ms │ {fps:7.2f} │")

            print("└" + "─" * 58 + "┘")

        # Find best performer per dataset
        print(f"\n{Colors.GREEN}Best Performers:{Colors.RESET}")
        for dataset in sorted(by_dataset.keys()):
            best = max(by_dataset[dataset], key=lambda r: r['fps'])
            print(f"  {dataset:12s}: {best['strategy']:25s} "
                  f"({best['fps']:6.2f} FPS)")

        # Overall speedup
        if len(self.strategies) > 1:
            print(f"\n{Colors.CYAN}Performance Analysis:{Colors.RESET}")

            # Compare strategies
            by_strategy = {}
            for result in self.results:
                strategy = result['strategy']
                if strategy not in by_strategy:
                    by_strategy[strategy] = []
                by_strategy[strategy].append(result['fps'])

            baseline_strategy = self.strategies[0]
            baseline_fps = sum(by_strategy[baseline_strategy]) / len(by_strategy[baseline_strategy])

            print(f"  Baseline ({baseline_strategy}): {baseline_fps:.2f} FPS avg")
            for strategy in self.strategies[1:]:
                avg_fps = sum(by_strategy[strategy]) / len(by_strategy[strategy])
                improvement = ((avg_fps - baseline_fps) / baseline_fps) * 100
                symbol = "📈" if improvement > 0 else "📉"
                print(f"  {symbol} {strategy:25s}: {avg_fps:.2f} FPS avg "
                      f"({improvement:+.1f}%)")

        print()

# ============================================================================
# Main
# ============================================================================

def main():
    parser = argparse.ArgumentParser(
        description="Multi-Strategy Benchmarking for RS-VIO",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python3 scripts/benchmark_strategies_runner.py --all
  python3 scripts/benchmark_strategies_runner.py --fast
  python3 scripts/benchmark_strategies_runner.py --euroc
  python3 scripts/benchmark_strategies_runner.py --strategies BasicRANSAC,IMUGuided --all
        """
    )

    parser.add_argument("--all", action="store_true",
                       help="Test all strategies on all datasets (4-6 hours)")
    parser.add_argument("--fast", action="store_true",
                       help="Quick test: BasicRANSAC on EuRoC only (5 minutes)")
    parser.add_argument("--euroc", action="store_true",
                       help="Test on EuRoC dataset only")
    parser.add_argument("--tum", action="store_true",
                       help="Test on TUM-VI dataset only")
    parser.add_argument("--4seasons", action="store_true",
                       help="Test on 4Seasons dataset only")
    parser.add_argument("--strategies", type=str,
                       help="Comma-separated strategies (default: all)")
    parser.add_argument("--no-build", action="store_true",
                       help="Skip build step")

    args = parser.parse_args()

    # Determine strategies to test
    all_strategies = [
        "BasicRANSAC",
        "IMUGuided",
        "TemporalConsistency",
        "HybridOpticalFlow"
    ]

    if args.strategies:
        strategies = [s.strip() for s in args.strategies.split(",")]
    elif args.fast:
        strategies = ["BasicRANSAC"]
    else:
        strategies = all_strategies

    # Determine datasets to test
    available_datasets = find_available_datasets()

    if args.fast:
        selected_datasets = {k: v for k, v in available_datasets.items() if k == "euroc"}
    elif args.euroc:
        selected_datasets = {k: v for k, v in available_datasets.items() if k == "euroc"}
    elif args.tum:
        selected_datasets = {k: v for k, v in available_datasets.items() if k == "tum"}
    elif args.__dict__.get('4seasons'):
        selected_datasets = {k: v for k, v in available_datasets.items() if k == "4seasons"}
    else:
        selected_datasets = available_datasets

    if not selected_datasets:
        log_error("No datasets selected or found")
        sys.exit(1)

    print()
    print(f"Strategies to test: {', '.join(strategies)}")
    print(f"Datasets to test: {', '.join(selected_datasets.keys())}")
    print()

    # Run benchmarks
    runner = BenchmarkRunner(strategies, selected_datasets, args.no_build)
    success = runner.run_all()

    if success:
        log_header("Benchmarking Complete")
        print(f"Results: {runner.results_file}")
        print(f"CSV: {runner.csv_file}")
        print()
        print("Next steps:")
        print(f"  1. Review results: cat {runner.csv_file}")
        print("  2. Detailed analysis: python3 scripts/analyze_benchmark_results.py")
        print("  3. Deploy optimal: just deploy-optimal")
    else:
        log_error("Benchmarking failed")
        sys.exit(1)

if __name__ == "__main__":
    main()
