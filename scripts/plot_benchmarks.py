#!/usr/bin/env python3
"""
Benchmark plotting utility for RS-VIO

Reads Criterion benchmark results and generates performance visualizations.
Supports comparing benchmarks across different runs and branches.
"""

import json
import sys
from pathlib import Path
from typing import Dict, List, Tuple
import subprocess

try:
    import matplotlib.pyplot as plt
    import matplotlib.dates as mdates
    from datetime import datetime
except ImportError:
    print("Error: matplotlib not installed. Install with: pip install matplotlib")
    sys.exit(1)


class CriterionBenchmarkParser:
    """Parse Criterion benchmark JSON output."""

    def __init__(self, criterion_dir: Path = None):
        if criterion_dir is None:
            criterion_dir = Path("target/criterion")
        self.criterion_dir = criterion_dir

    def find_benchmark_dirs(self) -> List[Path]:
        """Find all benchmark result directories."""
        if not self.criterion_dir.exists():
            return []
        return [d for d in self.criterion_dir.iterdir() if d.is_dir() and (d / "base").exists()]

    def read_benchmark(self, bench_dir: Path) -> Dict:
        """Read a single benchmark result."""
        base_dir = bench_dir / "base"
        estimates_file = base_dir / "estimates.json"

        if not estimates_file.exists():
            return None

        with open(estimates_file) as f:
            data = json.load(f)

        return {
            "name": bench_dir.name,
            "mean": data.get("mean", {}).get("point_estimate"),
            "std_dev": data.get("std_dev", {}).get("point_estimate"),
            "lower": data.get("mean", {}).get("lower_bound"),
            "upper": data.get("mean", {}).get("upper_bound"),
        }

    def read_all_benchmarks(self) -> Dict[str, Dict]:
        """Read all benchmark results."""
        benchmarks = {}
        for bench_dir in self.find_benchmark_dirs():
            result = self.read_benchmark(bench_dir)
            if result:
                benchmarks[result["name"]] = result
        return benchmarks


def plot_benchmark_comparison(benchmarks: Dict[str, Dict], output_file: str = "benchmark_comparison.png"):
    """Plot comparison of all benchmarks."""
    if not benchmarks:
        print("No benchmarks found")
        return

    names = list(benchmarks.keys())
    means = [benchmarks[name]["mean"] for name in names]
    std_devs = [benchmarks[name]["std_dev"] for name in names]

    # Convert to milliseconds
    means_ms = [m / 1_000_000 for m in means]
    std_devs_ms = [s / 1_000_000 for s in std_devs]

    fig, ax = plt.subplots(figsize=(14, 8))

    x_pos = range(len(names))
    ax.bar(x_pos, means_ms, yerr=std_devs_ms, capsize=5, alpha=0.7, color="steelblue")

    ax.set_xticks(x_pos)
    ax.set_xticklabels(names, rotation=45, ha="right")
    ax.set_ylabel("Time (ms)", fontsize=12)
    ax.set_title("Benchmark Performance Comparison", fontsize=14, fontweight="bold")
    ax.grid(axis="y", alpha=0.3)

    plt.tight_layout()
    plt.savefig(output_file, dpi=300, bbox_inches="tight")
    print(f"Saved: {output_file}")


def plot_resolution_scaling(benchmarks: Dict[str, Dict], output_file: str = "resolution_scaling.png"):
    """Plot performance scaling with resolution."""
    resolution_data = {}

    for name, data in benchmarks.items():
        if "resolution" in name.lower():
            # Extract resolution from benchmark name
            if "320x240" in name:
                res = "320×240"
                order = 0
            elif "640x480" in name:
                res = "640×480"
                order = 1
            elif "1280x960" in name:
                res = "1280×960"
                order = 2
            else:
                continue

            if res not in resolution_data:
                resolution_data[res] = {"order": order, "time": data["mean"] / 1_000_000}

    if not resolution_data:
        return

    # Sort by order
    sorted_data = sorted(resolution_data.items(), key=lambda x: x[1]["order"])
    resolutions = [x[0] for x in sorted_data]
    times = [x[1]["time"] for x in sorted_data]

    fig, ax = plt.subplots(figsize=(10, 6))
    ax.plot(resolutions, times, marker="o", linewidth=2, markersize=8, color="darkgreen")
    ax.fill_between(range(len(resolutions)), times, alpha=0.3, color="lightgreen")

    ax.set_ylabel("Latency (ms)", fontsize=12)
    ax.set_xlabel("Resolution", fontsize=12)
    ax.set_title("Performance Scaling with Resolution", fontsize=14, fontweight="bold")
    ax.grid(True, alpha=0.3)

    # Add value labels on points
    for i, (res, time) in enumerate(zip(resolutions, times)):
        ax.text(i, time + 1, f"{time:.1f}ms", ha="center", fontsize=10)

    plt.tight_layout()
    plt.savefig(output_file, dpi=300, bbox_inches="tight")
    print(f"Saved: {output_file}")


def plot_grid_size_scaling(benchmarks: Dict[str, Dict], output_file: str = "grid_scaling.png"):
    """Plot performance scaling with grid size."""
    grid_data = {}

    for name, data in benchmarks.items():
        if "grid" in name.lower() or "grid_size" in name.lower():
            # Try to extract grid size
            for size in [5, 10, 15, 20]:
                if str(size) in name:
                    if size not in grid_data:
                        grid_data[size] = data["mean"] / 1_000_000
                    break

    if not grid_data:
        return

    sizes = sorted(grid_data.keys())
    times = [grid_data[s] for s in sizes]

    fig, ax = plt.subplots(figsize=(10, 6))
    ax.plot(sizes, times, marker="s", linewidth=2, markersize=8, color="darkorange")
    ax.fill_between(sizes, times, alpha=0.3, color="lightyellow")

    ax.set_ylabel("Latency (ms)", fontsize=12)
    ax.set_xlabel("Grid Size", fontsize=12)
    ax.set_title("Performance Scaling with Grid Size", fontsize=14, fontweight="bold")
    ax.grid(True, alpha=0.3)

    # Add value labels
    for size, time in zip(sizes, times):
        ax.text(size, time + 0.3, f"{time:.1f}ms", ha="center", fontsize=10)

    plt.tight_layout()
    plt.savefig(output_file, dpi=300, bbox_inches="tight")
    print(f"Saved: {output_file}")


def plot_category_breakdown(benchmarks: Dict[str, Dict], output_file: str = "category_breakdown.png"):
    """Plot breakdown by benchmark category."""
    categories = {}

    for name, data in benchmarks.items():
        # Extract category from benchmark name
        if "_" in name:
            category = name.split("_")[0]
        else:
            category = name

        if category not in categories:
            categories[category] = []
        categories[category].append(data["mean"] / 1_000_000)

    if not categories:
        return

    # Calculate averages
    cat_names = list(categories.keys())
    cat_avg = [sum(times) / len(times) for times in categories.values()]

    fig, ax = plt.subplots(figsize=(12, 6))
    colors = ["steelblue", "darkorange", "darkgreen", "darkred", "purple"]
    bars = ax.bar(cat_names, cat_avg, color=colors[: len(cat_names)], alpha=0.7)

    ax.set_ylabel("Average Latency (ms)", fontsize=12)
    ax.set_title("Performance by Benchmark Category", fontsize=14, fontweight="bold")
    ax.grid(axis="y", alpha=0.3)

    # Add value labels on bars
    for bar in bars:
        height = bar.get_height()
        ax.text(
            bar.get_x() + bar.get_width() / 2,
            height,
            f"{height:.1f}ms",
            ha="center",
            va="bottom",
            fontsize=10,
        )

    plt.tight_layout()
    plt.savefig(output_file, dpi=300, bbox_inches="tight")
    print(f"Saved: {output_file}")


def main():
    """Main entry point."""
    import argparse

    parser = argparse.ArgumentParser(description="Plot RS-VIO benchmark results")
    parser.add_argument(
        "--criterion-dir",
        type=Path,
        default=Path("target/criterion"),
        help="Path to Criterion output directory",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("target/benchmark_plots"),
        help="Output directory for plots",
    )
    parser.add_argument(
        "--plot-types",
        nargs="+",
        default=["comparison", "resolution", "grid", "category"],
        choices=["comparison", "resolution", "grid", "category", "all"],
        help="Types of plots to generate",
    )

    args = parser.parse_args()

    # Expand 'all' to all plot types
    if "all" in args.plot_types:
        args.plot_types = ["comparison", "resolution", "grid", "category"]

    # Create output directory
    args.output_dir.mkdir(parents=True, exist_ok=True)

    # Parse benchmarks
    parser_obj = CriterionBenchmarkParser(args.criterion_dir)
    benchmarks = parser_obj.read_all_benchmarks()

    if not benchmarks:
        print(f"No benchmarks found in {args.criterion_dir}")
        return 1

    print(f"Found {len(benchmarks)} benchmarks")

    # Generate plots
    if "comparison" in args.plot_types:
        plot_benchmark_comparison(benchmarks, args.output_dir / "benchmark_comparison.png")

    if "resolution" in args.plot_types:
        plot_resolution_scaling(benchmarks, args.output_dir / "resolution_scaling.png")

    if "grid" in args.plot_types:
        plot_grid_size_scaling(benchmarks, args.output_dir / "grid_scaling.png")

    if "category" in args.plot_types:
        plot_category_breakdown(benchmarks, args.output_dir / "category_breakdown.png")

    print(f"\nAll plots saved to {args.output_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
