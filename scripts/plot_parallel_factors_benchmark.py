#!/usr/bin/env python3
"""
Plot parallel vs serial factor creation benchmark results.

Usage:
  python3 scripts/plot_parallel_factors_benchmark.py --input benchmark_results/parallel_factors_bench.txt --output benchmark_results/parallel_factors_bench.png

If --input is omitted, defaults to benchmark_results/parallel_factors_bench.txt.
"""

from __future__ import annotations

import argparse
import re
from pathlib import Path

try:
    import matplotlib.pyplot as plt
except ImportError as exc:
    raise SystemExit("matplotlib is required. Install with: pip install matplotlib") from exc


BENCH_PATTERN = re.compile(
    r"benchmark_parallel_factors\s+serial_ms=(\d+\.?\d*)\s+parallel_ms=(\d+\.?\d*)\s+speedup=(\d+\.?\d*)"
)


def parse_benchmarks(text: str) -> list[dict]:
    """Parse benchmark lines into a list of dicts."""
    results: list[dict] = []
    for line in text.splitlines():
        match = BENCH_PATTERN.search(line)
        if not match:
            continue
        serial_ms = float(match.group(1))
        parallel_ms = float(match.group(2))
        speedup = float(match.group(3))
        results.append(
            {
                "serial_ms": serial_ms,
                "parallel_ms": parallel_ms,
                "speedup": speedup,
            }
        )
    return results


def plot_results(results: list[dict], output: Path) -> None:
    if not results:
        raise SystemExit("No benchmark lines found in input.")

    # Use last result (most recent run) by default
    last = results[-1]
    serial_ms = last["serial_ms"]
    parallel_ms = last["parallel_ms"]
    speedup = last["speedup"]

    plt.style.use("seaborn-v0_8-whitegrid")
    fig, ax = plt.subplots(figsize=(8, 5), constrained_layout=True)

    labels = ["Serial", "Parallel"]
    values = [serial_ms, parallel_ms]
    colors = ["#4C78A8", "#59A14F"]

    bars = ax.bar(labels, values, color=colors)
    ax.set_ylabel("Time (ms)")
    ax.set_title("Parallel Factor Creation Benchmark")

    for bar, value in zip(bars, values):
        ax.text(
            bar.get_x() + bar.get_width() / 2,
            bar.get_height(),
            f"{value:.2f} ms",
            ha="center",
            va="bottom",
            fontsize=10,
        )

    ax.text(
        0.5,
        0.92,
        f"Speedup: {speedup:.2f}×",
        ha="center",
        va="center",
        fontsize=11,
        fontweight="bold",
        transform=ax.transAxes,
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=200)
    print(f"Saved plot: {output}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot parallel factor benchmark output.")
    parser.add_argument(
        "--input",
        type=Path,
        default=Path("benchmark_results/parallel_factors_bench.txt"),
        help="Input log file containing benchmark output",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("benchmark_results/parallel_factors_bench.png"),
        help="Output PNG file",
    )
    args = parser.parse_args()

    if not args.input.exists():
        raise SystemExit(f"Input file not found: {args.input}")

    text = args.input.read_text(encoding="utf-8")
    results = parse_benchmarks(text)
    plot_results(results, args.output)


if __name__ == "__main__":
    main()
