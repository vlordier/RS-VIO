#!/usr/bin/env python3
"""
Generate TUM-VI calibration validation report from comparison results.

This script:
1. Reads comparison results for three strategies (Baseline, Online Refinement, Offline Post-Processing)
2. Extracts delta% values for each strategy
3. Generates markdown report with improvement metrics

Usage:
    python tools/generate_tum_vi_report.py --input /tmp/comparison_results
"""

import argparse
import re
from pathlib import Path
from typing import Dict, Optional
import sys


def parse_delta_percent(file_path: Path) -> Optional[Dict[str, float]]:
    """
    Extract delta% values from comparison output file.
    
    Expected format:
    ...
    delta%      value1      value2      value3      value4
    ...
    """
    try:
        content = file_path.read_text()
    except FileNotFoundError:
        return None
    
    # Look for delta% line
    delta_pattern = r'delta%\s+(-?\d+\.?\d*)\s+(-?\d+\.?\d*)\s+(-?\d+\.?\d*)\s+(-?\d+\.?\d*)'
    match = re.search(delta_pattern, content)
    
    if not match:
        return None
    
    fx_pct, fy_pct, cx_pct, cy_pct = match.groups()
    
    return {
        'fx': float(fx_pct),
        'fy': float(fy_pct),
        'cx': float(cx_pct),
        'cy': float(cy_pct),
    }


def compute_improvement(baseline_pct: float, strategy_pct: float) -> float:
    """
    Compute improvement percentage.
    
    Improvement = (|baseline| - |strategy|) / |baseline| * 100
    Positive = improvement, negative = degradation
    """
    if baseline_pct == 0:
        return 0.0
    
    baseline_abs = abs(baseline_pct)
    strategy_abs = abs(strategy_pct)
    
    # Improvement as percentage reduction in error
    improvement = ((baseline_abs - strategy_abs) / baseline_abs) * 100.0
    return improvement


def format_delta_percent(value: float) -> str:
    """Format delta% value with sign."""
    if value >= 0:
        return f"+{value:.4f}%"
    else:
        return f"{value:.4f}%"


def format_improvement(value: float) -> str:
    """Format improvement value with color indicator."""
    if value > 0:
        return f"✓ {value:+.2f}%"  # Improvement
    elif value < 0:
        return f"✗ {value:+.2f}%"  # Degradation
    else:
        return "= 0.00%"  # No change


def generate_report(
    baseline: Dict[str, float],
    strategy2: Optional[Dict[str, float]],
    strategy3: Optional[Dict[str, float]]
) -> str:
    """Generate markdown report."""
    report = []
    report.append("# TUM-VI Calibration Validation Report\n")
    report.append("## Intrinsics Refinement Strategy Evaluation\n")
    
    # Baseline
    report.append("### Baseline (No Refinement)\n")
    report.append("| Parameter | Error % |\n")
    report.append("|-----------|----------|\n")
    report.append(f"| fx | {format_delta_percent(baseline['fx'])} |\n")
    report.append(f"| fy | {format_delta_percent(baseline['fy'])} |\n")
    report.append(f"| cx | {format_delta_percent(baseline['cx'])} |\n")
    report.append(f"| cy | {format_delta_percent(baseline['cy'])} |\n")
    report.append("\n")
    
    # Strategy 2: Online Refinement
    if strategy2:
        report.append("### Strategy 2: Online Refinement\n")
        report.append("| Parameter | Error % | Improvement |\n")
        report.append("|-----------|---------|-------------|\n")
        
        fx_imp = compute_improvement(baseline['fx'], strategy2['fx'])
        fy_imp = compute_improvement(baseline['fy'], strategy2['fy'])
        cx_imp = compute_improvement(baseline['cx'], strategy2['cx'])
        cy_imp = compute_improvement(baseline['cy'], strategy2['cy'])
        
        report.append(f"| fx | {format_delta_percent(strategy2['fx'])} | {format_improvement(fx_imp)} |\n")
        report.append(f"| fy | {format_delta_percent(strategy2['fy'])} | {format_improvement(fy_imp)} |\n")
        report.append(f"| cx | {format_delta_percent(strategy2['cx'])} | {format_improvement(cx_imp)} |\n")
        report.append(f"| cy | {format_delta_percent(strategy2['cy'])} | {format_improvement(cy_imp)} |\n")
        report.append("\n")
        
        # Average improvement
        avg_improvement = (fx_imp + fy_imp + cx_imp + cy_imp) / 4.0
        report.append(f"**Average Improvement: {format_improvement(avg_improvement)}**\n\n")
    else:
        report.append("### Strategy 2: Online Refinement\n")
        report.append("*No data available*\n\n")
    
    # Strategy 3: Offline Post-Processing
    if strategy3:
        report.append("### Strategy 3: Offline Post-Processing\n")
        report.append("| Parameter | Error % | Improvement |\n")
        report.append("|-----------|---------|-------------|\n")
        
        fx_imp = compute_improvement(baseline['fx'], strategy3['fx'])
        fy_imp = compute_improvement(baseline['fy'], strategy3['fy'])
        cx_imp = compute_improvement(baseline['cx'], strategy3['cx'])
        cy_imp = compute_improvement(baseline['cy'], strategy3['cy'])
        
        report.append(f"| fx | {format_delta_percent(strategy3['fx'])} | {format_improvement(fx_imp)} |\n")
        report.append(f"| fy | {format_delta_percent(strategy3['fy'])} | {format_improvement(fy_imp)} |\n")
        report.append(f"| cx | {format_delta_percent(strategy3['cx'])} | {format_improvement(cx_imp)} |\n")
        report.append(f"| cy | {format_delta_percent(strategy3['cy'])} | {format_improvement(cy_imp)} |\n")
        report.append("\n")
        
        # Average improvement
        avg_improvement = (fx_imp + fy_imp + cx_imp + cy_imp) / 4.0
        report.append(f"**Average Improvement: {format_improvement(avg_improvement)}**\n\n")
    else:
        report.append("### Strategy 3: Offline Post-Processing\n")
        report.append("*No data available*\n\n")
    
    # Best strategy
    report.append("## Summary\n")
    
    # Determine best strategy based on fx (primary focal length)
    strategies = [('Baseline', baseline['fx'])]
    if strategy2:
        strategies.append(('Online Refinement', strategy2['fx']))
    if strategy3:
        strategies.append(('Offline Post-Processing', strategy3['fx']))
    
    best_strategy = min(strategies, key=lambda x: abs(x[1]))
    
    report.append(f"**Best Strategy:** {best_strategy[0]}\n")
    report.append(f"**Best fx Error:** {format_delta_percent(best_strategy[1])}\n")
    report.append("\n")
    
    # Comparison table
    report.append("## Strategy Comparison\n")
    report.append("| Metric | Baseline | Online | Offline |\n")
    report.append("|--------|----------|--------|----------|\n")
    
    s2_fx = format_delta_percent(strategy2['fx']) if strategy2 else "N/A"
    s3_fx = format_delta_percent(strategy3['fx']) if strategy3 else "N/A"
    report.append(f"| fx error | {format_delta_percent(baseline['fx'])} | {s2_fx} | {s3_fx} |\n")
    
    s2_fy = format_delta_percent(strategy2['fy']) if strategy2 else "N/A"
    s3_fy = format_delta_percent(strategy3['fy']) if strategy3 else "N/A"
    report.append(f"| fy error | {format_delta_percent(baseline['fy'])} | {s2_fy} | {s3_fy} |\n")
    
    report.append("\n")
    
    return "".join(report)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Generate TUM-VI calibration validation report"
    )
    parser.add_argument(
        "--input",
        required=True,
        help="Directory with comparison results (strategy1.txt, strategy2.txt, strategy3.txt)"
    )
    
    args = parser.parse_args()
    
    input_dir = Path(args.input)
    if not input_dir.is_dir():
        print(f"Error: Input directory not found: {args.input}", file=sys.stderr)
        sys.exit(1)
    
    # Load baseline (Strategy 1)
    baseline_file = input_dir / "strategy1.txt"
    baseline = parse_delta_percent(baseline_file)
    
    if not baseline:
        print(f"Error: Could not parse baseline from {baseline_file}", file=sys.stderr)
        sys.exit(1)
    
    print(f"[INFO] Loaded baseline from {baseline_file}", file=sys.stderr)
    
    # Load Strategy 2 (Online Refinement)
    strategy2_file = input_dir / "strategy2.txt"
    strategy2 = parse_delta_percent(strategy2_file)
    
    if strategy2:
        print(f"[INFO] Loaded Strategy 2 from {strategy2_file}", file=sys.stderr)
    else:
        print(f"[WARNING] Strategy 2 not found at {strategy2_file}", file=sys.stderr)
    
    # Load Strategy 3 (Offline Post-Processing)
    strategy3_file = input_dir / "strategy3.txt"
    strategy3 = parse_delta_percent(strategy3_file)
    
    if strategy3:
        print(f"[INFO] Loaded Strategy 3 from {strategy3_file}", file=sys.stderr)
    else:
        print(f"[WARNING] Strategy 3 not found at {strategy3_file}", file=sys.stderr)
    
    # Generate report
    report = generate_report(baseline, strategy2, strategy3)
    
    # Output to stdout
    print(report)
    
    sys.exit(0)


if __name__ == "__main__":
    main()
