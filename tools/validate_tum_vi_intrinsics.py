#!/usr/bin/env python3
"""
Validate TUM-VI calibration by extracting refined intrinsics from estimator logs.

This script:
1. Extracts refined intrinsics from Estimator::save_refined_intrinsics() output
2. Compares against ground truth from camchain.yaml
3. Exports refinement metrics in YAML format

Usage:
    python tools/validate_tum_vi_intrinsics.py --output /tmp/refined.yaml
"""

import argparse
import yaml
import re
from pathlib import Path
from typing import List, Dict, Any
import sys


def parse_yaml_intrinsics(yaml_text: str) -> Dict[str, List[float]]:
    """Parse intrinsics from YAML content."""
    try:
        data = yaml.safe_load(yaml_text)
        if not data or 'camera' not in data:
            raise ValueError("Invalid YAML format: missing 'camera' section")
        
        camera = data['camera']
        result = {}
        
        for key in ['left_intrinsics', 'right_intrinsics', 'original_left_intrinsics', 'original_right_intrinsics']:
            if key in camera:
                if isinstance(camera[key], list):
                    result[key] = camera[key][:4]
                else:
                    raise ValueError(f"Invalid format for {key}: expected list")
        
        return result
    except yaml.YAMLError as e:
        raise ValueError(f"YAML parsing error: {e}")


def parse_intrinsics_from_camchain(camchain_path: str) -> Dict[str, List[float]]:
    """Extract ground truth intrinsics from camchain.yaml."""
    with open(camchain_path) as f:
        content = f.read()
    
    # Parse two intrinsics entries for left and right cameras
    matches = re.findall(r"intrinsics:\s*\[([^\]]+)\]", content)
    if len(matches) < 2:
        raise ValueError("Expected at least two intrinsics entries in camchain.yaml")
    
    left_vals = [float(v.strip()) for v in matches[0].split(",") if v.strip()][:4]
    right_vals = [float(v.strip()) for v in matches[1].split(",") if v.strip()][:4]
    
    return {
        'ground_truth_left': left_vals,
        'ground_truth_right': right_vals
    }


def compute_errors(ground_truth: List[float], measured: List[float]) -> Dict[str, float]:
    """Compute absolute and percentage errors."""
    if len(ground_truth) != len(measured):
        raise ValueError("Arrays must have same length")
    
    errors = {
        'absolute_fx': measured[0] - ground_truth[0],
        'absolute_fy': measured[1] - ground_truth[1],
        'absolute_cx': measured[2] - ground_truth[2],
        'absolute_cy': measured[3] - ground_truth[3],
    }
    
    errors['percent_fx'] = (errors['absolute_fx'] / ground_truth[0] * 100.0) if ground_truth[0] != 0 else 0.0
    errors['percent_fy'] = (errors['absolute_fy'] / ground_truth[1] * 100.0) if ground_truth[1] != 0 else 0.0
    errors['percent_cx'] = (errors['absolute_cx'] / ground_truth[2] * 100.0) if ground_truth[2] != 0 else 0.0
    errors['percent_cy'] = (errors['absolute_cy'] / ground_truth[3] * 100.0) if ground_truth[3] != 0 else 0.0
    
    return errors


def generate_validation_report(
    refined_intrinsics: Dict[str, List[float]],
    ground_truth: Dict[str, List[float]]
) -> Dict[str, Any]:
    """Generate comprehensive validation report."""
    report = {
        'validation': {
            'timestamp': None,
            'dataset': 'TUM-VI',
            'left_camera': {
                'refined': refined_intrinsics.get('left_intrinsics'),
                'original': refined_intrinsics.get('original_left_intrinsics'),
                'ground_truth': ground_truth.get('ground_truth_left'),
            },
            'right_camera': {
                'refined': refined_intrinsics.get('right_intrinsics'),
                'original': refined_intrinsics.get('original_right_intrinsics'),
                'ground_truth': ground_truth.get('ground_truth_right'),
            }
        }
    }
    
    # Compute errors for left camera
    if (refined_intrinsics.get('left_intrinsics') and 
        ground_truth.get('ground_truth_left')):
        report['validation']['left_camera']['error'] = compute_errors(
            ground_truth['ground_truth_left'],
            refined_intrinsics['left_intrinsics']
        )
        
        if refined_intrinsics.get('original_left_intrinsics'):
            report['validation']['left_camera']['original_error'] = compute_errors(
                ground_truth['ground_truth_left'],
                refined_intrinsics['original_left_intrinsics']
            )
    
    # Compute errors for right camera
    if (refined_intrinsics.get('right_intrinsics') and 
        ground_truth.get('ground_truth_right')):
        report['validation']['right_camera']['error'] = compute_errors(
            ground_truth['ground_truth_right'],
            refined_intrinsics['right_intrinsics']
        )
        
        if refined_intrinsics.get('original_right_intrinsics'):
            report['validation']['right_camera']['original_error'] = compute_errors(
                ground_truth['ground_truth_right'],
                refined_intrinsics['original_right_intrinsics']
            )
    
    return report


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Validate TUM-VI calibration improvements"
    )
    parser.add_argument(
        "--input",
        default="/tmp/refined_intrinsics.yaml",
        help="Path to refined intrinsics YAML from Estimator"
    )
    parser.add_argument(
        "--output",
        required=True,
        help="Output path for validation report"
    )
    parser.add_argument(
        "--dataset",
        default="/tmp/rs-vio-samples/tum_vi",
        help="Path to TUM-VI dataset"
    )
    
    args = parser.parse_args()
    
    # Check if input file exists
    input_path = Path(args.input)
    if not input_path.exists():
        print(f"Error: Input file not found: {args.input}", file=sys.stderr)
        sys.exit(1)
    
    # Read refined intrinsics
    try:
        refined_yaml = input_path.read_text()
        refined_intrinsics = parse_yaml_intrinsics(refined_yaml)
        print(f"[INFO] Loaded refined intrinsics from {args.input}")
    except Exception as e:
        print(f"Error parsing refined intrinsics: {e}", file=sys.stderr)
        sys.exit(1)
    
    # Read ground truth
    camchain_path = Path(args.dataset) / "dso" / "camchain.yaml"
    if not camchain_path.exists():
        print(f"Error: Ground truth not found: {camchain_path}", file=sys.stderr)
        sys.exit(1)
    
    try:
        ground_truth = parse_intrinsics_from_camchain(str(camchain_path))
        print(f"[INFO] Loaded ground truth from {camchain_path}")
    except Exception as e:
        print(f"Error parsing ground truth: {e}", file=sys.stderr)
        sys.exit(1)
    
    # Generate validation report
    try:
        report = generate_validation_report(refined_intrinsics, ground_truth)
        
        # Save report
        output_path = Path(args.output)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        
        with open(output_path, 'w') as f:
            yaml.dump(report, f, default_flow_style=False)
        
        print(f"[INFO] Validation report saved to {args.output}")
        
        # Print summary
        print("\n=== Validation Summary ===")
        if 'error' in report['validation']['left_camera']:
            errors = report['validation']['left_camera']['error']
            print(f"Left Camera fx error: {errors['percent_fx']:.4f}%")
            print(f"Left Camera fy error: {errors['percent_fy']:.4f}%")
        
        if 'error' in report['validation']['right_camera']:
            errors = report['validation']['right_camera']['error']
            print(f"Right Camera fx error: {errors['percent_fx']:.4f}%")
            print(f"Right Camera fy error: {errors['percent_fy']:.4f}%")
        
        sys.exit(0)
        
    except Exception as e:
        print(f"Error generating validation report: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
