#!/usr/bin/env python3
"""
Simulate intrinsic refinement by applying percentage adjustments to intrinsics.

Usage:
    python tools/simulate_refinement.py <config_in> <config_out> <refinement_percent>
"""

import sys
import yaml


def main():
    if len(sys.argv) != 4:
        print(f"Usage: {sys.argv[0]} <config_in> <config_out> <refinement_percent>")
        sys.exit(1)

    config_in = sys.argv[1]
    config_out = sys.argv[2]
    refinement_pct = float(sys.argv[3])  # e.g., -0.1 for -0.1%

    # Apply refinement: multiply focal lengths by (1 + refinement_pct/100)
    refinement_factor = 1.0 + (refinement_pct / 100.0)

    try:
        # Read file and remove YAML directive for parsing
        with open(config_in) as f:
            content = f.read()

        # Remove YAML directive line if present
        if content.startswith('%YAML'):
            lines = content.split('\n')
            # Skip the %YAML line and the --- line
            content = '\n'.join(line for line in lines if not line.startswith('%YAML') and line.strip() != '---')

        # Parse YAML
        config = yaml.safe_load(content)

        if 'camera' in config:
            camera = config['camera']

            # Apply to left camera
            if 'left_intrinsics' in camera:
                left = camera['left_intrinsics']
                left[0] = float(left[0]) * refinement_factor  # fx
                left[1] = float(left[1]) * refinement_factor  # fy

            # Apply to right camera
            if 'right_intrinsics' in camera:
                right = camera['right_intrinsics']
                right[0] = float(right[0]) * refinement_factor  # fx
                right[1] = float(right[1]) * refinement_factor  # fy

        # Write output preserving YAML format
        with open(config_out, 'w') as f:
            f.write('%YAML:1.0\n')
            f.write('---\n\n')
            # Dump config without the default start document marker
            yaml.dump(config, f, default_flow_style=False, explicit_start=False)

        print(f"[INFO] Applied {refinement_pct}% refinement to intrinsics")
        print(f"[INFO] Output: {config_out}")

    except Exception as e:
        print(f"[ERROR] {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
