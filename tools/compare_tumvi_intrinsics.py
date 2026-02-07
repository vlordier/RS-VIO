#!/usr/bin/env python3
import argparse
import re
from pathlib import Path
from typing import List


def parse_intrinsics_from_camchain(text: str) -> List[List[float]]:
    matches = re.findall(r"intrinsics:\s*\[([^\]]+)\]", text)
    if len(matches) < 2:
        raise ValueError("Expected at least two intrinsics entries in camchain.yaml")
    parsed = []
    for m in matches[:2]:
        vals = [float(v.strip()) for v in m.split(",") if v.strip()]
        if len(vals) < 4:
            raise ValueError("Intrinsics entry has fewer than 4 values")
        parsed.append(vals[:4])
    return parsed


def parse_intrinsics_from_config(text: str, key: str) -> List[float]:
    # Try single-line format first: key: [val1, val2, val3, val4]
    pattern = rf"{re.escape(key)}\s*:\s*\[([^\]]+)\]"
    match = re.search(pattern, text, re.DOTALL)
    if match:
        vals = [float(v.strip()) for v in match.group(1).split(",") if v.strip()]
        if len(vals) >= 4:
            return vals[:4]

    # Try multi-line YAML list format
    # key:
    # - val1
    # - val2
    # - val3
    # - val4
    pattern = rf"{re.escape(key)}:\s*\n((?:\s*-\s*[^\n]+\n)*)"
    match = re.search(pattern, text)
    if match:
        lines = match.group(1).split('\n')
        vals = []
        for line in lines:
            line = line.strip()
            if line.startswith('-'):
                val_str = line[1:].strip()
                try:
                    vals.append(float(val_str))
                except ValueError:
                    pass
        if len(vals) >= 4:
            return vals[:4]

    raise ValueError(f"Could not find {key} in config")


def format_row(label: str, values: List[float]) -> str:
    return f"{label:>10}  {values[0]:>10.6f}  {values[1]:>10.6f}  {values[2]:>10.6f}  {values[3]:>10.6f}"


def compare(label: str, dataset_vals: List[float], system_vals: List[float]) -> str:
    deltas = [system_vals[i] - dataset_vals[i] for i in range(4)]
    pct = [
        (d / dataset_vals[i] * 100.0) if dataset_vals[i] != 0 else 0.0
        for i, d in enumerate(deltas)
    ]
    lines = [
        f"{label}:",
        "             fx          fy          cx          cy",
        format_row("dataset", dataset_vals),
        format_row("system", system_vals),
        format_row("delta", deltas),
        format_row("delta%", pct),
        "",
    ]
    return "\n".join(lines)


def main() -> None:
    parser = argparse.ArgumentParser(description="Compare TUM-VI dataset intrinsics vs system intrinsics.")
    parser.add_argument("--dataset", default="/tmp/rs-vio-samples/tum_vi", help="Path to TUM-VI dataset root")
    parser.add_argument("--config", default="config/tum_vi.yaml", help="Path to system config YAML")
    args = parser.parse_args()

    camchain_path = Path(args.dataset) / "dso" / "camchain.yaml"
    config_path = Path(args.config)

    camchain_text = camchain_path.read_text()
    config_text = config_path.read_text()

    dataset_left, dataset_right = parse_intrinsics_from_camchain(camchain_text)
    system_left = parse_intrinsics_from_config(config_text, "left_intrinsics")
    system_right = parse_intrinsics_from_config(config_text, "right_intrinsics")

    print("TUM-VI intrinsics comparison")
    print(f"Dataset: {camchain_path}")
    print(f"System:  {config_path}")
    print("")
    print(compare("Left cam", dataset_left, system_left))
    print(compare("Right cam", dataset_right, system_right))


if __name__ == "__main__":
    main()
