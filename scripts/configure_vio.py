#!/usr/bin/env python3
"""
VIO Pipeline Configuration Helper

Interactive tool to select and customize VIO pipeline configurations
based on target platform and performance requirements.
"""

import argparse
import sys
from pathlib import Path

# Platform configurations with their characteristics
PLATFORMS = {
    "cpu_only": {
        "name": "CPU-Only (Raspberry Pi 5 / ARM)",
        "target_fps": 20,
        "memory_mb": 256,
        "gpu": False,
        "description": "Optimized for embedded ARM processors with limited resources",
    },
    "gpu_enabled": {
        "name": "GPU-Enabled (Jetson Nano/Xavier/Orin)",
        "target_fps": 30,
        "memory_mb": 1024,
        "gpu": True,
        "description": "Leverages GPU for learned features and descriptors",
    },
    "hard_realtime": {
        "name": "Hard Realtime (Minimal Latency)",
        "target_fps": 60,
        "memory_mb": 128,
        "gpu": False,
        "description": "Strict timing guarantees, minimal latency",
    },
    "balanced": {
        "name": "Balanced (Default)",
        "target_fps": 30,
        "memory_mb": 512,
        "gpu": False,
        "description": "Good quality/performance tradeoff for typical drones",
    },
}


def print_platform_info():
    """Print available platform configurations."""
    print("\n=== Available Platform Configurations ===\n")
    for key, config in PLATFORMS.items():
        print(f"{key:15} - {config['name']}")
        print(f"{'':15}   Target: {config['target_fps']} FPS, {config['memory_mb']} MB RAM")
        print(f"{'':15}   GPU: {'Yes' if config['gpu'] else 'No'}")
        print(f"{'':15}   {config['description']}")
        print()


def select_platform_interactive():
    """Interactive platform selection."""
    print_platform_info()
    while True:
        choice = input("Select platform [cpu_only/gpu_enabled/hard_realtime/balanced]: ").strip()
        if choice in PLATFORMS:
            return choice
        print(f"Invalid choice: {choice}. Please try again.")


def print_config_summary(platform: str):
    """Print configuration summary."""
    config = PLATFORMS[platform]
    print(f"\n=== Configuration Summary: {config['name']} ===")
    print(f"Target FPS:    {config['target_fps']}")
    print(f"Memory Budget: {config['memory_mb']} MB")
    print(f"GPU Enabled:   {'Yes' if config['gpu'] else 'No'}")
    print(f"Description:   {config['description']}")
    print()


def print_tuning_recommendations(platform: str):
    """Print platform-specific tuning recommendations."""
    print("\n=== Tuning Recommendations ===\n")

    if platform == "cpu_only":
        print("CPU-Only Platform:")
        print("  - Use fewer pyramid levels (2-3) to reduce compute")
        print("  - Reduce max_features to 150-200")
        print("  - Set fusion.num_frames to 3 to save memory")
        print("  - Disable loop closure if FPS drops below target")
        print("  - Consider FAST detector instead of Shi-Tomasi if CPU-limited")

    elif platform == "gpu_enabled":
        print("GPU-Enabled Platform:")
        print("  - Can use SuperPoint for keypoint detection on keyframes")
        print("  - Enable LightGlue for robust loop closure matching")
        print("  - Increase max_features to 300-400 for better accuracy")
        print("  - Use adaptive fusion strategy for best quality")
        print("  - Consider depth-aware fusion for challenging scenes")

    elif platform == "hard_realtime":
        print("Hard Realtime Platform:")
        print("  - Minimize feature count (80-150)")
        print("  - Use FAST/AGAST detector (faster than Shi-Tomasi)")
        print("  - Disable fusion (strategy = None)")
        print("  - Disable loop closure and bundle adjustment")
        print("  - Reduce stereo disparity search range to 64")
        print("  - Use larger grid cells (48px) to reduce detection overhead")

    elif platform == "balanced":
        print("Balanced Platform:")
        print("  - Adjust max_features based on scene complexity (150-300)")
        print("  - Enable rotation-only fusion for better features")
        print("  - Use ORB descriptors for keyframe matching")
        print("  - Tune pyramid_levels (2-4) based on motion speed")
        print("  - Monitor CPU usage and reduce feature count if needed")

    print()


def main():
    parser = argparse.ArgumentParser(
        description="VIO Pipeline Configuration Helper",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--list",
        action="store_true",
        help="List available platform configurations",
    )
    parser.add_argument(
        "--platform",
        choices=list(PLATFORMS.keys()),
        help="Select platform configuration",
    )
    parser.add_argument(
        "--interactive",
        action="store_true",
        help="Interactive configuration selection",
    )
    parser.add_argument(
        "--recommend",
        action="store_true",
        help="Show tuning recommendations",
    )

    args = parser.parse_args()

    # List platforms
    if args.list:
        print_platform_info()
        return 0

    # Interactive mode
    if args.interactive:
        platform = select_platform_interactive()
        print_config_summary(platform)
        print_tuning_recommendations(platform)

        config_path = Path(__file__).parent.parent / "configs" / f"{platform}.toml"
        print(f"Configuration file: {config_path}")
        print("\nTo use this configuration:")
        print(f"  VIOPipelineConfig::load_toml(\"{config_path}\")")
        return 0

    # Direct platform selection
    if args.platform:
        print_config_summary(args.platform)
        if args.recommend:
            print_tuning_recommendations(args.platform)

        config_path = Path(__file__).parent.parent / "configs" / f"{args.platform}.toml"
        print(f"\nConfiguration file: {config_path}")
        return 0

    # No arguments - show help
    parser.print_help()
    return 1


if __name__ == "__main__":
    sys.exit(main())
