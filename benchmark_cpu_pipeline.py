#!/usr/bin/env python3
"""
CPU Benchmarking script for feature tracking pipeline.

This script profiles the image pyramid + gradient computation to measure
baseline performance and determine if GPU acceleration is justified.

Targets:
1. Measure time breakdown: pyramid (ms) + gradient (ms) per frame
2. Analyze for 100 frames to get statistical significance
3. Decision gate: >60% of frame time -> GPU implementation justified

Usage:
    python3 benchmark_cpu_pipeline.py
"""

import numpy as np
import time
import json
from pathlib import Path
from datetime import datetime
from PIL import Image
import cv2

# ============================================================================
# Configuration
# ============================================================================

CONFIG = {
    "image_root": "/Users/vincent/Work/RS-VIO/exported_data/images/cam0",
    "num_frames": 100,  # Profile first 100 frames
    "frame_rate": 30.0,  # Hz
    "output_dir": Path("/Users/vincent/Work/RS-VIO/results"),
}

TARGET_FRAME_TIME = 1000.0 / CONFIG["frame_rate"]  # 33.33 ms per frame @ 30 FPS
GPU_DECISION_THRESHOLD = 0.60  # If pyramid+gradient > 60% of frame time, GPU worth it

CONFIG["output_dir"].mkdir(exist_ok=True)


# ============================================================================
# Image Pyramid Implementation
# ============================================================================

class ImagePyramid:
    """Build image pyramid with 4 levels (original, 2x down, 4x down, 8x down)."""
    
    def __init__(self, num_levels=4):
        self.num_levels = num_levels
    
    def build(self, image):
        """
        Build Gaussian pyramid.
        
        Args:
            image: Input grayscale image (H, W)
        
        Returns:
            List of pyramid levels (4 images of decreasing resolution)
        """
        pyramid = [image]
        current = image
        
        for i in range(1, self.num_levels):
            # Gaussian blur and downsample
            blurred = cv2.GaussianBlur(current, (5, 5), 1.0)
            downsampled = cv2.resize(blurred, (blurred.shape[1]//2, blurred.shape[0]//2),
                                    interpolation=cv2.INTER_LINEAR)
            pyramid.append(downsampled)
            current = downsampled
        
        return pyramid


# ============================================================================
# Gradient Computation
# ============================================================================

class GradientComputer:
    """Compute image gradients using Sobel filters."""
    
    def __init__(self):
        pass
    
    def compute_gradients(self, image):
        """
        Compute image gradients using Sobel operator.
        
        Args:
            image: Input grayscale image (H, W)
        
        Returns:
            Tuple of (gx, gy) gradient images
        """
        # Sobel operators
        gx = cv2.Sobel(image, cv2.CV_32F, 1, 0, ksize=3)
        gy = cv2.Sobel(image, cv2.CV_32F, 0, 1, ksize=3)
        
        return gx, gy
    
    def compute_pyramid_gradients(self, pyramid):
        """
        Compute gradients for each level of pyramid.
        
        Args:
            pyramid: List of image pyramid levels
        
        Returns:
            List of (gx, gy) tuples for each level
        """
        gradients = []
        for image in pyramid:
            gx, gy = self.compute_gradients(image)
            gradients.append((gx, gy))
        
        return gradients


# ============================================================================
# Benchmarking
# ============================================================================

def load_image(path):
    """Load grayscale image."""
    try:
        img = Image.open(path).convert('L')
        return np.array(img, dtype=np.float32)
    except Exception as e:
        print(f"Error loading {path}: {e}")
        return np.zeros((480, 640), dtype=np.float32)


def get_sorted_images(image_root, num_frames):
    """Get sorted list of image paths."""
    image_root = Path(image_root)
    
    # Get all PNG files
    images = sorted(image_root.glob("*.png"))
    
    # Return first num_frames
    return images[:num_frames]


def benchmark_pipeline():
    """Run benchmark on feature tracking pipeline."""
    
    print("\n" + "="*80)
    print("CPU BENCHMARKING - FEATURE TRACKING PIPELINE")
    print("="*80)
    
    # Initialize components
    pyramid_builder = ImagePyramid(num_levels=4)
    gradient_computer = GradientComputer()
    
    # Get images
    print(f"\nLoading first {CONFIG['num_frames']} images...")
    image_paths = get_sorted_images(CONFIG["image_root"], CONFIG["num_frames"])
    
    if len(image_paths) == 0:
        print(f"ERROR: No images found in {CONFIG['image_root']}")
        return
    
    print(f"Found {len(image_paths)} images")
    
    # Benchmark structure
    results = {
        "frames": [],
        "summary": {
            "num_frames": 0,
            "target_frame_time_ms": TARGET_FRAME_TIME,
            "pyramid_avg_ms": 0.0,
            "gradient_avg_ms": 0.0,
            "total_avg_ms": 0.0,
            "gpu_percentage": 0.0,
            "gpu_justified": False,
        }
    }
    
    pyramid_times = []
    gradient_times = []
    
    # Run benchmark
    print("\n" + "="*80)
    print("PROFILING")
    print("="*80)
    
    for i, image_path in enumerate(image_paths):
        if (i + 1) % 10 == 0:
            print(f"  Processing frame {i+1}/{len(image_paths)}...")
        
        # Load image
        image = load_image(image_path)
        
        # Benchmark pyramid
        t0 = time.perf_counter()
        pyramid = pyramid_builder.build(image)
        pyramid_time = (time.perf_counter() - t0) * 1000  # Convert to ms
        
        # Benchmark gradients
        t0 = time.perf_counter()
        gradients = gradient_computer.compute_pyramid_gradients(pyramid)
        gradient_time = (time.perf_counter() - t0) * 1000  # Convert to ms
        
        pyramid_times.append(pyramid_time)
        gradient_times.append(gradient_time)
        
        # Record frame result
        total_time = pyramid_time + gradient_time
        results["frames"].append({
            "frame_index": i,
            "image_path": str(image_path),
            "pyramid_ms": float(pyramid_time),
            "gradient_ms": float(gradient_time),
            "total_ms": float(total_time),
        })
    
    # Compute statistics
    pyramid_avg = np.mean(pyramid_times)
    pyramid_std = np.std(pyramid_times)
    
    gradient_avg = np.mean(gradient_times)
    gradient_std = np.std(gradient_times)
    
    total_avg = pyramid_avg + gradient_avg
    total_std = np.std([pyramid_times[i] + gradient_times[i] for i in range(len(pyramid_times))])
    
    gpu_percentage = (total_avg / TARGET_FRAME_TIME) * 100.0
    gpu_justified = gpu_percentage > (GPU_DECISION_THRESHOLD * 100.0)
    
    # Update summary
    results["summary"]["num_frames"] = len(image_paths)
    results["summary"]["pyramid_avg_ms"] = float(pyramid_avg)
    results["summary"]["pyramid_std_ms"] = float(pyramid_std)
    results["summary"]["gradient_avg_ms"] = float(gradient_avg)
    results["summary"]["gradient_std_ms"] = float(gradient_std)
    results["summary"]["total_avg_ms"] = float(total_avg)
    results["summary"]["total_std_ms"] = float(total_std)
    results["summary"]["gpu_percentage"] = float(gpu_percentage)
    results["summary"]["gpu_justified"] = bool(gpu_justified)
    
    # Print results
    print("\n" + "="*80)
    print("RESULTS")
    print("="*80)
    print(f"\nFrames profiled: {len(image_paths)}")
    print(f"Target frame time (30 FPS): {TARGET_FRAME_TIME:.2f} ms")
    
    print(f"\nImage Pyramid:")
    print(f"  Average: {pyramid_avg:.4f} ms")
    print(f"  Std Dev: {pyramid_std:.4f} ms")
    print(f"  Min: {min(pyramid_times):.4f} ms")
    print(f"  Max: {max(pyramid_times):.4f} ms")
    
    print(f"\nGradient Computation:")
    print(f"  Average: {gradient_avg:.4f} ms")
    print(f"  Std Dev: {gradient_std:.4f} ms")
    print(f"  Min: {min(gradient_times):.4f} ms")
    print(f"  Max: {max(gradient_times):.4f} ms")
    
    print(f"\nTotal (Pyramid + Gradient):")
    print(f"  Average: {total_avg:.4f} ms")
    print(f"  Std Dev: {total_std:.4f} ms")
    print(f"  As % of frame time: {gpu_percentage:.1f}%")
    
    print("\n" + "="*80)
    print("GPU DECISION")
    print("="*80)
    
    if gpu_justified:
        print(f"✓ GPU ACCELERATION JUSTIFIED")
        print(f"  {total_avg:.4f} ms ({gpu_percentage:.1f}%) > {GPU_DECISION_THRESHOLD*100:.0f}% threshold")
        print(f"  Expected speedup: ~4-8x (GPU vs CPU)")
        print(f"  Potential savings: {total_avg * 0.85:.2f}-{total_avg * 0.875:.2f} ms per frame")
    else:
        print(f"✗ GPU ACCELERATION NOT JUSTIFIED")
        print(f"  {total_avg:.4f} ms ({gpu_percentage:.1f}%) < {GPU_DECISION_THRESHOLD*100:.0f}% threshold")
        print(f"  Feature tracking is already fast enough on CPU")
        print(f"  GPU overhead would outweigh benefits")
    
    # Save results
    results["timestamp"] = datetime.now().isoformat()
    results["device"] = "Apple M4 Pro (CPU)"
    results["gpu_decision_threshold"] = GPU_DECISION_THRESHOLD
    
    results_path = CONFIG["output_dir"] / "benchmark_results.json"
    with open(results_path, 'w') as f:
        json.dump(results, f, indent=2)
    
    print(f"\nResults saved to {results_path}")
    
    return results


if __name__ == "__main__":
    try:
        results = benchmark_pipeline()
        if results["summary"]["gpu_justified"]:
            exit(0)  # GPU implementation recommended
        else:
            exit(1)  # CPU implementation sufficient
    except Exception as e:
        print(f"ERROR: {e}")
        import traceback
        traceback.print_exc()
        exit(2)
