#!/usr/bin/env python3
"""
Check available datasets for multi-domain training.
Shows what datasets have been exported and are ready for training.
"""

import json
from pathlib import Path
from typing import Dict, List

def check_datasets():
    """Check all available datasets"""
    
    print("="*70)
    print("AVAILABLE DATASETS FOR MULTI-DOMAIN TRAINING")
    print("="*70)
    print()
    
    # Check exported_data_multi directory
    multi_path = Path("/Users/vincent/Work/RS-VIO/exported_data_multi")
    single_path = Path("/Users/vincent/Work/RS-VIO/exported_data")
    
    datasets = []
    
    # Check multi-dataset exports
    if multi_path.exists():
        print(f"📁 Multi-dataset exports: {multi_path}")
        for dataset_dir in sorted(multi_path.iterdir()):
            if dataset_dir.is_dir():
                metadata = dataset_dir / "metadata.json"
                if metadata.exists():
                    with open(metadata) as f:
                        data = json.load(f)
                    datasets.append({
                        'name': dataset_dir.name,
                        'path': str(dataset_dir),
                        'frames': len(data),
                        'type': 'multi'
                    })
        print()
    
    # Check single dataset export
    if single_path.exists():
        metadata = single_path / "metadata.json"
        if metadata.exists():
            print(f"📁 Single dataset export: {single_path}")
            with open(metadata) as f:
                data = json.load(f)
            datasets.append({
                'name': 'tumvi_room1',
                'path': str(single_path),
                'frames': len(data),
                'type': 'single'
            })
            print()
    
    if not datasets:
        print("❌ No exported datasets found!")
        print()
        print("To export datasets:")
        print("  ./export_multi_datasets.sh")
        print()
        return
    
    # Print dataset summary
    print("DATASET SUMMARY")
    print("-" * 70)
    print(f"{'Dataset Name':<30} {'Frames':<10} {'Type':<10}")
    print("-" * 70)
    
    total_frames = 0
    for ds in datasets:
        print(f"{ds['name']:<30} {ds['frames']:<10} {ds['type']:<10}")
        total_frames += ds['frames']
    
    print("-" * 70)
    print(f"{'TOTAL':<30} {total_frames:<10}")
    print("=" * 70)
    print()
    
    # Check dataset diversity
    print("DATASET CHARACTERISTICS")
    print("-" * 70)
    
    environments = set()
    for ds in datasets:
        name = ds['name'].lower()
        if 'outdoor' in name or '4seasons' in name or 'magistrale' in name:
            env = '🌳 Outdoor'
        elif 'euroc' in name:
            env = '🏭 Industrial'
        else:
            env = '🏠 Indoor Lab'
        
        difficulty = 'Easy'
        if 'difficult' in name or 'hard' in name:
            difficulty = 'Difficult'
        elif 'medium' in name:
            difficulty = 'Medium'
        
        print(f"{ds['name']:<30} {env:<15} {difficulty}")
        environments.add(env)
    
    print("=" * 70)
    print()
    
    # Training recommendations
    print("TRAINING RECOMMENDATIONS")
    print("-" * 70)
    
    if len(datasets) == 1:
        print("⚠️  Only 1 dataset available - limited generalization")
        print("   Recommendation: Export more datasets for better robustness")
        print("   Run: ./export_multi_datasets.sh")
    elif len(datasets) < 3:
        print("⚠️  Only", len(datasets), "datasets - moderate generalization")
        print("   Recommendation: Add 2-3 more diverse datasets")
    else:
        print("✅ Multiple datasets available - good for robust training")
    
    print()
    
    if len(environments) == 1:
        print("⚠️  All datasets from same environment")
        print("   Recommendation: Add outdoor, industrial, or different lighting")
    else:
        print(f"✅ {len(environments)} different environments - good diversity")
    
    print()
    
    if total_frames < 3000:
        print(f"⚠️  Limited data: {total_frames} frames")
        print("   Recommendation: Export more frames or sequences")
    elif total_frames < 6000:
        print(f"✓  Adequate data: {total_frames} frames")
    else:
        print(f"✅ Excellent data: {total_frames} frames")
    
    print("=" * 70)
    print()
    
    # Training command
    print("TRAINING COMMAND")
    print("-" * 70)
    
    if datasets:
        dataset_names = ' '.join([ds['name'] for ds in datasets[:5]])  # Use up to 5 datasets
        print("python3 train_multi_domain_refinement.py \\")
        print(f"    --datasets {dataset_names} \\")
        print("    --epochs 30 \\")
        print("    --batch-size 8")
    
    print("=" * 70)


if __name__ == "__main__":
    check_datasets()
