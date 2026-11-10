# Quick Start: 256×256 + CLAHE Preprocessing

## Run Evaluation
```bash
python3 scripts/eval_all_tum_vi_256_4x.py --json results_256.json
```

## Enable Preprocessing for Existing Configs
```yaml
# In config/your_config.yaml
camera:
  preprocessing:
    enable: true
    clahe_clip_limit: 2.5
    clahe_tile_size: 16
```

## Preprocessing Parameters

### clip_limit (2.0-3.5)
- **2.0-2.5** (conservative): Preserves fine details, good for well-lit scenes
- **2.5-3.0** (balanced): Default, works for most scenes
- **3.0-3.5** (aggressive): High contrast enhancement, good for dark/shadowy scenes

### tile_size (8-16)
- **8** (fine-grained): More local detail, slower
- **16** (balanced): Default, good speed/quality trade-off
- **32** (coarse): Less local detail, faster but may lose structure

## Expected Performance

### Speed
- 512×512 baseline: 38 fps
- 256×256 with CLAHE: ~100 fps (3.7× speedup)

### Accuracy (room1 baseline)
- Without preprocessing: ATE=0.223m, scale=-4.020%
- With preprocessing: Testing...

## Files

| File | Purpose | Size |
|------|---------|------|
| src/image_preprocessing.rs | Core CLAHE + highlight shaping | 180 lines |
| config/tum_vi_256.yaml | 256×256 config with preprocessing | 99 lines |
| scripts/eval_all_tum_vi_256_4x.py | Evaluation script (4 runs per seq) | 288 lines |

## Decision Flowchart

```
Evaluation Completes
  ├─ room1 ATE improves >2%
  │  └─ ✓ Use as development baseline
  ├─ Other sequences regress <5%
  │  └─ ✓ Preprocessing is safe
  └─ Any sequence regresses >5%
     └─ ❌ Revert or try 384×384
```

## Benchmarks
- room1: 0.223m → ? (pending evaluation)
- slides1: 0.123m → ? (pending evaluation)
- corridor1: 0.062m → ? (pending evaluation)
- corridor2: 0.064m → ? (pending evaluation)

## Check Results

When evaluation completes:
```bash
cat /tmp/eval_256_results.json | python3 -m json.tool
```

Expected JSON structure:
```json
{
  "slides1": {
    "runs": [
      {"ate_rmse": 0.120, "scale": 1.001, "scale_pct": 0.1},
      ...
    ],
    "aggregate": {
      "ate_rmse_mean": 0.121,
      "ate_rmse_std": 0.005,
      ...
    }
  },
  ...
}
```

## Technical Details

- CLAHE: O(w×h) tile-based histogram equalization
- Highlight shaping: Sigmoid compression for bright pixels
- Preprocessing cost: ~0.25ms/frame (negligible)
- Grid adjustment: 30→16 for 256×256 (maintain feature density)

See IMPLEMENTATION_SUMMARY_256.md for full details.
