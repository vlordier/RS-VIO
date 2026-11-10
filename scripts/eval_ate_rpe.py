#!/usr/bin/env python3
"""
Robust trajectory evaluation for RS-VIO vs TUM-VI ground truth.

Metrics computed:
  - ATE (Absolute Trajectory Error) after SE(3) Umeyama alignment
      * RMSE, mean, median, std, max
      * Per-segment ATE (drift over 5-segment split)
      * Path-length-normalized drift (% of path length)
  - RPE (Relative Pose Error) at multiple time intervals
      * 0.1s, 1.0s, 5.0s, 10.0s
      * Both translation (m/s) and rotation (deg/s)
  - Trajectory statistics
      * Path length (GT and estimated)
      * Duration, GT coverage
      * Scale error (estimated path length / GT path length)

Usage:
    python3 scripts/eval_ate_rpe.py \
        --est  <dataset_path>/estimated_trajectory.txt \
        --gt   <dataset_path>/mav0/mocap0/data.csv \
        [--plot]                  # generate trajectory plot
        [--rpe_intervals 0.1,1.0,5.0,10.0]

GT formats supported:
  - TUM-VI dso/gt_imu.csv:  '# timestamp[ns],tx,ty,tz,qw,qx,qy,qz'
  - TUM-VI mav0/mocap0/data.csv: same format
"""

import argparse
import sys
import numpy as np
from scipy.spatial.transform import Rotation


def read_csv_gt(path):
    """Read TUM-VI CSV ground truth. Returns (timestamps_ns, positions, quats_xyzw)."""
    ts, pos, quat = [], [], []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split(',')
            if len(parts) < 8:
                continue
            t_ns = int(parts[0])
            tx, ty, tz = float(parts[1]), float(parts[2]), float(parts[3])
            qw, qx, qy, qz = float(parts[4]), float(parts[5]), float(parts[6]), float(parts[7])
            ts.append(t_ns)
            pos.append([tx, ty, tz])
            quat.append([qx, qy, qz, qw])  # scipy convention: xyzw
    return np.array(ts, dtype=np.int64), np.array(pos), np.array(quat)


def read_est(path):
    """Read TUM-format estimated trajectory. Returns (timestamps_ns, positions, quats_xyzw)."""
    ts, pos, quat = [], [], []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            if len(parts) < 8:
                continue
            t_s = float(parts[0])
            tx, ty, tz = float(parts[1]), float(parts[2]), float(parts[3])
            qx, qy, qz, qw = float(parts[4]), float(parts[5]), float(parts[6]), float(parts[7])
            ts.append(int(round(t_s * 1e9)))
            pos.append([tx, ty, tz])
            quat.append([qx, qy, qz, qw])
    return np.array(ts, dtype=np.int64), np.array(pos), np.array(quat)


def associate(ts_est, ts_gt, max_diff_ns=50_000_000):
    """Associate each estimated timestamp to nearest GT timestamp within max_diff_ns."""
    pairs = []
    for i, t in enumerate(ts_est):
        # Binary search
        lo, hi = 0, len(ts_gt) - 1
        while lo < hi:
            mid = (lo + hi) // 2
            if ts_gt[mid] < t:
                lo = mid + 1
            else:
                hi = mid
        best = lo
        if lo > 0 and abs(ts_gt[lo - 1] - t) < abs(ts_gt[lo] - t):
            best = lo - 1
        if abs(ts_gt[best] - t) <= max_diff_ns:
            pairs.append((i, best))
    return pairs


def umeyama_align(src, dst, with_scale=False):
    """
    Compute rigid/similarity SE(3) transform aligning src to dst.
    Returns (R, t, scale) such that dst ≈ scale * R @ src + t.
    """
    n = src.shape[0]
    mu_src = src.mean(0)
    mu_dst = dst.mean(0)
    src_c = src - mu_src
    dst_c = dst - mu_dst
    H = src_c.T @ dst_c / n
    U, S, Vt = np.linalg.svd(H)
    d = np.linalg.det(Vt.T @ U.T)
    D = np.diag([1, 1, d])
    R = Vt.T @ D @ U.T
    if with_scale:
        var_src = (src_c * src_c).sum() / n
        scale = (S * np.array([1, 1, d])).sum() / var_src if var_src > 0 else 1.0
    else:
        scale = 1.0
    t = mu_dst - scale * R @ mu_src
    return R, t, scale


def path_length(positions):
    """Cumulative 3D path length."""
    if len(positions) < 2:
        return 0.0
    diffs = np.diff(positions, axis=0)
    return float(np.linalg.norm(diffs, axis=1).sum())


def split_gt_segments(ts_gt, gap_threshold_s=5.0):
    """Return list of (start_idx, end_idx_exclusive) for continuous GT segments."""
    if len(ts_gt) == 0:
        return []
    gap_ns = int(gap_threshold_s * 1e9)
    diffs = np.diff(ts_gt)
    gap_positions = np.where(diffs > gap_ns)[0]
    segs = []
    start = 0
    for gp in gap_positions:
        segs.append((start, gp + 1))
        start = gp + 1
    segs.append((start, len(ts_gt)))
    return segs


def compute_ate(pos_est, pos_gt, pairs, segments=5, ts_gt=None):
    """ATE after SE(3) alignment. Returns (errors, seg_rmse, scale, gt_seg_info).

    If ts_gt has large gaps, each continuous GT segment is aligned independently
    to avoid cross-gap distortion of the Sim(3) scale estimate.
    gt_seg_info is a list of per-segment dicts (or None for single-segment GT).
    """
    est_m = pos_est[[p[0] for p in pairs]]
    gt_m  = pos_gt [[p[1] for p in pairs]]
    pair_gt_idx = np.array([p[1] for p in pairs])

    gt_segs = split_gt_segments(ts_gt) if ts_gt is not None else [(0, len(pos_gt))]
    multi_seg = len(gt_segs) > 1

    if multi_seg:
        all_errors = np.full(len(pairs), np.nan)
        seg_info = []
        for seg_start, seg_end in gt_segs:
            mask = (pair_gt_idx >= seg_start) & (pair_gt_idx < seg_end)
            if mask.sum() < 6:
                continue
            e, g = est_m[mask], gt_m[mask]
            R, t, _ = umeyama_align(e, g, with_scale=False)
            errs = np.linalg.norm((R @ e.T).T + t - g, axis=1)
            all_errors[mask] = errs
            _, _, sc = umeyama_align(e, g, with_scale=True)
            seg_info.append({
                'n': int(mask.sum()),
                'gt_path_m': float(path_length(g)),
                'ate_rmse_m': float(np.sqrt(np.mean(errs**2))),
                'scale': float(sc),
            })
        valid = ~np.isnan(all_errors)
        errors = all_errors[valid]
        total_path = sum(s['gt_path_m'] for s in seg_info)
        scale = sum(s['scale'] * s['gt_path_m'] for s in seg_info) / max(total_path, 1e-6) if seg_info else 1.0
    else:
        R, t, _ = umeyama_align(est_m, gt_m, with_scale=False)
        errors = np.linalg.norm((R @ est_m.T).T + t - gt_m, axis=1)
        _, _, scale = umeyama_align(est_m, gt_m, with_scale=True)
        seg_info = None

    seg_rmse = []
    n = len(errors)
    for i in range(segments):
        lo = (i * n) // segments
        hi = ((i + 1) * n) // segments
        seg_rmse.append(float(np.sqrt(np.mean(errors[lo:hi] ** 2))))

    return errors, seg_rmse, scale, seg_info


def compute_rpe(ts_est, pos_est, quat_est, pos_gt, quat_gt, pairs, delta_s):
    """RPE: compare relative transforms between est pose pairs and corresponding gt pose pairs."""
    delta_ns = int(delta_s * 1e9)
    est_idx = [p[0] for p in pairs]
    gt_idx  = [p[1] for p in pairs]
    ts_m = ts_est[est_idx]
    # Each pose participates in many RPE pairs (and multiple delta windows).
    # Convert each trajectory's quaternions once instead of repeatedly invoking
    # SciPy's quaternion parser inside the nested pair loop.
    rot_est = Rotation.from_quat(quat_est).as_matrix()
    rot_gt = Rotation.from_quat(quat_gt).as_matrix()

    trans_err = []
    rot_err = []

    for k in range(len(pairs)):
        target = ts_m[k] + delta_ns
        best = -1
        best_diff = delta_ns * 0.5
        # The timestamps are monotonic, so only the neighbors around the
        # target can be the nearest match. Bound the original forward scan to
        # its existing target+delta horizon, but find candidates in O(log n).
        insertion = int(np.searchsorted(ts_m, target, side='left'))
        first = max(k + 1, insertion)
        end = int(np.searchsorted(ts_m, target + delta_ns, side='right'))
        if insertion > k + 1:
            previous = insertion - 1
            # Keep the first index on duplicate timestamps, matching the old
            # forward scan's stable tie behavior.
            previous = max(
                k + 1,
                int(np.searchsorted(ts_m, ts_m[previous], side='left')),
            )
            candidates = (previous, first)
        else:
            candidates = (first,)
        for j in candidates:
            if j < k + 1 or j >= end:
                continue
            diff = abs(ts_m[j] - target)
            if diff < best_diff:
                best_diff = diff
                best = j
        if best < 0:
            continue

        i0, i1 = est_idx[k], est_idx[best]
        j0, j1 = gt_idx[k], gt_idx[best]

        def rel(pos, rotations, a, b):
            Ra = rotations[a]
            Rb = rotations[b]
            dR = Ra.T @ Rb
            dt = Ra.T @ (pos[b] - pos[a])
            return dR, dt

        dR_e, dt_e = rel(pos_est, rot_est, i0, i1)
        dR_g, dt_g = rel(pos_gt,  rot_gt,  j0, j1)

        err_R = dR_g.T @ dR_e
        err_t = dt_e - dt_g
        angle = np.arccos(np.clip((np.trace(err_R) - 1) / 2, -1, 1))

        trans_err.append(np.linalg.norm(err_t) / delta_s)
        rot_err.append(np.degrees(angle) / delta_s)

    return np.array(trans_err), np.array(rot_err)


def fmt_stats(arr):
    if len(arr) == 0:
        return "  (no data)"
    return (f"    RMSE   = {np.sqrt(np.mean(arr**2)):.4f}\n"
            f"    mean   = {np.mean(arr):.4f}\n"
            f"    median = {np.median(arr):.4f}\n"
            f"    max    = {np.max(arr):.4f}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--est', required=True)
    parser.add_argument('--gt', required=True)
    parser.add_argument('--rpe_intervals', default='0.1,1.0,5.0,10.0')
    parser.add_argument('--plot', action='store_true')
    parser.add_argument('--plot_out', default=None, help='Save plot to file (default: show)')
    parser.add_argument('--json', action='store_true', help='Output JSON summary')
    args = parser.parse_args()

    ts_gt, pos_gt, quat_gt = read_csv_gt(args.gt)
    ts_est, pos_est, quat_est = read_est(args.est)

    if len(ts_est) < 2 or len(ts_gt) < 2:
        print("ERROR: insufficient data", file=sys.stderr)
        sys.exit(1)

    pairs = associate(ts_est, ts_gt)
    if len(pairs) < 10:
        print(f"ERROR: too few associations ({len(pairs)})", file=sys.stderr)
        sys.exit(1)

    # Trajectory stats
    est_len = path_length(pos_est)
    gt_len  = path_length(pos_gt[[p[1] for p in pairs]])
    est_dur = (ts_est[-1] - ts_est[0]) * 1e-9
    gt_cov = (ts_gt[-1] - ts_gt[0]) * 1e-9

    # ATE
    ate_errors, seg_rmse, sim3_scale, gt_seg_info = compute_ate(pos_est, pos_gt, pairs, ts_gt=ts_gt)
    ate_rmse = float(np.sqrt(np.mean(ate_errors ** 2)))

    # RPE at multiple intervals
    intervals = [float(x) for x in args.rpe_intervals.split(',')]
    rpe_results = {}
    for dt in intervals:
        trans_err, rot_err = compute_rpe(ts_est, pos_est, quat_est, pos_gt, quat_gt, pairs, dt)
        rpe_results[dt] = (trans_err, rot_err)

    # --- Output ---
    if args.json:
        import json
        summary = {
            'pairs_evaluated': len(pairs),
            'est_poses': len(ts_est),
            'gt_poses': len(ts_gt),
            'est_path_length_m': est_len,
            'gt_path_length_m': gt_len,
            'est_duration_s': est_dur,
            'gt_coverage_s': gt_cov,
            'ate_rmse_m': ate_rmse,
            'ate_mean_m': float(np.mean(ate_errors)),
            'ate_max_m': float(np.max(ate_errors)),
            'ate_normalized_pct': 100.0 * ate_rmse / max(gt_len, 1e-6),
            'ate_per_segment_rmse': seg_rmse,
            'sim3_scale': sim3_scale,
            'scale_error_pct': 100.0 * (sim3_scale - 1.0),
            'rpe': {
                str(dt): {
                    'trans_rmse_m_per_s': float(np.sqrt(np.mean(te**2))) if len(te) else None,
                    'rot_rmse_deg_per_s': float(np.sqrt(np.mean(re**2))) if len(re) else None,
                    'n': int(len(te)),
                }
                for dt, (te, re) in rpe_results.items()
            },
        }
        print(json.dumps(summary, indent=2))
        return

    gt_segs_list = split_gt_segments(ts_gt)
    print(f"=== Trajectory Statistics ===")
    print(f"  Est poses:      {len(ts_est)}")
    print(f"  GT poses:       {len(ts_gt)}")
    print(f"  GT segments:    {len(gt_segs_list)} ({', '.join(f'{(ts_gt[e-1]-ts_gt[s])*1e-9:.1f}s' for s,e in gt_segs_list if e>s)})")
    print(f"  Pairs matched:  {len(pairs)} ({100.0*len(pairs)/len(ts_est):.1f}%)")
    print(f"  Duration:       {est_dur:.1f}s (est), {gt_cov:.1f}s (gt window)")
    print(f"  Path length:    {est_len:.2f} m (est), {gt_len:.2f} m (gt matched)")
    print(f"  Sim(3) scale:   {sim3_scale:.6f} ({100*(sim3_scale-1):.3f}%)")
    print()
    print(f"=== ATE (m, after SE3 alignment) ===")
    print(fmt_stats(ate_errors))
    print(f"    drift/length = {100.0 * ate_rmse / max(gt_len, 1e-6):.3f}% of path length")
    print(f"    per-segment RMSE (5 segments): {[f'{x:.3f}' for x in seg_rmse]}")
    if gt_seg_info:
        print(f"  Per-GT-segment breakdown:")
        for i, si in enumerate(gt_seg_info):
            print(f"    seg{i+1}: {si['n']} pairs, gt_path={si['gt_path_m']:.2f}m, ATE={si['ate_rmse_m']:.4f}m, scale={si['scale']:.4f}")
    print()
    for dt in intervals:
        te, re = rpe_results[dt]
        print(f"=== RPE Δ={dt:.2f}s (n={len(te)}) ===")
        print(f"  trans RMSE: {float(np.sqrt(np.mean(te**2))):.4f} m/s" if len(te) else "  (no data)")
        print(f"  rot RMSE:   {float(np.sqrt(np.mean(re**2))):.4f} deg/s" if len(re) else "  (no data)")

    if args.plot:
        try:
            import matplotlib
            if args.plot_out:
                matplotlib.use('Agg')
            import matplotlib.pyplot as plt

            est_m = pos_est[[p[0] for p in pairs]]
            gt_m  = pos_gt [[p[1] for p in pairs]]
            R, t, _ = umeyama_align(est_m, gt_m)
            est_aligned = (R @ est_m.T).T + t

            fig = plt.figure(figsize=(14, 5))

            ax1 = fig.add_subplot(131)
            ax1.plot(gt_m[:, 0], gt_m[:, 1], 'g-', label='GT', linewidth=1.5)
            ax1.plot(est_aligned[:, 0], est_aligned[:, 1], 'r--', label='Est (aligned)', linewidth=1)
            ax1.set_xlabel('X [m]'); ax1.set_ylabel('Y [m]')
            ax1.set_title('XY trajectory')
            ax1.legend(); ax1.axis('equal'); ax1.grid(True, alpha=0.3)

            ax2 = fig.add_subplot(132)
            ax2.plot(gt_m[:, 0], gt_m[:, 2], 'g-', label='GT', linewidth=1.5)
            ax2.plot(est_aligned[:, 0], est_aligned[:, 2], 'r--', label='Est (aligned)', linewidth=1)
            ax2.set_xlabel('X [m]'); ax2.set_ylabel('Z [m]')
            ax2.set_title('XZ trajectory')
            ax2.legend(); ax2.axis('equal'); ax2.grid(True, alpha=0.3)

            ax3 = fig.add_subplot(133)
            t_rel = (ts_est[[p[0] for p in pairs]] - ts_est[pairs[0][0]]) * 1e-9
            ax3.plot(t_rel, ate_errors, 'b-', linewidth=1)
            ax3.set_xlabel('time [s]'); ax3.set_ylabel('ATE [m]')
            ax3.set_title(f'Per-frame ATE (RMSE={ate_rmse:.3f}m)')
            ax3.grid(True, alpha=0.3)

            plt.tight_layout()
            if args.plot_out:
                plt.savefig(args.plot_out, dpi=100)
                print(f"\nPlot saved to {args.plot_out}")
            else:
                plt.show()
        except ImportError:
            print("\n(matplotlib not available — skip plotting)")


if __name__ == '__main__':
    main()
