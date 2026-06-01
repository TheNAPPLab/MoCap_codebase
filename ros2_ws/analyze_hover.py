#!/usr/bin/env python3
"""
MPC Hover Bag Analyzer
Reads all rosbag2 .db3 files in mpc_hover_bags/, parses pose data,
plots each flight, and identifies the most optimal hover.

Usage:
    python3 analyze_mpc_hover.py
    python3 analyze_mpc_hover.py --bags-dir /path/to/mpc_hover_bags
"""

import sqlite3
import struct
import math
import os
import glob
import argparse
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from pathlib import Path


# ---------------------------------------------------------------------------
# CDR deserialization helpers
# ---------------------------------------------------------------------------

def parse_pose_stamped(data: bytes):
    """
    Parse geometry_msgs/msg/PoseStamped from CDR bytes.
    Layout (little-endian):
      [0:4]   CDR header
      [4:8]   stamp.sec  (uint32)
      [8:12]  stamp.nanosec (uint32)
      [12:16] frame_id string length
      [16:16+len] frame_id chars (null-terminated)
      [pad to 8]
      [+24]   position  x, y, z  (float64 × 3)
      [+24]   orientation qx, qy, qz, qw (float64 × 4)
    """
    sec  = struct.unpack_from('<I', data, 4)[0]
    nsec = struct.unpack_from('<I', data, 8)[0]
    t    = sec + nsec * 1e-9

    # find pose data — empirically starts at offset 28 for 'world\0' frame_id
    offset = 28
    x, y, z          = struct.unpack_from('<ddd',  data, offset)
    qx, qy, qz, qw   = struct.unpack_from('<dddd', data, offset + 24)
    return t, x, y, z, qx, qy, qz, qw


def parse_named_pose_array(data: bytes):
    """
    Parse motion_capture_tracking_interfaces/msg/NamedPoseArray from CDR bytes.
    Returns list of (name, x, y, z, qx, qy, qz, qw).
    Quaternion components may be NaN when mocap loses tracking.
    """
    offset = 4
    sec  = struct.unpack_from('<I', data, offset)[0]; offset += 4
    nsec = struct.unpack_from('<I', data, offset)[0]; offset += 4
    t    = sec + nsec * 1e-9

    # frame_id string
    slen = struct.unpack_from('<I', data, offset)[0]; offset += 4
    offset += slen
    if offset % 4:
        offset += 4 - (offset % 4)

    # array of named poses
    alen = struct.unpack_from('<I', data, offset)[0]; offset += 4
    poses = []
    for _ in range(alen):
        nlen = struct.unpack_from('<I', data, offset)[0]; offset += 4
        name = data[offset:offset + nlen - 1].decode('utf-8', errors='replace')
        offset += nlen
        if offset % 4:
            offset += 4 - (offset % 4)
        # 7 float64 values: x y z qx qy qz qw
        vals = struct.unpack_from('<7d', data, offset); offset += 56
        poses.append((name, *vals))
    return t, poses


# ---------------------------------------------------------------------------
# Bag reading
# ---------------------------------------------------------------------------

def read_bag(db3_path: str) -> dict:
    """
    Read a single rosbag2 .db3 file and return parsed data.
    Returns dict with keys:
        'name'      : filename stem
        'pose'      : list of (t, x, y, z, qx, qy, qz, qw) from /cfB/pose
        'mocap'     : list of (t, x, y, z) from /poses (cfB drone only)
        'duration'  : total bag duration in seconds
    """
    conn = sqlite3.connect(db3_path)
    c    = conn.cursor()

    # map topic names to ids
    c.execute('SELECT id, name FROM topics')
    topic_map = {name: tid for tid, name in c.fetchall()}

    pose_data  = []
    mocap_data = []

    # /cfB/pose
    if '/cfB/pose' in topic_map:
        tid = topic_map['/cfB/pose']
        c.execute('SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp', (tid,))
        for ts, raw in c.fetchall():
            try:
                t, x, y, z, qx, qy, qz, qw = parse_pose_stamped(raw)
                pose_data.append((t, x, y, z, qx, qy, qz, qw))
            except struct.error:
                continue

    # /poses (mocap)
    if '/poses' in topic_map:
        tid = topic_map['/poses']
        c.execute('SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp', (tid,))
        for ts, raw in c.fetchall():
            try:
                t, poses = parse_named_pose_array(raw)
                for name, x, y, z, qx, qy, qz, qw in poses:
                    if 'cfB' in name or 'cf' in name.lower():
                        mocap_data.append((t, x, y, z))
            except struct.error:
                continue

    conn.close()

    # normalise timestamps to t=0
    src = pose_data if pose_data else mocap_data
    t0  = src[0][0] if src else 0.0

    pose_data  = [(t - t0, x, y, z, qx, qy, qz, qw) for t, x, y, z, qx, qy, qz, qw in pose_data]
    mocap_data = [(t - t0, x, y, z) for t, x, y, z in mocap_data]

    duration = (src[-1][0] - t0) if src else 0.0

    return {
        'name'    : Path(db3_path).stem,
        'path'    : db3_path,
        'pose'    : pose_data,
        'mocap'   : mocap_data,
        'duration': duration,
    }


# ---------------------------------------------------------------------------
# Flight metrics
# ---------------------------------------------------------------------------

def compute_metrics(bag: dict, target_z: float = 0.5) -> dict:
    """
    Compute hover quality metrics from a bag.

    Metrics:
        hover_z_mean   : mean altitude during hover phase
        hover_z_std    : std deviation of altitude during hover (lower = better)
        hover_xy_std   : std deviation of XY position during hover (lower = better)
        z_error_mean   : mean |z - target_z| during hover
        settle_time    : seconds until z first reaches within 5% of target
        overshoot_pct  : max z overshoot above target as % of target
        hover_duration : time spent hovering (z > 0.3m)
        score          : composite score (lower is better)
    """
    data = bag['pose'] if bag['pose'] else [(t, x, y, z, 0, 0, 0, 1) for t, x, y, z in bag['mocap']]
    if not data:
        return {}

    arr = np.array(data)  # columns: t x y z qx qy qz qw
    t_arr, x_arr, y_arr, z_arr = arr[:, 0], arr[:, 1], arr[:, 2], arr[:, 3]

    # hover phase: z > 0.3 m
    hover_mask = z_arr > 0.3
    if hover_mask.sum() < 5:
        return {'score': float('inf'), 'hover_duration': 0.0}

    hz  = z_arr[hover_mask]
    hx  = x_arr[hover_mask]
    hy  = y_arr[hover_mask]
    ht  = t_arr[hover_mask]

    hover_z_mean  = float(np.mean(hz))
    hover_z_std   = float(np.std(hz))
    hover_xy_std  = float(np.std(np.sqrt((hx - hx.mean())**2 + (hy - hy.mean())**2)))
    z_error_mean  = float(np.mean(np.abs(hz - target_z)))
    hover_duration= float(ht[-1] - ht[0])

    # settle time: first index where z >= 0.95 * target_z
    threshold     = 0.95 * target_z
    reach_idx     = np.argmax(z_arr >= threshold)
    settle_time   = float(t_arr[reach_idx]) if z_arr[reach_idx] >= threshold else float('inf')

    # overshoot
    max_z         = float(np.max(z_arr))
    overshoot_pct = max(0.0, (max_z - target_z) / target_z * 100)

    # composite score: weight altitude accuracy most heavily
    score = (
        2.0 * z_error_mean
        + 1.5 * hover_z_std
        + 1.0 * hover_xy_std
        + 0.1 * settle_time
        + 0.02 * overshoot_pct
    )

    return {
        'hover_z_mean'  : hover_z_mean,
        'hover_z_std'   : hover_z_std,
        'hover_xy_std'  : hover_xy_std,
        'z_error_mean'  : z_error_mean,
        'settle_time'   : settle_time,
        'overshoot_pct' : overshoot_pct,
        'hover_duration': hover_duration,
        'score'         : score,
    }


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------

COLORS = plt.cm.tab10.colors


def plot_all_flights(bags: list, metrics_list: list, best_idx: int, target_z: float,
                     save_path: str = None):
    """
    Plot overview of all flights on a shared axes, then detailed breakdown of best.
    """
    n = len(bags)
    fig = plt.figure(figsize=(18, 10))
    gs  = gridspec.GridSpec(2, 3, figure=fig, hspace=0.38, wspace=0.35)

    ax_z   = fig.add_subplot(gs[0, :2])   # altitude comparison (all flights)
    ax_xy  = fig.add_subplot(gs[0, 2])    # XY drift (all flights)
    ax_bar = fig.add_subplot(gs[1, 0])    # metric comparison bar chart
    ax_best_z  = fig.add_subplot(gs[1, 1])  # best flight altitude detail
    ax_best_xy = fig.add_subplot(gs[1, 2])  # best flight XY path

    # --- all flights altitude ---
    ax_z.axhline(target_z, color='gray', linestyle='--', linewidth=1.2, label=f'Target {target_z} m')
    for i, (bag, m) in enumerate(zip(bags, metrics_list)):
        data = bag['pose'] if bag['pose'] else [(t, x, y, z, 0, 0, 0, 1) for t, x, y, z in bag['mocap']]
        if not data:
            continue
        arr  = np.array(data)
        lw   = 2.5 if i == best_idx else 1.0
        alpha= 1.0 if i == best_idx else 0.5
        label= f"{bag['name']}" + (' ★ BEST' if i == best_idx else '')
        ax_z.plot(arr[:, 0], arr[:, 3], color=COLORS[i % 10],
                  linewidth=lw, alpha=alpha, label=label)

    ax_z.set_xlabel('Time (s)')
    ax_z.set_ylabel('Z altitude (m)')
    ax_z.set_title('Altitude Profile — All Flights')
    ax_z.legend(fontsize=7, loc='upper right', ncol=max(1, n // 5))
    ax_z.grid(True, alpha=0.3)

    # --- XY drift scatter (hover phase only) ---
    for i, (bag, m) in enumerate(zip(bags, metrics_list)):
        data = bag['pose'] if bag['pose'] else [(t, x, y, z, 0, 0, 0, 1) for t, x, y, z in bag['mocap']]
        if not data:
            continue
        arr   = np.array(data)
        hover = arr[arr[:, 3] > 0.3]
        if len(hover) < 2:
            continue
        lw   = 2.0 if i == best_idx else 0.8
        alpha= 1.0 if i == best_idx else 0.4
        ax_xy.plot(hover[:, 1] - hover[:, 1].mean(),
                   hover[:, 2] - hover[:, 2].mean(),
                   color=COLORS[i % 10], linewidth=lw, alpha=alpha)

    ax_xy.set_xlabel('X deviation (m)')
    ax_xy.set_ylabel('Y deviation (m)')
    ax_xy.set_title('XY Drift During Hover\n(centred at mean)')
    ax_xy.set_aspect('equal', 'box')
    ax_xy.grid(True, alpha=0.3)

    # --- bar chart of z_error_mean per flight ---
    names   = [b['name'] for b in bags]
    z_errs  = [m.get('z_error_mean', 0) for m in metrics_list]
    colors  = [COLORS[i % 10] for i in range(n)]
    bars    = ax_bar.bar(range(n), z_errs, color=colors, edgecolor='white', linewidth=0.5)
    bars[best_idx].set_edgecolor('gold'); bars[best_idx].set_linewidth(2.5)
    ax_bar.set_xticks(range(n))
    ax_bar.set_xticklabels([b['name'].replace('_', '\n') for b in bags],
                           fontsize=6, rotation=30, ha='right')
    ax_bar.set_ylabel('Mean |z error| (m)')
    ax_bar.set_title('Altitude Error per Flight\n(★ = best)')
    ax_bar.grid(True, axis='y', alpha=0.3)
    ax_bar.annotate('★', xy=(best_idx, z_errs[best_idx]),
                    ha='center', va='bottom', fontsize=14, color='goldenrod')

    # --- best flight detail: altitude ---
    best_bag = bags[best_idx]
    best_m   = metrics_list[best_idx]
    data     = best_bag['pose'] if best_bag['pose'] else \
               [(t, x, y, z, 0, 0, 0, 1) for t, x, y, z in best_bag['mocap']]
    arr      = np.array(data)

    ax_best_z.plot(arr[:, 0], arr[:, 3], color=COLORS[best_idx % 10], linewidth=1.8)
    ax_best_z.axhline(target_z,        color='gray',   linestyle='--', linewidth=1.2, label='Target')
    ax_best_z.axhline(target_z * 0.95, color='green',  linestyle=':',  linewidth=1.0, label='±5%')
    ax_best_z.axhline(target_z * 1.05, color='green',  linestyle=':',  linewidth=1.0)

    st = best_m.get('settle_time', None)
    if st and st < float('inf'):
        ax_best_z.axvline(st, color='orange', linestyle='--', linewidth=1.2,
                          label=f'Settle {st:.2f}s')

    ax_best_z.set_xlabel('Time (s)')
    ax_best_z.set_ylabel('Z altitude (m)')
    ax_best_z.set_title(f'Best Flight: {best_bag["name"]}\nAltitude Detail')
    ax_best_z.legend(fontsize=7)
    ax_best_z.grid(True, alpha=0.3)

    # --- best flight XY path ---
    hover = arr[arr[:, 3] > 0.3]
    ax_best_xy.plot(arr[:, 1],  arr[:, 2],  color='lightgray', linewidth=1, label='Full flight')
    if len(hover) > 1:
        ax_best_xy.plot(hover[:, 1], hover[:, 2], color=COLORS[best_idx % 10],
                        linewidth=1.6, label='Hover phase')
    ax_best_xy.set_xlabel('X (m)')
    ax_best_xy.set_ylabel('Y (m)')
    ax_best_xy.set_title(f'Best Flight XY Path')
    ax_best_xy.set_aspect('equal', 'box')
    ax_best_xy.legend(fontsize=8)
    ax_best_xy.grid(True, alpha=0.3)

    # --- metrics text box ---
    stats = (
        f"Score:           {best_m.get('score', float('inf')):.4f}\n"
        f"Mean Z:          {best_m.get('hover_z_mean', 0):.4f} m\n"
        f"Z std dev:       {best_m.get('hover_z_std', 0):.4f} m\n"
        f"Z error (mean):  {best_m.get('z_error_mean', 0):.4f} m\n"
        f"XY std dev:      {best_m.get('hover_xy_std', 0):.4f} m\n"
        f"Settle time:     {best_m.get('settle_time', float('inf')):.2f} s\n"
        f"Overshoot:       {best_m.get('overshoot_pct', 0):.1f}%\n"
        f"Hover duration:  {best_m.get('hover_duration', 0):.1f} s"
    )
    ax_best_z.text(
        0.02, 0.97, stats,
        transform=ax_best_z.transAxes,
        fontsize=6.5, verticalalignment='top', fontfamily='monospace',
        bbox=dict(facecolor='white', alpha=0.75, edgecolor='gray', boxstyle='round,pad=0.4')
    )

    fig.suptitle('MPC Hover Analysis', fontsize=14, fontweight='bold', y=1.01)

    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f'  Plot saved: {save_path}')
    else:
        plt.show()
    plt.close()


def print_summary_table(bags: list, metrics_list: list, best_idx: int):
    """Print a formatted summary table to stdout."""
    cols = ['Name', 'Z mean (m)', 'Z std (m)', 'Z err (m)', 'XY std (m)',
            'Settle (s)', 'Overshoot %', 'Score']
    widths = [30, 10, 10, 10, 10, 10, 12, 10]
    header = '  '.join(f'{c:<{w}}' for c, w in zip(cols, widths))
    sep    = '-' * len(header)

    print('\n' + sep)
    print(header)
    print(sep)
    for i, (bag, m) in enumerate(zip(bags, metrics_list)):
        if not m or 'score' not in m:
            continue
        tag  = ' ★' if i == best_idx else '  '
        vals = [
            bag['name'] + tag,
            f"{m.get('hover_z_mean', float('nan')):.4f}",
            f"{m.get('hover_z_std',  float('nan')):.4f}",
            f"{m.get('z_error_mean', float('nan')):.4f}",
            f"{m.get('hover_xy_std', float('nan')):.4f}",
            f"{m.get('settle_time',  float('inf')):.2f}",
            f"{m.get('overshoot_pct',float('nan')):.1f}",
            f"{m.get('score',        float('inf')):.4f}",
        ]
        print('  '.join(f'{v:<{w}}' for v, w in zip(vals, widths)))
    print(sep)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description='Analyse MPC hover rosbags')
    parser.add_argument(
        '--bags-dir',
        default=os.path.join(
            os.path.expanduser('~'),
            'Documents/Aaron2/ros2_ws/src/crazyswarm2/ros2_ws/src/crazyswarm2'
            '/crazyflie_examples/crazyflie_examples/mpc/mpc_hover_bags'
        ),
        help='Path to the mpc_hover_bags directory'
    )
    parser.add_argument(
        '--target-z',
        type=float,
        default=0.5,
        help='Target hover altitude in metres (default 0.5)'
    )
    parser.add_argument(
        '--save-plot',
        default='mpc_hover_analysis.png',
        help='Save plot to this path instead of displaying it (use "" to display)'
    )
    args = parser.parse_args()

    bags_dir = args.bags_dir
    if not os.path.isdir(bags_dir):
        print(f'[ERROR] Bags directory not found: {bags_dir}')
        print('  Pass --bags-dir /path/to/mpc_hover_bags')
        return

    # find all .db3 files (each is one bag, metadata.yaml lives alongside)
    db3_files = sorted(glob.glob(os.path.join(bags_dir, '**', '*.db3'), recursive=True))
    if not db3_files:
        db3_files = sorted(glob.glob(os.path.join(bags_dir, '*.db3')))
    if not db3_files:
        print(f'[ERROR] No .db3 files found under {bags_dir}')
        return

    print(f'\nFound {len(db3_files)} bag(s) in {bags_dir}\n')

    bags        = []
    metrics_list= []

    for path in db3_files:
        print(f'  Reading: {os.path.basename(path)} ...', end='', flush=True)
        try:
            bag = read_bag(path)
            m   = compute_metrics(bag, target_z=args.target_z)
            bags.append(bag)
            metrics_list.append(m)
            score_str = f"  score={m.get('score', float('inf')):.4f}" if m else '  (no pose data)'
            print(f'  {len(bag["pose"])} pose msgs  {bag["duration"]:.1f}s{score_str}')
        except Exception as e:
            print(f'  FAILED: {e}')

    if not bags:
        print('[ERROR] No bags were read successfully.')
        return

    # find best
    scores    = [m.get('score', float('inf')) for m in metrics_list]
    best_idx  = int(np.argmin(scores))
    best_name = bags[best_idx]['name']

    print(f'\n{"="*60}')
    print(f'  OPTIMAL FLIGHT: {best_name}  (score={scores[best_idx]:.4f})')
    print(f'{"="*60}')
    print_summary_table(bags, metrics_list, best_idx)

    save_path = args.save_plot if args.save_plot else None
    plot_all_flights(bags, metrics_list, best_idx, args.target_z, save_path=save_path)


if __name__ == '__main__':
    main()