#!/usr/bin/env python3
"""
plot_figure8_bags.py

Reads ROS2 bag files from the figure8_bags directory and generates
comparison plots grouped by test type. Each plot group contains:
  - Z position over time (altitude stability)
  - XY trajectory (path tracking)
  - Battery voltage over time

Usage:
    python3 plot_figure8_bags.py

Output:
    PNG plots saved to figure8_plots/ inside the figure8_bags directory.

Requirements:
    pip install rosbag2-py rclpy matplotlib numpy
    (Must be run with ROS2 sourced: source /opt/ros/humble/setup.bash)
"""

import os
import sqlite3
import struct
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from pathlib import Path
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


# ── Configuration ─────────────────────────────────────────────────────────────

BAGS_ROOT = Path(
    "/home/crazyflie/Documents/Aaron2/ros2_ws/src/crazyswarm2/ros2_ws/figure8_bags"
)
OUTPUT_DIR = BAGS_ROOT / "figure8_plots"

POSE_TOPIC   = "/cfB/pose"
STATUS_TOPIC = "/cfB/status"

# Baseline bag (timescale=1.5, scale_xy=0.7, takeoff_height=0.5)
BASELINE = {
    "label": "Baseline (ts=1.5, xy=0.7, h=0.5)",
    "bag":   "2026-05-26_12-04-44",
}

# ── All test groups with their bags ──────────────────────────────────────────
# Each entry: {"label": str, "bag": str (folder name inside BAGS_ROOT)}

TEST_GROUPS = {
    "Timescale Isolation": [
        {"label": "ts=1.2", "bag": "2026-05-26_11-40-35"},
        {"label": "ts=1.0", "bag": "2026-05-26_12-04-44"},  # also baseline
        {"label": "ts=0.8", "bag": "2026-05-26_12-21-50"},
        {"label": "ts=0.6", "bag": "2026-05-26_12-34-37"},
    ],
    "scale_xy Isolation": [
        {"label": "xy=0.8", "bag": "2026-05-26_12-49-27"},
        {"label": "xy=0.9", "bag": "2026-05-26_12-54-31"},
        {"label": "xy=1.0", "bag": "2026-05-26_13-14-00"},
        {"label": "xy=1.2", "bag": "2026-05-26_13-20-33"},
    ],
    "takeoff_height Isolation": [
        {"label": "h=0.7", "bag": "2026-05-26_13-54-40"},
        {"label": "h=1.0", "bag": "2026-05-26_13-55-24"},
        {"label": "h=1.2", "bag": "2026-05-26_13-57-05"},
    ],
    "Pair 1: timescale + scale_xy": [
        {"label": "ts=1.0, xy=0.9", "bag": "2026-05-26_14-18-20"},
        {"label": "ts=0.8, xy=0.9", "bag": "2026-05-26_14-28-43"},
        {"label": "ts=0.6, xy=1.0", "bag": "2026-05-26_14-36-50"},
        {"label": "ts=0.6, xy=1.2", "bag": "2026-05-26_14-47-36"},
    ],
    "Pair 2: timescale + takeoff_height": [
        {"label": "ts=1.0, h=0.7", "bag": "2026-05-26_14-59-57"},
        {"label": "ts=0.8, h=1.0", "bag": "2026-05-26_15-02-15"},
        {"label": "ts=0.6, h=1.2", "bag": "2026-05-26_15-21-22"},
    ],
    "Pair 3: scale_xy + takeoff_height": [
        {"label": "xy=0.9, h=0.7", "bag": "2026-05-27_11-45-20"},
        {"label": "xy=1.0, h=1.0", "bag": "2026-05-27_12-15-01"},
        {"label": "xy=1.2, h=1.2", "bag": "2026-05-27_12-22-32"},
    ],
}


# ── Bag reading helpers ───────────────────────────────────────────────────────

def find_db3(bag_folder: Path) -> Path:
    """Find the .db3 file inside a bag folder."""
    db3_files = list(bag_folder.glob("*.db3"))
    if not db3_files:
        raise FileNotFoundError(f"No .db3 file found in {bag_folder}")
    return db3_files[0]


def read_bag(bag_name: str):
    """
    Read pose and status messages from a bag folder.
    Returns:
        times_pose  : np.array of timestamps in seconds (relative to first msg)
        x, y, z     : np.arrays of position in metres
        times_batt  : np.array of timestamps in seconds
        voltage      : np.array of battery voltage values
    """
    bag_folder = BAGS_ROOT / bag_name
    if not bag_folder.exists():
        print(f"  [WARN] Bag folder not found: {bag_folder}")
        return None

    db3_path = find_db3(bag_folder)
    conn = sqlite3.connect(str(db3_path))
    cursor = conn.cursor()

    # Get topic ids
    cursor.execute("SELECT id, name, type FROM topics")
    topics = {row[1]: (row[0], row[2]) for row in cursor.fetchall()}

    pose_id   = topics.get(POSE_TOPIC,   (None, None))[0]
    status_id = topics.get(STATUS_TOPIC, (None, None))[0]

    # Get message type classes
    try:
        PoseStamped = get_message("geometry_msgs/msg/PoseStamped")
        Status      = get_message("crazyflie_interfaces/msg/Status")
    except Exception as e:
        print(f"  [WARN] Could not load message types: {e}")
        conn.close()
        return None

    times_pose, x_vals, y_vals, z_vals = [], [], [], []
    times_batt, volt_vals = [], []

    # Read pose messages
    if pose_id is not None:
        cursor.execute(
            "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp",
            (pose_id,)
        )
        for ts, data in cursor.fetchall():
            try:
                msg = deserialize_message(bytes(data), PoseStamped)
                times_pose.append(ts * 1e-9)
                x_vals.append(msg.pose.position.x)
                y_vals.append(msg.pose.position.y)
                z_vals.append(msg.pose.position.z)
            except Exception:
                continue

    # Read status messages
    if status_id is not None:
        cursor.execute(
            "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp",
            (status_id,)
        )
        for ts, data in cursor.fetchall():
            try:
                msg = deserialize_message(bytes(data), Status)
                times_batt.append(ts * 1e-9)
                volt_vals.append(msg.battery_voltage)
            except Exception:
                continue

    conn.close()

    if not times_pose:
        print(f"  [WARN] No pose data found in {bag_name}")
        return None

    # Make times relative to first pose message
    t0 = times_pose[0]
    times_pose = np.array(times_pose) - t0
    x_vals     = np.array(x_vals)
    y_vals     = np.array(y_vals)
    z_vals     = np.array(z_vals)

    if times_batt:
        times_batt = np.array(times_batt) - t0
        volt_vals  = np.array(volt_vals)
    else:
        times_batt = np.array([])
        volt_vals  = np.array([])

    return times_pose, x_vals, y_vals, z_vals, times_batt, volt_vals


# ── Plotting ──────────────────────────────────────────────────────────────────

BASELINE_COLOR = "black"
BASELINE_STYLE = "--"

COLORS = [
    "#E63946",  # red
    "#457B9D",  # blue
    "#2A9D8F",  # teal
    "#E9C46A",  # yellow
    "#F4A261",  # orange
    "#A8DADC",  # light blue
]


def plot_group(group_name: str, runs: list, baseline_data: dict):
    """Generate a 3-subplot comparison figure for one test group."""

    fig, axes = plt.subplots(3, 1, figsize=(12, 14))
    fig.suptitle(group_name, fontsize=14, fontweight="bold", y=0.98)

    ax_z    = axes[0]
    ax_xy   = axes[1]
    ax_batt = axes[2]

    # Plot baseline on all subplots
    bd = baseline_data
    if bd is not None:
        tp, xb, yb, zb, tb, vb = bd
        ax_z.plot(tp, zb, color=BASELINE_COLOR, linestyle=BASELINE_STYLE,
                  linewidth=1.5, label=BASELINE["label"], zorder=5)
        ax_xy.plot(xb, yb, color=BASELINE_COLOR, linestyle=BASELINE_STYLE,
                   linewidth=1.5, label=BASELINE["label"], zorder=5)
        if len(tb) > 0:
            ax_batt.plot(tb, vb, color=BASELINE_COLOR, linestyle=BASELINE_STYLE,
                         linewidth=1.5, label=BASELINE["label"], zorder=5)

    # Plot each run in the group
    for idx, run in enumerate(runs):
        color = COLORS[idx % len(COLORS)]
        label = run["label"]
        data  = run.get("data")

        if data is None:
            print(f"  [SKIP] No data for {label}")
            continue

        tp, xr, yr, zr, tb, vr = data

        ax_z.plot(tp, zr, color=color, linewidth=1.5, label=label)
        ax_xy.plot(xr, yr, color=color, linewidth=1.5, label=label)
        if len(tb) > 0:
            ax_batt.plot(tb, vr, color=color, linewidth=1.5, label=label)

    # Z position subplot
    ax_z.set_xlabel("Time (s)")
    ax_z.set_ylabel("Z Position (m)")
    ax_z.set_title("Altitude (Z) Over Time")
    ax_z.legend(loc="upper right", fontsize=8)
    ax_z.grid(True, alpha=0.3)

    # XY trajectory subplot
    ax_xy.set_xlabel("X Position (m)")
    ax_xy.set_ylabel("Y Position (m)")
    ax_xy.set_title("XY Trajectory")
    ax_xy.legend(loc="upper right", fontsize=8)
    ax_xy.grid(True, alpha=0.3)
    ax_xy.set_aspect("equal", adjustable="datalim")

    # Battery voltage subplot
    ax_batt.set_xlabel("Time (s)")
    ax_batt.set_ylabel("Battery Voltage (V)")
    ax_batt.set_title("Battery Voltage Over Time")
    ax_batt.legend(loc="upper right", fontsize=8)
    ax_batt.grid(True, alpha=0.3)

    plt.tight_layout(rect=[0, 0, 1, 0.97])

    # Save
    safe_name = group_name.replace(" ", "_").replace(":", "").replace("+", "plus")
    out_path  = OUTPUT_DIR / f"{safe_name}.png"
    plt.savefig(str(out_path), dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved: {out_path.name}")


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    print(f"Output directory: {OUTPUT_DIR}\n")

    # Load baseline data once
    print(f"Loading baseline: {BASELINE['bag']}")
    baseline_data = read_bag(BASELINE["bag"])
    if baseline_data is None:
        print("[ERROR] Could not load baseline bag. Aborting.")
        return

    # Load and plot each group
    for group_name, runs in TEST_GROUPS.items():
        print(f"\nProcessing: {group_name}")

        for run in runs:
            print(f"  Loading: {run['bag']}")
            run["data"] = read_bag(run["bag"])

        plot_group(group_name, runs, baseline_data)

    print("\nAll plots complete.")


if __name__ == "__main__":
    main()
