#!/usr/bin/env python3
"""
analyze_flights.py

Analyzes all ROS2 bag files under BAGS_ROOT for cfB figure-8 MPC flights.
For each bag it produces a set of diagnostic plots saved alongside the bag:

  position_vs_time.png   — x/y/z actual (mocap) vs Kalman estimate vs figure-8 reference
  xy_trajectory.png      — bird's-eye actual path vs reference figure-8
  tracking_error.png     — per-axis and total 3-D tracking error over time
  battery_status.png     — battery voltage + supervisor state over time

Usage:
    python3 analyze_flights.py

Dependencies:
    pip install matplotlib scipy numpy
"""

import os
import math
import struct
import sqlite3
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from pathlib import Path
from scipy.interpolate import interp1d

# ── Match these exactly to the globals in figure8_mpc_demo.py ─────────────────
HOVER_HEIGHT = 0.5    # m
SCALE_X      = 0.35   # m — figure-8 half-extent in X
SCALE_Y      = 0.35   # m — figure-8 half-extent in Y
PERIOD       = 8.0    # s — one complete figure-8 lap  (T = 2pi/omega)
NUM_LAPS     = 2      # number of laps flown
MPC_N        = 30     # prediction horizon steps
MPC_TS       = 0.05   # solver sample time (s)
# ─────────────────────────────────────────────────────────────────────────────

BAGS_ROOT = os.environ.get(
    "BAGS_ROOT",
    "/home/crazyflie/Documents/Aaron2/ros2_ws/src/crazyswarm2/ros2_ws/figure8_mpc_bags",
)


# ── Lissajous reference (matches figure8_mpc_demo.py exactly) ─────────────────

def figure8_reference(t, cx=0.0, cy=0.0):
    """
    Evaluate the analytic figure-8 reference at time t.
        x(t) = cx + SCALE_X * sin(omega * t)
        y(t) = cy + SCALE_Y * sin(2 * omega * t)
        z(t) = HOVER_HEIGHT

    Returns ref_pos = [x, y, z, yaw]
    """
    omega = 2.0 * math.pi / PERIOD
    x = cx + SCALE_X * math.sin(omega * t)
    y = cy + SCALE_Y * math.sin(2.0 * omega * t)
    return [x, y, HOVER_HEIGHT, 0.0]


def build_figure8_reference(cx=0.0, cy=0.0, dt=0.02):
    """
    Return (t_ref, x_ref, y_ref, z_ref) numpy arrays for NUM_LAPS complete
    figure-8 laps, sampled at dt seconds.

    Total duration = NUM_LAPS * PERIOD  (proven: T = 2pi/omega = PERIOD for
    one clean lap where all four position+velocity closure conditions hold).
    """
    total = NUM_LAPS * PERIOD
    t_arr = np.arange(0.0, total + dt, dt)
    refs  = np.array([figure8_reference(t, cx, cy) for t in t_arr])
    return t_arr, refs[:, 0], refs[:, 1], refs[:, 2]


# ── Bag parsing ───────────────────────────────────────────────────────────────

def _read_pose_stamped(data):
    """Parse geometry_msgs/PoseStamped CDR. Returns (t_sec, x, y, z)."""
    offset = 4
    sec, nanosec = struct.unpack_from("<iI", data, offset); offset += 8
    slen = struct.unpack_from("<I", data, offset)[0]; offset += 4
    offset += slen
    if offset % 4: offset += 4 - (offset % 4)
    x, y, z = struct.unpack_from("<3d", data, offset)
    return sec + nanosec * 1e-9, x, y, z


def _read_named_pose_array(data, target_name="cfB"):
    """
    Parse motion_capture_tracking_interfaces/NamedPoseArray CDR.
    Returns (t_sec, x, y, z) for the named robot, or None if not found.
    """
    offset = 4
    sec, nanosec = struct.unpack_from("<iI", data, offset); offset += 8
    slen = struct.unpack_from("<I", data, offset)[0]; offset += 4
    offset += slen
    if offset % 4: offset += 4 - (offset % 4)
    n_poses = struct.unpack_from("<I", data, offset)[0]; offset += 4
    t_sec = sec + nanosec * 1e-9
    for _ in range(n_poses):
        slen = struct.unpack_from("<I", data, offset)[0]; offset += 4
        name = data[offset:offset + slen].rstrip(b"\x00").decode(); offset += slen
        if offset % 4: offset += 4 - (offset % 4)
        x, y, z = struct.unpack_from("<3d", data, offset); offset += 24
        offset += 32  # skip quaternion
        if name == target_name:
            return t_sec, x, y, z
    return None


def _read_status(data):
    """
    Parse crazyflie_interfaces/Status CDR.
    Returns (t_sec, battery_voltage, supervisor_info).
    """
    offset = 4
    sec, nanosec = struct.unpack_from("<iI", data, offset); offset += 8
    slen = struct.unpack_from("<I", data, offset)[0]; offset += 4
    offset += slen + (4 - (offset + slen) % 4) % 4
    supervisor_info = struct.unpack_from("<H", data, offset)[0]; offset += 2
    offset += 2
    battery = struct.unpack_from("<f", data, offset)[0]
    return sec + nanosec * 1e-9, battery, supervisor_info


def parse_bag(db3_path):
    """Read a .db3 bag and return signal dicts with zero-based timestamps."""
    conn = sqlite3.connect(db3_path)
    cur  = conn.cursor()
    cur.execute("SELECT id, name FROM topics")
    topic_ids = {name: tid for tid, name in cur.fetchall()}

    def fetch(topic):
        tid = topic_ids.get(topic)
        if tid is None:
            return []
        cur.execute("SELECT data FROM messages WHERE topic_id=? ORDER BY rowid", (tid,))
        return [row[0] for row in cur.fetchall()]

    kalman = {"t": [], "x": [], "y": [], "z": []}
    for raw in fetch("/cfB/pose"):
        try:
            t, x, y, z = _read_pose_stamped(raw)
            kalman["t"].append(t); kalman["x"].append(x)
            kalman["y"].append(y); kalman["z"].append(z)
        except Exception:
            pass

    mocap = {"t": [], "x": [], "y": [], "z": []}
    for raw in fetch("/poses"):
        result = _read_named_pose_array(raw)
        if result:
            t, x, y, z = result
            mocap["t"].append(t); mocap["x"].append(x)
            mocap["y"].append(y); mocap["z"].append(z)

    status = {"t": [], "battery": [], "supervisor": []}
    for raw in fetch("/cfB/status"):
        try:
            t, bat, sup = _read_status(raw)
            status["t"].append(t)
            status["battery"].append(bat)
            status["supervisor"].append(sup)
        except Exception:
            pass

    conn.close()

    for d in (kalman, mocap, status):
        for k in d:
            d[k] = np.array(d[k])

    if len(mocap["t"]) and len(kalman["t"]):
        t0 = min(mocap["t"][0], kalman["t"][0])
    elif len(kalman["t"]):
        t0 = kalman["t"][0]
    elif len(mocap["t"]):
        t0 = mocap["t"][0]
    else:
        t0 = 0.0

    for d in (kalman, mocap, status):
        if len(d["t"]):
            d["t"] = d["t"] - t0

    return kalman, mocap, status, t0


# ── Plot helpers ──────────────────────────────────────────────────────────────

COLORS = {
    "mocap":   "#1f77b4",
    "kalman":  "#ff7f0e",
    "ref":     "#2ca02c",
    "error":   "#d62728",
    "error_x": "#9467bd",
    "error_y": "#8c564b",
    "error_z": "#e377c2",
    "battery": "#17becf",
}


def plot_position_vs_time(kalman, mocap, ref_t, ref_x, ref_y, ref_z, out_path):
    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    labels   = ["X (m)", "Y (m)", "Z (m)"]
    act_keys = ["x", "y", "z"]
    ref_arrs = [ref_x, ref_y, ref_z]

    for ax, label, key, ref_arr in zip(axes, labels, act_keys, ref_arrs):
        if len(mocap["t"]):
            ax.plot(mocap["t"], mocap[key], color=COLORS["mocap"],
                    alpha=0.6, lw=1, label="Mocap (raw)")
        if len(kalman["t"]):
            ax.plot(kalman["t"], kalman[key], color=COLORS["kalman"],
                    lw=1.5, label="Kalman estimate")
        ax.plot(ref_t, ref_arr, "--", color=COLORS["ref"],
                lw=1.5, label="Lissajous reference")
        ax.set_ylabel(label)
        ax.legend(loc="upper right", fontsize=8)
        ax.grid(True, alpha=0.3)

    axes[-1].set_xlabel("Time (s)")
    axes[0].set_title(
        f"Position vs Time — cfB Figure-8  "
        f"(PERIOD={PERIOD}s, SCALE_X={SCALE_X}m, SCALE_Y={SCALE_Y}m, {NUM_LAPS} lap(s))"
    )
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"  Saved: {out_path}")


def plot_xy_trajectory(mocap, ref_x, ref_y, out_path):
    fig, ax = plt.subplots(figsize=(7, 7))
    if len(mocap["x"]):
        ax.plot(mocap["x"], mocap["y"], color=COLORS["mocap"],
                lw=1.5, label="Actual (mocap)", alpha=0.8)
        ax.scatter(mocap["x"][0], mocap["y"][0], color=COLORS["mocap"],
                   s=80, zorder=5, marker="o", label="Start")
        ax.scatter(mocap["x"][-1], mocap["y"][-1], color=COLORS["mocap"],
                   s=80, zorder=5, marker="x", label="End")
    ax.plot(ref_x, ref_y, "--", color=COLORS["ref"],
            lw=2, label=f"Lissajous reference ({NUM_LAPS} lap(s))")
    ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)")
    ax.set_title(
        f"XY Trajectory — cfB Figure-8  "
        f"(SCALE_X={SCALE_X}m, SCALE_Y={SCALE_Y}m, PERIOD={PERIOD}s)"
    )
    ax.legend(); ax.grid(True, alpha=0.3)
    ax.set_aspect("equal")
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"  Saved: {out_path}")


def plot_tracking_error(mocap, ref_t, ref_x, ref_y, ref_z, out_path):
    if not len(mocap["t"]):
        print("  Skipping error plot — no mocap data")
        return

    t_eval = mocap["t"]
    mask = (t_eval >= ref_t[0]) & (t_eval <= ref_t[-1])
    if mask.sum() < 2:
        print("  Skipping error plot — mocap timestamps outside reference range")
        return

    t_m = t_eval[mask]
    interp_x = interp1d(ref_t, ref_x, bounds_error=False, fill_value="extrapolate")
    interp_y = interp1d(ref_t, ref_y, bounds_error=False, fill_value="extrapolate")
    interp_z = interp1d(ref_t, ref_z, bounds_error=False, fill_value="extrapolate")

    ex  = mocap["x"][mask] - interp_x(t_m)
    ey  = mocap["y"][mask] - interp_y(t_m)
    ez  = mocap["z"][mask] - interp_z(t_m)
    e3d = np.sqrt(ex**2 + ey**2 + ez**2)

    fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True)

    axes[0].plot(t_m, ex * 100, color=COLORS["error_x"], lw=1.2, label="ex")
    axes[0].plot(t_m, ey * 100, color=COLORS["error_y"], lw=1.2, label="ey")
    axes[0].plot(t_m, ez * 100, color=COLORS["error_z"], lw=1.2, label="ez")
    axes[0].axhline(0, color="k", lw=0.5, ls="--")
    axes[0].set_ylabel("Error (cm)")
    axes[0].set_title("Per-axis Tracking Error (Mocap − Lissajous Reference)")
    axes[0].legend(); axes[0].grid(True, alpha=0.3)

    rms  = float(np.sqrt(np.mean(e3d**2)))
    peak = float(np.max(e3d))
    axes[1].plot(t_m, e3d * 100, color=COLORS["error"], lw=1.5, label="3-D error")
    axes[1].axhline(rms * 100, color="k", lw=1, ls="--",
                    label=f"RMS = {rms*100:.1f} cm")
    axes[1].set_ylabel("3-D Error (cm)")
    axes[1].set_xlabel("Time (s)")
    axes[1].set_title(
        f"Total 3-D Tracking Error   |   RMS={rms*100:.1f} cm   Peak={peak*100:.1f} cm"
    )
    axes[1].legend(); axes[1].grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"  Saved: {out_path}  (RMS={rms*100:.1f} cm  Peak={peak*100:.1f} cm)")


def plot_battery_status(status, out_path):
    if not len(status["t"]):
        print("  Skipping battery plot — no status data")
        return

    fig, ax1 = plt.subplots(figsize=(10, 4))
    ax1.plot(status["t"], status["battery"], color=COLORS["battery"],
             lw=2, marker="o", ms=4, label="Battery (V)")
    ax1.set_ylabel("Voltage (V)", color=COLORS["battery"])
    ax1.tick_params(axis="y", labelcolor=COLORS["battery"])
    ax1.set_xlabel("Time (s)")
    ax1.set_title("Battery Voltage & Supervisor State")
    ax1.grid(True, alpha=0.3)

    ax2 = ax1.twinx()
    ax2.step(status["t"], status["supervisor"], where="post",
             color="#ff7f0e", lw=1.5, alpha=0.7, label="Supervisor info")
    ax2.set_ylabel("Supervisor info (bitmask)", color="#ff7f0e")
    ax2.tick_params(axis="y", labelcolor="#ff7f0e")

    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labels1 + labels2, loc="upper right", fontsize=8)

    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"  Saved: {out_path}")


# ── Bag discovery ─────────────────────────────────────────────────────────────

def find_bags(root):
    bags = []
    root_path = Path(root)
    if not root_path.exists():
        print(f"[WARN] BAGS_ROOT does not exist: {root}")
        return bags
    for entry in sorted(root_path.iterdir()):
        if not entry.is_dir():
            continue
        db3_files = list(entry.glob("*.db3"))
        if db3_files and not (entry / "xy_trajectory.png").exists():
            bags.append((entry, db3_files[0]))
    return bags


# ── Main ──────────────────────────────────────────────────────────────────────

def analyze_bag(bag_dir, db3_path):
    print(f"\n{'='*60}")
    print(f"Bag: {bag_dir.name}")
    print(f"File: {db3_path}")

    kalman, mocap, status, t0 = parse_bag(str(db3_path))

    print(f"  Mocap samples  : {len(mocap['t'])}")
    print(f"  Kalman samples : {len(kalman['t'])}")
    print(f"  Status samples : {len(status['t'])}")
    if len(mocap["t"]):
        print(f"  Flight duration: {mocap['t'][-1] - mocap['t'][0]:.1f} s")
    if len(status["battery"]):
        bmin, bmax = status["battery"].min(), status["battery"].max()
        print(f"  Battery: {bmax:.2f}V → {bmin:.2f}V  (drop={bmax-bmin:.2f}V)")

    # Centre the reference on the drone's actual starting position
    if len(mocap["x"]):
        cx, cy = float(mocap["x"][0]), float(mocap["y"][0])
    else:
        cx, cy = 0.0, 0.0
    print(f"  Reference centred on: cx={cx:.3f}, cy={cy:.3f}")

    ref_t, ref_x, ref_y, ref_z = build_figure8_reference(cx=cx, cy=cy)

    out = bag_dir
    plot_position_vs_time(kalman, mocap, ref_t, ref_x, ref_y, ref_z,
                          out / "position_vs_time.png")
    plot_xy_trajectory(mocap, ref_x, ref_y,
                       out / "xy_trajectory.png")
    plot_tracking_error(mocap, ref_t, ref_x, ref_y, ref_z,
                        out / "tracking_error.png")
    plot_battery_status(status,
                        out / "battery_status.png")


def main():
    print(f"Flight parameters:")
    print(f"  PERIOD={PERIOD}s  SCALE_X={SCALE_X}m  SCALE_Y={SCALE_Y}m")
    print(f"  NUM_LAPS={NUM_LAPS}  total_duration={NUM_LAPS * PERIOD}s")
    print(f"  omega={2*math.pi/PERIOD:.4f} rad/s  T_lap=PERIOD={PERIOD}s")

    bags = find_bags(BAGS_ROOT)
    if not bags:
        print(f"No bag directories found under: {BAGS_ROOT}")
        return

    print(f"\nFound {len(bags)} bag(s) to analyze.")
    for bag_dir, db3_path in bags:
        analyze_bag(bag_dir, db3_path)

    print(f"\nDone. Plots saved alongside each bag in:\n  {BAGS_ROOT}")


if __name__ == "__main__":
    main()