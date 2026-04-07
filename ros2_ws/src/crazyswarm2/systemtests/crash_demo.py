from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def smoothstep(x: np.ndarray) -> np.ndarray:
    x = np.clip(x, 0.0, 1.0)
    return x * x * (3.0 - 2.0 * x)


def generate_profile(dt: float = 0.05) -> tuple[np.ndarray, np.ndarray, float, float]:
    takeoff_time = 1.1
    crash_time = 6.6
    end_time = 9.2
    time = np.arange(0.0, end_time + dt, dt)

    x = np.zeros_like(time)
    y = np.zeros_like(time)
    z = np.zeros_like(time)

    climb = smoothstep((time - takeoff_time) / 1.2)
    translate = smoothstep((time - 2.0) / 2.2)
    instability = smoothstep((time - 5.3) / 0.9)
    crash = smoothstep((time - crash_time) / 0.85)

    x += 0.03 * climb + 0.42 * translate
    y += 0.05 * np.sin(1.8 * (time - 2.0)) * np.clip(translate, 0.0, 1.0)
    z += 0.42 * climb

    x += 0.025 * instability * np.sin(9.0 * time)
    y += 0.055 * instability * np.sin(10.5 * time + 0.4)
    z += 0.018 * np.sin(4.2 * time) * np.clip(translate, 0.0, 1.0)
    z -= 0.12 * instability

    x += 0.08 * crash
    y += -0.12 * crash
    z -= 0.56 * crash

    post_crash = time > (crash_time + 0.85)
    impact_idx = np.where(time <= (crash_time + 0.85))[0][-1]
    x[post_crash] = x[impact_idx] + 0.01 * np.exp(
        -(time[post_crash] - (crash_time + 0.85)) * 3.0
    )
    y[post_crash] = y[impact_idx] - 0.008 * np.exp(
        -(time[post_crash] - (crash_time + 0.85)) * 3.0
    )
    z[post_crash] = -0.05

    ground = time < takeoff_time
    x[ground] = 0.0
    y[ground] = 0.0
    z[ground] = 0.0

    z = np.maximum(z, -0.1)

    return time, np.column_stack((x, y, z)), takeoff_time, crash_time


def write_rosbag_style_csv(
    csv_path: Path,
    time: np.ndarray,
    position: np.ndarray,
    takeoff_time: float,
) -> None:
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    with csv_path.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["# t", " x", " y", " z"])
        for t, (x, y, z) in zip(time, position):
            writer.writerow([f"{t:.3f}", f"{x:.4f}", f"{y:.4f}", f"{z:.4f}"])
        writer.writerow([f"### takeoff time : {takeoff_time:.3f}"])


def plot_flight(csv_path: Path, plot_path: Path, takeoff_time: float, crash_time: float) -> None:
    data = np.loadtxt(csv_path, delimiter=",")
    time = data[:, 0]
    x = data[:, 1]
    y = data[:, 2]
    z = data[:, 3]

    dt = np.diff(time, prepend=time[0])
    dx = np.diff(x, prepend=x[0])
    dy = np.diff(y, prepend=y[0])
    dz = np.diff(z, prepend=z[0])
    speed = np.sqrt(dx * dx + dy * dy + dz * dz) / np.clip(dt, 1e-6, None)

    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    fig.suptitle("Drone Flight Log: Takeoff Then Crash")

    altitude_ax = axes[0, 0]
    altitude_ax.plot(time, z, color="#1f77b4", linewidth=2)
    altitude_ax.set_title("Altitude")
    altitude_ax.set_xlabel("Time [s]")
    altitude_ax.set_ylabel("Z [m]")
    altitude_ax.grid(True, alpha=0.3)

    xy_ax = axes[0, 1]
    xy_ax.plot(x, y, color="#ff7f0e", linewidth=2)
    xy_ax.set_title("XY Path")
    xy_ax.set_xlabel("X [m]")
    xy_ax.set_ylabel("Y [m]")
    xy_ax.axis("equal")
    xy_ax.grid(True, alpha=0.3)

    position_ax = axes[1, 0]
    position_ax.plot(time, x, label="x", linewidth=2)
    position_ax.plot(time, y, label="y", linewidth=2)
    position_ax.plot(time, z, label="z", linewidth=2)
    position_ax.set_title("Position Components")
    position_ax.set_xlabel("Time [s]")
    position_ax.set_ylabel("Position [m]")
    position_ax.grid(True, alpha=0.3)
    position_ax.legend()

    speed_ax = axes[1, 1]
    speed_ax.plot(time, speed, color="#9467bd", linewidth=2)
    speed_ax.set_title("Approximate Speed")
    speed_ax.set_xlabel("Time [s]")
    speed_ax.set_ylabel("Speed [m/s]")
    speed_ax.grid(True, alpha=0.3)

    fig.tight_layout()
    plot_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(plot_path, dpi=200, bbox_inches="tight")
    plt.close(fig)


def plot_3d_path(csv_path: Path, plot_path: Path, takeoff_time: float, crash_time: float) -> None:
    data = np.loadtxt(csv_path, delimiter=",")
    time = data[:, 0]
    x = data[:, 1]
    y = data[:, 2]
    z = data[:, 3]

    fig = plt.figure(figsize=(9, 7))
    ax = fig.add_subplot(111, projection="3d")

    points = ax.scatter(x, y, z, c=time, cmap="viridis", s=18)
    ax.plot(x, y, z, color="#1f77b4", linewidth=1.5, alpha=0.75)

    ax.set_title("Drone Flight Path in 3D")
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")
    ax.view_init(elev=24, azim=-58)
    ax.grid(True, alpha=0.3)
    ax.set_box_aspect((np.ptp(x) + 0.2, np.ptp(y) + 0.2, np.ptp(z) + 0.2))
    fig.colorbar(points, ax=ax, pad=0.08, shrink=0.75, label="Time [s]")

    plot_path.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(plot_path, dpi=200, bbox_inches="tight")
    plt.close(fig)


def main() -> None:
    script_dir = Path(__file__).resolve().parent
    ros2_ws_dir = script_dir.parents[2]

    parser = argparse.ArgumentParser(
        description="Generate a rosbag-style CSV and matplotlib plots for a crash flight."
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=ros2_ws_dir / "results" / "crash_demo",
        help="Directory for the generated CSV and plot.",
    )
    args = parser.parse_args()

    output_dir = args.output_dir.resolve()
    csv_path = output_dir / "takeoff_crash.csv"
    plot_path = output_dir / "takeoff_crash.png"
    plot_3d_path_file = output_dir / "takeoff_crash_3d.png"

    time, position, takeoff_time, crash_time = generate_profile()
    write_rosbag_style_csv(csv_path, time, position, takeoff_time)
    plot_flight(csv_path, plot_path, takeoff_time, crash_time)
    plot_3d_path(csv_path, plot_3d_path_file, takeoff_time, crash_time)

    print(f"Wrote flight log: {csv_path}")
    print(f"Wrote visualization: {plot_path}")
    print(f"Wrote 3D visualization: {plot_3d_path_file}")


if __name__ == "__main__":
    main()
