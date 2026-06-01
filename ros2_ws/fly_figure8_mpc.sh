#!/bin/bash
# fly_figure8_mpc.sh
#
# Fly cfB in a figure-8 using the MPC controller.
# Motive must be streaming NatNet before running this.
# Automatically cleans up old ROS processes before each run.
# Bag recordings are saved to figure8_mpc_bags/<timestamp>/
#
# Usage:
#   bash fly_figure8_mpc.sh

# ── Cleanup old processes ──────────────────────────────────────
echo "Cleaning up old ROS processes..."
pkill -f crazyflie_server
pkill -f cflib
pkill -f motion_capture_tracking_node
pkill -f poses_relay
pkill -f figure8_mpc_demo
pkill -f ros2_bag
sleep 3
echo "Cleanup done. Starting fresh..."
# ──────────────────────────────────────────────────────────────

source /opt/ros/humble/setup.bash
source ~/Documents/Aaron2/ros2_ws/src/crazyswarm2/ros2_ws/install/setup.bash

# ── Bag output directory ───────────────────────────────────────
BAGS_ROOT=/home/crazyflie/Documents/Aaron2/ros2_ws/src/crazyswarm2/ros2_ws/figure8_mpc_bags
BAG_DIR=$BAGS_ROOT/$(date +%Y-%m-%d_%H-%M-%S)
mkdir -p "$BAGS_ROOT"
echo "Bag recording will be saved to: $BAG_DIR"
# ──────────────────────────────────────────────────────────────

# ── Start bag recording in the background ─────────────────────
# Topics recorded:
#   /cfB/pose     — live position and orientation at 10 Hz (firmware logging)
#   /cfB/status   — battery voltage and pm_state at 1 Hz
#   /poses        — raw mocap pose stream
ros2 bag record \
    /cfB/pose \
    /cfB/status \
    /poses \
    -o "$BAG_DIR" &
BAG_PID=$!
echo "Bag recorder started (PID $BAG_PID)"
sleep 1
# ──────────────────────────────────────────────────────────────

# ── Run the flight ─────────────────────────────────────────────
# figure8_mpc_demo_launch.py starts the crazyflie stack + mocap +
# the figure8_mpc_demo node all in one process.
# This call blocks until the flight node exits (on_exit=Shutdown()).
ros2 launch crazyflie_examples figure8_mpc_demo_launch.py \
    backend:=cpp \
    mocap:=True \
    cf_name:=cfB \
    rviz:=False
# ──────────────────────────────────────────────────────────────

# ── Stop bag recording after flight completes ──────────────────
echo "Flight complete. Stopping bag recorder..."
kill $BAG_PID
wait $BAG_PID 2>/dev/null
echo "Bag saved to: $BAG_DIR"
# ──────────────────────────────────────────────────────────────