#!/bin/bash
# Fly cfB in a figure-8. Motive must be streaming before running this.
# Automatically cleans up old ROS processes before each run.
# Bag recordings are saved to /home/crazyflie/Documents/Aaron2/ros2_ws/src/crazyswarm2/ros2_ws/figure8_bags/<timestamp>/

# ── Cleanup old processes ──────────────────────────────────────
echo "Cleaning up old ROS processes..."
pkill -f crazyflie_server
pkill -f cflib
pkill -f motion_capture_tracking_node
pkill -f poses_relay
pkill -f figure8_demo
pkill -f ros2_bag
sleep 3
echo "Cleanup done. Starting fresh..."
# ──────────────────────────────────────────────────────────────

source /opt/ros/humble/setup.bash
source ~/Documents/Aaron2/ros2_ws/src/crazyswarm2/ros2_ws/install/setup.bash

# ── Bag output directory ───────────────────────────────────────
BAGS_ROOT=/home/crazyflie/Documents/Aaron2/ros2_ws/src/crazyswarm2/ros2_ws/figure8_bags
BAG_DIR=$BAGS_ROOT/$(date +%Y-%m-%d_%H-%M-%S)
mkdir -p "$BAGS_ROOT"
echo "Bag recording will be saved to: $BAG_DIR"
# ──────────────────────────────────────────────────────────────

# ── Start bag recording in the background ─────────────────────
# Topics recorded:
#   /cfB/pose       - live position and orientation at 10 Hz (from firmware logging)
#   /cfB/status     - battery voltage and pm_state at 1 Hz
#   /poses          - raw mocap pose stream
#   /figure8_path   - the planned trajectory marker (useful for post-flight path comparison)
ros2 bag record \
    /cfB/pose \
    /cfB/status \
    /poses \
    /figure8_path \
    -o "$BAG_DIR" &
BAG_PID=$!
echo "Bag recorder started (PID $BAG_PID)"
sleep 1  # Give the recorder a moment to initialize before the flight starts
# ──────────────────────────────────────────────────────────────

# ── Run the flight ────────────────────────────────────────────
ros2 launch crazyflie_examples figure8_demo_launch.py \
    backend:=cpp \
    mocap:=True \
    cf_name:=cfB \
    takeoff_height:=1.2 \
    timescale:=1.5\
    scale_xy:=1.2 \
    rviz:=False
# ──────────────────────────────────────────────────────────────

# ── Stop bag recording after the flight ends ──────────────────
echo "Flight complete. Stopping bag recorder..."
kill $BAG_PID
wait $BAG_PID 2>/dev/null
echo "Bag saved to: $BAG_DIR"
# ──────────────────────────────────────────────────────────────