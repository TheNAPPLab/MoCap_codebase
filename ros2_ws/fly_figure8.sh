#!/bin/bash
# Fly cfB in a figure-8. Motive must be streaming before running this.
# Motive must be streaming before running this.
# Automatically cleans up old ROS processes before each run.
 
# ── Cleanup old processes ──────────────────────────────────────
echo "Cleaning up old ROS processes..."
pkill -f crazyflie_server
pkill -f cflib
pkill -f motion_capture_tracking_node
pkill -f poses_relay
pkill -f figure8_demo
sleep 3
echo "Cleanup done. Starting fresh..."
# ─────────────────────────────────────────────────────────────


source /opt/ros/humble/setup.bash
source ~/Documents/Aaron2/ros2_ws/src/crazyswarm2/ros2_ws/install/setup.bash



exec ros2 launch crazyflie_examples figure8_demo_launch.py \
    backend:=cpp \
    mocap:=True \
    cf_name:=cfB \
    takeoff_height:=0.5 \
    timescale:=1.5 \
    scale_xy:=0.7 \
    rviz:=False
