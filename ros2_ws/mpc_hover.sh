#!/bin/bash
# Fly cfB in an MPC hover. Motive must be streaming before running this.
# Automatically cleans up old ROS processes before each run.
# Records a ROS2 bag for post-flight analysis.

# ── Cleanup old processes ──────────────────────────────────────
echo "Cleaning up old ROS processes..."
pkill -f crazyflie_server
pkill -f cflib
pkill -f motion_capture_tracking_node
pkill -f poses_relay
pkill -f "python3 mpc_hover.py"
pkill -f ros2_bag
sleep 3
echo "Cleanup done. Starting fresh..."
# ──────────────────────────────────────────────────────────────

source /opt/ros/humble/setup.bash
source ~/Documents/Aaron2/ros2_ws/src/crazyswarm2/ros2_ws/install/setup.bash
source ~/miniconda3/etc/profile.d/conda.sh
conda activate aaron2_env

MPC_DIR=/home/crazyflie/Documents/Aaron2/ros2_ws/src/crazyswarm2/ros2_ws/src/crazyswarm2/crazyflie_examples/crazyflie_examples/mpc
BAG_DIR=$MPC_DIR/mpc_hover_bags
mkdir -p $BAG_DIR

TIMESTAMP=$(date +"%Y-%m-%d_%H-%M-%S")
BAG_PATH=$BAG_DIR/$TIMESTAMP

# ── Start crazyflie stack in background ───────────────────────
echo "Starting crazyflie stack..."
ros2 launch crazyflie launch.py \
    backend:=cpp \
    mocap:=True \
    gui:=False \
    teleop:=False &
STACK_PID=$!
echo "Crazyflie stack started (PID $STACK_PID)"
echo "Waiting 8 seconds for stack to initialize and Kalman to converge..."
sleep 8
# ──────────────────────────────────────────────────────────────

# ── Start bag recording ───────────────────────────────────────
echo "Starting bag recording to $BAG_PATH ..."
ros2 bag record \
    /cfB/pose \
    /cfB/status \
    /poses \
    -o $BAG_PATH &
BAG_PID=$!
echo "Bag recording started (PID $BAG_PID)"
sleep 1
# ──────────────────────────────────────────────────────────────

# ── Run the MPC hover script ───────────────────────────────────
echo "Starting MPC hover..."
cd $MPC_DIR
python3 mpc_hover.py
# ──────────────────────────────────────────────────────────────

# ── Stop bag recording and shutdown stack ─────────────────────
echo "Hover complete. Stopping bag recording..."
kill $BAG_PID
wait $BAG_PID 2>/dev/null
echo "Bag saved to: $BAG_PATH"

echo "Shutting down crazyflie stack..."
kill $STACK_PID
wait $STACK_PID 2>/dev/null
echo "Done."
# ──────────────────────────────────────────────────────────────