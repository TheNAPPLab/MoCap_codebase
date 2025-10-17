# Ubuntu 22.04 + ROS 2 Humble base (ARM64 works on M2)
FROM ros:humble

# Avoid interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

# Core tools + rosdep + colcon + Gazebo bridge + rosbridge + useful ROS pkgs
RUN apt-get update && apt-get install -y --no-install-recommends \
    git build-essential cmake ninja-build pkg-config \
    python3-pip python3-venv python3-colcon-common-extensions python3-vcstool \
    python3-rosdep \
    ros-humble-ros-gz ros-humble-ros-gz-bridge \
    ros-humble-rosbridge-server \
    ros-humble-tf2-ros ros-humble-tf2-tools \
    ros-humble-geometry-msgs ros-humble-nav-msgs ros-humble-trajectory-msgs \
    && rm -rf /var/lib/apt/lists/*

# Python libs for smoothing + timing
RUN pip3 install --no-cache-dir --upgrade pip && \
    pip3 install --no-cache-dir numpy scipy matplotlib toppra

# Initialize rosdep (safe if already done)
RUN rosdep init || true
RUN rosdep update

# Default workdir where we’ll mount your Mac folder
WORKDIR /ws

# Keep the standard ROS entrypoint
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
