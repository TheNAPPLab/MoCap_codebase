#!/usr/bin/env python3
"""
Relay /poses_raw → /poses, stripping orientation (forces position-only extPos
to crazyflie_server). Publishes with deadline=10ms (100 Hz) to match the
server's QoS expectation. Throttles to 100 Hz max.
"""
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles, QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.duration import Duration
from motion_capture_tracking_interfaces.msg import NamedPoseArray

MAX_HZ = 100.0
MIN_PERIOD = 1.0 / MAX_HZ

class PosesRelay(Node):
    def __init__(self):
        super().__init__('poses_relay')
        self._last_pub = 0.0

        sub_qos = QoSPresetProfiles.SENSOR_DATA.value

        # Publisher QoS must match server's subscription: SENSOR_DATA + deadline=10ms
        pub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            deadline=Duration(nanoseconds=int(1e7)),  # 10ms = 100 Hz
        )

        self._pub = self.create_publisher(NamedPoseArray, 'poses', pub_qos)
        self._sub = self.create_subscription(
            NamedPoseArray, 'poses_raw', self._cb, sub_qos)

    def _cb(self, msg):
        now = time.monotonic()
        if now - self._last_pub < MIN_PERIOD:
            return
        self._last_pub = now
        nan = float('nan')
        for p in msg.poses:
            p.pose.orientation.x = nan
            p.pose.orientation.y = nan
            p.pose.orientation.z = nan
            p.pose.orientation.w = nan
        self._pub.publish(msg)

def main():
    rclpy.init()
    rclpy.spin(PosesRelay())

if __name__ == '__main__':
    main()
