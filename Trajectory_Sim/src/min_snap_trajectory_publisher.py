import json
import numpy as np
from scipy.interpolate import CubicSpline

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


class MinSnapTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('min_snap_trajectory_publisher')

        # Publisher for trajectory path
        self.publisher_ = self.create_publisher(Path, '/trajectory_path', 10)

        # Timer to publish periodically
        self.timer = self.create_timer(2.0, self.publish_trajectory)

        self.get_logger().info('MinSnapTrajectoryPublisher node initialized')

    def compute_min_snap(self, waypoints, dt=0.1):
        """
        Compute a smooth trajectory using cubic splines (approx min-snap).
        Returns a list of interpolated points.
        """
        if len(waypoints) < 2:
            return waypoints  # Not enough points to interpolate

        # Extract coordinates
        x = [wp['x'] for wp in waypoints]
        y = [wp['y'] for wp in waypoints]
        z = [wp['z'] for wp in waypoints]

        # Parameter t along the path
        t = np.linspace(0, len(waypoints)-1, len(waypoints))

        # Cubic splines (C2 continuous = smooth position, velocity, and approx min-snap)
        cs_x = CubicSpline(t, x)
        cs_y = CubicSpline(t, y)
        cs_z = CubicSpline(t, z)

        # Interpolated points
        t_fine = np.arange(0, t[-1], dt)
        trajectory = [{'x': float(cs_x(ti)),
                       'y': float(cs_y(ti)),
                       'z': float(cs_z(ti))}
                      for ti in t_fine]

        return trajectory

    def publish_trajectory(self):
        try:
            with open('solution.json', 'r') as f:
                data = json.load(f)

            drone = data['drones'][0]  # first drone only
            waypoints = drone['waypoints']

            smooth_path = self.compute_min_snap(waypoints)

            path_msg = Path()
            path_msg.header.frame_id = 'map'

            for pt in smooth_path:
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = pt['x']
                pose.pose.position.y = pt['y']
                pose.pose.position.z = pt['z']
                path_msg.poses.append(pose)

            self.publisher_.publish(path_msg)
            self.get_logger().info(
                f"Published min-snap trajectory with {len(path_msg.poses)} points for drone {drone.get('id', 0)}"
            )

        except Exception as e:
            self.get_logger().error(f"Failed to publish trajectory: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = MinSnapTrajectoryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
