import json
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')

        # Publisher for trajectory path
        self.publisher_ = self.create_publisher(Path, '/trajectory_path', 10)

        # Timer to publish periodically
        self.timer = self.create_timer(2.0, self.publish_path)

        self.get_logger().info('TrajectoryPublisher node initialized')

    def publish_path(self):
        try:
            with open('solution.json', 'r') as f:
                data = json.load(f)

            # Use only the first drone in the JSON
            drone = data['drones'][0]
            path = Path()
            path.header.frame_id = 'map'

            for wp in drone['waypoints']:
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = wp.get('x', 0.0)
                pose.pose.position.y = wp.get('y', 0.0)
                pose.pose.position.z = wp.get('z', 0.0)
                path.poses.append(pose)

            self.publisher_.publish(path)
            self.get_logger().info(
                f"Published {len(path.poses)} waypoints for drone {drone.get('id', 0)}"
            )

        except Exception as e:
            self.get_logger().error(f"Failed to publish trajectory: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
