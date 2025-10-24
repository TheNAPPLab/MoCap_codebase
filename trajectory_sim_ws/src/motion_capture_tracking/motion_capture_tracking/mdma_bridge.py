import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from julia import Main
import os


class MDMABridge(Node):
    def __init__(self):
        super().__init__('MDMA_bridge')

        # Path to your Julia MDMA project
        project_path = "/home/timothy/MultiDroneMultiActorFilming"  # update if needed
        os.chdir(project_path)

        # Initialize Julia environment
        self.get_logger().info("Initializing Julia and loading MDMA...")
        Main.eval('using MDMA')

        # ROS 2 communication setup
        self.subscription = self.create_subscription(
            String,
            '/experiment_request',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(String, '/waypoints', 10)

        self.get_logger().info("MDMA_bridge node ready for experiment requests.")

    def listener_callback(self, msg):
        """Runs the requested Julia experiment using PyCall."""
        experiment = msg.data.strip()
        self.get_logger().info(f"Received request: '{experiment}'")

        try:
            if experiment == "all":
                Main.eval('conf = ExperimentsConfig("./experiments")')
                Main.eval('run_all_experiments(conf)')
            else:
                Main.eval(f'conf = ExperimentsConfig("./experiments", ["{experiment}"])')
                Main.eval('run_all_experiments(conf)')

            result = f"Experiment '{experiment}' completed successfully."
            self.get_logger().info(result)
            self.publisher.publish(String(data=result))

        except Exception as e:
            error_msg = f"Julia experiment '{experiment}' failed: {e}"
            self.get_logger().error(error_msg)
            self.publisher.publish(String(data=error_msg))


def main(args=None):
    rclpy.init(args=args)
    node = MDMABridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
