"""Fly a single Crazyflie through a figure-8 and publish the path for RViz."""

from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from motion_capture_tracking_interfaces.msg import NamedPoseArray
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from visualization_msgs.msg import Marker

from crazyflie_py import Crazyswarm
from crazyflie_py.uav_trajectory import Trajectory
from crazyflie_interfaces.msg import Status


TRAJECTORY_ID = 0
TAKEOFF_DURATION = 2.5
POSITIONING_DURATION = 2.0
LAND_DURATION = 2.5
LAND_HEIGHT = 0.04
PATH_SAMPLES = 200
SUPERVISOR_INFO_CAN_BE_ARMED = 1
SUPERVISOR_INFO_IS_ARMED = 2
SUPERVISOR_INFO_CAN_FLY = 8


def _scale_trajectory_xy(traj, scale_xy):
    for poly in traj.polynomials:
        poly.px.p = np.array(poly.px.p) * scale_xy
        poly.py.p = np.array(poly.py.p) * scale_xy


def _build_path_marker(traj, offset):
    marker = Marker()
    marker.header.frame_id = "world"
    marker.ns = "figure8_demo"
    marker.id = 0
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.scale.x = 0.03
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 0.45
    marker.color.b = 0.1

    for sample_idx in range(PATH_SAMPLES + 1):
        t = traj.duration * sample_idx / PATH_SAMPLES
        state = traj.eval(t)
        point = Point()
        point.x = float(state.pos[0] + offset[0])
        point.y = float(state.pos[1] + offset[1])
        point.z = float(state.pos[2] + offset[2])
        marker.points.append(point)

    return marker


def _wait_for_supervisor_bits(server, cf, cf_name, required_bits, timeout_sec, description):
    start_time = server.get_clock().now()
    timeout_duration = Duration(seconds=timeout_sec)

    while rclpy.ok():
        supervisor = int(cf.get_status().get("supervisor", 0))
        if (supervisor & required_bits) == required_bits:
            return supervisor
        if (server.get_clock().now() - start_time) > timeout_duration:
            raise RuntimeError(
                f"{cf_name} did not reach {description} within {timeout_sec:.1f} s "
                f"(last supervisor_info={supervisor})."
            )
        rclpy.spin_once(server, timeout_sec=0.1)


def main():
    swarm = Crazyswarm()
    time_helper = swarm.timeHelper
    server = swarm.allcfs

    server.declare_parameter("cf_name", "")
    server.declare_parameter("takeoff_height", 0.8)
    server.declare_parameter("timescale", 1.5)
    server.declare_parameter("scale_xy", 0.7)
    server.declare_parameter("require_mocap_lock", False)
    server.declare_parameter("mocap_timeout", 8.0)
    server.declare_parameter("mocap_stale_timeout", 0.5)
    server.declare_parameter("auto_arm", True)
    server.declare_parameter("arm_timeout", 5.0)

    cf_name = server.get_parameter("cf_name").value
    takeoff_height = float(server.get_parameter("takeoff_height").value)
    timescale = float(server.get_parameter("timescale").value)
    scale_xy = float(server.get_parameter("scale_xy").value)
    require_mocap_lock = bool(server.get_parameter("require_mocap_lock").value)
    mocap_timeout = float(server.get_parameter("mocap_timeout").value)
    mocap_stale_timeout = float(server.get_parameter("mocap_stale_timeout").value)
    auto_arm = bool(server.get_parameter("auto_arm").value)
    arm_timeout = float(server.get_parameter("arm_timeout").value)

    if not server.crazyflies:
        raise RuntimeError(
            "No Crazyflies are enabled. Set one robot's 'enabled' field to true in crazyflies.yaml."
        )

    if cf_name:
        if cf_name not in server.crazyfliesByName:
            available = ", ".join(sorted(server.crazyfliesByName.keys()))
            raise RuntimeError(
                f"Crazyflie '{cf_name}' was not found. Enabled Crazyflies: {available}"
            )
        cf = server.crazyfliesByName[cf_name]
    else:
        cf = server.crazyflies[0]
        cf_name = cf.prefix.lstrip("/")

    traj = Trajectory()
    traj.loadcsv(Path(__file__).parent / "data/figure8.csv")
    _scale_trajectory_xy(traj, scale_xy)

    marker_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    marker_pub = server.create_publisher(Marker, "figure8_path", marker_qos)
    mocap_state = {"last_rx_time": None, "last_pose": None}

    def mocap_callback(msg):
        for named_pose in msg.poses:
            if named_pose.name == cf_name:
                mocap_state["last_rx_time"] = server.get_clock().now()
                mocap_state["last_pose"] = np.array(
                    [
                        named_pose.pose.position.x,
                        named_pose.pose.position.y,
                        named_pose.pose.position.z,
                    ]
                )
                
                break
    
    
    server.create_subscription(
        NamedPoseArray,
        "poses",
        mocap_callback,
        qos_profile_sensor_data,
    )

    battery_state = {"voltage": None} #initialize battery state dictionary

    # Callback to update battery voltage from Status messages
    def status_callback(msg):
        battery_state["voltage"] = msg.battery_voltage

    # Subscribe to Status messages to monitor battery voltage during the flight
    server.create_subscription(
        Status,
        f"{cf.prefix}/status",
        status_callback,
        10,
    )

    # NEW: Pre-Arm Battery Check Gate
# ----------------------------------------------------------------------------------------------------------------------
    server.get_logger().info("Checking pre-flight battery voltage...")
    start_time = server.get_clock().now()
    battery_timeout = Duration(seconds=3.0)  # Wait up to 3 seconds for a message
    
    while rclpy.ok() and battery_state["voltage"] is None:
        if (server.get_clock().now() - start_time) > battery_timeout:
            raise RuntimeError("Timeout waiting for battery status message. Aborting takeoff.")
        rclpy.spin_once(server, timeout_sec=0.1)

    initial_voltage = battery_state["voltage"]
    if initial_voltage is None:
        raise RuntimeError("Failed to read battery voltage before takeoff.")

    server.get_logger().info(f"Initial battery voltage detected: {initial_voltage:.2f} V")

    if initial_voltage <= 3.80:
        server.get_logger().error(
            f"CRITICAL: Battery voltage ({initial_voltage:.2f} V) is below the 3.80 V takeoff threshold!"
        )
        raise RuntimeError("Pre-flight battery check failed. Script terminated for safety.")
# ----------------------------------------------------------------------------------------------------------------------  
  
    if require_mocap_lock:
        start_time = server.get_clock().now()
        timeout_duration = Duration(seconds=mocap_timeout)
        stale_duration = Duration(seconds=mocap_stale_timeout)
        while rclpy.ok():
            now = server.get_clock().now()
            last_rx = mocap_state["last_rx_time"]
            if last_rx is not None and (now - last_rx) <= stale_duration:
                server.get_logger().info(f"Mocap lock acquired for {cf_name}.")
                break
            if (now - start_time) > timeout_duration:
                raise RuntimeError(
                    f"No fresh mocap pose received for {cf_name} within {mocap_timeout:.1f} s."
                )
            rclpy.spin_once(server, timeout_sec=0.1)

    if auto_arm:
        _wait_for_supervisor_bits(
            server,
            cf,
            cf_name,
            SUPERVISOR_INFO_CAN_BE_ARMED,
            arm_timeout,
            "an armable state",
        )
        supervisor = int(cf.get_status().get("supervisor", 0))
        if (supervisor & (SUPERVISOR_INFO_IS_ARMED | SUPERVISOR_INFO_CAN_FLY)) != (
            SUPERVISOR_INFO_IS_ARMED | SUPERVISOR_INFO_CAN_FLY
        ):
            server.get_logger().info(f"Arming {cf_name} before takeoff.")
            cf.arm(True)
            _wait_for_supervisor_bits(
                server,
                cf,
                cf_name,
                SUPERVISOR_INFO_IS_ARMED | SUPERVISOR_INFO_CAN_FLY,
                arm_timeout,
                "an armed, flyable state",
            )

    base_pose = np.array(cf.initialPosition, dtype=float)
    if mocap_state["last_pose"] is not None:
        base_pose = mocap_state["last_pose"].astype(float)

    start_pose = base_pose + np.array([0.0, 0.0, takeoff_height])
    takeoff_target_z = float(start_pose[2])
    marker = _build_path_marker(traj, start_pose)
    marker_pub.publish(marker)

    server.get_logger().info(
        f"Flying figure-8 with {cf_name} at {takeoff_height:.2f} m above start, "
        f"timescale {timescale:.2f}, xy scale {scale_xy:.2f}."
    )

    cf.uploadTrajectory(TRAJECTORY_ID, 0, traj)

    # Added Timer to find the duration of Takeoff to Landing for Stress Testing
    timer = server.get_clock().now()  # Start timer


    cf.takeoff(targetHeight=takeoff_target_z, duration=TAKEOFF_DURATION)
    time_helper.sleep(TAKEOFF_DURATION + 0.5)

    cf.goTo(start_pose, yaw=0.0, duration=POSITIONING_DURATION)
    time_helper.sleep(POSITIONING_DURATION + 0.5)

    cf.startTrajectory(TRAJECTORY_ID, timescale=timescale, relative=True)

    # NEW: Monitor battery during trajectory
# ----------------------------------------------------------------------------------------------------------------------
    elapsed = 0.0
    poll_interval = 0.5
    total_wait = traj.duration * timescale + 1.0
    emergency_land = False

    while elapsed < total_wait:
        time_helper.sleep(poll_interval)
        elapsed += poll_interval
        rclpy.spin_once(server, timeout_sec=0.0)
        voltage = battery_state["voltage"]
        if voltage is not None:
            if voltage <= 3.30:
                server.get_logger().warn(
                    f'CRITICAL BATTERY: {voltage:.2f}V - Landing immediately!'
                )
                emergency_land = True
                break
            elif voltage <= 3.50:
                server.get_logger().warn(
                    f'LOW BATTERY WARNING: {voltage:.2f}V - Land soon!'
                )
    if emergency_land:
        server.get_logger().warn('Emergency land complete. Disarming.')
        cf.land(targetHeight=LAND_HEIGHT, duration=LAND_DURATION)
        time_helper.sleep(LAND_DURATION + 0.5)
        cf.arm(False)
        return
# ----------------------------------------------------------------------------------------------------------------------

    elapsed = (server.get_clock().now() - timer).nanoseconds / 1e9  # End timer
    server.get_logger().info(f"Figure-8 flight completed in {elapsed:.2f} seconds.")

    cf.land(targetHeight=LAND_HEIGHT, duration=LAND_DURATION)
    time_helper.sleep(LAND_DURATION + 0.5)

    # -----------------------------------------------------------------
    # NEW: Active block that waits for the firmware to actually disarm
    # -----------------------------------------------------------------
    server.get_logger().info("Waiting for drone firmware to auto-disarm...")
    disarm_timeout_start = server.get_clock().now()
    
    # Keep checking the supervisor state until the armed bit (2) is gone
    while rclpy.ok():
        supervisor = int(cf.get_status().get("supervisor", 0))
        # SUPERVISOR_INFO_IS_ARMED = 2. If this bit is 0, it means it successfully disarmed!
        if (supervisor & SUPERVISOR_INFO_IS_ARMED) == 0:
            break
            
        # Safety timeout: if it takes more than 6 seconds, break anyway so the script doesn't hang forever
        if (server.get_clock().now() - disarm_timeout_start).nanoseconds / 1e9 > 6.0:
            server.get_logger().warn("Timed out waiting for firmware auto-disarm.")
            break
            
        rclpy.spin_once(server, timeout_sec=0.1)
    # -----------------------------------------------------------------

    # Force ROS2 to catch the final battery message right after disarm
    rclpy.spin_once(server, timeout_sec=0.1)

    # Now print your final voltage!
    if battery_state["voltage"] is not None:
        server.get_logger().info(f"Final battery voltage (POST-DISARM): {battery_state['voltage']:.2f}V")

    # Distance from mocap_pose to initial position at script start (not necessarily takeoff position)
    if mocap_state["last_pose"] is not None:
        final_position = mocap_state["last_pose"].astype(float)
        initial_position = base_pose
        distance = np.linalg.norm(final_position - initial_position)
        server.get_logger().info(
            f"Final position: {final_position}, Initial position: {initial_position}, "
            f"Distance from start: {distance:.2f} m"
        )
    
    server.get_logger().info("Mission complete. Triggering global launch shutdown.")
    server.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
