import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess
from launch.actions import IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.actions import TimerAction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _create_bag_recorder(context):
    cf_name = LaunchConfiguration("cf_name").perform(context).strip()
    backend = LaunchConfiguration("backend").perform(context).strip()
    bag_root = LaunchConfiguration("bag_root").perform(context).strip()

    if not bag_root:
        bag_root = os.path.join(os.getcwd(), "results", "figure8_bags")

    os.makedirs(bag_root, exist_ok=True)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    cf_label = cf_name if cf_name else "auto"
    bag_dir = os.path.join(bag_root, f"figure8_{backend}_{cf_label}_{timestamp}")

    topics = [
        "/tf",
        "/tf_static",
        "/rosout",
        "/figure8_path",
        "/poses",
        "/pointCloud",
    ]
    if cf_name:
        topics.extend([
            f"/{cf_name}/pose",
            f"/{cf_name}/status",
        ])

    return [
        LogInfo(msg=f"Recording flight data to {bag_dir}"),
        ExecuteProcess(
            cmd=["ros2", "bag", "record", "-o", bag_dir, *topics],
            output="screen",
        ),
    ]


def generate_launch_description():
    examples_share = get_package_share_directory("crazyflie_examples")
    crazyflie_share = get_package_share_directory("crazyflie")

    backend = LaunchConfiguration("backend")
    rviz = LaunchConfiguration("rviz")
    mocap = LaunchConfiguration("mocap")
    cf_name = LaunchConfiguration("cf_name")
    takeoff_height = LaunchConfiguration("takeoff_height")
    timescale = LaunchConfiguration("timescale")
    scale_xy = LaunchConfiguration("scale_xy")

    rviz_config = os.path.join(examples_share, "config", "figure8_demo.rviz")
    crazyflies_yaml = os.path.join(crazyflie_share, "config", "crazyflies.yaml")
    mocap_yaml = os.path.join(crazyflie_share, "config", "motion_capture.yaml")

    crazyflie = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(crazyflie_share, "launch"), "/launch.py"]
        ),
        launch_arguments={
            "backend": backend,
            "mocap": mocap,
            "gui": "False",
            "teleop": "False",
            "rviz": rviz,
            "rviz_config_file": rviz_config,
            "crazyflies_yaml_file": crazyflies_yaml,
            "motion_capture_yaml_file": mocap_yaml,
        }.items(),
    )

    figure8_demo = Node(
        package="crazyflie_examples",
        executable="figure8_demo",
        name="figure8_demo",
        parameters=[
            {
                "use_sim_time": PythonExpression(["'", backend, "' == 'sim'"]),
                "cf_name": cf_name,
                "takeoff_height": takeoff_height,
                "timescale": timescale,
                "scale_xy": scale_xy,
                "require_mocap_lock": PythonExpression(["'", mocap, "' == 'True'"]),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("backend", default_value="cpp"),
            DeclareLaunchArgument("rviz", default_value="True"),
            DeclareLaunchArgument("mocap", default_value="True"),
            DeclareLaunchArgument("cf_name", default_value=""),
            DeclareLaunchArgument("takeoff_height", default_value="0.8"),
            DeclareLaunchArgument("timescale", default_value="1.5"),
            DeclareLaunchArgument("scale_xy", default_value="0.7"),
            DeclareLaunchArgument("bag_root", default_value=""),
            crazyflie,
            OpaqueFunction(function=_create_bag_recorder),
            TimerAction(period=2.0, actions=[figure8_demo]),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=figure8_demo,
                    on_exit=[
                        LogInfo(msg="Figure-8 complete, stopping rosbag and shutting down."),
                        EmitEvent(event=Shutdown()),
                    ],
                )
            ),
        ]
    )
