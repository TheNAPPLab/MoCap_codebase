import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, Shutdown 
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    examples_share = get_package_share_directory("crazyflie_examples")
    crazyflie_share = get_package_share_directory("crazyflie")

    backend = LaunchConfiguration("backend")
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
            "rviz": LaunchConfiguration("rviz"),
            "rviz_config_file": rviz_config,
            "crazyflies_yaml_file": crazyflies_yaml,
            "motion_capture_yaml_file": mocap_yaml,
        }.items(),
    )

    figure8_mpc_demo = Node(
        package="crazyflie_examples",
        executable="figure8_mpc_demo",
        name="figure8_mpc_demo",
        output='screen',
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
        on_exit=Shutdown(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("backend", default_value="sim"),
            DeclareLaunchArgument("rviz", default_value="True"),
            DeclareLaunchArgument("mocap", default_value="True"),
            DeclareLaunchArgument("cf_name", default_value=""),
            DeclareLaunchArgument("takeoff_height", default_value="0.8"),
            DeclareLaunchArgument("timescale", default_value="1.5"),
            DeclareLaunchArgument("scale_xy", default_value="0.7"),
            crazyflie,
            figure8_mpc_demo,
        ]
    )
