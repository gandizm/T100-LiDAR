"""Agras Mid-360 driver (mid360_agras_node); rviz:=true also opens RViz with the raw point cloud."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("mid360_agras_ros")
    rviz = LaunchConfiguration("rviz")
    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="false", description="open RViz with /livox/points"),
        DeclareLaunchArgument("lidar_ip", default_value="192.168.1.10"),
        Node(package="mid360_agras_ros", executable="mid360_agras_node", name="mid360_agras", output="screen",
             parameters=[{"lidar_ip": ParameterValue(LaunchConfiguration("lidar_ip"), value_type=str)}]),
        # RViz needs the cloud's frame to exist in TF
        Node(package="tf2_ros", executable="static_transform_publisher", name="livox_frame_tf",
             arguments=["--frame-id", "map", "--child-frame-id", "livox_frame"], condition=IfCondition(rviz)),
        Node(package="rviz2", executable="rviz2", arguments=["-d", os.path.join(share, "rviz", "driver.rviz")],
             condition=IfCondition(rviz)),
    ])
