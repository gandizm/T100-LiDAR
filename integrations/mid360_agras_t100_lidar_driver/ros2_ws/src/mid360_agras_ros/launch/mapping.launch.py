"""Agras Mid-360 driver + FAST-LIO (hku-mars/FAST_LIO, unmodified) with our config; rviz:=false for headless."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("mid360_agras_ros")
    lio_share = get_package_share_directory("fast_lio")
    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("config", default_value=os.path.join(share, "config", "fastlio_mid360_agras.yaml"),
                              description="FAST-LIO parameter file"),
        Node(package="mid360_agras_ros", executable="mid360_agras_node", name="mid360_agras", output="screen"),
        Node(package="fast_lio", executable="fastlio_mapping", name="laser_mapping", output="screen",
             parameters=[LaunchConfiguration("config")]),
        Node(package="rviz2", executable="rviz2", arguments=["-d", os.path.join(lio_share, "rviz", "fastlio.rviz")],
             condition=IfCondition(LaunchConfiguration("rviz"))),
    ])
