import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory("livox_ros_driver2")
    default_config = os.path.join(package_share, "config", "T100_config.yaml")
    args = [
        DeclareLaunchArgument("device_ip", default_value="192.168.1.12"),
        DeclareLaunchArgument("bind_ip", default_value="0.0.0.0"),
        DeclareLaunchArgument("host_ip", default_value=""),
        DeclareLaunchArgument("publish_freq", default_value="10.0"),
        DeclareLaunchArgument("xfer_format", default_value="1"),
    ]

    driver = Node(
        package="livox_ros_driver2",
        executable="livox_ros_driver2_node",
        name="livox_lidar_publisher",
        output="screen",
        parameters=[default_config, {
            "device_ip": LaunchConfiguration("device_ip"),
            "bind_ip": LaunchConfiguration("bind_ip"),
            "host_ip": LaunchConfiguration("host_ip"),
            "publish_freq": ParameterValue(
                LaunchConfiguration("publish_freq"), value_type=float
            ),
            "xfer_format": ParameterValue(
                LaunchConfiguration("xfer_format"), value_type=int
            ),
        }],
    )
    return LaunchDescription(args + [driver])
