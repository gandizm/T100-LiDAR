from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("device_ip", description="T100 LiDAR IPv4 address"),
        DeclareLaunchArgument("bind_ip", default_value="0.0.0.0"),
        DeclareLaunchArgument("point_port", default_value="60001"),
        DeclareLaunchArgument("imu_port", default_value="60003"),
        DeclareLaunchArgument("publish_freq", default_value="10.0"),
        DeclareLaunchArgument("frame_id", default_value="livox_frame"),
        DeclareLaunchArgument("imu_frame_id", default_value="livox_frame"),
        DeclareLaunchArgument("line_count", default_value="4"),
        DeclareLaunchArgument("lidar_id", default_value="0"),
        DeclareLaunchArgument("auto_start", default_value="false"),
    ]

    bridge = Node(
        package="t100_livox_compat",
        executable="t100_livox_compat",
        name="t100_livox_compat",
        output="screen",
        parameters=[{
            "device_ip": LaunchConfiguration("device_ip"),
            "bind_ip": LaunchConfiguration("bind_ip"),
            "point_port": LaunchConfiguration("point_port"),
            "imu_port": LaunchConfiguration("imu_port"),
            "publish_freq": LaunchConfiguration("publish_freq"),
            "frame_id": LaunchConfiguration("frame_id"),
            "imu_frame_id": LaunchConfiguration("imu_frame_id"),
            "line_count": LaunchConfiguration("line_count"),
            "lidar_id": LaunchConfiguration("lidar_id"),
            "auto_start": LaunchConfiguration("auto_start"),
        }],
    )

    return LaunchDescription(arguments + [bridge])
