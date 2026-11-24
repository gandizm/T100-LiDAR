from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("device_ip", description="T100 LiDAR IPv4 address"),
        DeclareLaunchArgument("host_ip", default_value=""),
        DeclareLaunchArgument("bind_ip", default_value="0.0.0.0"),
        DeclareLaunchArgument("point_port", default_value="60001"),
        DeclareLaunchArgument("imu_port", default_value="60003"),
        DeclareLaunchArgument("publish_freq", default_value="10.0"),
        DeclareLaunchArgument("multi_topic", default_value="0"),
        DeclareLaunchArgument("data_src", default_value="0"),
        DeclareLaunchArgument("output_data_type", default_value="0"),
        DeclareLaunchArgument("user_config_path", default_value=""),
        DeclareLaunchArgument("xfer_format", default_value="1",
                              description="0=Livox PointCloud2, 1=Livox CustomMsg"),
        DeclareLaunchArgument("frame_id", default_value="livox_frame"),
        DeclareLaunchArgument("imu_frame_id", default_value="livox_frame"),
        DeclareLaunchArgument("line_count", default_value="4"),
        DeclareLaunchArgument("lidar_id", default_value="0"),
        DeclareLaunchArgument("configure_device", default_value="true"),
        DeclareLaunchArgument("auto_start", default_value="true"),
        DeclareLaunchArgument("auto_stop_on_exit", default_value="true"),
        DeclareLaunchArgument("restore_device_destinations", default_value="false"),
        DeclareLaunchArgument("max_point_range_m", default_value="150.0"),
    ]

    bridge = Node(
        package="t100_livox_compat",
        executable="t100_livox_compat",
        name="t100_livox_compat",
        output="screen",
        parameters=[{
            "device_ip": LaunchConfiguration("device_ip"),
            "host_ip": LaunchConfiguration("host_ip"),
            "bind_ip": LaunchConfiguration("bind_ip"),
            "point_port": LaunchConfiguration("point_port"),
            "imu_port": LaunchConfiguration("imu_port"),
            "publish_freq": LaunchConfiguration("publish_freq"),
            "multi_topic": LaunchConfiguration("multi_topic"),
            "data_src": LaunchConfiguration("data_src"),
            "output_data_type": LaunchConfiguration("output_data_type"),
            "user_config_path": LaunchConfiguration("user_config_path"),
            "xfer_format": LaunchConfiguration("xfer_format"),
            "frame_id": LaunchConfiguration("frame_id"),
            "imu_frame_id": LaunchConfiguration("imu_frame_id"),
            "line_count": LaunchConfiguration("line_count"),
            "lidar_id": LaunchConfiguration("lidar_id"),
            "configure_device": LaunchConfiguration("configure_device"),
            "auto_start": LaunchConfiguration("auto_start"),
            "auto_stop_on_exit": LaunchConfiguration("auto_stop_on_exit"),
            "restore_device_destinations": LaunchConfiguration("restore_device_destinations"),
            "max_point_range_m": LaunchConfiguration("max_point_range_m"),
        }],
    )

    return LaunchDescription(arguments + [bridge])
