# T100 as a Livox MID-360 ROS 2 integration replacement

The ROS 2 integration is split into two packages:

- livox_ros_driver2: a message/launch compatibility shim. It provides the
  same CustomMsg and CustomPoint ROS type names and the historical
  msg_MID360_launch.py entry point.
- t100_livox_compat: the actual T100 Ethernet/UDP receiver and ROS publisher.

This avoids building Livox SDK2 or running the upstream Livox device driver.
The upstream Livox-SDK/livox_ros_driver2 package must be removed from the
same colcon workspace because it has the same ROS package name as the shim.

## Build

Target ROS 2 Humble on Ubuntu 22.04 or Jazzy on Ubuntu 24.04.

~~~bash
git clone https://github.com/gandizm/T100-LiDAR.git ~/projects/T100-LiDAR
mkdir -p ~/ros2_ws/src
ln -s ~/projects/T100-LiDAR/ros2/livox_ros_driver2 ~/ros2_ws/src/livox_ros_driver2
ln -s ~/projects/T100-LiDAR/ros2/t100_livox_compat ~/ros2_ws/src/t100_livox_compat

cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
~~~

Do not clone the upstream Livox ROS Driver 2 package into this workspace.
The shim generates the same ROS message package name and the T100 bridge
publishes those message types.

## Launch

~~~bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py \
  device_ip:=192.168.1.10 host_ip:=192.168.1.20
~~~

The launch name is retained for minimal changes to existing MID360 launch
commands. The hardware connected to this driver must be a T100 Agras LiDAR.

The launch defaults to:
- configure and read back the point/IMU destination IP and UDP ports;
- start sampling;
- stop sampling on clean shutdown;
- keep the new destination settings after shutdown.

Use configure_device:=false when destinations are already configured.
Use restore_device_destinations:=true to restore the previous point/IMU
destination settings after a clean shutdown. auto_start:=false makes the node
receive-only.

## ROS interface

| Topic | Type |
|---|---|
| /livox/lidar | livox_ros_driver2/msg/CustomMsg (xfer_format=1) |
| /livox/lidar | sensor_msgs/msg/PointCloud2 (xfer_format=0) |
| /livox/imu | sensor_msgs/msg/Imu |

For FAST-LIO2, use xfer_format=1 and keep the existing Livox CustomMsg
subscriber and topic names. The default aggregation frequency is 10 Hz.

## Timing and data interpretation

The per-point time increment is computed as:

    point_interval_ns = time_interval * 100 / dot_num

This is the same formula used by the official Livox ROS Driver 2 SDK2
callback (src/comm/pub_handler.cpp). T100's protocol snapshot documents the
same time_interval field and 0.1 us unit.

The default MID360 line assignment is index modulo 4, matching the upstream
driver's kLineNumberMid360=4 convention. The T100-Agras variant's actual
channel mapping must still be confirmed using physical-device data. The
parameter line_count is configurable.

Zero XYZ placeholders and points beyond max_point_range_m are filtered.
Reflectivity is not used as a validity test. T100 tag and reflectivity bytes
are preserved.

IMU acceleration is converted from g to m/s^2. Gyroscope values remain
rad/s. Point and IMU time_type must match; mixed time domains are rejected.
For time_type=0, timestamps are passed through in the device-local time
domain, not converted to Unix wall time.

## Remaining hardware validation

This is a ROS-level replacement for the common MID360 integration path, not
a full Livox SDK2 emulator. Before flight or closed-loop control, verify on a
real T100:

1. destination configuration and persistent behavior across power cycles;
2. actual point and IMU rates, packet loss, and receive-buffer sizing;
3. point/IMU timestamp monotonicity and common time origin;
4. point_time_interval and timestamp reference using a real PCAP;
5. T100-Agras line mapping and coordinate/IMU axis conventions;
6. FAST-LIO2 startup, deskew, map stability, and long-duration behavior.

The repository has unit tests for the T100-to-Livox conversion helpers:

~~~bash
python -m unittest discover -s tests -v
~~~

A physical T100, ROS 2 colcon build, and FAST-LIO2 flight test are still
required for a production-readiness claim.
