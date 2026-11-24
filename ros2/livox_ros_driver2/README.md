# T100 driver with Livox ROS Driver2-compatible ROS2 interfaces

This package is a T100 Ethernet/UDP driver, not the official Livox ROS Driver2.
It deliberately reuses the ROS package name and the CustomMsg/CustomPoint
message definitions so ROS2 consumers such as HKU-MARS FAST-LIO2 can subscribe
without changing their message type or topic names.

It does not use Livox-SDK2 and does not emulate the Livox SDK2 device protocol.
Remove the official livox_ros_driver2 package from the same colcon workspace
when using this replacement; two packages with the same ROS package name cannot
coexist in one workspace.

## ROS interface

- livox_ros_driver2/msg/CustomMsg on /livox/lidar (default)
- sensor_msgs/msg/Imu on /livox/imu (default)
- xfer_format=1: Livox-compatible CustomMsg (recommended for FAST-LIO2)
- xfer_format=0: Livox-compatible packed PointCloud2 (PointXYZRTLT layout)

The driver aggregates T100 packets at publish_freq (10 Hz by default), maps the
T100 point interval to CustomPoint.offset_time, uses line = point_index % 4
to match the MID-360 convention in Livox ROS Driver2, passes the T100 tag byte
through unchanged, and converts T100 acceleration from g to m/s^2.

For time_type=0 (device uptime), the first packet establishes one shared
device-to-host epoch offset for both point and IMU streams. time_type=1 and
time_type=2 timestamps are passed through as synchronized device time.

## Build

Use ROS2 Humble on Ubuntu 22.04 (or another ROS2 distribution supported by
Livox ROS Driver2 / FAST-LIO2).

~~~bash
mkdir -p ~/t100_ws/src
cd ~/t100_ws/src
git clone https://github.com/gandizm/T100-LiDAR.git
git clone -b ROS2 https://github.com/hku-mars/FAST_LIO.git
cd ~/t100_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to fast_lio
source install/setup.bash
~~~

Do not clone the official Livox ROS Driver2 into this workspace and do not
install Livox-SDK2 just for this T100 path. This repository provides the
livox_ros_driver2 ROS interface package needed by FAST-LIO2.

## Run

Connect the T100 Ethernet adapter to the host NIC. Set the host NIC IPv4 address
to a static address in the same subnet as the T100. Then:

~~~bash
# Start the T100 driver. Replace the address with the actual T100 address.
ros2 launch livox_ros_driver2 msg_T100_launch.py device_ip:=192.168.1.12

# In another terminal, use the unchanged FAST-LIO2 MID-360 configuration.
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml rviz:=true
~~~

The msg_MID360_launch.py launch filename is also provided for workflows that
expect the official driver's launch naming. Both launch files start the T100
driver; they do not connect to a physical MID-360.

The driver automatically selects the host IPv4 route to the T100, configures
the point/IMU destination IP and UDP ports, reads the settings back, starts
sampling, and restores the previous destination settings and stops sampling on
normal shutdown. Set host_ip explicitly if the route-selected interface is
not the NIC connected to the T100.

## Important validation before relying on SLAM

This is the first ROS2 compatibility implementation. The wire layout and point
time conversion follow the published T100 protocol and the corresponding
MID-360 conversion convention, but real-hardware validation is still required.
In particular, verify:

1. actual T100 UDP destination behavior after writing host_pcl_* and
   host_imu_* parameters;
2. that time_interval * 100 / dot_num is the correct point interval for the
   firmware revision on your unit;
3. that time_type=0 point and IMU timestamps share the same device clock;
4. that raw T100 tag values pass FAST-LIO2's Livox tag filter
   ((tag & 0x30) == 0x10 or 0x00);
5. IMU axis directions, gyro units, acceleration scale, and LiDAR-IMU extrinsics;
6. packet rate, loss, point density, and CPU load on the target ARM board.

If time_interval is zero, point offsets are zero unless
point_interval_fallback_ns is explicitly set. Do not invent a fallback interval
without measuring it. point_interval_fallback_ns is an advanced override.

FAST-LIO2's mid360.yaml can keep its topic names and Livox message type, but
the extrinsic parameters must be calibrated for the T100's actual mounting.
The scan_line: 4 setting matches the four-line index convention emitted by
the MID-360 driver interface; it does not imply that T100 exposes physical ring
IDs in its UDP records.

This package does not claim compatibility with every Livox application, SDK2
API, LVX recorder, or proprietary Livox control command.
