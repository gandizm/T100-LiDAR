# T100 MID-360 ROS 2 integration replacement

## Goal

Keep the ROS interface used by existing Livox MID-360 + FAST-LIO2 projects,
while replacing the Livox SDK2 device driver with a native T100 Ethernet/UDP
receiver.

The implementation is split into:

- ros2/livox_ros_driver2: message and launch compatibility shim;
- ros2/t100_livox_compat: T100 protocol client and ROS 2 publisher.

The compatibility shim retains the ROS package name and CustomMsg/CustomPoint
message types. Do not build the upstream Livox-SDK/livox_ros_driver2 package
in the same workspace. This replacement does not include or emulate SDK2.

## Official point timing reference

The Livox ROS Driver 2 PubHandler::OnLivoxLidarPointCloudCallback computes:

    point_interval = data->time_interval * 100 / data->dot_num; // ns

The T100 protocol snapshot uses the same time_interval field and 0.1 us unit.
The bridge uses the same per-point increment, with packet timestamp as the
base. Reference:
https://github.com/Livox-SDK/livox_ros_driver2/blob/master/src/comm/pub_handler.cpp

The upstream driver assigns MID360 line IDs as point_index modulo
kLineNumberMid360 (4). The T100 bridge follows that convention by default,
but the Agras variant's actual channel mapping must be confirmed on hardware.

## Current ROS contract

- /livox/lidar: livox_ros_driver2/msg/CustomMsg by default;
- /livox/lidar: sensor_msgs/msg/PointCloud2 when xfer_format=0;
- /livox/imu: sensor_msgs/msg/Imu;
- default point publish frequency: 10 Hz.

The PointCloud2 layout is the packed 26-byte Livox PointXYZRTLT format.
The CustomMsg path preserves timebase and uint32 per-point offset_time.

## Device control

The bridge can configure and read back T100 point/IMU destination addresses,
start sampling, stop sampling on clean shutdown, and optionally restore the
previous destinations. The control protocol uses UDP 60000; point and IMU
data are received on the configured host ports (defaults 60001 and 60003).

## Known boundaries

- no Livox SDK2 device discovery/control API emulation;
- no LVX recording;
- no proof yet of T100-Agras laser-line mapping;
- no physical-device PCAP validation or FAST-LIO2 flight validation yet;
- no automatic axis remapping or IMU/LiDAR extrinsic calibration.

Before using for flight localization, capture real T100 point/IMU traffic and
validate packet loss, timebase agreement, timestamp monotonicity, point timing,
line mapping, and coordinate conventions.
