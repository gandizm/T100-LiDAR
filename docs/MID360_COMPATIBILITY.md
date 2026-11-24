# MID-360 ROS integration compatibility contract

## Goal

Present the T100 Agras Ethernet/UDP stream to ROS 2 consumers through the same
Livox ROS Driver 2 message contract and default topic names used by a MID-360.
The primary target is the ROS 2 branch of HKU-MARS FAST-LIO2.

This is not Livox SDK2 emulation. It does not implement Livox SDK2 device
discovery, its device handles, LVX recording, or arbitrary Livox control APIs.

## Package replacement strategy

The repository contains a minimal ROS package named livox_ros_driver2.
It provides the official CustomMsg and CustomPoint interface definitions
and a T100-backed livox_ros_driver2_node executable. The ROS package name is
intentional: FAST-LIO2 imports livox_ros_driver2/msg/CustomMsg and declares
a dependency on livox_ros_driver2.

Do not place the official Livox ROS Driver 2 package in the same colcon
workspace. It has the same ROS package name. The replacement does not link
Livox-SDK2 and does not start an official MID-360 node.

## Compatibility table

| MID-360 ROS driver output | T100 replacement |
|---|---|
| /livox/lidar | /livox/lidar |
| livox_ros_driver2/msg/CustomMsg | Same ROS interface |
| /livox/imu | /livox/imu |
| sensor_msgs/msg/Imu | Same ROS interface |
| 10 Hz cloud batching | 10 Hz default, configurable |
| frame_id=livox_frame | Same default |
| xfer_format=1 CustomMsg | Supported |
| xfer_format=0 PointCloud2 | Supported with PointXYZRTLT fields |
| Livox SDK2 device control | Not emulated; T100 native control protocol is used |

## T100-to-Livox field mapping

### Cartesian point packet

T100's documented Cartesian packet contains a 36-byte header and 96 records
of 14 bytes each. Each record contains signed int32 XYZ in millimeters,
uint8 reflectivity, and uint8 tag.

| T100 field | Livox CustomPoint field | Conversion |
|---|---|---|
| X/Y/Z int32 mm | x/y/z float32 m | Divide by 1000 |
| reflectivity uint8 | reflectivity uint8 | Pass through |
| tag uint8 | tag uint8 | Pass through unchanged |
| point index | line uint8 | index modulo 4, matching the MID-360 driver convention |
| packet timestamp + point interval | offset_time uint32 | Relative to CustomMsg timebase |

The Livox ROS Driver 2 implementation uses a MID-360 line count of four and
computes point spacing as:

point_interval_ns = time_interval * 100 / dot_num

The T100 header documents time_interval in 0.1 microsecond units. The
replacement applies the same formula, then assigns each point:

point_timestamp_ns = packet_timestamp_ns + point_index * point_interval_ns

When building a CustomMsg, timebase is the first point's absolute timestamp
and each offset_time is the point timestamp minus that timebase. FAST-LIO2's
Livox preprocessing path divides offset_time by 1,000,000 to obtain milliseconds.

The line field is a compatibility index, not a claim that the T100 UDP
record contains a physical laser ring ID. The raw T100 tag semantics must
still be verified against FAST-LIO2's filter, which accepts tags satisfying
(tag & 0x30) == 0x10 or 0x00.

### IMU

| T100 field | ROS Imu field | Conversion |
|---|---|---|
| gyro_x/y/z | angular_velocity.x/y/z | rad/s, pass through |
| acc_x/y/z | linear_acceleration.x/y/z | g × 9.80665 to m/s² |
| packet timestamp | header.stamp | Shared time mapping with point stream |
| orientation | orientation | Not provided; orientation_covariance[0] = -1 |

### Timestamp domains

- time_type=0: T100 device uptime. The driver establishes one host-epoch
  offset from the first packet and applies that same offset to both point and
  IMU streams. This preserves their relative device-clock timing without
  pretending that the device uptime is Unix time.
- time_type=1: PTP/gPTP synchronized timestamp; forwarded unchanged.
- time_type=2: GPS synchronized timestamp; forwarded unchanged.

The driver rejects packets if point and IMU streams use different time_type
values. A device reboot while the node remains running should be treated as a
new timestamp epoch; restart the bridge after a T100 reboot.

## FAST-LIO2 integration

The ROS 2 FAST-LIO2 config/mid360.yaml subscribes to /livox/lidar as
livox_ros_driver2/msg/CustomMsg and /livox/imu as sensor_msgs/msg/Imu.
Its lidar_type: 1 and scan_line: 4 settings therefore remain appropriate
for this interface. The T100 installation extrinsics must be calibrated; do
not blindly reuse the standard MID-360 mounting translation.

The primary run sequence is:

1. Build this repository and the FAST-LIO2 ROS 2 branch in one colcon workspace.
2. Do not clone the official Livox ROS Driver 2 package into that workspace.
3. Launch livox_ros_driver2 msg_T100_launch.py with the actual T100 IP.
4. Launch FAST-LIO2 with its existing mid360.yaml.
5. Verify /livox/lidar and /livox/imu rates, timestamps, and packet counters.
6. Validate tag acceptance, point timing, IMU units/axes, and extrinsics with a
   real PCAP and stationary/controlled-motion tests before flight use.

## Current limits

- The native C++ driver currently targets the documented 96-point Cartesian
  high-data packet and one-sample IMU packet.
- The point interval formula is based on the public T100 protocol description
  and the Livox MID-360 driver convention; it has not yet been measured on a
  physical T100 firmware revision.
- A zero or integer-rounded-to-zero point interval produces zero point offsets
  unless the advanced point_interval_fallback_ns parameter is set. Do not
  choose a fallback without measuring the device.
- The raw tag byte is not remapped. If real T100 tags do not match FAST-LIO2's
  accepted Livox tag values, a verified mapping will be required.
- The device destination parameter write/readback path must be checked against
  actual UDP destination addresses and ports. The public protocol notes that
  configured ports and observed stream ports may differ.
- No physical T100 hardware was available during this code implementation.
  This branch is an integration milestone, not a claim of field-validated SLAM.
