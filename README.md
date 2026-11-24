# T100 LiDAR Driver & Protocol Tools — Ethernet / UDP

[简体中文](README.zh-CN.md) · [Protocol reference](docs/protocol.en.md)

T100 LiDAR Python protocol driver and Ethernet/UDP tools for point cloud acquisition, IMU decoding, parameter access, sampling control, scan angle settings, and the firmware write mode entry command. Includes a local UDP simulator and a browser preview. This is an independent, unofficial implementation.

T100激光雷达驱动与通信协议：网口接入、点云获取、IMU解析、扫描角度设置、固件写入模式入口。

The simulator produces synthetic points; it does not demonstrate physical device performance.

## Common questions

### T100激光雷达驱动 / T100 LiDAR driver

The core Python client receives UDP point cloud and IMU data and sends protocol control requests. A ROS 2 drop-in integration is provided as two packages: a livox_ros_driver2 message/launch shim and the t100_livox_compat T100 UDP node. It preserves the common FAST-LIO2 ROS message/topic contract, but does not emulate Livox SDK2.

### T100激光雷达刷固件 / Firmware write mode entry

This project provides only the `00/07` request to enter firmware write mode. See the [entry command](docs/protocol.en.md#10-enter-firmware-write-mode).

### 大疆激光雷达协议 / DJI LiDAR protocol

This repository documents T100 Ethernet/UDP frames and provides an independent implementation. It is not an official DJI SDK and does not claim compatibility with every DJI LiDAR model.

### T100 point cloud acquisition and scan angles

Use `capture` for UDP reception and `roi` to read or change the scan window. See the [Chinese guide](README.zh-CN.md) and [protocol reference](docs/protocol.en.md).

## Quick start

Python 3.9 or newer. The application has no third-party runtime dependencies.

```sh
python3 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -e .
.venv/bin/t100 demo --port 18766
```

Open `http://127.0.0.1:18766`. On Windows, use `.venv\Scripts\python` and `.venv\Scripts\t100` instead. On macOS, the included `Start Demo.command` launches the preview directly from source.

The demo includes start/stop, yaw window changes with readback, 3D/top views, IMU data, complete TX/RX frames, CSV/JSON export, and the firmware write mode entry command. Its UDP and HTTP sockets are confined to `127.0.0.1`.

## Command-line examples

Generate the published update-entry example without sending anything:

```sh
t100 frame enter-write-mode --sequence 0x4810
t100 frame read --parameter pcl_data_type --sequence 0x0110
t100 parameters
```

Listen for discovery or data. These commands do not change device settings:

```sh
t100 discover --seconds 5
t100 --ip 192.0.2.10 capture --bind 192.0.2.50 --seconds 10 --pcap capture.pcap
t100 --ip 192.0.2.10 status
t100 --ip 192.0.2.10 read pcl_data_type
```

Addresses above are documentation examples. Configure the actual receiving address and destination ports for your device. `capture` uses point port 60001 and IMU port 60003 by default; both are configurable. `--ip` filters incoming data to that source address.

Explicit control commands send requests to device UDP port 60000:

```sh
t100 --ip 192.0.2.10 --trace start
t100 --ip 192.0.2.10 roi --start 0 --end 180 --backup roi-before.json
t100 --ip 192.0.2.10 roi --disable --backup roi-before-disable.json
t100 --ip 192.0.2.10 stop
t100 --ip 192.0.2.10 enter-write-mode
```

The final command sends `00/07` to enter firmware write mode. The browser's **Enter firmware write mode** button sends this command to the local simulator. **Resume simulated sampling** resets the local model. The CLI command sends the request to the device address you specify.

ROI changes preserve the eleven original ROI values in a new JSON file, disable cropping before writes, and read back each written parameter. On a partial failure, inspect the settings before restoring them. The generic `write` command is lower-level and requires the caller to preserve original values.

Control requests use a single socket with matching endpoint, sequence and command checks. Writes are sent once, without automatic retries. A successful ACK/readback does not by itself establish actual sampling, coverage, motor state, or a successful reboot.

## Protocol and implementation scope

| Component | Included |
|---|---|
| Python library | CRC8/CRC16, frame codec, 25 UIDs, single/batch reads, single writes, target control, sampling, ROI, reboot and update entry |
| Data decoder | 96-point XYZ packets, IMU with CRC32, status TLVs, sequence accounting |
| Simulator | Local UDP control peer, deterministic synthetic room, ROI cropping, point/IMU/status streams |
| Recording | PCAP with reconstructed IPv4/UDP headers, plus browser CSV and JSON snapshots |
| C header | Portable control-frame CRC/build/check helpers |
| Wireshark Lua | Control/data field dissection; CRC fields are displayed, not verified |

External time synchronization can be encoded with `t100 frame time-sync`; the public reference does not define its ACK or complete synchronization conditions. The simulator does not claim to reproduce undocumented device behavior. A ROS 2 Livox-message compatibility bridge is included under ros2/; see the integration section below.

PCAP files preserve received UDP payloads, timing and socket endpoints. Link-layer information and original IP headers are not captured. Browser CSV exports contain valid simulated points from the rolling two-second buffer. Sequence gaps are observable counters, not a measurement of network loss by themselves.

## Development

```sh
.venv/bin/python -m unittest discover -s tests -v
cc -std=c99 -Wall -Wextra -Werror -Iextras tests/test_header.c -o /tmp/t100-header-test
/tmp/t100-header-test
```

Wire examples in the public protocol are used as fixed compatibility fixtures. Other tests cover malformed packets, CRC failures, response matching, real loopback UDP exchanges, ROI/readback, HTTP controls, PCAP output, and the firmware write mode entry.

See [the Chinese quick start](README.zh-CN.md), [the published protocol snapshot](docs/protocol.en.md), [the protocol source record](docs/SOURCE.md).

## License

**Non-commercial use only. Commercial use is prohibited. No commercial authorization is offered.**

The original code and documentation are available under the [T100 Non-Commercial License 1.0](LICENSE). Non-commercial learning, research, modification and redistribution are permitted subject to its terms. Commercial products or services, paid repair work, business operations and commercial research or development are prohibited. Keep the copyright and license notices, and identify changes when redistributing modified copies. This is a source-available project, not an OSI-approved open-source license.

## ROS 2 / drop-in MID-360 integration

Two ROS 2 packages are provided:

- ros2/livox_ros_driver2: a message/launch compatibility shim preserving the
  public livox_ros_driver2/CustomMsg and CustomPoint definitions, executable
  name, and historical msg_MID360_launch.py launch entry.
- ros2/t100_livox_compat: the actual T100 Ethernet/UDP receiver, control
  client, IMU publisher, and point-cloud converter.

This lets existing FAST-LIO2 projects keep their Livox message type and
default topics while replacing the MID-360 device driver. In the same colcon
workspace, remove the upstream Livox-SDK/livox_ros_driver2 package; the
upstream package and this shim have the same ROS package name and must not
coexist. The shim does not include or emulate Livox SDK2.

### Build (ROS 2 Humble / Jazzy)

~~~sh
git clone https://github.com/gandizm/T100-LiDAR.git ~/projects/T100-LiDAR
mkdir -p ~/ros2_ws/src
ln -s ~/projects/T100-LiDAR/ros2/livox_ros_driver2 ~/ros2_ws/src/livox_ros_driver2
ln -s ~/projects/T100-LiDAR/ros2/t100_livox_compat ~/ros2_ws/src/t100_livox_compat

cd ~/ros2_ws
source /opt/ros/humble/setup.bash   # use jazzy on Ubuntu 24.04
colcon build --symlink-install
source install/setup.bash
~~~

### Run

~~~sh
ros2 launch livox_ros_driver2 msg_MID360_launch.py device_ip:=T100_IP host_ip:=ROS_HOST_IP
~~~

The launch file defaults to configuring and reading back the T100 point-cloud
and IMU destination addresses, starting sampling, and stopping sampling on
clean shutdown. If host_ip is omitted, the node selects the local IPv4 address
routed toward the device. Set configure_device:=false if destinations are
already configured. Set restore_device_destinations:=true to restore the
previous point/IMU destinations on clean shutdown.

### ROS interface

| Topic | ROS 2 type |
|---|---|
| /livox/lidar | livox_ros_driver2/msg/CustomMsg (xfer_format=1) |
| /livox/lidar | sensor_msgs/msg/PointCloud2 (xfer_format=0) |
| /livox/imu | sensor_msgs/msg/Imu |

For a typical FAST-LIO2 Livox configuration, retain /livox/lidar, /livox/imu,
and the Livox CustomMsg input. The driver publishes at 10 Hz by default. It
preserves raw T100 tag and reflectivity. The line assignment uses the same
index-modulo-four convention as the upstream MID360 driver; verify that the
Agras variant's physical channel mapping matches before treating line IDs as
calibrated channels.

The PointCloud2 path uses the packed 26-byte Livox PointXYZRTLT layout:
x/y/z/intensity float32, tag/line uint8, timestamp float64. Point timestamps
are absolute nanoseconds in the device time domain.

### Timing and validation

The point interval follows the official Livox ROS Driver 2 SDK2 callback:
time_interval * 100 / dot_num nanoseconds. See the upstream
pub_handler.cpp:
https://github.com/Livox-SDK/livox_ros_driver2/blob/master/src/comm/pub_handler.cpp

The T100 protocol uses the same Ethernet packet header fields and 0.1 us
time-interval unit. The bridge keeps the packet timestamp plus the per-point
interval and batches points into Livox CustomMsg timebases.

T100 time_type consistency is enforced across point and IMU streams. In
time_type=0, the protocol describes the timestamp as a device-local timebase
rather than Unix wall time; both streams are forwarded unchanged. Acceleration
is converted from g to m/s^2, and angular velocity remains rad/s.

Before using this as a flight-control localization source, validate a real
T100 PCAP for timestamp monotonicity, point/IMU timebase agreement, packet
loss, actual point and IMU rates, and the T100-Agras line mapping. The bridge
has not yet been validated against a physical T100 or run through a complete
FAST-LIO2 flight test. It preserves the ROS interface; it does not emulate
all SDK2 device discovery/control features or LVX recording.

The protocol conversion helpers have unit tests:

~~~sh
python -m unittest discover -s tests -v
~~~

See docs/ROS2_MID360_DROPIN.md and ros2/README.md.
