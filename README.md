# T100 LiDAR Driver & Protocol Tools — Ethernet / UDP

[简体中文](README.zh-CN.md) · [Protocol reference](docs/protocol.en.md)

T100 LiDAR Python protocol driver and Ethernet/UDP tools for point cloud acquisition, IMU decoding, parameter access, sampling control, scan angle settings, and the firmware write mode entry command. Includes a local UDP simulator and a browser preview. This is an independent, unofficial implementation.

T100激光雷达驱动与通信协议：网口接入、点云获取、IMU解析、扫描角度设置、固件写入模式入口。

The simulator produces synthetic points; it does not demonstrate physical device performance.

## Common questions

### T100激光雷达驱动 / T100 LiDAR driver

The core Python client receives UDP point cloud and IMU data and sends protocol control requests. The native C++ ROS 2 replacement driver is under ros2/livox_ros_driver2 and provides the Livox ROS Driver 2 message interfaces and default topics; ros2/t100_livox_compat remains as a Python reference bridge. Neither emulates the Livox SDK2 device-control/discovery API.

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

External time synchronization can be encoded with `t100 frame time-sync`; the public reference does not define its ACK or complete synchronization conditions. The simulator does not claim to reproduce undocumented device behavior. A native C++ ROS 2 driver is included; see the MID-360 compatibility section below. The repository does not include or emulate the Livox SDK2 device API.

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

## ROS 2: use T100 as a MID-360-compatible source

The repository includes a native C++ ROS 2 driver package at ros2/livox_ros_driver2. It receives T100's native Ethernet/UDP protocol and publishes the Livox ROS Driver 2 message contract and default topics:

| Topic | ROS 2 message type |
|---|---|
| /livox/lidar | livox_ros_driver2/msg/CustomMsg |
| /livox/imu | sensor_msgs/msg/Imu |

The goal is to let FAST-LIO2 and other ROS consumers of Livox CustomMsg switch to T100 without changing their subscription types or default topic names. The driver connects to the T100 IP directly. It does not launch the official MID-360 node and does not require Livox-SDK2.

### Compatibility behavior

- The ROS package name remains livox_ros_driver2. This repository supplies the CustomMsg/CustomPoint interfaces and a livox_ros_driver2_node executable.
- xfer_format=1 publishes CustomMsg (recommended for FAST-LIO2); xfer_format=0 publishes the Livox PointXYZRTLT PointCloud2 layout.
- Point packets are aggregated at publish_freq (10 Hz by default). Per-point offsets use the MID-360 driver formula: time_interval × 100 / dot_num nanoseconds.
- Point line is point_index % 4, following the four-line MID-360 convention. The raw T100 tag byte is passed through unchanged.
- Gyroscope values are published in rad/s. T100 acceleration is documented in g and is converted to m/s^2.
- For time_type=0 (device uptime), one shared device-to-host epoch offset is used for both LiDAR and IMU. time_type=1/2 timestamps are passed through as synchronized device time.
- By default, the node configures the T100 point/IMU destination IP and ports, reads them back, starts sampling, and restores the original destinations and stops sampling on normal shutdown.

This is ROS message/topic compatibility, not Livox SDK2 device-protocol emulation. Applications that depend on SDK2 APIs, LVX recording, or proprietary Livox control commands are not automatically compatible.

### Build

Example: Ubuntu 22.04 + ROS 2 Humble. A minimal livox_ros_driver2 message/interface package is included in this repository. Do not clone the official Livox ROS Driver 2 into the same workspace: two ROS packages with the same package name cannot coexist.

~~~bash
mkdir -p ~/t100_ws/src
cd ~/t100_ws/src
git clone https://github.com/gandizm/T100-LiDAR.git
git clone -b ROS2 https://github.com/hku-mars/FAST_LIO.git
cd ~/t100_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to fast_lio
source install/setup.bash
~~~

### Run with the existing FAST-LIO2 MID-360 configuration

Start the driver, replacing the example address with the actual T100 IP:

~~~bash
ros2 launch livox_ros_driver2 msg_T100_launch.py device_ip:=192.168.1.12
~~~

If the host has multiple NICs and automatic route selection chooses the wrong one, specify host_ip:

~~~bash
ros2 launch livox_ros_driver2 msg_T100_launch.py device_ip:=192.168.1.12 host_ip:=192.168.1.20
~~~

In another terminal, start the ROS 2 branch of FAST-LIO2:

~~~bash
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml rviz:=true
~~~

FAST-LIO2's default /livox/lidar and /livox/imu topic names and CustomMsg type can remain unchanged. The T100 mounting pose differs from a standard MID-360 installation, so extrinsic_T / extrinsic_R must still be calibrated for the actual mounting.

Check the streams:

~~~bash
ros2 topic info /livox/lidar -v
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
~~~

### Required real-device validation

The driver now implements protocol parsing, ROS conversion, destination configuration, sampling control, and point batching. It has not yet completed closed-loop validation on a physical T100. Capture real PCAPs and verify:

1. Actual destination behavior after writing host_pcl_ip/port and host_imu_ip/port. The protocol notes that configured ports and observed streaming destinations may differ.
2. Whether time_interval × 100 / dot_num matches the real per-point interval for the firmware revision on your unit.
3. Whether time_type=0 point and IMU timestamps share one device clock. The driver rejects packets if the two streams use different time_type values.
4. Whether raw T100 tag values pass FAST-LIO2's Livox filter (tag & 0x30 equals 0x10 or 0x00). The driver does not rewrite tag values.
5. IMU axis directions, gyro units, acceleration scale, and LiDAR-IMU extrinsics.
6. Actual packet rate, loss, point density, 10 Hz batching latency, and CPU use on Orange Pi 5B / RDK X5.

If time_interval is zero or the calculated point interval is zero, the driver still publishes the point cloud but all point offsets in that packet are equal. This is not reliable motion deskew. Only set point_interval_fallback_ns after measuring a valid fallback interval.

The repository retains ros2/t100_livox_compat as a Python reference bridge. For high point rates and the primary FAST-LIO2 path, use the native C++ driver above.
