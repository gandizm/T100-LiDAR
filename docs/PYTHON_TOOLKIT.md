> Legacy documentation for the original Python protocol toolkit. The recommended native C++ / ROS 2 / FAST-LIO2 integration is documented in the [main README](../README.md). The ROS section below describes the optional Python reference bridge, not the primary driver.

# T100 LiDAR Driver & Protocol Tools — Ethernet / UDP

[简体中文](README.zh-CN.md) · [Protocol reference](docs/protocol.en.md)

T100 LiDAR Python protocol driver and Ethernet/UDP tools for point cloud acquisition, IMU decoding, parameter access, sampling control, scan angle settings, and the firmware write mode entry command. Includes a local UDP simulator and a browser preview. This is an independent, unofficial implementation.

T100激光雷达驱动与通信协议：网口接入、点云获取、IMU解析、扫描角度设置、固件写入模式入口。

The simulator produces synthetic points; it does not demonstrate physical device performance.

## Common questions

### T100激光雷达驱动 / T100 LiDAR driver

The core Python client receives UDP point cloud and IMU data and sends protocol control requests. An optional ROS 2 compatibility package is included under ros2/t100_livox_compat; it publishes the same Livox ROS Driver 2 message types and default topics. It is not a kernel driver and does not emulate the Livox SDK2 device-control/discovery API.

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

External time synchronization can be encoded with `t100 frame time-sync`; the public reference does not define its ACK or complete synchronization conditions. The simulator does not claim to reproduce undocumented device behavior. No ROS/ROS2 integration is included.

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

## ROS 2 / Livox MID360 interface compatibility

The optional ros2/t100_livox_compat package receives native T100 UDP point-cloud and IMU packets and publishes:

| Topic | ROS 2 type |
|---|---|
| /livox/lidar | livox_ros_driver2/msg/CustomMsg |
| /livox/imu | sensor_msgs/msg/Imu |

This keeps the usual Livox ROS Driver 2 output contract: `xfer_format:=1` publishes `CustomMsg` (the default for FAST-LIO2), while `xfer_format:=0` publishes the official Livox `PointXYZRTLT` PointCloud2 field layout. It is a ROS-message compatibility bridge, not an SDK2 device emulator: the official Livox ROS Driver 2 package is needed for its generated CustomMsg type, but its MID360 node is not launched.

### Build

1. Build/install the official Livox ROS Driver 2 package in the same ROS 2 workspace. This bridge uses its CustomMsg and CustomPoint definitions.
2. Put this repository under the workspace src/ directory.
3. Source the ROS 2 distribution and build:

```sh
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to t100_livox_compat
source install/setup.bash
```

### Run

First configure the T100 to send point-cloud and IMU UDP traffic to the ROS 2 host. The default receive ports are 60001 and 60003; verify the actual destination with a packet capture before changing them. Start sampling separately unless auto_start:=true is explicitly requested.

```sh
t100 --ip <T100_IP> start
ros2 launch t100_livox_compat t100_mid360_compat.launch.py \
  device_ip:=<T100_IP> bind_ip:=<ROS_HOST_IP>
```

The bridge publishes at 10 Hz by default. Set `xfer_format:=1` for Livox CustomMsg or `xfer_format:=0` for the official Livox PointCloud2 layout. Useful checks:

```sh
ros2 topic info /livox/lidar -v
ros2 topic hz /livox/lidar
ros2 topic echo /livox/imu --once
```

### Compatibility scope and validation status

- Supported input packets: the documented 96-point Cartesian-high packet (data_type=1) and one-sample IMU packet (data_type=0).
- Both output formats follow the Livox ROS Driver 2 point layout. PointCloud2 uses x/y/z/intensity/tag/line/timestamp fields with the official 26-byte packed point step. Point timing follows the Livox SDK2/ROS Driver 2 calculation: time_interval * 100 / dot_num nanoseconds per point. Points are batched into CustomMsg messages with a uint64 timebase and uint32 relative offset_time.
- The default line_count=4 follows the official MID360 driver setting. Confirm the T100 channel/ring interpretation with real captures before relying on it.
- T100 acceleration is documented in g and is converted to m/s² for sensor_msgs/Imu; angular velocity remains rad/s.
- Device timestamps are forwarded unchanged so point and IMU streams retain their common device time domain. In time_type=0, this is the device's since-startup time, not Unix epoch time.
- This first bridge milestone has protocol/unit tests but has not been validated against a physical T100. Confirm actual packet destinations, timestamp monotonicity, time-type agreement, point interval behavior, line mapping, and IMU units with real-device PCAPs.
- The bridge does not implement Livox SDK2 discovery/control emulation, LVX recording, or every Livox ROS Driver 2 configuration feature. It aims to leave downstream ROS consumers unchanged, not every SDK-level integration.
