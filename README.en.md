# T100 / MID360-agras LiDAR: UDP, ROS 2 and FAST-LIO2

This repository now includes a complete T100 Agras LiDAR integration snapshot under
\`integrations/mid360_agras_t100_lidar_driver/\`: a standalone C++17 UDP driver,
a ROS 2 node publishing Livox-compatible messages, a FAST-LIO2 workspace,
360-degree FOV/ROI tools, Linux scripts, and Windows point-cloud viewers.

The root Python package (\`src/t100_lidar/\`) remains as a complementary protocol
research toolkit and UDP simulator. For real-time ROS 2 + FAST-LIO2, start with
the imported C++ integration, not the experimental Python bridge.

## Quick start

Target platform documented by the upstream project: Ubuntu 22.04 + ROS 2 Humble.
Install ROS 2 first, then:

~~~bash
cd integrations/mid360_agras_t100_lidar_driver
sudo ./scripts/setup_network.sh
./scripts/build_all.sh --deps
./scripts/run.sh demo
./scripts/run.sh view
./scripts/run.sh mapping_norviz
~~~

The LiDAR Ethernet port is a custom Type-C connector, **not USB-C**. The
upstream measured operating point is about 11V / 0.345A; its full input range
has not been established. Default network settings are LiDAR
192.168.1.10, host 192.168.1.20/24, point UDP 60001, IMU UDP 60003.

ROS outputs:

- \`/livox/lidar\`: \`livox_ros_driver2/CustomMsg\`
- \`/livox/imu\`: \`sensor_msgs/Imu\`
- \`/livox/points\`: \`sensor_msgs/PointCloud2\` for RViz

See the [Chinese main README](README.md), [protocol reference](integrations/mid360_agras_t100_lidar_driver/docs/protocol.en.md),
[integration notes](docs/AGRAS_INTEGRATION.md), and [license notes](docs/LICENSING.md).

The imported upstream snapshot is GPL-2.0; Livox message definitions are MIT.
The original root Python toolkit remains under T100 Non-Commercial License 1.0.
Do not apply one license to the other component.
