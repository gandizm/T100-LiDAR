# MID360-agras / T100 工程整合说明

## 导入来源

- 上游仓库：https://github.com/AkiyamaYukina/mid360_agras_t100_lidar_driver
- 导入提交：\`492b4a237741a051f6cd0eb118dabd4e1b756e90\`
- 上游提交日期：2026-11-24
- 导入位置：\`integrations/mid360_agras_t100_lidar_driver/\`
- 导入策略：保留完整目录结构、README、协议文档、图片、脚本、C++ 驱动、ROS 2 工作区和第三方许可证；不把上游代码改写成根目录 Python 包的一部分。

## 为什么采用这个工程作为主路线

此前根目录 Python 工具包主要解决协议解析、控制命令、模拟器和浏览器预览。新上游项目补齐了实机接入所需的主链：

1. 独立 C++17/POSIX UDP 接收器；
2. 点云和 IMU 解码、逐点时间、帧组装、丢包统计；
3. ROS 2 节点，发布 Livox CustomMsg、Imu 和 RViz PointCloud2；
4. FAST-LIO2 ROS 2 工作区和针对该雷达的配置；
5. 网络设置、360° FOV/ROI 工具、建图录制与地图导出；
6. Windows 实时点云与 PCD 查看器。

因此后续以该 C++/ROS 2 路线为主；根目录 Python 工具作为协议研究和交叉验证工具保留。根目录旧的 \`ros2/t100_livox_compat\` 是实验性 Python 桥接，不应和导入工程的 C++ ROS 节点混为一谈。

## 重要接线定义变化

导入工程的 A 面针脚图（\`integrations/mid360_agras_t100_lidar_driver/docs/images/pinout.zh-CN.png\`）将 PPS 标为 A4、GPS 报文标为 A8，A5/A9 标为 NC。这与此前对话中用户提供的另一张 24Pin 图存在冲突。

在本仓库中，主 README 以新导入工程的 A 面图作为当前工作基准，并明确保留冲突提示。任何自制 PCB 在上电前都应按实物插头方向、A/B 面和万用表连通性再次确认；不应把两张图的针脚定义拼接使用。

## 实测和未经验证的边界

上游 README 记录了 Jetson Orin Nano + ROS 2 Humble 的编译、静止收包和实机建图结果。它没有给出 Orange Pi 5B 或 RDK X5 的性能测试。因此：

- 不声称已经在 RDK X5 / Orange Pi 5B 上跑通；
- 不声称高速无人机或强振动环境下定位可靠；
- 不把 11V / 0.345A 的工作点当成额定输入范围；
- 不把 \`roi_en\` 的回读值单独当作 360° 解锁成功证明；
- PTP/GPS 时间模式、真实点时间、tag 语义、IMU 轴向和外参仍要在实际设备上核验。

## 后续开发建议

1. 在真实 T100 上复核 Type-C A 面引脚和电源启动电流。
2. 保存原始点云/IMU PCAP，核对时间戳、UDP 目的地址、点间隔、包序号和丢包。
3. 在 Ubuntu 22.04 + ROS 2 Humble 上复现上游 Orin Nano 路线。
4. 将同一套 C++ 驱动移植/编译到 RDK X5 或 Orange Pi 5B，记录 CPU 占用、包丢失、10Hz 延迟和 IMU 频率。
5. 用 FAST-LIO2 做静止、直线、转弯、原地旋转和重复路线测试。
6. 只有完成时间同步、外参、tag/line 和动态测试后，再评估无人机飞行使用。

## 文件与许可

导入子工程保留上游 GPL-2.0 LICENSE；其中 Livox 消息定义保留 MIT LICENSE.txt；FAST-LIO / ikd-Tree 的第三方来源见子工程 \`docs/SOURCE.md\`。根目录 Python 工具仍使用 T100 Non-Commercial License 1.0。许可边界见 [LICENSING.md](LICENSING.md)。
