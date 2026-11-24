# T100 激光雷达驱动与通信协议（Ethernet / UDP）

[English](README.md) · [完整协议](docs/protocol.zh-CN.md)

T100激光雷达驱动与通信协议工具：通过 Ethernet/UDP 获取点云和 IMU，读取或设置参数、控制采样启停、调整扫描角度，并提供固件写入模式入口指令。依据公开协议独立实现，非官方 SDK。包含本地 UDP 模拟雷达与浏览器预览。

演示中的房间、点云及 IMU 数据均为合成数据。

## 常见问题

### T100激光雷达驱动在哪里？

核心 Python 客户端提供 UDP 点云与 IMU 接收解析、控制命令和参数读写。另外新增 ros2/t100_livox_compat ROS 2 兼容包，可发布 Livox ROS Driver 2 的消息类型和默认话题。它不是内核驱动，也不模拟 Livox SDK2 的设备发现与控制 API。

### T100激光雷达刷固件：怎么进入固件写入模式？

本项目只提供 `00/07` 进入固件写入模式的指令。具体命令见[进入固件写入模式](#进入固件写入模式)。

### 大疆激光雷达协议：这个项目提供什么？

本仓库提供 T100 激光雷达 Ethernet/UDP 通信协议参考和独立实现，不是大疆官方 SDK，不代表兼容大疆全部型号的激光雷达。

### T100激光雷达点云怎么获取、角度怎么更改？

通过 UDP 接收点云与 IMU；用 `roi` 读取或调整扫描窗口。见[获取点云](#获取点云)、[更改角度](#更改角度)和[完整协议](docs/protocol.zh-CN.md)。

## 打开预览

Mac 上双击 `Start Demo.command`，或者执行：

```sh
python3 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -e .
.venv/bin/t100 demo --port 18766
```

浏览器打开 `http://127.0.0.1:18766`。程序需要 Python 3.9 及以上，运行时只使用 Python 标准库。Windows 对应命令在 `.venv\Scripts` 目录中。

预览支持采样启停、0–360° 单窗口设置、参数回读、3D 旋转和俯视、IMU、完整收发报文、CSV/JSON 导出和固件写入模式入口。模拟设备与网页均只绑定本地回环地址。

## 获取点云

```sh
t100 discover --seconds 5
t100 --ip 192.0.2.10 capture --bind 192.0.2.50 --seconds 10 --pcap capture.pcap
```

上面的地址仅为文档示例，实际使用时替换为设备和接收端地址。被动收包不会自动启动雷达，也不会自动修改网络配置。默认监听点云 `60001`、IMU `60003`，均可通过参数更改；设备的真实目的地址需另行核对。

```sh
t100 --ip 192.0.2.10 status
t100 --ip 192.0.2.10 read pcl_data_type
t100 --ip 192.0.2.10 --trace start
t100 --ip 192.0.2.10 stop
```

目标状态查询、实际状态推送和真实数据流分别观察。ACK 或参数读回本身不能证明电机、点云覆盖和重启结果。

## 更改角度

```sh
t100 --ip 192.0.2.10 roi
t100 --ip 192.0.2.10 roi --start 0 --end 180 --backup roi-before.json
t100 --ip 192.0.2.10 roi --disable --backup roi-before-disable.json
```

修改前保存 11 项 ROI 原值，先关闭窗口，逐项写入并回读，最后写入使能位。备份文件必须是新文件，避免覆盖先前记录。部分写入失败时，先核查状态，再决定如何恢复。底层 `write` 命令不代替调用方的配置备份。

## 进入固件写入模式

只生成完整请求帧：

```sh
t100 frame enter-write-mode --sequence 0x4810
```

结果：`55 0D 04 33 28 AD 10 48 40 00 07 76 F9`。

向实际设备发送进入固件写入模式的请求：

```sh
t100 --ip 192.0.2.10 enter-write-mode
```

网页的“进入固件写入模式”按钮向本地模拟设备发送 `00/07`。“恢复模拟采样”重置本地模型。上述 CLI 命令会向指定设备发送入口请求。

## 其他文件

- `extras/t100_protocol.h`：独立 C 控制帧及 CRC 辅助函数。
- `extras/wireshark/t100.lua`：Wireshark 字段解析插件，显示 CRC 字段，不执行 CRC 校验。
- `docs/`：公开协议快照与来源记录。
- `tests/`：公开报文样例、异常报文、UDP 模拟设备和本地服务测试。

PCAP 记录保留 UDP 载荷、接收时间和端点，IPv4/UDP 头由程序重建，不能当作完整链路层抓包。网页 CSV 是最近两秒内的有效模拟点。没有加入 ROS2、厂商 SDK 或未实现的平台支持描述。

## 使用许可

**仅限非商业使用，禁止商用，不提供商业授权。**

本仓库原创代码和文档采用 [T100 非商业使用许可 1.0](LICENSE)。允许在遵守条款的前提下进行非商业学习、研究、修改和分享；禁止用于商业产品或服务、收费维修、企业经营和商业研发。再分发时须保留版权与许可声明，修改版须标明修改。代码公开可见，不属于 OSI 定义的开源许可。英文 `LICENSE` 为完整条款，本段是中文摘要。

## ROS 2 / Livox MID360 接口兼容

新增的 ros2/t100_livox_compat 接收 T100 原生 UDP 点云和 IMU，并发布与 Livox ROS Driver 2 相同的默认接口：

| Topic | ROS 2 消息类型 |
|---|---|
| /livox/lidar | livox_ros_driver2/msg/CustomMsg |
| /livox/imu | sensor_msgs/msg/Imu |

接口同时提供两种官方驱动风格：`xfer_format:=1` 发布 CustomMsg（FAST-LIO2 通常使用此格式，且为默认值）；`xfer_format:=0` 发布官方 Livox PointXYZRTLT PointCloud2 字段布局。注意：这是 ROS 消息接口兼容，不是 SDK2 设备仿真。需要在同一个 ROS 2 工作空间构建官方 Livox ROS Driver 2，以提供 CustomMsg 和 CustomPoint 消息定义，但不需要启动官方 MID360 驱动节点。

### 编译

1. 在同一个 ROS 2 工作空间中构建/安装官方 Livox ROS Driver 2。
2. 将本仓库放入工作空间的 src/。
3. 以 Humble 为例：

```sh
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to t100_livox_compat
source install/setup.bash
```

### 运行

先将 T100 的点云和 IMU UDP 目的地址配置为 ROS 2 主机。默认监听端口为点云 60001、IMU 60003；实际目的端口应先通过抓包确认。默认不自动启动雷达采样：

```sh
t100 --ip <T100_IP> start
ros2 launch t100_livox_compat t100_mid360_compat.launch.py \
  device_ip:=<T100_IP> bind_ip:=<ROS_HOST_IP>
```

默认点云发布频率为 10 Hz。可用 `xfer_format:=1` 选择 CustomMsg，或 `xfer_format:=0` 选择官方 Livox PointCloud2 格式。检查接口：

```sh
ros2 topic info /livox/lidar -v
ros2 topic hz /livox/lidar
ros2 topic echo /livox/imu --once
```

### 兼容范围与当前验证状态

- 当前支持协议文档中的 96 点 Cartesian High 点云包（data_type=1）和单样本 IMU 包（data_type=0）。
- 两种输出格式都沿用 Livox ROS Driver 2 的点字段定义。PointCloud2 使用 x/y/z/intensity/tag/line/timestamp，点步长为官方 26 bytes。点时间偏移沿用 Livox SDK2 / ROS Driver 2 的计算方式：time_interval * 100 / dot_num，单位 ns。多个 UDP 包会组帧为 CustomMsg，使用 uint64 timebase 和 uint32 相对 offset_time。
- 默认 line_count=4，依据官方 MID360 驱动设置；T100 实际通道编号仍需用真实数据确认。
- T100 文档中的加速度单位为 g，发布到 ROS sensor_msgs/Imu 前转换为 m/s²；角速度保持 rad/s。
- 点云和 IMU 时间戳原样转发，以保留设备内部共同时间基准。若 time_type=0，时间为设备启动以来的时间，不是 Unix 时间。
- 当前版本有协议和单元测试，但尚未在真实 T100 硬件上验证。需要用真实 PCAP 检查目的端口、时间戳单调性、两路时间类型一致性、点间隔、line 映射和 IMU 单位。
- 当前不实现 Livox SDK2 设备发现/控制仿真、LVX 录制，也不覆盖官方驱动的全部配置功能。目标是保持下游 ROS 消息接口，而不是兼容所有 SDK 层功能。
