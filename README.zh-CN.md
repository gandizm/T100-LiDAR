# T100 激光雷达驱动与通信协议（Ethernet / UDP）

[English](README.md) · [完整协议](docs/protocol.zh-CN.md)

T100激光雷达驱动与通信协议工具：通过 Ethernet/UDP 获取点云和 IMU，读取或设置参数、控制采样启停、调整扫描角度，并提供固件写入模式入口指令。依据公开协议独立实现，非官方 SDK。包含本地 UDP 模拟雷达与浏览器预览。

演示中的房间、点云及 IMU 数据均为合成数据。

## 常见问题

### T100激光雷达驱动在哪里？

核心 Python 客户端提供 UDP 点云与 IMU 接收解析、控制命令和参数读写。原生 C++ ROS 2 替代驱动位于 ros2/livox_ros_driver2，提供 Livox ROS Driver 2 的消息接口和默认话题；ros2/t100_livox_compat 保留为 Python 参考桥接。它们都不模拟 Livox SDK2 的设备发现与控制 API。

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

PCAP 记录保留 UDP 载荷、接收时间和端点，IPv4/UDP 头由程序重建，不能当作完整链路层抓包。网页 CSV 是最近两秒内的有效模拟点。不包含厂商 SDK2 或其他平台支持；ROS 2 兼容驱动见下文。

## 使用许可

**仅限非商业使用，禁止商用，不提供商业授权。**

本仓库原创代码和文档采用 [T100 非商业使用许可 1.0](LICENSE)。允许在遵守条款的前提下进行非商业学习、研究、修改和分享；禁止用于商业产品或服务、收费维修、企业经营和商业研发。再分发时须保留版权与许可声明，修改版须标明修改。代码公开可见，不属于 OSI 定义的开源许可。英文 `LICENSE` 为完整条款，本段是中文摘要。

## ROS 2：让 T100 替代 MID360 的接入层

本仓库现在包含一个原生 C++ ROS 2 驱动包：ros2/livox_ros_driver2。它使用 T100 自己的 Ethernet/UDP 协议接收点云和 IMU，但对下游发布 Livox ROS Driver 2 的消息接口和默认话题：

| Topic | ROS 2 消息类型 |
|---|---|
| /livox/lidar | livox_ros_driver2/msg/CustomMsg |
| /livox/imu | sensor_msgs/msg/Imu |

它的设计目标是让 FAST-LIO2 等使用 Livox CustomMsg 的项目不必修改订阅消息类型和话题。启动时使用 T100 的 IP，不再启动官方 MID360 驱动，也不需要 Livox-SDK2。

### 兼容方式

- ROS 包名仍为 livox_ros_driver2，提供 CustomMsg.msg 和 CustomPoint.msg，并提供同名风格的 livox_ros_driver2_node 可执行程序。
- xfer_format=1 发布 CustomMsg，是 FAST-LIO2 推荐路径；xfer_format=0 发布 Livox PointXYZRTLT 布局的 PointCloud2。
- 点云按 publish_freq 聚合，默认 10 Hz；每点 offset_time 按 Livox MID360 驱动的公式计算：time_interval × 100 / dot_num，单位 ns。
- line 使用 point_index % 4，沿用 Livox MID360 的四线编号约定；T100 原始 tag 直接透传，不擅自改写。
- IMU 角速度按 rad/s 发布；T100 文档中的加速度单位为 g，发布前乘以 9.80665 转换为 m/s²。
- time_type=0 是设备启动后的时间。驱动使用同一个设备到主机时间偏移映射点云和 IMU 时间戳，保持两路相对同步；time_type=1/2 则使用设备同步时间戳。
- 默认自动把 T100 点云和 IMU 目的地址配置到当前主机，并在正常退出时恢复原设置；也会请求启动采样，退出时请求停止。

**重要：** 这里实现的是 ROS 消息/话题兼容，不是 Livox SDK2 设备协议仿真。依赖 SDK2 API、LVX 录制或官方专有控制接口的程序不能因此自动兼容。

### 编译

以 Ubuntu 22.04 + ROS 2 Humble 为例。仓库内已经包含一个精简的 livox_ros_driver2 消息接口包，因此不要再把官方 Livox ROS Driver 2 克隆到同一个工作空间；两个同名 ROS 包不能同时存在。

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

### 运行：尽量保持 FAST-LIO2 原配置不变

先确认 T100 与主机以太网连通，再启动驱动：

~~~bash
ros2 launch livox_ros_driver2 msg_T100_launch.py device_ip:=192.168.1.12
~~~

将 192.168.1.12 替换成实际 T100 IP。如果主机有多张网卡且自动路由选择不正确，可以显式指定 host_ip：

~~~bash
ros2 launch livox_ros_driver2 msg_T100_launch.py device_ip:=192.168.1.12 host_ip:=192.168.1.20
~~~

另一个终端启动 FAST-LIO2 ROS 2 分支：

~~~bash
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml rviz:=true
~~~

FAST-LIO2 的默认 /livox/lidar、/livox/imu 话题名和 CustomMsg 类型不需要改。T100 的安装位置与 MID360 不同，extrinsic_T / extrinsic_R 仍必须按实际安装重新标定。

检查数据：

~~~bash
ros2 topic info /livox/lidar -v
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
~~~

### 首次上机必须验证的项目

当前驱动已经实现协议解析、ROS 消息转换、设备目的地址配置、采样启停和数据聚合，但尚未在真实 T100 硬件上完成闭环验证。请先录制真实 PCAP，再验证：

1. 设备是否真的按配置的 host_pcl_ip/port、host_imu_ip/port 发包；协议文档提醒配置回读端口与实际目的端口可能不一致。
2. time_interval × 100 / dot_num 是否与当前固件版本的真实点间隔一致。
3. time_type=0 时点云与 IMU 是否共享同一个设备时钟；两路 time_type 不一致时驱动会拒绝混合发布。
4. T100 原始 tag 是否符合 FAST-LIO2 的 Livox 点筛选条件（tag & 0x30 为 0x10 或 0x00）。目前不强行改 tag，避免伪造点属性。
5. IMU 三轴方向、角速度单位、加速度比例和雷达到 IMU 的外参。
6. 实际点云包率、丢包、点密度、10 Hz 组帧延迟，以及 Orange Pi 5B / RDK X5 上的 CPU 占用。

如果 time_interval 为零或计算出的点间隔为零，驱动默认仍发布点云，但每点 offset_time 会相同；这不能视为已经具备可靠的运动去畸变。只有在实测确认后，才设置 point_interval_fallback_ns。

仓库还保留 ros2/t100_livox_compat 作为 Python 参考桥接实现；高点率和正式 FAST-LIO2 接入优先使用上面的原生 C++ 驱动。
