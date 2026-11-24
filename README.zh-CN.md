# T100 激光雷达驱动与通信协议（Ethernet / UDP）

[English](README.md) · [完整协议](docs/protocol.zh-CN.md)

T100激光雷达驱动与通信协议工具：通过 Ethernet/UDP 获取点云和 IMU，读取或设置参数、控制采样启停、调整扫描角度，并提供固件写入模式入口指令。依据公开协议独立实现，非官方 SDK。包含本地 UDP 模拟雷达与浏览器预览。

演示中的房间、点云及 IMU 数据均为合成数据。

## 常见问题

### T100激光雷达驱动在哪里？

核心 Python 客户端提供 UDP 点云与 IMU 接收解析、控制命令和参数读写。ROS 2 部分由 livox_ros_driver2 消息兼容包和 t100_livox_compat T100 UDP 节点组成，可保留常见 FAST-LIO2 的 ROS 消息/话题接口，但不模拟 Livox SDK2。

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

PCAP 记录保留 UDP 载荷、接收时间和端点，IPv4/UDP 头由程序重建，不能当作完整链路层抓包。网页 CSV 是最近两秒内的有效模拟点。ROS 2 兼容桥接包位于 ros2/，具体支持范围见下文。

## 使用许可

**仅限非商业使用，禁止商用，不提供商业授权。**

本仓库原创代码和文档采用 [T100 非商业使用许可 1.0](LICENSE)。允许在遵守条款的前提下进行非商业学习、研究、修改和分享；禁止用于商业产品或服务、收费维修、企业经营和商业研发。再分发时须保留版权与许可声明，修改版须标明修改。代码公开可见，不属于 OSI 定义的开源许可。英文 `LICENSE` 为完整条款，本段是中文摘要。

## ROS 2 / MID-360 接入替代

仓库现在提供两个 ROS 2 包：

- ros2/livox_ros_driver2：保留公开的 livox_ros_driver2/CustomMsg、
  CustomPoint 消息定义、常用可执行文件名称和 msg_MID360_launch.py 启动入口。
- ros2/t100_livox_compat：真正负责 T100 Ethernet/UDP 收包、参数配置、
  采样控制、点云转换和 IMU 发布。

这样现有 FAST-LIO2 工程可以继续使用原来的 Livox 消息类型和默认话题，
只替换 MID-360 设备驱动。注意：同一个 colcon 工作空间中必须移除上游
Livox-SDK/livox_ros_driver2，因为本仓库提供了同名兼容包，不能同时存在。
这个兼容包不包含，也不模拟 Livox SDK2。

### 编译（ROS 2 Humble / Jazzy）

~~~sh
git clone https://github.com/gandizm/T100-LiDAR.git ~/projects/T100-LiDAR
mkdir -p ~/ros2_ws/src
ln -s ~/projects/T100-LiDAR/ros2/livox_ros_driver2 ~/ros2_ws/src/livox_ros_driver2
ln -s ~/projects/T100-LiDAR/ros2/t100_livox_compat ~/ros2_ws/src/t100_livox_compat

cd ~/ros2_ws
source /opt/ros/humble/setup.bash   # Ubuntu 24.04 使用 jazzy
colcon build --symlink-install
source install/setup.bash
~~~

### 启动

~~~sh
ros2 launch livox_ros_driver2 msg_MID360_launch.py device_ip:=T100_IP host_ip:=ROS_HOST_IP
~~~

默认会配置并回读 T100 点云/IMU 的 UDP 目的地址、启动采样，并在正常退出时停止采样。
不填写 host_ip 时，节点会根据到雷达的路由自动选择本机 IPv4 地址。
如果已经手动配置过目的地址，可设置 configure_device:=false。
设置 restore_device_destinations:=true 可以在正常退出时恢复原来的点云/IMU 目的地址。

### ROS 接口

| 话题 | ROS 2 类型 |
|---|---|
| /livox/lidar | livox_ros_driver2/msg/CustomMsg（xfer_format=1） |
| /livox/lidar | sensor_msgs/msg/PointCloud2（xfer_format=0） |
| /livox/imu | sensor_msgs/msg/Imu |

常见 FAST-LIO2 Livox 配置可以继续使用 /livox/lidar、/livox/imu
和 Livox CustomMsg 输入。默认点云发布频率为 10 Hz。T100 原始 tag 和反射率会保留。
line 编号暂时沿用上游 MID360 驱动的 index modulo 4 规则；在把 line 当成真实
标定通道前，仍需确认 Agras 版本的实际通道映射。

PointCloud2 使用 Livox PointXYZRTLT 的紧凑 26 字节布局：
x/y/z/intensity 为 float32，tag/line 为 uint8，timestamp 为 float64。
timestamp 保存设备时间域中的绝对纳秒时间戳。

### 时间戳与验证

点间隔沿用官方 Livox ROS Driver 2 的 SDK2 回调计算方式：
time_interval * 100 / dot_num ns。可参考官方 pub_handler.cpp：
https://github.com/Livox-SDK/livox_ros_driver2/blob/master/src/comm/pub_handler.cpp

T100 协议使用相同的以太网数据头字段和 0.1 μs 时间间隔单位。
桥接程序保留包时间戳和逐点时间间隔，再聚合为 Livox CustomMsg。

节点会检查点云与 IMU 的 time_type 是否一致。对于 time_type=0，
协议描述的是设备本地时间域，而不是 Unix 时间；两路时间戳原样转发。
加速度从 g 转换为 m/s²，角速度保持 rad/s。

在把它用于无人机飞控定位前，仍需用真实 T100 PCAP 验证时间戳单调性、
点云/IMU 时间基准、丢包、实际点云和 IMU 频率，以及 T100 Agras 的 line 映射。
目前还没有真实 T100 硬件上的完整验证，也没有完成 FAST-LIO2 实机飞行测试。
它保留 ROS 接口，但不模拟 SDK2 的全部设备发现/控制功能，也不支持 LVX 录制。

协议转换辅助函数测试：

~~~sh
python -m unittest discover -s tests -v
~~~

另见 docs/ROS2_MID360_DROPIN.md 和 ros2/README.md。
