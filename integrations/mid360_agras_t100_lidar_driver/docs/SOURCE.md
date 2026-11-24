# 来源记录

[English](SOURCE.en.md) · [返回首页](../README.md)

## 第三方代码

| 路径 | 上游 | 提交 | 许可证 | 改动 |
|---|---|---|---|---|
| `ros2_ws/src/FAST_LIO/` | [hku-mars/FAST_LIO](https://github.com/hku-mars/FAST_LIO)，`ROS2` 分支 | `cc3c9e8b1469b33b8072764d6db2348ed6dc597f`（2026-10-25） | GPL-2.0 | 源码未改动。为减小体积，没有放 `doc/` 里的演示动图、大图和论文 PDF，以及占位文件 `PCD/1`；新增空文件 `PCD/.gitkeep`。所以它的 README 里有些图片显示不出来。 |
| `ros2_ws/src/FAST_LIO/include/ikd-Tree/` | [hku-mars/ikd-Tree](https://github.com/hku-mars/ikd-Tree) | `e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4`（2022-02-08） | GPL-2.0 | 未改动。上游是 git 子模块，这里直接放文件，不需要执行 `git submodule`。 |
| `ros2_ws/src/livox_ros_driver2/msg/`、`LICENSE.txt` | [Livox-SDK/livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) | 与 `21445540f0d100dc86a7e6df312dd70bbdb4afdf`（2026-09-21）一致 | MIT | 消息定义未改动。同目录的 `CMakeLists.txt` 和 `package.xml` 是本仓库写的，只编译消息。包名沿用 Livox 的名字，因为 FAST-LIO 要包含 `livox_ros_driver2/msg/custom_msg.hpp`。 |

2026-11-24 用 git blob 哈希逐个比对过：除新增的 `PCD/.gitkeep` 外，本仓库里 FAST_LIO 和 ikd-Tree 的每个文件都和上游对应提交完全一致。

## 协议和实测

- [协议说明](protocol.zh-CN.md)写的是 `linux/mid360_agras.cpp` 和 `tools/mid360_fov.py` 的实际行为，加上 2026-11-05 到 2026-11-22 的实机测量。
- 2026-11-24 交叉核对：`tools/mid360_fov.py` 的 `encode()` 能逐字节复现 [sdcnm/T100-LiDAR](https://github.com/sdcnm/T100-LiDAR) 公开的 5 条请求报文（`32/12` 两条、`32/14`、`32/50`、`32/51`）。
- Type-C 引脚为实测；A4（PPS）和 A8（GPS 报文）的定义在 2026-11-22 加入，A8 没有验证过能用。

## 文档结构

文档的组织方式（中英文首页、`docs/` 下的协议说明和本文件）参考了 [sdcnm/T100-LiDAR](https://github.com/sdcnm/T100-LiDAR)，没有使用它的文字和代码。它采用自己的非商业许可，本仓库采用 GPL-2.0。
