# 许可证边界

本仓库现在是一个包含多个独立组件的集合，不再适合用一句“整个仓库都是同一种许可证”概括。

| 路径 | 组件 | 许可证 |
|---|---|---|
| 根目录 \`src/t100_lidar/\`、\`extras/\`、\`tests/\` 及其原创文档 | 原有 Python 协议工具、模拟器和辅助代码 | T100 Non-Commercial License 1.0 |
| \`integrations/mid360_agras_t100_lidar_driver/\` | 导入的 MID360-agras / T100 驱动工程 | GPL-2.0，保留该目录内 LICENSE |
| \`integrations/.../ros2_ws/src/FAST_LIO/\` | FAST-LIO2 与 ikd-Tree | GPL-2.0，来源记录见该子工程 \`docs/SOURCE.md\` |
| \`integrations/.../ros2_ws/src/livox_ros_driver2/\` | Livox CustomMsg / CustomPoint 消息定义 | MIT，保留 \`LICENSE.txt\` |

## 重要说明

- 根目录 \`LICENSE\` 仅适用于原有且由本仓库版权方有权许可的原创材料；它不覆盖明确带有独立许可证的导入子工程。
- 不得删除导入子工程的 GPL-2.0 LICENSE、Livox MIT LICENSE.txt、版权声明或来源记录。
- 修改 GPL 子工程并对外分发时，必须遵守 GPL-2.0 的对应义务，包括提供相应源码和保留许可声明。
- 不要把根目录的非商业限制附加到 GPL-2.0 子工程，也不要把 GPL-2.0 误认为整个根目录 Python 工具自动变成 GPL。
- 如果未来把两套代码复制、链接或合并成一个不可分割的衍生作品，许可影响需要重新审查。此说明是工程层面的许可边界记录，不构成法律意见。

导入上游：AkiyamaYukina/mid360_agras_t100_lidar_driver，commit \`492b4a237741a051f6cd0eb118dabd4e1b756e90\`。
