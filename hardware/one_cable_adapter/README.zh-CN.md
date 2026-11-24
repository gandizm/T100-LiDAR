# T100 主动一线通转接器（Rev A 方案）

> 状态：电气架构与器件候选已确定；**不是可直接投板的最终 KiCad 原理图**。真实 T100 启动峰值电流、输入电压上限、RTL8153B 两对线工作方式及 USB 主机供电能力尚需实测。未完成这些验证前，不要按本文直接批量制作。

## 0. 设计目标

- 主机：RDK X5、Orange Pi 5B、普通 PC。
- 主机端：固定 USB 3.0 Type-A male pigtail；RDK X5、Orange Pi 5B 直接插 USB-A Host，只有 USB-C 的 PC 使用 USB-C-to-USB-A 3.0 转接头。
- 雷达端：T100 专用 24Pin Type-C plug，不能当普通 USB-C 使用。
- 一根 Cat5e 四对双绞线同时传输 100BASE-TX 和 11V 电源。
- 主机识别为 USB Ethernet 网卡；Linux 侧使用系统 r8152 驱动。
- 目标正常工作点：T100 端 11.0V，实测约 0.345A；该值是工作点，不是已确认的完整额定输入范围。

## 1. 总体原理

~~~text
PC / RDK X5 / Orange Pi 5B
          |
          | USB 3.0 Type-A Host
          | C-only PC: USB-C -> USB-A 3.0 adapter
          v
+---------------------------------------------------+
|             T100 主动转接器（host pod）           |
|                                                   |
| USB 3.0 Type-A captive plug                       |
|   |                                               |
|   +-- USB2 D+/D- ---------------------------------+
|   +-- USB3 SuperSpeed pairs ------------------+  |
|   |                                           |  |
|   |                                      RTL8153B|
|   |                                           |  |
| 5V VBUS -> input eFuse -> +5V_SYS -------------+  |
|                    |                              |
|                    +--> TPS61088 boost -> 11V     |
|                                           |       |
|                                output eFuse/OVP   |
|                                           |       |
|             +11V --------------------------+---- RJ45 4,5
|             GND ------------------------------- RJ45 7,8
|                                                   |
| RTL8153B MDI -> 10/100 magnetics -> RJ45 1,2,3,6 |
+---------------------------------------------------+
                                                    |
                                     Cat5e T568B 4-pair cable
                                                    |
                                   T100 custom Type-C plug
                                                    |
                                  11V + Ethernet -> T100
~~~

主机端是标准 USB 3.0 Type-A Host；雷达端是 DJI Agras 定制 Type-C。两端外形和电气定义完全不同。

## 2. 主机 USB 3.0 Type-A 输入

### J1：USB 3.0 Type-A male captive cable

第一版固定采用 USB 3.0 Type-A Host，原因是 RDK X5 和 Orange Pi 5B 都有明确的 USB-A Host 口；两块板的 Type-C 数据接口都不是本项目应该依赖的主机供电口。这样可以删掉主机端 Type-C 的 CC 控制器、SuperSpeed 翻转 MUX 和 MCU，减少器件、固件和故障点。

- USB 3.0 Type-A VBUS（5V）接输入 eFuse。
- USB GND 接板上 GND。
- USB2 D+/D- 直接接 RTL8153B USB2 数据端。
- USB3 SuperSpeed TX/RX 四组差分线直接接 RTL8153B。
- 连接器附近布置 USB2/USB3 低电容 ESD 保护。
- 使用带 SuperSpeed 线对的 USB 3.0 线缆，不要用只有 USB2 D+/D- 的充电线。

RDK X5：使用 USB 3.0 Type-A Host。它的 USB Type-C QuickLink 是 USB2 Device（ADB/Fastboot 等），另一个 Type-C 是整板 5V/5A 电源输入，不要把它们当作外设 Host 口。

Orange Pi 5B：使用 USB 3.0 Type-A Host。其 5V/4A Type-C 是整板电源输入；中间的 Type-C 数据口用于设备连接/烧录等，不把它作为本版的供电 Host 口。

普通 PC：优先 USB 3.0 Type-A；如果只有 USB-C，使用支持 USB 3.x 的 USB-C-to-USB-A adapter。该适配方式通常按 USB 3.x legacy 供电预算工作，不会自动取得 USB-C 1.5A/3A 电流广告。

### 输入电流限制

不使用 TUSB320/PD 控制器。输入 eFuse 预留两个硬件档位：
- GENERIC：0.90A，适用于标准 USB 3.x Type-A Host；
- RDK：1.00A，仅在 RDK X5 已确认的 1A USB3 端口使用。

通过 0Ω 电阻/焊桥选择档位，禁止把 1A 档当作所有 PC USB-A 口的默认设置。USB2.0 Host（500mA 档）不支持本方案的单线供电。

## 3. USB Ethernet 芯片

### U4：Realtek RTL8153B

选择理由：
- USB 3.0 host interface；
- 10/100/1000 Ethernet PHY；
- Linux r8152 驱动生态成熟；
- T100 只需要 100BASE-TX，带宽不需要千兆，但 USB 3.0 总线有利于主机侧供电额度。

**关键约束：** T100 是 100BASE-TX，只用两对数据线。主板使用普通 RJ45 jack（无内置磁性器件），外接两通道 10/100 磁性器件。RTL8153B 需要配置/协商到 100Mbps；其额外千兆 MDI 通道不接到 RJ45 电源线。必须在首版样机上验证 RTL8153B 在两对 100BASE-TX 磁性器件下能稳定建链。不要把四对千兆 MagJack 直接拿来做备用线对供电。

RTL8153B 的供电轨、时钟、复位、EEPROM/OTP 配置及去耦，必须遵循所采购芯片具体 revision 的 Realtek reference design；不要只凭通用 USB 网卡模块图片猜引脚。

## 4. Ethernet 磁性器件与 RJ45

### T1：Pulse H1102NL（候选）

H1102NL 是单口 10/100BASE-TX 1:1 磁性器件，含 TX/RX 两个变压器通道。按其数据手册的 PHY-side / cable-side 引脚连接，并遵循 RTL8153B PHY 参考设计处理中心抽头和终端网络。

RJ45 J2 必须是**无内置磁性器件的普通 8P8C 插座**。数据对通过 T1；备用线对直接用于直流供电：

- RJ45 1、2：橙色 Ethernet pair；
- RJ45 3、6：绿色 Ethernet pair；
- RJ45 4、5：蓝色 pair，直接接 +11V；
- RJ45 7、8：棕色 pair，直接接 GND。

不要在 RJ45 4/5、7/8 上放 Bob-Smith 终端电阻、LED、磁性器件绕组或其他到地网络，否则会给 11V 供电造成负载/短路。数据对的中心抽头终端只按 H1102NL 和 RTL8153B 参考设计处理。

## 5. 5V -> 11V 升压与保护

### U5：输入 eFuse，TPS25947xx

放在 USB VBUS 与 +5V_SYS 之间：
- 反向电流阻断；
- 输入过压/欠压；
- 可编程限流；
- 可调软启动；
- FAULT / POWER-GOOD 输出；
- ILIM 档位由 MCU 根据 TUSB320 current-mode detection 选择。

电流限值通过 0Ω 档位选择：GENERIC 0.90A、RDK 1.00A。最终 ILIM 电阻按实际订购的 TPS25947xx 数据手册/计算器确定，不要从其他 TPS2594x 型号照抄。

### U6：TPS61088 同步 Boost

目标：5V 输入，11.0V 输出，连续输出能力按 0.5A 设计目标预留。

初步计算值（需 WEBENCH/仿真及实测复核）：

| 项目 | Rev A 起始值 | 说明 |
|---|---:|---|
| VOUT | 11.0V | T100 实测工作点 |
| RFB_TOP | 245kΩ | VOUT -> FB |
| RFB_BOTTOM | 30.1kΩ | FB -> AGND |
| VREF | 1.204V | TPS61088 PWM reference |
| L1 | 4.7µH | Isat ≥4A，低 DCR 屏蔽电感 |
| FSW | 600kHz | 由 FSW-SW 电阻设定 |
| RFREQ | 255kΩ（初值） | 按 5V/11V/600kHz 公式计算 |
| RILIM | 301kΩ（初值） | PFM 公式约 3.95A nominal switch peak limit |
| CIN | 2×22µF / 10V X7R + 100nF | 注意 5V DC bias 后的有效容量 |
| COUT | 3×22µF / 25V X7R + 100nF | 注意 11V DC bias 后的有效容量 |
| CVCC | 1µF | 靠近 VCC |
| CBOOT | 100nF | BOOT-SW |
| CSS | 100nF | 约 24ms 软启动初值 |
| COMP | 待 WEBENCH | 不照抄 3.3V->9V/3A 参考设计 |

TPS61088 的 FB reference 是 1.204V，不是 0.6V。RFB_TOP=245kΩ、RFB_BOTTOM=30.1kΩ 时，理论输出约 11.0V。

RFREQ 按 TI 公式：
RFREQ = 4 × (1/fSW − tDELAY × VOUT/VIN) / CFREQ
其中 CFREQ=23pF、tDELAY=89ns。以上 255kΩ 是 5V、11V、600kHz 的初始近似值。

MODE 保持悬空以选择 PFM light-load 模式。RILIM 按 PFM 公式：
RILIM = 1,190,000 / ILIM
301kΩ 对应约 3.95A nominal switch peak limit。TI 指出器件公差和温度下最小限流可能比计算值低约 1.3A；需按最低 VIN、最大 VOUT/IOUT、L 容差和最低 FSW 重新核算。以 5V 输入、11V 输出、0.5A 输出、85% 效率、L=4.7µH（按 -30% 容差）及 600kHz 估算，电感峰值约 2.05A；301kΩ 对应的最坏情况限流约 2.65A，初步有余量，但最终仍需按实际器件和温度验证。

### U7：输出 eFuse，TPS25947xx

放在 Boost 输出与网线供电线对之间：
- 过流/短路保护；
- 软启动/输出斜率；
- 可调 OVLO；
- 输出限流初值约 0.7A，需以实测 T100 启动峰值校准；
- 11.8V 仅作为初步 OVP 设计目标，**不是 T100 已确认的最大允许输入电压**。在确认雷达 VIN 最大值前，不得把该阈值当作安全额定值。

Boost 本身的 13.2V OVP 高于 T100 已知 11V 工作点太多，因此不能只依赖 TPS61088 内部 OVP。

## 6. RJ45 -> T100 Type-C 线缆针脚

使用 T568B、24AWG 纯铜 Cat5e 四对双绞线。以下是雷达端 plug 的 A 面接点定义，按上游实测 A 面图：

| RJ45 针脚 | T568B 颜色 | 线缆用途 | T100 Type-C A 面 |
|---:|---|---|---|
| 1 | 橙白 | ETH1 数据对 | A10 |
| 2 | 橙 | ETH1 数据对 | A11 |
| 3 | 绿白 | ETH2 数据对 | A2 |
| 6 | 绿 | ETH2 数据对 | A3 |
| 4 | 蓝 | +11V | A6 |
| 5 | 蓝白 | +11V | A7 |
| 7 | 棕白 | GND | A1 |
| 8 | 棕 | GND | A12 |

线对分配：
- 橙色 pair（1/2）：Ethernet；
- 绿色 pair（3/6）：Ethernet；
- 蓝色 pair（4/5）：两芯并联 +11V；
- 棕色 pair（7/8）：两芯并联 GND。

A4、A8、A5、A9 不接。不要把旧版图片中 PPS/GPS/IO 的冲突定义混入这条供电线。T100 插头只按新上游工程的 A 面针脚图制作；不要在转接板上把 A、B 两面对应触点再短接。制作后先逐芯导通检查，再接雷达。

建议先用 1–3m 24AWG 纯铜线验证；之后再测 5m/10m 的压降和丢包。禁止使用 CCA 铜包铝线。

**这不是 IEEE PoE。** RJ45 的 4/5、7/8 带有 11V 被动电源。该 RJ45 只能接专用 T100 线，绝不能插普通交换机、路由器、电脑网口或标准 PoE 设备。

## 7. 主机兼容性与供电预算

T100 上游实测约 11V × 0.345A = 3.80W。仅雷达本身在 90% boost 效率下，就需要约 0.84A 的 5V 输入；再加 RTL8153B 和转换损耗，整机输入可能接近 0.9–1.0A，启动峰值尚未测量。

因此：
- USB 3.0 Type-A 的标准 900mA 供电档可能处于临界区；
- RDK X5 的 USB 3.0 Type-A 端口标称约 5V/1A，仍需实测整机启动电流；
- Orange Pi 5B 的板级 5V/4A 电源输入不等于每个 USB 端口都能输出 4A，必须实测其 USB host 端口；
- USB 2.0 host（500mA 档）不在单线供电保证范围内。

本设计能让三类主机使用同一套线缆/模块，但**不能保证任何一个 USB 口都能提供足够的 5V 功率**。RDK X5 的 USB3 口标称 1A；Orange Pi 5B 的 USB 端口单口输出电流没有在手册中明确给出；普通 PC USB3-A 通常是 900mA 档。若主机只给 500mA，物理上就无法稳定输出 3.8W 雷达功耗。输入 eFuse 应在供电不足时限流/关断，而不是让主机 USB 口反复掉电。

预留一个不装连接器的 5V_AUX 测试焊盘和反灌隔离位置，正常一线通不接；仅用于台架定位供电不足问题。若普通 PC USB3-A 不能稳定供电，下一版再做 USB-C UFP 直接输入（增加 TUSB320 + SuperSpeed MUX），利用源端 1.5A/3A current advertisement；这不是第一版的必需器件。

## 8. 接地与隔离注意

H1102NL 的数据变压器提供数据对隔离，但如果 11V 电源回流线直接接回 USB GND，就会通过供电回路把主机地与 T100 地连接起来，整体不再具有完整的 Ethernet galvanic isolation。

同一台机器人/无人机内部、主机与雷达共用电池地时，非隔离供电通常更简单；如果 PC、雷达和其他设备还有不同的保护地/电源地连接，必须评估地环路和浪涌风险。需要保持完整隔离时，应改用隔离型 5V->11V DC-DC，但会增加体积、成本和损耗，且进一步提高 USB 侧输入电流。

## 9. PCB 布局规则

1. USB 3.x SuperSpeed 按 90Ω 差分阻抗布线，MUX 靠近 Type-C 连接器；USB2 D+/D− 按 90Ω 差分。
2. RTL8153B MDI 到 H1102NL 按 100Ω 差分；变压器靠近 RJ45。
3. RJ45 4/5、7/8 只走电源，不进入磁性器件、Bob-Smith 网络或 PHY。
4. TPS61088 的 VIN 电容、L1、SW、VOUT 电容紧凑放置；SW 铜皮尽量小。
5. Boost 电感和 SW 节点远离 RTL8153B、H1102NL、USB3 MUX 和 Ethernet 差分线。
6. +11V/GND 电源线加宽，按 0.7A 保护限值设计；RJ45 电源针脚处放置输出电容和瞬态保护。
7. USB-C、RJ45 两端分别放置低电容 ESD 保护器件。

## 10. 首版验收顺序

1. **先测真实 T100**：11V 台式电源下记录冷启动峰值、稳态电流、启动时间和工作温升。
2. **先做 USB NIC 板**：验证 RTL8153B + 两对 H1102NL 在 100BASE-TX 下稳定建链，确认 4/5、7/8 与 PHY 完全隔离。
3. **验证供电注入**：先用电子负载代替 T100，在 RJ45 4/5、7/8 输出 11V，检查短路、过流、热插拔和压降。
4. **验证一线通**：接 T100，确认 USB NIC 枚举、100M 链路、UDP 60001/60003 连续收包。
5. **测主机功耗**：分别在 RDK X5、Orange Pi 5B、普通 PC 上测 USB VBUS 电压、电流、启动是否掉电。
6. **最后才画最终 PCB**：用真实启动峰值调整输入 eFuse、输出 eFuse、软启动和 OVP。

## 11. 参考资料

- [整合仓库的 T100 A 面 pinout 图片](../../integrations/mid360_agras_t100_lidar_driver/docs/images/pinout.zh-CN.png)
- [T100 上游协议与实测说明](../../integrations/mid360_agras_t100_lidar_driver/docs/protocol.zh-CN.md)
- [Realtek RTL8153B](https://www.realtek.com/Product/Index?id=4078)
- [TI TUSB320LAI datasheet](https://www.ti.com/lit/ds/symlink/tusb320lai.pdf)
- [TI TPS61088 datasheet](https://www.ti.com/lit/ds/symlink/tps61088.pdf)
- [TI TPS25947 product/datasheet](https://www.ti.com/product/TPS25947)
- [Pulse H1102NL datasheet](https://productfinder.pulseeng.com/doc_type/WEB301/doc_num/H1102NL/doc_part/H1102NL.pdf)
