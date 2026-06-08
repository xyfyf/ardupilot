# Air780E + ArduPilot LTE 4G 数传与 UOM 云平台完整集成指南

> 版本：2026-06-08（合并版）
> 适用固件：ArduPilot 4.7+ / ChibiOS / STM32H7 系列飞控（以 EFT_CAAC 为例）
> 模组：合宙 Air780E（**必须是标准 AT 固件**）
> 核心脚本：`LTE_modem.lua`（部署到 SD 卡 `APM/scripts/`）
> 目标读者：飞控研发工程师、外场测试工程师、无人机商业化决策人员

本文档由一线飞控研发工程师总结，详细记录了在 STM32H7 平台（以 EFT-CAAC 为例）上集成合宙 Air780E 4G 模组（AT固件）的全流程。本指南不仅涵盖了 `LTE_modem.lua` 脚本部署与 PPP 网络栈配置，还深度剖析并解决了 STM32H7 芯片底层 DMA 锁死勘误与 ADC 硬件冲突导致的数传丢包难题，并完整给出了对接 **UOM MQTT 云平台**的激活与遥测流程。

---

## 一、概览与五个 ID 速查（UOM 平台）

对接 UOM/MQTT 时，平台文档与 `LTE_modem.lua` 使用下列 **5 个 ID**（请先读本节，再往下看接线与参数）。

| 变量名 | 说明 | 示例 | 在协议中的位置 |
| :--- | :--- | :--- | :--- |
| **vendor_id** | 厂商 ID，用于区分不同厂家接入 | `eft` | **MQTT 主题路径**中的固定段，例如 `uav/up/telemetry/eft/{fcu_id}`；**不在** JSON 报文里 |
| **fcu_id** | 飞控设备在平台上的唯一标识符 | `EFT2605210001` | 激活 JSON `{"fcu_id":"…","timestamp":…}`；主题后缀 `{fcu_id}`；MQTT Client ID |
| **uas_id** | 航空器实名登记号（机体 ID，Remote ID） | 常与 fcu_id 相同 | 遥测 JSON 字段 `uas_id`；MP 写入 `LTE_UAS_W01~10` |
| **operator_id** | 运营人/操控员登记号（CAA 类） | `202605250941` | 遥测 JSON 字段 `operator_id`；MP 写入 `LTE_OP_W01~10` |
| **user_id** | 用户/自我声明（展示用，非强制实名 ID） | 任务说明文字 | 遥测 JSON；优先 MP 的 **SELF_ID** 描述，否则参数 `LTE_USER_ID` |

**关系摘要**

- **vendor_id**：本方案固定为 **`eft`**，写在主题里，脚本 `uom.TOPIC_*` 已写死；换厂商须改脚本并与云平台约定路径。
- **fcu_id** 与 **uas_id**：脚本里 **fcu_id = uas_id**（同一字符串）；未配置 uas_id 时 fcu_id 退化为 **`default_sn`**（多台飞控切勿共用）。
- **operator_id**、**user_id**：仅出现在**遥测 JSON**，不参与 MQTT 主题命名。
- 另有 **op_lat / op_lng / op_alt**（操控员经纬高），属于位置字段，不是 ID。

**配置入口**：Mission Planner → Remote ID / OpenDroneID 下发 BASIC_ID、OPERATOR_ID、SELF_ID、SYSTEM；飞控持久化到 `LTE_UAS_*`、`LTE_OP_*`、`LTE_OP_LAT/LNG/ALT`。

---

## 二、硬件准备

### 2.1 模块选型

| 项目 | 要求 |
| :--- | :--- |
| **模块型号** | 合宙 Air780E（基于紫光展锐 EC618 芯片） |
| **固件类型** | **必须是标准 AT 固件**（不能是 DTU 固件或 LuatOS 固件）。`LTE_modem.lua` 脚本依赖 AT 指令集（如 `AT+CPIN?`、`ATD*99#`）来控制模块，DTU/LuatOS 固件无法正确响应这些指令 |
| **SIM 卡** | Nano SIM，确保已实名认证、已激活、有流量余额、未欠费 |
| **天线** | 必须接好 4G LTE 天线（IPEX 接头扣紧），不接天线会导致射频功放反射烧毁或模块死机 |
| **供电** | 建议**独立 5V 供电**（≥500mA），避免与飞控共用 BEC 触发拉低 |

### 2.2 接线方法

**⚠️ 极度隐蔽的硬件级大坑（决定走哪个 SERIAL 口）：**
原本 4G 模块应该接在常规的数传接口（如 `COM1`），但在 EFT-CAAC 等采用 STM32H743 的部分飞控上，`COM1` 对应的物理引脚（PA2/PA3）与底层 ADC 引脚存在硬件复用冲突。如果在该口开启 DMA 进行高速 PPP 通信，会触发芯片级 DMA 锁死勘误（**Errata 2.20.6**），导致疯狂丢包、死机、以及无法联网（详见第七节"坑 7"）。

**推荐做法**：把 Air780E 接到 **`USART1`（PA9/PA10，丝印通常是 GPS1）**。在 EFT_CAAC 出厂固件里这一路是 **`SERIAL3`**。
**本文后续参数统一按 `SERIAL1_*` 编写**：请把 Air780E 接到**你这块飞控在 `hwdef.dat` 的 `SERIAL_ORDER` 里排在第 1 路的那组 UART**（丝印可能是 RSV1、Telem1 等，以原理图为准）。EFT_CAAC 出厂映射：`SERIAL1 = RSV1（USART2，PA2/PA3）`，若你实际接在 GPS1（USART1，PA9/PA10），需把全文中 `SERIAL1_*` / `BRD_SER1_*` 全部改为 `SERIAL3_*` / `BRD_SER3_*`。

```
飞控 SERIALn 口     Air780E 开发板
─────────           ──────────────
   TX  ──────────>     RX
   RX  <──────────     TX
   GND ──────────>     GND
   5V  ──────────>     5V (建议独立 5V)
```

### 2.3 CTS / RTS 流控线

**不需要接，也不能接。**

Air780E 的大多数转接板/开发板没有引出 CTS/RTS 引脚。
同时必须在飞控参数中关闭硬件流控：

```
BRD_SER1_RTSCTS = 0
```

如果此参数为 1 或 2（自动），飞控会因为检测不到 CTS 信号而锁死串口，导致脚本报 `could not find serial port`。

---

## 三、固件编译（关键前置条件）

### 3.1 PPP 网络支持 + DMA 独占

ArduPilot 默认的 EFT_CAAC 固件**不包含 PPP 网络协议栈**。
如果不添加 PPP 支持就直接使用 `LTE_PROTOCOL = 48`（PPP 模式），会出现死循环：

```
LTE_modem: connected     ← 模块拨号成功，回复 CONNECT
(10秒沉默)                ← 飞控底层没有 PPP 驱动，无法进行 LCP/IPCP 协商
LTE_modem: timeout       ← 脚本判定超时
LTE_modem: sent reset    ← 强制重启模块
LTE_modem: step ATI      ← 重新开始，无限循环
```

### 3.2 修改 hwdef.dat

在 `libraries/AP_HAL_ChibiOS/hwdef/EFT_CAAC/hwdef.dat` 文件末尾添加：

```
# 开启 PPP 网络后端，支持 LTE 模块
define AP_NETWORKING_BACKEND_PPP 1

# 给 LTE 串口分配独占 DMA 通道，规避 H743 ADC/DMA 冲突
DMA_NOSHARE SPI1* SPI2* USART1*
```

> **缺少第一行**：PPP 驱动不存在，`connected` 后立即 `timeout` 死循环。
> **缺少第二行**：H743 DMA 冲突导致 `PPP[0]: reconnecting` 死循环，永远拿不到 IP。

### 3.3 重新编译并刷入固件

```bash
cd ardupilot
./waf configure --board EFT_CAAC
./waf copter
```

编译产物位于 `build/EFT_CAAC/bin/arducopter.apj`，通过 Mission Planner 的 "Load custom firmware" 刷入飞控。

> **注意**：如果使用 ArduPilot 在线自定义固件构建器 (<https://custom.ardupilot.org>)，需要在构建选项中勾选 `Networking` 和 `PPP` 两个 Feature。

---

## 四、参数配置（分阶段，每阶段之间必须重启）

### 阶段一：开启 Lua 脚本引擎和物理串口

```
SCR_ENABLE       = 1         # 启用 Lua 脚本
SCR_HEAP_SIZE    = 409600    # Lua 堆：LTE_modem+UOM+禁飞区共用；勿超 ~450000 会整机 OOM
SERIAL1_PROTOCOL = 28        # 将 SERIAL1 交给 Lua 脚本控制（28 = Scripting）
SERIAL1_BAUD     = 115       # 波特率 115200
SERIAL1_OPTIONS  = 0         # 无特殊选项
BRD_SER1_RTSCTS  = 0         # 关闭硬件流控
SCR_SDEV_EN      = 1         # 启用脚本虚拟串行设备
```

**断电重启飞控。**

### 阶段二：配置虚拟串口协议，上传脚本

重启后参数表中会出现 `SCR_SDEV1_PROTO` 等新参数。

```
SCR_SDEV1_PROTO = 48        # 虚拟串口 1 协议设为 PPP
NET_ENABLE      = 1         # 启用 ArduPilot 网络栈（极其关键！没有这个 PPP 不会工作）
```

同时将 `LTE_modem.lua` 文件上传到飞控 SD 卡的 `APM/scripts/` 目录。

**再次断电重启飞控。**

### 阶段三：配置 LTE 脚本参数

重启后参数表中会出现 `LTE_` 开头的参数。

```
LTE_ENABLE      = 1         # 启用 LTE 驱动
LTE_SERPORT     = 0         # ⚠️ 坑！这里填 0，不是 1（见第七节坑 1）
LTE_SCRPORT     = 0         # ⚠️ 坑！这里填 0，不是 1（见第七节坑 1）
LTE_PROTOCOL    = 48        # PPP 模式（必须与 SCR_SDEV1_PROTO 一致）
LTE_BAUD        = 115200    # 数据传输波特率
LTE_IBAUD       = 115200    # 模块初始波特率
LTE_TIMEOUT     = 10        # 连接超时（秒），0=禁用
LTE_OPTIONS     = 0         # 正常；调试时设 63 开启全量串口日志
```

**断电重启飞控。**

### 阶段四：UOM MQTT 云平台参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `LTE_UOM_ENABLE` | **1** | 启用 UOM MQTT 上报 |
| `LTE_UOM_IP0` | **47** | MQTT Broker IP 第 1 段 |
| `LTE_UOM_IP1` | **120** | 第 2 段 |
| `LTE_UOM_IP2` | **16** | 第 3 段 |
| `LTE_UOM_IP3` | **113** | 第 4 段（默认 47.120.16.113） |
| `LTE_UOM_PORT` | **1883** | MQTT 端口 |

> 如需修改 MQTT 账密，编辑 `LTE_modem.lua` 中 `UOM_MQTT_USER` / `UOM_MQTT_PASS`。

### 阶段五：Remote ID / OpenDroneID（写入 UAS ID）

在 MP 的 **Remote ID** 标签页（或 Initial Setup → Mandatory Hardware → Remote ID）配置：

| 字段 | 说明 | 对应 LTE 参数 |
|------|------|--------------|
| **UAS ID（BASIC_ID）** | 实名登记号，最多 20 字符，如 `EFT2605210001` | `LTE_UAS_W01~10`（自动持久化） |
| **Operator ID** | 运营人编号，如 `202605250941` | `LTE_OP_W01~10` |
| **Self ID（描述）** | 自我声明，可空 | `odid.self_desc`（内存） |
| **System（操控员位置）** | 地面站 GPS 坐标 | `LTE_OP_LAT/LNG/ALT` |

> ⚠️ UAS ID 长度须 **≥ 12 字符**，否则脚本等待完整 ID 后再订阅。

Remote ID 模块参数：

```
DID_ENABLE   = 1
DID_MAVPORT  = 4
```

### 阶段六：网络出口（公网 UDP 直连，可选，与 UOM 并行）

若同时需要把 MAVLink 通过 4G 直接转发给私有公网服务器（用 Mission Planner 远程连接），还需要配下面这组：

```
NET_P1_TYPE     = 1         # UDP Client（飞控主动向服务器发送 UDP 数据包）
                            # 可选值: 0=Disabled, 1=UDP client, 2=UDP server,
                            #         3=TCP client, 4=TCP server
NET_P1_PROTOCOL = 2         # MAVLink2
NET_P1_IP0      = x         # 服务器公网 IP 第 1 段
NET_P1_IP1      = x         # 第 2 段
NET_P1_IP2      = x         # 第 3 段
NET_P1_IP3      = x         # 第 4 段
NET_P1_PORT     = 14550     # 服务器监听端口
```

> 公网服务器（relay.py）搭建详见第十一节。

> 禁飞区脚本相关：`NFZ_PAGE_SIZE = 10`（HTTP 每页条数；勿 9999，JSON 会过大）。

---

## 五、SD 卡脚本部署

```
/APM/scripts/
  └── LTE_modem.lua       ← 唯一需要放的文件（包含 PPP 拨号 + UOM MQTT）
```

无需其他脚本配合 UOM 功能。

---

## 六、正常启动日志（参考）

从开机到 PPP 联网 + UOM 激活完成约 **30~60 秒**（含 GPS 冷启动）：

```
LTE_modem: starting                    ← Lua 脚本启动
LTE_modem: step ATI                    ← 正在串口上寻找模块
LTE_modem: sent reset                  ← 首次上电先软复位退出 PPP 模式
LTE_modem: found modem: Air780         ← 识别到 Air780E（AT 固件）
LTE_modem: step BAUD                   ← 设置/确认波特率
LTE_modem: step CPIN                   ← 检查 SIM 卡
LTE_modem: step CONFIG                 ← 配置频段/运营商
LTE_modem: step CREG                   ← 检查蜂窝网络注册
LTE_modem: CREG OK                     ← 成功注册到基站
LTE_modem: step CGACT                  ← 激活 PDP 上下文
LTE_modem: CGACT OK                    ← 网络上下文激活成功
LTE_modem: step PPPOPEN                ← 发送 ATD*99# 拨号
LTE_modem: connected                   ← 模块回复 CONNECT，PPP 数据链路建立
PPP[0]: reconnecting                   ← 飞控底层 PPP 驱动开始 LCP/IPCP 协商（正常）
NET: IP      10.x.x.x                  ← 运营商分配了 IP 地址
NET: Mask    255.255.255.255
NET: Gateway 10.x.x.x
EKF3 IMU0 origin set                   ← GPS 3D Fix（时间戳变为有效值）
UOM: MQTT connected                    ← 成功连接 MQTT Broker
UOM: 已发送订阅 uav/down/activation/eft/EFT2605210001
UOM: 进入激活阶段 (SUBACK)             ← 正常；若显示 SUBACK timeout 也可继续
UOM: UP uav/up/activation/eft/EFT2605210001 ts=177[9xxxxxxxxx]
                                       ← ts= 开头 177… 是 50 字节 GCS 截断，
                                          下一行是剩余数字，合并为 13 位 Unix 毫秒
UOM: DOWN uav/down/activation/eft/EFT2605210001
UOM激活[0]: 激活成功
UOM: 设备激活成功，可以开始上报数据
UOM: 激活完成，开始上报遥测
UOM#1  uas=EFT2605210001 op=202605250941
UOM#1  olat=31.xxxxx olng=117.xxxxx
... (每 10 秒打印计数)
```

看到 `NET: IP` 即代表飞控已通过 4G 联网；看到 `UOM激活[0]: 激活成功` 即代表 UOM 云平台对接成功。

---

## 七、核心踩坑记录

### 坑 1：`LTE_SERPORT` 和 `LTE_SCRPORT` 的编号含义

**这是最容易犯的错误。**

`LTE_SERPORT` 的值**不是** `SERIALn` 中的 `n`！
它是 **"飞控中第几个协议被设置为 28 (Scripting) 的串口"**，**从 0 开始数**。

| 你的配置 | Lua 脚本看到的索引 |
| :--- | :--- |
| 只有 `SERIAL1_PROTOCOL = 28` | 它是第 1 个 Scripting 串口 → 索引 **0** |
| `SERIAL1` 和 `SERIAL2` 都设为 28 | SERIAL1 索引 0，SERIAL2 索引 1 |

同理，`LTE_SCRPORT` 是虚拟串口的索引（从 0 开始）。
`SCR_SDEV1` 对应索引 **0**，`SCR_SDEV2` 对应索引 **1**。

**如果你只有一个 Scripting 串口和一个虚拟串口，两个都填 0。**

错误设置（填 1）会导致：`LTE_modem: could not find serial port`

### 坑 2：固件缺少 PPP 支持

标准 EFT_CAAC 固件不包含 `AP_NETWORKING_BACKEND_PPP`。
不编译进去就使用 PPP 模式，表现为：反复 `connected` → 10 秒 `timeout` → `reset` → 死循环。
必须在 `hwdef.dat` 中添加 `define AP_NETWORKING_BACKEND_PPP 1` 并重新编译。

### 坑 3：`NET_ENABLE` 未开启

即使固件编译了 PPP 支持，如果 `NET_ENABLE` 不等于 1，PPP 驱动不会启动。
表现与坑 2 完全相同（`connected` 后 `timeout`）。

### 坑 4：硬件流控未关闭

`BRD_SER1_RTSCTS` 默认可能是 2（自动检测）。
Air780E 没有 CTS/RTS 引脚，流控会锁死串口。
必须手动设为 0。

### 坑 5：AT 固件 vs DTU/LuatOS 固件

Air780E 有多种固件版本。**必须使用标准 AT 固件。**
DTU 固件和 LuatOS 固件无法响应 `LTE_modem.lua` 中使用的标准 AT 指令集（如 `ATI`、`AT+CPIN?`、`AT+CGACT`、`ATD*99#`）。
如果模块运行了错误的固件，飞控会一直卡在 `step ATI`（完全找不到模块）。

### 坑 6：SIM 卡未插好或未激活

表现为：`found modem: Air780` 之后，一直卡在 `step CPIN`（疯狂重复）。
脚本在发送 `AT+CPIN?` 等待模块回复 `+CPIN: READY`，但因为 SIM 卡未就绪而无法通过。
排查：检查卡的方向、是否插紧、是否欠费、是否需要实名激活。

### 坑 7：STM32H7 飞控串口 DMA 冲突导致 PPP 握手失败（`PPP[0]: reconnecting` 死循环）

**极度隐蔽的硬件级大坑！**
在一些 STM32H743 飞控（如 EFT_CAAC）上，如果将 LTE 模块接在复用了 ADC 引脚的串口（例如 `USART2` 对应的 `PA2/PA3` 引脚，同时也是 `ADC1_IN14/15`），会因为 **STM32H743 芯片勘误 (Errata 2.20.6)** 和 ADC DMA 并发冲突，导致串口 DMA 流锁死或频繁丢包。

**表现：**
- 现象 1（强行开 DMA）：刚开机就卡死在 `LTE_modem: step ATI`。
- 现象 2（关闭 DMA - NODMA）：能走完前面的 AT 指令并显示 `LTE_modem: connected`，但随后疯狂刷 `PPP[0]: reconnecting`，永远无法获取 `NET: IP`。这是因为 PPP 阶段数据量大（115200 波特率下每秒近 11KB），没有 DMA 导致硬件 FIFO（H7 仅 8 字节）频繁溢出丢包，PPP (LCP/IPCP) 握手校验直接失败。

**终极解决方案（避开硬件冲突引脚）：**
1. **换硬件接口**：不要使用与 ADC 冲突的串口（如 PA2/PA3、PA0/PA1）。将 LTE 模块换到一个干净的串口，例如这块板子上的 `USART1`（PA9/PA10，即 GPS1 接口；**出厂 EFT_CAAC 上映射为 `SERIAL3`**，若改接该口请同步把参数从 `SERIAL1_*` 改回 `SERIAL3_*`）。
2. **分配独占 DMA**：在 `hwdef.dat` 中，将其它不重要的外设和串口设为 `NODMA`（例如 COM1/2 设为 NODMA），并给 LTE 模块所在的干净串口分配独占 DMA 通道：`DMA_NOSHARE SPI1* SPI2* USART1*`。
3. 保持 `LTE_BAUD = 115200`，DMA 畅通后 PPP 秒连，不再重连！

### 坑 8：UOM `无UTC ts inst=nil st=-1`

GPS 尚无 3D 定位，脚本拿不到 UTC 时间戳，激活请求无法构造。
室外等待 1~3 分钟（冷启动）；脚本会自动在获得定位后继续。

### 坑 9：UOM 激活一直超时（`激活响应超时，重试`）

1. 确认 GPS 已 3D 定位（日志有 `EKF3 origin set`）
2. 确认 ts 是 13 位数（`1779xxxxxxx`，GCS 截断成两行属正常）
3. 联系云端确认设备 `EFT2605210001` 已在 UOM 平台注册
4. 用 MQTTX 工具订阅 `uav/up/activation/eft/#` 确认 UP 到达云端

### 坑 10：`PreArm: Compasses inconsistent`

双罗盘（板载 QMC5883P + u-blox 内置磁）方向偏差超标。
解决：**室外做罗盘校准**（MP → Initial Setup → Mandatory Hardware → Compass），或临时设 `COMPASS_USE2 = 0` 只用主罗盘。

---

## 八、调试工具

### 8.1 SD 卡日志

脚本会在 SD 卡根目录生成 `LTE_modem.log`，记录所有与模块的 AT 指令交互。
可以用记事本打开，搜索 `ERROR` 或 `CONNECT` 关键词来定位问题。

### 8.2 开启详细调试

设置 `LTE_OPTIONS = 63`（开启全量日志），重启后所有原始串口数据都会被记录到 `LTE_modem.log`。

### 8.3 模块指示灯（AT 固件）

| NET LED 状态 | 含义 |
| :--- | :--- |
| 亮 0.2 秒，灭 1.8 秒 | 搜索网络中 |
| 亮 1.8 秒，灭 0.2 秒 | 待机（已注册但未激活数据） |
| 亮 0.125 秒，灭 0.125 秒 | 网络已激活（正常工作状态） |

### 8.4 MQTTX 抓 UOM 报文

下载 [MQTTX](https://mqttx.app) → 新建连接到 `47.120.16.113:1883` → 订阅以下主题观察实时数据：

```
uav/up/activation/eft/#         ← 飞控发出的激活请求
uav/down/activation/eft/#       ← 云端下发的激活响应
uav/up/telemetry/eft/#          ← 飞控发出的遥测
```

---

## 九、完整参数清单（一览表）

以 EFT_CAAC + `SERIAL1` + PPP 模式 + UOM MQTT 为例：

```
# ── Lua 脚本引擎 ──
SCR_ENABLE          1
SCR_HEAP_SIZE       409600
SCR_SDEV_EN         1
SCR_SDEV1_PROTO     48
NFZ_PAGE_SIZE       10        # 禁飞区脚本（如使用）

# ── 物理串口 ──
SERIAL1_PROTOCOL    28
SERIAL1_BAUD        115
SERIAL1_OPTIONS     0
BRD_SER1_RTSCTS     0

# ── LTE 脚本参数 ──
LTE_ENABLE          1
LTE_SERPORT         0
LTE_SCRPORT         0
LTE_PROTOCOL        48
LTE_BAUD            115200
LTE_IBAUD           115200
LTE_TIMEOUT         10
LTE_OPTIONS         0          # 调试时改 63

# ── 网络栈 ──
NET_ENABLE          1

# ── UOM MQTT（默认地址 47.120.16.113:1883）──
LTE_UOM_ENABLE      1
LTE_UOM_IP0         47
LTE_UOM_IP1         120
LTE_UOM_IP2         16
LTE_UOM_IP3         113
LTE_UOM_PORT        1883

# ── Remote ID（经 MP Remote ID 界面写入，脚本自动持久化）──
DID_ENABLE          1
DID_MAVPORT         4
# LTE_UAS_W01~10  (BASIC_ID 写入后自动)
# LTE_OP_W01~10   (OPERATOR_ID 写入后自动)
# LTE_OP_LAT/LNG/ALT (操控员位置)

# ── 公网 UDP 出口（如需私有地面站 4G 直连，可选）──
NET_P1_TYPE         1
NET_P1_PROTOCOL     2
NET_P1_IP0          (服务器 IP 第 1 段)
NET_P1_IP1          (服务器 IP 第 2 段)
NET_P1_IP2          (服务器 IP 第 3 段)
NET_P1_IP3          (服务器 IP 第 4 段)
NET_P1_PORT         14550
```

---

## 十、UOM MQTT 主题与数据格式

### 10.1 激活请求（飞控 → 云）

```
Topic:   uav/up/activation/eft/{fcu_id}
Payload: {"fcu_id":"EFT2605210001","timestamp":1779687040488}
```

### 10.2 激活响应（云 → 飞控）

```
Topic:   uav/down/activation/eft/{fcu_id}
Payload: {"code":0,"message":"设备激活成功，可以开始上报数据","timestamp":1779687040488}
```

| code | 含义 | 脚本处理 |
|------|------|---------|
| 0 | 激活成功 | → READY，1Hz 遥测 |
| 1 | 服务器异常 | 断开，30s 后重试 |
| 2 | 设备不存在 | 断开，30s 后重试（需云端注册） |
| 3 | 实名验证失败 | 同上 |
| 4 | 状态上报失败 | 同上 |
| 5 | 激活中请稍候 | 8s 后重试 |

### 10.3 遥测（飞控 → 云，1Hz）

```
Topic:   uav/up/telemetry/eft/{fcu_id}
Payload: {
  "ts": 1779687040488,
  "lng": 117.2723, "lat": 31.77866,
  "alt": 1.2, "alt_gps": 36.4,
  "speed": 0.02, "yaw": 180.5, "pitch": 0.1, "roll": -0.2,
  "accuracy": 80, "sys_status_bit": 0,
  "user_id": "0",
  "uas_id": "EFT2605210001",
  "operator_id": "202605250941",
  "op_lat": 31.77866, "op_lng": 117.2723, "op_alt": 36.0
}
```

---

## 十一、公网中转服务器搭建（地面站远程连接）

### 11.1 为什么需要公网服务器

飞控通过 4G 拨号后获得的是**运营商内网 IP**（如 `10.x.x.x`），地面站电脑通常也在 NAT 后面，双方**互相不可达**。需要一台有**固定公网 IPv4** 的服务器做中转。

```
飞控 Air780E ──4G──▶ 公网服务器:14550 ──▶ 你的电脑 Mission Planner:14551
     (运营商内网 IP)        (固定公网 IP)        (本地任意 IP)
```

### 11.2 服务器选型建议

| 项目 | 推荐 |
| :--- | :--- |
| **厂商** | 阿里云 / 腾讯云轻量应用服务器（新用户有免费试用） |
| **地域** | 国内地域（华北、华东等），延迟比新加坡低 |
| **规格** | 最低配即可（1 核 2G），MAVLink 中转流量极小 |
| **系统镜像** | Ubuntu 22.04（教程资料最全） |
| **关键条件** | 必须有**固定公网 IPv4**（控制台显示"公"字的那个） |

### 11.3 中转脚本（relay.py）

系统自带 Python3，无需安装任何依赖，直接运行。
将以下内容保存为 `/root/relay.py`：

```python
#!/usr/bin/env python3
"""
MAVLink UDP 双向中转：
  飞控(4G) --UDP--> 服务器:14550 --> 地面站:14551
  地面站   --UDP--> 服务器:14551 --> 飞控
"""
import socket, threading, time

DRONE_PORT = 14550   # 接收飞控数据的端口
GCS_PORT   = 14551   # 地面站连入的端口

drone_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
drone_sock.bind(('0.0.0.0', DRONE_PORT))

gcs_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
gcs_sock.bind(('0.0.0.0', GCS_PORT))

gcs_clients = set()
drone_addr   = [None]

def gcs_thread():
    """接收地面站数据，记录其地址，并转发给飞控"""
    while True:
        data, addr = gcs_sock.recvfrom(65535)
        gcs_clients.add(addr)
        if drone_addr[0]:
            drone_sock.sendto(data, drone_addr[0])

def drone_thread():
    """接收飞控数据，转发给所有已知地面站"""
    while True:
        data, addr = drone_sock.recvfrom(65535)
        drone_addr[0] = addr
        for c in list(gcs_clients):
            try:
                gcs_sock.sendto(data, c)
            except Exception:
                gcs_clients.discard(c)

threading.Thread(target=gcs_thread,  daemon=True).start()
threading.Thread(target=drone_thread, daemon=True).start()

print(f"中转启动: 飞控->{DRONE_PORT}, 地面站->{GCS_PORT}")
while True:
    time.sleep(10)
    print(f"飞控地址: {drone_addr[0]}, 地面站数量: {len(gcs_clients)}")
```

**后台运行（重要：SSH 断开后不停止）**：

```bash
nohup python3 /root/relay.py > /tmp/relay.log 2>&1 &
echo "PID: $!"
```

**验证端口已经监听**：

```bash
ss -unlp | grep -E "14550|14551"
# 正常输出：
# UNCONN 0 0  0.0.0.0:14550  0.0.0.0:*  users:(("python3",pid=xxxx,fd=3))
# UNCONN 0 0  0.0.0.0:14551  0.0.0.0:*  users:(("python3",pid=xxxx,fd=4))
```

### 11.4 防火墙放通端口（必须操作）

在云控制台 → 轻量应用服务器 → **防火墙** 页面，添加入站规则：

| 协议 | 端口 | 说明 |
| :--- | :--- | :--- |
| UDP | 14550 | 飞控数据入站 |
| UDP | 14551 | 地面站连入 |

> 只配置系统防火墙（firewalld/iptables）不够，云控制台的安全组/防火墙规则优先级更高，必须两处都放通。

### 11.5 飞控参数（以服务器 IP `8.145.61.235` 为例）

```
NET_P1_TYPE  = 1      # UDP client
NET_P1_IP0   = 8
NET_P1_IP1   = 145
NET_P1_IP2   = 61
NET_P1_IP3   = 235
NET_P1_PORT  = 14550
```

填完参数后**重启飞控**。

### 11.6 Mission Planner 连接方法

1. 右上角连接方式下拉选 **UDPCI**
2. 点击**连接**
3. 弹出框：
   - **Remote host**：填服务器公网 IP（如 `8.145.61.235`）
   - **Port**：填 `14551`
4. 点 OK，等待 5~10 秒，看到姿态球转动即成功

### 11.7 服务器监控命令

```bash
# 查看中转进程是否在运行
ps aux | grep relay.py

# 实时查看中转日志（每 10 秒打印一次飞控地址和地面站数量）
tail -f /tmp/relay.log

# 查看实时流量（需安装：yum install -y iftop）
iftop -n -P

# 查看 UDP 端口监听状态
ss -unlp | grep -E "14550|14551"

# 重启中转脚本
pkill -f relay.py
nohup python3 /root/relay.py > /tmp/relay.log 2>&1 &
```

### 11.8 开机自动启动（可选）

```bash
# 写入 systemd 服务
cat > /etc/systemd/system/mavrelay.service << 'EOF'
[Unit]
Description=MAVLink UDP Relay
After=network.target

[Service]
ExecStart=/usr/bin/python3 /root/relay.py
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

systemctl daemon-reload
systemctl enable mavrelay
systemctl start mavrelay

# 查看服务状态
systemctl status mavrelay
```

---

## 十二、完整验证检查清单

按顺序逐项确认，所有项目通过即代表 4G 数传 + UOM 云对接完全打通：

**硬件层**
- [ ] Air780E 使用**标准 AT 固件**（不是 DTU/LuatOS）
- [ ] Air780E 使用**独立电源供电**（≥500mA）
- [ ] 天线已接好（IPEX 接头扣紧）
- [ ] SIM 卡已插紧、已激活、有流量余额
- [ ] TX/RX 交叉接线，GND 共地

**固件层**
- [ ] 固件已编译 `define AP_NETWORKING_BACKEND_PPP 1`
- [ ] 固件已编译 `DMA_NOSHARE SPI1* SPI2* USART1*`
- [ ] LTE_modem.lua 已放在 SD 卡 `/APM/scripts/`

**参数层**
- [ ] `BRD_SER1_RTSCTS = 0`（关闭流控）
- [ ] `SERIAL1_PROTOCOL = 28`（交给 Lua 脚本）
- [ ] `NET_ENABLE = 1`
- [ ] `SCR_SDEV1_PROTO = 48`
- [ ] `LTE_SERPORT = 0`，`LTE_SCRPORT = 0`（0-based 索引）
- [ ] Remote ID 已写入 BASIC_ID（UAS ID 长度 ≥ 12）

**联网与激活**
- [ ] 飞控消息栏出现 `NET: IP xxx.xxx.xxx.xxx`
- [ ] 飞控消息栏出现 `UOM: MQTT connected`
- [ ] 飞控消息栏出现 `UOM激活[0]: 激活成功`

**远程地面站（如启用 NET_P1）**
- [ ] 云服务器 `relay.py` 在运行，两个端口监听正常
- [ ] 云控制台防火墙放通 UDP 14550/14551
- [ ] Mission Planner 通过 UDPCI 方式成功连接

---

## 十三、常见进阶问题

### 13.1 Mission Planner 显示"连接质量低/丢包严重"怎么办？

如果看到"丢包 214015，质量 3%"这种极其夸张的数据，通常**不是真实链路差，而是计数器累积差值**。

- **原因**：飞控上电后 MAVLink 序列号一直在递增。如果地面站是飞控上电很久之后才连上的，地面站看到序列号中间出现"巨大缺口"，就会把这些没收到的包算作"丢包"。
- **解决**：在 Mission Planner 的"连接统计"窗口右上角，点击 **`Reset`**（重置计数器），质量会立刻恢复到真实的 80%~99%。如果重置后仍然很低（<60%），再排查 4G 信号强度或服务器延迟。

### 13.2 4G 数传到底耗多少流量？

以默认 MAVLink 遥测频率（约 1.8 KB/s ~ 2.0 KB/s）计算：

- **每小时**：约 6.5 MB
- **一天飞 8 小时**：约 52 MB
- **一个月（每天 8 小时）**：约 1.5 GB

**结论**：买一张 1~3GB/月的物联网卡（通常每月几块钱到十几块钱）就足够供一架飞机高强度使用了。

---

## 十四、底层原理：服务器如何给 Air780 发消息？（NAT 穿透）

很多人有疑问：服务器（或地面站）怎么知道向哪里发送数据，才能让数据穿透 4G 运营商的网络准确到达 Air780E 模块？

**解答：这依赖于 UDP 的 NAT 穿透（打洞）机制。**

1. **飞控主动发包（打洞）**：飞控网络栈通过底层的 PPP 驱动，把 MAVLink 封装成 UDP 包。这些 UDP 包通过 Air780E 发给公网服务器的 `14550` 端口。经过运营商的 NAT 网关时，网关会记录一条映射（例如：`公网IP:公网端口 <-> 模块内网IP:内网端口`）。
2. **服务器记录来源地址**：`relay.py` 中转脚本收到数据后，会**动态记录**这架飞机的来源地址（即运营商 NAT 映射后的公网 IP 和端口）。
   ```python
   data, addr = drone_sock.recvfrom(65535)
   drone_addr[0] = addr  # 记住这个地址
   ```
3. **原路返回**：当地面站需要发送指令（如切模式、发航线）给飞机时，`relay.py` 直接把指令发送到刚才记录的那个 `drone_addr[0]`。
4. **NAT 转发**：数据到达运营商 NAT 网关后，网关查表，直接将数据转发到 Air780E 所在的内网 IP，最终到达飞控。

**核心结论**：服务器不需要知道 Air780E 的任何硬件信息或物理位置。只要飞控保持 MAVLink 心跳包发送（维持 NAT 映射不断），服务器就能随时把数据发回给飞机。

---

## 十五、商业化拓展方案

作为无人机公司，如果要将 4G 数传作为卖点，需要解决**多设备、多客户、权限隔离**的问题。

### 15.1 如何区分不同客户的飞机？

1. **端口隔离（初级方案，目前适用）**
   给每架飞机（或每个客户）分配唯一的 UDP 端口。
   - 客户 A：出厂参数配置 `NET_P1_PORT = 15001`
   - 客户 B：出厂参数配置 `NET_P1_PORT = 15002`
   - 服务器上运行多个 `relay.py`，分别监听不同的端口进行数据转发。

2. **MAVLink System ID 路由（中级方案）**
   飞控参数 `SYSID_THISMAV` 设为飞机的唯一编号（如 1~255）。所有飞机连到同一个 UDP 端口，服务器网关解析 MAVLink 协议，根据 `system_id` 将数据分发到不同客户的地面站连接。

3. **连接层 Token 鉴权（高级方案）**
   修改 `LTE_modem.lua` 脚本，在连接建立初期发送包含设备识别码（SN/Token）的鉴权包，服务器网关校验身份后才建立数据隧道。

### 15.2 商业化 MVP（最小可行产品）落地路径

1. **硬件**：在无人机内部集成 Air780E 模块 + 独立 5V 降压模块。
2. **服务器**：购买云服务器（按月/年付费，极低成本），配置 `systemd` 开机自启 `relay.py`，配置防火墙端口。
3. **生产烧录**：整理一套标准参数 `.param` 模板。出厂时，只需修改模板中的 `NET_P1_PORT` 等极少参数，即可绑定到对应的客户。
4. **客户交付**：给客户交付无人机时，附带一张已实名激活的物联网 SIM 卡（按年计费，打包进售价），并告知客户对应的服务器 IP 和专属连接端口。客户用 Mission Planner 即可实现无视距离的 4G 遥控。

---

## 十六、技术价值与商业推广：如何向技术主管/老板汇报

作为飞控工程师，在成功打通并验证了基于 Air780E 的 4G PPP 数传链路后，你可以从以下几个维度向技术主管或老板汇报该方案的核心商业与技术价值，推动该技术在公司产品线中的落地：

### 16.1 极佳的成本效益（降本）

- **传统方案痛点**：传统的远距离数传（如 SiK 915MHz 数传、大功率图数传一体机）不仅硬件成本高昂（几百到数千元不等），而且受制于无线电发射功率、地形遮挡和城市电磁干扰，实际图传/数传距离往往只有几公里。
- **Air780E 方案优势**：Air780E 模组硬件成本极低（单模组通常在 20~30 元人民币左右）。物联网卡流量成本极低（按年计费仅需十几元）。以不到 50 元的边际成本，彻底替换掉原本几百上千元的传统数传电台，极大地降低了整机 BOM（物料清单）成本。

### 16.2 突破物理距离限制（增效）

- **"无视距离"的控制能力**：依托成熟的 4G 蜂窝网络，只要飞机和地面站所在位置有手机信号，就能实现**全国乃至全球范围内的低延迟遥测与遥控**。
- **解锁全新应用场景**：这使得超视距（BVLOS）飞行、跨城市物流配送、偏远地区长航时巡检、无人值守机巢（Dock）远程调度等高级商业场景成为可能。这是传统点对点数传绝对无法做到的。

### 16.3 摆脱对特定地面站硬件的依赖

- **纯软件化地面站**：客户不再需要在电脑或遥控器上插一个物理的"数传接收端"天线。
- **多端协同指挥**：老板/客户甚至可以在出差途中，直接在手机或平板上打开 Mission Planner 或自研的网页端（WebGCS），输入对应的云服务器 IP 和端口，就能实时看到飞机的状态。这为指挥中心（Command Center）集中调度多架无人机打下了基础。

### 16.4 技术壁垒与护城河（体现研发价值）

- **底层硬件坑的排雷**：向主管展示我们团队不仅仅是"调参侠"。我们在适配过程中，甚至跨越了芯片级的底层缺陷（如 STM32H743 的 DMA 勘误 2.20.6 与 ADC 冲突），通过修改底层的 `hwdef` 硬件定义文件、分配独占 DMA 通道，才从根本上解决了高波特率下的 PPP 丢包难题。这证明了团队具备深度掌控飞控底层软硬件交互的能力。
- **原生协议栈的优势**：我们使用的是 ArduPilot 原生的 PPP 网络协议栈和 Lua 脚本引擎，而不是外挂第三方黑盒的 DTU 透传模块。这意味着数据链路在飞控内部是完全透明、可控、且效率最高的，为未来接入 4G/5G 图传、视频流、乃至基于 MAVLink 2 的安全加密（Signing）奠定了坚实的架构基础。
- **UOM 民航云对接**：脚本已实现合规的 BASIC_ID / OPERATOR_ID 上链、激活握手、1Hz 遥测，可直接对接 CAAC UOM 平台，满足超视距与商业运营的合规要求。

### 16.5 汇报建议话术（One-Pitch）

> *"老板，我们目前已经成功攻克了底层的硬件兼容难题，将 4G 蜂窝通信原生地集成到了我们的飞控架构中。这套方案不仅能让我们**把单机的数传成本削减 90% 以上**，更重要的是，它彻底打破了通信距离的瓶颈——**只要有手机信号的地方，我们就能远程控制飞机**。同时我们已经完成 UOM 民航云的对接，整套方案是开箱即用的合规商业级产品。我们不仅能借此推出支持超视距飞行的行业无人机，还可以很方便地接入我们自己的云端指挥系统，这绝对是我们下一代产品的核心卖点。"*

---

## 十七、外场测试工程师（Field Test Engineer）连网与测试 SOP

当固件和参数在实验室配置完毕，交接给外场测试工程师进行实际飞行测试时，请外测工程师严格按照以下流程进行连接和保障，以确保测试安全顺利。

### 17.1 测试前检查（地面准备）

1. **硬件确认**：检查 4G 天线是否已经拧紧（切勿不接天线开机，会烧射频功放），SIM 卡是否插好。
2. **通电顺序**：由于 4G 模块启动需要大电流，请确保给飞机通上主动力电（电池），而不是仅仅插 USB 供电。
3. **状态确认**：飞机上电后等待约 1~3 分钟。由于外场没有屏幕，可以留意飞控的蜂鸣器提示音，或者（如果在现场）先用本地 USB/蓝牙/数传连接一次，在 Mission Planner 消息栏看到 `NET: IP 10.x.x.x` 即可拔掉本地线缆，说明 4G 已经成功上线。

### 17.2 Mission Planner 远程连接步骤（核心）

外场工程师的电脑（或连着手机热点的平板电脑）只需要有互联网连接即可，无需插任何物理数传接收机。

1. 打开 **Mission Planner** 地面站软件。
2. 在右上角的**连接方式**下拉菜单中，选择 **`UDPCI`**（注意：是 UDP Client，不是 UDP）。
3. 点击右侧的 **`连接 (Connect)`** 按钮。
4. 此时会弹出一个输入 IP 的对话框，输入我们的云服务器公网 IP：
   👉 **`8.145.61.235`**
5. 点击确定后，会弹出第二个对话框要求输入端口（Port），输入对应的转发端口：
   👉 **`14551`**
6. 点击确定。此时界面应该会显示 "Getting Params..."（正在获取参数），等待参数下载读条完成，即代表 **4G 远程连接成功！**

### 17.3 外场飞行注意事项与常见排障

- **安全保底（极其重要）**：在最初的几次外场 4G 测试中，**强烈建议测试工程师同时带上物理遥控器（如 ELRS 或接收机）作为保底控制链路**。一旦飞到 4G 信号盲区导致连接断开，可以通过物理遥控器随时接管切回 Loiter 或触发 RTL（一键返航）。
- **"假丢包"现象**：如果 MP 显示链路质量（Link Quality）只有 5% 甚至狂报丢包，不要慌。在 MP 的"连接统计（Stats）"窗口右上角点击 **`Reset`（重置）** 按钮。这通常是因为飞控开机很久后地面站才连上，导致 MAVLink 序列号断层产生的假丢包。重置后如果稳定在 80%~99%，则说明 4G 链路非常健康。
- **参数加载过慢**：如果连接时参数读条长达两三分钟，说明当前所处空域的运营商基站拥堵或 4G 信号极弱，此时强行超视距飞行风险较高。正常的 4G 参数加载应该在 10~20 秒内完成。
- **连不上服务器**：
  - 检查外测电脑/热点本身是否有网。
  - 检查飞机是否停在没有手机信号的死角（如地下车库、偏远山区）。
  - 如果前两者都正常，请立即联系后台研发，让研发在服务器端查看 `relay.py` 是否有收到飞控主动发来的 UDP 心跳包。如果没有收到，说明飞机端网络未通（可能卡死或欠费）。

---

## 附录 A：与原文件的对应关系

本文档由以下两个原始文档合并而来（已去重整合）：

| 原文件 | 对应章节 |
| :--- | :--- |
| `LTE 4G 使用说明.md` | 第一、二、三、四（阶段一~三、六）、五、六、七、八、九、十一、十二、十三、十四、十五、十六、十七节 |
| `LTE_UOM_SOP.md` | 第一、二、三、四（阶段四、五）、五、六、七、九、十节 |

如需查阅原始版本，请参考 git 历史。
