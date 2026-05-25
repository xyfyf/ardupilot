# EFT_CAAC + Air780E 4G LTE + UOM 云平台激活 · 配置 SOP

> 版本：2026-05-25  
> 适用固件：ArduCopter V4.7.0-dev EFT_CAAC（含 `AP_NETWORKING_BACKEND_PPP`）  
> 模组：合宙 Air780E（**必须是标准 AT 固件**）  
> 脚本：`LTE_modem.lua`（放在 SD 卡 `/APM/scripts/`）

---

## 一、硬件接线

```
飞控 SERIAL1 (GPS1 座子 USART1 PA9/PA10)   Air780E 模组
──────────────────────────────────────────  ──────────────
TX  ──────────────────────────────────────► RX
RX  ◄──────────────────────────────────────  TX
GND ──────────────────────────────────────► GND
5V  ──────────────────────────────────────► VCC (建议独立 5V 供电)
```

**不接 CTS/RTS**（Air780E 无硬件流控引脚）

---

## 二、固件编译前置条件

`libraries/AP_HAL_ChibiOS/hwdef/EFT_CAAC/hwdef.dat` 末尾必须有：

```
define AP_NETWORKING_BACKEND_PPP 1
DMA_NOSHARE SPI1* SPI2* USART1*
```

> 缺少第一行：PPP 驱动不存在，`connected` 后立即 `timeout` 死循环。  
> 缺少第二行：H743 DMA 冲突导致 `PPP[0]: reconnecting` 死循环。

---

## 三、Mission Planner 参数配置

### 3.1 串口与 PPP 驱动（必须）

| 参数 | 值 | 说明 |
|------|-----|------|
| `SERIAL1_PROTOCOL` | **28** | Scripting，交给 Lua 脚本控制 |
| `SERIAL1_BAUD` | **115** | 115200 bps |
| `SERIAL1_OPTIONS` | **0** | 无特殊选项 |
| `BRD_SER1_RTSCTS` | **0** | ⚠️ 必须关闭流控，否则 ATI 无回包 |
| `SCR_SDEV_EN` | **1** | 开启 Scripting 虚拟串口 |
| `SCR_SDEV1_PROTO` | **48** | PPP 协议桥接 |
| `NET_ENABLE` | **1** | 开启网络栈 |
| `SCR_ENABLE` | **1** | 开启 Lua 脚本引擎 |
| `SCR_HEAP_SIZE` | **204800** | 堆内存（不足会 OOM 崩溃） |

### 3.2 LTE 模组参数（由脚本动态注册）

| 参数 | 值 | 说明 |
|------|-----|------|
| `LTE_ENABLE` | **1** | 启用 LTE 驱动 |
| `LTE_PROTOCOL` | **48** | PPP 模式 |
| `LTE_BAUD` | **115200** | 与模组通信波特率 |
| `LTE_IBAUD` | **115200** | 上电初始波特率 |
| `LTE_TIMEOUT` | **10** | 连接超时（秒），0=禁用 |
| `LTE_OPTIONS` | **0** | 正常；调试时设 63 开启全量串口日志 |

### 3.3 UOM MQTT 云平台参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `LTE_UOM_ENABLE` | **1** | 启用 UOM MQTT 上报 |
| `LTE_UOM_IP0` | **47** | MQTT Broker IP 第1段 |
| `LTE_UOM_IP1` | **120** | 第2段 |
| `LTE_UOM_IP2` | **16** | 第3段 |
| `LTE_UOM_IP3` | **113** | 第4段（默认 47.120.16.113） |
| `LTE_UOM_PORT` | **1883** | MQTT 端口 |

> 如需修改 MQTT 账密，编辑 `LTE_modem.lua` 中 `UOM_MQTT_USER` / `UOM_MQTT_PASS`。

### 3.4 Remote ID / OpenDroneID（写入 UAS ID）

在 MP 的 **Remote ID** 标签页（或 Initial Setup → Mandatory Hardware → Remote ID）配置：

| 字段 | 说明 | 对应 LTE 参数 |
|------|------|--------------|
| **UAS ID（BASIC_ID）** | 实名登记号，最多 20 字符，如 `EFT2605210001` | `LTE_UAS_W01~10`（自动持久化） |
| **Operator ID** | 运营人编号，如 `202605250941` | `LTE_OP_W01~10` |
| **Self ID（描述）** | 自我声明，可空 | `odid.self_desc`（内存） |
| **System（操控员位置）** | 地面站 GPS 坐标 | `LTE_OP_LAT/LNG/ALT` |

> ⚠️ UAS ID 长度须 ≥ 12 字符，否则脚本等待完整 ID 后再订阅。

---

## 四、SD 卡脚本部署

```
/APM/scripts/
  └── LTE_modem.lua       ← 唯一需要放的文件
```

无需其他脚本配合 UOM 功能。

---

## 五、正常启动日志（参考）

从开机到激活完成约 **30~60 秒**（含 GPS 冷启动）：

```
LTE_modem: starting
LTE_modem: step ATI
LTE_modem: sent reset            ← 首次上电先软复位退出 PPP 模式
LTE_modem: found modem: Air780   ← 识别到模组
LTE_modem: step BAUD
LTE_modem: step CPIN
LTE_modem: step CONFIG
LTE_modem: step CREG
LTE_modem: CREG OK               ← 注册到基站
LTE_modem: step CGACT
LTE_modem: CGACT OK              ← PDP 上下文激活
LTE_modem: step PPPOPEN
LTE_modem: connected             ← ATD*99# 拨号成功
PPP[0]: reconnecting             ← 飞控底层 PPP 开始 LCP/IPCP 协商（正常）
NET: IP      10.x.x.x            ← 运营商分配 IP
NET: Mask    255.255.255.255
NET: Gateway 10.x.x.x
EKF3 IMU0 origin set             ← GPS 3D Fix（时间戳变为有效值）
UOM: MQTT connected              ← 成功连接 MQTT Broker
UOM: 已发送订阅 uav/down/activation/eft/EFT2605210001
UOM: 进入激活阶段 (SUBACK)       ← 正常；若显示 SUBACK timeout 也可继续
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

---

## 六、常见故障排查

### 卡在 `step ATI` / 无 `found modem`

| 检查项 | 正确值 |
|--------|--------|
| `SERIAL1_PROTOCOL` | 28 |
| `BRD_SER1_RTSCTS` | **0** |
| 接线 TX/RX | 交叉连接 |
| 模组固件 | **标准 AT 固件**（非 DTU/LuatOS） |
| 模组供电 | 独立 5V，≥500mA |

调试：设 `LTE_OPTIONS=63`，重启后查 SD 卡 `LTE_modem.log`。

### 卡在 `step CPIN`

SIM 卡未就绪：检查 SIM 方向、是否插紧、欠费、实名激活。

### `PPP[0]: reconnecting` 死循环，始终不出 `NET: IP`

STM32H743 DMA 冲突（详见固件编译条件）。  
确认 `hwdef.dat` 有 `DMA_NOSHARE SPI1* SPI2* USART1*`，重新编译固件。

### `UOM: 无UTC ts inst=nil st=-1`

GPS 尚无 3D 定位。室外等待 1~3 分钟（冷启动）；脚本会自动在获得定位后继续。

### 激活一直超时（`激活响应超时，重试`）

1. 确认 GPS 已 3D 定位（日志有 `EKF3 origin set`）
2. 确认 ts 是 13 位数（`1779xxxxxxx`，GCS 截断成两行属正常）
3. 联系云端确认设备 `EFT2605210001` 已在 UOM 平台注册
4. 用 MQTTX 工具订阅 `uav/up/activation/eft/#` 确认 UP 到达云端

### `PreArm: Compasses inconsistent`

双罗盘（板载 QMC5883P + u-blox 内置磁）方向偏差超标。  
解决：**室外做罗盘校准**（MP → Initial Setup → Mandatory Hardware → Compass），  
或临时设 `COMPASS_USE2=0` 只用主罗盘。

---

## 七、UOM 五个 ID 速查

| 变量 | 说明 | 来源 | 在哪体现 |
|------|------|------|---------|
| `vendor_id` | 厂商固定值 `eft` | 脚本写死 | MQTT 主题路径 |
| `fcu_id` | 设备主键 = uas_id | BASIC_ID / LTE_UAS_W | UP/DOWN 主题后缀、MQTT Client ID |
| `uas_id` | 实名登记号 | MP Remote ID → BASIC_ID | 遥测 JSON `uas_id` |
| `operator_id` | 运营人编号 | MP Remote ID → OPERATOR_ID | 遥测 JSON `operator_id` |
| `user_id` | 自我声明 | MP Remote ID → SELF_ID 或 `LTE_USER_ID` | 遥测 JSON `user_id` |

---

## 八、MQTT 主题与数据格式

### 激活请求（飞控 → 云）
```
Topic:   uav/up/activation/eft/{fcu_id}
Payload: {"fcu_id":"EFT2605210001","timestamp":1779687040488}
```

### 激活响应（云 → 飞控）
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

### 遥测（飞控 → 云，1Hz）
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

## 九、参数一览（完整清单）

```
# ── 串口 & PPP ──
SERIAL1_PROTOCOL    28
SERIAL1_BAUD        115
BRD_SER1_RTSCTS     0
SCR_SDEV_EN         1
SCR_SDEV1_PROTO     48
NET_ENABLE          1
SCR_ENABLE          1
SCR_HEAP_SIZE       204800

# ── LTE 驱动 ──
LTE_ENABLE          1
LTE_PROTOCOL        48
LTE_BAUD            115200
LTE_IBAUD           115200
LTE_TIMEOUT         10
LTE_OPTIONS         0       # 调试时改 63

# ── UOM MQTT（默认地址 47.120.16.113:1883）──
LTE_UOM_ENABLE      1
LTE_UOM_IP0         47
LTE_UOM_IP1         120
LTE_UOM_IP2         16
LTE_UOM_IP3         113
LTE_UOM_PORT        1883

# ── Remote ID（经 MP Remote ID 界面写入，脚本自动持久化）──
# LTE_UAS_W01~10    对应 UAS ID 字符串（20字符分10组存储）
# LTE_OP_W01~10     对应 Operator ID
# LTE_OP_LAT/LNG/ALT 操控员位置

# ── Remote ID 模块 ──
DID_ENABLE          1
DID_MAVPORT         4
```
