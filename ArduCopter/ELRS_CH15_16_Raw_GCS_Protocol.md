# ELRS CH15/CH16 原始通道 — 地面站对接说明

> 适用版本：ArduCopter（本仓库 EFT_CAAC 定制版）
> 飞控侧实现：`libraries/AP_RCProtocol/AP_RCProtocol_CRSF.cpp`、`libraries/AP_HAL_ChibiOS/hwdef/EFT_CAAC/`
> 文档版本：v1.0（2026-06）

---

## 1. 概述

本固件对 **RC 通道 15、16** 启用了 ELRS 扩展 Aux 原始值透传模式。飞控内部存储并上报的是 **CRSF 11bit 原始数值（0–2047）**，**不是** 标准 PWM 微秒（1000–2000 μs）。

因此会出现以下现象，属于**预期行为**：

| 观测位置 | CH15/CH16 低位读数 | 单位 |
| --- | --- | --- |
| ELRS Configurator / 遥控器 | ~1000 | PWM μs（1000–2000） |
| 飞控 `RC_CHANNELS` / 地面站 RC 监视 | ~190 | CRSF raw（0–2047） |

**地面站工程师请勿将两者直接对比**，必须先做单位换算。

---

## 2. 根本原因

### 2.1 固件编译开关

`EFT_CAAC/hwdef.dat` 中定义：

```text
define AP_CRSF_ELRS_RAW_AUX15_16_ENABLED 1
```

效果：CH15/CH16 在 CRSF 解码时**跳过** `TICKS_TO_US` PWM 映射，直接提取 11bit 位域原始值写入 `radio_in`。

### 2.2 默认参数范围

`EFT_CAAC/defaults.parm` 中：

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `RC15_MIN` | 0 | 通道 15 下限（raw） |
| `RC15_MAX` | 2047 | 通道 15 上限（raw） |
| `RC15_TRIM` | 1024 | 通道 15 中位（raw） |
| `RC16_MIN` | 0 | 通道 16 下限（raw） |
| `RC16_MAX` | 2047 | 通道 16 上限（raw） |
| `RC16_TRIM` | 1024 | 通道 16 中位（raw） |

> **注意**：不要将 `RC15_OPTION` / `RC16_OPTION` 设为辅助开关功能，否则会干扰原始值透传语义。

### 2.3 两套数值体系

```
┌─────────────────┐     CRSF 11bit      ┌──────────────────┐
│  ELRS 发射端     │ ──► 0–2047 raw ──► │  飞控 CH15/CH16   │ ──► MAVLink RC_CHANNELS
│  显示 1000–2000  │     (本固件保留)     │  radio_in ≈ 190   │     chan15_raw ≈ 190
│  PWM μs          │                     │  不做 us 映射      │
└─────────────────┘                     └──────────────────┘
         │                                        │
         │  标准通道 CH1–14 会映射为 1000–2000 μs   │
         └────────────────────────────────────────┘
```

CH1–CH14 仍走标准 CRSF → PWM μs 映射；**仅 CH15/CH16 保留 raw**。

---

## 3. 换算公式

CRSF 标准 PWM 映射（TBS 定义，CH1–CH14 使用）：

```text
PWM_us = raw × 5/8 + 880
```

反推（地面站将飞控 raw 转为 μs 显示时使用）：

```text
raw = (PWM_us - 880) × 8/5
```

### 3.1 常用对照表

| ELRS / 遥控器显示 (μs) | 飞控 `chan15_raw` / `chan16_raw` (raw) | 说明 |
| --- | --- | --- |
| 1000 | **192** | 最低位（用户常见 ~190 即此值） |
| 1500 | **992** | 中位 |
| 2000 | **1792** | 最高位 |

验算：`192 × 5/8 + 880 = 1000` ✓

### 3.2 地面站伪代码

```python
def raw_to_pwm_us(raw: int) -> int:
    """将飞控 CH15/CH16 raw 转为标准 PWM μs，供 UI 显示"""
    return int(raw * 5 / 8 + 880)

def pwm_us_to_raw(pwm_us: int) -> int:
    """将 ELRS μs 转为飞控 raw，供逻辑判断"""
    return int((pwm_us - 880) * 8 / 5)

# 示例
assert raw_to_pwm_us(192) == 1000
assert pwm_us_to_raw(1000) == 192
```

```javascript
// JavaScript
const rawToPwmUs = (raw) => Math.round(raw * 5 / 8 + 880);
const pwmUsToRaw = (us) => Math.round((us - 880) * 8 / 5);
```

---

## 4. MAVLink 数据来源

### 4.1 推荐消息：`RC_CHANNELS`（ID 65）

飞控通过 `GCS_MAVLINK::send_rc_channels()` 发送，`chan15_raw` / `chan16_raw` 字段直接取自 `rc().get_radio_in()`，**即 raw 值**。

| 字段 | 通道 | 本固件单位 | 典型低位值 |
| --- | --- | --- | --- |
| `chan15_raw` | RC15 | CRSF raw (0–2047) | ~192 |
| `chan16_raw` | RC16 | CRSF raw (0–2047) | ~192 |

- `chancount`：有效通道数（含 15/16）
- 其他通道 `chan1_raw`–`chan14_raw` 仍为 μs（1000–2000）

### 4.2 地面站 UI 建议

| 显示项 | 建议 |
| --- | --- |
| CH1–CH14 | 直接显示 μs，与常规 ArduPilot 一致 |
| CH15–CH16 | **二选一**：<br>① 显示 raw 并标注单位 `raw (0–2047)`<br>② 调用 `raw_to_pwm_us()` 换算后显示 μs，标注"已换算" |
| 告警阈值 | 对 CH15/16 使用 raw 域判断，勿套用 1000/1500/2000 |

### 4.3 不推荐直接使用的字段

- `RC_CHANNELS` 的 `chan15_raw` **不要**与 ELRS Configurator 的 1000 直接做差值报警
- `control_in` / 归一化值（-4500~4500 或百分比）对 CH15/16 在 raw 模式下**无物理 μs 含义**，勿用于显示

---

## 5. 舵机输出（SERVO 直通）

若参数配置为：

| 参数 | 值 | 含义 |
| --- | --- | --- |
| `SERVO15_FUNCTION` | 65 | RCIN15 直通 |
| `SERVO16_FUNCTION` | 66 | RCIN16 直通 |

则 AUX 引脚输出脉冲宽度 = `radio_in`（raw），**不是** 1000–2000 μs。

| 摇杆位置 | 引脚 PWM 输出 |
| --- | --- |
| 最低 | ~192 μs |
| 中位 | ~992 μs |
| 最高 | ~1792 μs |

下游设备若需要标准 1000–2000 μs，应改用 `SERVO15_FUNCTION=154`（RCIN15Scaled）/ `SERVO16_FUNCTION=155`（RCIN16Scaled），或关闭 raw 模式（需改固件）。

---

## 6. 参数速查

### 6.1 飞控侧（已固化默认值）

```text
RC15_MIN=0    RC15_MAX=2047   RC15_TRIM=1024
RC16_MIN=0    RC16_MAX=2047   RC16_TRIM=1024
```

### 6.2 地面站逻辑判断示例

判断 CH15 是否在"低位"（等效 ELRS 1000 μs）：

```python
RAW_LOW  = 192    # ≈ 1000 μs
RAW_MID  = 992    # ≈ 1500 μs
RAW_HIGH = 1792   # ≈ 2000 μs
DEADZONE = 50     # raw 域死区，按需求调整

def is_ch15_low(chan15_raw: int) -> bool:
    return chan15_raw < RAW_LOW + DEADZONE
```

若更习惯 μs 域，先换算再判断：

```python
def is_ch15_low_us(chan15_raw: int) -> bool:
    return raw_to_pwm_us(chan15_raw) < 1050
```

---

## 7. 常见问题

### Q1：ELRS 显示 1000，地面站 RC 监视显示 190，是不是坏了？

**不是。** 190 ≈ 192，即 CRSF raw 最低位，对应 ELRS 1000 μs。请按第 3 节换算。

### Q2：为什么 CH1–CH14 正常是 1000–2000，只有 15/16 不一样？

本固件仅对 CH15/CH16 启用 `AP_CRSF_ELRS_RAW_AUX15_16_ENABLED`，用于扩展 Aux 原始数据透传（如自定义传感器/开关量），其余通道保持 ArduPilot 标准行为。

### Q3：地面站要不要改 MAVLink 解析？

- **不需要改协议**，`RC_CHANNELS.chan15_raw` 仍是 `uint16`
- **需要改显示/逻辑**：识别 CH15/16 为 raw 域，或做 `raw_to_pwm_us()` 换算

### Q4：脚本 `rc:get_pwm(15)` 返回什么？

返回 `radio_in`，即 raw（约 190），**不是** μs。脚本内需自行换算：

```lua
local raw = rc:get_pwm(15)
local pwm_us = raw * 5 / 8 + 880
```

---

## 8. 代码索引（供联调）

| 文件 | 内容 |
| --- | --- |
| `libraries/AP_HAL_ChibiOS/hwdef/EFT_CAAC/hwdef.dat` | 编译开关 `AP_CRSF_ELRS_RAW_AUX15_16_ENABLED` |
| `libraries/AP_HAL_ChibiOS/hwdef/EFT_CAAC/defaults.parm` | RC15/RC16 MIN/MAX/TRIM 默认值 |
| `libraries/AP_RCProtocol/AP_RCProtocol_CRSF.cpp` | `apply_raw_aux15_16()` 原始值提取 |
| `libraries/AP_RCProtocol/AP_RCProtocol_Backend.cpp` | `decode_11bit_channel_raw()` 11bit 解码 |
| `libraries/GCS_MAVLink/GCS_Common.cpp` | `send_rc_channels()` MAVLink 上报 |
| `libraries/SRV_Channel/SRV_Channel_aux.cpp` | RCIN15/16 舵机直通逻辑 |

---

## 9. 版本记录

| 版本 | 日期 | 说明 |
| --- | --- | --- |
| v1.0 | 2026-06 | 首版，说明 ELRS CH15/CH16 raw 与 PWM μs 差异及地面站对接方式 |

---

## 10. 联系与确认项

地面站开发完成后，建议联调确认：

- [ ] CH15/CH16 低位：飞控 raw ≈ 192，换算 μs = 1000
- [ ] CH15/CH16 中位：飞控 raw ≈ 992，换算 μs = 1500
- [ ] CH15/CH16 高位：飞控 raw ≈ 1792，换算 μs = 2000
- [ ] CH1–CH14 仍显示 1000–2000 μs，不受影响
- [ ] UI 对 CH15/16 标注正确单位（raw 或换算后 μs）
