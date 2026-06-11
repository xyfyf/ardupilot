# ZZZ EVTOL-H753 Flight Controller

ZZZ EVTOL-H753（原 ZZZ-H743-Wing 工程）为 **行业级固定翼 / VTOL** 飞控核心板，MCU 为 **STM32H753VIT6**，采用 **外置 PDB-A + 霍尔电流计** 分体架构。

**电源板**：仅配套 **`ZZZ-EVTOL-PDB-A`**，**不**使用消费线 `ZZZ-EVTOL-PDB-B`（F405/H743 底板）。硬件引脚对齐 **AET-H743-Basic** / **Matek H743-WING**，便于 **ArduPilot** / **iNav**。

详细市场分析见 `fixed-wing-fc-h7-design-2026.md`；PDB-A 线序见下文。

---

## Features

| 项目 | 规格 |
|------|------|
| MCU | **STM32H753VIT6**，480MHz，2MB Flash（硬件加密） |
| IMU | 双 ICM-42688-P（SPI1 + SPI4） |
| 气压计 | SPL06 / DPS310（I2C2） |
| OSD | 数字图传 MSP DisplayPort（USART3） |
| 存储 | microSD，SDMMC 4-bit |
| PWM | 13 路（部分 BIDIR，支持 VTOL DShot 遥测） |
| 串口 | 8× UART + 2× USB |
| CAN | **2×** CAN1（PD0/1）+ CAN2（**PB5/PD10**） |
| 电源采样 | 双路 V/I ADC（来自外置 PDB） |
| 空速 | I2C 数字空速扩展 |
| 罗盘 | GPS 座外置罗盘（I2C1） |

---

## 设计创新点（相对 AET / Matek / SpeedyBee）

### 1. 外置 PDB-A 专用架构（不与 PDB-B 共用）

- 飞控 **不焊接** 大电流；通过 **PDB-A** 8P 排线接收双路 V/I 与逻辑 5V。
- **PDB_EN（PE2 / GPIO 83）**：切断 **PDB-A** 高压侧（以 PDB-A 原理图为准）。
- `HAL_BATT_*_SCALE` 为 **PDB-A** 缺省值；量产按霍尔型号实测。**禁止** 插接消费 `PDB-B` 线束。

### 2. 固定翼出厂串口映射

| 端口 | 默认功能 | 说明 |
|------|----------|------|
| SERIAL0 | USB | 调参 / 刷机 |
| SERIAL1 | MAVLink2 | TELEM1 |
| SERIAL2 | GPS | GPS1 |
| SERIAL3 | **MSP DisplayPort** | HD 图传（O3/O4 等），相对 AET 默认 MAVLink 的改进 |
| SERIAL4 | GPS | GPS2 / 罗盘口 |
| SERIAL5 | USB（SLCAN） | 第二 USB |
| SERIAL6 | RCInput | SBUS / CRSF |
| SERIAL7 | MAVLink2 | **WiFi / 蓝牙** 模块（对齐 AET 无线调参习惯） |
| SERIAL8 | None | Remote ID / 用户扩展 |

### 3. RC 输入增强

- **RCININT** 脉宽捕获 + **USART6 ALT** 全双工 UART，无需额外硬件即可接 CRSF / ELRS（相对 AET 仅 USART6 的改进）。

### 4. VTOL / 多舵面 PWM

- **13 路 PWM**，通道 1/3/5/7 带 **BIDIR**，便于 QuadPlane 电机 DShot 双向遥测。
- PWM13 默认 NeoPixel（`defaults.parm`）。

### 5. 继电器默认绑定

- GPIO 81 — VTX 9V 电源（`RELAY2`，**PB2**，原 PD10 已让给 CAN2_TX）
- GPIO 82 — 摄像头切换（`RELAY3`）
- GPIO 83 — PDB 使能（`RELAY4`）

### 6. 传感器与维护

- 双 **ICM42688** + `HAL_DEFAULT_INS_FAST_SAMPLE 3`（高采样，对标 TBS Lucid H7）。
- **BUILD_ABIN** + Bootloader **SD 卡刷机**（`AP_BOOTLOADER_FLASH_FROM_SD_ENABLED`）。
- IMU2 片选命名为 `IMU2_CS`（PC13），与 AET `IMU3_CS` 同脚，语义更清晰。

### 7. iNav 双栈（硬件侧）

引脚与 Matek H743-WING / AET-H743-Basic 同族，建议在 iNav 仓库以 `target.h` 克隆 Matek H743 Wing V3 后仅修改 `board_name` 与 LED，并提交 PR。

---

## PDB-A 排线（8-pin GH1.25，仅 H753）

| Pin | 信号 |
|-----|------|
| 1 | VBAT → FC ADC 分压 |
| 2 | GND |
| 3 | CURR（主包电流） |
| 4 | VOLT（主包电压，若与 Pin1 合并则 NC） |
| 5 | CURR2（副路） |
| 6 | VOLT2（副路） |
| 7 | +5V 逻辑供电 |
| 8 | **KEY→GND**（防呆；**禁止** 插 PDB-B） |

> DroneCAN 完整总线在飞控 **J_CAN1** / **J_CAN2**，不经 8P 差分对。

### CAN2 扩展口（J_CAN2，GH1.25 4P 建议）

| Pin | 信号 | MCU（至 CAN 收发器） |
|-----|------|---------------------|
| 1 | CAN2_H | 总线 |
| 2 | CAN2_L | 总线 |
| 3 | GND | |
| 4 | +5V 可选 | 外设供电 |

收发器：`CAN2_TX`←**PD10**，`CAN2_RX`→**PB5**；静默控制 **PB4**。

各 CAN 总线 **120Ω 终端** 用跳线；静默脚：CAN1=PD3，CAN2=**PB4**。

---

## UART Mapping

- SERIAL0 → USB
- SERIAL1 → USART1（TELEM1）
- SERIAL2 → USART2（GPS1）
- SERIAL3 → USART3（MSP DisplayPort）
- SERIAL4 → UART4（GPS2）
- SERIAL5 → USB2（SLCAN）
- SERIAL6 → USART6（RC）
- SERIAL7 → UART7（TELEM2 / WiFi-BT）
- SERIAL8 → UART8（USER）

## PWM Output

13 路输出，分组与 AET-H743-Basic 相同；通道 1/3/5/7 支持 DShot BIDIR。

| 通道 | 建议丝印 |
|------|----------|
| 1–4 | AIL-L / AIL-R / ELE / RUD 或 M1–M4 |
| 5–6 | THR / FLAP |
| 7–10 | AUX1–4 |
| 11–12 | GEAR / PARACHUTE |
| 13 | LED |

## Loading Firmware

首次刷写：按住 Boot 键上电，USB DFU 加载 `*_with_bl.hex`。  
之后：使用 `*.apj` 通过 Mission Planner / QGC 升级。

```bash
./waf configure --board ZZZ-EVTOL-H753
./waf plane
```

固件目录：https://firmware.ardupilot.org （board 名称 `ZZZ-EVTOL-H753`，合入上游后可用）

## Compass

- **外置**：GPS 六芯线 I2C1，自动探测 IST8310 / QMC5883P 等  
- **板载**：当前 `hwdef.dat` 未声明板载罗盘，I2C2 仅用于 SPL06 / DPS310  
- 上电后完成罗盘校准；飞行中可优先使用 GPS 罗盘（地面站设置优先级）

## Battery Monitoring

- 主路：`BATT_VOLT_PIN=10`，`BATT_CURR_PIN=11`
- 副路：`BATT2_VOLT_PIN=18`，`BATT2_CURR_PIN=7`
- 电流/电压比例请根据外置 PDB 实测标定。
