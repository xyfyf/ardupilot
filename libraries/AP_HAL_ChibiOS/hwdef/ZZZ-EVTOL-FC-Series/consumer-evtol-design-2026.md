# ZZZ eVTOL 飞控系列产品与 IO 规划（2026）

> **板卡**：`ZZZ-EVTOL-H753`（行业）｜`ZZZ-EVTOL-H743`（消费进阶）｜`ZZZ-EVTOL-F405`（消费入门）
> **电源板**：**H753 专用 `ZZZ-EVTOL-PDB-A`**；**F405 / H743 共用 `ZZZ-EVTOL-PDB-B`**（A/B **不互换**）
> **商务锚点**：消费板力争量产 BOM+SMT ¥120 / ¥180 档；H753 行业板单 MCU 已 ¥111@100，整机 BOM 通常 ¥250+。

---

## 1. 产品线架构

```text
  ┌─────────────────────────┐       ┌─────────────────────────┐
  │ ZZZ-EVTOL-PDB-A         │       │ ZZZ-EVTOL-PDB-B         │
  │ 行业电源（仅 H753）      │       │ 消费电源（F405↔H743）    │
  │ 150–200A / 双路 V·I     │       │ 120–150A 垂起           │
  └───────────┬─────────────┘       └───────────┬─────────────┘
              │ GH1.25 8P                       │ GH1.25 8P
              │ （与 PDB-B 不通用）              │
     ┌────────▼────────┐            ┌───────────┴───────────┐
     │ ZZZ-EVTOL-H753  │            │                       │
     │ 行业 / 双CAN    │     ┌──────▼───────┐   ┌───────▼────────┐
     │ SDMMC           │     │ZZZ-EVTOL-F405│   │ ZZZ-EVTOL-H743 │
     └─────────────────┘     │ 入门          │   │ 进阶           │
                             └──────────────┘   └────────────────┘
```

---

## 2. 核心硬件配置总表

| 项目 | ZZZ-EVTOL-H753 (行业) | ZZZ-EVTOL-H743 (消费进阶) | ZZZ-EVTOL-F405 (消费入门) |
|---|---|---|---|
| **Board ID** | 7121 | 7123 | 7122 |
| **MCU** | **STM32H753VIH6** (TFBGA-100) | STM32H743VIT6 (LQFP-100) | STM32F405RGT6 (LQFP-64) |
| **IMU** | 双 **ICM-42688-P** (SPI1+SPI4) | **BMI088+BMI270** (SPI1+SPI4) | 单 **BMI270** (SPI1) |
| **气压计** | SPL06 | SPL06 | SPL06 |
| **罗盘** | 仅外置 (I2C1) | 仅外置 (I2C1) | 仅外置 (I2C1) |
| **OSD** | 纯数字 (无模拟OSD) | AT7456E (SPI2) | AT7456E (SPI2) |
| **日志** | SDMMC 4-bit | SDMMC 4-bit | SDMMC 4-bit |
| **PWM** | 13 路 (1/3/5/7 BIDIR) | 12 路 (1/3/5/7 BIDIR) | 7 路 (1/2 BIDIR) |
| **UART** | 8 UART + 2x USB | 6 UART + USB | 4 UART + USB |
| **CAN** | **双 CAN** (CAN1+CAN2) | **单 CAN** (CAN1) | 无 |
| **电池 ADC** | 双路 V/I | 双路 V/I | 单路 V/I |
| **空速** | I2C 数字空速 | I2C 数字空速 | I2C 数字空速 |
| **配套 PDB** | **ZZZ-EVTOL-PDB-A** | **ZZZ-EVTOL-PDB-B** | **ZZZ-EVTOL-PDB-B** |

---

## 3. 接口与引脚分配总表

### 3.1 串口与总线 (Serial, CAN, USB)

| 丝印/功能 | ZZZ-EVTOL-H753 | ZZZ-EVTOL-H743 | ZZZ-EVTOL-F405 |
|---|---|---|---|
| **J_USB** (USB) | PA11/12 (OTG1) | PA11/12 (OTG1) | PA11/12 (OTG1) |
| **J_TELEM1** | USART1 (PA9/10) | USART1 (PA9/10) | USART1 (PA9/10) |
| **J_GPS1** (GPS+I2C1) | USART2 (PD5/6) | USART2 (PD5/6) | USART3 (PB10/11) |
| **J_RC** | USART6 (PC6/7) | USART6 (PC6/7) | USART2 (PA2/3) |
| **J_VTX** (数字图传) | USART3 (PD8/9) | USART3 (PD8/9) | UART4 (PA0/1) |
| **J_GPS2** | UART4 (PB8/9) | UART4 (PB8/9) | — |
| **J_TELEM2** | UART7 (PE7/8) | UART7 (焊盘) | — |
| **USER / 备用** | UART8, OTG2 | — | — |
| **J_CAN1** | PD0/1 (Silent: PD3) | PD0/1 (Silent: PD3) | — |
| **J_CAN2** | PB5/PD10 (Silent: PB4) | — | — |

### 3.2 PWM 输出

| PWM | H753 引脚 | H743 引脚 | F405 引脚 | 建议功能 |
|---|---|---|---|---|
| **1** | PB0 (BIDIR) | PB0 (BIDIR) | PB1 (BIDIR) | M1 |
| **2** | PB1 | PB1 | PB0 (BIDIR) | M2 |
| **3** | PA0 (BIDIR) | PA0 (BIDIR) | PB6 | M3 |
| **4** | PA1 | PA1 | PB7 | M4 |
| **5** | PA2 (BIDIR) | PA2 (BIDIR) | PA8 | RUD |
| **6** | PA3 | PA3 | PB14 | THR |
| **7** | PD12 (BIDIR) | PD12 (BIDIR) | PA15 | FLAP |
| **8~10** | PD13/14/15 | PD13/14/15 | — | AUX |
| **11~12** | PE5/6 | PE5/6 | — | GEAR/PARA |
| **13** | PA8 | — | — | LED |

### 3.3 专用控制引脚 (GPIO)

| 功能 | H753 | H743 | F405 |
|---|---|---|---|
| **VTX_PWR** (图传电源) | PB2 (GPIO81) | PD10 (GPIO81) | PC13 (GPIO81) |
| **PDB_EN** (电源板使能) | PE2 (GPIO83) | PE2 (GPIO83) | — |
| **CAM_SW** (相机切换) | PD11 (GPIO82) | — | — |
| **LED** | PE3/PE4 | PE3/PE4 | PC6/PC7 |
| **蜂鸣器** | — | — | PC15 |
| **SWD** | PA13/14 | PA13/14 | PA13/14 |

---

## 4. 电源板与防呆设计 (PDB)

> **核心原则**：行业机 (H753) 独享 PDB-A，消费机 (H743/F405) 共用 PDB-B。**两者禁止互换**。

*   **ZZZ-EVTOL-PDB-A (仅 H753)**: 
    *   **规格**: 150–200A，双路 V/I 采样。
    *   **防呆**: 8P 排线的 Pin8 **短接 GND**。飞控端通过 PE2 (PDB_EN) 控制高压侧。
*   **ZZZ-EVTOL-PDB-B (F405/H743)**: 
    *   **规格**: 120–150A 垂起。F405 仅用单路 V/I，H743 用满双路 V/I。
    *   **防呆**: 8P 排线的 Pin8 **悬空 (NC)**。
*   **CAN 总线**: 完整 CAN_H / CAN_L 走独立接口 (J_CAN1 / J_CAN2)，**不经过** 8P PDB 排线。

---

## 5. 外设说明 (罗盘与图传)

### 5.1 磁力计 (罗盘)
三板均**无板载磁力计**，统一采用 **GPS 外置 I2C 罗盘**：
*   **首选**: **QMC5883P** (I2C `0x0D`)，替代已停售的 5883L。
*   **二供**: **IST8310** (I2C `0x0E`)，缺货时备选。
*   出厂 `defaults.parm` 默认开启 `COMPASS_AUTODEC=1`，hwdef 已声明自动探测。

### 5.2 图传 (VTX)
三板均支持数字/模拟图传（同一时刻二选一）：
*   **模拟图传 (AT7456E)**: F405 和 H743 标配 AT7456E (SPI2)，通过 `J_VTX_ANA` 输出 CVBS。H753 纯数字，无模拟 OSD。
*   **数字图传 (DJI O4 / HD MSP)**: 通过 `J_VTX` 串口直连（H753/H743 对应 USART3，F405 对应 UART4）。出厂默认协议 `SERIALx_PROTOCOL = 42` (MSP DisplayPort)。

---

## 6. 立创商城 BOM 成本核算 (2026-05)

> **采购建议**：每月用 LCSC 购物车导出复核；外置罗盘 5883P 优先、IST8310 二供；H753 评估 H743+功能裁剪 是否比 H753 芯片更省 BOM。

### 6.1 核心器件单价参考
| 器件 | LCSC 编号 | ¥@100 | 备注 |
|---|---|---|---|
| **STM32H753VIH6** | C1343158 | ~111.0 | H753 专用 (TFBGA-100) |
| **STM32H743VIT6** | C114409等 | ~41.0 | H743 专用 |
| **STM32F405RGT6** | C15742 | ~15.0 | F405 专用 |
| **ICM-42688P**| C46550687 | ~77.0 | 必须有TDK |
| **BMI270** | C2836813 | ~11.8 | F405/H743 用 |
| **BMI088** | C194919 | ~28.0 | H743 用 |
| **SPL06-001** | C2684428 | ~3.4 | 三板标配 |
| **AT7456E** | C82351 | ~15.7 | F405/H743 标配 |
| **QMC5883P** | C2847467 | ~6.8 | 外置罗盘首选 |
| **IST8310** | C2683055 | ~10.0 | 外置罗盘二供 |

### 6.2 单板物料合计粗算 (@100pcs)
| 模块 | F405 | H743 | H753 (双 HXY 42688) |
|---|---|---|---|
| MCU | 16 | 43 | 111 |
| IMU | 12 | 35 | 36 (2×18) |
| Baro+SD+OSD | 28 | 28 | 20 |
| 被动/晶振/接插件/PCB | 30 | 36 | 44 |
| **小计 (料+板)** | **~86** | **~142** | **~211** |
| SMT+测试 (估) | 22 | 28 | 35 |
| **到厂成本粗算** | **~108** | **~170** | **~246** |

---

## 7. hwdef 编译与验证

| 板名 | hwdef 路径 | Board ID |
|---|---|---|
| ZZZ-EVTOL-H753 | `../ZZZ-EVTOL-H753/hwdef.dat` | 7121 |
| ZZZ-EVTOL-H743 | `../ZZZ-EVTOL-H743/hwdef.dat` | 7123 |
| ZZZ-EVTOL-F405 | `../ZZZ-EVTOL-F405/hwdef.dat` | 7122 |

**编译命令**:
```bash
./waf configure --board ZZZ-EVTOL-H753
./waf plane
```