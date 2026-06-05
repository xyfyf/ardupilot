# ZZZ EVTOL-F405

消费级垂起 / QuadPlane 飞控核心板：**STM32F405 + BMI270 + AT7456E 模拟 OSD + 板载罗盘 + SPL06**，外置 **`ZZZ-EVTOL-PDB-B`**，与 **`ZZZ-EVTOL-H743`** 共用（**不** 配 `ZZZ-EVTOL-H753` / PDB-A）。

## 传感器与 OSD

| 器件 | 接口 | 说明 |
|------|------|------|
| BMI270 | SPI1 | 六轴 IMU（**无磁力计**） |
| **AT7456E** | **SPI2 PB12** | **标配模拟 OSD**；CVBS **J_VTX_ANA** |
| QMC5883P / **IST8310** 二选一 | I2C1 `0x0D` / `0x0E` | **板载罗盘** |
| SPL06 | I2C1 `0x76` | 气压计 |
| W25Q128 | SPI3 | 黑匣子日志 |
| 外置罗盘 | GPS 座 I2C 分支 | 自动探测 |

## 图传（二选一，勿同时混接视频）

| 方式 | 接口 | 默认参数 |
|------|------|----------|
| **模拟 5.8G** | **J_VTX_ANA** → AT7456 → 模拟 VTX | 地面站 OSD = MAX7456 |
| **DJI O4 / HD** | **J_VTX** UART4 PA0/1 | `SERIAL4_PROTOCOL=42` MSP DisplayPort |
| VTX 9V 使能 | **PC13** GPIO81 | `RELAY1` |

> **唯一 SKU**：原理图必须贴 **AT7456**；不设无 OSD 版本。

## 磁力计硬件注意

1. 板载罗盘（5883P 或 IST8310）布置在 PCB 边缘，与 PDB 8P 电源入口保持 ≥15 mm；**同一位置只贴一颗**。  
2. GPS 连接器 I2C 与板载 mag 并联时，外置罗盘使用 `I2C:ALL_EXTERNAL` 探测。  
3. 首飞前检查地面站 **HW ID** 中 onboard + external 均正常。

## 接口摘要

- **PWM1–4**：垂起电机（**四路 DShot BIDIR**）  
- **LED**：PC6 / PC7（**PA13/14 专用于 SWD**）  
- **PWM5–10**：舵面 / 副翼 / 襟翼  
- **USART3**：GPS  
- **USART2**：RC（CRSF / SBUS）  
- **UART4**：MSP DisplayPort（HD 图传）  
- **ADC PC5**：模拟空速  

## 编译

```bash
./waf configure --board ZZZ-EVTOL-F405
./waf plane
```
