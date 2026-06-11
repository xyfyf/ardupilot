# ZZZ EVTOL-H743

消费级垂起进阶核心板：**STM32H743 + BMI088 + BMI270 + SPL06 + AT7456E + SDMMC**，CAN，外置 **`ZZZ-EVTOL-PDB-B`**（与 **F405** 互换，**不** 用于 H753）。

## 传感器

| 器件 | 接口 | 说明 |
|------|------|------|
| BMI088 | SPI1 | 六轴 IMU（双 CS，**无磁力计**） |
| BMI270 | SPI4 | 第二 IMU（**无磁力计**） |
| AT7456E | SPI2 | 模拟 OSD |
| SPL06 / DPS310 | I2C2 `0x76` | 气压计 |
| 外置罗盘 | I2C1（GPS 口） | 自动探测 QMC5883P / IST8310 |
| MicroSD | SDMMC1 | 黑匣子日志 |

## 存储

采用 **SDMMC1 4-bit MicroSD** 记录日志。刷机与调参走 USB / UART7 bootloader。

## 磁力计

- **I2C2**：板载气压计  
- **I2C1**：GPS 六芯线 SDA/SCL，外置罗盘  

与 F405 相同策略：GPS 外置罗盘；**配套 PDB-B**（与 F405 互换，不用 H753 / PDB-A）。

## 编译

```bash
./waf configure --board ZZZ-EVTOL-H743
./waf plane
```
