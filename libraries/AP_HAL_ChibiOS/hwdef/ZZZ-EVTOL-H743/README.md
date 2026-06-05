# ZZZ EVTOL-H743

消费级垂起进阶核心板：**STM32H743 + BMI088 + 板载 QMC5883P**，**W25Q128 SPI4 日志**（无 SD 卡座），CAN，外置 **`ZZZ-EVTOL-PDB-B`**（与 **F405** 互换，**不** 用于 H753）。

## 传感器

| 器件 | 接口 | 说明 |
|------|------|------|
| BMI088 | SPI1 | 六轴 IMU（双 CS，**无磁力计**） |
| QMC5883P / **IST8310** 二选一 | I2C2 `0x0D` / `0x0E` | **板载罗盘** |
| SPL06 / DPS310 | I2C2 `0x76` | 气压计 |
| 外置罗盘 | I2C1（GPS 口） | `HAL_PROBE_EXTERNAL_I2C_COMPASSES` |
| W25Q128 | SPI4 | 黑匣子日志，CS=**PC8**，SCK/MISO/MOSI=**PE12/13/14** |

## 存储

与 `ZZZ-EVTOL-F405` 相同采用 **SPI Flash**，不焊接 microSD 卡座（省 BOM、减振动接触不良）。刷机与调参走 USB。

## 磁力计

- **I2C2**：板载气压计 + 罗盘（5883P 或 IST8310，**只贴一颗**）  
- **I2C1**：GPS 六芯线 SDA/SCL，外置罗盘  

与 H753 相同策略：板载 + GPS 外置；**配套 PDB-B**（与 F405 互换，不用 H753 / PDB-A）。

## 编译

```bash
./waf configure --board ZZZ-EVTOL-H743
./waf plane
```
