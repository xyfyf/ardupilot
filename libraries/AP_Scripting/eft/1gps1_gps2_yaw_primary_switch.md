# 1gps1_gps2_yaw_primary_switch 脚本说明

脚本文件: `1gps1_gps2_yaw_primary_switch.lua`

## 1. 适用对象

EFT_CAAC 飞控板，GPS 接线约定：
- **GPS1** = ublox GPS（instance 0）
- **GPS2** = UM982 双天线 RTK，挂在 SERIAL7（instance 1）

## 2. 功能

| 检测到的 GPS 情况                                      | EK3_SRC1_YAW   | GPS_PRIMARY |
|-------------------------------------------------------|----------------|-------------|
| **GPS2 (UM982) 在线 & 定位状态 >= 3D Fix**            | `2` (GPS yaw)  | `1` (GPS2)  |
| **只识别到 GPS1**（GPS2 不在线 / 状态 < 3D Fix）      | `1` (Compass)  | `0` (GPS1)  |

切换条件：连续 5 秒满足才执行切换（防抖动）。
切换后通过 `param:set_and_save` 持久化到 flash，下次断电重启仍生效。

## 3. 安全策略

- **`EK3_SRC1_YAW` 是 EKF 主航向源**，飞行中切换会触发 EKF yaw 重对齐，姿态可能出现扰动。所以脚本设计为：
  - **未解锁 (Disarmed)**：执行真正的参数切换。
  - **已解锁 (Armed)**：仅在状态发生变化时通过地面站告警提示，不修改参数。
- 启动后延迟 5 秒再开始第一次判定，等 GPS / UM982 上电稳定。
- 仅在状态真正发生变化时写 flash，避免反复磨损。

## 4. 安装

1. 把 `1gps1_gps2_yaw_primary_switch.lua` 拷贝到飞控 SD 卡 `APM/scripts/` 目录。
2. 确认参数：
   - `SCR_ENABLE = 1`
   - `GPS1_TYPE`、`GPS2_TYPE` 已正确配置（`defaults.parm` 内 `GPS1_TYPE=1`, `GPS2_TYPE=25`）
3. 重启飞控，地面站应提示：`1gps1_gps2_yaw_primary_switch loaded`。

## 5. 测试用例

### 用例 1：GPS2 在线 (UM982 RTK 正常)
- **操作**：未解锁状态下，GPS2 达到 3D Fix 或以上。
- **预期**：约 5 秒后地面站提示
  `GPS2(UM982) RTK OK: EK3_SRC1_YAW=2(GPS), GPS_PRIMARY=1(GPS2)`，
  并且参数被保存到 flash。

### 用例 2：拔掉/屏蔽 UM982
- **操作**：未解锁状态下，断开 UM982 与 SERIAL7 的连接，或让 UM982 卫星信号变差到 < 3D Fix。
- **预期**：约 5 秒后地面站提示
  `GPS2 not detected: EK3_SRC1_YAW=1(Compass), GPS_PRIMARY=0(GPS1)`。
  此时飞机回退到 GPS1 + 磁罗盘 yaw。

### 用例 3：飞行中切换抑制
- **操作**：解锁起飞，飞行中拔掉 UM982 RX 线模拟 GPS2 丢失。
- **预期**：脚本**不**修改 `EK3_SRC1_YAW` / `GPS_PRIMARY`，仅每 10 秒最多一次告警：
  `Armed: GPS source state changed GPS2_OK -> GPS1_ONLY (params kept)`。
  保证飞行中 EKF yaw 不被重对齐。

### 用例 4：落地后恢复
- **操作**：用例 3 后落地上锁，UM982 仍未恢复。
- **预期**：约 5 秒后参数被切换为 GPS1 + Compass yaw，地面站提示同用例 2。

## 6. 注意事项

- 本脚本和 `1gps_rtk_failsafe_land.lua`（飞行中 RTK 异常保命）功能上**互补**：
  - 本脚本：起飞前根据 GPS2 是否在线决定 EKF yaw 源 / 主 GPS。
  - `1gps_rtk_failsafe_land.lua`：飞行中 RTK 出现"离谱异常值"时切换 GPS 或迫降。
  两者可同时启用。
- 如果你不想让脚本动 `GPS_PRIMARY`（例如希望 `GPS_AUTO_SWITCH` 自己决定），把脚本里 `apply_state` 中对 `GPS_PRIMARY` 的两行注释掉即可。
