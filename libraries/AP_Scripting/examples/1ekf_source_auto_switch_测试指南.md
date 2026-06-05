# EKF3 数据源自动降级与保命脚本测试指南

本脚本 (`1ekf_source_auto_switch.lua`) 旨在提升无人机的飞行安全性：**起飞前强制要求 GPS 正常（否则禁止解锁），飞行中若 GPS/RTK 突发故障，自动切换 EKF 数据源并切入定高模式保命。**

## 1. 测试前准备
1. 确认脚本已放入飞控 SD 卡 `AP_Scripting/scripts/` 目录，并开启 Lua 功能 (`SCR_ENABLE = 1`)。
2. 确认主数据源 `EK3_SRC1` 依赖 GPS。
3. 重启飞控，地面站应提示 `EKF Source Auto-Switch Script Loaded`。

## 2. 测试用例与预期结果

### 用例 1：起飞前 RTK 异常拦截测试
*   **操作**：在未解锁状态下，保持 GPS 为普通 3D Fix 或 No Fix（未达到 RTK Float/Fixed）。
*   **预期**：脚本会强制飞控使用主源 (`EK3_SRC1`)。此时尝试解锁，飞控原生预检会拦截并报错，**不允许解锁**，必须等待 RTK 达到要求。

### 用例 2：正常 RTK 起飞测试
*   **操作**：等待 GPS 达到 RTK Float 或 RTK Fixed。
*   **预期**：地面站提示 `GPS Good: Switched to EKF Source 1`。切入 Loiter 或定高模式，**正常解锁起飞**。
### 用例 3：飞行中 RTK 突发故障保命测试 (核心)
*   **操作**：无人机在空中悬停（Loiter 或 Auto 模式）时，迅速用锡纸罩住 GPS 天线，模拟空中 RTK/GPS 丢失。
*   **预期**：
    1. 地面站立即提示 `GPS Bad: Switched to EKF Source 2 (Baro)`。
    2. 脚本会自动将飞行模式**强制切换为 AltHold (定高模式)**，并提示 `Emergency: Switched to AltHold mode`。
    3. 无人机自动降级为气压计定高和姿态增稳，**不会发生严重掉高或姿态失控**，飞手可以手动接管飞行。

### 用例 4：飞行中磁力计干扰测试
*   **操作**：在用例 3（已切入备用源和定高模式）的基础上，拿强磁铁靠近飞控。
*   **预期**：地面站提示 `Compass bad, EK3_SRC2_YAW set to None`。无人机放弃磁力计，仅靠陀螺仪维持姿态，**继续保持增稳和定高**。

### 用例 5：降落上锁恢复测试
*   **操作**：降落并上锁。
*   **预期**：上锁后，脚本重新激活“起飞前强制 GPS”逻辑，地面站提示 `Disarmed: Forced EKF Source 1 (GPS required to arm)`。此时若未达到 RTK 状态，将无法再次解锁。