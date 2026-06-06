# 磁罗盘两步快速校准 — 地面站对接文档

**固件版本：** `v2.0.2`（`ardupilot-ubuntu` 分支）  
**适用硬件：** EFT_CAAC 及基于本固件的定制飞控  
**文档日期：** 2026-06-06

---

## 1. 概述

本固件对 ArduPilot 标准磁罗盘校准做了深度定制，实现了**两步快速旋转校准**：

1. **第一步**：飞控水平放置，绕偏航轴旋转 ≥360°
2. **第二步**：飞控机头朝下（俯仰 < −30°），绕偏航轴再旋转 ≥360°

校准全程使用**标准 MAVLink 协议**，无需任何非标消息，地面站只需实现 MAVLink 标准接口即可完成对接。

---

## 2. MAVLink 消息接口

### 2.1 发起校准（地面站 → 飞控）

使用标准命令：`MAV_CMD_DO_START_MAG_CAL`（ID = 42424）

| param | 含义 | 推荐值 |
|-------|------|--------|
| param1 | 罗盘掩码（0 = 校准全部） | `0` |
| param2 | 失败后自动重试 | `0`（禁用，避免无限重试） |
| param3 | 完成后自动保存 | `1`（建议开启） |
| param4 | 延迟启动秒数 | `0` |
| x | 完成后自动重启 | `0` |

> **注意：** 飞控必须处于**未解锁（disarmed）**状态，否则返回 `MAV_RESULT_FAILED`。

**MAVLink 发送示例（Python / pymavlink）：**

```python
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_DO_START_MAG_CAL,
    0,       # confirmation
    0,       # param1: 0=全部罗盘
    0,       # param2: retry=0
    1,       # param3: autosave=1
    0,       # param4: delay=0
    0, 0, 0  # x/y/z: autoreboot=0
)
```

---

### 2.2 取消校准（地面站 → 飞控）

`MAV_CMD_DO_CANCEL_MAG_CAL`（ID = 42426），param1 = 0（取消全部）

---

### 2.3 进度上报（飞控 → 地面站）

飞控持续以约 **2 Hz** 发送 `MAG_CAL_PROGRESS`（消息 ID = 191）：

| 字段 | 类型 | 含义 |
|------|------|------|
| `compass_id` | uint8 | 罗盘编号（0-based） |
| `cal_mask` | uint8 | 正在校准的罗盘掩码 |
| `cal_status` | uint8 | 当前校准状态（见下方枚举） |
| `attempt` | uint8 | 当前尝试次数 |
| `completion_pct` | float | 完成百分比（0~100） |
| `completion_mask` | uint8[10] | 球面方向覆盖掩码 |

**`cal_status` 枚举值（`MAG_CAL_STATUS`）：**

| 值 | 名称 | 含义 |
|----|------|------|
| 0 | `MAG_CAL_NOT_STARTED` | 未开始 |
| 1 | `MAG_CAL_WAITING_TO_START` | 等待启动 |
| 2 | `MAG_CAL_RUNNING_STEP_ONE` | 采集中（水平阶段） |
| 3 | `MAG_CAL_RUNNING_STEP_TWO` | 采集中（朝下阶段 / 拟合阶段） |
| 4 | `MAG_CAL_SUCCESS` | 成功 |
| 5 | `MAG_CAL_FAILED` | 失败 |
| 6 | `MAG_CAL_BAD_ORIENTATION` | 安装方向错误 |
| 7 | `MAG_CAL_BAD_RADIUS` | 地磁场强度异常 |

> **本固件进度条映射建议：**
> - `status == 2`，`completion_pct` 在 0→50：显示"第1步 水平旋转"进度
> - `status == 2`，`completion_pct` 在 50→100：显示"第2步 机头朝下旋转"进度
> - `status == 3`：显示"计算中..."
> - `status == 4`：显示"校准成功，请重启"
> - `status >= 5`：显示"校准失败"

---

### 2.4 结果上报（飞控 → 地面站）

校准结束（成功或失败）后，飞控发送一次 `MAG_CAL_REPORT`（消息 ID = 192）：

| 字段 | 类型 | 含义 |
|------|------|------|
| `compass_id` | uint8 | 罗盘编号 |
| `cal_mask` | uint8 | 参与校准的罗盘掩码 |
| `cal_status` | uint8 | 最终状态（4=成功，5=失败等） |
| `autosaved` | uint8 | 1=已自动保存到飞控 |
| `fitness` | float | 拟合误差均方根（越小越好，< 5 为优秀） |
| `ofs_x/y/z` | float | 硬铁偏置（mGauss） |
| `diag_x/y/z` | float | 软铁对角缩放 |
| `offdiag_x/y/z` | float | 软铁非对角缩放 |

**地面站推荐处理逻辑：**

```
收到 MAG_CAL_REPORT：
  if cal_status == 4 (SUCCESS):
    if autosaved == 1:
      提示"校准成功，参数已保存，请重启飞控"
    else:
      提示"校准成功，请手动保存参数并重启飞控"
    显示 fitness 值供参考
  else:
    提示"校准失败（原因: status=X），请重试"
```

---

### 2.5 飞控主动推送的 STATUSTEXT 提示

本固件在关键节点通过 `STATUSTEXT`（消息 ID = 253）发送中文提示，地面站可直接展示：

| 时机 | 内容示例 | severity |
|------|---------|---------|
| 校准启动 | `MagCal #1: 请保持水平旋转一圈` | NOTICE |
| 水平阶段完成 | `MagCal #1: 水平完成 请机头朝下旋转` | NOTICE |
| 校准成功 | `MagCal #1: 指南针校准成功` | INFO |

> `#N` 为罗盘编号，从 1 开始（与地面站常见显示一致）。  
> 若同时校准多颗罗盘，会分别推送各自编号的消息。

---

## 3. 校准流程时序

```
地面站                              飞控
  │                                  │
  │── MAV_CMD_DO_START_MAG_CAL ──►  │  开始校准
  │                                  │
  │◄── STATUSTEXT: "请保持水平旋转" ─│
  │                                  │
  │◄── MAG_CAL_PROGRESS (循环) ─────│  status=2, pct 0→50
  │                                  │  （用户水平旋转飞控 ≥360°）
  │                                  │
  │◄── STATUSTEXT: "水平完成 请机头朝下" │
  │                                  │
  │◄── MAG_CAL_PROGRESS (循环) ─────│  status=2, pct 50→100
  │                                  │  （用户机头朝下旋转 ≥360°）
  │                                  │
  │◄── MAG_CAL_PROGRESS ────────────│  status=3（拟合计算中）
  │                                  │
  │◄── STATUSTEXT: "指南针校准成功" ─│
  │                                  │
  │◄── MAG_CAL_REPORT ──────────────│  status=4, autosaved=1
  │                                  │
（用户重启飞控）
```

---

## 4. 用户操作规范

地面站 UI 校准引导界面应提示用户按以下步骤操作：

1. **解锁前**确保飞控未解锁（否则飞控拒绝校准）
2. **发起校准**后，平放飞控，绕偏航轴**匀速旋转一整圈（360°以上）**
   - 转速建议：30~90°/秒（约 4~12 秒一圈）
   - 旋转时保持 pitch 角 > −20°（近似水平）
3. 收到"水平完成"提示后，**将机头朝下倾斜 30° 以上**，继续绕偏航轴旋转一整圈
   - 俯仰角需保持 < −30°（可用进度条或角度指示辅助）
4. 收到"指南针校准成功"后，**重启飞控**使参数生效

---

## 5. 关键参数说明（供飞控工程师参考）

以下参数在 `EFT_CAAC/defaults.parm` 中已预置，地面站无需修改：

| 参数 | 值 | 说明 |
|------|-----|------|
| `COMPASS_CAL_FIT` | 32 | 拟合误差门槛（默认 16，放宽便于快速校准） |
| `COMPASS_OFFS_MAX` | 1800 | 最大允许偏置（mGauss） |
| `COMPASS_AUTO_ROT` | 0 | 禁用自动方向检测（本固件不依赖此功能） |

固件内部关键常量（`CompassCalibrator.h`，v2.0.2）：

| 常量 | 值 | 说明 |
|------|----|------|
| `COMPASS_CAL_NUM_SAMPLES` | 100 | 总采集点数（50 水平 + 50 朝下） |
| `COMPASS_CAL_PHASE1_SAMPLES` | 50 | 第一阶段（水平）点数目标 |
| `COMPASS_CAL_SPACING_SAMPLES` | 100 | 间距计算用虚拟 N |
| `COMPASS_CAL_PHASE1_PITCH_MIN` | −0.349 rad（−20°） | 水平阶段最低 pitch 限制 |
| `COMPASS_CAL_PHASE2_PITCH_MAX` | −0.524 rad（−30°） | 朝下阶段最高 pitch 限制 |

---

## 6. 错误处理建议

| 飞控输出 | 原因 | 建议处理 |
|---------|------|---------|
| `MAV_RESULT_FAILED` | 飞控已解锁 | 提示用户先解锁 → 关闭电机 |
| `cal_status = 5 (FAILED)` | 拟合误差过大 | 提示重试；检查周围是否有强磁场干扰 |
| `fitness > 16` | 校准精度较低 | 警告用户，建议重新校准 |
| `autosaved = 0` | 未自动保存 | 提示手动保存参数（`MAV_CMD_PREFLIGHT_STORAGE`） |
| PreArm: Check mag field (xy diff > 100) | 两颗罗盘校准结果差异过大 | 提示重新校准 |

---

## 7. 版本记录

| 版本 | 说明 |
|------|------|
| v2.0.2 | 总点数 100（水平50+朝下50），增加 360° 旋转锁 |
| v2.0.1 | 总点数 60，增加 360° 旋转锁，跳过方向检查和 thin_samples |
| v2.0.0 | 两步校准初版（30+10 点） |

---

*本文档由飞控固件团队维护，如有疑问请联系固件工程师。*
