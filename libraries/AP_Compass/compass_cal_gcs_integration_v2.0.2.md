# 磁罗盘两步快速校准 — 地面站对接文档

**固件版本：** `v2.0.3`（`ardupilot-ubuntu` 分支）  
**适用硬件：** EFT_CAAC 及基于本固件的定制飞控  
**文档日期：** 2026-06-06

---

## 1. 概述

本固件对 ArduPilot 标准磁罗盘校准做了深度定制，实现了**两步旋转校准**，支持同时校准多颗（≥1）磁力计：

| 步骤 | 姿态要求 | IMU 旋转要求 |
|------|----------|-------------|
| 第一步 | 飞控近似水平（pitch > −20°） | 偏航累计 ≥ 360° |
| 第二步 | 机头朝下（pitch < −30°） | 先采满样本，**再额外偏航 ≥ 360°** |

> **关键设计**：第二步要求在 50 个样本采满**之后**，IMU 检测到从零起算的完整一圈，才开始椭球拟合。即"先收样，再转圈"，保证数据质量。

校准全程使用**标准 MAVLink 协议**，无需任何非标消息。

---

## 2. MAVLink 消息接口

### 2.1 发起校准（地面站 → 飞控）

`MAV_CMD_DO_START_MAG_CAL`（ID = 42424）

| param | 含义 | 推荐值 |
|-------|------|--------|
| param1 | 罗盘掩码（0 = 校准全部） | `0` |
| param2 | 失败后自动重试 | `0`（禁用） |
| param3 | 完成后自动保存 | `1` |
| param4 | 延迟启动秒数 | `0` |
| x | 完成后自动重启 | `0` |

> 飞控必须处于**未解锁（disarmed）**状态，否则返回 `MAV_RESULT_FAILED`。

```python
# pymavlink 示例
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_DO_START_MAG_CAL,
    0,        # confirmation
    0,        # param1: 0 = 全部罗盘
    0,        # param2: retry = 0
    1,        # param3: autosave = 1
    0,        # param4: delay = 0
    0, 0, 0   # x/y/z: autoreboot = 0
)
```

---

### 2.2 取消校准（地面站 → 飞控）

`MAV_CMD_DO_CANCEL_MAG_CAL`（ID = 42426），param1 = 0（取消全部）

---

### 2.3 进度上报（飞控 → 地面站）

飞控以约 **2 Hz** 发送 `MAG_CAL_PROGRESS`（消息 ID = 191），**每颗磁力计独立发送一条**：

| 字段 | 类型 | 含义 |
|------|------|------|
| `compass_id` | uint8 | 罗盘编号（0-based） |
| `cal_mask` | uint8 | 本次参与校准的罗盘位掩码 |
| `cal_status` | uint8 | 当前校准状态（见下方枚举） |
| `attempt` | uint8 | 当前尝试次数 |
| `completion_pct` | float | 完成百分比（0~100） |
| `completion_mask` | uint8[10] | 球面方向覆盖位掩码 |

**`cal_status` 枚举（`MAG_CAL_STATUS`）：**

| 值 | 名称 | 含义 |
|----|------|------|
| 0 | `MAG_CAL_NOT_STARTED` | 未开始 |
| 1 | `MAG_CAL_WAITING_TO_START` | 等待启动 |
| 2 | `MAG_CAL_RUNNING_STEP_ONE` | 采集中（含水平阶段和朝下采样阶段） |
| 3 | `MAG_CAL_RUNNING_STEP_TWO` | 椭球拟合计算中 |
| 4 | `MAG_CAL_SUCCESS` | 成功 |
| 5 | `MAG_CAL_FAILED` | 失败 |
| 6 | `MAG_CAL_BAD_ORIENTATION` | 安装方向错误 |
| 7 | `MAG_CAL_BAD_RADIUS` | 地磁场强度异常 |

**进度条映射建议（按 compass_id 分别显示）：**

| `cal_status` | `completion_pct` | 建议 UI 文案 |
|---|---|---|
| 2 | 0 → 50 | 第1步：水平旋转中… |
| 2 | 50 → 99 | 第2步：采样中（请保持机头朝下）… |
| 3 | — | 计算中… |
| 4 | — | ✓ 成功，请重启 |
| ≥5 | — | ✗ 失败，请重试 |

---

### 2.4 结果上报（飞控 → 地面站）

校准结束后，**每颗磁力计各发送一次** `MAG_CAL_REPORT`（消息 ID = 192）：

| 字段 | 类型 | 含义 |
|------|------|------|
| `compass_id` | uint8 | 罗盘编号 |
| `cal_mask` | uint8 | 参与校准的罗盘掩码 |
| `cal_status` | uint8 | 最终状态（4=成功，5=失败等） |
| `autosaved` | uint8 | 1=已自动保存 |
| `fitness` | float | 拟合误差均方根（< 5 优秀，< 10 可用） |
| `ofs_x/y/z` | float | 硬铁偏置（mGauss） |
| `diag_x/y/z` | float | 软铁对角缩放 |
| `offdiag_x/y/z` | float | 软铁非对角缩放 |

**多磁力计完成判定：**

```
对每个 compass_id（0, 1, ...）维护收到的 MAG_CAL_REPORT：
  - 当 cal_mask 中所有 bit 对应的 compass_id 均已收到 REPORT 且 status == 4
    → 所有罗盘校准成功，提示"请重启飞控"
  - 若任一 compass_id 的 status != 4
    → 提示"磁力计 #N 校准失败，建议全部重新校准"
```

---

### 2.5 STATUSTEXT 提示（飞控 → 地面站）

飞控在关键节点通过 `STATUSTEXT`（消息 ID = 253）发送中文提示，**每颗磁力计独立推送**：

| 时机 | 内容格式 | severity |
|------|---------|---------|
| 校准启动 | `MagCal #N: 请保持水平旋转一圈` | NOTICE |
| 水平阶段完成，切换到朝下 | `MagCal #N: 水平完成 请机头朝下旋转` | NOTICE |
| 校准成功 | `MagCal #N: 指南针校准成功` | INFO |

> `N` 为罗盘编号，从 **1** 开始（`compass_id + 1`）。  
> 双磁力计时，#1 和 #2 的消息**相互独立**，进度可能不同步，地面站应分开显示。

---

## 3. 单磁力计校准时序

```
地面站                              飞控 (#1)
  │                                  │
  ├── MAV_CMD_DO_START_MAG_CAL ────► │
  │                                  │
  │ ◄── STATUSTEXT: "MagCal #1: 请保持水平旋转一圈"
  │ ◄── MAG_CAL_PROGRESS  status=2, pct 0→50    （水平采样中）
  │                                  │  用户水平旋转 ≥360°
  │ ◄── STATUSTEXT: "MagCal #1: 水平完成 请机头朝下旋转"
  │ ◄── MAG_CAL_PROGRESS  status=2, pct 50→100  （朝下采样中）
  │                                  │  用户机头朝下旋转满一整圈（采样+旋转同步完成）
  │ ◄── MAG_CAL_PROGRESS  status=3             （椭球拟合中）
  │ ◄── STATUSTEXT: "MagCal #1: 指南针校准成功"
  │ ◄── MAG_CAL_REPORT    status=4, autosaved=1
  │
（提示用户重启飞控）
```

---

## 4. 双磁力计校准时序

两颗磁力计**同时启动、独立运行**，各自跟踪自己的采样状态和旋转量。用户的旋转动作对两颗磁力计同时生效，但因采样门槛（`accept_sample` 间距算法）不同，进度可能略有差异。

```
地面站            飞控 #1                          飞控 #2
  │                │                                │
  ├── MAV_CMD_DO_START_MAG_CAL ──────────────────►  │
  │                │                                │
  │ ◄── STATUSTEXT: "MagCal #1: 请保持水平旋转一圈"
  │ ◄── STATUSTEXT: "MagCal #2: 请保持水平旋转一圈"
  │                │                                │
  │ ◄── PROGRESS   │ #1 status=2, pct↑             │
  │ ◄── PROGRESS   │                 #2 status=2, pct↑
  │                │    ──── 用户水平旋转一圈 ────   │
  │                │                                │
  │ ◄── STATUSTEXT: "MagCal #1: 水平完成 请机头朝下旋转"
  │ ◄── STATUSTEXT: "MagCal #2: 水平完成 请机头朝下旋转"
  │   （两者几乎同时，偶尔先后相差几秒）
  │                │                                │
  │ ◄── PROGRESS   │ #1 status=2, pct 50→100       │
  │ ◄── PROGRESS   │                 #2 status=2, pct 50→100
  │                │    ── 用户机头朝下旋转满一圈 ── │
  │                │                                │
  │ ◄── PROGRESS   │ #1 status=3（拟合中）          │
  │ ◄── PROGRESS   │                 #2 status=3
  │ ◄── STATUSTEXT: "MagCal #1: 指南针校准成功"
  │ ◄── STATUSTEXT: "MagCal #2: 指南针校准成功"
  │ ◄── MAG_CAL_REPORT  compass_id=0, status=4
  │ ◄── MAG_CAL_REPORT  compass_id=1, status=4
  │
  └── 所有罗盘 status=4 → 提示"请重启飞控"
```

### 4.1 双磁力计 GCS 推荐处理逻辑

```python
compasses = {}  # compass_id → { status, pct, report }

def on_mag_cal_progress(msg):
    cid = msg.compass_id
    compasses.setdefault(cid, {})['status'] = msg.cal_status
    compasses[cid]['pct'] = msg.completion_pct
    update_ui(cid)

def on_mag_cal_report(msg):
    cid = msg.compass_id
    compasses.setdefault(cid, {})['report'] = msg
    if all_done():
        show_result()

def all_done():
    # cal_mask 中每个 bit 对应的磁力计都收到了 REPORT
    for cid, data in compasses.items():
        if 'report' not in data:
            return False
    return True

def show_result():
    failed = [cid for cid, d in compasses.items()
              if d['report'].cal_status != 4]
    if failed:
        show_error(f"磁力计 #{[c+1 for c in failed]} 校准失败，请重试")
    else:
        show_success("所有罗盘校准成功，请重启飞控")
```

### 4.2 双磁力计进度不同步处理

两颗磁力计进度可能相差几秒。地面站 UI 建议：

- **分别显示两个进度条**，并标注罗盘编号
- 当一颗已显示"样本已满 请旋转"，而另一颗还在采样时，**不要提前告知用户停止旋转**
- 只有当**两颗磁力计均收到 `MAG_CAL_REPORT` 且 status=4** 后，才提示重启

---

## 5. 用户操作规范（UI 引导）

地面站 UI 校准引导界面应提示用户按以下步骤操作：

1. **确认飞控未解锁**（Disarmed），否则飞控拒绝校准
2. **发起校准**，平放飞控，绕偏航轴**匀速旋转一整圈（≥360°）**
   - 建议转速：30~90°/秒（约 4~12 秒一圈）
   - 保持 pitch > −20°（近似水平）
3. 收到"水平完成"提示后，**将机头朝下倾斜 30° 以上**，继续旋转
   - 朝下采样无需特定转速，正常旋转即可
4. 收到"**样本已满 请保持朝下旋转一圈**"提示后，**在保持朝下的状态下再转一整圈**
   - 这是最终触发椭球拟合的关键步骤
5. 收到"指南针校准成功"后，**重启飞控**使参数生效

> 双磁力计时，步骤 2~5 对两颗磁力计同时生效，用户无需重复操作。

---

## 6. 关键参数说明

以下参数在 `EFT_CAAC/defaults.parm` 中已预置，地面站无需修改：

| 参数 | 值 | 说明 |
|------|-----|------|
| `COMPASS_CAL_FIT` | 32 | 拟合误差门槛（默认 16，适当放宽） |
| `COMPASS_OFFS_MAX` | 1800 | 最大允许硬铁偏置（mGauss） |
| `COMPASS_AUTO_ROT` | 0 | 禁用自动方向检测 |

固件内部关键常量（`CompassCalibrator.h`，v2.0.3）：

| 常量 | 值 | 说明 |
|------|----|------|
| `COMPASS_CAL_NUM_SAMPLES` | 100 | 总采集点数 |
| `COMPASS_CAL_PHASE1_SAMPLES` | 50 | 第一阶段（水平）目标点数 |
| `COMPASS_CAL_SPACING_SAMPLES` | 100 | 间距计算用虚拟 N |
| `COMPASS_CAL_PHASE1_PITCH_MIN` | −0.349 rad（−20°） | 水平阶段最低 pitch |
| `COMPASS_CAL_PHASE2_PITCH_MAX` | −0.524 rad（−30°） | 朝下阶段最高 pitch |

---

## 7. 错误处理建议

| 现象 | 原因 | 建议处理 |
|------|------|---------|
| `MAV_RESULT_FAILED` | 飞控已解锁 | 提示先解锁飞控（关电机） |
| `cal_status = 5 (FAILED)` | 拟合误差过大 | 提示远离强磁场后重试 |
| `fitness > 16` | 校准精度较低 | 警告，建议重新校准 |
| `autosaved = 0` | 未自动保存 | 提示手动执行 `MAV_CMD_PREFLIGHT_STORAGE` |
| `PreArm: Check mag field (xy diff > 100)` | 两颗罗盘校准结果差异过大 | 提示重新校准全部罗盘 |
| #1 成功但 #2 失败 | 某颗磁力计数据质量差 | 提示**重新校准全部**（两颗需同时完成） |

---

## 8. 版本记录

| 版本 | 说明 |
|------|------|
| v2.0.3 | 朝下旋转改为样本满后再计圈（防止半圈完成），新增"样本已满"STATUSTEXT，补充双磁力计对接说明 |
| v2.0.2 | 总点数 100（水平50+朝下50），增加 IMU 陀螺仪积分 360° 旋转锁 |
| v2.0.1 | 总点数 60，增加旋转锁，跳过方向检查和 thin_samples |
| v2.0.0 | 两步校准初版（30+10 点） |

---

*本文档由飞控固件团队维护，如有疑问请联系固件工程师。*
