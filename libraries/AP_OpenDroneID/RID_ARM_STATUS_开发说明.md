# RID 模块 → 飞控「能否解锁」开发说明

面向 RID（Remote ID / 远程识别）模块固件开发工程师。

**结论：用现成 OpenDroneID 协议即可，飞控解锁逻辑不用改代码。**  
RID 按本文发消息；飞控参数打开强制检查后，消息内容直接决定能不能解锁。

---

## 1. 用哪条消息

| 项 | 值 |
| --- | --- |
| 消息名 | `OPEN_DRONE_ID_ARM_STATUS` |
| msgid | **12918**（MAVLink common） |
| 方向 | **RID → 飞控**（单向） |

字段只有两个：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `status` | uint8 | `0` = `GOOD_TO_ARM`（允许解锁）；`1` = `PRE_ARM_FAIL_GENERIC`（不允许） |
| `error` | char[50] | 不允许时的原因文字，最多 49 有效字符 + `'\0'`，飞手端原样显示 |

「能不能解锁」由 **RID 自己判定**；飞控只做：

1. 3 秒内有没有收到这条消息  
2. `status` 是不是 `0`

---

## 2. 交互时序

```text
RID                                      飞控
 |                                         |
 |-- ARM_STATUS (status=FAIL/GOOD) 1Hz --->|
 |-- ARM_STATUS ... ---------------------->|  缓存最新包 + 时间戳
 |                                         |
 |                    飞手点解锁时：        |
 |                    · 距上次包 ≤ 3000ms  |
 |                    · status == GOOD     |
 |                    → 通过本项检查        |
```

- **周期**：固定约 **1 Hz**，允许/不允许都要发  
- **上电即发**：先 FAIL 再转 GOOD 没有问题  
- **不能断流**：停发超过 **3 秒** → 飞控报 `"ARM_STATUS not available"`，拒绝解锁  

---

## 3. 飞控侧条件（RID 必须知道）

### 3.1 必须打开强制检查

只有 `DID_OPTIONS` 的 bit0 **`EnforceArming`** 打开时，飞控才会用 ARM_STATUS 拦解锁。

| 参数 | 作用 | EFT 当前默认 | 要强制 RID 控解锁时 |
| --- | --- | --- | --- |
| `DID_ENABLE` | 打开 OpenDroneID | `1` | 保持 `1` |
| `DID_OPTIONS` | bit0 = EnforceArming | **`0`（未强制）** | 改为 **`1`** |
| `DID_MAVPORT` | RID 所在 SERIALn 口号 | `2` | 必须等于 RID 实际串口 |

> 飞控固件不用改。联调前让飞控/地面站把 `DID_OPTIONS=1` 配好（或改 `defaults.parm`）。

### 3.2 只认「RID 那一路」串口

飞控收到 12918 时：

```text
if (接收通道 == DID_MAVPORT 对应的 MAVLink 通道)
    才更新 arm_status
否则
    丢弃
```

EFT 硬件注释里 RID 接在 **SERIAL4（REMOTEID）**，但默认 `DID_MAVPORT=2`。  
**两边必须一致**，否则飞控永远收不到你的 ARM_STATUS：

- RID 走 SERIAL4 → `DID_MAVPORT` 应为 **4**  
- RID 走 SERIAL2 → `DID_MAVPORT` 保持 **2**

### 3.3 打开 EnforceArming 后还会多查几项

同一次预解锁里，除 ARM_STATUS 外还会检查（飞控已实现）：

| 检查 | 失败提示 |
| --- | --- |
| BasicID 已设 UA 类型 | `"UA_TYPE required in BasicID"` |
| 操作员经纬度已设置（非 0,0） | `"operator location must be set"` |
| SYSTEM / SYSTEM_UPDATE 新鲜（约 3 s） | `"SYSTEM not available"` |
| ARM_STATUS 新鲜（约 3 s） | `"ARM_STATUS not available"` |
| ARM_STATUS.status != GOOD | 透传 RID 的 `error` 字符串 |

这些靠标准 OpenDroneID 消息满足（GCS 或飞控配置侧写入 BasicID / SYSTEM 等）。  
**RID 本模块的核心任务仍是周期发 12918。**

---

## 4. RID 推荐实现

### 4.1 发送伪代码

```c
#include <string.h>

void rid_send_arm_status_1hz(void)
{
    mavlink_open_drone_id_arm_status_t arm = {0};
    const char *reason = NULL;

    /* 按产品定义排查；命中第一条就报这条 */
    if (!gps_ok()) {
        reason = "RID: no GPS fix";
    } else if (!operator_id_ok()) {
        reason = "RID: operator ID missing";
    } else if (!network_ok()) {
        reason = "RID: no network/registration";
    } else if (!self_test_ok()) {
        reason = "RID: self-test failed";
    } else if (!uas_serial_ok()) {
        reason = "RID: UAS serial invalid";
    }

    if (reason == NULL) {
        arm.status = MAV_ODID_ARM_STATUS_GOOD_TO_ARM;  /* 0 */
        arm.error[0] = '\0';
    } else {
        arm.status = MAV_ODID_ARM_STATUS_PRE_ARM_FAIL_GENERIC;  /* 1 */
        strncpy(arm.error, reason, sizeof(arm.error) - 1);
        arm.error[sizeof(arm.error) - 1] = '\0';
    }

    mavlink_msg_open_drone_id_arm_status_send_struct(chan, &arm);
}
```

要点：

1. **一次只报一条原因**（优先级串行），飞手更好懂  
2. **全部通过才发 GOOD**  
3. **必须 1 Hz 持续发**，与当前好坏无关  

### 4.2 多原因短码（可选）

`error` 只有 50 字节，调试时可拼短码，勿超 49 字节：

```c
/* 例："RID FAIL: GPS,OPID,NET" */
```

量产建议仍用「一次一条中文/英文短句」。

---

## 5. RID 侧开发清单

```text
1. MAVLink 口接到飞控 RID 串口（与 DID_MAVPORT 一致）
2. 上电后 1 Hz 发 OPEN_DRONE_ID_ARM_STATUS(12918)
3. 自检失败 → status=1 + error；全过 → status=0
4. 与飞控确认：DID_ENABLE=1、DID_OPTIONS=1、DID_MAVPORT=RID 串口号
5. 联调解锁：FAIL 能拦住；GOOD + 其它前置满足后能解锁；停发 3s 被拦
```

---

## 6. 自测清单

- [ ] 上电后 1 Hz 稳定发 12918，中途不断流  
- [ ] `DID_OPTIONS=1` 且 `DID_MAVPORT` 与接线一致  
- [ ] 人为制造各失败原因，飞手能看到对应 `error`，且无法解锁  
- [ ] 全部通过后 `status=0`，在其它 OpenDroneID 前置也满足时可解锁  
- [ ] 拔掉 RID / 停发 >3 s → 飞控报 `"ARM_STATUS not available"`  
- [ ] `error` ≤ 49 字节，无乱码  
- [ ] 从错误串口发 12918 时，飞控**不应**更新状态（验证通道过滤）

---

## 7. 常见问题

**Q1. 要不要自定义 MAV_CMD / 私有消息？**  
A. 不需要。标准 12918 即可。

**Q2. 飞控要改代码吗？**  
A. 不需要。收包与预解锁检查已在 `AP_OpenDroneID` 里。

**Q3. 发了 GOOD 仍解不了锁？**  
A. 查：`DID_OPTIONS` 是否为 1；`DID_MAVPORT` 是否对口；BasicID / 操作员位置 / SYSTEM 是否满足；是否断流超时。

**Q4. GCS 能不能替 RID 发 ARM_STATUS？**  
A. 若 GCS 不在 `DID_MAVPORT` 那路，飞控会丢弃。应以 RID 模块本机周期发送为准。

---

## 8. 相关文件

| 位置 | 说明 |
| --- | --- |
| `libraries/AP_OpenDroneID/AP_OpenDroneID.cpp` → `handle_msg()` | 收 12918（仅 `_chan`） |
| 同文件 `pre_arm_check_nolock()` | 3 s 新鲜度 + status + 其它前置 |
| 本文件 | RID 解锁对接说明 |
| `FactorySN_RID对接说明.md` | RID **读取**产品型号 SN（无关解锁） |
| `EFT_RID_CONFIG_地面站开发说明4.md` | 地面站查 RID 配置/状态（含 ARM 新鲜度位） |
