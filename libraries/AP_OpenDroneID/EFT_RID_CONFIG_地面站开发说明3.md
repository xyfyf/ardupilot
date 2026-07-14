# EFT Remote ID 配置查询 — 地面站开发说明

面向地面站（GCS）工程师。

飞控提供一对**请求/应答**消息，用于查询当前 Remote ID（RID）配置与运行状态（UAS ID、Operator ID、操作员位置、RID 模块是否在线等）。

| 项 | 内容 |
|----|------|
| 适用固件 | EFT 定制 ArduPilot（需 `AP_OPENDRONEID_ENABLED=1`，如 EFT_CAAC） |
| 消息定义源文件 | `modules/mavlink/message_definitions/v1.0/eft.xml` |
| 如何进入方言 | `common.xml` 中已 `<include>eft.xml</include>`（走 common / ardupilotmega / all 生成均可） |
| 飞控处理入口 | `AP_OpenDroneID::handle_rid_config_request()` |

---

## 1. 功能说明

| 动作 | 消息 | msgid |
|------|------|------:|
| 地面站 → 飞控：查询 / 清除 | `EFT_RID_CONFIG_REQUEST` | **517** |
| 飞控 → 地面站：应答 | `EFT_RID_CONFIG_STATUS` | **518** |

回包包含：

- UAS ID、Operator ID、Self ID
- 操作员经纬高
- DID 参数（`DID_ENABLE` / `DID_MAVPORT` / `DID_OPTIONS` / `DID_CANDRIVER`）
- RID 发射模块 ARM 状态与原因文字
- 汇总位图 `status_flags`（UI 可先读这个）

**517 的 `type` 字段：**

| type | 含义 |
|-----:|------|
| 0 | 仅查询当前状态，然后回 518 |
| 1 | **清除** RID 运行配置/状态后，再回 518 |

`type=1` 清除效果：

- 清空 UAS ID / Operator ID / Self ID / 操作员位置 / ARM_STATUS 缓存与新鲜度
- DID 参数恢复为默认并掉电保存：`DID_ENABLE=1`，`DID_MAVPORT=2`，`DID_OPTIONS=0`，`DID_CANDRIVER=0`
- 清除后本机可重新接收并写入新的 `OPEN_DRONE_ID_*` 配置

写入身份信息仍用标准 OpenDroneID：`BASIC_ID(12900)` / `OPERATOR_ID(12905)` / `SELF_ID(12903)` / `SYSTEM(12904)`。  
**不要**用 `MAV_CMD_REQUEST_MESSAGE(512)` 去“请求 517/518/12900”。

---

## 2. 强制要求：用仓库 eft.xml 重新生成绑定

### 2.1 CRC 必须一致

| msgid | 名称 | 载荷长度 | **CRC_EXTRA（必须）** |
|------:|------|----------:|----------------------:|
| 517 | `EFT_RID_CONFIG_REQUEST` | **4** | **209** |
| 518 | `EFT_RID_CONFIG_STATUS` | 127 | **197** |

地面站若手写消息类、用了旧定义或字段名不一致，CRC_EXTRA 会对不上。  
飞控解析失败时会**静默丢包**，表现为「发出 517，永远收不到 518」。

> 注意：相对旧版 3 字节请求（CRC=213），现已增加 `type` 字段，**必须按 4 字节 / CRC=209 重新生成绑定**。

### 2.2 字段必须完全一致（517）

```
uint8_t target_system
uint8_t target_component
uint8_t seq
uint8_t type          // 0=查询, 1=清除后查询
```

名称、类型、顺序都不能改，否则 CRC 会变。

### 2.3 生成步骤（给对接工程师）

1. 使用本仓库（或同步后的）  
   `modules/mavlink/message_definitions/v1.0/eft.xml`  
   以及已 include 它的 `common.xml`
2. 用 mavgen / 贵司现有流水线，对 **common**（或 ardupilotmega / all）重新生成 C# / Python / C 绑定
3. 确认生成结果中存在：
   - `MAVLINK_MSG_ID_EFT_RID_CONFIG_REQUEST == 517`
   - `MAVLINK_MSG_ID_EFT_RID_CONFIG_REQUEST_CRC == 209`
   - `MAVLINK_MSG_ID_EFT_RID_CONFIG_REQUEST_LEN == 4`
   - `MAVLINK_MSG_ID_EFT_RID_CONFIG_STATUS_CRC == 197`
4. **重编地面站**后再联调（只改 xml、不重生绑定无效）

### 2.4 用抓包自检 CRC（强烈建议）

正确发出的一帧 517 示例含义：

```
FD 04 00 00 <seq> <gcs_sys> <gcs_comp> 05 02 00 <target_sys> <target_comp> <req_seq> <type> <crc_lo> <crc_hi>
                 |----- mavlink2 header -----| |msgid=517 LE| |-------- payload 4B ---------| |-- crc --|
```

| 字节含义 | 正确值 |
|----------|--------|
| msgid | `05 02 00` → 517 |
| payload | 例如 `01 00 01 00` → target=1, comp=0, seq=1, type=0（查询） |
| payload 清除 | 例如 `01 00 02 01` → target=1, seq=2, **type=1（清除）** |
| CRC_EXTRA | 打包时必须用 **209** |

---

## 3. 交互时序

```
地面站                              飞控
  │                                   │
  │── EFT_RID_CONFIG_REQUEST (517) ──>│  USB / 数传等任意 GCS MAVLink 口
  │                                   │  （不是 DID_MAVPORT RID 专用口）
  │                                   │  读取 AP_OpenDroneID 内存状态
  │<─ EFT_RID_CONFIG_STATUS (518) ────│  同一通道回发
```

规则：

| 项 | 说明 |
|----|------|
| 协议形态 | 普通消息请求/应答，**无** `COMMAND_ACK` |
| `seq` | 地面站自增；518 原样回显，用于匹配 |
| 限流 | 飞控 **200 ms** 内重复请求直接忽略（无回包） |
| 建议刷新间隔 | ≥ 1 秒 |
| `DID_ENABLE=0` | 仍会回 518，但配置字段多为空，`status_flags` bit0=0 |

---

## 4. 请求：EFT_RID_CONFIG_REQUEST（517）

| 字段 | 类型 | 填法 |
|------|------|------|
| `target_system` | uint8 | 飞控 SYSID（心跳里的 system）。**0 = 广播**（本机也会处理） |
| `target_component` | uint8 | 一般填 **0** |
| `seq` | uint8 | 自增序号 |
| `type` | uint8 | **0**=仅查询；**1**=清除 RID 配置/状态后查询 |

### 4.1 不要搞混「源地址」和「目标地址」

日志里常见：

```
即将发送: MSG_EFT_RID_CONFIG_REQUEST(#517) → sys=255:190
```

这里的 `255:190` 通常是**地面站自己的源 sys/comp**，不是发给飞控的 target。  
真正发给飞控的是 **payload** 里的 `target_system`（应为 `1` 或 `0`）。

---

## 5. 应答：EFT_RID_CONFIG_STATUS（518）

载荷长度 **127** 字节。字符串为定长 char 数组，解析时取到第一个 `\0`；MAVLink v2 可能裁剪尾部零字节，不要写死“一定收到满长度”。

### 5.1 路由

| 字段 | 类型 | 含义 |
|------|------|------|
| `target_system` | uint8 | 请求方 GCS 的 sysid |
| `target_component` | uint8 | 请求方 GCS 的 compid |
| `seq` | uint8 | 与请求相同 |

### 5.2 DID 参数

| 字段 | 类型 | 飞控参数 | 说明 |
|------|------|----------|------|
| `did_enable` | uint8 | `DID_ENABLE` | 0/1 |
| `did_mavport` | int8 | `DID_MAVPORT` | RID 串口号；-1=未用 |
| `did_options` | uint8 | `DID_OPTIONS` | 见下表 |
| `did_can_driver` | uint8 | `DID_CANDRIVER` | 0=未用 |

**`did_options`**

| Bit | 含义 |
|-----|------|
| 0 | EnforceArming：强制 RID 预解锁检查 |
| 1 | AllowNonGPSPosition |
| 2 | LockUASIDOnFirstBasicIDRx：锁定 UAS ID |

### 5.3 身份与声明

| 字段 | 类型 | 说明 |
|------|------|------|
| `ua_type` | uint8 | `MAV_ODID_UA_TYPE` |
| `id_type` | uint8 | `MAV_ODID_ID_TYPE`；**0 = 未配置 Basic ID** |
| `uas_id` | char[20] | 实名登记号 / 机体 ID |
| `op_id_type` | uint8 | `MAV_ODID_OPERATOR_ID_TYPE` |
| `operator_id` | char[20] | 运营人登记号 |
| `desc_type` | uint8 | `MAV_ODID_DESC_TYPE` |
| `self_desc` | char[23] | 自我声明 |

### 5.4 操作员位置

| 字段 | 类型 | 单位 | 显示 |
|------|------|------|------|
| `operator_latitude` | int32 | degE7 | `/ 1e7` |
| `operator_longitude` | int32 | degE7 | `/ 1e7` |
| `operator_altitude_geo` | float | m | 大地高 |

### 5.5 RID 模块状态

| 字段 | 类型 | 说明 |
|------|------|------|
| `arm_status` | uint8 | `MAV_ODID_ARM_STATUS`：0=允许解锁，1=不允许 |
| `arm_error` | char[32] | 不允许时的原因（自 RID 模块，可能截断） |
| `arm_status_age_ms` | uint16 | 距上次 ARM_STATUS（12918）毫秒数；**65535=从未收到** |
| `system_age_ms` | uint16 | 距上次 SYSTEM/SYSTEM_UPDATE；**65535=从未收到** |

### 5.6 `status_flags`（优先读这个）

| Bit | 名称 | =1 表示 |
|-----|------|---------|
| 0 | DID_ENABLED | `DID_ENABLE=1` |
| 1 | BASIC_ID_SET | Basic ID 已配置 |
| 2 | OPERATOR_ID_SET | Operator ID 非空 |
| 3 | OPERATOR_LOC_SET | 操作员经/纬至少一个非 0 |
| 4 | SELF_ID_SET | Self ID 非空 |
| 5 | ARM_STATUS_FRESH | 3 秒内收到 ARM_STATUS |
| 6 | SYSTEM_FRESH | 3 秒内收到 SYSTEM / SYSTEM_UPDATE |
| 7 | TX_ONLINE | 5 秒内收到 ARM_STATUS（模块在线） |
| 8 | ARM_GOOD | `arm_status == GOOD_TO_ARM` |
| 9 | PREARM_PASS | 飞控 RID 预解锁检查当前可通过 |
| 10 | UAS_ID_LOCKED | UAS ID 已从持久化区锁定加载 |
| 11~31 | 预留 | |

UI 建议：

```
先显示 status_flags 红绿灯
需要细节时再展开 uas_id / operator_id / arm_error
```

常见「挡解锁」：

```
bit5=0  → RID ARM_STATUS 超时/未收到
bit8=0  → RID 模块判定不可解锁（看 arm_error）
bit9=0  → 飞控预解锁不通过
```

---

## 6. 推荐联调流程

1. 连接飞控，收到 HEARTBEAT（记下 `sysid`，一般是 1）  
2. （可选）用 `OPEN_DRONE_ID_*` 下发 UAS ID / Operator ID / 操作员位置  
3. 查询：发 517，`type=0`，`seq=N`；清除：发 517，`type=1`，`seq=N`  
4. 等待 518，校验 `seq==N`  
5. 展示 `uas_id`、`operator_id`、`status_flags`（清除后 ID 应为空，DID 为默认值）

写入后立刻查询时，建议间隔 ≥ 200 ms（飞控限流）。

---

## 7. 代码示例

### 7.1 Python（pymavlink，须用重生后的 dialect）

```python
import time
from pymavlink import mavutil

master = mavutil.mavlink_connection('COM10', baud=57600, dialect='ardupilotmega')
# 若贵司只生成 common：dialect='common'
master.wait_heartbeat()

def query_rid_config(seq_no: int, req_type: int = 0, timeout: float = 3.0):
    # req_type: 0=查询, 1=清除后查询
    master.mav.eft_rid_config_request_send(
        master.target_system,  # target_system：飞控 sysid；也可用 0
        0,                     # target_component
        seq_no,
        req_type,
    )
    t_end = time.time() + timeout
    while time.time() < t_end:
        msg = master.recv_match(type='EFT_RID_CONFIG_STATUS', blocking=True, timeout=1.0)
        if msg is not None and msg.seq == seq_no:
            return msg
    return None

def cstr(raw):
    if isinstance(raw, bytes):
        return raw.split(b'\x00')[0].decode('utf-8', errors='replace')
    return str(raw).split('\x00')[0]

st = query_rid_config(1, 0)          # 查询
# st = query_rid_config(2, 1)        # 清除后再回状态
if st is None:
    print('超时：检查 CRC_EXTRA 是否为 209，以及是否已重生绑定')
else:
    print('UAS ID:', cstr(st.uas_id))
    print('Operator ID:', cstr(st.operator_id))
    print('DID:', st.did_enable, st.did_mavport, st.did_options, st.did_can_driver)
    print('status_flags: 0x%08X' % st.status_flags)
    print('arm_status:', st.arm_status, 'arm_error:', cstr(st.arm_error))
```

### 7.2 C# 集成要点

1. 从本仓库 `eft.xml`（经 `common.xml` include）重新生成消息类  
2. 确认 `EFT_RID_CONFIG_REQUEST`：**LEN=4，CRC=209**，含 `type` 字段  
3. 发送后按 msgid=518 收包，匹配 `seq`  
4. 刷新周期 ≥ 1s；清除用 `type=1`  
5. 写入仍走 `OPEN_DRONE_ID_*`，写完再发 `type=0` 核对

---

## 8. 无应答排查表

| 现象 | 原因 | 处理 |
|------|------|------|
| 发出 517，完全无 518 | **CRC_EXTRA ≠ 209**（最常见；旧版 213 已废弃） | 按仓库 eft.xml 重生绑定并重编 GCS |
| 同上 | 飞控未刷含本功能的固件 | 确认固件含 `handle_rid_config_request` |
| 同上 | `target_system` 既非 0 也非飞控 sysid | 填心跳中的 system id 或 0 |
| 偶发无应答 | 200 ms 限流 / TX buffer 满 | 降频重试 |
| 日志写 `sys=255:190` | 多为 GCS 源地址 | 看 payload 的 target_system |
| 用 `MAV_CMD_REQUEST_MESSAGE` 要 12900/517 | **不被支持** | 改发本消息 517 |
| 能收到 518 但 ID 全空 | 尚未成功写入 OpenDroneID | 先发 BASIC_ID / OPERATOR_ID 再查 |
| bit5/bit7=0 | RID 模块未连或未发 ARM_STATUS | 查 RID 硬件与 12918 |
| bit6=0 | 无操作员位置 | 下发 SYSTEM / SYSTEM_UPDATE |

---

## 9. 与写入类 OpenDroneID 消息的关系

| msgid | 消息 | 方向 | 用途 |
|------:|------|------|------|
| 517 | `EFT_RID_CONFIG_REQUEST` | GCS→FC | **查询**（本文） |
| 518 | `EFT_RID_CONFIG_STATUS` | FC→GCS | **查询应答**（本文） |
| 12900 | `OPEN_DRONE_ID_BASIC_ID` | GCS→FC | 写入 UAS ID |
| 12905 | `OPEN_DRONE_ID_OPERATOR_ID` | GCS→FC | 写入 Operator ID |
| 12903 | `OPEN_DRONE_ID_SELF_ID` | GCS→FC | 写入自我声明 |
| 12904 | `OPEN_DRONE_ID_SYSTEM` | GCS→FC | 写入操作员位置等 |
| 12918 | `OPEN_DRONE_ID_ARM_STATUS` | RID模块→FC | 解锁状态（GCS 一般不收） |

---

## 10. 修订记录

| 日期 | 说明 |
|------|------|
| 2026-07-13 | 初版 |
| 2026-07-14 | 补充 CRC、common.xml include、抓包自检；明确不可用 REQUEST_MESSAGE |
| 2026-07-14 | 517 增加 `type`（0 查询 / 1 清除）；LEN=4，CRC_EXTRA=**209**；清除后 DID 默认 ENABLE=1/MAVPORT=2/OPTIONS=0/CANDRIVER=0 |
