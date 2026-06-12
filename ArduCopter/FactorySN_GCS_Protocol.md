# 工厂序列号（FactorySN）地面站对接协议

> 适用版本：ArduCopter（本仓库定制版）
> 飞控侧实现：`ArduCopter/FactorySN.{h,cpp}`、`ArduCopter/GCS_MAVLink_Copter.cpp`
> 文档版本：v1.0（2026-06）

---

## 1. 概述

飞控在参数表中注册了 4 组工厂序列号，对应原产品铭牌的 4 个字段：

| 铭牌字段 | 含义 | 参数前缀 |
| --- | --- | --- |
| 产品型号（去横杠） | product_model | `SN_PROD` |
| 出厂编号 | factory_sn | `SN_FACT` |
| 机身编号 | frame_sn | `SN_FRM` |
| 飞控 SN | fc_sn | `SN_FC` |

每组用 7 个 `int32` 参数存储，每个参数装 3 个 ASCII 字节，单组最大容量 21 个字符（满足 ≤20 字符需求）。

**写入语义为"严格一次性写入"**：一旦该组任意一段在 EEPROM 中被写为非 0 值，**下次上电之后整组就只读了**，地面站再发 `PARAM_SET` 会被飞控拒绝。

---

## 2. 参数清单

总共 **28 个参数**，全部 `MAV_PARAM_TYPE_INT32`，默认值 `0`。

| 字段 | 参数名 |
| --- | --- |
| product_model | `SN_PROD1`、`SN_PROD2`、`SN_PROD3`、`SN_PROD4`、`SN_PROD5`、`SN_PROD6`、`SN_PROD7` |
| factory_sn | `SN_FACT1`、`SN_FACT2`、`SN_FACT3`、`SN_FACT4`、`SN_FACT5`、`SN_FACT6`、`SN_FACT7` |
| frame_sn | `SN_FRM1`、`SN_FRM2`、`SN_FRM3`、`SN_FRM4`、`SN_FRM5`、`SN_FRM6`、`SN_FRM7` |
| fc_sn | `SN_FC1`、`SN_FC2`、`SN_FC3`、`SN_FC4`、`SN_FC5`、`SN_FC6`、`SN_FC7` |

参数名大小写不敏感（飞控统一按大写匹配）。

---

## 3. 字符编解码规则

每个 `SN_xxxN` 是一个 `int32`，按"大端"次序装 3 个 ASCII 字节：

```text
int32_value = (byte0 << 16) | (byte1 << 8) | byte2
```

| 位 | 含义 |
| --- | --- |
| `byte0` (bits 23..16) | 该段的第 1 个字符 |
| `byte1` (bits 15..8) | 该段的第 2 个字符 |
| `byte2` (bits 7..0) | 该段的第 3 个字符 |

### 约束

- **必须保证 `int32_value <= 0x7F7F7F`（8,355,711）**，否则数值在 MAVLink float 编码时会丢精度。换句话说：每个字节都必须是 ASCII（高 bit 为 0），最大 `0x7E (~)`。
- 不足 3 字符的尾段，用 `0x00` 填充；解码时遇到 `0x00` 立即终止。
- 任意 `SN_xxxN == 0` 表示该段未使用。
- 整组全 0 表示该 SN 尚未编程，飞控**不会锁定**该组。

### 示例

`product_model = "EFT0X610PMES026F0001"`（20 字符）的编码：

| 参数 | 3 字符 | hex | int32 (十进制) |
| --- | --- | --- | --- |
| `SN_PROD1` | `EFT` | `0x45 46 54` | `4548180` |
| `SN_PROD2` | `0X6` | `0x30 58 36` | `3168310` |
| `SN_PROD3` | `10P` | `0x31 30 50` | `3223632` |
| `SN_PROD4` | `MES` | `0x4D 45 53` | `5063507` |
| `SN_PROD5` | `026` | `0x30 32 36` | `3158582` |
| `SN_PROD6` | `F00` | `0x46 30 30` | `4599856` |
| `SN_PROD7` | `01\0` | `0x30 31 00` | `3158272` |

---

## 4. 代码参考实现

### 4.1 Python

```python
def encode_sn(ascii_str: str) -> list[int]:
    """ASCII 串 -> 7 个 int32；超过 21 字符截断，不足补 \\x00。"""
    data = ascii_str.encode("ascii")[:21].ljust(21, b"\x00")
    return [
        (data[i*3] << 16) | (data[i*3+1] << 8) | data[i*3+2]
        for i in range(7)
    ]

def decode_sn(chunks: list[int]) -> str:
    """7 个 int32 -> ASCII 串；遇到 0x00 截断。"""
    out = bytearray()
    for v in chunks:
        for shift in (16, 8, 0):
            b = (v >> shift) & 0xFF
            if b == 0:
                return out.decode("ascii", errors="replace")
            out.append(b)
    return out.decode("ascii", errors="replace")

assert decode_sn(encode_sn("EFT0X610PMES026F0001")) == "EFT0X610PMES026F0001"
```

### 4.2 JavaScript / TypeScript

```javascript
function encodeSN(asciiStr) {
  const bytes = new Uint8Array(21);
  const enc = new TextEncoder().encode(asciiStr).slice(0, 21);
  bytes.set(enc);
  const chunks = [];
  for (let i = 0; i < 7; i++) {
    chunks.push(
      (bytes[i*3] << 16) | (bytes[i*3+1] << 8) | bytes[i*3+2]
    );
  }
  return chunks;
}

function decodeSN(chunks) {
  const out = [];
  for (const v of chunks) {
    for (const shift of [16, 8, 0]) {
      const b = (v >> shift) & 0xFF;
      if (b === 0) {
        return new TextDecoder().decode(new Uint8Array(out));
      }
      out.push(b);
    }
  }
  return new TextDecoder().decode(new Uint8Array(out));
}
```

### 4.3 C / C++

```c
#include <stdint.h>
#include <string.h>

void encode_sn(const char *ascii, int32_t chunks_out[7]) {
    uint8_t buf[21] = {0};
    size_t n = strnlen(ascii, 21);
    memcpy(buf, ascii, n);
    for (int i = 0; i < 7; i++) {
        chunks_out[i] = ((int32_t)buf[i*3]   << 16)
                      | ((int32_t)buf[i*3+1] <<  8)
                      |  (int32_t)buf[i*3+2];
    }
}

size_t decode_sn(const int32_t chunks[7], char *ascii_out, size_t out_size) {
    size_t pos = 0;
    for (int i = 0; i < 7; i++) {
        uint32_t v = (uint32_t)chunks[i];
        uint8_t bytes[3] = {
            (uint8_t)((v >> 16) & 0xFF),
            (uint8_t)((v >>  8) & 0xFF),
            (uint8_t)( v        & 0xFF),
        };
        for (int b = 0; b < 3; b++) {
            if (bytes[b] == 0 || pos + 1 >= out_size) {
                ascii_out[pos] = '\0';
                return pos;
            }
            ascii_out[pos++] = (char)bytes[b];
        }
    }
    ascii_out[pos] = '\0';
    return pos;
}
```

---

## 5. MAVLink 消息时序

### 5.1 写入（GCS → 飞控）

对每个 `SN_xxxN` 发一条 `PARAM_SET`（msg id `23`）：

| 字段 | 类型 | 值 |
| --- | --- | --- |
| `target_system` | `uint8_t` | 飞控 SYSID（默认 `1`） |
| `target_component` | `uint8_t` | 飞控 COMPID（默认 `1` = `MAV_COMP_ID_AUTOPILOT1`） |
| `param_id` | `char[16]` | 如 `"SN_PROD1"`，剩余字节补 `0x00` |
| `param_value` | `float` | `(float)int32_value`，**直接强转**（不要 bit-cast） |
| `param_type` | `uint8_t` | `6` = `MAV_PARAM_TYPE_INT32` |

> 提示：因为我们把每段的 int 值限制在 `<= 0x7F7F7F`，远小于 `2^24`，float 强转可无损往返。

### 5.2 写入确认

飞控收到 `PARAM_SET` 后**总是**广播一条 `PARAM_VALUE`（msg id `22`）：

```text
PARAM_VALUE.param_id    = "SN_PROD1"
PARAM_VALUE.param_value = 当前飞控内存中的真实值（float）
PARAM_VALUE.param_type  = MAV_PARAM_TYPE_INT32 (6)
PARAM_VALUE.param_count = <总参数数>
PARAM_VALUE.param_index = -1
```

**GCS 必须校验回包的 `param_value` 等于刚才写入的值。**

| 回包 `param_value` 与发送值对比 | 含义 |
| --- | --- |
| 相等 | 写入成功 |
| 不相等（且 GCS 会同时收到 `STATUSTEXT "Factory SN locked (...)"`） | 写入被拒，该组已被锁定 |

### 5.3 写入被拒（STATUSTEXT 警告）

锁定状态下，每次 `PARAM_SET` 会触发一条 `STATUSTEXT`（msg id `253`）：

```text
STATUSTEXT.severity = 4  (MAV_SEVERITY_WARNING)
STATUSTEXT.text     = "Factory SN locked (SN_PROD1)"
```

括号里是被尝试改写的参数名。建议 GCS UI 同时监听这条警告并展示给用户。

### 5.4 读取

两种方式任选其一：

**A. 单个读取** — `PARAM_REQUEST_READ`（msg id `20`）

```text
target_system     = 1
target_component  = 1
param_id          = "SN_PROD1"
param_index       = -1   # 用 param_id 查询
```

飞控以 `PARAM_VALUE` 回复。

**B. 批量读取** — `PARAM_REQUEST_LIST`（msg id `21`）

```text
target_system     = 1
target_component  = 1
```

飞控会把全部参数（包括 28 个 `SN_*`）逐条 `PARAM_VALUE` 流式回送。

> 读取**不受锁定影响**，任何时候都能读。

### 5.5 启动横幅（可选监听）

每次飞控上电会通过 `STATUSTEXT`（severity = `6` `MAV_SEVERITY_INFO`）发 4 条消息：

```text
ProductModel: <decoded ASCII>   或   ProductModel: <unset>
FactorySN:    <decoded ASCII>   或   FactorySN: <unset>
FrameSN:      <decoded ASCII>   或   FrameSN: <unset>
FC_SN:        <decoded ASCII>   或   FC_SN: <unset>
```

GCS 可以解析这些 STATUSTEXT 快速展示 SN，也可以忽略并自己读参数解码。

---

## 6. 锁定语义详解

### 6.1 触发条件

> **某组（如 `SN_PROD*`）的任意一段在 EEPROM 中存有非 0 值 → 下次上电该组锁定，任何 `PARAM_SET` 被拒绝。**

锁定状态在飞控启动初期 `init_ardupilot()` 中、通过 `FactorySN::snapshot_lock_state()` 一次性快照得出，运行期不再变化。

### 6.2 单次会话内允许多次写入

同一次开机内允许重复写、写完才重启的设计是有意为之，目的是让工厂烧录工具能一次性写完 7 段。流程：

```text
[首次上电]
    ↓
snapshot: 所有组未锁定（EEPROM 全 0）
    ↓
GCS PARAM_SET SN_PROD1..7 → 全部成功
    ↓
GCS 发送 MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN (246, param1=1)
    ↓
[飞控重启]
    ↓
snapshot: SN_PROD 组锁定（EEPROM 非 0）
    ↓
GCS PARAM_SET SN_PROD1 → 被拒，回 STATUSTEXT "Factory SN locked"
```

### 6.3 解锁

**当前实现中没有解锁通道**，严格 write-once。若工厂返修流程需要"重置后重写"，需要重新刷固件擦除 EEPROM。

如需在线解锁机制（例如通过自定义 `MAV_CMD_USER_x` + 厂家密钥才允许清零），请联系飞控固件维护者增加。

---

## 7. 烧录工具推荐流程（GCS 工具侧）

```text
1. 连接飞控，建立 MAVLink 链路
2. PARAM_REQUEST_READ SN_PROD1..7（或 PARAM_REQUEST_LIST）
   → 校验：当前是否全 0？否则提示"该组已锁定，请先确认是否需要返修流程"
3. 用户输入 ASCII SN（≤ 20 字符，只允许 ASCII 可打印字符 0x20-0x7E）
4. encode_sn(ascii) → 7 个 int32
5. 对每段发 PARAM_SET，等待 PARAM_VALUE 回包并校验数值一致
6. 若有任一段校验失败 → 标红 + 中止
7. 全部段写入成功 → 发 MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN (param1=1, param2=0)
8. 飞控重启后重新连接
9. 再次 PARAM_REQUEST_READ + decode_sn() → 与用户输入比对
10. 比对一致 → 烧录成功；不一致 → 报错（基本不会发生）
11. （可选）再发一条 PARAM_SET 测试锁定是否生效，应收到 "Factory SN locked"
```

---

## 8. 常见问答

**Q1. 为什么每段只用 3 字节而不是 4 字节？**
A. MAVLink 的 `PARAM_VALUE` 用 `float` 编码数值，IEEE 754 float 只能精确表示 ≤ 2²⁴ 的整数。3 字节 ASCII 最大 `0x7F7F7F (8,355,711) < 2²⁴ (16,777,216)`，可无损往返；4 字节就会丢精度。

**Q2. 同一会话内能反复写同一段吗？**
A. 能。锁定快照只在启动时拍一次。

**Q3. 我能不能只写 product_model 这一组，其他组之后再写？**
A. 可以。4 个组的锁定状态相互独立。某组保持全 0 就一直可写。

**Q4. 重启会清空已写入的 SN 吗？**
A. 不会，SN 与其它 ArduPilot 参数一样保存在 EEPROM（实际是 STM32H7 的参数页 Page 14），重启后保留。

**Q5. 是否会被 `FORMAT VERSION` 升级清掉？**
A. 不会，AP_Param 在格式升级时会保留同名参数。

**Q6. ASCII 中是否允许小写字母 / 空格 / 符号？**
A. 允许 ASCII 范围 `0x20`（空格）~ `0x7E`（`~`）。但 `0x00` 在数据中表示"该段结束"，**不能出现在中间**。建议工厂工具限制为 `[0-9A-Z]`，避免误用。

**Q7. 锁定状态如何在 GCS UI 暴露？**
A. 推荐 GCS 在显示 SN 区域同时展示一个"锁定"图标——逻辑很简单：读到的某组任意一段 ≠ 0 就显示锁定（与飞控的判定一致）。

---

## 9. 错误处理建议（GCS 侧）

| 场景 | 飞控反馈 | GCS 处理建议 |
| --- | --- | --- |
| `PARAM_SET` 后未收到 `PARAM_VALUE` | 无 | 超时（建议 1s）后重发，最多 3 次 |
| 收到 `PARAM_VALUE` 但值不等于发送值 | 同时收到 `STATUSTEXT "Factory SN locked"` | 立即停止整组写入，UI 提示"该组已被锁定" |
| `PARAM_REQUEST_READ` 后查不到 | 无 | 检查参数名拼写、飞控版本（确认是定制固件） |
| ASCII 串含非 `0x20-0x7E` 字符 | N/A（飞控只检查数值） | GCS 写入前校验并拒绝 |
| ASCII 串 > 20 字符 | N/A | GCS 写入前校验并拒绝 |
| 用户尝试输入 `0x7F` (DEL) | N/A | GCS 建议禁止，因为虽不违反精度约束，但属不可打印字符 |

---

## 10. 参考飞控侧源码位置

| 文件 | 内容 |
| --- | --- |
| `ArduCopter/FactorySN.h` | 类声明、常量（`NUM_CHUNKS = 7`、`BYTES_PER_CHUNK = 3`） |
| `ArduCopter/FactorySN.cpp` | `var_info[]`（28 项）、`snapshot_lock_state()`、`is_param_locked()`、`send_banner()` |
| `ArduCopter/Parameters.h` | 在 `ParametersG2` 中新增 `FactorySN factory_sn;` |
| `ArduCopter/Parameters.cpp` | `var_info2[]` 中新增 `AP_SUBGROUPINFO(factory_sn, "SN_", 14, ParametersG2, FactorySN)` |
| `ArduCopter/system.cpp` | `init_ardupilot()` 启动时 `snapshot_lock_state()` + `send_banner()` |
| `ArduCopter/GCS_MAVLink_Copter.cpp` | `handle_message()` 中拦截 `MAVLINK_MSG_ID_PARAM_SET`，锁定时回弹当前值并发 STATUSTEXT |

---

## 附录 A：测试用例对照表

可作为 GCS 工具自动化测试参考。

| 编号 | 操作 | 期望结果 |
| --- | --- | --- |
| TC-01 | 首次上电后读 `SN_PROD1..7` | 全部 = 0 |
| TC-02 | 收到启动 banner | 4 条 `<unset>` |
| TC-03 | PARAM_SET `SN_PROD1` = 4548180 | 收到 PARAM_VALUE，值 = 4548180 |
| TC-04 | 同会话内再次 PARAM_SET `SN_PROD1` = 9999 | 写入成功，PARAM_VALUE = 9999 |
| TC-05 | 写完 7 段，重启 | 启动 banner 显示已解码 ASCII |
| TC-06 | 重启后 PARAM_SET `SN_PROD1` | 收到 PARAM_VALUE 显示旧值 + STATUSTEXT "Factory SN locked (SN_PROD1)" |
| TC-07 | 重启后 PARAM_SET `SN_FACT1` | 写入成功（未锁定组互不影响） |
| TC-08 | 写入 int32 = 0x7F7F7F | 写入并读回均为 8355711 |
| TC-09 | 写入 int32 = 0x80000000 | 在 MAVLink float 编码时被截断，回包不一致；GCS 应拒绝该输入 |
