# 工厂序列号（FactorySN）地面站对接协议

> 适用版本：ArduCopter（本仓库定制版）
> 飞控侧实现：`ArduCopter/FactorySN.{h,cpp}`、`ArduCopter/GCS_MAVLink_Copter.cpp`
> 文档版本：v1.1（2026-06，**锁定语义改为严格 per-chunk write-once**）

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

**严格 per-chunk write-once 语义**：**每个 `SN_xxxN` 一旦当前值非 0，立刻锁定**，无需重启。任何想把它改成不同值的 `PARAM_SET` 都会被拒绝并回弹原值。**写错就只能刷固件清 EEPROM**——所以 GCS 工厂工具必须先做严格校验后再下发。
- 允许"同值重写"通过（GCS 周期性 sync 不会触发警告）
- 不允许改成不同值，**包括从非 0 改成 0**（清零也算修改）

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
| 不相等（且 GCS 会同时收到 `STATUSTEXT "Factory SN locked (...)"`） | 写入被拒，该段已锁定（当前值非 0） |

> ⚠️ 注意 GCS UI 弹的"Saved"对话框**不能**作为成功依据，必须看回包。我们已经看到过 GCS UI 显示"Saved"但实际飞控拒绝的情况。

### 5.3 写入被拒（STATUSTEXT 警告）

当某个 SN_xxxN 当前值非 0、地面站又试图改成**不同**的值时，飞控会发一条 `STATUSTEXT`（msg id `253`）：

```text
STATUSTEXT.severity = 4  (MAV_SEVERITY_WARNING)
STATUSTEXT.text     = "Factory SN locked (SN_PROD1)"
```

括号里是被尝试改写的参数名。建议 GCS UI 同时监听这条警告并展示给用户。

**例外**：如果新值与当前值相等（同值重写），飞控会让它通过，不会发警告。这是为了让 GCS 周期性 sync 不会刷屏。

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

> **每个 `SN_xxxN` 参数独立锁定**：当前值 ≠ 0 立刻锁，下一条想改它的 PARAM_SET 直接被拒。**与是否重启无关**。

锁定检查在每次 `PARAM_SET` 到来时实时跑（`FactorySN::is_param_locked()`），读的就是参数在 RAM 中的当前值（与 EEPROM 同步）。所以：
- 第一次写：当前值 = 0 → 允许
- 第二次想改它：当前值 ≠ 0 → 拒绝

### 6.2 同值重写允许通过

为了避免地面站周期性 PARAM_SET sync 不停触发警告，**新值与当前值完全相等的写入会被放行**：

```text
SN_PROD1 当前值 = 4539988
GCS PARAM_SET SN_PROD1 = 4539988 → 允许（同值，无操作）
GCS PARAM_SET SN_PROD1 = 4540000 → 拒绝（不同值，已锁定）
GCS PARAM_SET SN_PROD1 = 0       → 拒绝（不同值，已锁定 ← 清零也是修改）
```

### 6.3 工厂烧录流程下的影响

**每段必须一次写对**，不能写错后再改正。所以：

```text
[全 0 飞控上电]
    ↓
GCS 收用户输入 → 严格校验 ASCII (0x20-0x7E)、长度 ≤ 20
    ↓
GCS encode → 7 个 int32
    ↓
GCS PARAM_SET 第 1 段 → 飞控接受、写 EEPROM、立即锁
    ↓
GCS PARAM_SET 第 2 段 → 接受
    ↓
... 直到第 7 段
    ↓
GCS 主动重新读 7 段、解码、与原 ASCII 完全比对
    ↓
任何一段不一致 → 烧录失败、设备报废（需擦 EEPROM 重来）
```

特别提醒：**段与段之间不能 reboot**，因为一旦写错某段，重启后它仍然是错的，并且仍然锁定，永远改不回。

### 6.4 同一组的部分段允许追加吗？

允许。**锁是 per-chunk 的，不是 per-group。** 比如你只写了 `SN_PROD1..5`，那么 `SN_PROD6`、`SN_PROD7` 当前值还是 0 → 仍然可以写。
但是注意一致性：如果你 ASCII 编码是按 21 字符对齐的，第二次写入"剩余段"时，前面已经写的段必须保持原值不变（同值重写会被允许）。

### 6.5 解锁

**当前实现中没有解锁通道**，严格 write-once。若工厂返修流程需要"重置后重写"，需要重新刷固件擦除 EEPROM 参数区。

如需在线解锁机制（例如通过自定义 `MAV_CMD_USER_x` + 厂家密钥才允许清零），请联系飞控固件维护者增加。

---

## 7. 烧录工具推荐流程（GCS 工具侧）

```text
1. 连接飞控，建立 MAVLink 链路
2. PARAM_REQUEST_READ SN_PROD1..7（或 PARAM_REQUEST_LIST）
   → 校验：要写的那一组 7 段是否全 0？
     - 全 0 → 可以正常烧录
     - 部分非 0 → 已部分锁定，提示用户"该组已有写入，请确认是否继续/返修"
3. 用户输入 ASCII SN
   → 严格校验：长度 ≤ 20，每个字符 ∈ [0x20, 0x7E]（推荐再收紧到 [0-9A-Z]）
   → 一旦确认下发，没有反悔机会
4. encode_sn(ascii) → 7 个 int32
5. 对每段发 PARAM_SET：
   a. 等待对应 PARAM_VALUE 回包（超时 1s 重试最多 3 次）
   b. 校验 PARAM_VALUE.param_value == 发送值
   c. 同时监听 STATUSTEXT，若有 "Factory SN locked (...)" → 立即终止流程
   d. 任一段失败 → 标红 + 整个流程中止；剩余段不要继续发
6. 全部段成功 → 不需要 reboot，再次 PARAM_REQUEST_READ + decode_sn() → 与用户输入完全比对
7. 比对一致 → 烧录成功
   比对不一致 → 罕见，理论上 step 5b 已经拦截；如果发生，标记设备需返修
8. （强烈推荐）再发一条 PARAM_SET 把 SN_PROD1 改成不同值做"防护自检"，必须收到
   STATUSTEXT "Factory SN locked (SN_PROD1)"，验证锁工作正常
```

> ⚠️ 与 v1.0 文档不同点：v1.0 流程要求烧录后 reboot 才生效锁；v1.1 起**锁立即生效**，烧录后无需 reboot。但**段与段之间也不能 reboot**——因为重启不会清除已写入的锁定。

---

## 8. 常见问答

**Q1. 为什么每段只用 3 字节而不是 4 字节？**
A. MAVLink 的 `PARAM_VALUE` 用 `float` 编码数值，IEEE 754 float 只能精确表示 ≤ 2²⁴ 的整数。3 字节 ASCII 最大 `0x7F7F7F (8,355,711) < 2²⁴ (16,777,216)`，可无损往返；4 字节就会丢精度。

**Q2. 能反复写同一段吗？**
A. 只能写"相同的值"。当前值是 0 时随便写；一旦写入非 0，此后只接受**完全相同**的值（用于支持周期 sync），不接受任何不同值（包括把它清零）。

**Q3. 我能不能只写 product_model 这一组，其他组之后再写？**
A. 可以。28 个段彼此完全独立，没写过（值 = 0）的段都可以正常写。

**Q4. 同一组内的某些段没写完，能不能后续再补？**
A. 可以。`SN_PROD1..5` 写过、`SN_PROD6..7` 还是 0 时，可以单独写 6、7。但前 5 段后续就别想再改了。

**Q5. 重启会清空已写入的 SN 吗？**
A. 不会，SN 与其它 ArduPilot 参数一样保存在 EEPROM（实际是 STM32H7 的参数页 Page 14），重启后保留，锁也保留（因为锁是看运行时值是否非 0）。

**Q6. 是否会被 `FORMAT VERSION` 升级清掉？**
A. 不会，AP_Param 在格式升级时会保留同名参数。

**Q7. ASCII 中是否允许小写字母 / 空格 / 符号？**
A. 允许 ASCII 范围 `0x20`（空格）~ `0x7E`（`~`）。但 `0x00` 在数据中表示"该段结束"，**不能出现在中间**。建议工厂工具限制为 `[0-9A-Z]`，避免误用。

**Q8. 锁定状态如何在 GCS UI 暴露？**
A. 推荐 GCS 在显示 SN 区域同时展示一个"锁定"图标——逻辑很简单：读到的某段 ≠ 0 就显示锁定（与飞控的判定一致）。

**Q9. 已经写错了，能不能擦掉重写？**
A. 不能通过 MAVLink 擦。必须把固件刷新（或用 STM32 编程器把整片 EEPROM 区域擦干净）。这是 write-once 的代价。

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
| `ArduCopter/FactorySN.cpp` | `var_info[]`（28 项）、`is_param_locked()`（实时查当前 AP_Int32 值）、`send_banner()` |
| `ArduCopter/Parameters.h` | 在 `ParametersG2` 中新增 `FactorySN factory_sn;` |
| `ArduCopter/Parameters.cpp` | `var_info2[]` 中新增 `AP_SUBGROUPINFO(factory_sn, "SN_", 14, ParametersG2, FactorySN)` |
| `ArduCopter/system.cpp` | `init_ardupilot()` 启动时调用 `send_banner()` |
| `ArduCopter/GCS_MAVLink_Copter.cpp` | `handle_message()` 中拦截 `MAVLINK_MSG_ID_PARAM_SET`，已锁定且新值≠旧值时回弹当前值并发 STATUSTEXT |

---

## 附录 A：测试用例对照表

可作为 GCS 工具自动化测试参考。

| 编号 | 操作 | 期望结果 |
| --- | --- | --- |
| TC-01 | 首次上电后读 `SN_PROD1..7` | 全部 = 0 |
| TC-02 | 收到启动 banner | 4 条 `<unset>` |
| TC-03 | PARAM_SET `SN_PROD1` = 4539988 | 收到 PARAM_VALUE，值 = 4539988 |
| TC-04 | **不重启**，再 PARAM_SET `SN_PROD1` = 9999 | **拒绝**：PARAM_VALUE 回 4539988 + STATUSTEXT "Factory SN locked (SN_PROD1)" |
| TC-05 | **不重启**，再 PARAM_SET `SN_PROD1` = 0（尝试清零） | **拒绝**：PARAM_VALUE 回 4539988 + STATUSTEXT |
| TC-06 | **不重启**，再 PARAM_SET `SN_PROD1` = 4539988（同值重写） | 允许：PARAM_VALUE 回 4539988，**无** STATUSTEXT 警告 |
| TC-07 | 同会话写 `SN_PROD2` = 3168310（首次写） | 写入成功 |
| TC-08 | 写完 7 段，重启 | 启动 banner 显示已解码 ASCII |
| TC-09 | 重启后 PARAM_SET `SN_PROD1` = 任何不同值 | 拒绝 |
| TC-10 | 重启后 PARAM_SET `SN_FACT1` = 任何值（首次写） | 写入成功（不同组段独立） |
| TC-11 | 写入 int32 = 8355711（= 0x7F7F7F） | 写入并读回均为 8355711 |
| TC-12 | 写入 int32 = 0x80000000 | 在 MAVLink float 编码时被截断，回包不一致；GCS 应拒绝该输入 |
