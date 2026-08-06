# MAVLink 包头切换（FD / EF）— 地面站开发说明

面向地面站（GCS）工程师。

飞控默认在 **USB（Type-C）** 和 **LINK 数传** 上发 **`0xEF`** 包头；地面站可通过一条命令切到标准 **`0xFD`**。  
**记忆性已在飞控侧完成**：切到 FD 后会掉电保存，下次上电仍发 FD，地面站**不必每次开机再发一次**。

| 项 | 内容 |
|----|------|
| 适用固件 | EFT 定制 ArduPilot（如 EFT_CAAC / E616 / X6100） |
| 消息定义源文件 | `modules/mavlink/message_definitions/v1.0/eft.xml` |
| 消息 | `MAV_FRAMING_OVERRIDE_CMD`，**msgid = 516** |
| 持久参数 | `MAV_TX_MAGIC`（飞控内部保存，地面站可读） |
| 飞控处理入口 | `GCS_MAVLINK::packetReceived()`（`GCS_Common.cpp`） |

---

## 0. 地面站要不要开发？

| 现状 | 要不要改 | 说明 |
|------|----------|------|
| **已经能发 516 切 FD/EF** | **基本不用改协议** | 记忆由飞控自动做；可选：UI 提示「已保存、重启仍有效」，或读 `MAV_TX_MAGIC` 显示当前模式 |
| **还不会发 516** | **需要开发** | 按本文实现发包 + 双包头收包 |
| **只能解析 FD、不能解析 EF** | **必须开发收包** | 出厂默认是 EF，连不上就切不了 FD |

**结论：**

- 协议侧：仍是发 **516**，**没有新 msgid**，也没有新字段。
- 产品侧：建议补 UI（「切标准 FD / 恢复 EF」）和状态显示；记忆不需要地面站再写 Flash。

---

## 1. 功能说明

| 方向 | 消息 | msgid |
|------|------|------:|
| 地面站 → 飞控 | `MAV_FRAMING_OVERRIDE_CMD` | **516** |
| 飞控 → 地面站 | 无专用应答；会发 `STATUSTEXT` 确认（如 `FrameOverride active: STX=0xFD ... saved`） | 253 |

只影响飞控**发出去**的 MAVLink2 帧头；飞控**接收**仍同时认 FD/EF（EF 口会把收到的 `0xEF` 当 `0xFD` 解析），所以即使用户已经切到 FD，地面站仍可用 FD 再发 516 改回去。

| 出厂默认 | 切 FD 后 | 再断电上电 |
|----------|----------|------------|
| USB/LINK 发 **EF** | 立即改发 **FD**，并写入 `MAV_TX_MAGIC=253` | 仍发 **FD** |
| 其它串口本来就是 FD | 不变 | 不变 |

---

## 2. 消息定义（必须一致）

### 2.1 常量

| 项 | 值 |
|----|---:|
| msgid | **516** |
| 载荷长度 | **4** |
| **CRC_EXTRA** | **253** |

CRC 不对时飞控会**静默丢包**，表现为「点了切换没反应」。

### 2.2 字段（线序：按类型从大到小排）

| 偏移 | 类型 | 字段 | 说明 |
|-----:|------|------|------|
| 0 | `uint16_t` | `crc` | 仅当强制 CRC 时使用；一般业务填 `0` |
| 2 | `uint8_t` | `cmd` | 见下表 |
| 3 | `uint8_t` | `magic` | 仅 `cmd` 的 bit1=1 时有效 |

### 2.3 `cmd` 取值（常用就前两行）

| cmd | 含义 | 是否记忆包头 |
|----:|------|--------------|
| **0** | 强制发 **FD（0xFD）**，CRC 正常计算 | **是** → `MAV_TX_MAGIC=253` |
| 1 | 强制 FD + 强制 CRC（调试用） | 包头是；CRC **不**记 |
| **2** | 使用 `magic` 字段作包头，CRC 正常 | **是** |
| 3 | 使用 `magic` + 强制 CRC（调试用） | 包头是；CRC **不**记 |

`cmd` 位含义：bit0=强制 CRC，bit1=使用 `magic` 字段。

---

## 3. 地面站该发什么（产品接口）

### 3.1 切到标准 FD（对接 Mission Planner / 标准 GCS）

```
cmd   = 0
magic = 0      // 忽略
crc   = 0      // 忽略
```

飞控立即改发 FD，并掉电保存。下次上电仍是 FD，**无需再发**。

### 3.2 恢复 / 切到 EF（仅 USB/LINK；RID 仍为 FD）

任选其一（效果相同：只改 `HAL_MAVLINK_EF_MAGIC_SERIAL_MASK` 口，**不会**把 RID 改成 EF）：

**方式 A**

```
cmd   = 2
magic = 0xEF
crc   = 0
```

**方式 B（恢复板级默认）**

```
cmd   = 2
magic = 0
crc   = 0
```

> 注意：旧固件把 `magic=0xEF` 当成「全通道强制 EF」。新固件起 `0` / `0xEF` 都是 mask 模式。

### 3.3 伪代码

```python
# 切 FD 并记忆
send_mavlink(
    msgid=516,
    payload=struct.pack("<HBB", 0, 0, 0),  # crc, cmd, magic
)

# 恢复 EF 并记忆
send_mavlink(
    msgid=516,
    payload=struct.pack("<HBB", 0, 2, 0xEF),
)
```

若用官方生成函数：

```c
mavlink_msg_mav_framing_override_cmd_send(chan, /*cmd*/0, /*magic*/0, /*crc*/0);     // FD
mavlink_msg_mav_framing_override_cmd_send(chan, /*cmd*/2, /*magic*/0xEF, /*crc*/0); // EF
```

---

## 4. 收包：必须支持双包头

飞控可能发 `0xEF` 或 `0xFD` 开头的 MAVLink2。

| 阶段 | 飞控 TX 包头 | 地面站 RX |
|------|--------------|-----------|
| 出厂 / 已恢复 EF | `0xEF` | 必须认 EF |
| 已切 FD / 记忆为 FD | `0xFD` | 必须认 FD |

建议解析器：

1. 看到 `0xFD` 或 `0xEF` 都当作 MAVLink2 帧起始；
2. 后续长度、seq、CRC 规则与标准 MAVLink2 相同（只是首字节不同）；
3. **不要**在切 FD 后关掉 EF 识别——用户可能再切回 EF。

地面站自己**发出去**的包：用标准 `0xFD` 即可（飞控能收）。

---

## 5. 可选：读参数确认当前记忆

| 参数名 | 含义 |
|--------|------|
| `MAV_TX_MAGIC` | `0` 或 `239(0xEF)` = 板级 EF 掩码（USB/LINK 发 EF，RID 仍 FD）；`253` = 强制全通道 FD；其它非 0 = 强制该字节 |

UI 建议：

- `0` → 显示「出厂 EF」
- `253` → 显示「标准 FD（已记忆）」
- 发 516 后等 `STATUSTEXT` 含 `FrameOverride active` / `saved`，再刷新参数

也可用 `PARAM_SET` 直接写 `MAV_TX_MAGIC`（`253` 或 `0`），一般不如发 516 直观；发 516 会立刻改帧并保存。

---

## 6. 用仓库 eft.xml 生成绑定

与其它 EFT 消息相同：用本仓库

`modules/mavlink/message_definitions/v1.0/eft.xml`

重新生成 MAVLink 绑定（pymavlink / mavgen 等），确认：

- 名称：`MAV_FRAMING_OVERRIDE_CMD`
- msgid：**516**
- len：**4**
- CRC_EXTRA：**253**

字段名/类型/顺序不要改，否则 CRC 会变。

---

## 7. 联调检查清单

- [ ] 出厂上电：抓包可见飞控 TX 以 **`EF`** 开头  
- [ ] 发 `cmd=0`：立刻变成 **`FD`**，并收到 `FrameOverride ... saved`  
- [ ] 读 `MAV_TX_MAGIC` == **253**  
- [ ] 断电再上电：仍是 **`FD`**，且地面站**不用再发 516** 也能连上  
- [ ] 发 `cmd=2, magic=0xEF`：USB/LINK 为 **EF**，RID 仍为 **FD**；`MAV_TX_MAGIC` 为 **239**；再断电仍如此
- [ ] 发 `cmd=2, magic=0`：同上（板级掩码）  
- [ ] CRC_EXTRA 错误时：无 STATUSTEXT、帧头不变（丢包）

---

## 8. 和「记忆性」相关的常见误解

| 误解 | 实际情况 |
|------|----------|
| 地面站要自己写 Flash / 本地记住再每次上电重发 | **不需要**；飞控已存 `MAV_TX_MAGIC` |
| 记忆要新 msgid | **不需要**；仍是 516 |
| 切 FD 后飞控收不到地面站的 FD 包 | **能收到**；接收未改 |
| CRC 强制也会记忆 | **不会**；只有包头 magic 记忆 |



    <message id="516" name="MAV_FRAMING_OVERRIDE_CMD">
      <description>Command to override the outgoing MAVLink2 frame format on ALL channels in real time (takes effect immediately).
      The start-of-frame magic is persisted (MAV_TX_MAGIC) and restored on the next power-up.
      The cmd value is a 2-bit field: bit0 controls the CRC, bit1 controls the start-of-frame magic byte.
      The magic field carries the start-of-frame byte to use, and the crc field carries the 16-bit CRC value to use.
      cmd=0: force magic byte 0xFD (MAVLink2 default) and remember it across reboot. CRC = normal (computed). magic/crc fields ignored.
      cmd=1: force magic byte 0xFD (persisted), CRC = forced to the value in the crc field.
      cmd=2: use the magic field byte (persisted; magic=0 restores board default e.g. 0xEF). CRC = normal (computed).
      cmd=3: use the magic field byte (persisted), CRC = forced to the value in the crc field.
      Only affects transmitted MAVLink2 frames; reception is unchanged so this command can always be re-sent to revert.
      CRC override is runtime-only and is not persisted across reboot.</description>
      <field type="uint8_t" name="cmd">Override command (0-3): bit0=force CRC to the crc field, bit1=use the magic field byte instead of 0xFD.</field>
      <field type="uint8_t" name="magic">Start-of-frame magic byte to use when bit1 of cmd is set (e.g. 0xEF). magic=0 restores board default. Ignored when bit1 is clear.</field>
      <field type="uint16_t" name="crc">CRC value (little-endian) to force into the 2-byte checksum field when bit0 of cmd is set. Ignored when bit0 is clear. Not persisted.</field>
    </message>
