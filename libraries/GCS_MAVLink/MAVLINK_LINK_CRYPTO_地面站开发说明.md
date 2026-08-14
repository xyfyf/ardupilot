# MAVLink 数传链路加密 — 地面站开发说明

面向地面站（GCS）工程师。

飞控可在指定 MAVLink 串口（通常是**数传**）上，把进出的原始字节流包成加密信封，防止电台空口被窃听。  
**这不是新的 MAVLink msgid**，也不改 XML；地面站要在「串口字节 ↔ MAVLink 解析器」之间加一层加解密。

| 项 | 内容 |
|----|------|
| 适用固件 | EFT 定制 ArduPilot（已开 `AP_MAVLINK_LINK_CRYPTO_ENABLED`，如 EFT_CAAC） |
| 飞控实现 | `libraries/GCS_MAVLink/GCS_MAVLink_Crypto.{h,cpp}` |
| 开关参数 | `MAVx_OPTIONS` **bit3**（数值 **8**，Encrypt link），改完需**重启** |
| 算法 | **ChaCha20（IETF / monocypher `crypto_ietf_chacha20`）**，仅保密，不做 AEAD |
| 密钥派生 | `BLAKE2b-256(MASTER \|\| "EFT-LINK-v1" \|\| fc_sn)` → 32 字节 |
| 密钥材料 | 飞控 **飞控SN（SN_FC）** + 厂商 MASTER（与日志加密 MASTER **不是同一把**） |
| 参考脚本 | `Tools/eft_log/decrypt_mavlink_link.py`（纯 Python 标准库） |

---

## 0. 地面站要不要开发？

| 现状 | 要不要改 | 说明 |
|------|----------|------|
| 只连 **USB**，数传口未开加密 | **不用改** | USB 默认保持明文 |
| 要连已开加密的**数传** | **必须开发** | 收：解信封；发：封信封。否则连不上 |
| 已有 EFT 加密日志解析 | **可复用部分** | KDF/ChaCha20 同族，但 **盐值、MASTER、帧格式都不同**，不能直接套日志代码 |

**结论：** 数传加密开启后，地面站必须在该物理口做**双向**加解密；MAVLink 业务层（HEARTBEAT、参数、任务等）不变。

---

## 1. 功能说明（先建立正确心智模型）

```
地面站业务 ←→ MAVLink 编解码 ←→【加解密层】←→ 串口/数传 ←→ 飞控【加解密层】←→ MAVLink
```

| 方向 | 线上看到的 | 地面站要做的 |
|------|------------|--------------|
| 飞控 → 地面 | `0xA5` 开头的信封 | 拆信封 → ChaCha20 解密 → 得到 `0xFD`/`0xEF` 明文 → 喂给现有解析器 |
| 地面 → 飞控 | 同样要发 `0xA5` 信封 | 把本来要写出的 MAVLink 字节先加密再写串口 |
| USB（未开 bit3） | 仍是普通 MAVLink | 不要加密 |

> **重要：** MAVLink 库发一帧时常会 **分 3 次 write**（帧头 / payload / CRC）。  
> 飞控对**每一次 write** 各包一层信封，所以线上常见长度模式：`10 + 9 + 2` 或 `10 + 8 + 2`。  
> 地面站解密后把多段明文**按顺序拼成字节流**再解析即可；不要假设「一个 A5 = 一条完整 MAVLink」。

---

## 2. 信封格式（线序）

每个信封：

```
偏移  长度  字段
0     1     MAGIC = 0xA5
1     2     LEN   = 密文长度，uint16 小端
3     12    NONCE
15    LEN   CIPHERTEXT（与明文等长）
```

| 常量 | 值 |
|------|---:|
| MAGIC | `0xA5` |
| NONCE 长度 | 12 |
| 固定开销 | 15 字节（1+2+12） |
| 单段明文最大 | ≤ 280（`MAVLINK_MAX_PACKET_LEN`） |

### 2.1 NONCE 布局（12 字节）

| 字节 | 含义 |
|-----:|------|
| 0..6 | 开机随机前缀（本端固定 7 字节，重启会变） |
| 7 | MAVLink channel 号（飞控填自己的 chan；地面站可填 0，飞控不校验此字节） |
| 8..11 | 本端单调计数器，**大端** uint32，每发一信封 +1 |

**(key, nonce) 绝不能重复。** 地面站必须自备随机前缀 + 独立计数器，不要抄飞控抓包里的 nonce。

### 2.2 抓包样例（已实测）

明文阶段（加密未生效或未开该口）：

```
FD 09 00 00 ...          ← 标准 MAVLink2
```

加密生效后：

```
A5 0A 00 1C F4 98 F8 5D 6B 6A 04 00 00 00 00 4C 06 1C BD 06 A2 72 EC F8 B1
│  │     │                          │  │              └ 密文 10 字节
│  │     │                          │  └ ctr = 0
│  │     │                          └ chan = 4
│  │     └ nonce 前缀 7 字节
│  └ LEN = 0x000A
└ MAGIC
```

用飞控 SN `TAA010126FCYF001` 解密后可还原 HEARTBEAT、`OP_LOC` 等。

---

## 3. 密钥怎么算（必须与飞控一致）

### 3.1 飞控 SN（`fc_sn`）

与工厂 SN / 加密日志相同来源：参数 `SN_FC1` … `SN_FC7` 拼成 ASCII 字符串。

- UI「飞控SN」字段，例如：`TAA010126FCYF001`
- 若未写入 SN，飞控侧退化为字符串 `NOSN`（地面站联调时勿用错）

地面站连上后：用已有工厂 SN 读取协议拿飞控 SN，或让飞手手动填入。

### 3.2 MASTER（32 字节，当前开发默认）

与日志加密的 `EFT_LOG_MASTER_KEY` **相互独立**。当前固件默认：

```
ef 54 4c 49 4e 4b 4b 45 59 21 10 11 12 13 14 15
16 17 18 19 1a 1b 1c 1d 1e 1f 20 21 22 23 24 25
```

（ASCII 前缀为 `EFTLINKKEY!` + 填充。）

> 量产前固件会更换 MASTER；地面站需可配置（环境变量或本地安全配置），与飞控同步。  
> 参考脚本可用环境变量：`MAVLINK_LINK_MASTER_KEY`（64 个 hex 字符）。

### 3.3 KDF

```
key[32] = BLAKE2b-256(
            MASTER (32 bytes)
         || "EFT-LINK-v1"          // 不含结尾 '\0'
         || fc_sn 的 ASCII 字节
         )
```

注意盐是 **`EFT-LINK-v1`**，不是日志的 `EFT-LOG-v1`。

### 3.4 加解密

对每个信封：

```
明文 = ChaCha20_IETF(key, nonce12, 密文)
密文 = ChaCha20_IETF(key, nonce12, 明文)
```

- 与 monocypher `crypto_ietf_chacha20()` 一致（内部 CTR 从 **0** 起，针对**本信封**）
- 流密码，加解密同一函数
- **不是** Poly1305 AEAD；完整性仍靠 MAVLink 自身 CRC（及可选 signing）

Python 参考实现：`Tools/eft_log/decrypt_mavlink_link.py`（含 `--self-check`）。

---

## 4. 飞控侧如何开关（方便联调）

### 4.1 参数

| 参数 | 含义 |
|------|------|
| `MAVx_OPTIONS` | bit3 = 8 → 该 MAV 通道加密 |

`MAVx` **不是** `SERIALx` 同号，而是：所有 `SERIALn_PROTOCOL=2` 的口按 n 从小到大排序后的序号（MAV1 起）。

### 4.2 EFT_CAAC 默认口序（`defaults.parm`）

| 顺序 | 物理口 | 常见用途 | 参数名 |
|-----:|--------|----------|--------|
| 1 | SERIAL0 | USB | MAV1 |
| 2 | SERIAL1 | GPS1 接头 | MAV2 |
| 3 | SERIAL2 | RSV2 | MAV3 |
| 4 | SERIAL4 | 主数传（注释 primary telem） | **MAV4** |
| 5 | SERIAL6 | LINK | MAV5 |

联调建议：

```
MAV4_OPTIONS = 8      # 或实际数传对应的 MAVx
MAV1_OPTIONS = 0      # USB 保持明文，方便救砖/调试
```

改完**重启飞控**。抓包应从 `FD` 变成大量 `A5`。

### 4.3 编译开关

板级 hwdef 需：

```
define AP_MAVLINK_LINK_CRYPTO_ENABLED 1
```

未定义时整段加密代码不编译，`MAVx_OPTIONS` bit3 无效。

---

## 5. 地面站实现要点

### 5.1 接收路径（必做）

1. 从串口读原始字节。
2. 状态机找 `0xA5` → 读 LEN → 读 12 字节 nonce → 读 LEN 字节密文。
3. 用派生 key + nonce 解密，得到明文字节。
4. 明文喂给现有 MAVLink 解析器（仍可能是 `0xFD` 或 `0xEF` 包头）。
5. LEN 非法（0 或过大）或中途超时：丢弃，重新找下一个 `0xA5`。

### 5.2 发送路径（必做）

1. 业务层照常生成 MAVLink 字节（可整帧，也可像飞控一样分片 write）。
2. 对每一段要写出的缓冲区：生成 nonce（本端前缀 + chan + 计数器++）→ ChaCha20 加密 → 前面加 `A5 | LEN_le | NONCE`。
3. 把信封写入串口。
4. **禁止**在已开加密的口上直接写裸 `FD`/`EF` 帧，飞控收不到。

### 5.3 连接哪条口、要不要加密

| 口 | 建议 |
|----|------|
| USB | 明文；地面站走老逻辑 |
| 数传（已设 OPTIONS=8） | 必须走加解密层 |
| 同一地面站多链路 | 按链路分别开关，不要全局一刀切 |

### 5.4 推荐产品形态

- 连接向导：选择「加密数传」→ 填写/读取飞控 SN → 自动派生密钥。
- 抓包诊断：可显示「当前为 A5 信封 / 明文 FD」。
- 与「包头 FD/EF 切换（msgid 516）」正交：先解密得到明文，再按原有 FD/EF 规则解析。

---

## 6. 自检清单（对接验收）

| # | 能力 | 通过标准 |
|---|------|----------|
| 1 | 识别信封 | 线上出现 `A5`，能按 LEN 完整收齐 |
| 2 | 拿到正确 fc_sn | 与工厂「飞控SN」一致（如 `TAA010126FCYF001`） |
| 3 | KDF | `BLAKE2b-256(MASTER \|\| "EFT-LINK-v1" \|\| sn)`，MASTER 与固件一致 |
| 4 | 解密 | 解出后出现 `0xFD`/`0xEF`，能解出 HEARTBEAT |
| 5 | 加密回传 | 飞控能应答参数/心跳（证明 RX 路径也通） |
| 6 | 分片 | 不要求一个信封=一帧；字节流拼起来能解析 |
| 7 | USB 兜底 | 未开加密的 USB 仍可明文连接 |

**只做了收包解密、没做发包加密 → 半通，地面站指令飞控收不到。**

---

## 7. 向 AI / 同事提问（复制即用）

```
请阅读 libraries/GCS_MAVLink/MAVLINK_LINK_CRYPTO_地面站开发说明.md
和 Tools/eft_log/decrypt_mavlink_link.py，
判断当前地面站是否支持 EFT MAVLink 数传链路加密；
缺什么请列出改造步骤（必须含双向加解密）。
```

本地快速验证抓包：

```bash
python3 Tools/eft_log/decrypt_mavlink_link.py -i capture.bin --fc-sn TAA010126FCYF001 --print
python3 Tools/eft_log/decrypt_mavlink_link.py --self-check
```

---

## 8. 与「加密飞行日志」对照（避免混用）

| 项 | 数传链路加密（本文） | 飞行日志加密 |
|----|----------------------|--------------|
| 场景 | 实时串口/电台 | SD 卡 `.EFT` 文件 |
| MAGIC | 每段 `0xA5` | 文件头 `EFTL` |
| 盐 | `EFT-LINK-v1` | `EFT-LOG-v1` |
| MASTER | `AP_MAVLINK_LINK_CRYPTO_MASTER_KEY` | `AP_LOGGER_EFT_MASTER_KEY` |
| Nonce | 每段 12 字节，在线携带 | 文件头一次 12 字节，整文件 CTR |
| 参考脚本 | `decrypt_mavlink_link.py` | `decrypt_eft.py` |

---

## 9. 常见问题

**Q: 开了加密后 Mission Planner 直接连数传失败？**  
A: 正常。MP 不认识 `A5` 信封。需地面站自研加解密层，或中间加解密网关再转给 MP。

**Q: 为什么解密后很多段不以 FD 开头？**  
A: 一帧常拆成头/载荷/CRC 三段，只有第一段以 FD 开头；三段都解密成功，拼起来才是完整帧。

**Q: chan 字节地面站填多少？**  
A: 飞控解密不校验 chan；建议填 `0`，并保证本端 nonce 前缀随机、计数器递增即可。

**Q: 和 msgid 516 包头切换冲突吗？**  
A: 不冲突。516 作用在明文 MAVLink 层；链路加密在更外层。
