# 本仓库特有的约束

写给在这个 fork 上工作的人和 AI agent。这里**只记录本 fork 与上游 ArduPilot 不同、且踩过坑的地方**。上游通用的 AI 协作规范（代码风格、提交信息、PR 流程等）在 `master` 分支的同名文件里，需要时去那边查。

---

## 1. 编译

```bash
./waf configure --board EFT_CAAC
./waf copter
```

只能在**仓库根目录**跑，`build/` 是产物目录，里面没有 `waf`。

如果报"权限不够"，说明你在的分支上 `waf` 的执行位是丢的——它自 `b008e59941`（2026-05-09 加 EFT_CAAC hwdef 那次）起被记成 `100644`，上游一直是 `100755`。`dev-algo` 已由 `45a30e73ad` 修复，`ardupilot-ubuntu` 尚未。临时绕过用 `python3 ./waf`，根治是 `chmod +x waf` 并提交。

Flash 余量很紧，目前只剩约 **16.8 KB**（1687107 / 1703923 已用）。加代码前先看一眼 BUILD SUMMARY 的 `Free Flash`。

## 2. 改 MAVLink 消息定义是两步操作

**这是本仓库最容易踩的坑，改错了不报任何错。**

上游 ArduPilot 不跟踪 `libraries/GCS_MAVLink/include/`，那是 mavgen 的构建产物目录。本仓库把 448 个生成的头文件**入库**在那里，而 C 语言引号形式的 `#include "include/mavlink/v2.0/..."` 优先搜索当前文件所在目录，所以：

> 编译器读的永远是**源码树里入库的那份**，mavgen 每次写进 `build/` 的那份**没有任何 TU 会 include**。

后果是：只改 `modules/mavlink/message_definitions/` 下的 XML 然后编译，**改动会被静默丢弃**，固件里根本没有新消息。2026-08-05 加进 `eft.xml` 的 msgid 519/520/521（`UOM_ARM_STATUS` / `UOM_FC_STATUS` / `UOM_OPERATOR_ID`）就是这样漏了三周，直到 `7921e9dbb9` 才补进固件。

正确流程：

```bash
# 1. 改完 XML，重新生成
./waf clean && ./waf copter

# 2. 把产物同步回源码树
cp -r build/EFT_CAAC/libraries/GCS_MAVLink/include/mavlink/v2.0/. \
      libraries/GCS_MAVLink/include/mavlink/v2.0/

# 3. 必须再 clean 一次，见下
./waf clean && ./waf copter

# 4. 校验，必须输出 0
diff -rq build/EFT_CAAC/libraries/GCS_MAVLink/include/mavlink/v2.0 \
         libraries/GCS_MAVLink/include/mavlink/v2.0 | wc -l
```

**第 3 步的 clean 不能省。** 增量编译不会重新编译依赖这些入库头文件的 TU——实测同步完头文件直接 `./waf copter`，产出的固件与同步前**逐字节相同**，会让人误以为改动生效了。只有 clean 之后才真正吃进去。

同步会顺带改掉所有 dialect 的 `MAVLINK_BUILD_DATE` 和 `MAVLINK_*_XML_HASH`，几十个文件的 diff 是重新生成的固有噪声，不是异常。

## 3. 不要删除 xyfyf/pymavlink 仓库

定制的 pymavlink 改动只存在于 <https://github.com/xyfyf/pymavlink> 分支 `custom-messages`：

| commit | 内容 |
| --- | --- |
| `f130827f` | 按通道支持 EF 包头发送与解析 |
| `723581df` | msgid 516 帧格式覆盖的魔数和 CRC 字段 |

历史上 `xyfyf/mavlink` 的 `.gitmodules` 把 pymavlink 指向官方 `ArduPilot/pymavlink.git`。那样也能正常工作，是因为 GitHub 的 fork network 共享对象存储，官方 URL 可以取到 fork 里的 sha——**但那是隐式依赖**。

一旦 `xyfyf/pymavlink` 被删除，对象可能被 GC，所有人的克隆会同时断链，而且报错完全看不出根因：`Tools/ardupilotwaf/git_submodule.py:69` 对状态为 `+` 的子模块跑 `git merge-base`，遇到不存在的对象 exit 128，异常穿透成 `WafError: Build failed`。

**这一条已在 `xyfyf/mavlink` 的 `83de5360`（2026-08-20）修掉**，`.gitmodules` 改成显式指向 fork：

```
[submodule "pymavlink"]
	path = pymavlink
	url = https://github.com/xyfyf/pymavlink.git
	branch = custom-messages
```

注意**修复要生效，本仓库记录的 `modules/mavlink` gitlink 必须 ≥ `83de5360`**。在此之前 gitlink 停在 `606cc585`，正好差这一个提交，于是修复躺在远程分支上而克隆出来的仍是旧版——加子模块引用时容易漏掉这半步。

另：`.gitmodules` 用的是绝对 HTTPS 地址，而本机 `modules/mavlink/pymavlink` 的 origin 可能是 SSH。两者指向同一仓库，不影响使用；`git submodule sync` 会把本地 remote 覆盖成 HTTPS，如果你依赖 SSH key 推送就别跑它。

## 4. 帧格式覆盖默认是开启的

`MAV_TX_MAGIC` 默认值 0 = 板级 EF 掩码模式，`EFT_CAAC/hwdef.dat` 规定 **USB(SERIAL0) 和 LINK 遥测(SERIAL6/UART8) 出厂就发 `0xEF` 包头**，不是标准 MAVLink2 的 `0xFD`。

所以任何用**官方** pymavlink 的工具（MAVProxy、`sim_vehicle.py`、autotest、日志解析脚本）都解不出飞控的遥测。仓库里 44 处 `import pymavlink`，`Tools/autotest/sim_vehicle.py:43` 把 `modules/mavlink` 加进 `sys.path`，用的就是子模块里那份定制版——别把它换成 pip 装的版本。

调试时可以看 `mav_tx_override_hits` 计数器确认覆盖逻辑是否真的生效。

## 5. 子模块

`git submodule update --init --recursive` 可以正常跑通（`8bf9f2e035` 清掉了 `mcp-servers/` 下三个没有 `.gitmodules` 条目的 gitlink，那是 `00c9d6c88b` 的误提交，之前会让这条命令直接 fatal）。

**注意 `ardupilot-ubuntu` 上尚未修复**，那三个 gitlink 还在，在该分支上这条命令依然会 fatal，需要加 `-- modules` 限定路径绕过。

如果以后再遇到"某子模块路径在 .gitmodules 中未找到 url"，检查是不是又有内嵌 git 仓库被 `git add .` 扫进索引了。

另：`mcp-servers/ros-mcp/` 不是子模块，是整份第三方源码树入库（207 个文件、91 MB），与固件编译无关，`waf` 不碰它。

## 6. 日志配置：Replay 与控制分析是两条独立的路

### 估计器可以离线在环重放，控制器不行

`Tools/Replay` 能把日志里录下的**原始传感器输入**按原时序喂回 EKF3 离线重跑，用同一份真实飞行数据验证估计器改动。这条路对控制器**不成立**：日志记的是"当时那套控制律"作用下的轨迹，换了控制律飞机响应就变了，后续传感器读数不再成立。估计器能重放是因为它对飞行只读——同样输入进去，输出不改变输入本身。

编译 Replay 需要切到 SITL 工具链，会覆盖当前板子配置：

```bash
./waf configure --board sitl && ./waf replay     # 产出 build/sitl/tool/Replay
./waf configure --board EFT_CAAC                 # 记得切回来
```

Replay 输出的日志带 `.EFT` 后缀（本仓库改过日志命名），pymavlink 的 `mavutil.mavlink_connection()` 按扩展名判断格式会认不出来、返回一堆 `BAD_DATA`。改用 `DFReader.DFReader_binary(path)` 绕过，或把文件改名成 `.BIN`。

### 要跑 Replay 必须先开的参数

```
LOG_REPLAY   = 1      # 重启生效
LOG_DISARMED = 1      # 必须，否则缺解锁前数据，Replay 起不来
LOG_FILE_BUFSIZE      # 官方建议调大，防丢帧
```

判断一份日志能不能跑 Replay，**不要 grep 文件里有没有 `RFRH` 字符串**——日志开头的 FMT 表会声明固件认识的全部消息类型，每份日志都能匹配到。要按实际记录条数查：

```python
from pymavlink import DFReader
m = DFReader.DFReader_binary(path)
# 统计 RFRH/RFRF/REV2/RSO2/RWA2/RISH 的真实条数，全为 0 就是没开
```

**不需要为了 Replay 去开 `IMU_FAST`。** 常规 `IMU` 消息记的是瞬时加速度/角速度，而 EKF 消费的是 `delta_velocity`/`delta_angle` 积分增量加每帧 `dt`（见 `AP_DAL/LogStructure.h` 的 `RISI`）。无论把 `IMU` 频率调多高都重建不出 EKF 的输入。`LOG_REPLAY` 在 DAL 边界截取的数据流天然 1:1，节拍来自 `AP_NavEKF3.cpp` 的 `dal.start_frame()`，即主循环频率——`EFT_CAAC/defaults.parm` 设的是 `SCHED_LOOP_RATE 400`，所以是 400 Hz。开 `IMU_FAST` 只会增加冗余数据和 SD 卡压力。

### 控制分析需要的是另一个位

`LOG_REPLAY` 完全不影响控制相关日志。`ATT`/`RATE`/`CTUN`/`PID*`/`RCOU`/`MOTB` 的频率由 `LOG_BITMASK` 对应位和各自的调度任务决定（`ten_hz_logging_loop` 10 Hz、`twentyfive_hz_logging` 25 Hz、`fast_loop` 400 Hz）。

当前默认 `LOG_BITMASK = 145374` 开了 `ATTITUDE_MED`(bit1) 但**没开 `ATTITUDE_FAST`**(bit0)，所以 `ATT`/`RATE` 只有 10 Hz。姿态环跑在 400 Hz、带宽通常 10–30 Hz，10 Hz 记录的奈奎斯特只到 5 Hz，**高频振荡和超调全部混叠掉了**。做控制律优化要加上 bit0，同时建议补上 bit5（`NTUN`，位置控制器内部量 `PSCN/PSCE/PSCD`）：

```
LOG_BITMASK  = 145407     # = 145374 + 1(bit0) + 32(bit5)
LOG_REPLAY   = 1
LOG_DISARMED = 1
```

### 一个架次就能兼顾，不需要分开飞

早期版本的本节写过"两套配置日志都很大，建议分架次飞"，以及"2026-08-20 那批日志有一架次 `ATT`/`RATE` 只有 4.9 Hz，说明 SD 卡在丢帧，开 `ATTITUDE_FAST` 前先换高速卡"。**这两条都已被实测推翻，别再照着做。**

- 4.9 Hz 是**统计口径造成的假象**：`LOG_DISARMED = 0` 时一份文件里有多个架次，架次间的空档被算进了分母。按 `RATE` 自身首末时间戳算，00000231 是 2232 条 / 223.1 s = **10.00 Hz**；三份日志的 `DSF.Dp` 全为 0，**一帧都没丢**。不需要换卡。
- 带宽和缓冲也够：`LOG_BITMASK=145407` + `LOG_REPLAY=1` 约 196 KB/s，按实测线性外推缓冲峰值约占 200 KiB 的 34%。

真正要盯的不是巡航段而是**开档头 2 秒**——FMT 表加千余条 `PARM` dump 的突发，在今天 28 KB/s 的配置下就已经吃掉 39% 的缓冲。

完整的位定义、四种组合对比、余量核算和飞后必查项见 `~/UAV/flight-test/日志记录配置指南.md`。

### 现成的检查与分析工具

```bash
# 判定一份日志能否跑 Replay，通过就直接给出重放命令
python3 Tools/eft_log_analysis/check_replay_ready.py <log.bin>

# 从真实飞行数据算控制器性能指标，多份日志可横向对比
python3 Tools/eft_log_analysis/log_control_metrics.py <log.bin> [...] --csv out.csv
```

细节见 `Tools/eft_log_analysis/README.md`。
