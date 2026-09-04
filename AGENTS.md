# 本仓库特有的约束

写给在这个 fork 上工作的人和 AI agent。这里**只记录本 fork 与上游 ArduPilot 不同、且踩过坑的地方**。上游通用的 AI 协作规范（代码风格、提交信息、PR 流程等）在 `master` 分支的同名文件里，需要时去那边查。

---

## 1. 编译

```bash
./waf configure --board EFT_CAAC
./waf copter
```

只能在**仓库根目录**跑，`build/` 是产物目录，里面没有 `waf`。

如果报"权限不够"，是**整批文件的执行位在 `b008e59941`（2026-05-09 加 EFT_CAAC hwdef 那次）被误提交掉了**。那次动了 654 个文件，其中 **591 个是纯模式变更 `100755 → 100644`**（blob 前后完全相同，只掉权限，实质改动只有约 63 个），像是从不保留 POSIX 权限的路径导入代码时整批带进来的。

受影响的不只是 `waf`：`Tools/autotest/sim_vehicle.py`、`Tools/autotest/run_in_terminal_window.sh`、`hwdef/scripts/*.py`、各种 `build-*.sh` 全在内。**症状是新克隆下来、切到本分支后直接跑 `sim_vehicle.py` 就报 `PermissionError: run_in_terminal_window.sh`**——而这是 ArduPilot 官方文档的标准入口。

| 分支 | 状态 |
| --- | --- |
| `dev-algo` | **已全量修复**（601 个文件，纯模式提交） |
| `ardupilot-ubuntu` | **尚未修复**，仍会踩 |

在未修复的分支上临时绕过：`python3 ./waf`、`python3 Tools/autotest/sim_vehicle.py`，或直接调 `build/sitl/bin/arducopter`。根治办法是按 `master` 的模式整批恢复：

```bash
git ls-tree -r master | awk '$1=="100755"{print $4}' | while read -r f; do
  git cat-file -e "HEAD:$f" 2>/dev/null && git update-index --chmod=+x -- "$f"
done
```

提交前用 `git diff --cached --stat` 确认是 `0 insertions(+), 0 deletions(-)`——只改模式、不碰内容。

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

---

## 7. 领域划分与分支

长期规划是按领域并行推进算法开发，不同领域由不同人负责，问题按领域指派。本节给出领域的**文件归属依据**——直接照搬上游的库目录划分，不另起一套。

### 7.0 三个领域，不是四个

```
规控（规划 + 控制）   P01 P02 P04 P05 P06 P07     一个岗位
数据融合              P03
系统辨识              机体参数标定（供给上面两个）
```

**规划与控制合为一个岗位。** 七个软件问题里六个落在规控，而且它们之间的耦合全在规控内部：

- **P05 电子围栏**要同时改 `AC_Avoid`/`AC_PosControl`（限速器）与 `AC_Fence`（几何）；
- **P07 圆周抽动**要同时改 `AC_ArcNav`（入弧、曲率）与姿态环；
- **P04 的自动迫降**——降级后 `AP_Motors` 知道少了一台电机、偏航已放弃、可行域缩到三维，但 `AC_WPNav`、`AC_PosControl`、`mode_auto` **完全不知情**（`grep _failed_motor` 在这三处为空）。规划会继续下达飞机已经做不到的转弯。目前靠「告警 + 人工接管」兜住，但声明 §3.1 写的能力是「受控应急着陆」，真要做自动迫降，规划必须知道可行域缩了。

**这三条都是接口问题，而接口两边都不归谁管正是分家的代价。** 合成一个岗位就没有这个接口。

**数据融合与规控之间的接口反而是干净的**：EKF 给状态，规控用状态。P03 与其余六个问题几乎不碰同一个文件。

**系统辨识是供给方，不是平级领域**：

```
系统辨识 ──┬─→ 控制   质量、推力曲线、MOT_THST_EXPO、VFF 增益、惰走 τ
           └─→ 规划   最大制动加速度 g·tan(ANGLE_MAX)、可达偏航速率、转弯半径下限
```

`AC_Avoid` 中 `accel_cap = GRAVITY_MSS * tanf(veh_angle_max_rad)` 即为一例：规划的制动剖面，参数来自控制侧的能力，而那个能力值得靠辨识给准。**排路线图时辨识应先行，不是并列。**

### 7.1 领域 ↔ 库目录 ↔ 问题

上游 151 个库按职责分组如下。前三个领域与本项目的划分一一对应。

| 领域 | 主要库目录 | 本项目问题 |
| :-- | :-- | :-- |
| **导航 / 数据融合** | `AP_NavEKF2/3`、`AP_AHRS`、`AP_InertialNav`、`AP_GPS`、`AP_Compass`、`AP_Baro`、`AP_InertialSensor`、`AP_OpticalFlow`、`AP_RangeFinder`、`AP_VisualOdom`、`AP_Beacon` | P03 |
| **制导 / 规划** | `AC_WPNav`（含 `AC_ArcNav`）、`AC_Avoidance`、`AC_Fence`、`AP_Mission`、`AP_Follow`、`AC_PrecLand`、`AP_SmartRTL`、`AP_Rally`、`AP_Terrain` | P06、P07、P05 |
| **控制** | `AC_AttitudeControl`（含 `AC_PosControl`）、`AC_PID`、`AP_Motors`、`AC_AutoTune`、`AC_CustomControl`、`AP_Quicktune` | P01、P02、P04、P05、P07 |
| **系统辨识** | 上游无对应领域；本项目落在 `Tools/`（`fit_vel_ff.py` 等）与机体参数标定 | 质量、推力曲线、惰走 τ、转速上报下界、VFF 增益 |
| 传感器与驱动 | `AP_RCProtocol`、`SRV_Channel`、`AP_BLHeli`、`AP_FETtecOneWire`、`AP_KDECAN`、`AP_ESC_Telem`、`AP_CANManager`、`AP_DroneCAN` | （P04 依赖 ESC 遥测） |
| 安全 / 失效保护 | `AP_Arming`、`AP_AdvancedFailsafe`、`AP_Parachute`、`AC_Fence`、`AP_BattMonitor` | P04、P05 |
| 仿真与验证 | `SITL`、`AP_HAL_SITL`、`AP_Logger`、`AP_DAL`（Replay） | **共享层，见 7.4** |
| 系统 / 平台 | `AP_HAL_*`、`AP_Scheduler`、`AP_Param`、`StorageManager` | — |
| 通信 / 地面站 | `GCS_MAVLink`、`AP_MSP`、`AP_OSD`、`AP_Networking`、`AP_DDS` | — |
| 载荷 | `AC_Sprayer`、`AP_Camera`、`AP_Mount`、`AP_Gripper` | 尚未纳入问题册 |

**三点值得注意：**

1. **上游没有独立的「系统辨识」领域。** 上游做通用固件，机体参数由用户自行整定，辨识散在 `AC_AutoTune` / `Quicktune`（在线）与 `Tools/`（离线）。本项目做特定机型，辨识因此成为独立领域，**而且是其余领域的供给方**——质量与推力曲线是 P04 余量核算的输入，惰走 τ 与转速上报下界是 P04 检测器标定的输入，VFF 增益是 P02 的输入。**排路线图时它应当先行，不是并列。**
2. **「安全 / 失效保护」横跨控制与规划**，上游也不把它单列成领域，而是散在各处。这解释了为什么 P04 与 P05 在按领域划分时都难归位。
3. **`AC_Sprayer` 上游已有。** 后续做喷洒相关算法（流量-地速匹配、断喷续喷）时，那是第十个领域，目前尚未进问题册。

### 7.2 问题指派：按主要改动落在哪

| 问题 | 主要改动 | 指派领域 |
| :-- | :-- | :-- |
| P01 着陆末段砸地 | `AC_PosControl` 垂向、落地检测、`crash_check` | 控制 |
| P02 高速反拉抽动 | `AC_AttitudeControl_Multi`、`ATC_VFF_*` | 控制（VFF 增益由辨识供给） |
| P04 动力失效容错 | `AP_Motors`、`AP_ESC_Telem` | 控制 |
| **P05 电子围栏** | `AC_Avoid` + `AC_PosControl`（限速器为主） | **控制**（越界改 `AC_Fence`） |
| P06 平滑航迹 | `AC_WPNav`、`AC_ArcNav`、`mode_auto` | 规划 |
| **P07 圆周抽动** | `AC_ArcNav`（入弧与曲率为主） | **规划**（越界改姿态环） |
| P03 罗盘与 IMU 对齐 | `AP_Compass`、EKF3 / GSF | 数据融合 |
| 机体参数标定 | `Tools/` | 系统辨识 |
| P08 动力不一致 | 无（基线自述「软件侧无解」） | 转硬件整改 |

### 7.3 文件归属是**评审责任**，不是**修改权限**

P05 归控制，但必须改 `AC_Fence`；P07 归规划，但要动姿态环。**如果按文件划死修改权限，这两个问题根本没法做**——围栏修复要限速器与几何同时在场才能验证，拆到两条分支上哪条都跑不出正确结果。

```
谁负责这个问题   → 在自己分支上完成整改，包含越界的那部分
越界动了谁的文件 → 合并前请该领域的人看一眼
```

### 7.4 共享层不进领域分支

```
Tools/ 、libraries/SITL/ 、libraries/AP_HAL_SITL/ 、docs/ 、AGENTS.md
        → 一律直接提交 dev-algo
```

这条最容易破，后果也最大。统计近 80 个提交，**真正被多个问题编号动过的文件只有 `Tools/eft_issue_repro/reproduce.py` 与 `regression.py`**——四个领域各自在自己分支上改这两个文件，冲突是必然的。

> 推论：**系统辨识那条分支可能基本是空的。** 它的交付物是数据与参数，落在 `Tools/`，而 `Tools/` 属共享层 → 直接进 `dev-algo`。它很少动飞控代码。

### 7.5 同步节奏是硬性的

```
每天    从 dev-algo 单向同步一次
每周    至少合回 dev-algo 一次
```

长期分支唯一的杀手是分叉。2026-09-04 一天之内就反向合了三次 `dev-algo → dev-p04`，而那还只是两条分支、一个领域。四条分支不定节奏，一个月后就合不动了。

### 7.6 试飞固件只能来自 `dev-algo`

```
领域分支    只用于开发，从不刷机
dev-algo    集成主干，试飞固件从这里打 tag（v3.0.xx）
```

日志里的 `FIRMWARE_VERSION` 必须能对上一个 tag（`v3.0.33` ↔ 日志 `00000219`，`v3.0.34` ↔ `00000211`）。**从领域分支刷出去的固件无法追溯**——那个状态包含哪些领域的什么版本，事后说不清。

### 7.8 分支命名

```
wip-<问题编号>-<在做什么>

wip-P04-land-timeout        告警后兜底超时
wip-P04-telem-loss          遥测失联当失效信号
wip-P06-arc-avoid-terrain   圆弧接入避障与地形约束
infra-sweep-tool            不属于任何问题时用 infra- / docs-
```

**两半都要有：**

- **问题编号**——`git ls-remote --heads origin` 一眼看出谁在做哪个问题，且与提交信息的 `P04:`、`regression.py` 的 `pid="P04"` 三处一致；
- **在做什么**——界定范围。`dev-p04` 之所以变成筐，是因为名字**只有**编号，任何 P04 沾边的东西都能往里塞（最终装进了 P06 的三点定圆与两条电池提交）。

> **前缀为什么不用 `pr-`**：`pr` = pull request，是上游 ArduPilot 的习惯（子模块里可见 `pr-canfd-lencheck`、`pr-cxx-wrappers`）。但本仓库近 200 个提交里 `Merge pull request` **零次**，合并全是本地 `git merge`——用这个前缀会暗示一个不存在的流程。`wip-` 准确描述它是什么：在做中、合并后即删。

> **防止变成筐的不是名字，是 7.5 的三天寿命规则。** 名字只界定意图，寿命才是强制力。

### 7.7 一个终端一个目录

**不同终端绝不共用同一个目录。** 2026-09-04 出过事故：一方未提交的 `AP_Motors` 改动被另一方 `git add` 卷进了一个描述完全不同的提交（`22191ad308`），历史里没有它的来由；分支也在一方准备提交时被切走。

多机协作时**分支建了立刻推**——只在本地存在的分支等于不存在。同日另一台机器建了 `dev-control` 与 `dev-planning` 却未推送，从服务器上完全看不见。

```bash
git push -u origin <branch>        # 建了就推，分支名是唯一的协调信号
git ls-remote --heads origin       # 开工前先看谁在做什么
```

推送顺序固定：**先子模块，后父仓库指针**。反过来会让别人拉到一个指向尚不存在的 commit 的指针。

## 8. 提交信息与问题挂靠

追溯靠标识号，不靠分支。分支是临时的工作容器，标识号是永久的追溯轴——审查方问的是"P04 涉及哪些改动、哪些测试、哪些证据"，不会问"P04 在哪条分支上"。

### 格式

```
P05: 一句话说清改了什么

正文：为什么这么改，依据是什么，实测数字。

R-01, R-08
```

- **主题行必须以问题编号开头**，多个用逗号：`P05,P06: …`
- **末行单列关闭的评审发现编号**（若有）。写在正文里 `git log --grep` 也能捞到，但单列一行便于机器提取。
- 不属于任何问题的改动用三个前缀之一——**这不是可选项**，16 条评审发现里就有 3 条不属于任何 P：

```
infra:   Tools/、libraries/SITL/、AP_HAL_SITL/  共享层
docs:    文档
chore:   权限位、子模块指针一类
```

### 查得到才算挂上了

```bash
git log --grep="^P05"      # 一个问题的全部改动，跨分支
git log --grep="R-08"      # 一条评审发现的落地情况
```

### 评审发现（R-01…R-16）的建议归属

出自 `docs/dev-algo-review-2026-08-25.md`。**待确认，确认后请在该文档中为每条补一列「挂靠」。**

| R | 内容摘要 | 挂靠 |
| :-- | :-- | :-- |
| R-01 | 姿态模式围栏松杆无法主动刹车 | P05 |
| R-02 | 分配矩阵奇异仍返回成功，可能输出全零 | P04 |
| R-03 | `MOT_FAIL_YAW` 与重分配语义不一致 | P04 |
| R-04 | 重分配饱和未反馈 PID 抗积分 | P04 |
| R-05 | 圆弧临时限幅缺退出清理 | P07 |
| R-06 | 入弧条件不足 | P07 |
| R-07 | AUTO 前视与圆弧可行性非原子 | P06, P07 |
| R-08 | 圆弧直驱位控，绕过围栏/避障/地形 | **P05, P06** |
| R-09 | GUIDED 圆弧回传陈旧目标 | P07 |
| R-10 | 偏航权限代码为空，任务语义被隐式改写 | P06 |
| R-11 | 集中回归吞错误、复用旧结果假通过 | **infra** |
| R-12 | 速度前馈辨识 lane 与在线可观测性 | P02 |
| R-13 | 多电机低转速告警没节流 | P04 |
| R-14 | 手工失效参数是持久化飞行陷阱 | P04 |
| R-15 | SITL 地效把机体气动阻力一起放大 | **infra**（影响 P01） |
| R-16 | 文档注释落后于实现 | **docs** |

R-02、R-04、R-14 实际上已经由降级分配器的实现关闭（见 `docs/P04降级分配器_算法与防护_20260904.md` 的防护 C、⑧ 回喂、解锁联锁），**但当时的提交信息里没写 R 编号，所以从 R 那一侧看不出它已关闭**。挂靠要解决的正是这件事：从任一端都能查到另一端。

## 9. 目录布局与数据

### 一条分支一个目录

现行做法（2026-09-04 确认）：

| 目录 | 父仓库分支 | `ardupilot` 分支 | 角色 |
| :-- | :-- | :-- | :-- |
| `~/UAV` | `main` | **`dev-algo`** | **主干**，共享层的改动落在这里 |
| `~/UAV-p04` | `main` | `dev-p04` | 专题 |

专题目录按 `~/UAV-<分支后缀>` 命名，与分支名对应。

**不要两个人或两个 agent 共用同一个目录。** 2026-09-04 实际发生过：一方未提交的 `AP_Motors` 改动被另一方 `git add` 卷进了一个描述完全不同的提交（`22191ad308`），历史里没有它的来由；分支也在一方准备提交时被从 `dev-algo` 切到 `dev-p04`。**分支解决不了共用目录的问题，独立目录才能。**

目前各目录是**独立克隆**（各自一份 1.2 GB 对象库）。若要省磁盘可改用 `git worktree add`，共享对象库、各自独立 index，效果相同：

```bash
git worktree add ../ardu-guidance dev-guidance
```

### `data/` 只在 `~/UAV` 里

飞行日志、真机参数、地面站视频都在 `~/UAV/data/`，而该目录被 `.gitignore` 第 9 行排除，**因此不会出现在任何其他克隆或 worktree 中**。

在专题目录里分析真机数据时，路径要显式指向 `~/UAV/data/`，不要在本目录下找。

> 顺带：用这些数据前**先核日志内的 `EFT_CAAC` 序列号**，不要采信文件名和目录归属。`data/` 下同时存放着多架飞机的日志——带 ESC 消息的属于在研机 `X1001202505060001`（`EFT_CAAC 001F0032`），不带的属于其他飞机（如 X6100F `EFT9988776655` / `001E0042`）。2026-09-03 曾因为混用这两架的数据得出过一个会造成事故的错误结论。
