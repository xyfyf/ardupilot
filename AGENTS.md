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

## 7. 分支：主干开发

**日常开发全部在 `dev-algo` 上做。** 不按问题开分支，也不按领域开分支。

### 为什么不按领域分

2026-09-04 试过一版按 GNC 分层的方案（`dev-control` / `dev-planning` / `dev-nav`），推演到实际使用时立刻垮掉：

**P05 电子围栏要同时改 `AC_Avoid`/`AC_PosControl`（限速器）和 `AC_Fence`（几何、越界动作）。** 分到两条分支上，开发时要来回切分支或 stash；更要命的是**没法验证**——围栏修复必须限速器与几何同时在场才能测，拆开后哪条分支都跑不出正确结果。

**一个连自己都验证不了的分支划分没有存在价值。** P07 圆周抽动同理，跨 `AC_ArcNav` 与姿态环。

### 隔离本身买到的东西也比预期少

统计近 80 个提交，**真正被多个问题编号动过的文件只有测试工具**：

```
Tools/eft_issue_repro/reproduce.py    P03、P04 与未标号提交都在改
Tools/eft_issue_repro/regression.py   P02、P04 与未标号提交都在改
```

算法代码基本是单问题的——**不同问题本来就不太会碰同一个文件，分支提供的隔离大部分是冗余的**。而它的成本是实的：切换、stash、分叉、反向合并（2026-09-04 一天内三次）。

### 主干开发在这里为什么安全

前提是本仓库已经形成的习惯：**新能力一律参数门控、默认关闭**。

```
MOT_FAIL_RPM 0    MOT_FAIL_ROVR 0    MOT_FAIL_YTRK 0
MOT_STOP_DECL 0   SIM_ENGINE_TAU 0   MOT_FAIL_IDX 0
```

这些默认值使代码进主干后**行为与合并前逐位相同**，启用与否是参数的事。上游 ArduPilot 也是这么做的——EKF、控制、导航、驱动全在一个 `master` 上，没有任何按子系统划分的长期分支，只有短命的 `pr-*` 分支和发布分支。

### 什么时候才开分支

只有**无法参数门控**的改动才需要：

- 改既有参数的默认值
- 重构共享路径
- 动 EKF 这类关不掉的东西

**短命**：做完即合即删，不长期存在。命名 `pr-<一句话>`，与上游习惯一致。

### 并发靠目录，不靠分支

见第 9 节。分支解决不了两个 agent 共用一个目录的问题，独立目录才能。

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
