# EFT 日志离线分析工具

本 fork 专用，不属于上游 ArduPilot。两个脚本都只读日志，不改任何东西。

用仓库自带的 `modules/mavlink/pymavlink`（含 EF 包头支持），路径相对脚本自身解析，不依赖当前工作目录。**不要**改用 pip 装的 pymavlink——它解不出本机型默认发出的 `0xEF` 包头，详见根目录 `AGENTS.md` 第 4 节。

## check_replay_ready.py

判定一份 `.bin` 能否用于 `Tools/Replay` 离线在环重放。

```bash
python3 Tools/eft_log_analysis/check_replay_ready.py <log.bin> [...]
```

逐项检查四个硬性前提：`LOG_REPLAY=1` 与 `LOG_DISARMED>=1` 两个参数、`RFRH`/`RFRF`/`RISH`/`RISI` 帧骨架、各传感器的 replay 记录、以及帧连续性（丢帧会让重放在缺口处失真）。通过就直接把重放命令打出来，不通过会逐条列出缺什么。

判定按**实际记录条数**，不是搜字符串。日志开头的 FMT 表会声明固件认识的全部消息类型，所以 `grep RFRH` 在任何一份日志上都能命中，看起来像有其实没有——这是个很容易踩的坑。

## replay_ab.py

同一份日志用两组参数各重放一遍 EKF，逐点对比。

```bash
python3 Tools/eft_log_analysis/replay_ab.py <log.bin> \
        --a EK3_BARO_HDOP=3.0 --b EK3_BARO_HDOP=0
```

这是**真正的反事实**：输入完全相同（同一份录下来的传感器时序），只有参数不同，差异百分之百来自那个参数。机上跑影子滤波器做不到——影子与主滤波器共享状态，也改不了"当时实际融合了什么"。`05dc8de718` 删掉的 EBFH 就是那种做法。

改了 EKF 代码想验证时同样适用：分别用改前/改后的固件编译 `Replay`，两次跑同一份日志。

需要先编译 Replay，注意它会覆盖当前板子配置：

```bash
./waf configure --board sitl && ./waf replay
./waf configure --board EFT_CAAC     # 记得切回来
```

## log_to_sitl_scenario.py

把日志拆成 SITL 可复现的场景，让整个固件（EKF + 控制器 + 混控）闭环重跑。

```bash
python3 Tools/eft_log_analysis/log_to_sitl_scenario.py <log.bin>
cd <场景目录> && ./run_sitl.sh
```

产出参数、起点、模式与杆量时间线、任务航点、SITL 覆盖参数、以及一份待填的机型模型。**仿真轨迹会和真实飞行发散**，那是物理模型与实机的差异；`model.json` 里的 `mass` / `diagonal_size` / `disc_area` 日志里没有，必须人工填，否则推重比不对。

## log_control_metrics.py

从真实飞行数据量化控制器表现：姿态环/角速率环跟踪误差（RMS、P95、最大、偏置）、高度环、PID 积分限幅比例、振动与加速度计削顶，并检查各消息的实际记录频率是否够用。

```bash
python3 Tools/eft_log_analysis/log_control_metrics.py <log.bin> [...]
python3 Tools/eft_log_analysis/log_control_metrics.py data/*.bin --csv out.csv
```

飞行段用 `CTUN.ThO >= 0.15` 判定，排除地面数据。单位取自日志自带的元数据而非硬编码（`CTUN.CRt` 是 cm/s 不是 m/s）。

这是**开环性能评估**，不是仿真。控制器无法离线在环重放——日志记的是当时那套控制律产生的轨迹，换一套控制律飞机响应就变了，后续传感器读数不再成立。估计器可以重放是因为它对飞行只读。所以这个脚本的用法是横向对比：不同架次、不同参数下同一套指标谁更好。

注意默认 `LOG_BITMASK` 没开 `ATTITUDE_FAST`，`ATT`/`RATE` 只有 10 Hz，而姿态环跑在 400 Hz。脚本会对低于 50 Hz 的记录发出混叠警告——那种情况下高频振荡和超调是看不到的，指标只反映低频段。
