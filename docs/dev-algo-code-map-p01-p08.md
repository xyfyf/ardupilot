# P01–P08 代码接入点索引

日期：2026-09-02 · 分支 `dev-algo` · 落点 `1492834136`

问题编号与目标口径的**权威定义在 `~/UAV/ArduPilot六旋翼问题与算法验证目标基线.md`**，本文
不复述。这里只回答一个问题：**要动 P0x，去哪个文件的哪一行。**

行号对应上述落点，改动后会漂移；函数名与参数名是稳定的检索锚点，行号只是加速定位。

---

## 0. 先看时序：所有接入点都落在这条链上

```text
传感器后端 → AP_InertialSensor / AP_Compass / AP_GPS / AP_RangeFinder
           → AP_AHRS + EKF3
           → Mode::run()                     ArduCopter/mode_*.cpp
           → AC_WPNav / AC_Loiter / AC_Circle / AC_ArcNav
           → AC_PosControl                   位置 P → 速度 PID → 加速度 → 倾角
           → AC_AttitudeControl              推力向量 + 航向 → 角速度目标
           → 角速度 PID (+ VFF)
           → AP_MotorsMatrix                 混控 / 失效重分配
           → PWM / ESC
```

调度入口是 `ArduCopter/Copter.cpp:113` 的 `scheduler_tasks[]`。表头的 `FAST_TASK` 序列
每个主循环周期无条件按表序跑完（EFT_CAAC 为 400 Hz），顺序是：

| 序 | 任务 | 位置 |
| :-- | :-- | :-- |
| 1 | `ins.update()` | `AP_InertialSensor` |
| 2 | `run_rate_controller_main` | `ArduCopter/Attitude.cpp:10` |
| 3 | `motors_output_main` | `ArduCopter/motors.cpp:120` |
| 4 | `read_AHRS` | `ArduCopter/Copter.cpp:913` |
| 5 | `read_inertia` | `ArduCopter/inertia.cpp:4` |
| 6 | `update_flight_mode` → `flightmode->run()` | `ArduCopter/mode.cpp:410` |

**角速度环与电机输出排在 AHRS 和飞行模式之前，是刻意的一拍流水线**：本周期底层用上一
周期的姿态目标，本周期模式算出的目标供下一周期用。跨层改动不能假定"同一时刻原子更新"。

`AC_PosControl` 的两条通道分开调用，模式层就已分流：

- 水平：`update_NE_controller()` `AC_PosControl.cpp:692`
- 垂直：`update_U_controller()` `AC_PosControl.cpp:1095`

模式层还夹一个 `AltHoldModeState` 状态机（MotorStopped / Landed_Ground_Idle /
Landed_Pre_Takeoff / Takeoff / Flying），**所有地面相关行为都挂在它上面**。

---

## P01 着陆末段砸地与落地识别滞后

| 接入点 | 位置 | 作用 |
| :-- | :-- | :-- |
| 下降段控制 | `ArduCopter/mode_land.cpp:54` `ModeLand::run()` | 着陆模式主循环 |
| 垂向下降律 | `ArduCopter/mode.cpp:671` `Mode::land_run_vertical_control()` | `LAND_SPEED` / `LAND_ALT_LOW` 的实际执行处，所有模式的着陆共用 |
| 落地判定 | `ArduCopter/land_detector.cpp:37` `update_land_detector()` | 加速度平稳 + 下降率低 + 油门低三条件计时 |
| 判定结果落地 | `ArduCopter/land_detector.cpp:207` `set_land_complete()` | 置位后才允许上锁 |
| 油门混合 | `ArduCopter/land_detector.cpp:281` `update_throttle_mix()` | 决定姿态与油门的权限配比 |
| 增益衰减 | `AC_AttitudeControl::landed_gain_reduction()`，由 `mode.cpp:410` 每周期调用 | 落地后压低增益抑制地面共振 |

**日志**：`LDET`（`land_detector.cpp:194`），记录判定标志位与计数器，用于回答"卡在哪个
条件上"。

**仿真侧支撑**（现象要能复现才谈得上验证）：

- 近地增升 `libraries/SITL/SIM_Frame.cpp:739`，Cheeseman-Bennett 静态项 ×
  一阶滞后（`ground_effect_tau`）。**滞后项比静态项更关键**——气垫的建立与消散有时间
  常数，控制器把油门修到已建立的气垫上，气垫消失时就被甩下来，这正是砸地的机理。
- 涡环 `libraries/SITL/SIM_Motor.cpp:225` `Motor::calc_thrust()`，`vrs_gain` 为 0 时禁用。
- 机型参数在 `Tools/eft_issue_repro/eft_hexa.json`：`ground_effect_gain = 0.26`、
  `ground_effect_radius = 0.5`、`ground_effect_tau = 0.15`。

**复现**：`reproduce.py landing`（AUTO 起飞 → 两航点 → `NAV_LAND`）。加 `--baseline` 把
物理项归零做 A/B，确认现象来自模型而非脚本。

**当前状态**：触地下降速度已压到 0.43 m/s，但**触地到上锁仍需 1.80 s**，落地识别滞后
未关闭。`TODO.md` P1（`SID_AXIS` 20~24 垂向系统辨识）是这一项的前置。

---

## P02 高速运动时反拉抽动

| 接入点 | 位置 | 作用 |
| :-- | :-- | :-- |
| 角速度环 | `AC_AttitudeControl_Multi.cpp:475` `rate_controller_run_dt()` | PID 主体 |
| 速度力矩前馈 | `AC_AttitudeControl_Multi.cpp:508` `update_velocity_feedforward()` | 抵消平动气流在桨盘上产生的力矩 |

**前馈加在 PID 输出之后**（`_motors.set_roll(... + _vel_ff.x)`），这意味着它天然落在
`landed_gain_reduction()` 的作用范围**之外**。N-02 修的就是这个：不解锁、spool 非
`THROTTLE_UNLIMITED`、速度估计无效时清零并复位滤波器，再按 `_landed_gain_ratio` 同比例
渐隐。**漏掉这层门控的后果是坡地起飞侧翻**，下游任何增益衰减都收不回来。

**参数**（`AC_AttitudeControl_Multi.cpp:320` 起）：

| 参数 | 默认 | 含义 |
| :-- | --: | :-- |
| `ATC_VFF_RLL` | 0 | 每 m/s 机体侧向速度前馈的归一化滚转力矩 |
| `ATC_VFF_PIT` | 0 | 同上，对应机体前向速度，通常与 RLL 反号 |
| `ATC_VFF_MAX` | 0.15 | 前馈上限 |
| `ATC_VFF_OPT` | 0 | bit0 = 减去风估计（默认用地速，不额外信任估计器） |

**默认全零，出厂行为不变。**

**日志**：`VFF`（`ArduCopter/Log.cpp:559`），字段 `RawR/RawP/OutR/OutP/Scl`，可直接看限幅
前后与渐隐比例。

**增益来源**：`Tools/eft_issue_repro/fit_vel_ff.py`，从日志 `PIDP.I` 对机体前向速度做回归。

**复现**：`reproduce.py reverse`（LOITER 加速到 5 m/s 后突然反向，重复三次）。

**当前状态**：**未关闭，回归条目是护栏不是达标线。** VFF 把 I 项负担降 67%
（0.1213 → 0.0397），但**峰值俯仰速率没有下降**，不能凭 SITL 数字宣称抽动消失。

---

## P03 磁力计与 IMU 坐标系自动对齐

这一项的主体在**离线工具**，不在固件里。

| 接入点 | 位置 | 作用 |
| :-- | :-- | :-- |
| 偏差注入 | `libraries/AP_Compass/AP_Compass_SITL.cpp` | SITL 里人为加装配偏差，用于造出已知真值 |
| 辨识与评估 | `Tools/eft_issue_repro/reproduce.py:1705` `run_mag_align()` / `:1787` `summarise_mag_align()` | 用不含磁罗盘的航向做基准，在稳定直线段上取样 |
| 补偿落地 | `COMPASS_*` 自定义旋转参数 | 辨识结果最终写回参数，不新增固件代码 |

**缺口**：离散 90° 旋转上游已有，**小角度残差是空白**——这才是本项要补的部分。

**复现**：`reproduce.py mag-align`（默认 60 m 直线段、25 m 高）。

**已知局限**：GPS 地速最小二乘交叉验证**对多旋翼不适用**（基线文档 §5 已论证），不要
再走这条路。

---

## P04 电机或桨叶失效容错

改动量最大的一项：`AP_MotorsMatrix` +480 行，`AP_MotorsMulticopter` +58 行。

| 接入点 | 位置 | 作用 |
| :-- | :-- | :-- |
| 混控主体 | `AP_MotorsMatrix.cpp:216` `output_armed_stabilizing()` | 正常六旋翼混控 |
| 失效检测 | `AP_MotorsMatrix.cpp:605` `update_failure_detection()` | 盯 ESC RPM，判定电机停转 |
| 降级分配 | `AP_MotorsMatrix.cpp:682` `allocate_redistributed()` | 摘除失效电机后的伪逆约束分配 |

**参数**（`AP_MotorsMulticopter.cpp:239` 起）：

| 参数 | 默认 | 含义 |
| :-- | --: | :-- |
| `MOT_FAIL_IDX` | 0 | 手动指定失效电机（测试用） |
| `MOT_FAIL_RPM` | 0 | 低于此转速视为停转，0 = 关闭检测 |
| `MOT_FAIL_TIME` | 200 ms | 持续多久才判定，防遥测丢包误判 |
| `MOT_FAIL_THST` | 0.15 | 只在指令推力高于此值时才判定 |
| `MOT_FAIL_YAW` | 0 | 保留多少偏航权限（把偏航放回需求） |
| `MOT_FAIL_OPP` | 0 | 是否同时停对角电机 |
| `MOT_FAIL_ALLOC` | 1 | 分配器模式 |
| `MOT_FAIL_YSUP` | 0.7 | 抑制寄生偏航力矩的权重 |

`MOT_FAIL_YSUP` 与 `MOT_FAIL_YAW` **不是一回事**：前者是在最小范数解里挑残余偏航力矩小
的那个（目标为零），后者是把偏航放回需求让控制器去跟。六旋翼停一个电机后无论如何守不
住航向，`YSUP` 买到的是**转得慢**，不是转停。要看的是**平均偏航率不是峰值**——峰值是切换
瞬态，平均值才决定机头扫过多少、飞手有多少时间。

**日志**：`MALC`（`ArduCopter/Log.cpp:555`），字段 `Fail,DThr,DRll,DPit,DYaw,AThr,ARll,APit,AYaw`，
即需求与实际分配的四轴对照，用来看分配器残差。

**仿真侧**：`SIM_Motor.cpp` 的 `set_thrust_scale()` 支持逐电机推力散布，`max_rpm`（模型里
3000）供 ESC 遥测——**没有 RPM 遥测检测器就永远不触发**。

**复现**：`reproduce.py motor-fail`；误报检查用 `uturn-auto`。

**安全边界（必须写进任何对外结论）**：只证明了"**有 ESC RPM 遥测的单电机停转**、目标构型、
指定参数和风场"可控。**桨叶断裂/脱桨表现为电机超速与强振动，低转速检测器明确不覆盖。**

> 基线文档 §6 有一条"更正（2026-08-28）：降级分配存在油门量纲错误，本节此前实测值全部
> 无效"。引用 P04 的历史数字前先确认是更正之后的。

---

## P05 电子围栏预测约束与边界稳定飞行

**三条互不相同的链路，命名和安全目标必须分开：**

| 链路 | 位置 | 性质 |
| :-- | :-- | :-- |
| ① 违规检测 | `AC_Fence`，由 `Copter.cpp:154` `SCHED_TASK(fence_check, 25 Hz)` 驱动 | 只判定，不干预 |
| ② 违规动作 | `ArduCopter/fence.cpp:39` `Copter::fence_check()` | **越界之后**的处置 |
| ③ 预防限制 | `AC_Avoid`，各模式内调用 | **不让越界** |

**②本 fork 已改**：越界后统一切 `LOITER`（不可用才退 `BRAKE` → `LAND`），依靠 Loiter 自身
刹车 + 围栏避障拦住，同时完整保留飞手摇杆权。原上游会夺权切 RTL/Brake/Land。

**③分速度层与姿态层：**

| 层 | 入口 | 适用模式 |
| :-- | :-- | :-- |
| 速度层 | `AC_Avoid.cpp:200` `adjust_velocity()` → `:137` `adjust_velocity_fence()` | Loiter、Guided（杆量即速度） |
| 姿态层 | `AC_Avoid.cpp:509` `adjust_lean_for_fence_rad()` | AltHold `mode_althold.cpp:88`、PosHold `:91`、Stabilize `:21`、Drift `:160`、FlowHold `:348` |

姿态层是本 fork 新增的，是**硬限制**：`adjust_roll_pitch_rad()`（传感器避障）故意只压到
机体权限的 75% 好让飞手能压过障碍读数，而围栏是另一种承诺，向外分量直接钳死。

**刹车剖面：`b34dc4c113`（2026-09-02）已改，这是 `~/UAV/TODO.md` P3 的首要待办**

改之前：剖面减速度取 `AC_AVOID_ACCEL_CMSS_MAX = 1.0 m/s²` 硬编码（`AC_Avoid.h:12`），
`AC_Avoid.cpp:211/403` 用 `MIN()` 压住传入值，**`AVOID_ACCEL_MAX = 4` 在这条路径上完全
无效**。刹车距离是 `v²/(2a)`，1.0 m/s² 下 7 m/s 进场要 25.5 m，**超过 2026-08-31 那场围栏
的半宽（22–26 m）本身**——**装不进场地的剖面不是安全裕度，是必然越界**，该架次如实越界
6 次、最深 18.6 m。

改之后：`adjust_velocity_fence()` 增加末位参数 `accel_cmss_max`，默认仍是那个常数，**速度层
调用方行为不变**；姿态层 `adjust_lean_for_fence_rad()` 传入机体自己的
`g·tan(veh_angle_max_rad)`（本机 2.63 m/s²）。同一次刹车降到 **9.3 m**。

同时 `AC_AVOID_FENCE_PROFILE_FRAC` 由 0.4 提到 **1.0**（`AC_Avoid.h:46`）。裕度改由指令律
提供：`a_req = a_prof·(v_out/v_allow)²`，在机体能力处饱和——超速用**超过剖面代价**的减速度
来答，剖面本身就不必留小。

> **代价要记住**：`FRAC = 1.0` 时剖面即能力，跟踪剖面已花光倾角预算，平方律没有增长空间。
> **抗风与估计误差的储备，现在是"剖面减扰动"而不是"剖面减机体能力"。**这是现场数据换来的
> 取舍——剖面装得进场地，代价是原先藏在 de-rating 里的那份储备。要买回来就下调 `FRAC`，
> 它是 SITL 冲刺扫描显示风中裕度过薄时该二分的那个旋钮。

**尚未完成**（同 `TODO.md` P3）：冲刺场景 SITL 补测（3/5/7 m/s 直冲，场地按真机 22–26 m
半宽而非 60 m 六边形）、风包线按顺风/逆风分列、复飞前把 `FENCE_MARGIN` 改回 ≥5 m 与
`FENCE_ACTION` 改为 4。

风的作用是把预算再吃掉一半，造成 **2.8 倍方向不对称**：顺风侧有效减速度 0.52–0.65、逆风侧
1.38–1.46 m/s²。**2026-08-31 真机架次越界 6 次、最深 18.57 m**，全部集中在顺风的南/西两边。
现场"有两个方向围栏失效"的说法要撤掉——四条边判据都正常，差别全部来自风。

**复现**：`reproduce.py fence`（逐档撞边界）、`route`（AUTO 作业航线）。回归条目 8 条，
覆盖 ALT_HOLD / POSHOLD / STABILIZE / DRIFT / LOITER × 风速档。

**命名警告**：现有 AUTO 用例验证的是"**越界后的 Brake 距离**"，不是"预防性不越界"。这两个
安全目标不能混用同一个名字。

---

## P06 全航程三维连续轨迹、匀速跟踪与平滑避障

| 接入点 | 位置 | 作用 |
| :-- | :-- | :-- |
| 匀速转弯生成器 | `libraries/AC_WPNav/AC_ArcNav.{h,cpp}`（新建，736 行） | 回旋曲线过渡 + 定曲率弧 |
| AUTO 接入 | `ArduCopter/mode_auto.cpp:7` `auto_arc_nav`，`:595` `set_arc()`，`:635` `update()` | 任务航线里的 U 型掉头 |
| GUIDED 接入 | `ArduCopter/mode_guided.cpp:18` `guided_arc_nav` | 三阶设定点直喂 |

**为什么不能用航点**：SCurve 混合直线段，拐角速度掉到 `v·cos(θ/2)`，且要求每段足够长才
展得开。田间 U 型掉头是半个喷幅半径上的 180° 反向，对两者都是最坏情况——**SITL 实测 2 m/s
下掉速 83%，调多大的 `WPNAV_ACCEL` / `WPNAV_JERK` 都救不回来。**

**为什么不是纯圆弧**：直线接圆弧会让曲率在一个采样内从 0 跳到 1/r，横向加速度和切线转率
跟着阶跃（r=3 m、2 m/s 实测 0→1.33 m/s²、0→38°/s）。没有物理系统能跟阶跃，结果是机头平均
落后切线 9.6°。所以插入回旋曲线（clothoid）让曲率随弧长线性爬升，螺旋段长
`L_s = v³/(r·jerk)`。曲率斜坡转成航向是 Fresnel 积分，**没有闭式解，路径是数值积分的**。

**两条设计约束**：

- `AC_ARCNAV_TILT_FRACTION = 0.7`——转弯只准吃倾角权限的 70%，余下留给跟踪修正和风。
  实测按满权限设计的弧会退化成 31% 掉速，按 70% 设计掉速在 5% 以内。
- 参考点**按飞机实际跟得上的速率推进**，不按墙钟时间（沿迹误差调速器，同
  `AC_WPNav::advance_wp_target_along_track()`）。纯按时间推进会让参考跑掉，误差增长、
  控制器饱和，最后飞的是对的圆但速度只剩零头。

`set_arc()` **拒绝飞不了的请求**而不是默默降速，GCS 会收到具体原因
（`mode_guided.cpp:250/271/283/290`）。

**日志**：`ARCN`（`mode_auto.cpp:671` / `mode_guided.cpp:365`），字段
`Prog,Gov,Spd,Tgt,PErr,HdgE,Spir,Alat,HdgR`。

**复现**：`reproduce.py uturn`（AUTO 任务）、`uturn-guided`（绕开 SCurve）、`uturn-arcnav`。

**当前状态**：已测工况有效（掉速 ≤1%），但**仍绕开动态避障、地形跟随和飞行中围栏变化**，
生命周期契约未完全收口。`mode.cpp:474` 有一句注释明确写着 ArcNav 不引用规划器和围栏。

---

## P07 小半径圆周飞行抽动

| 接入点 | 位置 | 作用 |
| :-- | :-- | :-- |
| 圆周参考生成 | `libraries/AC_WPNav/AC_Circle.cpp:201` `update_ms()` | 生成 NE 位置/速度/加速度目标 |
| 转弯加速度余量 | `AC_Circle.cpp:383`，`AC_CIRCLE_ACCEL_MARGIN = 0.5` | 稳态转弯只准用一半可用横向加速度 |
| 目标消费 | `AC_PosControl.cpp:692` `update_NE_controller()` | 输入契约的另一端 |

**瓶颈在参考生成和位置控制输入契约，不在姿态环跟踪。**余量留 50% 的理由与 ArcNav 的 70%
同源：进入圆周、抗风、修正位置误差抽的是同一份加速度，全部投给稳态转弯就没有飞的余地。

**明确的反向结论**：**仅提高 `WPNAV_ACCEL` 会让结果更差**，不应现场盲调。

**复现**：`reproduce.py circle`（CIRCLE 模式 2 m 半径，1.0/1.5/2.0/2.5 m/s 四档）、
`loiter-circle`（LOITER 杆量画圈，无 `AC_Circle` 自限速，可打到饱和）。

**当前状态**：**回归条目守的是自动 CIRCLE 模式，不构成对 P07 的验证**（`03b87206c7` 已把
该条目从 P07 改挂 P06）。P07 的实际问题是**飞手手动打杆绕圈**时机身抽动，**该复现场景尚未
建立**。回归里那条倾角饱和护栏只保证自动模式不退化。

---

## P08 动力系统不一致性与常态偏航配平

固件侧**没有新增代码**——这一项当前是标定与检查问题，不是算法问题。

| 接入点 | 位置 | 作用 |
| :-- | :-- | :-- |
| 逐电机推力散布 | `libraries/SITL/SIM_Frame.cpp:513` `motor_thrust_scale`，`:611` 注入 | 仿真里造出不一致 |
| 推力模型 | `libraries/SITL/SIM_Motor.cpp:225` `calc_thrust()` | 散布最终作用处 |

真实动力系统不可能一致：电机与 ESC 公差、桨距散布、安装角误差都会让各臂在同一指令下推力
不同。这既是 P08 本身的主题，**也决定了 P04 的失效切换能有多平顺**——电机完全一致时重分配
是纯算术。

**证据**：来自 00000219 / 00000220 / 00000231 三份实机日志的常态偏航配平量。
**仿真定量映射（2026-08-28）**已给出"需要多大安装偏差才能造成这个现象"。

**当前状态**：**卡在现场**（`TODO.md` P5）。候选物理原因需要现场检查，**不需要飞行**。
软件侧推不动。

---

## 附：验证闭环的入口

```text
现场架次 → .EFT 日志（ChaCha20-CTR 加密，密钥由文件名 SN 派生）
        → 解密 → DFReader.DFReader_binary 解析（不能用 mavutil 按扩展名嗅探）
        → Tools/eft_log_analysis/log_control_metrics.py       量化
        → Tools/eft_log_analysis/log_to_sitl_scenario.py      转 SITL 场景
        → Tools/eft_issue_repro/reproduce.py <场景>           单项复现
        → Tools/eft_issue_repro/regression.py [--only P0x]    集中回归
        → ./waf configure --board EFT_CAAC && ./waf copter    发布固件
```

**回归判据分两类，读结果前先看清是哪一类**：

- **护栏**——P02、P07 这类未关闭的项，只保证不比当前基线更差，数字不是达标线；
- **达标线**——其余项。

**估计器与控制器的验证路径不对称**：`Tools/Replay` 能把日志里的原始传感器输入按原时序喂回
EKF3 离线重跑，因为估计器对飞行只读；**控制器不行**——换了控制律飞机响应就变了，后续传感器
读数不再成立。所以控制改动只能靠 SITL 场景，不能靠重放。

**并行作业**：`reproduce.py` 用 `/tmp/ardupilot-eft-issue-repro-sitl.lock` 拒绝并发，因为所有
场景共用同一组 SITL 端口。开跑前先
`pgrep -af "[a]rducopter|[r]egression.py|[r]eproduce.py"`，有别人的进程就等，不要杀。
