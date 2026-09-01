# dev-algo 架构审查、验证结果与集中改进建议

日期：2026-09-01

对象：EFT 教练机/培训机，ArduPilot 六旋翼 `HEXA / DJI_X`

分支：`dev-algo`

审查落点：`54663c19ff` 及其父提交

## 1. 结论先行

当前分支已经形成一套有价值的“问题复现—算法原型—SITL 回归—日志量化”闭环，尤其是单电机停转降级、固定磁偏角辨识、姿态模式围栏和协调圆弧。但它还不是可直接宣称七项问题全部解决的生产版本。

本轮最重要的结论如下：

1. `EFT_CAAC` 与 SITL 都能完成编译；最终 `EFT_CAAC` 固件只剩 **7,196 B Flash**，已经进入必须管理的工程风险区。
2. 现有集中回归原始结果为 13/15；另外两项由并发 SITL 抢占固定端口造成，并非算法失败。独立复测后，两项算法判据均通过。
3. P01 已把触地下降速度压到 0.43 m/s，但触地到上锁仍需 1.80 s，落地识别慢和触地后撤推问题仍未关闭。
4. P02 的 VFF 候选在同一 5 m/s 急反向场景中显著减轻 I 项负担，但峰值俯仰速率没有下降，不能仅凭 SITL 数字宣称“抽动已消失”。
5. P04 只证明“有 ESC RPM 的单电机停转、目标构型、指定参数和风场”可控；桨叶断裂/脱桨可能表现为电机超速与强振动，当前低转速检测器明确不覆盖。
6. P05 手动姿态模式和 LOITER 的现有围栏场景表现良好；AUTO 用例验证的是越界后的 Brake 距离，不是预防性不越界，命名和安全目标必须拆开。
7. P06 的 AUTO/GUIDED 协调圆弧在已测工况有效，但仍绕开动态避障、地形跟随和飞行中围栏变化，生命周期契约也未完全收口。
8. P07 的主要瓶颈在圆周参考生成和位置控制输入契约，而不是姿态环跟踪。仅提高 `WPNAV_ACCEL` 会使结果更差，不应现场盲调。
9. 现阶段不建议把整个飞控改成 INDI/增量式控制。应先补齐实机模型、日志、分配器残差反馈和轨迹前馈；若以后引入 INDI，放在可切换的自定义控制器边界内做 A/B。

## 2. 分支谱系与同步风险

`dev-algo` 不是从当前 ArduPilot 上游主线新建：

- 分叉基点：`8bf9f2e035437534ea8d461c42b9c605c281f310`；
- 时间：2026-08-20 19:16:30 +09:00；
- 基点说明：`Drop three stray gitlinks under mcp-servers/`；
- 当前相对该基点有 68 个提交；
- 当前相对 `origin/ardupilot-ubuntu` 为 69 个独有提交、缺少 6 个提交；
- 当前相对 `origin/master` 为 218 个独有提交、缺少 3,919 个提交；共同祖先为 2025-08-20 的 `aabbe500aa`。

因此不能把“拉取最新 `origin/master`”理解成直接 merge/rebase 后即可试飞。建议建立三层分支：

1. `ardupilot-ubuntu`：培训机量产/现场基线；
2. `dev-algo`：七项问题的集成验证分支；
3. 每项算法的短生命周期分支：如 `dev/p02-vff`、`dev/p04-allocation`。

上游同步应先在专门的 integration 分支完成编译、参数迁移和全回归，再进入 `dev-algo`；不要在飞行试验前临时吞入数千个上游差异。

本轮还遇到同一父提交上的两个并发 N-02 实现：本地 `957454cbfe` 与远端 `d79de03cd6`。合并提交 `54663c19ff` 保留了本地 `_landed_gain_ratio = 0.0f` 初始化和落地渐隐，同时接纳远端 reverse 场景修复，未 force-push 或丢弃任一侧历史。

## 3. ArduPilot 在本项目中的实际执行链路

目标构型的控制主链如下：

```text
传感器后端/驱动
  -> INS/AHRS/EKF3 状态估计
  -> Copter flight mode（AUTO/LOITER/ALT_HOLD/...）
  -> AC_WPNav / AC_Loiter / AC_ArcNav
  -> AC_PosControl（位置->速度->加速度/推力向量）
  -> AC_AttitudeControl（推力向量/航向->角速度目标）
  -> 角速度 PID + VFF
  -> AP_MotorsMatrix（六旋翼混控/失效重分配）
  -> PWM/ESC/电机
```

`Copter::fast_loop()` 以目标板 400 Hz 运行。当前循环先执行角速度控制和电机输出，再更新 AHRS、惯性状态和 flight mode 目标，因此存在刻意的一周期流水线：本周期低层使用上一周期模式目标，本周期模式计算供下一周期使用。新增算法必须尊重这个时序，不能把跨层状态当作“同一时刻原子更新”。

主要接入点：

| 能力 | 当前模块 | 关键接入 |
| --- | --- | --- |
| 着陆 | `ArduCopter/mode_land.cpp`、`land_detector.cpp` | U 轴位置控制、落地检测、spool state |
| 高速反拉 | `AC_AttitudeControl_Multi` | 角速度 PID 后的速度力矩前馈 |
| 磁对齐 | Compass、AHRS/EKF3、离线工具 | 自定义旋转参数、SITL 注入、日志辨识 |
| 动力失效 | `AP_MotorsMatrix` | RPM 检测、摘除、伪逆约束分配、yaw 降级 |
| 围栏 | `AC_Fence`、`AC_Avoid`、各 flight mode | 违规检测、预防限制、违规动作三条不同链路 |
| 平滑航线 | `AC_WPNav`、`AC_ArcNav`、AUTO/GUIDED | S 曲线、协调圆弧、航向/曲率约束 |
| 圆周抽动 | `AC_Circle`、`AC_PosControl` | 圆周参考生成、NE 位置/速度/加速度输入 |

## 4. 本轮实际改动

本轮没有实现新的大算法，只合并和补齐已由测试直接证明必要的最小改动：

### 4.1 N-02 气流力矩前馈飞行状态门控

涉及：

- `libraries/AC_AttitudeControl/AC_AttitudeControl.h`
- `libraries/AC_AttitudeControl/AC_AttitudeControl_Multi.cpp`

行为：

- `_landed_gain_ratio` 显式初始化为 0；
- 未解锁、spool state 非 `THROTTLE_UNLIMITED`、速度估计无效时，VFF 清零并复位滤波器；
- VFF 随 `landed_gain_reduction()` 的同一比例渐隐，避免 PID 已退让而 PID 后置前馈仍与地面反力对抗。

该改动只在 `ATC_VFF_RLL/PIT` 非零时产生效果；默认值为 0，默认飞行行为保持关闭。

### 4.2 reverse 场景入口修复

涉及：`Tools/eft_issue_repro/reproduce.py`

`run_reverse()` 原先读取未定义变量 `mode`，每次都在起飞后触发 `NameError`。现改为显式默认参数 `mode="LOITER"`，P02 专项得以真正运行。

### 4.3 固定 SITL 端口互斥

涉及：`Tools/eft_issue_repro/reproduce.py`

所有场景共用 TCP 5760 及辅助端口。本轮出现两个回归进程并发，后启动实例报 `Address already in use`，下一用例还可能误连到另一实例。新增非阻塞文件锁：检测到并发场景时立即给出占用进程号，不启动第二架 SITL，避免把基础设施冲突包装成算法失败。

这只是止住误连；集中报告仍应进一步区分 `INFRA_ERROR / INVALID_SCENARIO / FAIL / PASS`。

## 5. 构建与验证记录

### 5.1 构建

| 项目 | 结果 | 关键数据 |
| --- | --- | --- |
| `./waf configure --board sitl && ./waf copter` | 通过 | SITL 使用 `-Werror`；最终合并态链接通过 |
| `./waf configure --board EFT_CAAC && ./waf copter` | 通过 | Flash 使用 1,696,735 B；剩余 7,196 B |
| `python3 -m py_compile Tools/eft_issue_repro/*.py Tools/eft_log_analysis/*.py` | 通过 | 只能挡语法错误，挡不住本次未定义自由变量 |
| `git diff --check` | 通过 | 无空白错误 |
| 固定端口锁冲突自测 | 通过 | 已有锁时立即拒绝，不启动 SITL |

Flash 从本轮初次构建的 7,252 B 下降到最终 7,196 B。建议设置 CI 红线：低于 16 KiB 报警，低于 8 KiB 禁止合入非必要功能；最终阈值由 bootloader、参数区和升级策略共同确认。

### 5.2 集中回归

原始报告：`Tools/eft_issue_repro/runs/regression-20260901-163939/regression.md`

原始 15 项中 13 项直接通过。P04 误报和 P06 GUIDED 圆弧因另一回归实例占用/污染固定端口而运行失败；独立重跑通过。因此本轮可以说“15 个已有算法判据均取得通过结果”，但不能说“最终提交上有一轮干净的 15/15 集中回归”。修复互斥锁后应重新跑一轮作为下一提交的正式基线。

| 问题 | 场景 | 结果与关键指标 |
| --- | --- | --- |
| P01 | 着陆 | 触地 0.43 m/s；触地到上锁 1.80 s |
| P04 | 6 号电机停转，4 m/s 风 | 检测 0.22 s；滚转峰值 6.7°；稳态 2.0°；掉高 0.04 m |
| P04 | 正常 AUTO 掉头误报 | 独立复测无 stopped/degraded 消息 |
| P03 | 固定 30° 磁偏角 | 辨识 29.93°，653 样本 |
| P06 | AUTO 圆弧 | 进弧前最低 2.98 m/s；弧内最低 3.00 m/s |
| P06 | GUIDED 5 m/s 圆弧 | `ARCN` 有效区间最低 4.889 m/s；掉速 2.21% |
| P06 | 偏航能力 | 78.4°/s；yaw 混控峰值 0.61 |
| P05 | AUTO 违规后 Brake | 最远 45.8 m；对 40 m 围栏越界 5.8 m 后刹住 |
| P05 | LOITER 严格档 | 最小栏内余量 1.93 m；无越界 |
| P05 | ALT_HOLD/POSHOLD/STABILIZE/DRIFT | 2 m/s 风下最小余量约 5.02–5.03 m；无越界 |
| P05 | LOITER 多档边界 | 最小实际余量 4.93 m；最大进入余量线 0.071 m；无越界 |

GUIDED 场景同时存在一套包含入弧前加速段的粗窗口，会给出 38% 掉速；固件 `ARCN.Prog` 有效区间给出 2.21%。验收必须只保留一套定义清楚的口径，否则同一架次会得到相反结论。

### 5.3 P02 高速反拉 A/B

共同条件：问题物理模型、LOITER、约 5.4 m/s 后连续三次急反向。

| 指标 | VFF=0 | `RLL=-0.0088, PIT=0.0088` | 变化 |
| --- | ---: | ---: | ---: |
| pitch I span | 0.1213 | 0.0397 | -67% |
| pitch I RMS | 0.0261 | 0.0059 | -77% |
| pitch 姿态误差均值 | 0.90° | 0.28° | -69% |
| 三次峰值 pitch 姿态误差 | 3.39–3.50° | 2.98–3.00° | 约 -12% 至 -15% |
| 峰值 pitch rate | 116–118°/s | 118°/s 左右 | 未改善 |

结论：前馈对“长期由 I 项承担速度相关配平”的归因有支持，但峰值角速度和主观抽动没有闭环。下一步必须用同一实机动作、视频时间码和日志事件标记做 A/B；非零参数暂不进入培训机默认值。

### 5.4 P07 2 m 半径圆周

当前默认参数：

| 目标速度 | 理论向心倾角 | 最大目标倾角 | 实际均速 |
| ---: | ---: | ---: | ---: |
| 1.0 m/s | 2.9° | 3.3° | 0.99 m/s |
| 1.5 m/s | 6.5° | 6.4° | 1.39 m/s |
| 2.0 m/s | 11.5° | 8.0° | 1.50 m/s |
| 2.5 m/s | 17.7° | 8.0° | 1.46 m/s |

姿态误差均值约 0.11–0.39°，说明姿态控制器基本跟上了它收到的目标；目标本身没有提供足够的向心加速度。

仅将 `WPNAV_ACCEL` 提到 600 的反例中，2.5 m/s 档最大目标倾角反而约 4.6°、实际均速约 0.77 m/s。源码中 `AC_Circle` 会据可用加速度提高参考角速度，但调用 `input_pos_vel_accel_NE_m()` 时仍给零速度、零加速度。正确方向是生成一致的圆周位置、切向速度和向心加速度三阶参考，并在接受前按倾角、yaw、速度和 jerk 做可行性检查。

## 6. 七项问题的当前状态与建议

### P01 着陆末段与触地后对抗：部分关闭

已关闭：现有 SITL 的触地速度回归门限。

未关闭：触地后 1.80 s 才上锁；刚性地面模型不能复现起落架弹性、冲击峰值和触地后六路 PWM 极差。

建议：

1. 将验收拆成 `末段下降率、触地冲击、land_complete 延迟、撤推延迟、触地后电机不均衡` 五项；
2. 增加柔性起落架/地面接触模型，或接 Gazebo 接触模型；
3. 现场日志至少提供 `CTUN/PSC/ATT/RATE/MOTB/RCOUT/EV` 与同步视频；
4. 不要用“最终上锁时间”替代 land detector 各内部条件的逐项诊断。

### P02 高速运动反拉抽动：原型有效，未交付

建议：

1. 把 reverse 加入集中回归，并定义峰值角速度、角加速度/jerk、姿态误差、I 项换向时间；
2. 增加 `VFF input/output/fly_scale/active EKF lane/airspeed or groundspeed source` 日志；
3. 覆盖 3/5/7/10 m/s、空载/满载、前后/左右反拉、顺逆风和不同电量；
4. 参数由实机回归辨识，不直接采用本轮 0.0088；
5. 若 PID 后置前馈继续保留，应把合成后的执行器饱和反馈纳入抗积分饱和。

### P03 磁力计与 IMU 坐标对齐：辨识链路通过，自动补偿未交付

建议：

1. 只把稳定的固定安装偏角写入 `ROTATION_CUSTOM_* / CUST_ROT*`，不要飞行中无条件在线改坐标；
2. 在线估计必须有可观测性门槛、置信度、多罗盘一致性、EKF lane 一致性和回滚；
3. 增加 0° 负样本、±15/30/60/90°、不同倾角、磁干扰和双罗盘故障注入；
4. 先做“地面/标定航线自动建议值 + 人工确认”，再考虑空中闭环写参数。

### P04 电机或桨叶失效：单电机停转部分关闭

建议：

1. 分开检测“电机停转、部分失推、桨叶脱落/断裂、ESC 遥测丢失”；RPM 低只覆盖第一类；
2. 将约束分配器的 achieved wrench、各轴残差、夹紧电机、可用控制裕度反馈给 PID anti-windup 和 failsafe；
3. 对 1–6 号电机、0/2/4/6 m/s 风、不同航向、载荷/电量、不同失效高度做矩阵测试；
4. `MOT_FAIL_IDX` 必须有解锁/重启清理和显式告警，避免持久参数把下一架次带入降级混控；
5. 桨叶失效应融合 RPM、命令-RPM 残差、ESC 电流、IMU 振动和角加速度创新；
6. 目标是受控应急着陆和限制区域内落地，不承诺完整任务与航向保持。

### P05 电子围栏内稳定飞行：手动模式部分关闭

建议：

1. 测试和文档区分：目标合法性检查、飞行中预防限制、越界后 failsafe；
2. 将真实 22–26 m 田块、`FENCE_MARGIN=1 m`、凹多边形、顶点/边心、顺风和 GNSS 偏差纳入回归；
3. AUTO 若目标是绝不越界，不能只依赖越界后 Brake，任务路径与 OA 必须在生成阶段受围栏约束；
4. 失去位置估计时不能静默放行，应定义 BRAKE/LOITER/LAND/RTL 的可配置降级。

### P06 航线、航点、起飞到田间和避障平滑：核心圆弧部分关闭

建议：

1. 统一使用连续位置、速度、加速度，必要时 jerk/曲率连续的轨迹契约；
2. 起飞点到较低田面应联合规划水平和垂向 S 曲线，而不是“先升高—平飞—直降”的折线；
3. 障碍绕行轨迹必须回到同一平滑器，不让 Dijkstra/BendyRuler 的折线直接成为控制目标；
4. `AC_ArcNav` 需补模式退出、任务跳转、failsafe、拒绝后回退、动态围栏/障碍、terrain alt 的统一生命周期；
5. GUIDED 圆弧里“飞手保持 yaw 权限”的空分支应明确为真正的混合权限，或删除误导注释；
6. 以作业段速度误差、转弯半径、航向误差、加速度、jerk、喷幅重叠/漏喷为联合验收指标。

### P07 圆周飞行抽动：已定位主要结构缺口，未关闭

建议：

1. `AC_Circle` 输出与圆周几何一致的切向速度和向心加速度，不再只移动位置点；
2. 将 `ANGLE_MAX`、`PSC_ANGLE_MAX`、当前 thrust margin、yaw rate/accel 同时纳入可行速度；
3. 达不到目标速度时应明确限速并记录原因，不让参考点跑远后靠位置误差追赶；
4. circle 与 P06 共用轨迹/曲率能力接口，避免两套圆弧生成器逐渐分叉；
5. 增加 2/5/10 m 半径、顺逆时针、速度阶跃、风和载荷回归，并用视频事件标记验证“抽动”。

## 7. 横向工程建议和优先级

### P0：下一次实机试飞前

1. 再跑一轮互斥锁修复后的完整集中回归，保存一份干净 15/15 报告；
2. 将 P02 reverse 和 P07 circle 加入集中回归，禁止专项入口长期游离；
3. 为 Flash 建立 CI 预算，当前 7,196 B 不宜继续无门槛增长；
4. P04 自动摘除在真机保持关闭，直到所有电机位置、误报、部分失推和 allocator residual 完成验证；
5. 明确 P01 的触地后撤推验收，不以触地速度通过代替整项关闭。

### P1：取得首批现场日志后

1. 完成培训机质量、三轴惯量、轴距/电机坐标、单桨盘面积、总盘面积、推力曲线和电机时常辨识；当前 50 kg、2 m 轴距和 4.71 m² 总盘面积只是占位值；
2. 用同一架次的参数快照、日志、视频和场景说明做可重复 A/B；
3. 实现 P04 achieved-wrench/残差反馈和表驱动分配器测试；
4. 实现 P06/P07 统一三阶轨迹接口与可行性检查；
5. 完成 P03 多罗盘、多 lane 和负样本验证。

### P2：架构演进

1. 将七项算法的状态、输入、输出、限幅原因和回退原因统一写日志，避免只靠 STATUSTEXT；
2. 把“规划器产生目标”和“控制器跟踪目标”之间的契约显式化；
3. 若评估 INDI，先作为 `AC_CustomControl` 或等价可切换控制器，保持 PID 基线和同架次 A/B，不一次性替换位置、姿态、分配全链路。

## 8. 推荐的验收门槛

任何一项从“仿真通过”转为“实机可用”至少需要：

1. 模型参数来源明确，不再使用占位质量/惯量/盘面积；
2. baseline 与 candidate 使用相同固件基线、参数快照、场景和环境；
3. SITL 快速门禁、完整回归、系留/低风险实机、正常作业包线四级通过；
4. 日志和视频时间对齐，异常动作有事件标记；
5. 有失败判据、回退策略和恢复验证，而不只是均值变好；
6. 代码、参数、构建产物、日志、视频、场景说明能由提交号追溯。

## 9. 资料依据

本轮源码结论以仓库当前代码为准，并参考 ArduPilot 官方架构资料：

- Code Overview: <https://ardupilot.org/dev/docs/apmcopter-code-overview.html>
- Learning ArduPilot: <https://ardupilot.org/dev/docs/learning-ardupilot-introduction.html>
- Sensor Drivers: <https://ardupilot.org/dev/docs/code-overview-sensor-drivers.html>
- Threading: <https://ardupilot.org/dev/docs/learning-ardupilot-threading.html>
- Copter Attitude Control: <https://ardupilot.org/dev/docs/apmcopter-programming-attitude-control-2.html>
- Copter Position Control and Navigation: <https://ardupilot.org/dev/docs/code-overview-copter-poscontrol-and-navigation.html>
- Copter Motors Library: <https://ardupilot.org/dev/docs/code-overview-copter-motors-library.html>
- Geofencing: <https://ardupilot.org/copter/docs/common-geofencing-landing-page.html>

仓库内连续审查记录：

- `docs/dev-algo-review-2026-08-25.md`
- `docs/dev-algo-verify-2026-08-31.md`
- `docs/dev-algo-review-2026-08-31b.md`
- `Tools/eft_issue_repro/README.md`
