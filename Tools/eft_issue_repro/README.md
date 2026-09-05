# EFT 植保六旋翼问题复现

这套工具固定使用 `HEXA / DJI_X`（`FRAME_CLASS=2`、`FRAME_TYPE=13`、六路电机）。

## 场景

`reproduce.py <场景>` 共 14 个，按挂靠的问题编号排列：

| 场景 | 问题 | 复现什么 |
| :-- | :-- | :-- |
| `landing` | P01 | AUTO 任务起飞 → 两航点 → `NAV_LAND`，看恒速触地与落地检测延迟 |
| `reverse` | P02 | 高速运动中突然反拉，看姿态误差与速率环 I 项换向（LOITER 5 m/s 为代表点） |
| `mag-align` | P03 | 磁罗盘偏航未对准的辨识 |
| `motor-fail` | P04 | 单电机停转 / 掉桨的检测与降级重分配 |
| `uturn-auto` | P04 P06 | AUTO 掉头；同时用作 P04 检测器的**误报**场景 |
| `route` | P05 | 围栏下的 AUTO 航线与刹车动作 |
| `fence` | P05 | 满杆冲栏，量各模式的实际围控能力 |
| `fence-sprint` | P05 | 同上，冲刺工况（姿态层围栏） |
| `circle` | P06 | 自动绕圈，倾角饱和护栏 |
| `yaw-step` | P06 | 偏航阶跃，偏航能力辨识 |
| `uturn-arcnav` | P06 | 用 `AC_ArcNav` 做协调转弯 |
| `uturn-guided` | P06 | 同上，GUIDED 接口路径 |
| `uturn` | P06 | 掉头基线（未进集中回归） |
| `loiter-circle` | P07 | 手动绕圈，小半径抽动 |

`uturn` 与 `uturn-guided` 目前没有 `regression.py` 条目——手动跑可以，但**不受回归保护**。

## 运行

```bash
./waf configure --board sitl
./waf copter

python3 Tools/eft_issue_repro/reproduce.py landing
python3 Tools/eft_issue_repro/reproduce.py motor-fail --motor 6 --detect

# 关闭新增物理项，跑相同动作作基线 A/B
python3 Tools/eft_issue_repro/reproduce.py landing --baseline
```

单个场景是调试用的。**判断有没有改坏东西请跑集中回归**，它按问题编号组织全部条目：

```bash
python3 Tools/eft_issue_repro/regression.py                 # 全部
python3 Tools/eft_issue_repro/regression.py --only P04      # 只跑一个问题
python3 Tools/eft_issue_repro/regression.py --list          # 列出条目不执行
```

`reproduce.py` 用 `/tmp/ardupilot-eft-issue-repro-sitl.lock` 互斥，同机第二个实例会
直接退出并报出占用者 PID——各场景端口固定，撞了就不是算法问题了。绕过它直接跑
`build/sitl/bin/arducopter` 时这把锁不生效。

渲染视频：

```bash
python3 Tools/eft_issue_repro/render_video.py landing \
  <coupled-result.json> <baseline-result.json> --output <目录>

python3 Tools/eft_issue_repro/render_video.py reverse \
  <coupled-result.json> <baseline-result.json> --output <目录>
```

## 模型里补了什么

ArduPilot 默认多旋翼模型没有大桨来流引起的速度相关力矩，也没有近地气垫建立/溃散。`SIM_Frame` 的自定义 JSON 新增了默认关闭的字段：

- `velocity_torque_gain`：横向速度→滚转力矩、前向速度→俯仰力矩；
- `ground_effect_height`、`ground_effect_collapse_height`、`ground_effect_gain`、`ground_effect_vspeed_gain`：近地增升及接触前溃散。

所有新字段默认值为零，已有 SITL 机型行为不变。`eft_hexa.json` 当前把速度力矩校准到 5 m/s 代表工况下 I 项约 ±0.05，与 00000231 真机日志同量级。后续需要在多个速度、载荷、反拉幅度和方向上验证触发边界。

## 当前边界

质量 50 kg、惯量、2 m 轴距和 4.71 m² 总桨盘面积是首轮占位值，不是实机辨识结果。当前可以复现控制链和问题量级，但不能宣称逐点等同真机。默认 SITL 地面仍是刚性接触，因此能复现约 0.5 m/s 触地和约 1.8 s 检测延迟，不能复现真机起落架弹性、3 g 冲击波形或触地后 271–347 PWM 的电机极差；这些需下一步加入柔性接地模型或 Gazebo 接触模型。
