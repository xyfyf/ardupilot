# EFT 植保六旋翼问题复现

这套工具固定使用 `HEXA / DJI_X`（`FRAME_CLASS=2`、`FRAME_TYPE=13`、六路电机），复现两个场景：

1. 地面站上传 AUTO 任务：起飞 → 两个航点 → `NAV_LAND`，观察恒速触地与落地检测延迟。
2. LOITER 下打杆到 5 m/s，突然反向打满杆，观察姿态误差和速率环 I 项换向。

## 运行

```bash
./waf configure --board sitl
./waf copter

python3 Tools/eft_issue_repro/reproduce.py landing
python3 Tools/eft_issue_repro/reproduce.py reverse

# 关闭新增物理项，跑相同动作作基线 A/B
python3 Tools/eft_issue_repro/reproduce.py landing --baseline
python3 Tools/eft_issue_repro/reproduce.py reverse --baseline
```

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

所有新字段默认值为零，已有 SITL 机型行为不变。`eft_hexa.json` 当前把速度力矩校准到 5 m/s 时 I 项约 ±0.05，与 00000231 真机日志同量级。

## 当前边界

质量 50 kg、惯量、2 m 轴距和 4.71 m² 总桨盘面积是首轮占位值，不是实机辨识结果。当前可以复现控制链和问题量级，但不能宣称逐点等同真机。默认 SITL 地面仍是刚性接触，因此能复现约 0.5 m/s 触地和约 1.8 s 检测延迟，不能复现真机起落架弹性、3 g 冲击波形或触地后 271–347 PWM 的电机极差；这些需下一步加入柔性接地模型或 Gazebo 接触模型。
