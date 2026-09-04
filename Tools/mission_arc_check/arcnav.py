"""AC_ArcNav 可飞性判定的地面镜像。

这是同一套数学的**第二份实现**，因此有两条纪律：

1. **逐行对着 C++ 写。** 每个函数下面都注明它镜像的是 `libraries/AC_WPNav/
   AC_ArcNav.cpp` 的哪一段。不做等价变形、不合并常数、不"顺手化简"——化简过的
   式子在浮点下不等价，而不等价的地方正好是判定翻转的地方。
2. **用 float32 仿真。** 飞控里全程是 `float`，x86-64 上 SSE 的 float 运算就是真
   32 位。这里每一步运算后 `f32()` 一次，与 C++ 逐步同精度。不这么做的话，双精度
   算出来的边界与飞控的边界差一点点，而"差一点点"就是可飞与不可飞的分界。

漂移由 `tests/test_crosscheck.py` 挡：它现场构建并运行 gtest
`libraries/AC_WPNav/tests/test_arcnav_feasibility.cpp --dump-table`，拿 C++ 的输出
和本文件比。不比对仓库里的表——表一旦落盘就会在 C++ 改动后静默过期，那种"看起来
有外部基准"的通过比没有基准更坏。
"""

import math
import struct
from dataclasses import dataclass

# --- 与 C++ 编译期常数一一对应 -------------------------------------------------
# 改这里之前先确认那边也改了；对不上时对拍会失败，那正是它存在的意义。
GRAVITY_MSS = 9.80665                   # AP_Math/definitions.h:45
FLT_EPSILON = 1.1920928955078125e-07    # <cfloat>

TILT_FRACTION = 0.7                     # AC_ARCNAV_TILT_FRACTION
YAW_RATE_FRACTION = 0.5                 # AC_ARCNAV_YAW_RATE_FRACTION
SMOOTHSTEP_PEAK = 1.5                   # AC_ARCNAV_SMOOTHSTEP_PEAK
HEADING_LEAD_S = 0.5                    # AC_ARCNAV_HEADING_LEAD_S

# 这四个是编译期常数而非参数：现场改参数文件动不了它们，报告里据此说明来源。
COMPILE_TIME_CONSTANTS = {
    "AC_ARCNAV_TILT_FRACTION": TILT_FRACTION,
    "AC_ARCNAV_YAW_RATE_FRACTION": YAW_RATE_FRACTION,
    "AC_ARCNAV_SMOOTHSTEP_PEAK": SMOOTHSTEP_PEAK,
    "AC_ARCNAV_HEADING_LEAD_S": HEADING_LEAD_S,
}


def f32(x):
    """把双精度中间结果舍成 float32，模拟飞控里的单精度运算。"""
    return struct.unpack("<f", struct.pack("<f", x))[0]


def is_positive(x):
    """AP_Math.h:65 —— 注意判据是 >= FLT_EPSILON 而不是 > 0。

    差别不是学究：偏航上限设成 1e-9 时飞控认为"未设置、不检查"，直接放行。
    地面工具若按 > 0 判，就会去检查一条飞控根本不检查的约束，两边结论不一致。
    """
    return f32(x) >= FLT_EPSILON


def safe_sqrt(x):
    """AP_Math.cpp 的 safe_sqrt：负数与 NaN 一律给 0，不抛异常。"""
    ret = math.sqrt(x) if x >= 0.0 else float("nan")
    if math.isnan(ret):
        return 0.0
    return f32(ret)


def constrain_float(v, lo, hi):
    if math.isnan(v):
        return (lo + hi) * 0.5
    return lo if v < lo else (hi if v > hi else v)


@dataclass
class Limits:
    """AC_ArcNav::Limits —— 判定实际读到的全部量，五个标量。

    yaw_rate_max_rads 是**机体实测可达偏航速率**，不是 ATC_SLEW_YAW。
    这是 P06 六个空洞里 D 的直接对策，所以本类不给它默认值：调用方必须显式给。
    """
    lean_angle_max_rad: float
    jerk_ne_msss: float
    yaw_rate_max_rads: float
    yaw_accel_max_radss: float
    heading_lead_s: float = HEADING_LEAD_S


def tilt_budget_rad(lim):
    """镜像 AC_ArcNav.cpp `tilt_budget_rad`。"""
    return f32(lim.lean_angle_max_rad * constrain_float(TILT_FRACTION, 0.1, 1.0))


def max_speed_for_radius_ms(lim, radius_m):
    """镜像 AC_ArcNav.cpp `max_speed_for_radius_ms`。

    a = g·tan(倾角预算)，而维持圆周需要 a = v²/r。
    """
    if not is_positive(radius_m):
        return 0.0
    t = f32(math.tan(tilt_budget_rad(lim)))
    return safe_sqrt(f32(f32(GRAVITY_MSS * t) * radius_m))


def min_radius_for_speed_m(lim, speed_ms):
    """镜像 AC_ArcNav.cpp `min_radius_for_speed_m`。"""
    a_max = f32(GRAVITY_MSS * f32(math.tan(tilt_budget_rad(lim))))
    if not is_positive(a_max):
        return 0.0
    return f32(f32(speed_ms * speed_ms) / a_max)


def required_spiral_len_m(lim, radius_m, speed_ms):
    """镜像 AC_ArcNav.cpp `required_spiral_len_m`。

    过渡长度取三个下界的最大值，三者分别来自位置支路、航向前视、航向支路：

      v³/(r·jerk)   横向加加速度不超限
      v·τ           不短于航向前视距离，否则前视点跨过过渡段、偏航指令第一帧就满速率
      二次方程正根   指令偏航角加速度不超限，含前视淡入带来的 (1 + lead'(s)) 因子

    第三项那个二次方程不能省成 k·v²/(r·ψ̈)。少了 lead'(s) 这个因子会把过渡定短，
    而定短的过渡正是"本来能跟上、变成跟不上"的那种。
    """
    if not is_positive(radius_m) or not is_positive(speed_ms):
        return 0.0

    jerk = lim.jerk_ne_msss
    spiral_len_m = 0.0
    if is_positive(jerk):
        spiral_len_m = f32(f32(f32(speed_ms * speed_ms) * speed_ms) / f32(radius_m * jerk))

    spiral_len_m = max(spiral_len_m, f32(speed_ms * lim.heading_lead_s))

    if is_positive(lim.yaw_accel_max_radss):
        a = f32(f32(SMOOTHSTEP_PEAK * f32(speed_ms * speed_ms))
                / f32(radius_m * lim.yaw_accel_max_radss))
        lead_m = f32(speed_ms * lim.heading_lead_s)
        min_len_m = f32(0.5 * f32(a + safe_sqrt(f32(f32(a * a) + f32(f32(4.0 * a) * lead_m)))))
        spiral_len_m = max(spiral_len_m, min_len_m)

    return spiral_len_m


def plan_feasible(lim, radius_m, speed_ms):
    """镜像 AC_ArcNav.cpp `plan_feasible`。

    返回 (可行, lead_in_m, 拒绝原因)。C++ 只返回 bool，原因在那边靠三个 return
    的先后顺序隐含着；地面报告必须说出"差在哪"，所以这里把原因显式带出来。
    先后顺序与 C++ 完全一致——倾角先判、偏航后判——否则同一个两项都超的弯，
    两边报出来的主因会不一样。
    """
    if not is_positive(radius_m) or not is_positive(speed_ms):
        return False, 0.0, "degenerate"

    if speed_ms > max_speed_for_radius_ms(lim, radius_m):
        return False, 0.0, "lean"

    if is_positive(lim.yaw_rate_max_rads):
        if f32(speed_ms / radius_m) > f32(lim.yaw_rate_max_rads * YAW_RATE_FRACTION):
            return False, 0.0, "yaw_rate"

    lead_in_m = f32(required_spiral_len_m(lim, radius_m, speed_ms)
                    + f32(speed_ms * lim.heading_lead_s))
    return True, lead_in_m, None


@dataclass
class UTurnPlan:
    radius_m: float
    spiral_len_m: float
    duration_s: float
    peak_yaw_rate_rads: float
    rate_limited: bool


def plan_from_yaw_capability(speed_ms, sweep_rad, yaw_rate_max_rads, yaw_accel_max_radss):
    """镜像 AC_ArcNav.cpp `plan_from_yaw_capability`。

    从偏航能力反推掉头，方向与"从行距定半径"相反——因为偏航能力是硬约束，
    行距是软约束（可跳行、可外扩）。报告里"改成什么可行"的半径由它给。
    """
    plan = UTurnPlan(0.0, 0.0, 0.0, 0.0, False)
    sweep_rad = abs(sweep_rad)
    if not (is_positive(speed_ms) and is_positive(sweep_rad)
            and is_positive(yaw_rate_max_rads) and is_positive(yaw_accel_max_radss)):
        return plan

    ramp_sweep_rad = f32(f32(SMOOTHSTEP_PEAK * f32(yaw_rate_max_rads * yaw_rate_max_rads))
                         / yaw_accel_max_radss)
    plan.rate_limited = ramp_sweep_rad <= sweep_rad

    if plan.rate_limited:
        plan.peak_yaw_rate_rads = yaw_rate_max_rads
        plan.duration_s = f32(f32(f32(SMOOTHSTEP_PEAK * yaw_rate_max_rads) / yaw_accel_max_radss)
                              + f32(sweep_rad / yaw_rate_max_rads))
    else:
        plan.peak_yaw_rate_rads = safe_sqrt(f32(f32(sweep_rad * yaw_accel_max_radss)
                                                / SMOOTHSTEP_PEAK))
        plan.duration_s = f32(f32(2.0 * f32(SMOOTHSTEP_PEAK * plan.peak_yaw_rate_rads))
                              / yaw_accel_max_radss)

    plan.radius_m = f32(speed_ms / plan.peak_yaw_rate_rads)
    plan.spiral_len_m = f32(f32(SMOOTHSTEP_PEAK * f32(speed_ms * plan.peak_yaw_rate_rads))
                            / yaw_accel_max_radss)
    return plan
