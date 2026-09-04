"""读任务文件与参数文件，并把参数翻成 AC_ArcNav 真正会用到的那五个量。

这里的每一步换算都对应飞控里的一行代码，注释给出出处。校验报告要求"每一项都能
追到是哪个约束、用的哪个参数值"，而参数到约束之间隔着好几层换算与单位转换，
不写出处的话，报告里那个数字对现场是不可复核的。
"""

import math
import os
from dataclasses import dataclass, field

from arcnav import Limits, HEADING_LEAD_S

MAV_CMD_NAV_WAYPOINT = 16
MAV_CMD_NAV_LOITER_TURNS = 18
MAV_CMD_NAV_SPLINE_WAYPOINT = 82
MAV_CMD_NAV_TAKEOFF = 22
MAV_CMD_NAV_LAND = 21
MAV_CMD_NAV_RETURN_TO_LAUNCH = 20

# 会产生一个空间位置、因而构成航段端点的命令。非 NAV 命令（改速度、开喷等）
# 不占位置，遍历时要跳过——否则"前一段航段"会被算成零长。
NAV_CMDS = {
    MAV_CMD_NAV_WAYPOINT, MAV_CMD_NAV_LOITER_TURNS, MAV_CMD_NAV_SPLINE_WAYPOINT,
    MAV_CMD_NAV_TAKEOFF, MAV_CMD_NAV_LAND, MAV_CMD_NAV_RETURN_TO_LAUNCH,
}

CMD_NAMES = {
    MAV_CMD_NAV_WAYPOINT: "NAV_WAYPOINT",
    MAV_CMD_NAV_LOITER_TURNS: "NAV_LOITER_TURNS",
    MAV_CMD_NAV_SPLINE_WAYPOINT: "NAV_SPLINE_WAYPOINT",
    MAV_CMD_NAV_TAKEOFF: "NAV_TAKEOFF",
    MAV_CMD_NAV_LAND: "NAV_LAND",
    MAV_CMD_NAV_RETURN_TO_LAUNCH: "NAV_RTL",
}


class InputError(Exception):
    pass


# --------------------------------------------------------------------------
# 任务文件
# --------------------------------------------------------------------------

@dataclass
class MissionItem:
    seq: int
    frame: int
    command: int
    params: tuple      # param1..param4
    x: float           # 纬度（度）
    y: float           # 经度（度）
    z: float           # 高度（米）

    @property
    def name(self):
        return CMD_NAMES.get(self.command, "CMD_%d" % self.command)

    @property
    def is_nav(self):
        return self.command in NAV_CMDS


def load_mission(path):
    """读 QGC WPL 110（`.waypoints`，也是 MAVProxy `wp save` 的输出格式）。

    不支持 QGroundControl 的 `.plan`（JSON）。现场派工用的是前者，后者要另写解析，
    没有就明说，不要装作读懂了——读错一个字段，整份报告就是在校验另一条航线。
    """
    if not os.path.exists(path):
        raise InputError("任务文件不存在：%s" % path)
    with open(path) as fh:
        lines = [ln.rstrip("\n") for ln in fh]
    if not lines or not lines[0].startswith("QGC WPL"):
        raise InputError("不是 QGC WPL 格式（首行应为 'QGC WPL 110'）：%s" % path)

    items = []
    for lineno, ln in enumerate(lines[1:], start=2):
        if not ln.strip() or ln.lstrip().startswith("#"):
            continue
        f = ln.split("\t") if "\t" in ln else ln.split()
        if len(f) < 12:
            raise InputError("%s 第 %d 行只有 %d 列，应为 12 列" % (path, lineno, len(f)))
        try:
            items.append(MissionItem(
                seq=int(f[0]), frame=int(f[2]), command=int(f[3]),
                params=(float(f[4]), float(f[5]), float(f[6]), float(f[7])),
                x=float(f[8]), y=float(f[9]), z=float(f[10])))
        except ValueError as exc:
            raise InputError("%s 第 %d 行解析失败：%s" % (path, lineno, exc))
    if not items:
        raise InputError("任务文件里没有任何航点：%s" % path)
    return items


def loiter_turns_radius_m(item):
    """把 param3 还原成飞控真正会用的半径，含 AP_Mission 的量化。

    镜像 AP_Mission.cpp:1122-1130 存、mode_auto.cpp:1877-1881 取，两步合起来：

        半径 ≤ 255 m：uint8_t radius_m = abs_radius   ← C 的浮点转整数是**截断**
        半径 > 255 m：先除以 10 再截断，取用时乘回 10

    这一步必须复现，否则校验的是一个飞机不会用的半径。任务里写 2.5 m，机上是
    2 m——更紧的弯、更高的偏航需求，按 2.5 校验就会偏乐观。

    返回 (生效半径, 计划半径, 是否被量化改变, 是否逆时针)。
    """
    planned = abs(item.params[2])
    ccw = item.params[2] < 0
    if planned <= 255:
        effective = float(int(planned))          # uint8_t 截断
    else:
        effective = float(min(255, int(planned * 0.1))) * 10.0
    return effective, planned, effective != planned, ccw


def loiter_turns_sweep_rad(item):
    """圈数 → 扫掠角。param1 是圈数，可小于 1（AP_Mission 用 ×256 的位存）。

    U 型掉头是半圈，param1 = 0.5，扫掠 π。整圈及以上则是绕圈，扫掠 2π 的倍数。
    """
    turns = abs(item.params[0])
    return turns * 2.0 * math.pi


# --------------------------------------------------------------------------
# 参数文件
# --------------------------------------------------------------------------

def load_params(paths):
    """读 `.param` / `.parm`。支持 `名 值`、`名,值`，以及 QGC 的 5 列格式。

    可给多份，**后者覆盖前者**。现场导出通常是一份，但 SITL 一次运行的生效参数
    是分层叠加的（copter.parm 默认 + 机型 parm + 场景 --set 覆盖），只读其中一层
    会按错误的参数值判定，而报告看起来完全正常。
    """
    if isinstance(paths, str):
        paths = [paths]
    params = {}
    for path in paths:
        _load_params_into(path, params)
    if not params:
        raise InputError("参数文件里没解析出任何参数：%s" % ", ".join(paths))
    return params


def _load_params_into(path, params):
    if not os.path.exists(path):
        raise InputError("参数文件不存在：%s" % path)
    with open(path) as fh:
        for ln in fh:
            ln = ln.split("#")[0].strip()
            if not ln:
                continue
            f = ln.replace(",", " ").split()
            if len(f) == 2:
                name, val = f
            elif len(f) >= 5 and f[0].isdigit() and f[1].isdigit():
                name, val = f[2], f[3]          # QGC: sysid compid 名 值 类型
            else:
                continue
            try:
                params[name.upper()] = float(val)
            except ValueError:
                continue


@dataclass
class Provenance:
    """每个量的出处：值、单位、来自哪个参数、参数在不在文件里。

    报告靠它回答"这个数哪来的"。缺参数时走默认值是常态而不是异常——现场导出的
    参数文件只含改过的项——但**走了默认值必须说出来**，否则报告读起来像是按现场
    配置算的，实际按固件默认算的。
    """
    value: float
    unit: str
    source: str
    present: bool
    note: str = ""


@dataclass
class ResolvedLimits:
    limits: Limits
    provenance: dict = field(default_factory=dict)
    warnings: list = field(default_factory=list)


def resolve_limits(params, yaw_rate_max_degs, yaw_rate_source,
                   yaw_accel_max_degss=None):
    """参数 + 实测偏航能力 → AC_ArcNav::Limits。

    `yaw_rate_max_degs` 没有默认值，必须由调用方显式给。这是 P06 空洞 D 的直接
    对策：飞控这条链取的是 `ATC_SLEW_YAW`（见 mode_auto.cpp:1882），那是**允许
    值**不是**能力值**，参数设得高于机体真实能力时，判定就会放行飞不出来的弯。
    地面工具不能重复这个错误，所以它拒绝从参数里推这个数。
    """
    prov = {}
    warnings = []

    def get(name, default, unit, note=""):
        present = name in params
        val = params.get(name, default)
        prov[name] = Provenance(val, unit, name, present,
                                note or ("" if present else "参数文件中没有，用固件默认值"))
        return val

    # 倾角上限：AC_PosControl::get_lean_angle_max_rad()（AC_PosControl.cpp:1190）
    #   PSC_ANGLE_MAX > 0 时用它（单位：度），否则回退到 ANGLE_MAX（单位：厘度）
    psc_angle_max_deg = get("PSC_ANGLE_MAX", 0.0, "deg")
    angle_max_cd = get("ANGLE_MAX", 3000.0, "cdeg")
    if psc_angle_max_deg > 0:
        lean_rad = math.radians(psc_angle_max_deg)
        lean_src = "PSC_ANGLE_MAX=%.4g deg" % psc_angle_max_deg
    else:
        lean_rad = math.radians(angle_max_cd / 100.0)
        lean_src = "ANGLE_MAX=%.4g cdeg（PSC_ANGLE_MAX 为 0，回退）" % angle_max_cd
    prov["倾角上限"] = Provenance(math.degrees(lean_rad), "deg", lean_src, True,
                                  "转弯只准用其中 70%（AC_ARCNAV_TILT_FRACTION）")

    # 过渡长度用的 jerk：AC_PosControl::get_shaping_jerk_NE_msss() 直接返回
    # PSC_JERK_XY（AC_PosControl.cpp:315）。全仓库没有任何地方调
    # set_shaping_jerk_NE_msss()，所以 WPNAV_JERK 不参与——这一条容易搞错。
    jerk = get("PSC_JERK_XY", 5.0, "m/s³",
               note="过渡长度用的是 PSC_JERK_XY，不是 WPNAV_JERK")
    if "WPNAV_JERK" in params and "PSC_JERK_XY" not in params:
        warnings.append(
            "参数文件里有 WPNAV_JERK=%.4g 但没有 PSC_JERK_XY。转弯过渡长度取的是 "
            "PSC_JERK_XY（默认 5.0），WPNAV_JERK 不影响它——若本意是调转弯，改错了参数。"
            % params["WPNAV_JERK"])

    # 作业速度：mode_auto.cpp:1881 取 wp_nav->get_default_speed_NE_ms()，即 WPNAV_SPEED
    speed_cms = get("WPNAV_SPEED", 1000.0, "cm/s")
    prov["作业速度"] = Provenance(speed_cms / 100.0, "m/s",
                                  "WPNAV_SPEED=%.4g cm/s" % speed_cms,
                                  "WPNAV_SPEED" in params)

    # 偏航角加速度：ATC_ACCEL_Y_MAX（厘度/s²），AC_AttitudeControl.h:112
    accel_y_cdss = get("ATC_ACCEL_Y_MAX", 27000.0, "cdeg/s²")
    if yaw_accel_max_degss is not None:
        yaw_accel_radss = math.radians(yaw_accel_max_degss)
        prov["偏航角加速度上限"] = Provenance(
            yaw_accel_max_degss, "deg/s²", "命令行显式给定", True, "覆盖 ATC_ACCEL_Y_MAX")
    else:
        yaw_accel_radss = math.radians(accel_y_cdss / 100.0)
        prov["偏航角加速度上限"] = Provenance(
            accel_y_cdss / 100.0, "deg/s²", "ATC_ACCEL_Y_MAX=%.5g cdeg/s²" % accel_y_cdss,
            "ATC_ACCEL_Y_MAX" in params)

    # 偏航速率上限 —— 这里与飞控**故意不同**。
    yaw_rate_rads = math.radians(yaw_rate_max_degs)
    prov["偏航速率能力"] = Provenance(
        yaw_rate_max_degs, "deg/s", yaw_rate_source, True,
        "机体实测能力；转弯只准用其中 50%（AC_ARCNAV_YAW_RATE_FRACTION）")

    # 把飞控当前会用的那个值也算出来，用于对照——这正是空洞 D 的可见化。
    slew_yaw_cds = get("ATC_SLEW_YAW", 6000.0, "cdeg/s")
    rate_y_max_degs = get("ATC_RATE_Y_MAX", 0.0, "deg/s")
    if rate_y_max_degs > 0:
        fc_rate_degs = min(rate_y_max_degs, slew_yaw_cds / 100.0)
        fc_src = "min(ATC_RATE_Y_MAX=%.4g, ATC_SLEW_YAW=%.4g cdeg/s)" % (
            rate_y_max_degs, slew_yaw_cds)
    else:
        fc_rate_degs = slew_yaw_cds / 100.0
        fc_src = "ATC_SLEW_YAW=%.4g cdeg/s（ATC_RATE_Y_MAX 为 0）" % slew_yaw_cds
    prov["飞控在用的偏航上限"] = Provenance(
        fc_rate_degs, "deg/s", fc_src, True,
        "AC_AttitudeControl.cpp:182 get_slew_yaw_max_rads()，这是**允许值**不是能力值")

    if fc_rate_degs > yaw_rate_max_degs:
        warnings.append(
            "机上偏航上限 %.4g °/s 高于机体实测能力 %.4g °/s（差 %.0f%%）。飞行中的判定"
            "（mode_auto.cpp:1882）用的是前者，因此机上会放行本报告判为不可行的弯——"
            "这就是空洞 D。本报告按实测能力判，两边不一致时以本报告为准。"
            % (fc_rate_degs, yaw_rate_max_degs,
               100.0 * (fc_rate_degs - yaw_rate_max_degs) / yaw_rate_max_degs))

    prov["航向前视时间"] = Provenance(
        HEADING_LEAD_S, "s", "AC_ARCNAV_HEADING_LEAD_S（编译期常数，现场参数改不了）",
        True)

    lim = Limits(lean_angle_max_rad=lean_rad, jerk_ne_msss=jerk,
                 yaw_rate_max_rads=yaw_rate_rads,
                 yaw_accel_max_radss=yaw_accel_radss,
                 heading_lead_s=HEADING_LEAD_S)
    return ResolvedLimits(lim, prov, warnings), speed_cms / 100.0


# --------------------------------------------------------------------------
# 几何
# --------------------------------------------------------------------------

EARTH_RADIUS_M = 6378137.0


def to_local_ne(items, origin=None):
    """经纬度 → 以首点为原点的局部 NE 米。

    等距圆柱投影。作业地块跨度是百米量级，这个尺度上它与更严谨的投影差在毫米，
    而航段长度与行距的判定只需要米级——用更复杂的投影不会改变任何一条结论。
    """
    if origin is None:
        origin = (items[0].x, items[0].y)
    lat0, lon0 = origin
    coslat = math.cos(math.radians(lat0))
    out = []
    for it in items:
        n = math.radians(it.x - lat0) * EARTH_RADIUS_M
        e = math.radians(it.y - lon0) * EARTH_RADIUS_M * coslat
        out.append((n, e))
    return out


def dist(a, b):
    return math.hypot(b[0] - a[0], b[1] - a[1])
