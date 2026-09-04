#!/usr/bin/env python3
"""从飞行日志拟合 ATC_VFF_RLL / ATC_VFF_PIT。

大桨来流产生的力矩正比于空速，当前只有速率环 I 项在配平它。所以「该前馈多少」
这个问题的答案就写在日志里：把 I 项对机体速度做回归，斜率即增益。

    python3 Tools/eft_issue_repro/fit_vel_ff.py <log.bin> [...]

输出每轴的斜率、相关系数和样本数，并直接给出可照抄的参数行。相关系数低
说明该轴的 I 项主要不是被速度驱动的，此时不要照搬斜率——先查别的原因。

**用有匀速直线段的日志拟合，不要用持续绕圈的日志。** 这个方法的前提是
「I 项已经配平到位」，所以它必须有时间收敛。绕圈时机体系速度方向持续旋转，
I 项在追一个转速与自身带宽同量级的目标、从来没收敛过，回归斜率会系统性偏小。
2026-08-26 在同一架 SITL 机体上实测：反拉日志拟合俯仰 +0.0088（r=0.916），
同一物理模型的绕圈日志只给出 +0.0050（r=0.665）——低估约 43%。

速度来自哪一条 EKF lane
-----------------------
本机默认启用两条 EKF3 lane。此前本工具固定读 core 0，而 ATT.Yaw 和飞控在线
用的速度都来自**当时的 primary core**——primary 不是 0、或飞行中发生 lane
切换时，回归的两侧就不属于同一个估计器，斜率被污染且无从察觉。

现按 XKF4/NKF4 的 PI 字段（primary core index）逐段选取对应 core 的速度，
并剔除切换点附近 LANE_SWITCH_GUARD_S 秒内的样本——切换瞬间两条 lane 的状态
会有阶跃差，而 I 项要过若干个时间常数才跟上，那段数据两侧都不可信。

VFF 消息（若存在）
------------------
固件启用 ATC_VFF_* 后会写 VFF 消息，其中 FF/FR 就是控制器实际用的机体系
速度（取自活动 AHRS、已做风修正与滤波）。有它就不需要上面那套「选 lane +
转机体系」的重建，本工具会自动优先使用并在输出里注明。

但**拟合架次通常没有 VFF**：拟合的前提是前馈关闭、让 I 项独自承担配平，而
增益为零时固件不写该消息。所以 VFF 路径的用途是**留出航段交叉验证**——用
拟合出的增益飞第二架次，再跑一次本工具，看 I 项是否已被前馈接管（斜率应
显著趋近于零）。只用同一架次拟合并验收是不成立的。

速度来源按 VFF > XKF1/NKF1 > PSCN/PSCE 的顺序取。真机现场用的
LOG_BITMASK=145407 并不写 XKF1，所以第三条不是备份而是常态路径：PSCN/PSCE
的 VN/VE 是位置控制器读到的同一个 EKF 输出，用于本拟合等价，但只在位置
控制器运行的模式下记录（LOITER/AUTO/GUIDED 有，STABILIZE/ALT_HOLD 没有），
且不带 lane 信息，切换保护失效。

用真机日志时注意：ATT/PID 至少要 10 Hz（bit12）。
"""

import argparse
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
AP_ROOT = os.path.normpath(os.path.join(HERE, os.pardir, os.pardir))
PYMAVLINK = os.path.join(AP_ROOT, "modules", "mavlink")
if PYMAVLINK not in sys.path:
    sys.path.insert(0, PYMAVLINK)

from pymavlink import DFReader  # noqa: E402

# 只用飞行段。地面上 I 项被复位、速度恒零，会把回归拉向原点。
MIN_SPEED_MS = 0.5

# lane 切换前后各剔除多少秒。取 2 s：切换是状态阶跃，而速率环 I 项的时间常数
# 在零点几秒量级，两三个时间常数后才谈得上重新配平。
LANE_SWITCH_GUARD_S = 2.0


def interp(series, t):
    """在 (time, value) 升序序列上线性插值。超出范围返回 None。"""
    if not series or t < series[0][0] or t > series[-1][0]:
        return None
    lo, hi = 0, len(series) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if series[mid][0] <= t:
            lo = mid
        else:
            hi = mid
    t0, v0 = series[lo]
    t1, v1 = series[hi]
    if t1 == t0:
        return v0
    return v0 + (v1 - v0) * (t - t0) / (t1 - t0)


def step_at(series, t):
    """阶跃量取值：返回 t 时刻之前最后一次记录的值。primary core 是整数索引，
    插值没有意义。"""
    if not series or t < series[0][0]:
        return None
    lo, hi = 0, len(series) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if series[mid][0] <= t:
            lo = mid
        else:
            hi = mid
    return series[hi][1] if series[hi][0] <= t else series[lo][1]


def fit(xs, ys):
    """最小二乘斜率与相关系数。"""
    n = len(xs)
    if n < 20:
        return None
    mx = sum(xs) / n
    my = sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    syy = sum((y - my) ** 2 for y in ys)
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    if sxx <= 0 or syy <= 0:
        return None
    return sxy / sxx, sxy / math.sqrt(sxx * syy), n


def collect(path):
    m = DFReader.DFReader_binary(path)
    att, pidr, pidp, vff = [], [], [], []
    psc_n, psc_e = [], []
    vel_by_core = {}
    primary = []
    while True:
        msg = m.recv_match(type=["ATT", "XKF1", "NKF1", "XKF4", "NKF4",
                                 "PIDR", "PIDP", "VFF", "PSCN", "PSCE"])
        if msg is None:
            break
        t = msg.TimeUS * 1e-6
        mt = msg.get_type()
        if mt == "ATT":
            att.append((t, math.radians(msg.Yaw)))
        elif mt in ("XKF1", "NKF1"):
            vel_by_core.setdefault(getattr(msg, "C", 0), []).append((t, msg.VN, msg.VE))
        elif mt in ("XKF4", "NKF4"):
            # 每个 core 都写一行，PI 是全局的 primary 索引，取哪一行都一样
            pi = getattr(msg, "PI", None)
            if pi is not None and pi >= 0:
                if not primary or primary[-1][1] != pi:
                    primary.append((t, int(pi)))
        elif mt == "PIDR":
            pidr.append((t, msg.I))
        elif mt == "PIDP":
            pidp.append((t, msg.I))
        elif mt == "VFF":
            vff.append((t, msg.FF, msg.FR, msg.Scl))
        elif mt == "PSCN":
            psc_n.append((t, msg.VN))
        elif mt == "PSCE":
            psc_e.append((t, msg.VE))
    return att, vel_by_core, primary, pidr, pidp, vff, psc_n, psc_e


def body_vel_from_ekf(t, att, vel_series_by_core, primary, cores_seen):
    """按 t 时刻的 primary core 取速度，转到机体系。"""
    if primary:
        core = step_at(primary, t)
        if core is None:
            # 早于第一条 XKF4 的样本：沿用最早记录的 primary，而不是丢掉
            core = primary[0][1]
    else:
        core = 0 if (not cores_seen or 0 in cores_seen) else min(cores_seen)
    if core not in vel_series_by_core:
        return None
    vn_s, ve_s = vel_series_by_core[core]
    yaw = interp(att, t)
    n = interp(vn_s, t)
    e = interp(ve_s, t)
    if yaw is None or n is None or e is None:
        return None
    # 机体系：x 前，y 右
    return (n * math.cos(yaw) + e * math.sin(yaw),
            -n * math.sin(yaw) + e * math.cos(yaw))


def analyse(path):
    att, vel_by_core, primary, pidr, pidp, vff, psc_n, psc_e = collect(path)
    if not pidr and not pidp:
        return None, "缺少 PIDR/PIDP（LOG_BITMASK 需开 bit12）"

    info = {"cores": sorted(vel_by_core), "switches": max(0, len(primary) - 1),
            "dropped_switch": 0, "source": None, "vff_faded": 0}

    # 优先用 VFF：那是控制器自己用过的速度，不需要重建
    use_vff = len(vff) >= 20
    if use_vff:
        info["source"] = "VFF"
        vff_ff = [(t, ff) for t, ff, _, _ in vff]
        vff_fr = [(t, fr) for t, _, fr, _ in vff]
        vff_scl = [(t, s) for t, _, _, s in vff]
    else:
        if not att:
            return None, "缺少 ATT，无法把 NED 速度转到机体系"
        if vel_by_core:
            info["source"] = ("XKF1/NKF1 primary lane" if primary
                              else "XKF1/NKF1 (无 PI 字段，退回单 lane)")
            prepared = {c: ([(t, a) for t, a, _ in rows], [(t, b) for t, _, b in rows])
                        for c, rows in vel_by_core.items()}
            switch_times = [t for t, _ in primary[1:]]
        elif len(psc_n) >= 20 and len(psc_e) >= 20:
            # 现场默认的 LOG_BITMASK 不写 XKF1——EKF 那组消息要另外开位。
            # PSCN/PSCE 的 VN/VE 是位置控制器读到的 NED 实测速度，同一个
            # EKF 的输出，只是走了控制器这条路记下来，用于本拟合等价。
            # 代价有二：只有位置控制器在跑的模式才写（LOITER/AUTO/GUIDED 有，
            # STABILIZE/ALT_HOLD 没有），且拿不到 lane 信息，切换保护失效。
            info["source"] = "PSCN/PSCE（位置控制器实测速度，无 lane 信息）"
            info["cores"] = []
            prepared = {0: (psc_n, psc_e)}
            switch_times = []
        else:
            return None, ("缺少 XKF1/NKF1 与 PSCN/PSCE，且日志中无 VFF，"
                          "无法换算机体速度")

    samples = {"roll": ([], []), "pitch": ([], [])}
    for axis, pid in (("roll", pidr), ("pitch", pidp)):
        for t, i_term in pid:
            if use_vff:
                scl = interp(vff_scl, t)
                if scl is None or scl < 0.999:
                    # 落地淡出段：前馈被缩放，I 项也不在稳态配平
                    info["vff_faded"] += 1
                    continue
                vx = interp(vff_ff, t)
                vy = interp(vff_fr, t)
                if vx is None or vy is None:
                    continue
            else:
                if any(abs(t - st) < LANE_SWITCH_GUARD_S for st in switch_times):
                    info["dropped_switch"] += 1
                    continue
                bv = body_vel_from_ekf(t, att, prepared, primary, info["cores"])
                if bv is None:
                    continue
                vx, vy = bv
            if math.hypot(vx, vy) < MIN_SPEED_MS:
                continue
            # 横滚看侧向来流，俯仰看前向来流
            samples[axis][0].append(vy if axis == "roll" else vx)
            samples[axis][1].append(i_term)

    out = {axis: fit(xs, ys) for axis, (xs, ys) in samples.items()}
    return (out, info), None


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("logs", nargs="+")
    ap.add_argument("--min-r", type=float, default=0.5,
                    help="低于此相关系数只提示不给参数行（默认 0.5）")
    args = ap.parse_args(argv)

    for path in args.logs:
        print("=" * 68)
        print(os.path.basename(path))
        res, err = analyse(path)
        if err:
            print("  跳过:", err)
            continue
        out, info = res

        print("  速度来源: %s" % info["source"])
        if info["source"].startswith("PSCN"):
            print("            日志没有 XKF1/NKF1，退回位置控制器实测速度；"
                  "该消息只在位置控制器运行的模式下记录")
        elif info["source"] != "VFF":
            print("  EKF lane: 见到 core %s；primary 切换 %d 次"
                  % (info["cores"], info["switches"]))
            if info["switches"]:
                print("            切换点 ±%.1f s 内剔除 %d 个样本"
                      % (LANE_SWITCH_GUARD_S, info["dropped_switch"]))
        elif info["vff_faded"]:
            print("            落地淡出段剔除 %d 个样本" % info["vff_faded"])

        rows = {"roll": ("ATC_VFF_RLL", "机体右向速度", "PIDR.I"),
                "pitch": ("ATC_VFF_PIT", "机体前向速度", "PIDP.I")}
        for axis in ("roll", "pitch"):
            name, xdesc, ydesc = rows[axis]
            fitted = out.get(axis)
            if fitted is None:
                print("  %-12s 样本不足，跳过" % axis)
                continue
            slope, r, n = fitted
            print("  %-12s %s vs %s：斜率 %+.5f /(m/s)，r = %+.3f，n = %d"
                  % (axis, ydesc, xdesc, slope, r, n))
            if abs(r) < args.min_r:
                print("               相关系数偏低，该轴 I 项主要不是速度驱动的，先别照搬")
            else:
                print("               %s %.4f" % (name, slope))

        if info["source"] == "VFF":
            print("  注：本架次前馈是开着的，I 项已被它接管一部分，此处的斜率是"
                  "**残余**而非应设增益。")
            print("      交叉验证的判据是斜率显著趋近于零；若仍与拟合值同量级，"
                  "说明前馈没起作用。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
