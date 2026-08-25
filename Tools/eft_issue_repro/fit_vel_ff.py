#!/usr/bin/env python3
"""从飞行日志拟合 ATC_VFF_RLL / ATC_VFF_PIT。

大桨来流产生的力矩正比于空速，当前只有速率环 I 项在配平它。所以「该前馈多少」
这个问题的答案就写在日志里：把 I 项对机体速度做回归，斜率即增益。

    python3 Tools/eft_issue_repro/fit_vel_ff.py <log.bin> [...]

输出每轴的斜率、相关系数和样本数，并直接给出可照抄的参数行。相关系数低
说明该轴的 I 项主要不是被速度驱动的，此时不要照搬斜率——先查别的原因。

用真机日志时注意：ATT/PID 至少要 10 Hz（bit12），速度取自 XKF1/NKF1。
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
    att, vel, pidr, pidp = [], [], [], []
    while True:
        msg = m.recv_match(type=["ATT", "XKF1", "NKF1", "PIDR", "PIDP"])
        if msg is None:
            break
        t = msg.TimeUS * 1e-6
        mt = msg.get_type()
        if mt == "ATT":
            att.append((t, math.radians(msg.Yaw)))
        elif mt in ("XKF1", "NKF1"):
            # 多核只取 core 0，不同核的状态不该混在一起回归
            if getattr(msg, "C", 0) == 0:
                vel.append((t, msg.VN, msg.VE))
        elif mt == "PIDR":
            pidr.append((t, msg.I))
        elif mt == "PIDP":
            pidp.append((t, msg.I))
    return att, vel, pidr, pidp


def analyse(path):
    att, vel, pidr, pidp = collect(path)
    if not att or not vel:
        return None, "缺少 ATT 或 XKF1/NKF1，无法换算机体速度"
    if not pidr and not pidp:
        return None, "缺少 PIDR/PIDP（LOG_BITMASK 需开 bit12）"

    yaw_series = att
    vn = [(t, a) for t, a, _ in vel]
    ve = [(t, b) for t, _, b in vel]

    samples = {"roll": ([], []), "pitch": ([], [])}
    for axis, pid in (("roll", pidr), ("pitch", pidp)):
        for t, i_term in pid:
            yaw = interp(yaw_series, t)
            n = interp(vn, t)
            e = interp(ve, t)
            if yaw is None or n is None or e is None:
                continue
            # 机体系：x 前，y 右
            vx = n * math.cos(yaw) + e * math.sin(yaw)
            vy = -n * math.sin(yaw) + e * math.cos(yaw)
            if math.hypot(vx, vy) < MIN_SPEED_MS:
                continue
            # 横滚看侧向来流，俯仰看前向来流
            samples[axis][0].append(vy if axis == "roll" else vx)
            samples[axis][1].append(i_term)

    out = {}
    for axis, (xs, ys) in samples.items():
        out[axis] = fit(xs, ys)
    return out, None


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
        rows = {"roll": ("ATC_VFF_RLL", "机体右向速度", "PIDR.I"),
                "pitch": ("ATC_VFF_PIT", "机体前向速度", "PIDP.I")}
        for axis in ("roll", "pitch"):
            name, xdesc, ydesc = rows[axis]
            fitted = res.get(axis)
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
    return 0


if __name__ == "__main__":
    sys.exit(main())
