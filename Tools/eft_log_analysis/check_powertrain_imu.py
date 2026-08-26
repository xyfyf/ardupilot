#!/usr/bin/env python3
"""验证立项书提出的替代根因：IMU 失真 + 外部动力系统不一致性。

    python3 Tools/eft_log_analysis/check_powertrain_imu.py <log.bin> [...]

立项书对教练机问题的判断是「大机动飞行抽动和下降砸地，主要原因大概率集中在
IMU 失真、外部动力系统不一致性」。这与当前 P01/P02 的根因结论（近地气动托举、
速度相关气动力矩 + 速率环 I 项追不上）不同，属于**尚未排除的替代根因**，
必须显式验证而不是默认否定。

本脚本只用既有日志，不需要新的现场数据，检查两件事：

动力系统一致性
    六台电机在相同油门下是否给出相同转速与电流。用 ESC 遥测逐台统计，
    离散度大说明电机/桨/电调之间存在稳态差异，混控要靠常态补偿来抵消——
    那正是「动力系统不一致性」的可测形态。另看 RCOU 六路指令本身的不对称，
    它反映的是控制器为了配平已经付出了多少。

IMU 失真
    VIBE 的振动量与削顶计数；多台 IMU 之间的加速度分歧，并按机动强度分档。
    「大机动时失真」这一说法的可证伪形式是：高角速率段的 IMU 间分歧显著大于
    低角速率段。若两档相当，则该说法在本份日志上不成立。

判据是相对的，不是绝对阈值——横向比较六台电机之间、两台 IMU 之间、以及
高低机动两档之间的差异。
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

MOTORS = 6
FLYING_THROTTLE = 0.15      # MOTB.ThrOut 低于此值视为地面段
LOW_RATE_DPS = 20.0         # 低机动档上限
HIGH_RATE_DPS = 60.0        # 高机动档下限


def stats(xs):
    if not xs:
        return None
    n = len(xs)
    mean = sum(xs) / n
    var = sum((x - mean) ** 2 for x in xs) / n
    return mean, math.sqrt(var), min(xs), max(xs)


def interp(series, t):
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
    return v0 if t1 == t0 else v0 + (v1 - v0) * (t - t0) / (t1 - t0)


def collect(path):
    m = DFReader.DFReader_binary(path)
    thr = []                       # (t, ThrOut)
    rcou = []                      # (t, [C1..C6])
    esc = {}                       # instance -> [(t, rpm, curr)]
    vibe = {}                      # imu -> [(t, vx, vy, vz, clip)]
    imu = {}                       # instance -> [(t, ax, ay, az, gyro_mag_dps)]
    while True:
        msg = m.recv_match(type=["MOTB", "RCOU", "ESC", "VIBE", "IMU"])
        if msg is None:
            break
        t = msg.TimeUS * 1e-6
        mt = msg.get_type()
        if mt == "MOTB":
            thr.append((t, msg.ThrOut))
        elif mt == "RCOU":
            rcou.append((t, [getattr(msg, "C%d" % i) for i in range(1, MOTORS + 1)]))
        elif mt == "ESC":
            esc.setdefault(int(msg.Instance), []).append(
                (t, float(msg.RPM), float(msg.Curr)))
        elif mt == "VIBE":
            vibe.setdefault(int(msg.IMU), []).append(
                (t, msg.VibeX, msg.VibeY, msg.VibeZ, int(msg.Clip)))
        elif mt == "IMU":
            g = math.degrees(math.sqrt(msg.GyrX ** 2 + msg.GyrY ** 2 + msg.GyrZ ** 2))
            imu.setdefault(int(msg.I), []).append(
                (t, msg.AccX, msg.AccY, msg.AccZ, g))
    return thr, rcou, esc, vibe, imu


def flying_window(thr):
    """飞行段的起止时间。用推力输出判定，排除地面。"""
    flying = [t for t, v in thr if v >= FLYING_THROTTLE]
    return (flying[0], flying[-1]) if flying else (None, None)


def report_powertrain(esc, rcou, t0, t1):
    print("  动力系统一致性")
    if not esc:
        print("    ESC 遥测缺失，无法判定")
    else:
        rows = []
        for inst in sorted(esc):
            seg = [(r, c) for t, r, c in esc[inst] if t0 <= t <= t1 and r > 0]
            if not seg:
                continue
            rs = stats([r for r, _ in seg])
            cs = stats([c for _, c in seg])
            rows.append((inst, rs, cs, len(seg)))
        if not rows:
            print("    飞行段内无有效 ESC 样本")
        else:
            print("    %-6s %-12s %-12s %-8s" % ("电机", "转速均值", "电流均值", "样本"))
            for inst, rs, cs, n in rows:
                print("    %-7d %-13.0f %-13.2f %d" % (inst + 1, rs[0], cs[0], n))
            rmeans = [r[1][0] for r in rows]
            cmeans = [r[2][0] for r in rows]
            rm = sum(rmeans) / len(rmeans)
            cm = sum(cmeans) / len(cmeans)
            print("    转速离散度 %.1f%%（极差 %.0f / 均值 %.0f）"
                  % (100 * (max(rmeans) - min(rmeans)) / rm if rm else 0,
                     max(rmeans) - min(rmeans), rm))
            if cm > 0:
                print("    电流离散度 %.1f%%（极差 %.2f A / 均值 %.2f A）"
                      % (100 * (max(cmeans) - min(cmeans)) / cm,
                         max(cmeans) - min(cmeans), cm))
            worst = max(rows, key=lambda r: abs(r[1][0] - rm))
            print("    偏离最大：电机 %d，转速 %+.1f%%"
                  % (worst[0] + 1, 100 * (worst[1][0] - rm) / rm if rm else 0))

    seg = [v for t, v in rcou if t0 <= t <= t1]
    if seg:
        per = [stats([s[i] for s in seg]) for i in range(MOTORS)]
        means = [p[0] for p in per]
        mm = sum(means) / MOTORS
        print("    RCOU 指令：六路均值 %s" % " ".join("%.0f" % x for x in means))
        print("    指令不对称度 %.1f%%（极差 %.0f PWM / 均值 %.0f）——"
              "这是控制器为配平已付出的量"
              % (100 * (max(means) - min(means)) / mm if mm else 0,
                 max(means) - min(means), mm))


def report_imu(vibe, imu, t0, t1):
    print("  IMU 失真")
    for i in sorted(vibe):
        seg = [(vx, vy, vz, cl) for t, vx, vy, vz, cl in vibe[i] if t0 <= t <= t1]
        if not seg:
            continue
        mag = sorted(math.sqrt(a * a + b * b + c * c) for a, b, c, _ in seg)
        clip = seg[-1][3] - seg[0][3]
        print("    IMU%d 振动模长 均值 %.1f / P95 %.1f / 峰值 %.1f m/s²，削顶增量 %d"
              % (i, sum(mag) / len(mag), mag[int(len(mag) * 0.95)], mag[-1], clip))

    insts = sorted(imu)
    if len(insts) < 2:
        print("    只有一台 IMU，无法做互比")
        return
    base = [(t, (ax, ay, az, g)) for t, ax, ay, az, g in imu[insts[0]] if t0 <= t <= t1]
    if not base:
        return
    for other in insts[1:]:
        ser = {k: [(t, v[k]) for t, v in
                   [(tt, (ax, ay, az)) for tt, ax, ay, az, _ in imu[other]]]
               for k in range(3)}
        low, high = [], []
        for t, (ax, ay, az, g) in base:
            d = []
            for k, val in enumerate((ax, ay, az)):
                o = interp(ser[k], t)
                if o is None:
                    d = None
                    break
                d.append(val - o)
            if not d:
                continue
            diff = math.sqrt(sum(x * x for x in d))
            if g < LOW_RATE_DPS:
                low.append(diff)
            elif g >= HIGH_RATE_DPS:
                high.append(diff)
        if not low or not high:
            print("    IMU%d vs IMU%d：机动分档样本不足（低 %d / 高 %d）"
                  % (insts[0], other, len(low), len(high)))
            continue
        lo = math.sqrt(sum(x * x for x in low) / len(low))
        hi = math.sqrt(sum(x * x for x in high) / len(high))
        print("    IMU%d vs IMU%d 加速度分歧 RMS：低机动(<%.0f°/s) %.3f m/s²，"
              "高机动(≥%.0f°/s) %.3f m/s²，比值 %.2f"
              % (insts[0], other, LOW_RATE_DPS, lo, HIGH_RATE_DPS, hi,
                 hi / lo if lo else float("nan")))
        if lo and hi / lo < 1.5:
            print("           → 高机动段分歧未显著放大，「大机动 IMU 失真」在本份日志上不成立")
        elif lo:
            print("           → 高机动段分歧明显放大，该说法在本份日志上有支持")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("logs", nargs="+")
    args = ap.parse_args(argv)
    for path in args.logs:
        print("=" * 74)
        print(os.path.basename(path))
        thr, rcou, esc, vibe, imu = collect(path)
        t0, t1 = flying_window(thr)
        if t0 is None:
            print("  没有飞行段（MOTB.ThrOut 从未超过 %.2f）" % FLYING_THROTTLE)
            continue
        print("  飞行段 %.1f s" % (t1 - t0))
        report_powertrain(esc, rcou, t0, t1)
        report_imu(vibe, imu, t0, t1)
    return 0


if __name__ == "__main__":
    sys.exit(main())
