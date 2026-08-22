#!/usr/bin/env python3
"""从 ArduPilot .bin 日志离线计算控制器性能指标。

用法:
    python3 log_control_metrics.py <log.bin> [log2.bin ...]
    python3 log_control_metrics.py --csv out.csv <log.bin> ...

只读日志，不改任何东西。给出姿态环/角速率环的跟踪误差、高度环表现、
PID 饱和比例、振动水平，并检查各消息的实际记录频率是否够用。

注意: 控制器无法做在环重放(闭环耦合)，这里做的是开环性能评估——
从真实飞行数据里量化"当时那套控制律表现如何"，用于横向对比不同架次
或不同参数下的结果。
"""
import sys
import os
import math
from collections import defaultdict

# 用仓库自带的定制版 pymavlink（含 EF 包头支持），而不是 pip 装的原版。
# 路径相对本脚本解析：Tools/eft_log_analysis/ -> modules/mavlink/
_AP = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   os.pardir, os.pardir, "modules", "mavlink")
_AP = os.path.normpath(_AP)
if os.path.isdir(_AP) and _AP not in sys.path:
    sys.path.insert(0, _AP)

from pymavlink import DFReader  # noqa: E402

# 姿态环带宽通常 10-30 Hz，角速率环更高。低于这个采样率的记录会混叠，
# 高频振荡、超调这类特征直接丢失。
MIN_USEFUL_HZ = 50.0

# 判定"在飞"的油门下限（CTUN.ThO 归一化 0-1）
FLYING_THROTTLE = 0.15


def _pct(sorted_vals, q):
    if not sorted_vals:
        return float("nan")
    i = min(len(sorted_vals) - 1, max(0, int(round(q * (len(sorted_vals) - 1)))))
    return sorted_vals[i]


class Stat:
    """累积一路误差信号的统计量。"""

    __slots__ = ("vals",)

    def __init__(self):
        self.vals = []

    def add(self, v):
        if v is not None and not math.isnan(v):
            self.vals.append(v)

    def summary(self):
        if not self.vals:
            return None
        n = len(self.vals)
        rms = math.sqrt(sum(v * v for v in self.vals) / n)
        a = sorted(abs(v) for v in self.vals)
        return {
            "n": n,
            "rms": rms,
            "p95": _pct(a, 0.95),
            "max": a[-1],
            "bias": sum(self.vals) / n,
        }


def wrap180(d):
    """把角度差折到 [-180, 180]，处理偏航过零。"""
    return (d + 180.0) % 360.0 - 180.0


def analyse(path):
    log = DFReader.DFReader_binary(path)

    # 先扫一遍 CTUN 确定在飞窗口，避免地面段污染统计
    fly_t0 = fly_t1 = None
    while True:
        m = log.recv_match(type="CTUN")
        if m is None:
            break
        if getattr(m, "ThO", 0) >= FLYING_THROTTLE:
            t = m.TimeUS
            if fly_t0 is None:
                fly_t0 = t
            fly_t1 = t

    if fly_t0 is None:
        return {"path": path, "error": "未检测到飞行段（CTUN.ThO 始终低于阈值）"}

    log = DFReader.DFReader_binary(path)

    att = {k: Stat() for k in ("roll", "pitch", "yaw")}
    rate = {k: Stat() for k in ("roll", "pitch", "yaw")}
    alt = {k: Stat() for k in ("alt", "crt")}
    pid_lim = defaultdict(lambda: [0, 0])   # 轴 -> [饱和帧数, 总帧数]
    vibe_max = [0.0, 0.0, 0.0]
    clip_first = {}
    clip_last = {}
    counts = defaultdict(int)
    tspan = {}

    # 单位直接取自日志自带的元数据，不靠猜（CTUN.CRt 是 cm/s 不是 m/s）
    units = {}
    types = ["ATT", "RATE", "CTUN", "PIDR", "PIDP", "PIDY", "VIBE"]
    while True:
        m = log.recv_match(type=types)
        if m is None:
            break
        t = m.TimeUS
        typ = m.get_type()

        # 记录频率按全程算，不限于飞行段
        counts[typ] += 1
        span = tspan.get(typ)
        tspan[typ] = (t if span is None else span[0], t)

        if not (fly_t0 <= t <= fly_t1):
            continue

        if typ == "CTUN" and "alt" not in units:
            cols = m.get_fieldnames()
            fmt = next((f for f in log.formats.values() if f.name == "CTUN"), None)
            u = getattr(fmt, "units", None) if fmt else None
            if u:
                units["alt"] = u[cols.index("Alt")] or "m"
                units["crt"] = u[cols.index("CRt")] or "?"

        if typ == "ATT":
            att["roll"].add(m.DesRoll - m.Roll)
            att["pitch"].add(m.DesPitch - m.Pitch)
            att["yaw"].add(wrap180(m.DesYaw - m.Yaw))
        elif typ == "RATE":
            rate["roll"].add(m.RDes - m.R)
            rate["pitch"].add(m.PDes - m.P)
            rate["yaw"].add(m.YDes - m.Y)
        elif typ == "CTUN":
            alt["alt"].add(m.DAlt - m.Alt)
            alt["crt"].add(m.DCRt - m.CRt)
        elif typ in ("PIDR", "PIDP", "PIDY"):
            axis = {"PIDR": "roll", "PIDP": "pitch", "PIDY": "yaw"}[typ]
            slot = pid_lim[axis]
            slot[1] += 1
            # Flags bit0 = 积分限幅生效
            if int(getattr(m, "Flags", 0)) & 1:
                slot[0] += 1
        elif typ == "VIBE":
            for i, f in enumerate(("VibeX", "VibeY", "VibeZ")):
                vibe_max[i] = max(vibe_max[i], getattr(m, f, 0.0))
            inst = getattr(m, "IMU", 0)
            c = getattr(m, "Clip", 0)
            clip_first.setdefault(inst, c)
            clip_last[inst] = c

    rates = {}
    for typ, n in counts.items():
        t0, t1 = tspan[typ]
        dur = (t1 - t0) / 1e6
        rates[typ] = n / dur if dur > 0 else float("nan")

    return {
        "path": path,
        "duration": (fly_t1 - fly_t0) / 1e6,
        "att": {k: v.summary() for k, v in att.items()},
        "rate": {k: v.summary() for k, v in rate.items()},
        "alt": {k: v.summary() for k, v in alt.items()},
        "pid_lim": dict(pid_lim),
        "vibe_max": vibe_max,
        "clips": {i: clip_last[i] - clip_first.get(i, 0) for i in clip_last},
        "rates": rates,
        "units": units,
    }


def _row(label, s, unit):
    if not s:
        return f"  {label:12s} (无数据)"
    return (f"  {label:12s} RMS {s['rms']:8.3f}   P95 {s['p95']:8.3f}   "
            f"最大 {s['max']:8.3f}   偏置 {s['bias']:+8.3f}  {unit}")


def report(r):
    print("=" * 78)
    print(os.path.basename(r["path"]))
    print("=" * 78)
    if "error" in r:
        print("  " + r["error"])
        return

    print(f"飞行时长: {r['duration']:.1f} s\n")

    print("姿态环跟踪误差 (期望 - 实际)")
    for k, name in (("roll", "横滚"), ("pitch", "俯仰"), ("yaw", "偏航")):
        print(_row(name, r["att"][k], "度"))

    print("\n角速率环跟踪误差 (期望 - 实际)")
    for k, name in (("roll", "横滚"), ("pitch", "俯仰"), ("yaw", "偏航")):
        print(_row(name, r["rate"][k], "度/秒"))

    print("\n高度环")
    print(_row("高度", r["alt"]["alt"], r["units"].get("alt", "米")))
    print(_row("爬升率", r["alt"]["crt"], r["units"].get("crt", "?")))

    if r["pid_lim"]:
        print("\nPID 积分限幅触发比例")
        for axis, (hit, tot) in sorted(r["pid_lim"].items()):
            if tot:
                print(f"  {axis:12s} {hit}/{tot} = {100.0 * hit / tot:.1f}%")

    vx, vy, vz = r["vibe_max"]
    print(f"\n振动峰值   X {vx:.1f}  Y {vy:.1f}  Z {vz:.1f}  m/s/s"
          f"   (>30 需关注, >60 通常已影响 EKF)")
    if r["clips"]:
        tot = sum(r["clips"].values())
        print(f"加速度计削顶(clip)增量: {r['clips']}  合计 {tot}"
              + ("   ← 非零说明量程被打满" if tot else ""))

    print("\n各消息实际记录频率")
    for typ in sorted(r["rates"]):
        hz = r["rates"][typ]
        warn = ""
        if typ in ("ATT", "RATE") and hz < MIN_USEFUL_HZ:
            warn = f"   ← 偏低，奈奎斯特仅 {hz / 2:.0f} Hz，高频特征会混叠"
        print(f"  {typ:6s} {hz:7.1f} Hz{warn}")


def main(argv):
    csv_out = None
    args = list(argv)
    if "--csv" in args:
        i = args.index("--csv")
        csv_out = args[i + 1]
        del args[i:i + 2]

    if not args:
        print(__doc__)
        return 1

    results = []
    for p in args:
        if not os.path.exists(p):
            print(f"跳过（不存在）: {p}", file=sys.stderr)
            continue
        r = analyse(p)
        results.append(r)
        report(r)
        print()

    if csv_out:
        import csv
        with open(csv_out, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["log", "duration_s",
                        "att_roll_rms_deg", "att_pitch_rms_deg", "att_yaw_rms_deg",
                        "rate_roll_rms_dps", "rate_pitch_rms_dps", "rate_yaw_rms_dps",
                        "alt_rms_m", "crt_rms", "crt_unit", "vibe_max"])
            for r in results:
                if "error" in r:
                    continue
                g = lambda d, k: (d[k] or {}).get("rms", float("nan"))  # noqa: E731
                w.writerow([os.path.basename(r["path"]), f"{r['duration']:.1f}",
                            f"{g(r['att'], 'roll'):.4f}", f"{g(r['att'], 'pitch'):.4f}",
                            f"{g(r['att'], 'yaw'):.4f}",
                            f"{g(r['rate'], 'roll'):.4f}", f"{g(r['rate'], 'pitch'):.4f}",
                            f"{g(r['rate'], 'yaw'):.4f}",
                            f"{g(r['alt'], 'alt'):.4f}", f"{g(r['alt'], 'crt'):.4f}",
                            r["units"].get("crt", "?"), f"{max(r['vibe_max']):.1f}"])
        print(f"已写出 {csv_out}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
