#!/usr/bin/env python3
"""判定一份 ArduPilot .bin 日志能否用于 Tools/Replay 离线在环重放。

用法:
    python3 check_replay_ready.py <log.bin> [log2.bin ...]

只读日志。逐项检查 Replay 的硬性前提，给出结论和缺什么。

不要用 grep 搜文件里有没有 "RFRH" 来判断——日志开头的 FMT 表会声明
固件认识的全部消息类型，每份日志都能匹配到。必须按实际记录条数查。
"""
import sys
import os

# 用仓库自带的定制版 pymavlink（含 EF 包头支持），而不是 pip 装的原版。
# 路径相对本脚本解析：Tools/eft_log_analysis/ -> modules/mavlink/
_AP = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   os.pardir, os.pardir, "modules", "mavlink")
_AP = os.path.normpath(_AP)
if os.path.isdir(_AP) and _AP not in sys.path:
    sys.path.insert(0, _AP)

from pymavlink import DFReader  # noqa: E402

# 帧骨架，缺任何一条都无法重放
CORE = ["RFRH", "RFRF", "RISH", "RISI"]

# EKF3 的事件/原点标记
EKF3 = ["REV3", "RSO3"]

# 各传感器可选，取决于机上装了什么；有就该有对应的 replay 记录
SENSORS = {
    "气压计": ["RBRH", "RBRI"],
    "磁罗盘": ["RMGH", "RMGI"],
    "GPS": ["RGPH", "RGPI", "RGPJ"],
    "测距仪": ["RRNH", "RRNI"],
    "光流": ["ROFH"],
    "空速": ["RASH", "RASI"],
    "视觉里程计": ["RVOH"],
    "信标": ["RBCH", "RBCI"],
}

ALL_R = CORE + EKF3 + [m for v in SENSORS.values() for m in v]

# 帧间隔超过这个倍数视为丢帧
GAP_FACTOR = 5.0


def check(path):
    log = DFReader.DFReader_binary(path)

    counts = {m: 0 for m in ALL_R}
    params = {}
    frame_ts = []
    armed_us = None
    first_r_us = None

    types = ALL_R + ["PARM", "EV", "ARM"]
    while True:
        m = log.recv_match(type=types)
        if m is None:
            break
        typ = m.get_type()

        if typ == "PARM":
            if m.Name in ("LOG_REPLAY", "LOG_DISARMED", "LOG_BITMASK",
                          "LOG_FILE_BUFSIZE", "SCHED_LOOP_RATE"):
                params[m.Name] = m.Value
            continue

        if typ == "ARM":
            if armed_us is None and getattr(m, "ArmState", 0):
                armed_us = m.TimeUS
            continue

        if typ == "EV":
            # EV id 10 = ARMED
            if armed_us is None and getattr(m, "Id", 0) == 10:
                armed_us = m.TimeUS
            continue

        counts[typ] += 1
        if first_r_us is None:
            first_r_us = m.TimeUS
        if typ == "RFRH":
            frame_ts.append(m.TimeUS)

    # 帧率与丢帧
    hz = float("nan")
    gaps = 0
    worst_gap = 0.0
    if len(frame_ts) > 2:
        span = (frame_ts[-1] - frame_ts[0]) / 1e6
        if span > 0:
            hz = len(frame_ts) / span
            nominal = 1e6 / hz
            for a, b in zip(frame_ts, frame_ts[1:]):
                d = b - a
                if d > nominal * GAP_FACTOR:
                    gaps += 1
                    worst_gap = max(worst_gap, d / 1e6)

    return {
        "path": path,
        "counts": counts,
        "params": params,
        "hz": hz,
        "frames": len(frame_ts),
        "gaps": gaps,
        "worst_gap": worst_gap,
        "armed_us": armed_us,
        "first_r_us": first_r_us,
    }


def report(r):
    print("=" * 74)
    print(os.path.basename(r["path"]))
    print("=" * 74)

    c = r["counts"]
    p = r["params"]
    blockers = []

    # 1. 参数
    lr = p.get("LOG_REPLAY")
    ld = p.get("LOG_DISARMED")
    print("参数")
    print(f"  LOG_REPLAY       {('未记录' if lr is None else int(lr))}"
          + ("" if lr == 1 else "   ← 必须为 1"))
    print(f"  LOG_DISARMED     {('未记录' if ld is None else int(ld))}"
          + ("" if (ld or 0) >= 1 else "   ← 必须为 1 或 2，否则缺解锁前数据"))
    for k in ("LOG_BITMASK", "LOG_FILE_BUFSIZE", "SCHED_LOOP_RATE"):
        if k in p:
            print(f"  {k:16s} {int(p[k])}")
    if lr != 1:
        blockers.append("LOG_REPLAY 未开启")
    if (ld or 0) < 1:
        blockers.append("LOG_DISARMED 未开启")

    # 2. 帧骨架
    print("\n帧骨架（缺任一条都无法重放）")
    for m in CORE:
        ok = c[m] > 0
        print(f"  {m:6s} {c[m]:>8d} {'✓' if ok else '✗ 缺失'}")
        if not ok:
            blockers.append(f"{m} 无记录")

    print("\nEKF3 事件标记")
    for m in EKF3:
        print(f"  {m:6s} {c[m]:>8d} {'✓' if c[m] else '（无，若从未设过原点可接受）'}")

    # 3. 传感器
    print("\n传感器输入")
    for name, msgs in SENSORS.items():
        tot = sum(c[m] for m in msgs)
        detail = " ".join(f"{m}:{c[m]}" for m in msgs)
        print(f"  {name:12s} {tot:>8d}   {detail}"
              + ("" if tot else "   （机上无此传感器则正常）"))

    # 4. 帧率与连续性
    print("\n帧连续性")
    if r["frames"] < 3:
        print("  RFRH 帧不足，无法评估")
    else:
        print(f"  RFRH 帧数 {r['frames']}，实际 {r['hz']:.1f} Hz"
              f"（应接近 SCHED_LOOP_RATE {int(p.get('SCHED_LOOP_RATE', 0)) or '?'}）")
        if r["gaps"]:
            print(f"  检测到 {r['gaps']} 处丢帧，最长间隔 {r['worst_gap']:.2f} s"
                  "   ← SD 卡写入跟不上，重放会在缺口处失真")
            blockers.append(f"{r['gaps']} 处丢帧")
        else:
            print("  无明显丢帧 ✓")

    # 5. 解锁前数据
    print("\n解锁前数据")
    if r["armed_us"] is None:
        print("  未找到解锁事件（可能整段都未解锁）")
    elif r["first_r_us"] is None:
        print("  无 replay 记录，无法判断")
    else:
        lead = (r["armed_us"] - r["first_r_us"]) / 1e6
        if lead > 0:
            print(f"  解锁前有 {lead:.1f} s 的 replay 数据 ✓")
        else:
            print("  解锁前无 replay 数据   ← EKF 初始化段缺失，Replay 起不来")
            blockers.append("缺解锁前数据")

    print()
    if blockers:
        print("结论: 不能用于 Replay")
        for b in blockers:
            print(f"  - {b}")
        print("\n下次试飞前设置：LOG_REPLAY=1、LOG_DISARMED=1，重启飞控后再飞。")
        print("日志体积会明显变大，建议同时调大 LOG_FILE_BUFSIZE 并使用高速 SD 卡。")
    else:
        print("结论: 可以用于 Replay ✓")
        print("\n  ./waf configure --board sitl && ./waf replay")
        print(f"  build/sitl/tool/Replay {r['path']}")
        print("  # 产出日志在 ./logs/ 下，注意扩展名是 .EFT，用 DFReader_binary 读")


def main(argv):
    if not argv:
        print(__doc__)
        return 1
    bad = 0
    for p in argv:
        if not os.path.exists(p):
            print(f"跳过（不存在）: {p}", file=sys.stderr)
            bad = 1
            continue
        report(check(p))
        print()
    return bad


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
