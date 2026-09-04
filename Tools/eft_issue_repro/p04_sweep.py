#!/usr/bin/env python3
"""P04 单动力失效的矩阵扫描与重复性验证。

regression.py 里的 P04 只有三条固定条目——停转、掉桨、误报——用来守住回归。
本工具解决的是另外两个问题：

  1. **覆盖**。失效位置、风速、MOT_FAIL_YTRK 的组合要一起看才有意义。
     单点结论外推不到整个包线：SITL 六台电机对称，几何最不利位置是 3/6，
     但那是几何意义上的；风一大，配平方向也参与决定谁最难。

  2. **重复性**。这一条更要紧。SITL 不是确定性的——EKF 收敛、风的相位、
     起飞时刻都会变，同一组参数跑两次结果不会逐位相同。**在知道架次间散布
     有多大之前，任何"改了参数指标从 A 变成 B"的结论都不成立**，因为 B−A
     可能整个落在噪声里。本工具先量散布，再谈效应。

用法：

    # 先量重复性：同一点跑 5 次，看散布
    p04_sweep.py --motors 6 --wind 4 --ytrk 0 --repeat 5

    # 再看效应：同一点两种 YTRK 各跑 5 次
    p04_sweep.py --motors 6 --wind 4 --ytrk 0 1 --repeat 5

    # 全矩阵（很慢，六位置 × 三风速 × 两档 = 36 架次）
    p04_sweep.py --motors 1 2 3 4 5 6 --wind 0 4 8 --ytrk 0 1

判据与 regression.py 的 P04 条目一致，不另立标准：

    仍解锁 且 检测延迟 < 0.5 s 且 |滚转稳态| < 10° 且 掉高 < 2 m

结果落在 runs/sweep-<时间戳>/ 下，同时给出 JSON 和可直接贴进文档的表格。
"""

import argparse
import datetime
import json
import os
import statistics
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPRO = os.path.join(HERE, "reproduce.py")


def _m(result, key, default=None):
    """先查顶层，再查 metrics——两处都有场景专属字段。与 regression.py 同源。"""
    if key in result:
        return result[key]
    return result.get("metrics", {}).get(key, default)


def _alt_loss(r):
    before = _m(r, "alt_before_m")
    lowest = _m(r, "alt_min_after_fail_m")
    if before is None or lowest is None:
        return float("inf")
    return before - lowest


def _detect_delay(r):
    fs = (_m(r, "fail_time_ms") or 0) / 1000.0
    for t, s in r.get("statustext", []):
        if "Motor" in s and ("stopped" in s or "degraded" in s):
            return t / 1000.0 - fs
    return 99.0


# 判据照抄 regression.py 的 P04「单电机停转-检测与降级」条目。刻意不另立
# 标准：扫描发现的问题必须能直接对上回归，否则两套数字会各说各话。
def _passed(r):
    return (bool(_m(r, "still_armed_after_watch"))
            and _detect_delay(r) < 0.5
            and abs(_m(r, "roll_steady_deg") or 999) < 10.0
            and _alt_loss(r) < 2.0)


# 每格记录的量。顺序即表格列序。
FIELDS = [
    ("detect_delay_s", "检测延迟 s", 2),
    ("roll_err_max_deg", "滚转峰值 °", 2),
    ("roll_steady_deg", "滚转稳态 °", 2),
    ("pitch_err_max_deg", "俯仰峰值 °", 2),
    ("yaw_rate_mean_degs", "偏航速率均 °/s", 1),
    ("yaw_rate_max_degs", "偏航速率峰 °/s", 1),
    ("yaw_drift_max_deg", "偏航漂移 °", 1),
    ("horiz_drift_max_m_s", "水平漂移 m/s", 3),
    ("motor_saturation_frac", "电机饱和", 3),
]


def run_one(motor, wind, ytrk, rep, outdir, timeout_s):
    """跑一个格子。变体名带上全部维度，否则 runs/ 里分不清是哪一格。"""
    variant = "sweep_m%d_w%d_y%d_r%d" % (motor, wind, ytrk, rep)
    cmd = [sys.executable, REPRO, "motor-fail", "--variant", variant,
           "--motor", str(motor), "--detect",
           "--set", "SIM_WIND_SPD=%d" % wind,
           "--set", "SIM_WIND_DIR=90",
           "--set", "MOT_FAIL_YTRK=%d" % ytrk]
    log_path = os.path.join(outdir, "%s.log" % variant)
    started = time.time()
    try:
        with open(log_path, "w", encoding="utf-8") as log:
            proc = subprocess.run(cmd, cwd=HERE, timeout=timeout_s,
                                  stdout=log, stderr=subprocess.STDOUT)
    except subprocess.TimeoutExpired:
        return {"ok": False, "note": "超时", "metrics": {}}
    if proc.returncode != 0:
        return {"ok": False, "note": "returncode=%d" % proc.returncode, "metrics": {}}

    import glob
    # 只认本次之后落盘的结果；5 s 容差应付时间戳粒度。与 regression.py 同。
    hits = [h for h in glob.glob(os.path.join(HERE, "runs", "*%s" % variant, "result.json"))
            if os.path.getmtime(h) >= started - 5.0]
    if not hits:
        return {"ok": False, "note": "本次未产生结果", "metrics": {}}
    hits.sort()
    r = json.load(open(hits[-1], encoding="utf-8"))
    met = {}
    for key, _label, _nd in FIELDS:
        v = _m(r, key)
        met[key] = v if isinstance(v, (int, float)) else None
    met["detect_delay_s"] = _detect_delay(r)
    met["alt_loss_m"] = _alt_loss(r)
    return {"ok": _passed(r), "note": "", "metrics": met, "run": hits[-1]}


def _spread(vals):
    """返回 (均值, 极差, 标准差)。样本少于 2 个时标准差为 0。"""
    vs = [v for v in vals if isinstance(v, (int, float))]
    if not vs:
        return None, None, None
    mean = statistics.mean(vs)
    rng = max(vs) - min(vs)
    sd = statistics.stdev(vs) if len(vs) > 1 else 0.0
    return mean, rng, sd


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--motors", nargs="*", type=int, default=[6],
                    help="失效电机编号，默认 6（最不利之一）")
    ap.add_argument("--wind", nargs="*", type=int, default=[4],
                    help="侧风速度 m/s，默认 4，与 regression 的 P04 条目一致")
    ap.add_argument("--ytrk", nargs="*", type=int, default=[0],
                    help="MOT_FAIL_YTRK 取值，默认 0")
    ap.add_argument("--repeat", type=int, default=1,
                    help="每格重复次数。>1 时输出均值与极差——先量散布再谈效应")
    ap.add_argument("--timeout", type=int, default=900, help="单架次超时秒数")
    args = ap.parse_args()

    stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    outdir = os.path.join(HERE, "runs", "sweep-%s" % stamp)
    os.makedirs(outdir, exist_ok=True)

    cells = [(m, w, y) for m in args.motors for w in args.wind for y in args.ytrk]
    total = len(cells) * args.repeat
    print("P04 扫描：%d 格 × %d 次 = %d 架次，结果落在 %s"
          % (len(cells), args.repeat, total, outdir))
    print()

    results = {}
    done = 0
    for (m, w, y) in cells:
        runs = []
        for rep in range(args.repeat):
            done += 1
            print("  [%d/%d] 电机%d 风%dm/s YTRK=%d 第%d次 ... "
                  % (done, total, m, w, y, rep + 1), end="", flush=True)
            res = run_one(m, w, y, rep, outdir, args.timeout)
            print("通过" if res["ok"] else "未通过 %s" % res["note"])
            runs.append(res)
        results["m%d_w%d_y%d" % (m, w, y)] = runs

    # ---- 表格 ----
    print()
    print("=" * 100)
    hdr = "%-22s %-6s" % ("工况", "通过")
    for _k, label, _nd in FIELDS:
        hdr += " %14s" % label
    print(hdr)
    print("-" * 100)
    for (m, w, y) in cells:
        runs = results["m%d_w%d_y%d" % (m, w, y)]
        npass = sum(1 for r in runs if r["ok"])
        row = "%-22s %-6s" % ("电机%d 风%d YTRK=%d" % (m, w, y),
                              "%d/%d" % (npass, len(runs)))
        for key, _label, nd in FIELDS:
            mean, rng, _sd = _spread([r["metrics"].get(key) for r in runs])
            if mean is None:
                row += " %14s" % "—"
            elif args.repeat > 1:
                row += " %14s" % ("%.*f±%.*f" % (nd, mean, nd, rng / 2.0))
            else:
                row += " %14.*f" % (nd, mean)
        print(row)
    print("=" * 100)

    if args.repeat > 1:
        print()
        print("上表为 均值±极差/2。**先看极差再看差异**：若两组均值之差不大于各自")
        print("极差，那个差异就落在架次间噪声里，不能算作效应。")

    payload = {"stamp": stamp, "args": vars(args),
               "cells": {k: [{"ok": r["ok"], "note": r["note"],
                              "metrics": r["metrics"], "run": r.get("run")}
                             for r in v]
                         for k, v in results.items()}}
    with open(os.path.join(outdir, "sweep.json"), "w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, indent=2)
    print()
    print("JSON: %s" % os.path.join(outdir, "sweep.json"))

    allpass = all(r["ok"] for v in results.values() for r in v)
    return 0 if allpass else 1


if __name__ == "__main__":
    sys.exit(main())
