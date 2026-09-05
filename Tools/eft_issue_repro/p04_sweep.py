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

**入口默认是先验申报（--entry declare）**，与现场第一阶段一致：失效瞬间直接写
MOT_FAIL_IDX，检测器不参与。此前本工具把入口写死成 --detect，测的一直是第一阶段
不会走的那条路，而且拿「检测延迟 < 0.5 s」当判据——惰走 τ=0.3 s 时实测延迟 0.60 s，
整批会被判成不合格。

用法：

    # 先量重复性：同一点跑 5 次，看散布
    p04_sweep.py --motors 6 --wind 4 --ytrk 0 --repeat 5

    # 动力余量分层（唯一已知会改变定性结论的维度）
    p04_sweep.py --motors 1 2 3 4 5 6 --hover-thr 0.161 0.40 0.55 --ytrk 0 1

    # 第二阶段：检测器在回路内
    p04_sweep.py --entry detect --motors 6 --wind 4 8

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
import glob
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

def _num(result, key, missing):
    """取数值指标；**只有真的缺失才用 missing 兜底**。

    曾经写成 `_m(r, key) or 999`：0.0 在 Python 里是假值，于是一次滚转稳态
    恰好为 0 的完美架次会被当成「没测到」判为不合格。判据里凡是「没有就当很差」
    的写法都要显式判 None，否则最好的结果和最坏的结果走同一条分支。
    """
    v = _m(result, key)
    return missing if v is None else v


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


def _solver_failures(run_json):
    """从 MALC 日志数出「求解失败并静默回退」的周期数。

    这是新增的一列，而且是这张表里唯一能直接看出**算法边界**的量。分配器失败
    时会悄悄回退到前向混控——飞机照飞、姿态可能还过判据，但那一段带着摘列后
    的力矩失衡。实测即便余量充足的回归架次里也有 2.3% 的周期是这样。

    Res 取自 AP_MotorsMatrix::AllocResult：0 成功，非 0 各有原因。
    """
    d = os.path.dirname(run_json)
    logs = glob.glob(os.path.join(d, "logs", "*.EFT")) + \
           glob.glob(os.path.join(d, "logs", "*.BIN"))
    if not logs:
        return None, None
    try:
        from pymavlink import DFReader
        m = DFReader.DFReader_binary(sorted(logs)[-1])
    except Exception:
        return None, None
    total = bad = 0
    worst = 0
    while True:
        x = m.recv_match(type="MALC")
        if x is None:
            break
        total += 1
        if getattr(x, "Res", 0) != 0:
            bad += 1
            worst = max(worst, int(x.Res))
    if total == 0:
        return None, None
    return round(bad / float(total), 4), worst


# 判据与 regression.py 的 P04 条目同源，但**按入口分开**：
#
# 先验申报（--entry declare，现场第一阶段用的就是这条）里检测器根本不参与，
# 降级由脚本申报触发、时刻已知，「检测延迟」不是一个有意义的量——拿它当判据
# 会把本来正确的架次判成不合格。反过来，检测路径必须守住这一条。
#
# 此前本工具把入口写死成 --detect 且无条件套用 0.5 s 判据，于是它测的一直是
# 现场第一阶段**不会走**的那条路。
def _passed(r, entry):
    ok = (bool(_m(r, "still_armed_after_watch"))
          and abs(_num(r, "roll_steady_deg", 999)) < 10.0
          and _alt_loss(r) < 2.0)
    if entry == "detect":
        ok = ok and _detect_delay(r) < 0.5
    return ok


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
    # 掉高有判据，上冲没有，而降级瞬态恰恰可能造成上冲——只记 alt_min 时
    # 这一路看不见，而"看不见"和"没发生"在结果里长得一样。
    ("alt_rise_3s_m", "3s上冲 m", 3),
    ("motor_saturation_frac", "电机饱和", 3),
    # 饱和判据 2026-09-05 才修好（原门限 1990 高于有效上限 1905，恒为 0）。
    # 峰值一并记下，否则「饱和 0.000」无从复核是真没饱和还是门限又错了。
    ("motor_pwm_peak", "PWM峰值", 0),
    # wrap180 的偏航漂移上限 180°，看不出转圈；这一列才是真实累计转动。
    ("yaw_total_rotation_deg", "累计转动 °", 1),
    # 求解失败占比：唯一直接反映算法边界的量，从 MALC 的 Res 字段数出来。
    ("solver_fail_frac", "求解失败占比", 4),
]


def _key(m, w, d, y, h):
    return "m%d_w%d_d%d_y%d%s" % (m, w, d, y, "" if h is None else "_h%.2f" % h)


def run_one(motor, wind, ytrk, rep, outdir, timeout_s, entry="declare",
            hover_thr=None, wind_dir=90):
    """跑一个格子。变体名带上全部维度，否则 runs/ 里分不清是哪一格。"""
    variant = "sweep_%s_m%d_w%d_d%d_y%d%s_r%d" % (
        entry[:3], motor, wind, wind_dir, ytrk,
        "" if hover_thr is None else "_h%03d" % round(hover_thr * 100), rep)
    # declare = 先验申报：失效瞬间直接写 MOT_FAIL_IDX，检测器不参与。
    #           这是现场第一阶段实际走的路径。
    # detect  = 由转速判据自行发现，第二阶段才用。
    cmd = [sys.executable, REPRO, "motor-fail", "--variant", variant,
           "--motor", str(motor),
           "--degrade" if entry == "declare" else "--detect",
           "--set", "SIM_WIND_SPD=%d" % wind,
           "--set", "SIM_WIND_DIR=%d" % wind_dir,
           "--set", "MOT_FAIL_YTRK=%d" % ytrk]
    if hover_thr is not None:
        # 动力余量维：同时改模型的 hoverThrOut 与 MOT_THST_HOVER，否则闭环
        # 失配会混进结果里，分不清是余量不够还是整定不匹配。
        cmd += ["--model-set", "hoverThrOut=%.3f" % hover_thr,
                "--set", "MOT_THST_HOVER=%.3f" % hover_thr]
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
    frac, worst = _solver_failures(hits[-1])
    met["solver_fail_frac"] = frac
    met["solver_worst_result"] = worst
    return {"ok": _passed(r, entry), "note": "", "metrics": met, "run": hits[-1]}


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
    ap.add_argument("--entry", choices=("declare", "detect"), default="declare",
                    help="降级由谁触发。declare=先验申报（现场第一阶段，默认）；"
                         "detect=转速判据自行发现（第二阶段）")
    ap.add_argument("--hover-thr", nargs="*", type=float, default=[None],
                    help="动力余量：模型 hoverThrOut 与 MOT_THST_HOVER 同步设为此值。"
                         "不给则用模型原值。这是唯一已知会改变定性结论的维度")
    ap.add_argument("--wind-dir", nargs="*", type=int, default=[90],
                    help="风向角，默认只跑 90")
    ap.add_argument("--repeat", type=int, default=1,
                    help="每格重复次数。>1 时输出均值与极差——先量散布再谈效应")
    ap.add_argument("--timeout", type=int, default=900, help="单架次超时秒数")
    args = ap.parse_args()

    stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    outdir = os.path.join(HERE, "runs", "sweep-%s" % stamp)
    os.makedirs(outdir, exist_ok=True)

    cells = [(m, w, d, y, h)
             for m in args.motors for w in args.wind for d in args.wind_dir
             for y in args.ytrk for h in args.hover_thr]
    total = len(cells) * args.repeat
    print("P04 扫描：%d 格 × %d 次 = %d 架次，结果落在 %s"
          % (len(cells), args.repeat, total, outdir))
    print()

    results = {}
    done = 0
    for (m, w, d, y, h) in cells:
        runs = []
        for rep in range(args.repeat):
            done += 1
            print("  [%d/%d] 电机%d 风%dm/s@%d° YTRK=%d%s 第%d次 ... "
                  % (done, total, m, w, d, y,
                     "" if h is None else " 余量%.2f" % h, rep + 1),
                  end="", flush=True)
            res = run_one(m, w, y, rep, outdir, args.timeout,
                          entry=args.entry, hover_thr=h, wind_dir=d)
            print("通过" if res["ok"] else "未通过 %s" % res["note"])
            runs.append(res)
        results[_key(m, w, d, y, h)] = runs

    # ---- 表格 ----
    print()
    print("=" * 100)
    hdr = "%-26s %-6s" % ("工况", "通过")
    for _k, label, _nd in FIELDS:
        hdr += " %14s" % label
    print(hdr)
    print("-" * 100)
    for (m, w, d, y, h) in cells:
        runs = results[_key(m, w, d, y, h)]
        npass = sum(1 for r in runs if r["ok"])
        row = "%-26s %-6s" % (_key(m, w, d, y, h), "%d/%d" % (npass, len(runs)))
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
