#!/usr/bin/env python3
"""地面规划约束校验：这条航线可不可飞，不可行的差在哪，改成什么才可行。

    python3 Tools/mission_arc_check/mission_arc_check.py \\
        任务.waypoints 参数.param \\
        --yaw-rate-max 57 --yaw-rate-source "真机偏航阶跃辨识 2026-09-xx，20° 倾角下"

`--yaw-rate-max` 与 `--yaw-rate-source` 都是必填，没有默认值。这不是苛刻：

飞行中的判定（`ArduCopter/mode_auto.cpp:1882`）取的是 `ATC_SLEW_YAW`，那是**允许
值**不是**能力值**。参数设得高于机体真实能力时，判定会放行一个飞不出来的弯。地面
工具若也从参数推这个数，就只是把同一个错误搬到地面。所以它必须由人显式给出，并且
必须说明这个数从哪来——报告会把这句话原样印上去，让读报告的人自己判断可信度。

退出码：0 全部可行；1 有转弯不可行；2 输入有误。
"""

import argparse
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import arcnav        # noqa: E402
import check         # noqa: E402
import fence as fence_mod  # noqa: E402
import inputs        # noqa: E402

# SITL 占位机型的实测可达偏航速率，供无真机数据时贯通流程用。
#
# **54.7–59.1 不是一个不确定区间，是两个配置各自的实测值**（基线第 8 节"偏航前馈"
# 一节，同条件 R=3 m、2 m/s）：
#
#     ATC_RAT_YAW_FF = 0      → 54.7 °/s
#     ATC_RAT_YAW_FF = 0.30   → 59.1 °/s
#
# 把它当区间、取中值 57，会在两头都给出错的答案：FF 开着时低估 4%，FF 关着时
# 高估 4%。而这条边界上 4% 足以翻转结论——本工具第一次跑验收航线时就翻错了一次。
# 偏航前馈直接产生力矩、不必等误差攒起来，所以它改变的是**能力**而不只是跟踪质量。
#
# 另需注意：这两个数都是从 SITL 模型推出来的，而模型的惯量 [12.5,12.5,25] kg·m²、
# 50 kg 质量、18 N·m/(m/s) 力矩系数都是首轮占位/反向校准值，不是实测机体属性
# （见 docs/dev-algo-consolidated-review-2026-09-01.md）。所以它们继承了占位量的
# 不确定度，不能当真机值用。
SITL_YAW_FF_ON_DEGS = 59.1
SITL_YAW_FF_OFF_DEGS = 54.7
SITL_YAW_FF_THRESHOLD = 0.30

_SITL_CAVEAT = "——SITL 模型值，模型常数含占位量，非实测机体属性，不可当真机值用"


def sitl_yaw_capability(params):
    """按参数文件里的 ATC_RAT_YAW_FF 选 SITL 实测值，并说明选了哪个、为什么。

    返回 (度/秒, 警告或 None, 出处文字)。
    """
    ff = params.get("ATC_RAT_YAW_FF", 0.0)
    if ff >= SITL_YAW_FF_THRESHOLD:
        return SITL_YAW_FF_ON_DEGS, None, (
            "SITL 占位机型实测 %.4g °/s（20° 作业倾角下，ATC_RAT_YAW_FF=%.4g 开启）%s"
            % (SITL_YAW_FF_ON_DEGS, ff, _SITL_CAVEAT))
    if ff <= 0.0:
        return SITL_YAW_FF_OFF_DEGS, None, (
            "SITL 占位机型实测 %.4g °/s（20° 作业倾角下，ATC_RAT_YAW_FF=0）%s"
            % (SITL_YAW_FF_OFF_DEGS, _SITL_CAVEAT))
    # 中间值没有实测点，取保守的那端而不是插值——插出来的数没有任何一次实测支撑它。
    return SITL_YAW_FF_OFF_DEGS, (
        "ATC_RAT_YAW_FF=%.4g 落在两个实测点（0 与 %.4g）之间，没有对应的实测偏航能力。"
        "已取保守端 %.4g °/s；不插值，因为插出来的数没有任何一次实测支撑。"
        % (ff, SITL_YAW_FF_THRESHOLD, SITL_YAW_FF_OFF_DEGS)), (
        "SITL 占位机型实测 %.4g °/s（保守端，ATC_RAT_YAW_FF=%.4g 无实测点）%s"
        % (SITL_YAW_FF_OFF_DEGS, ff, _SITL_CAVEAT))


def build_parser():
    p = argparse.ArgumentParser(
        description="AC_ArcNav 规划约束的地面校验",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    p.add_argument("mission", help="任务文件（QGC WPL 110，.waypoints）")
    p.add_argument("params", nargs="+",
                   help="参数文件（.param / .parm），可给多份，后者覆盖前者")
    g = p.add_mutually_exclusive_group(required=True)
    g.add_argument("--yaw-rate-max", type=float, metavar="度/秒",
                   help="机体实测可达偏航速率，必须是作业倾角下的值，不是悬停值")
    g.add_argument("--yaw-from-sitl-model", action="store_true",
                   help="用 SITL 占位机型的实测值贯通流程（按参数里的 ATC_RAT_YAW_FF "
                        "选 %.4g 或 %.4g °/s），报告会标注它含占位量"
                        % (SITL_YAW_FF_OFF_DEGS, SITL_YAW_FF_ON_DEGS))
    p.add_argument("--yaw-rate-source", metavar="出处",
                   help="上一项那个数从哪来。与 --yaw-rate-max 同时必填")
    p.add_argument("--yaw-accel-max", type=float, metavar="度/秒²",
                   help="偏航角加速度实测能力，覆盖 ATC_ACCEL_Y_MAX")
    p.add_argument("--speed", type=float, metavar="米/秒",
                   help="作业速度，覆盖 WPNAV_SPEED")
    p.add_argument("--json", metavar="路径", help="同时把结果写成 JSON")
    return p


def render(mission_path, params_path, resolved, speed_ms, findings, out):
    w = out.write
    w("=" * 78 + "\n")
    w("规划约束校验报告\n")
    w("=" * 78 + "\n")
    w("任务文件  %s\n" % mission_path)
    for i, pp in enumerate(params_path):
        w("参数文件  %s%s\n" % (pp, "  [覆盖前者]" if i else ""))
    w("\n")

    w("-- 判定用到的量，及其出处 " + "-" * 51 + "\n")
    for name, pv in resolved.provenance.items():
        flag = "" if pv.present else "  [默认值]"
        w("  %-22s %10.5g %-8s  %s%s\n"
          % (name, pv.value, pv.unit, pv.source, flag))
        if pv.note:
            w("  %-22s %s\n" % ("", "· " + pv.note))
    w("  %-22s %10.5g %-8s  %s\n"
      % ("判定用作业速度", speed_ms, "m/s", "见上 WPNAV_SPEED 或 --speed"))
    w("\n")
    w("  编译期常数（现场参数改不了，改需重新编译固件）：\n")
    for k, v in arcnav.COMPILE_TIME_CONSTANTS.items():
        w("    %-32s %s\n" % (k, v))
    w("\n")

    if resolved.warnings:
        w("-- 需要注意 " + "-" * 64 + "\n")
        for i, msg in enumerate(resolved.warnings, 1):
            w("  [%d] %s\n" % (i, msg))
        w("\n")

    if not findings:
        w("任务里没有 NAV_LOITER_TURNS 项。AC_ArcNav 的协调转弯只由它触发\n"
          "（mode_auto.cpp:1875），普通航点转弯走 SCurve 混合，不在本工具判定范围内。\n")
        return

    w("-- 逐个转弯 " + "-" * 64 + "\n")
    n_bad = 0
    for f in findings:
        mark = "可行" if f.ok else "不可行"
        if not f.ok:
            n_bad += 1
        w("\n  [seq %d] %s   半径 %.4g m   扫掠 %.4g°   速度 %.4g m/s\n"
          % (f.seq, mark, f.radius_effective_m, math.degrees(f.sweep_rad), f.speed_ms))
        if f.feasible:
            w("          入弧点 seq %d，前段长 %.4g m，要求 lead-in %.4g m %s\n"
              % (f.entry_seq, f.prev_leg_m, f.lead_in_m,
                 "（够）" if f.lead_in_satisfied else "（不够）"))
        else:
            # 判不可行时 plan_feasible 提前返回，根本没算 lead_in，印 0 会被读成
            # 「这个弯不要求前段长度」。前段长度照印，那是任务的事实。
            w("          入弧点 seq %d，前段长 %.4g m；lead-in 未计算"
              "（转弯判不可行时 plan_feasible 提前返回，不产出这个数）\n"
              % (f.entry_seq, f.prev_leg_m))
        if f.row_spacing_m is not None:
            w("          任务几何量出的行距 %.4g m" % f.row_spacing_m)
            if f.rows_needed is not None:
                w("，当前第 %d 行 / 至少要第 %d 行，距下一档 %.3g m"
                  % (f.rows_skipped, f.rows_needed, f.step_margin_m))
            w("\n")
        if f.fence is not None:
            fv = f.fence
            if fv.breached:
                w("          围栏：**越界**（%s），最深 %.3g m\n"
                  % (fence_mod.TYPE_NAMES.get(fv.breach_type, "?"), -fv.worst_margin_m))
            elif not fv.conclusive:
                w("          围栏：**结论不完整**——见下方说明，不可当作栏内\n")
            elif fv.checked_types:
                names = "、".join(fence_mod.TYPE_NAMES[t]
                                  for t in (1, 2, 4, 8) if fv.checked_types & t)
                extra = ("，最近处余 %.3g m（扫掠 %.0f°）"
                         % (fv.worst_margin_m, fv.worst_at_deg or 0.0)
                         ) if fv.worst_margin_m is not None else ""
                w("          围栏：栏内（已查 %s）%s\n" % (names, extra))
            for note in fv.unverifiable:
                w("          围栏：%s\n" % note)
            for note in fv.notes:
                w("          围栏：%s\n" % note)
        if f.margin_pct is not None:
            # 可行不等于稳妥。余量薄的转弯要让人看见薄在哪。
            w("          余量 %.1f%%（%s 最紧）%s\n"
              % (100.0 * f.margin_pct, f.margin_which,
                 "  ← 余量很薄，参数微调或侧风即可翻档"
                 if f.margin_pct < 0.10 else ""))
        if not f.feasible:
            w("          原因：%s\n" % check.REASON_TEXT.get(f.reason, f.reason))
        elif not f.lead_in_satisfied:
            w("          原因：前一段航段不够长，兑现不了 lead-in 承诺\n")
        for note in f.notes:
            w("          注：%s\n" % note)
        for r in f.remedies:
            w("          改法：%s\n" % r)

    w("\n" + "-" * 78 + "\n")
    w("共 %d 个转弯，%d 个不可行。\n" % (len(findings), n_bad))
    w("\n")
    w("本报告判四项：倾角预算、偏航速率、前段 lead-in 长度、整条弧的围栏采样。\n")
    w("**不判**地形、电量、喷洒，以及普通航点转弯的 SCurve 掉速。\n")
    w("围栏采的是**理想圆**，与机上 arc_within_fence() 同法；回旋过渡段并不严格落在\n")
    w("那个圆上，所以余量很薄时理想圆判栏内不等于实飞不出栏。\n")
    w("报告说可行不等于飞得出来——判据是 SITL 或真机的实飞轨迹，不是本工具的输出。\n")


def main(argv=None):
    args = build_parser().parse_args(argv)

    if not args.yaw_from_sitl_model and not args.yaw_rate_source:
        print("错误：--yaw-rate-max 必须与 --yaw-rate-source 一起给。\n"
              "     偏航能力是本校验最松紧敏感的输入，报告必须说明它从哪来。",
              file=sys.stderr)
        return 2

    try:
        items = inputs.load_mission(args.mission)
        params = inputs.load_params(args.params)
        # SITL 快捷方式要看参数才能定，所以放在读完参数之后。
        if args.yaw_from_sitl_model:
            yaw_rate, ff_warning, yaw_source = sitl_yaw_capability(params)
        else:
            yaw_rate, ff_warning = args.yaw_rate_max, None
            yaw_source = args.yaw_rate_source
        resolved, speed_ms = inputs.resolve_limits(
            params, yaw_rate, yaw_source, args.yaw_accel_max)
        if ff_warning:
            resolved.warnings.insert(0, ff_warning)
    except inputs.InputError as exc:
        print("错误：%s" % exc, file=sys.stderr)
        return 2

    if args.speed is not None:
        speed_ms = args.speed
        resolved.provenance["作业速度"] = inputs.Provenance(
            speed_ms, "m/s", "命令行 --speed", True, "覆盖 WPNAV_SPEED")

    fence_cfg = fence_mod.from_params(params)
    findings = check.check_mission(items, resolved.limits, speed_ms, fence_cfg)
    render(args.mission, args.params, resolved, speed_ms, findings, sys.stdout)

    if args.json:
        with open(args.json, "w") as fh:
            json.dump({
                "mission": args.mission,
                "params": args.params,
                "yaw_rate_max_degs": yaw_rate,
                "yaw_rate_source": yaw_source,
                "speed_ms": speed_ms,
                "warnings": resolved.warnings,
                "turns": [vars(f) for f in findings],
            }, fh, ensure_ascii=False, indent=2)

    return 1 if any(not f.ok for f in findings) else 0


if __name__ == "__main__":
    sys.exit(main())
