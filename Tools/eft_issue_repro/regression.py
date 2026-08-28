#!/usr/bin/env python3
"""集中回归：把所有问题的专项场景在同一套模型与参数下跑一遍，出一张总表。

单独仿真回答「这个问题解决了吗」，集中回归回答**「解决它有没有碰坏别的」**。
两者都需要：本项目的改动集中在同一套 SIM_Frame 与同一个飞控上，一次改动
波及多个场景是常态而非例外——AUTO 的协调转弯分支曾经误捕获 LAND 命令，
同时挂住 landing 与 uturn-auto 两个场景，正是靠两处一起失败才被认出是同
一处改动引起的。有了这张表，那种情况改完当场就能看见。

每个场景配一条**验收判据**。没有判据的表只是数字堆砌，看不出「行不行」。
判据的门限取自已经复现并归因过的实测值，写在 SUITE 里，可随结论更新。

用法:
  regression.py                 跑全部
  regression.py --only P04 P06  只跑指定问题
  regression.py --list          列出条目不执行
"""

import argparse
import datetime
import json
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPRO = os.path.join(HERE, "reproduce.py")


def _m(result, key, default=None):
    """先查顶层，再查 metrics——两处都有场景专属字段。"""
    if key in result:
        return result[key]
    return result.get("metrics", {}).get(key, default)


# 掉头场景共用的偏航配置。协调转弯把约束压在多旋翼最弱的偏航轴上，默认增益
# 下 R=6 m/3 m/s 会被 ArcNav 以「偏航速率超预算」正当拒绝，那不是回归失败。
YAW_CFG = ["--set", "WPNAV_SPEED=300", "--set", "ATC_SLEW_YAW=12000",
           "--set", "ATC_RAT_YAW_P=0.60", "--set", "ATC_RAT_YAW_I=0.12",
           "--set", "ATC_RAT_YAW_FF=0.30"]

SUITE = [
    dict(
        pid="P01", name="着陆末段触地速度", case="landing", args=[],
        metrics=lambda r: {"触地下降速度": "%.2f m/s" % (_m(r, "touch_speed_m_s_down") or -1),
                           "触地到上锁": "%.2f s" % (_m(r, "touch_to_disarm_s") or -1)},
        # 门限取自已复现的基线：正常降落触地约 0.43 m/s，明显变差即为回归
        check=lambda r: (_m(r, "touch_speed_m_s_down") or 9) < 0.60,
        why="触地速度超过 0.6 m/s 说明末段减速链路被破坏",
    ),
    dict(
        pid="P04", name="单电机停转-检测与降级", case="motor-fail",
        # 必须带风。无风下的验证是不充分的：同一架次保留偏航时无风存活、
        # 2 m/s 即坠毁，而植保作业几乎总有风。判据不含风就通不过真实环境。
        args=["--motor", "6", "--detect", "--set", "SIM_WIND_SPD=4",
              "--set", "SIM_WIND_DIR=90"],
        metrics=lambda r: {"检测延迟": "%.2f s" % _detect_delay(r),
                           "滚转误差峰值": "%.1f°" % (_m(r, "roll_err_max_deg") or -1),
                           "滚转稳态": "%.1f°" % (_m(r, "roll_steady_deg") or -1),
                           "水平漂移": "%.1f m/s" % (_m(r, "horiz_drift_max_m_s") or -1)},
        # 6 号为最不利失效位置（正右，滚转力臂最大）
        # 判据落在**姿态**上。航向在单发失效后是被放弃的那一维——权限只够
        # 保姿态或保航向，不能兼得，实测保航向在 2 m/s 风下即坠毁。
        check=lambda r: (bool(_m(r, "still_armed_after_watch")) and _detect_delay(r) < 0.5
                         and abs(_m(r, "roll_steady_deg") or 999) < 10.0),
        why="4 m/s 风下失效后须保持姿态可控、检测在 0.5 s 内；不降级的基线是 16.8 s 后坠毁",
    ),
    dict(
        pid="P04", name="检测器误报", case="uturn-auto",
        args=["--swath", "12"] + YAW_CFG + ["--set", "MOT_FAIL_RPM=300",
              "--set", "MOT_FAIL_TIME=200", "--set", "MOT_FAIL_THST=0.15"],
        metrics=lambda r: {"误报次数": str(_motor_msgs(r)),
                           "弧内最低速": "%.2f m/s" % (_m(r, "arc_speed_min_m_s") or -1)},
        check=lambda r: _motor_msgs(r) == 0,
        why="正常作业全程不得误报——误摘一台好电机等于亲手制造要避免的事故",
    ),
    dict(
        pid="P06", name="协调转弯-GUIDED", case="uturn-arcnav",
        # 弧速取 5 m/s 时 WPNAV_SPEED 必须跟着放到 500：作业直线段受
        # pos_control 的速度上限约束，压在 3 m/s 上就加不到入弧所需的速度，
        # 弧会退化成航点式掉头（首轮回归即因此报出 53% 掉速）。
        args=["--swath", "24", "--speed", "5"] + YAW_CFG + ["--set", "WPNAV_SPEED=500"],
        # 用 arc_speed_dip_pct（按 ARCN.Prog 界定）而非场景脚本的粗窗口
        metrics=lambda r: {"弧内掉速": "%.1f%%" % (_m(r, "arc_speed_dip_pct") or -1),
                           "航向误差均值": "%.1f°" % (_m(r, "arc_hdg_err_mean_deg") or -1),
                           "实飞半径": "%.1f m" % (_m(r, "flown_radius_mean_m") or -1)},
        check=lambda r: bool(_m(r, "accepted")) and (_m(r, "arc_speed_dip_pct") or 99) < 10.0,
        why="匀速掉头的立身之本；航点式掉头在同条件下掉速 53%",
    ),
    dict(
        pid="P06", name="协调转弯-AUTO任务", case="uturn-auto",
        args=["--swath", "12"] + YAW_CFG,
        metrics=lambda r: {"进弧前尾段最低速": "%.2f m/s" % (_m(r, "pre_arc_speed_min_m_s") or -1),
                           "弧内最低速": "%.2f m/s" % (_m(r, "arc_speed_min_m_s") or -1)},
        # 进弧前不减速是 AUTO 接入的核心难点：LOITER_TURNS 原本归在 always stop
        check=lambda r: (_m(r, "pre_arc_speed_min_m_s") or 0) > 2.0
                        and (_m(r, "arc_speed_min_m_s") or 0) > 2.7,
        why="进弧前若减速到接近零，匀速掉头无从谈起",
    ),
    dict(
        pid="P03", name="磁罗盘偏航未对准辨识", case="mag-align",
        args=["--set", "SIM_MAG1_ANGL_Z=30", "--set", "COMPASS_EXTERNAL=1"],
        metrics=lambda r: {"辨识偏差": "%.2f°" % (_m(r, "mag_yaw_bias_deg") or -99),
                           "样本数": str(_m(r, "mag_align_samples") or 0)},
        # 注入 30°，GSF 参考自带约 ±3° 偏置，故给 25~35 的窗口
        check=lambda r: 25.0 < (_m(r, "mag_yaw_bias_deg") or -99) < 35.0,
        why="注入 30° 应辨识出 30° 附近；偏出窗口说明磁航向计算或参考基准出了问题",
    ),
    dict(
        pid="P06", name="偏航能力辨识", case="yaw-step", args=[],
        metrics=lambda r: {"悬停可达偏航速率": "%.1f °/s" % (_m(r, "yaw_rate_max_pos_degs") or -1),
                           "混控输出峰值": "%.2f" % (_m(r, "yaw_out_peak") or -1)},
        check=lambda r: (_m(r, "yaw_rate_max_pos_degs") or 0) > 50.0,
        why="偏航能力是协调转弯的硬约束，掉了说明偏航通道被改坏",
    ),
    dict(
        # AUTO 下围栏不拦飞机，只在越界后触发动作——这是 ArduPilot 的设计，
        # 不是缺陷。本项不判「不许越界」（那必然失败），而是盯**刹车动作还管不管用**：
        # 实测 FENCE_ACTION=4 从越界到停住耗 5.8 m，判据留到 10 m。
        # 航线自然最远约 61 m，围栏设 40 m，必然越界。
        pid="P05", name="围栏-AUTO刹车动作", case="route",
        args=["--set", "FENCE_ENABLE=1", "--set", "FENCE_TYPE=2",
              "--set", "FENCE_RADIUS=40", "--set", "FENCE_MARGIN=5",
              "--set", "FENCE_ACTION=4"],
        metrics=lambda r: {"距Home最远": "%.1f m" % (r.get("max_radius_m") or -1),
                           "越界量": "%.1f m" % ((r.get("max_radius_m") or 0) - 40.0)},
        check=lambda r: 40.0 < (r.get("max_radius_m") or 0) < 50.0,
        why="AUTO 下必然越界（水平航迹不过避障）；但 Brake 动作须在 10 m 内刹住，"
            "超出说明刹车链路坏了，低于 40 m 则说明测试没真正逼近围栏",
    ),
    dict(
        pid="P05", name="电子围栏边界-LOITER", case="fence", args=[],
        metrics=lambda r: {
            "最小实际余量": "%.2f m" % min((s.get("margin_achieved_m", 9)
                                       for s in r.get("steps", [])), default=-1),
            "最大冲入余量线": "%.3f m" % max((s.get("margin_overshoot_m", -9)
                                        for s in r.get("steps", [])), default=-1),
            "越界档数": str(sum(1 for s in r.get("steps", []) if s.get("breached")))},
        # LOITER 下主动避障生效，四档接近都应停在余量线内不越界。
        # 这一项只覆盖 LOITER；AUTO 下围栏不拦飞机（见基线文档第 7 节），
        # 那条由下面的 P05-AUTO 项单独盯。
        check=lambda r: all(not s.get("breached") for s in r.get("steps", []))
                        and all(s.get("margin_overshoot_m", 9) < 0.5 for s in r.get("steps", [])),
        why="LOITER 有主动避障，四档接近都不该越界；越界或冲入余量线超 0.5 m 说明避障链路坏了",
    ),
]


def _detect_delay(r):
    fs = (_m(r, "fail_time_ms") or 0) / 1000.0
    for t, s in r.get("statustext", []):
        if "Motor" in s and ("stopped" in s or "degraded" in s):
            return t / 1000.0 - fs
    return 99.0


def _motor_msgs(r):
    return sum(1 for _, s in r.get("statustext", []) if "Motor" in s and ("stopped" in s or "degraded" in s))


def run_one(item, outdir, timeout_s):
    variant = "reg_%s_%s" % (item["pid"], item["case"])
    cmd = [sys.executable, REPRO, item["case"], "--variant", variant] + item["args"]
    try:
        subprocess.run(cmd, cwd=HERE, timeout=timeout_s,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except subprocess.TimeoutExpired:
        return {"ok": False, "note": "超时", "metrics": {}}

    import glob
    hits = sorted(glob.glob(os.path.join(HERE, "runs", "*%s" % variant, "result.json")))
    if not hits:
        return {"ok": False, "note": "未产生结果", "metrics": {}}
    result = json.load(open(hits[-1], encoding="utf-8"))
    try:
        metrics = item["metrics"](result)
    except Exception as exc:                      # noqa: BLE001
        metrics = {"解析失败": str(exc)}
    try:
        ok = bool(item["check"](result))
    except Exception as exc:                      # noqa: BLE001
        return {"ok": False, "note": "判据异常: %s" % exc, "metrics": metrics}
    return {"ok": ok, "note": "", "metrics": metrics, "run": hits[-1]}


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--only", nargs="*", help="只跑这些问题编号，如 P04 P06")
    ap.add_argument("--list", action="store_true", help="列出条目不执行")
    ap.add_argument("--timeout", type=int, default=900, help="单条超时秒数")
    args = ap.parse_args()

    suite = SUITE
    if args.only:
        want = {p.upper() for p in args.only}
        suite = [i for i in SUITE if i["pid"] in want]
    if args.list:
        for i in suite:
            print("%-5s %-24s %s" % (i["pid"], i["name"], i["case"]))
        return 0

    stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    outdir = os.path.join(HERE, "runs", "regression-%s" % stamp)
    os.makedirs(outdir, exist_ok=True)

    rows = []
    for n, item in enumerate(suite, 1):
        print("[%d/%d] %s %s ..." % (n, len(suite), item["pid"], item["name"]),
              flush=True)
        res = run_one(item, outdir, args.timeout)
        rows.append(dict(item_pid=item["pid"], item_name=item["name"],
                         case=item["case"], why=item["why"], **res))
        print("        %s  %s" % ("通过" if res["ok"] else "未通过",
                                  "  ".join("%s=%s" % kv for kv in res["metrics"].items())
                                  or res["note"]), flush=True)

    lines = ["# 集中回归结果 %s" % stamp, "",
             "| 问题 | 场景 | 结果 | 关键指标 | 判据 |",
             "| :-- | :-- | :-- | :-- | :-- |"]
    for r in rows:
        lines.append("| `%s` | %s | %s | %s | %s |" % (
            r["item_pid"], r["item_name"],
            "**通过**" if r["ok"] else "**未通过**",
            "；".join("%s %s" % kv for kv in r["metrics"].items()) or r["note"],
            r["why"]))
    passed = sum(1 for r in rows if r["ok"])
    lines += ["", "共 %d 条，通过 %d 条。" % (len(rows), passed)]

    md = os.path.join(outdir, "regression.md")
    open(md, "w", encoding="utf-8").write("\n".join(lines) + "\n")
    json.dump(rows, open(os.path.join(outdir, "regression.json"), "w", encoding="utf-8"),
              ensure_ascii=False, indent=2)
    print("\n" + "\n".join(lines))
    print("\n结果:", md)
    return 0 if passed == len(rows) else 1


if __name__ == "__main__":
    sys.exit(main())
