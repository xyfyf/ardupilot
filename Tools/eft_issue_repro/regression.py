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
import time
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

def _rev_metrics(r):
    ev = r.get("reversals", [])
    m = r.get("metrics", {})
    return {"反拉次数": str(len(ev)),
            "峰值俯仰速率": "%.0f °/s" % max((e.get("peak_pitch_rate_deg_s", 0) for e in ev), default=-1),
            "峰值俯仰误差": "%.2f°" % max((e.get("peak_pitch_error_deg", 0) for e in ev), default=-1),
            "俯仰I项跨度": "%.4f" % m.get("pitch_i_span", -1)}


def _rev_check(r):
    ev = r.get("reversals", [])
    m = r.get("metrics", {})
    # 护栏而非达标线：P02 尚未关闭，只保证不比现基线更差。
    #
    # **基线必须取自默认配置**，这一条踩过坑：门槛最初按 pitch_i_span < 0.08 设，
    # 数字是从 runs/…-audit_P02_vff0088 里读的——那份开了 ATC_VFF_PIT=0.0088，
    # i_span 0.0397；而回归跑的是默认配置（VFF=0），实测 0.1213，于是护栏第一次
    # 运行就误报。VFF 把 I 项负担降 67%（0.1213 -> 0.0397）正是 P02 候选方案的
    # 效果，不是基线。
    #
    # 默认配置基线（2026-09-01，两次独立运行一致）：
    #   峰值俯仰速率 118 °/s、峰值俯仰误差 3.51°、pitch_i_span 0.1213
    return (len(ev) >= 3
            and max((e.get("peak_pitch_rate_deg_s", 0) for e in ev), default=999) < 150.0
            and max((e.get("peak_pitch_error_deg", 0) for e in ev), default=999) < 5.5
            and m.get("pitch_i_span", 999) < 0.18)


def _circle_metrics(r):
    st = r.get("steps", [])
    return {"档数": str(len(st)),
            "倾角饱和占比": "%.3f" % max((s.get("tilt_saturated_frac", 0) for s in st), default=-1),
            "姿态误差峰值": "%.2f°" % max((s.get("att_err_max_deg", 0) for s in st), default=-1)}


def _circle_check(r):
    st = r.get("steps", [])
    # 倾角饱和是 P07 的核心失效形态——一旦饱和圆周参考就跟不上，抽动随之出现。
    return (len(st) >= 2
            and max((s.get("tilt_saturated_frac", 1.0) for s in st), default=1.0) < 0.01
            and max((s.get("att_err_max_deg", 999) for s in st), default=999) < 5.0)


def _p07_step(r, speed):
    """取指定速度档。按 target_speed_m_s 找，不靠下标——档位顺序改了也不会错位。"""
    for st in r.get("steps", []):
        if abs(st.get("target_speed_m_s", -1) - speed) < 1e-6:
            return st
    return {}


def _p07_metrics(r):
    fast = _p07_step(r, 2.5)
    slow = _p07_step(r, 1.0)
    return {"2.5m/s 姿态误差": "%.2f°" % fast.get("att_err_mean_deg", -1),
            "1.0m/s 姿态误差": "%.2f°" % slow.get("att_err_mean_deg", -1),
            "2.5m/s 倾角饱和占比": "%.2f" % fast.get("tilt_saturated_frac", -1),
            "2.5m/s 实测半径": "%.2f m" % fast.get("actual_radius_mean_m", -1)}


def _p07_check(r):
    fast = _p07_step(r, 2.5)
    if not fast:
        return False
    # 两件事分开判，缺一不可。
    #
    # 其一，这一档必须真的顶到限幅——2 m 半径下 2.5 m/s 需要 17.7 度而
    # ANGLE_MAX 只有 15 度，所以饱和是这个工况的定义特征。没饱和说明测试根本
    # 没跑到它声称的工况（ANGLE_MAX 被改回 2000 就会这样），那种“通过”比失败
    # 更坏。
    if fast.get("tilt_saturated_frac", 0.0) < 0.5:
        return False
    # 且饱和必须是对着 15 度这个实机限幅发生的。放宽 LOIT_ANG_MAX 之后飞机照样
    # 会顶到新上限、饱和占比照样很高，但那是另一台飞机的工况，拿它的数字与本条
    # 的门限比没有意义。
    if abs(r.get("lean_limit_deg", -1.0) - 15.0) > 0.51:
        return False
    # 其二，抽动本身。默认配置（前馈关）实测 1.91 度，门限留到 2.6 度：既高于
    # 2026-08-27 记录的 2.24 度，又能在姿态环或来流力矩前馈被改坏时报警。
    return (fast.get("att_err_mean_deg", 99.0) < 2.6)


def _fence_metrics(r):
    steps = r.get("steps", [])
    return {"最小余量": "%.2f m" % min((s.get("margin_achieved_m", -99) for s in steps), default=-99),
            "停稳均值": "%.2f m" % min((s.get("hold_margin_mean_m", -99) for s in steps), default=-99),
            "越界档数": str(sum(1 for s in steps if s.get("breached")))}


def _sprint_metrics(r):
    steps = r.get("steps", [])
    return {"最低巡航": "%.2f m/s" % min((s.get("cruise_speed_m_s", -99) for s in steps), default=-99),
            "最小余量": "%.2f m" % min((s.get("margin_achieved_m", -99) for s in steps), default=-99),
            "越界档数": str(sum(1 for s in steps if s.get("breached")))}


def _sprint_check(r):
    steps = r.get("steps", [])
    # 「最低巡航 >= 2 m/s」不是性能指标，是**脚手架自检**。冲刺场景调试期间报废过
    # 三版，共同症状都是飞机根本没动起来而判据照样通过：射线距离用错时起跑瞬间就
    # 冻结、纯 P 增益收缩到零时巡航只有 0.07 m/s。没飞就不越界，那种「通过」比失败
    # 更坏。速度门槛把这类假通过挡在外面。
    return (len(steps) >= 3
            and not any(s.get("start_failed") or s.get("invalid_direction") for s in steps)
            and all(not s.get("breached") for s in steps)
            and min((s.get("margin_achieved_m", -99) for s in steps), default=-99) > 0.0
            and min((s.get("cruise_speed_m_s", -99) for s in steps), default=-99) >= 2.0)


def _sprint_check_speed(r):
    """在守住围栏之外，再卡「围栏附近还跑得动」。

    剖面减速度决定的不是守不守得住——命令律饱和到机体能力，任何剖面都刹得住——
    而是**飞手在边界附近被允许跑多快**。真机 15° 倾角下实测，剖面 1.05 m/s² 把飞机
    钳在 3.00/3.03/4.48 m/s，剖面 2.63 m/s² 给到 5.03/6.49/7.11 m/s，两者余量都是
    1.0~1.3 m。作业机在有界地块里，那 20 m 就是整片地头。
    门槛 4.5 m/s 卡在两者之间：剖面若退回降额，这一条立刻红。
    """
    steps = r.get("steps", [])
    return (_sprint_check(r)
            and min((s.get("cruise_speed_m_s", -99) for s in steps), default=-99) >= 4.5)


# 冲刺场景共用的围栏形状：外接 27.7 m 的正六边形、旋转 30°，边心距（半宽）24 m，
# 落在真机 2026-08-31 架次的 22–26 m 区间内。正北撞的是边心而不是顶点。
SPRINT_FENCE = ["--polyfence-sides", "6", "--polyfence-radius", "27.7",
                "--polyfence-rotate", "30",
                # 必须对齐真机倾角。脚手架默认 ANGLE_MAX=2000（20°），而真机
                # defaults.parm 是 1500（15°）——剖面减速度正是 g·tan(ANGLE_MAX)，
                # 20° 给 3.57 m/s²、15° 只有 2.63，差 36%。同一组 A/B 在 20° 下两条
                # 剖面完全重合、看不出任何差别，在 15° 下巡航速度差一倍：**结论在
                # 15° 与 20° 之间是反的**，跑错倾角等于没测。
                "--set", "ANGLE_MAX=1500", "--set", "PSC_POSXY_P=1.5"]


def _fence_check(r):
    steps = r.get("steps", [])
    return (len(steps) >= 4
            and all(not s.get("breached") for s in steps)
            and min((s.get("margin_achieved_m", -99) for s in steps), default=-99) > 0.0)


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
                           "水平漂移": "%.1f m/s" % (_m(r, "horiz_drift_max_m_s") or -1),
                           "掉高": "%.2f m" % _alt_loss(r)},
        # 6 号为最不利失效位置（正右，滚转力臂最大）
        # 判据落在**姿态**上。航向在单发失效后是被放弃的那一维——权限只够
        # 保姿态或保航向，不能兼得，实测保航向在 2 m/s 风下即坠毁。
        check=lambda r: (bool(_m(r, "still_armed_after_watch")) and _detect_delay(r) < 0.5
                         and abs(_m(r, "roll_steady_deg") or 999) < 10.0)
                        and _alt_loss(r) < 2.0,
        why="4 m/s 风下失效后须保持姿态可控、检测在 0.5 s 内；不降级的基线是 16.8 s 后坠毁",
    ),
    dict(
        pid="P04", name="掉桨-检测与降级", case="motor-fail", tag="prop_shed",
        # 掉桨与停转是两个方向相反的信号：停转是转速掉下去，掉桨是桨没了、
        # 负载没了、转速反而冲上去。MOT_FAIL_RPM 只认前者，这一条守的是后者。
        # 对混控来说两者后果相同（那个点不出力），所以降级路径共用，判据分开。
        args=["--motor", "6", "--detect", "--shed", "--set", "SIM_WIND_SPD=4",
              "--set", "SIM_WIND_DIR=90"],
        metrics=lambda r: {"检测延迟": "%.2f s" % _detect_delay(r),
                           "滚转稳态": "%.1f°" % (_m(r, "roll_steady_deg") or -1),
                           "水平漂移": "%.1f m/s" % (_m(r, "horiz_drift_max_m_s") or -1),
                           "掉高": "%.2f m" % _alt_loss(r)},
        check=lambda r: (bool(_m(r, "still_armed_after_watch")) and _detect_delay(r) < 0.5
                         and abs(_m(r, "roll_steady_deg") or 999) < 10.0)
                        and _alt_loss(r) < 2.0,
        why="掉桨须在 0.5 s 内检出并降级；判据与停转同为姿态可控，因为失去的是同一维",
    ),
    dict(
        pid="P04", name="检测器误报", case="uturn-auto",
        # 两条判据一起开：真机上就是一起开的，只验其中一条不算数。
        args=["--swath", "12"] + YAW_CFG + ["--set", "MOT_FAIL_RPM=300",
              "--set", "MOT_FAIL_TIME=200", "--set", "MOT_FAIL_THST=0.15",
              "--set", "MOT_FAIL_ROVR=1.15"],
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
    # ---- P02 / P07 专项入口纳入集中回归 ----
    #
    # 这两个场景此前只有专项命令行入口，游离在集中回归之外，改坏了没人报警。姿态层
    # 围栏栽过同一个跟头：交付现场的功能靠手工跑的一条命令验收，之后任何人改动
    # AC_Avoid 或那几个模式都不会触发告警。
    #
    # 两条判据都是**护栏**，不是达标线。P02 与 P07 均未关闭，数字只保证不比当前基线
    # 更差；把护栏读成「问题已解决」正是审查里点名警告过的错误。
    dict(
        pid="P02", name="高速反拉-抽动护栏", case="reverse", tag="reverse_guard",
        args=[],
        metrics=_rev_metrics,
        check=_rev_check,
        why="反拉是抽动最容易复现的工况。峰值俯仰速率与 I 项跨度是抽动的两个侧面，"
            "任一显著上升说明姿态环或前馈被改坏了",
    ),
    dict(
        pid="P06", name="自动绕圈-倾角饱和护栏", case="circle", tag="circle_guard",
        args=[],
        metrics=_circle_metrics,
        check=_circle_check,
        why="⚠ 本条守的是**自动 CIRCLE 模式**（run_circle 用 set_mode_wait(CIRCLE) 加 "
            "CIRCLE_RATE/CIRCLE_RADIUS）。而 P07 的实际问题是**飞手手动打杆绕圈时机身"
            "抽动**，走的是姿态链路而非 AC_Circle 的参考生成——两者不是一条路径。"
            "本条作为 AC_Circle 的回归护栏仍然有效（限速基准若被改回只看 WPNAV_ACCEL "
            "会立刻报警），但**不构成对 P07 的验证**；P07 由下一条守。",
    ),
    dict(
        pid="P07", name="手动绕圈-小半径抽动", case="loiter-circle", tag="p07_manual_circle",
        # 不传 ANGLE_MAX。LOITER 的飞手倾角走 loiter_nav->get_angle_max_rad()，
        # 即 LOIT_ANG_MAX——eft_hexa.parm 里已是 15，与实机一致，本来就绑得住。
        # 基线文档曾记「必须把 ANGLE_MAX 由 2000 改回 1500，否则倾角饱和不可能
        # 复现」，那对这条路径不成立：实测 1500 与 2000 四档结果逐位相同，而把
        # LOIT_ANG_MAX 由 15 放到 20 才让指令倾角由 16.4 升到 19.9 度。
        args=[],
        metrics=_p07_metrics,
        check=_p07_check,
        why="P07 是**飞手手动打杆**绕 2 m 小圈时的抽动，走姿态链路，与上一条的 "
            "AC_Circle 参考生成不是一条路径。杆量按 A·cos(ωt)/A·sin(ωt) 旋转，"
            "直接映射到倾角指令，因此能真的顶到倾角上限。根因已定为来流力矩"
            "（同 P02）：默认配置四档姿态误差 0.29/0.80/1.47/1.91 度，而按本档"
            "工况正确定号的前馈（ATC_VFF_RLL -0.012 / PIT +0.012）把它压到 "
            "0.03/0.10/0.24/0.43 度，降幅 77-90%，已贴住关闭速度力矩的理论下限 "
            "0.40 度。本条守默认配置，同时校验两件事：该档确实进入了饱和工况，且生效"
            "倾角上限确实是实机的 15 度（LOIT_ANG_MAX）——放宽之后照样会饱和，但那"
            "是另一台飞机的工况，数字不可比",
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
        # 更严的一档：改用 polyfence（路径规划器唯一认的形式）且余量压到 2 m。
        # LOITER 的接近速度上限由倾角决定——实测指令顶到 PSC_ANGLE_MAX=20° 时
        # 速度峰值 6.33 m/s，再设高的 LOIT_SPEED 也上不去，故本项即全速度包线。
        pid="P05", name="围栏-LOITER严格档", tag="fence_loiter_strict", case="fence",
        args=["--fence-mode", "LOITER", "--polyfence-radius", "60",
              "--set", "FENCE_ENABLE=1", "--set", "FENCE_TYPE=4",
              "--set", "FENCE_MARGIN=2", "--set", "FENCE_ACTION=0"],
        # 判据用 margin_achieved_m 与 breached，两者都是到边界的**法向**距离，
        # 圆形与多边形通用。不要退回 closest_radius_m —— 那是到起飞点的径向距离，
        # 只对圆形围栏等价，对多边形既算错余量也漏判越界。
        metrics=lambda r: {
            "最深进入": "%.2f m" % max((s.get("closest_radius_m", 0)
                                    for s in r.get("steps", [])), default=-1),
            "最小栏内余量": "%.2f m" % min((s.get("margin_achieved_m", 99)
                                       for s in r.get("steps", [])), default=-1),
            "越界档数": str(sum(1 for s in r.get("steps", []) if s.get("breached")))},
        check=lambda r: (len(r.get("steps", [])) >= 4
                         and all(not s.get("breached") for s in r.get("steps", []))
                         and min((s.get("margin_achieved_m", -99)
                                  for s in r.get("steps", [])), default=-99) > 0.0),
        why="LOITER 是唯一有主动围控的手动模式，也是作业期间的推荐模式。"
            "余量压到 2 m 仍不得越界；越界说明避障链路坏了",
    ),
    # ---- 姿态层围栏硬限 ----
    #
    # 交付现场的功能至今没有自动化条目守着，验收数据只来自提交信息里手工跑的命令行。
    # 任何人改动 AC_Avoid、AC_Fence 或这几个模式的输入路径，集中回归都不会报警。
    #
    # **每条都带风速维度**，这不是可选项：无风下四个模式全过，而 2 m/s 风曾经把
    # ALT_HOLD 推到栏外 86 m —— 只测无风会给出系统性偏乐观的结论。
    #
    # 判据用「越界档数 = 0」加「最小余量 > 0」，两者都是到边界的法向距离，与飞行
    # 方位无关。刻意不固定 --fence-heading：turn_to_heading() 的机体系方向约定尚未
    # 定论（见 TODO.md P2），依赖它反而会引入不可信的重复性。
    dict(
        pid="P05", name="姿态层围栏-ALT_HOLD-无风", tag="fence_althold_calm", case="fence",
        args=["--fence-mode", "ALT_HOLD", "--polyfence-radius", "60",
              "--polyfence-sides", "6",
              "--set", "FENCE_MARGIN=5", "--set", "FENCE_ACTION=0"],
        metrics=_fence_metrics,
        check=_fence_check,
        why="姿态层硬限的无风基线。飞手满杆冲栏不得越界",
    ),
    dict(
        pid="P05", name="姿态层围栏-ALT_HOLD-2m/s风", tag="fence_althold_w2", case="fence",
        args=["--fence-mode", "ALT_HOLD", "--polyfence-radius", "60",
              "--polyfence-sides", "6",
              "--set", "FENCE_MARGIN=5", "--set", "FENCE_ACTION=0",
              "--set", "SIM_WIND_SPD=2", "--set", "SIM_WIND_DIR=0"],
        metrics=_fence_metrics,
        check=_fence_check,
        why="风是这条链路的真实工况。缺位置项时限制器会以 a_wind*T 的恒定速度"
            "被推穿余量——这一条就是为守住那个修复而设",
    ),
    dict(
        pid="P05", name="姿态层围栏-POSHOLD-2m/s风", tag="fence_poshold_w2", case="fence",
        args=["--fence-mode", "POSHOLD", "--polyfence-radius", "60",
              "--polyfence-sides", "6",
              "--set", "FENCE_MARGIN=5", "--set", "FENCE_ACTION=0",
              "--set", "SIM_WIND_SPD=2", "--set", "SIM_WIND_DIR=0"],
        metrics=_fence_metrics,
        check=_fence_check,
        why="POSHOLD 打杆段走姿态链路，与 ALT_HOLD 同一条代码路径",
    ),
    dict(
        pid="P05", name="姿态层围栏-STABILIZE-2m/s风", tag="fence_stabilize_w2", case="fence",
        args=["--fence-mode", "STABILIZE", "--polyfence-radius", "60",
              "--polyfence-sides", "6",
              "--set", "FENCE_MARGIN=5", "--set", "FENCE_ACTION=0",
              "--set", "SIM_WIND_SPD=2", "--set", "SIM_WIND_DIR=0"],
        metrics=_fence_metrics,
        check=_fence_check,
        why="手动油门模式。fence_throttle 用于顶到与其他模式相当的高度，"
            "否则贴地飞行的围栏结论不可信",
    ),
    dict(
        pid="P05", name="姿态层围栏-DRIFT-2m/s风", tag="fence_drift_w2", case="fence",
        args=["--fence-mode", "DRIFT", "--polyfence-radius", "60",
              "--polyfence-sides", "6",
              "--set", "FENCE_MARGIN=5", "--set", "FENCE_ACTION=0",
              "--set", "SIM_WIND_SPD=2", "--set", "SIM_WIND_DIR=0"],
        metrics=_fence_metrics,
        check=_fence_check,
        why="DRIFT 编译进 EFT_CAAC 且此前完全没有硬限，实测越界 1032 m。"
            "接入点与其他三个不同——roll 被速度误差项覆盖、pitch 被自动刹车覆盖，"
            "限制器必须接在送进姿态控制器之前",
    ),
    # ---- 姿态层围栏：冲刺场景 ----
    #
    # 与上面五条测的**不是一回事**。上面是「逼近后停住」的稳态侵入，飞机全程被限制器
    # 压着加速，从没以作业速度巡航过；这里是以 3/5/7 m/s 巡航撞围栏，考的是刹车距离
    # v²/(2a) 够不够用——而那是场地尺寸的函数，所以场地必须按真机的半宽 24 m，
    # 不能用上面那个 60 m 六边形（60 m 对任何速度都够用，测不出东西）。
    dict(
        pid="P05", name="姿态层围栏-冲刺-ALT_HOLD-无风", tag="sprint_althold_calm",
        case="fence-sprint",
        args=["--fence-mode", "ALT_HOLD"] + SPRINT_FENCE
             + ["--set", "FENCE_MARGIN=5", "--set", "FENCE_ACTION=0"],
        metrics=_sprint_metrics,
        check=_sprint_check,
        why="推荐配置下的冲刺基线。三档都不得进余量带",
    ),
    dict(
        pid="P05", name="姿态层围栏-冲刺-现场工况", tag="sprint_field_cfg",
        case="fence-sprint",
        args=["--fence-mode", "ALT_HOLD"] + SPRINT_FENCE
             + ["--sprint-speeds", "5,7,10",
                "--set", "FENCE_MARGIN=1", "--set", "FENCE_ACTION=0",
                "--set", "SIM_WIND_SPD=5", "--set", "SIM_WIND_DIR=180"],
        metrics=_sprint_metrics,
        check=_sprint_check_speed,
        why="复刻 2026-08-31 架次的工况：余量压到 1 m、5 m/s 顺风推向围栏，"
            "速度加到 5/7/10 找失守边界。这一条同时卡两件事——不得越界（盯命令律），"
            "且围栏附近巡航不得低于 4.5 m/s（盯剖面减速度）。"
            "实测剖面 2.63 给 5.03/6.49/7.11 m/s，退回 1.05 只剩 3.00/3.03/4.48",
    ),
    dict(
        pid="P05", name="电子围栏边界-LOITER", tag="fence_loiter_edge", case="fence", args=[],
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


def _alt_loss(r):
    """失效后的掉高。判据必须看这一项：曾经有一个混控量纲错误让飞机从 15 m
    一路掉到地面、触地后靠满油门浮在 1.9 m，而当时所有判据——姿态、漂移、
    是否仍解锁——**全部通过**。姿态好看是因为它当时正稳稳地往下掉。"""
    before = r.get("alt_before_m")
    lowest = r.get("alt_min_after_fail_m")
    if before is None or lowest is None:
        return float("inf")     # 拿不到高度就不能算通过
    return before - lowest


def _detect_delay(r):
    fs = (_m(r, "fail_time_ms") or 0) / 1000.0
    for t, s in r.get("statustext", []):
        if "Motor" in s and ("stopped" in s or "degraded" in s):
            return t / 1000.0 - fs
    return 99.0


def _motor_msgs(r):
    return sum(1 for _, s in r.get("statustext", []) if "Motor" in s and ("stopped" in s or "degraded" in s))


def _check_unique_variants():
    """重名会让日志互相覆盖、结果目录分不清是哪一条——启动即报，别等跑完才发现。"""
    seen = {}
    for it in SUITE:
        v = "reg_%s_%s" % (it["pid"], it.get("tag") or it["case"])
        if v in seen:
            raise SystemExit("变体名重复: %s\n  %s\n  %s\n请给其中一条加 tag=" 
                             % (v, seen[v], it["name"]))
        seen[v] = it["name"]


def run_one(item, outdir, timeout_s):
    """跑一条并读**本次**产生的结果。

    三处曾经同时失效，合起来会把「运行失败」报成「通过」：
      1. subprocess.run() 的返回码没人看；
      2. stdout/stderr 全丢进 DEVNULL，失败时连原因都不留；
      3. 随后按 variant 名做全局 glob 再取 sorted(...)[-1]。目录名以时间戳开头，
         字典序即时间序，所以取到的是**历史上最新**的一次，不是刚跑的这一次。
    本次运行没产生结果时，第 3 条会安静地读上一次的 result.json 交上去。
    """
    # tag 而不是 case：同一 case 可以有多条目（fence 就有七条），全都叫
    # reg_P05_fence 会让 <variant>.log 互相覆盖、runs/ 目录也分不出是哪一条。
    variant = "reg_%s_%s" % (item["pid"], item.get("tag") or item["case"])
    cmd = [sys.executable, REPRO, item["case"], "--variant", variant] + item["args"]
    log_path = os.path.join(outdir, "%s.log" % variant)
    started = time.time()
    try:
        with open(log_path, "w", encoding="utf-8") as log:
            proc = subprocess.run(cmd, cwd=HERE, timeout=timeout_s,
                                  stdout=log, stderr=subprocess.STDOUT)
    except subprocess.TimeoutExpired:
        return {"ok": False, "note": "超时（输出见 %s）" % os.path.basename(log_path),
                "metrics": {}}
    if proc.returncode != 0:
        return {"ok": False,
                "note": "运行失败 returncode=%d（输出见 %s）"
                        % (proc.returncode, os.path.basename(log_path)),
                "metrics": {}}

    import glob
    # 只认本次运行之后落盘的结果。留 5 s 容差应付文件系统时间戳粒度。
    hits = [h for h in glob.glob(os.path.join(HERE, "runs", "*%s" % variant, "result.json"))
            if os.path.getmtime(h) >= started - 5.0]
    hits.sort()
    if not hits:
        return {"ok": False,
                "note": "本次未产生结果（输出见 %s）" % os.path.basename(log_path),
                "metrics": {}}
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
    _check_unique_variants()
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
