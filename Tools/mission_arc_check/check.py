"""整条航线过一遍，逐个转弯给结论、原因和改法。

回答三个问题（见 docs/P06规划约束校验_方向与范围.md）：

  1. 这条航线能不能飞完？
  2. 不可行的那个，差在哪？
  3. **要改成什么才可行？**

第三条是重点。只说"不可行"没有用——现场要知道改哪个数、改到多少。
"""

import math
from dataclasses import dataclass, field

import arcnav
import fence as fence_mod
import inputs


# 拒绝原因的中文说法，与 arcnav.plan_feasible 的返回值一一对应。
REASON_TEXT = {
    "lean": "倾角预算不够",
    "yaw_rate": "偏航速率不够",
    "degenerate": "半径或速度为零",
}


@dataclass
class TurnFinding:
    seq: int                    # LOITER_TURNS 在任务里的序号
    entry_seq: int              # 入弧点（前一个 NAV 航点）的序号
    radius_planned_m: float
    radius_effective_m: float
    radius_quantised: bool
    sweep_rad: float
    speed_ms: float
    feasible: bool
    reason: str = None          # C++ 先返回的那一项，与机上会走的分支一致
    violations: list = field(default_factory=list)   # 实际超标的**全部**约束
    lead_in_m: float = 0.0
    prev_leg_m: float = 0.0
    lead_in_satisfied: bool = True
    row_spacing_m: float = None
    fence: object = None        # FenceVerdict；None = 本次未做围栏校验
    margin_pct: float = None    # 最紧那一项还剩多少余量，判可行时才有意义
    margin_which: str = None
    rows_skipped: int = None    # 当前掉到第几行
    rows_needed: int = None     # 至少要掉到第几行
    step_margin_m: float = None # 距离"还得多跳一行"还有多少米
    remedies: list = field(default_factory=list)
    notes: list = field(default_factory=list)

    @property
    def ok(self):
        """三项都过才算过。

        围栏那项「结论不完整」（缺多边形顶点）时**不算过**——把"没查出问题"
        当成"没问题"，正是这条工具线一路在防的那种错觉。
        """
        if not (self.feasible and self.lead_in_satisfied):
            return False
        if self.fence is not None and (self.fence.breached or not self.fence.conclusive):
            return False
        return True


def _perp_distance(pt, line_a, line_b):
    """点到直线的垂距。直线退化成一点时返回点距。"""
    dn, de = line_b[0] - line_a[0], line_b[1] - line_a[1]
    length = math.hypot(dn, de)
    if length < 1e-6:
        return inputs.dist(pt, line_a)
    return abs(de * (pt[0] - line_a[0]) - dn * (pt[1] - line_a[1])) / length


def required_radius_m(lim, speed_ms):
    """当前速度下能通过判定的最小半径，两项约束取较严者。

    倾角一项 min_radius_for_speed_m 直接给；偏航一项来自 v/r ≤ ψ̇·50%，
    反解得 r ≥ v/(ψ̇·50%)。两者哪个更严取决于半径大小：紧弯偏航先顶住，
    大弯倾角先顶住。报告要说的是"改到多少"，所以必须取 max 而不是任选一个。
    """
    r_lean = arcnav.min_radius_for_speed_m(lim, speed_ms)
    r_yaw = 0.0
    if arcnav.is_positive(lim.yaw_rate_max_rads):
        r_yaw = speed_ms / (lim.yaw_rate_max_rads * arcnav.YAW_RATE_FRACTION)
    return max(r_lean, r_yaw), r_lean, r_yaw


def constraint_margin(lim, radius_m, speed_ms):
    """最紧那一项还剩多少余量，返回 (比例, 名字)。

    只报"可行"是不够的：余量 3% 和余量 60% 是完全不同的处境，前者一次参数微调
    或一阵侧风就翻档。本工具第一次跑验收航线时，那条实飞掉速 0.19% 的航线余量
    只有 3%——正因为余量这么薄，偏航能力取 57 还是 59.1 就足以翻转结论。
    """
    v_max, v_lean, v_yaw = max_feasible_speed_ms(lim, radius_m)
    if not is_positive_speed(v_max):
        return None, None
    which = "倾角预算" if v_lean <= v_yaw else "偏航能力"
    return 1.0 - speed_ms / v_max, which


def is_positive_speed(v):
    return v is not None and v > 1e-6 and v != float("inf")


def max_feasible_speed_ms(lim, radius_m):
    """当前半径下能通过判定的最大速度，同样两项取较严者。"""
    v_lean = arcnav.max_speed_for_radius_ms(lim, radius_m)
    v_yaw = float("inf")
    if arcnav.is_positive(lim.yaw_rate_max_rads):
        v_yaw = radius_m * lim.yaw_rate_max_rads * arcnav.YAW_RATE_FRACTION
    return min(v_lean, v_yaw), v_lean, v_yaw


def check_mission(items, lim, speed_ms, fence_cfg=None):
    """遍历任务，返回 [TurnFinding]。

    fence_cfg 给了就逐条弧过一遍围栏；不给则该项不判，报告里也不会显示"通过"。
    """
    # 局部坐标以任务首项为原点。围栏的圆形项判的是**到 home 的距离**
    # （AC_Fence.cpp:891 用 AP::ahrs().get_home()），而 .waypoints 的 seq 0
    # 就是 home，所以原点即 home，两者对齐。
    ne = inputs.to_local_ne(items)
    # 只有 NAV 命令占位置；改速度、开喷之类夹在中间不构成航段端点。
    nav = [(i, it, ne[i]) for i, it in enumerate(items) if it.is_nav]

    findings = []
    for k, (idx, it, _pos) in enumerate(nav):
        if it.command != inputs.MAV_CMD_NAV_LOITER_TURNS:
            continue
        if k == 0:
            # 任务第一项就是 LOITER_TURNS：没有前一段航段，飞控那条
            # set_next_wp 的路径也不会走到，跳过并说明。
            continue

        r_eff, r_plan, quantised, _ccw = inputs.loiter_turns_radius_m(it)
        sweep = inputs.loiter_turns_sweep_rad(it)
        feasible, lead_in_m, reason = arcnav.plan_feasible(lim, r_eff, speed_ms)

        entry_idx, _entry_it, entry_pos = nav[k - 1]
        prev_leg_m = inputs.dist(nav[k - 2][2], entry_pos) if k >= 2 else 0.0

        f = TurnFinding(
            seq=it.seq, entry_seq=items[entry_idx].seq,
            radius_planned_m=r_plan, radius_effective_m=r_eff,
            radius_quantised=quantised, sweep_rad=sweep, speed_ms=speed_ms,
            feasible=feasible, reason=reason, lead_in_m=lead_in_m,
            prev_leg_m=prev_leg_m)

        if quantised:
            f.notes.append(
                "任务里写的半径 %.4g m，机上按 %.4g m 执行——AP_Mission 把它存成整数米"
                "（AP_Mission.cpp:1122，浮点转整数是截断）。按写的值校验会偏乐观，"
                "本报告用的是生效值。" % (r_plan, r_eff))

        # 行距：转弯把飞机横向挪了多少，就是它连接的两行的间距。
        # 从任务坐标量出来，不假设它等于 2×半径。
        if k >= 2 and k + 1 < len(nav):
            f.row_spacing_m = _perp_distance(nav[k + 1][2], nav[k - 2][2], entry_pos)

        # 围栏：整条弧按 5° 采样过一遍，与机上 Mode::arc_within_fence() 同法。
        # LOITER_TURNS 的位置是**圆心**（mode_auto.cpp 的 loc_from_cmd），
        # 入弧点是前一个 NAV 航点，两者定出起始方位角。
        if fence_cfg is not None:
            f.fence = fence_mod.check_arc(
                fence_cfg, home_ne=ne[0], centre_ne=_pos,
                start_ne=entry_pos, radius_m=r_eff, sweep_rad=sweep,
                alt_m=it.z)
            if f.fence.breached:
                name = fence_mod.TYPE_NAMES.get(f.fence.breach_type, "围栏")
                f.notes.append(
                    "整条弧有一段在%s围栏外（最深越界 %.3g m，出现在扫掠 %.0f° 处）。"
                    "机上 Mode::arc_within_fence() 也会拒绝它，但**拒绝拦不住越界**——"
                    "退回的普通盘旋路径同样不受围栏约束，实测拒绝后 0.7 s 仍报越界"
                    "（d77cb9757b）。所以这条必须在派工前改掉。"
                    % (name, -f.fence.worst_margin_m, f.fence.worst_at_deg or 0.0))
                f.remedies.append(
                    "把掉头挪进栏内，或放宽围栏——注意围栏还要另留刹车距离，"
                    "AUTO 越界后实测冲出 5.9 m（基线 §7）")

        if not feasible:
            _add_infeasible_remedies(f, lim, r_eff, speed_ms)
        else:
            f.margin_pct, f.margin_which = constraint_margin(lim, r_eff, speed_ms)
            _add_row_skip_remedy(f, lim, r_eff, speed_ms,
                                 required_radius_m(lim, speed_ms)[0])
            # 空洞 C：判定给了 lead_in 这个**要求**，但飞控里没有任何地方
            # 检查前一段航段是否满足它。这里检查。
            if k >= 2 and prev_leg_m < lead_in_m:
                f.lead_in_satisfied = False
                _add_lead_in_remedies(f, lim, r_eff, speed_ms, prev_leg_m)
            elif k < 2:
                f.notes.append(
                    "入弧点前只有一个 NAV 航点，量不出前段长度，未校验 lead-in 要求 "
                    "%.4g m。" % lead_in_m)

        findings.append(f)
    return findings


def violated_constraints(lim, radius_m, speed_ms):
    """本次超标的**全部**约束，不受 C++ 短路顺序影响。

    plan_feasible() 一碰到超标就返回，所以它只说得出第一项。给改法必须知道全部：
    两项都超时按第一项算出的"降到多少"，在另一项上仍然不可行——给错的数比不给
    更坏，现场会照着改然后照样飞不出来。
    """
    out = []
    if speed_ms > arcnav.max_speed_for_radius_ms(lim, radius_m):
        out.append("lean")
    if arcnav.is_positive(lim.yaw_rate_max_rads):
        if speed_ms / radius_m > lim.yaw_rate_max_rads * arcnav.YAW_RATE_FRACTION:
            out.append("yaw_rate")
    return out


def _add_infeasible_remedies(f, lim, radius_m, speed_ms):
    r_req, r_lean, r_yaw = required_radius_m(lim, speed_ms)
    v_max, v_lean, v_yaw = max_feasible_speed_ms(lim, radius_m)

    f.violations = violated_constraints(lim, radius_m, speed_ms)
    if len(f.violations) > 1:
        f.notes.append(
            "倾角与偏航两项同时超标。机上只会报出先判的那一项（倾角），"
            "但决定改到多少的是较严的那一项——下面的改法按较严者给。")

    if v_max > 0:
        binding = "倾角预算" if v_lean <= v_yaw else "偏航能力"
        f.remedies.append(
            "速度降到 %.3g m/s 以下（当前 %.3g；倾角允许 %.3g、偏航允许 %.3g，"
            "%s 顶住）" % (v_max, speed_ms, v_lean, v_yaw, binding))
    if r_req > 0:
        binding_r = "倾角预算" if r_lean >= r_yaw else "偏航能力"
        f.remedies.append(
            "或速度保持 %.3g m/s，半径开到 %.3g m 以上"
            "（倾角要 %.3g m、偏航要 %.3g m，%s 顶住）"
            % (speed_ms, r_req, r_lean, r_yaw, binding_r))

    _add_row_skip_remedy(f, lim, radius_m, speed_ms, r_req)

    # 从偏航能力反推的掉头，作为"这台机器在这个速度下最紧能做多紧"的参考。
    plan = arcnav.plan_from_yaw_capability(
        speed_ms, f.sweep_rad or math.pi,
        lim.yaw_rate_max_rads * arcnav.YAW_RATE_FRACTION, lim.yaw_accel_max_radss)
    if plan.radius_m > 0:
        f.notes.append(
            "按偏航能力反推：该速度下最紧可做 R=%.3g m，过渡 %.3g m，掉头用时 %.3g s"
            % (plan.radius_m, plan.spiral_len_m, plan.duration_s))


def _add_row_skip_remedy(f, lim, radius_m, speed_ms, r_req):
    """跳行建议，连同它到台阶边缘的距离。

    跳行数是 ceil(R_req / (行距/2))，**ceil 把连续量压成了台阶**。后果是参数
    改动常常一行也省不掉：现场那份参数、行距 5 m 下，偏航上限从 50 提到 59.1 °/s
    （+18%）在 3/4/5 m/s 三档上跳行数分别仍是 3/4/5 行，一行未省。而"离台阶边缘
    0.1 m"和"离 2 m"是完全不同的处境——前者一次微调就翻档。所以只报 R_min 现场
    不知道该干什么，必须同时报在台阶上的位置。

    真正能少跳一行的是降速：把速度降到 (k-1) 行对应半径所允许的最大值即可。
    这个数报出来，现场才知道"慢一点"具体是多慢、换回来多少。
    """
    if not f.row_spacing_m or f.row_spacing_m <= 0.1 or r_req <= 0:
        return
    half_w = f.row_spacing_m / 2.0
    k = max(1, math.ceil(r_req / half_w - 1e-9))
    f.rows_needed = k
    f.rows_skipped = max(1, int(round(radius_m / half_w)))
    # 还能再吃多少 R_req 才需要多跳一行
    f.step_margin_m = k * half_w - r_req
    if k <= 1:
        return

    f.remedies.append(
        "或跳行：行距 %.3g m 下要掉到第 %d 行（半径 %.3g m），当前是第 %d 行；"
        "距离「还得多跳一行」还有 %.3g m 余量"
        % (f.row_spacing_m, k, k * half_w, f.rows_skipped, f.step_margin_m))

    # 少跳一行需要慢到多少
    if k >= 2:
        v_one_less, _vl, _vy = max_feasible_speed_ms(lim, (k - 1) * half_w)
        if is_positive_speed(v_one_less) and v_one_less < speed_ms:
            f.remedies.append(
                "或降速换少跳一行：%.3g m/s 时第 %d 行就够（当前 %.3g m/s 要跳到第 %d 行）。"
                "跳行数是取整的，所以提高偏航限幅未必换得来少跳一行——要跨过台阶才算数，"
                "上面那个「距下一档 %.3g m」就是台阶还有多远。"
                % (v_one_less, k - 1, speed_ms, k, f.step_margin_m))


def _add_lead_in_remedies(f, lim, radius_m, speed_ms, prev_leg_m):
    spiral_m = arcnav.required_spiral_len_m(lim, radius_m, speed_ms)
    f.notes.append(
        "转弯本身可飞，但它要求前一段航段不短于 %.3g m 且在这段距离上不减速"
        "（过渡 %.3g m + 前视 %.3g m），实际只有 %.3g m。"
        "plan_feasible() 会返回可行——它拿到的是一个转弯，不是整条任务，"
        "无从检查这一条（AC_ArcNav.cpp:206）。"
        % (f.lead_in_m, spiral_m, speed_ms * lim.heading_lead_s, prev_leg_m))
    f.remedies.append("把入弧点前的航段加长到 %.3g m 以上（还差 %.3g m）"
                      % (f.lead_in_m, f.lead_in_m - prev_leg_m))

    # 降速同时缩短过渡与前视，是不加长航段时唯一的办法。二分求最大可行速度。
    lo, hi = 0.0, speed_ms
    for _ in range(60):
        mid = (lo + hi) / 2.0
        ok, lead, _ = arcnav.plan_feasible(lim, radius_m, mid)
        if ok and lead <= prev_leg_m:
            lo = mid
        else:
            hi = mid
    if lo > 0.05:
        f.remedies.append("或速度降到 %.3g m/s（此时 lead-in 降到 %.3g m，前段够用）"
                          % (lo, arcnav.plan_feasible(lim, radius_m, lo)[1]))
    else:
        f.remedies.append("或改用更大半径——该半径下没有任何速度能让 lead-in 装进 "
                          "%.3g m 的前段" % prev_leg_m)
