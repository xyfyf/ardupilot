"""围栏校验：整条弧是否落在栏内。

这一条为什么必须在地面做，`d77cb9757b` 的提交说明自己写了：

> 一个圆心加半径本身就跨出围栏的 LOITER_TURNS 航点，根本不可能在栏内飞完，
> **正确的处理是在任务上传时拒绝而不是飞到那里再说**。记入待办。

而且它记了机上那道检查「必要但不充分」：`Mode::arc_within_fence()` 拒绝之后，AUTO
退回的普通盘旋路径**同样不受围栏约束**，实测拒绝后 0.7 s 仍报 Polygon fence breached。
所以机上那道拦不住越界，只能换一条路径越界；**地面这道才是唯一能真正阻止它的地方**。

采样方式与 `Mode::arc_within_fence()`（`ArduCopter/mode.cpp`）逐点一致：每 5° 扫掠
一个采样点、步数夹在 [8, 360]、判定用同一组 `FENCE_*`。不一致的话，地面说"栏内"
而机上判"栏外"（或反过来），两边谁也不知道该信谁。
"""

import math
from dataclasses import dataclass, field

# AC_Fence.h:15-18
FENCE_TYPE_ALT_MAX = 1
FENCE_TYPE_CIRCLE = 2
FENCE_TYPE_POLYGON = 4
FENCE_TYPE_ALT_MIN = 8

TYPE_NAMES = {
    FENCE_TYPE_ALT_MAX: "高度上限",
    FENCE_TYPE_CIRCLE: "圆形",
    FENCE_TYPE_POLYGON: "多边形",
    FENCE_TYPE_ALT_MIN: "高度下限",
}

# mode.cpp:arc_within_fence —— 每 5° 一个采样点，步数夹在 [8, 360]
SAMPLE_STEP_RAD = math.radians(5.0)
SAMPLE_STEPS_MIN = 8
SAMPLE_STEPS_MAX = 360


@dataclass
class FenceConfig:
    """从 `.param` 解出的围栏配置。

    多边形顶点**不在参数文件里**——它们经 MAVLink 上传、存在飞控里，所以
    `polygon` 只能由调用方另行提供。给不出时不能当作"没有多边形围栏"。
    """
    enable: bool
    configured_types: int          # FENCE_TYPE
    radius_m: float                # FENCE_RADIUS
    alt_max_m: float               # FENCE_ALT_MAX
    alt_min_m: float               # FENCE_ALT_MIN
    margin_m: float                # FENCE_MARGIN
    polygon_ne: list = field(default_factory=list)   # [(北, 东)] 局部米；空=未提供

    @property
    def polygon_configured(self):
        return bool(self.configured_types & FENCE_TYPE_POLYGON)

    @property
    def polygon_available(self):
        return len(self.polygon_ne) >= 3

    def enabled_types(self):
        """镜像 AC_Fence::get_enabled_fences()。

            FENCE_ENABLE=0            → 0，一律不查
            FENCE_ENABLE=1            → _enabled_fences = FENCE_TYPE & ~ALT_MIN
            get_enabled_fences()      = _enabled_fences & present()
            present()                 = FENCE_TYPE & (CIRCLE|ALT_MIN|ALT_MAX)
                                        多边形另需顶点已加载

        两处合起来的净效果：**ALT_MIN 永远不参与**。
        `check_destination_within_fence()` 里那个 ALT_MIN 分支经这条路进不来——
        它被 `& ~AC_FENCE_TYPE_ALT_MIN`（AC_Fence.cpp:174 与 :236）先摘掉了。
        照着源码逐字读会以为下限也在管，实际不管。
        """
        if not self.enable:
            return 0
        enabled = self.configured_types & ~FENCE_TYPE_ALT_MIN
        present = self.configured_types & (FENCE_TYPE_CIRCLE | FENCE_TYPE_ALT_MIN
                                           | FENCE_TYPE_ALT_MAX)
        if self.polygon_available:
            present |= FENCE_TYPE_POLYGON
        return enabled & present


@dataclass
class FenceVerdict:
    checked_types: int = 0         # 实际查了哪几类
    unverifiable: list = field(default_factory=list)  # 想查但缺数据的
    breached: bool = False
    breach_type: int = 0
    worst_margin_m: float = None   # 离边界最近还有多少米（负=越界多少）
    worst_at_deg: float = None     # 出现在扫掠的第几度
    notes: list = field(default_factory=list)

    @property
    def conclusive(self):
        """结论是否完整。缺多边形顶点时不完整——那时"未发现越界"不等于"栏内"。"""
        return not self.unverifiable


def _point_in_polygon(pt, poly):
    """射线法。与 AC_PolyFence_loader::breached() 的边界取舍未必逐位一致，
    因此边界附近的判定以余量报出，不做"刚好压线"的强断言。"""
    n, e = pt
    inside = False
    j = len(poly) - 1
    for i in range(len(poly)):
        ni, ei = poly[i]
        nj, ej = poly[j]
        if (ei > e) != (ej > e):
            if n < (nj - ni) * (e - ei) / (ej - ei) + ni:
                inside = not inside
        j = i
    return inside


def _dist_to_polygon_edge(pt, poly):
    """点到多边形边界的最短距离，用于报余量。"""
    best = float("inf")
    for i in range(len(poly)):
        a = poly[i]
        b = poly[(i + 1) % len(poly)]
        dn, de = b[0] - a[0], b[1] - a[1]
        seg2 = dn * dn + de * de
        if seg2 < 1e-9:
            best = min(best, math.hypot(pt[0] - a[0], pt[1] - a[1]))
            continue
        t = max(0.0, min(1.0, ((pt[0] - a[0]) * dn + (pt[1] - a[1]) * de) / seg2))
        best = min(best, math.hypot(pt[0] - (a[0] + t * dn), pt[1] - (a[1] + t * de)))
    return best


def check_arc(cfg, home_ne, centre_ne, start_ne, radius_m, sweep_rad, alt_m):
    """整条弧过一遍围栏。坐标为以 home 为原点的局部 NE 米。

    采样与 `Mode::arc_within_fence()` 一致：起点方位角起算，每 5° 一点。

    **采的是理想圆。** 与机上一样，回旋过渡段并不严格落在这个圆上（`d77cb9757b`
    的"未做"里点了这一条）。所以余量很薄时，理想圆判"栏内"不等于实飞不出栏。
    """
    v = FenceVerdict()
    types = cfg.enabled_types()

    if cfg.polygon_configured and not cfg.polygon_available:
        # 说出来，而不是当作没有多边形围栏。缺顶点时"未发现越界"是个错觉。
        v.unverifiable.append(
            "FENCE_TYPE 里开了多边形围栏，但顶点不在参数文件里（经 MAVLink 上传、"
            "存在飞控中）。本次未校验多边形，**不能据此认为这条弧在栏内**。"
            "要校验请另行提供围栏顶点。")

    if types == 0 and not v.unverifiable:
        v.notes.append("按 FENCE_ENABLE / FENCE_TYPE，本次没有任何生效的围栏。")
        return v
    v.checked_types = types

    start_rel = (start_ne[0] - centre_ne[0], start_ne[1] - centre_ne[1])
    if math.hypot(*start_rel) < 0.01:
        v.notes.append("入弧点与圆心重合，定不出起始方位角，未采样。")
        return v
    th0 = math.atan2(start_rel[1], start_rel[0])

    steps = int(abs(sweep_rad) / SAMPLE_STEP_RAD)
    steps = max(SAMPLE_STEPS_MIN, min(SAMPLE_STEPS_MAX, steps))

    # 高度类先判，它与采样点无关
    if types & FENCE_TYPE_ALT_MAX and alt_m is not None:
        if alt_m > cfg.alt_max_m:
            v.breached, v.breach_type = True, FENCE_TYPE_ALT_MAX
            v.worst_margin_m = cfg.alt_max_m - alt_m
            return v

    worst = float("inf")
    worst_at = None
    for i in range(steps + 1):
        th = th0 + sweep_rad * (i / steps)
        pt = (centre_ne[0] + radius_m * math.cos(th),
              centre_ne[1] + radius_m * math.sin(th))
        deg = math.degrees(abs(sweep_rad)) * (i / steps)

        if types & FENCE_TYPE_CIRCLE:
            d = math.hypot(pt[0] - home_ne[0], pt[1] - home_ne[1])
            margin = cfg.radius_m - d
            if margin < worst:
                worst, worst_at = margin, deg
            if d > cfg.radius_m and not v.breached:
                v.breached, v.breach_type = True, FENCE_TYPE_CIRCLE

        if types & FENCE_TYPE_POLYGON:
            inside = _point_in_polygon(pt, cfg.polygon_ne)
            d_edge = _dist_to_polygon_edge(pt, cfg.polygon_ne)
            margin = d_edge if inside else -d_edge
            if margin < worst:
                worst, worst_at = margin, deg
            if not inside and not v.breached:
                v.breached, v.breach_type = True, FENCE_TYPE_POLYGON

    if worst != float("inf"):
        v.worst_margin_m = worst
        v.worst_at_deg = worst_at

    return v


def from_params(params, polygon_ne=None):
    """`.param` → FenceConfig。默认值取自 AC_Fence 的参数表。"""
    return FenceConfig(
        enable=bool(params.get("FENCE_ENABLE", 0)),
        configured_types=int(params.get("FENCE_TYPE", 7)),
        radius_m=params.get("FENCE_RADIUS", 300.0),
        alt_max_m=params.get("FENCE_ALT_MAX", 100.0),
        alt_min_m=params.get("FENCE_ALT_MIN", -10.0),
        margin_m=params.get("FENCE_MARGIN", 10.0),
        polygon_ne=list(polygon_ne or []),
    )
