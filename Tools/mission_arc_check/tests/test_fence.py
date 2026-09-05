"""围栏校验的测试。

重点不在"栏外能不能判出来"——那是显然的。重点在**结论不完整时不能报通过**：
多边形顶点不在参数文件里，缺了它而报"栏内"，就是这条工具线一路在防的那种
"已经检查过"的错觉。
"""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import check   # noqa: E402
import fence   # noqa: E402


def cfg(enable=True, types=fence.FENCE_TYPE_CIRCLE, radius=100.0,
        alt_max=100.0, polygon=None):
    return fence.FenceConfig(
        enable=enable, configured_types=types, radius_m=radius,
        alt_max_m=alt_max, alt_min_m=-10.0, margin_m=10.0,
        polygon_ne=list(polygon or []))


def arc(cfg_, centre=(60.0, 6.0), start=(60.0, 0.0), r=6.0,
        sweep=math.pi, alt=15.0, home=(0.0, 0.0)):
    """默认几何 = run_uturn_auto --swath 12：作业线沿北 0→60 m，圆心在 (60, 6)。"""
    return fence.check_arc(cfg_, home_ne=home, centre_ne=centre, start_ne=start,
                           radius_m=r, sweep_rad=sweep, alt_m=alt)


class EnabledTypes(unittest.TestCase):
    """镜像 AC_Fence::get_enabled_fences()。查错了类型 = 判的是另一件事。"""

    def test_fence_disabled_checks_nothing(self):
        self.assertEqual(0, cfg(enable=False, types=7).enabled_types())

    def test_alt_min_is_never_enabled(self):
        """FENCE_TYPE 里开了下限也不查——AC_Fence.cpp:174 把它摘掉了。

        check_destination_within_fence() 里那个 ALT_MIN 分支照字面读像是在管，
        实际经这条路进不来。这条钉住，免得地面按源码字面实现成"下限也查"。
        """
        c = cfg(types=fence.FENCE_TYPE_ALT_MIN | fence.FENCE_TYPE_CIRCLE)
        self.assertEqual(fence.FENCE_TYPE_CIRCLE, c.enabled_types())

    def test_polygon_needs_points_to_count_as_present(self):
        """没有顶点时多边形不算 present——与 AC_Fence::present() 一致。"""
        c = cfg(types=fence.FENCE_TYPE_POLYGON)
        self.assertEqual(0, c.enabled_types())
        c.polygon_ne = [(0, 0), (100, 0), (100, 100), (0, 100)]
        self.assertEqual(fence.FENCE_TYPE_POLYGON, c.enabled_types())


class MissingPolygonIsNotAPass(unittest.TestCase):
    """缺顶点时"没查出越界"必须不等于"栏内"。"""

    def test_verdict_is_flagged_inconclusive(self):
        v = arc(cfg(types=fence.FENCE_TYPE_POLYGON | fence.FENCE_TYPE_CIRCLE,
                    radius=1000.0))
        self.assertFalse(v.breached, "圆形围栏很大，本身不该越界")
        self.assertFalse(v.conclusive, "多边形没查，结论不完整")
        self.assertTrue(any("不能据此认为" in u for u in v.unverifiable), v.unverifiable)

    def test_inconclusive_fence_makes_the_turn_not_ok(self):
        """结论不完整要一路传到 TurnFinding.ok，否则报告顶上仍显示"可行"。"""
        f = check.TurnFinding(
            seq=4, entry_seq=3, radius_planned_m=6.0, radius_effective_m=6.0,
            radius_quantised=False, sweep_rad=math.pi, speed_ms=3.0, feasible=True)
        self.assertTrue(f.ok, "先确认没有围栏时它是通过的")
        f.fence = arc(cfg(types=fence.FENCE_TYPE_POLYGON, radius=1000.0))
        self.assertFalse(f.ok, "结论不完整时不能算通过")


class CircleFence(unittest.TestCase):

    def test_arc_well_inside_passes_and_reports_margin(self):
        v = arc(cfg(radius=100.0))
        self.assertFalse(v.breached)
        self.assertTrue(v.conclusive)
        # 弧最远点约在 (66, 6)，离 home 约 66.3 m，余量约 33.7 m
        self.assertAlmostEqual(33.7, v.worst_margin_m, delta=0.5)

    def test_arc_poking_out_is_caught(self):
        """围栏北界压到 62 m —— d77cb9757b 的 SITL 对照用的就是这一档。

        弧顶在 N=66 m，所以必然出栏。那次实测机上判"拒绝"，本工具也该判越界。
        """
        v = arc(cfg(radius=62.0))
        self.assertTrue(v.breached)
        self.assertEqual(fence.FENCE_TYPE_CIRCLE, v.breach_type)
        self.assertLess(v.worst_margin_m, 0.0)
        self.assertAlmostEqual(-4.3, v.worst_margin_m, delta=0.5)

    def test_sampling_matches_the_vehicle(self):
        """步数公式与 mode.cpp 一致：clamp(|sweep|/5°, 8, 360)。

        步数不同 = 采样点不同 = 掠过边界的那一小段可能被跳过，
        于是地面说栏内、机上说栏外。
        """
        for sweep_deg, want in ((180, 36), (10, 8), (3600, 360), (30, 8)):
            steps = int(math.radians(sweep_deg) / fence.SAMPLE_STEP_RAD)
            steps = max(fence.SAMPLE_STEPS_MIN, min(fence.SAMPLE_STEPS_MAX, steps))
            self.assertEqual(want, steps, "扫掠 %d° 时步数应为 %d" % (sweep_deg, want))


class AltFence(unittest.TestCase):

    def test_arc_above_ceiling_is_caught(self):
        v = arc(cfg(types=fence.FENCE_TYPE_ALT_MAX, alt_max=10.0), alt=15.0)
        self.assertTrue(v.breached)
        self.assertEqual(fence.FENCE_TYPE_ALT_MAX, v.breach_type)

    def test_arc_below_ceiling_passes(self):
        v = arc(cfg(types=fence.FENCE_TYPE_ALT_MAX, alt_max=30.0), alt=15.0)
        self.assertFalse(v.breached)


class PolygonFence(unittest.TestCase):

    SQUARE = [(-20.0, -20.0), (80.0, -20.0), (80.0, 80.0), (-20.0, 80.0)]

    def test_arc_inside_polygon_passes(self):
        v = arc(cfg(types=fence.FENCE_TYPE_POLYGON, polygon=self.SQUARE))
        self.assertFalse(v.breached)
        self.assertTrue(v.conclusive)
        self.assertGreater(v.worst_margin_m, 0.0)

    def test_arc_crossing_the_north_edge_is_caught(self):
        tight = [(-20.0, -20.0), (62.0, -20.0), (62.0, 80.0), (-20.0, 80.0)]
        v = arc(cfg(types=fence.FENCE_TYPE_POLYGON, polygon=tight))
        self.assertTrue(v.breached)
        self.assertEqual(fence.FENCE_TYPE_POLYGON, v.breach_type)


class NoFenceConfigured(unittest.TestCase):

    def test_says_so_rather_than_silently_passing(self):
        v = arc(cfg(enable=False))
        self.assertFalse(v.breached)
        self.assertTrue(v.conclusive)
        self.assertTrue(any("没有任何生效的围栏" in n for n in v.notes), v.notes)


if __name__ == "__main__":
    unittest.main(verbosity=2)
