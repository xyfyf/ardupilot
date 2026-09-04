"""遍历与判定的单元测试。

这一层测的不是那套数学（那由 test_crosscheck.py 对着 C++ 比），而是**工具自己
加的东西**：任务解析、半径量化、前段长度、行距推算、以及"改成什么可行"。
这些在 C++ 里没有对应物，所以只能在这里钉住。
"""

import math
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import arcnav    # noqa: E402
import check     # noqa: E402
import inputs    # noqa: E402
import mission_arc_check  # noqa: E402

HOME_LAT, HOME_LON = 35.363261, 149.165230
EARTH_R = 6378137.0


def ne_to_latlon(n, e):
    lat = HOME_LAT + math.degrees(n / EARTH_R)
    lon = HOME_LON + math.degrees(e / (EARTH_R * math.cos(math.radians(HOME_LAT))))
    return lat, lon


def write_uturn_mission(path, leg_m, swath_m, turns=0.5, radius_m=None):
    """与 reproduce.py 的 run_uturn_auto 同几何：作业线 -> LOITER_TURNS -> 下一行。

    半径可单独给，用来构造"任务里写的半径"与行距不一致的情形。
    """
    R = swath_m / 2.0 if radius_m is None else radius_m
    a = ne_to_latlon(0.0, 0.0)
    b = ne_to_latlon(leg_m, 0.0)
    c = ne_to_latlon(leg_m, swath_m / 2.0)      # 圆心
    d = ne_to_latlon(0.0, swath_m)              # 下一行终点
    rows = [
        (16, 0, 0, 0, 0, HOME_LAT, HOME_LON, 0.0),
        (22, 0, 0, 0, 0, HOME_LAT, HOME_LON, 15.0),
        (16, 0, 0, 0, 0, a[0], a[1], 15.0),
        (16, 0, 0, 0, 0, b[0], b[1], 15.0),
        (18, turns, 1.0, R, 0.0, c[0], c[1], 15.0),
        (16, 0, 0, 0, 0, d[0], d[1], 15.0),
        (21, 0, 0, 0, 0, HOME_LAT, HOME_LON, 0.0),
    ]
    with open(path, "w") as fh:
        fh.write("QGC WPL 110\n")
        for seq, r in enumerate(rows):
            fh.write("%d\t%d\t3\t%d\t%.8f\t%.8f\t%.8f\t%.8f\t%.8f\t%.8f\t%.6f\t1\n"
                     % (seq, 1 if seq == 0 else 0, r[0], r[1], r[2], r[3], r[4],
                        r[5], r[6], r[7]))
    return path


def sitl_limits(yaw_rate_degs=57.0):
    """与 eft_hexa.parm 一致：PSC_ANGLE_MAX=20°、PSC_JERK_XY 默认 5.0。"""
    return arcnav.Limits(
        lean_angle_max_rad=math.radians(20.0),
        jerk_ne_msss=5.0,
        yaw_rate_max_rads=math.radians(yaw_rate_degs),
        yaw_accel_max_radss=math.radians(270.0),   # ATC_ACCEL_Y_MAX 默认 27000 cdeg/s²
        heading_lead_s=arcnav.HEADING_LEAD_S)


class MissionParsing(unittest.TestCase):

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()

    def tearDown(self):
        self.tmp.cleanup()

    def path(self, name="m.waypoints"):
        return os.path.join(self.tmp.name, name)

    def test_rejects_non_wpl_file(self):
        p = self.path()
        with open(p, "w") as fh:
            fh.write("这不是任务文件\n")
        with self.assertRaises(inputs.InputError):
            inputs.load_mission(p)

    def test_radius_is_truncated_the_way_ap_mission_truncates_it(self):
        """行距 5 m 时半径 2.5 m，机上按 2 m 飞。

        这不是舍入误差，是 AP_Mission.cpp:1124 的 `uint8_t radius_m = abs_radius`
        —— C 的浮点转整数是截断。按 2.5 校验会偏乐观：真正飞的弯更紧，偏航需求
        高 25%。
        """
        items = inputs.load_mission(write_uturn_mission(self.path(), 60.0, 5.0))
        lt = [it for it in items if it.command == inputs.MAV_CMD_NAV_LOITER_TURNS][0]
        eff, planned, quantised, _ccw = inputs.loiter_turns_radius_m(lt)
        self.assertEqual(2.5, planned)
        self.assertEqual(2.0, eff)
        self.assertTrue(quantised)

    def test_large_radius_uses_the_times_ten_path(self):
        items = inputs.load_mission(
            write_uturn_mission(self.path(), 600.0, 600.0, radius_m=300.0))
        lt = [it for it in items if it.command == inputs.MAV_CMD_NAV_LOITER_TURNS][0]
        eff, planned, _q, _ccw = inputs.loiter_turns_radius_m(lt)
        self.assertEqual(300.0, planned)
        self.assertEqual(300.0, eff)

    def test_sweep_comes_from_turn_count(self):
        items = inputs.load_mission(write_uturn_mission(self.path(), 60.0, 12.0))
        lt = [it for it in items if it.command == inputs.MAV_CMD_NAV_LOITER_TURNS][0]
        self.assertAlmostEqual(math.pi, inputs.loiter_turns_sweep_rad(lt), places=6)


class Traversal(unittest.TestCase):

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()

    def tearDown(self):
        self.tmp.cleanup()

    def path(self, name="m.waypoints"):
        return os.path.join(self.tmp.name, name)

    def test_wide_turn_at_working_speed_is_feasible(self):
        """R=6 m、3 m/s——回归里 run_uturn_auto --swath 12 的那条，实飞可行。"""
        items = inputs.load_mission(write_uturn_mission(self.path(), 60.0, 12.0))
        # 偏航能力取实测区间上沿 59.1：需求 v/r=28.6°/s，预算 29.55°/s
        f = check.check_mission(items, sitl_limits(59.1), 3.0)
        self.assertEqual(1, len(f))
        self.assertTrue(f[0].feasible, f[0].reason)
        self.assertTrue(f[0].lead_in_satisfied)

    def test_same_turn_fails_at_the_low_end_of_the_measured_band(self):
        """同一条弯，偏航能力取实测区间下沿 54.7 就判不可行。

        需求 28.6 °/s，预算 27.35 °/s。这条弯坐在实测区间的两侧——不是工具的
        毛病，是这条航线本来就没有余量。真机偏航辨识的必要性就在这里：区间宽
        25% 时，判定结论在区间内会翻转。
        """
        items = inputs.load_mission(write_uturn_mission(self.path(), 60.0, 12.0))
        f = check.check_mission(items, sitl_limits(54.7), 3.0)
        self.assertFalse(f[0].feasible)
        self.assertEqual("yaw_rate", f[0].reason)

    def test_tight_turn_is_refused_and_both_violations_are_named(self):
        """行距 5 m（半径量化后 2 m）、3 m/s：倾角与偏航**同时**超标。

        倾角预算 20°×70%=14°，允许 2.21 m/s；偏航预算 57°/s×50%=28.5°/s，
        允许 0.995 m/s。C++ 先判倾角，所以机上报的是倾角——报告要与机上一致，
        但不能只按倾角给改法：降到 2.21 m/s 在偏航上仍然飞不出来。
        """
        f = check.check_mission(
            inputs.load_mission(write_uturn_mission(self.path(), 60.0, 5.0)),
            sitl_limits(57.0), 3.0)[0]
        self.assertFalse(f.feasible)
        self.assertEqual("lean", f.reason, "应与 C++ 的检查顺序一致")
        self.assertEqual(["lean", "yaw_rate"], f.violations)
        self.assertTrue(f.radius_quantised)
        self.assertTrue(any("速度降到" in r for r in f.remedies), f.remedies)
        self.assertTrue(any("半径开到" in r for r in f.remedies), f.remedies)
        self.assertTrue(any("跳行" in r for r in f.remedies), f.remedies)

    def test_recommended_speed_satisfies_every_constraint_not_just_the_first(self):
        """回归测试：两项同时超标时，建议速度曾只按先判的那项算，仍不可行。

        给错的数比不给更坏——现场会照着改，然后照样飞不出来，而且会认为"已经
        按报告改过了"。
        """
        lim = sitl_limits(57.0)
        f = check.check_mission(
            inputs.load_mission(write_uturn_mission(self.path(), 60.0, 5.0)),
            lim, 3.0)[0]
        v_max, v_lean, v_yaw = check.max_feasible_speed_ms(lim, f.radius_effective_m)
        self.assertLess(v_yaw, v_lean, "本例应当是偏航更严，否则测不到那个 bug")
        self.assertEqual(v_yaw, v_max)
        self.assertTrue(arcnav.plan_feasible(lim, f.radius_effective_m, v_max * 0.999)[0])
        # 报告里印的就是这个数
        self.assertTrue(any("%.3g" % v_max in r for r in f.remedies), f.remedies)

    def test_recommended_radius_satisfies_every_constraint(self):
        lim = sitl_limits(57.0)
        r_req, r_lean, r_yaw = check.required_radius_m(lim, 3.0)
        self.assertEqual(max(r_lean, r_yaw), r_req)
        self.assertTrue(arcnav.plan_feasible(lim, r_req * 1.001, 3.0)[0])

    def test_short_previous_leg_is_caught_although_the_turn_itself_is_flyable(self):
        """空洞 C：转弯本身可飞，前一段却兑现不了 lead-in。

        plan_feasible() 对这种情形返回 true——它拿到的是一个转弯而不是整条任务，
        无从检查。这正是把校验挪到地面、按任务遍历才能发现的那一类。
        """
        lim = sitl_limits(59.1)
        # 作业段压到 2 m：转弯（R=6 m、3 m/s）本身可飞，但 lead-in 要 3.0 m
        # （过渡 1.5 m + 前视 1.5 m）。取 3.0 会正好压在等号上，测不出方向。
        items = inputs.load_mission(write_uturn_mission(self.path(), 2.0, 12.0))
        f = check.check_mission(items, lim, 3.0)[0]
        self.assertTrue(f.feasible, "转弯本身应当可飞，否则测的不是这条")
        self.assertFalse(f.lead_in_satisfied)
        self.assertFalse(f.ok)
        self.assertGreater(f.lead_in_m, f.prev_leg_m)
        self.assertTrue(any("加长" in r for r in f.remedies), f.remedies)

    def test_row_spacing_is_measured_from_geometry_not_assumed(self):
        """行距从任务坐标量，不假设等于 2×半径。

        任务里半径写错、或掉头几何本就不是"半径 = 行距/2"时，假设出来的行距是
        错的，跳行建议会跟着错。"""
        items = inputs.load_mission(
            write_uturn_mission(self.path(), 60.0, 12.0, radius_m=3.0))
        f = check.check_mission(items, sitl_limits(57.0), 3.0)[0]
        self.assertEqual(3.0, f.radius_effective_m)
        self.assertAlmostEqual(12.0, f.row_spacing_m, delta=0.05)

    def test_mission_without_loiter_turns_yields_nothing(self):
        """普通航点转弯走 SCurve，不由 AC_ArcNav 处理，不该被误判。"""
        p = self.path()
        with open(p, "w") as fh:
            fh.write("QGC WPL 110\n")
            for seq, (n, e) in enumerate([(0, 0), (60, 0), (60, 12), (0, 12)]):
                lat, lon = ne_to_latlon(n, e)
                fh.write("%d\t%d\t3\t16\t0\t0\t0\t0\t%.8f\t%.8f\t15.0\t1\n"
                         % (seq, 1 if seq == 0 else 0, lat, lon))
        self.assertEqual([], check.check_mission(
            inputs.load_mission(p), sitl_limits(), 3.0))


class ParamResolution(unittest.TestCase):

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()

    def tearDown(self):
        self.tmp.cleanup()

    def write_params(self, text, name="p.param"):
        p = os.path.join(self.tmp.name, name)
        with open(p, "w") as fh:
            fh.write(text)
        return p

    def test_psc_angle_max_wins_over_angle_max(self):
        p = self.write_params("ANGLE_MAX 3000\nPSC_ANGLE_MAX 20\nWPNAV_SPEED 300\n")
        resolved, speed = inputs.resolve_limits(inputs.load_params(p), 57.0, "测试")
        self.assertAlmostEqual(math.radians(20.0), resolved.limits.lean_angle_max_rad,
                               places=6)
        self.assertAlmostEqual(3.0, speed, places=6)

    def test_falls_back_to_angle_max_when_psc_is_zero(self):
        p = self.write_params("ANGLE_MAX 3000\nPSC_ANGLE_MAX 0\n")
        resolved, _ = inputs.resolve_limits(inputs.load_params(p), 57.0, "测试")
        self.assertAlmostEqual(math.radians(30.0), resolved.limits.lean_angle_max_rad,
                               places=6)

    def test_hole_d_is_reported_when_slew_yaw_exceeds_capability(self):
        """ATC_SLEW_YAW=12000（120°/s）配 57°/s 的机体——回归那条正是如此。"""
        p = self.write_params("ATC_SLEW_YAW 12000\nPSC_ANGLE_MAX 20\n")
        resolved, _ = inputs.resolve_limits(inputs.load_params(p), 57.0, "测试")
        self.assertTrue(any("空洞 D" in w for w in resolved.warnings), resolved.warnings)

    def test_wpnav_jerk_without_psc_jerk_is_flagged(self):
        """过渡长度用 PSC_JERK_XY，改 WPNAV_JERK 不影响转弯——容易搞错，要提示。"""
        p = self.write_params("WPNAV_JERK 4\nPSC_ANGLE_MAX 20\n")
        resolved, _ = inputs.resolve_limits(inputs.load_params(p), 57.0, "测试")
        self.assertAlmostEqual(5.0, resolved.limits.jerk_ne_msss, places=6)
        self.assertTrue(any("WPNAV_JERK" in w for w in resolved.warnings),
                        resolved.warnings)

    def test_provenance_marks_defaults(self):
        p = self.write_params("PSC_ANGLE_MAX 20\n")
        resolved, _ = inputs.resolve_limits(inputs.load_params(p), 57.0, "测试")
        self.assertFalse(resolved.provenance["WPNAV_SPEED"].present)
        self.assertIn("默认", resolved.provenance["WPNAV_SPEED"].note)


class MultipleTurns(unittest.TestCase):
    """现场航线有很多个掉头，不是一个。

    单转弯的测试测不到遍历本身：序号对不对、"前一段航段"取的是不是各自那一段。
    取错的话所有转弯会共用第一段的长度，而报告看上去完全正常。
    """

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()

    def tearDown(self):
        self.tmp.cleanup()

    def write_multi(self, legs, swath_m):
        rows = [(16, 0, 0, 0, 0, HOME_LAT, HOME_LON, 0.0),
                (22, 0, 0, 0, 0, HOME_LAT, HOME_LON, 15.0)]
        for i, leg in enumerate(legs):
            e = i * swath_m
            a = ne_to_latlon(0.0 if i % 2 == 0 else leg, e)
            b = ne_to_latlon(leg if i % 2 == 0 else 0.0, e)
            rows.append((16, 0, 0, 0, 0, a[0], a[1], 15.0))
            rows.append((16, 0, 0, 0, 0, b[0], b[1], 15.0))
            if i < len(legs) - 1:
                c = ne_to_latlon(leg if i % 2 == 0 else 0.0, e + swath_m / 2.0)
                rows.append((18, 0.5, 1.0, swath_m / 2.0, 0.0, c[0], c[1], 15.0))
        rows.append((21, 0, 0, 0, 0, HOME_LAT, HOME_LON, 0.0))
        path = os.path.join(self.tmp.name, "multi.waypoints")
        with open(path, "w") as fh:
            fh.write("QGC WPL 110\n")
            for seq, r in enumerate(rows):
                fh.write("%d\t%d\t3\t%d\t%.8f\t%.8f\t%.8f\t%.8f\t%.8f\t%.8f\t%.6f\t1\n"
                         % (seq, 1 if seq == 0 else 0, r[0], r[1], r[2], r[3],
                            r[4], r[5], r[6], r[7]))
        return path

    def test_every_turn_gets_its_own_previous_leg(self):
        """每个转弯的前段长度取的是它自己那一段，不是共用第一段。"""
        path = self.write_multi([60.0, 60.0, 8.0, 60.0], 12.0)
        fs = check.check_mission(inputs.load_mission(path), sitl_limits(59.1), 3.0)
        self.assertEqual(3, len(fs))
        self.assertEqual([4, 7, 10], [f.seq for f in fs])
        self.assertEqual([3, 6, 9], [f.entry_seq for f in fs])
        legs = [round(f.prev_leg_m) for f in fs]
        self.assertEqual([60, 60, 8], legs, "第三个转弯前面那段只有 8 m")

    def test_one_short_leg_fails_only_its_own_turn(self):
        """把中间一段压到 lead-in 之下，只该判它一个不可行。"""
        path = self.write_multi([60.0, 60.0, 2.0, 60.0], 12.0)
        fs = check.check_mission(inputs.load_mission(path), sitl_limits(59.1), 3.0)
        self.assertEqual([True, True, False], [f.ok for f in fs])
        self.assertTrue(fs[2].feasible, "转弯本身可飞，卡的是前段长度")
        self.assertFalse(fs[2].lead_in_satisfied)


class FieldConfig(unittest.TestCase):
    """现场那份参数（X6100 2026.09.02）下的判定。

    与 SITL 不同的是 ATC_RATE_Y_MAX=50 被显式给了，于是 get_slew_yaw_max_rads()
    取 min(50, ATC_SLEW_YAW=60) = 50 °/s——比 SITL 模型推出的 54.7–59.1 更低。
    也就是说现场参数下的偏航预算是 25 °/s，而不是 SITL 的 27.35–29.55。
    """

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()

    def tearDown(self):
        self.tmp.cleanup()

    def test_regression_route_is_infeasible_under_field_yaw_limit(self):
        """R=6 m、3 m/s 这条在 SITL 下飞得出来，在现场 50 °/s 的限幅下判不可行。

        需求 v/r = 0.5 rad/s = 28.65 °/s，预算 50 × 50% = 25 °/s。
        这不是"勉强不过"，是超了 15%——SITL 的结论不能直接搬到现场。
        """
        lim = sitl_limits(50.0)
        path = os.path.join(self.tmp.name, "m.waypoints")
        f = check.check_mission(
            inputs.load_mission(write_uturn_mission(path, 60.0, 12.0)), lim, 3.0)[0]
        self.assertFalse(f.feasible)
        self.assertEqual("yaw_rate", f.reason)

    def test_binding_constraint_switches_with_speed(self):
        """倾角项 R ∝ v²、偏航项 R ∝ v，所以主导约束会随速度换项。

        低速偏航先卡住，高速倾角先卡住。交点在 v = a_max/ω_budget。
        改法必须按当前速度下的两项分别算再取较严者，不能假定谁主导。
        """
        lim = sitl_limits(50.0)
        a_max = arcnav.GRAVITY_MSS * math.tan(arcnav.tilt_budget_rad(lim))
        omega = math.radians(50.0) * arcnav.YAW_RATE_FRACTION
        v_cross = a_max / omega

        _r, r_lean_lo, r_yaw_lo = check.required_radius_m(lim, v_cross * 0.5)
        self.assertGreater(r_yaw_lo, r_lean_lo, "低速应当偏航主导")
        _r, r_lean_hi, r_yaw_hi = check.required_radius_m(lim, v_cross * 2.0)
        self.assertGreater(r_lean_hi, r_yaw_hi, "高速应当倾角主导")


class RemediesAreActuallyFeasible(unittest.TestCase):
    """扫一遍网格：凡是判不可行的，给出的改法必须真的可行。

    这条比逐例断言强，因为主导约束会随速度和半径换项——手挑的几个例子容易全
    落在同一侧，测不到换项处。
    """

    def test_recommended_speed_and_radius_hold_across_the_grid(self):
        bad = []
        for yaw_degs in (50.0, 54.7, 59.1, 120.0):
            lim = sitl_limits(yaw_degs)
            for radius_m in (1.0, 2.0, 3.0, 5.0, 6.0, 9.0, 12.0, 20.0, 40.0):
                for speed_ms in (1.0, 2.0, 3.0, 4.5, 6.0, 8.0, 10.0):
                    if arcnav.plan_feasible(lim, radius_m, speed_ms)[0]:
                        continue
                    v_max, _, _ = check.max_feasible_speed_ms(lim, radius_m)
                    if v_max > 0.01:
                        ok, _lead, why = arcnav.plan_feasible(lim, radius_m, v_max * 0.999)
                        if not ok:
                            bad.append("ψ̇=%g r=%g v=%g → 建议降到 %.4g 仍不可行（%s）"
                                       % (yaw_degs, radius_m, speed_ms, v_max, why))
                    r_req, _, _ = check.required_radius_m(lim, speed_ms)
                    if r_req > 0.01:
                        ok, _lead, why = arcnav.plan_feasible(lim, r_req * 1.001, speed_ms)
                        if not ok:
                            bad.append("ψ̇=%g r=%g v=%g → 建议开到 %.4g 仍不可行（%s）"
                                       % (yaw_degs, radius_m, speed_ms, r_req, why))
        self.assertEqual([], bad[:10], "%d 处改法给错" % len(bad))


class SitlYawCapabilitySelection(unittest.TestCase):
    """54.7–59.1 不是不确定区间，是 ATC_RAT_YAW_FF 两个取值各自的实测值。

    取中值 57 会在两头各错约 4%，而验收那条航线的余量不到 1%——本工具第一次跑
    验收时就因此把一条实飞掉速 0.19% 的航线判成了不可行。
    """

    def test_ff_on_picks_the_higher_measured_value(self):
        v, warn, src = mission_arc_check.sitl_yaw_capability({"ATC_RAT_YAW_FF": 0.30})
        self.assertEqual(mission_arc_check.SITL_YAW_FF_ON_DEGS, v)
        self.assertIsNone(warn)
        self.assertIn("0.3", src)

    def test_ff_off_picks_the_lower_measured_value(self):
        v, warn, _src = mission_arc_check.sitl_yaw_capability({})
        self.assertEqual(mission_arc_check.SITL_YAW_FF_OFF_DEGS, v)
        self.assertIsNone(warn)

    def test_intermediate_ff_is_conservative_and_says_so(self):
        v, warn, _src = mission_arc_check.sitl_yaw_capability({"ATC_RAT_YAW_FF": 0.15})
        self.assertEqual(mission_arc_check.SITL_YAW_FF_OFF_DEGS, v)
        self.assertIsNotNone(warn)
        self.assertIn("不插值", warn)

    def test_the_acceptance_route_is_feasible_with_the_flown_configuration(self):
        """验收那条航线：FF=0.30 飞的，就得用 59.1 判，不能用中值 57。

        59.1×50% = 29.55 °/s > 需求 28.65 → 可行（实飞掉速 0.19%）
        57  ×50% = 28.5  °/s < 需求 28.65 → 误判不可行
        """
        need_degs = math.degrees(3.0 / 6.0)
        self.assertLess(need_degs,
                        mission_arc_check.SITL_YAW_FF_ON_DEGS * arcnav.YAW_RATE_FRACTION)
        self.assertGreater(need_degs, 57.0 * arcnav.YAW_RATE_FRACTION)


if __name__ == "__main__":
    unittest.main(verbosity=2)
