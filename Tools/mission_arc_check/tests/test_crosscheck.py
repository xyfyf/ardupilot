"""对拍：Python 镜像 vs C++ 本体。

判据不由被测对象自己给出——这是本项目反复踩到同一个坑之后定下的规矩。校验工具
的输出正确与否，不能由校验工具自己说了算。

分两层：
  单元层（本文件）  由 C++ 的 AC_ArcNav 界定，逐点比对两份实现
  系统层（SITL）    由实飞轨迹界定，见 README

**表是现场生成的，不读仓库里的文件。** 一份提交进仓库的表，在 C++ 改了而没人重新
生成时，对拍照样通过——比对的是陈旧快照。那和"工具自证"是同一种错觉，只是更隐蔽，
因为它看起来有外部基准。所以这里宁可在没有构建环境时**明确跳过并说出来**，也不
退回去读旧表。

用法：
    python3 -m unittest discover Tools/mission_arc_check/tests
    python3 Tools/mission_arc_check/tests/test_crosscheck.py     # 同上，直接跑
"""

import os
import subprocess
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import arcnav  # noqa: E402

# Tools/mission_arc_check/tests/ -> 仓库根
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))
GTEST_BIN = os.path.join(REPO_ROOT, "build", "sitl", "tests", "test_arcnav_feasibility")
GTEST_SRC = os.path.join(REPO_ROOT, "libraries", "AC_WPNav", "tests",
                         "test_arcnav_feasibility.cpp")

# 两份实现都在 float32 上算，剩下的差异只来自 libm 的 tanf/sqrtf 与 Python 的
# double 版本在末位上的取舍。1e-6 相对误差远大于那点差、又远小于任何有物理意义的
# 差别——真漂了不会只漂 1e-6。
REL_TOL = 1e-6

FLOAT_COLS = ["lead_in_m", "required_spiral_len_m", "max_speed_for_radius_ms",
              "min_radius_for_speed_m", "plan_radius_m", "plan_spiral_len_m",
              "plan_duration_s"]
BOOL_COLS = ["feasible", "plan_rate_limited"]


def build_gtest():
    """现场构建。构建不了就返回原因，由调用方跳过——不退回读旧表。"""
    waf = os.path.join(REPO_ROOT, "waf")
    if not os.path.exists(waf):
        return "找不到 %s" % waf
    cache = os.path.join(REPO_ROOT, "build", "c4che", "sitl_cache.py")
    if not os.path.exists(cache):
        return "未为 sitl configure（找不到 %s），先跑 ./waf configure --board sitl" % cache
    proc = subprocess.run([sys.executable, waf, "tests",
                           "--targets=tests/test_arcnav_feasibility"],
                          cwd=REPO_ROOT, capture_output=True, text=True)
    if proc.returncode != 0:
        return "构建失败：\n%s" % proc.stdout[-2000:]
    return None


def dump_table():
    out = subprocess.run([GTEST_BIN, "--dump-table"],
                         capture_output=True, text=True, check=True).stdout
    lines = out.strip().splitlines()
    header = lines[0].split(",")
    return [dict(zip(header, line.split(","))) for line in lines[1:]]


class CrossCheck(unittest.TestCase):

    rows = None
    skip_reason = None

    @classmethod
    def setUpClass(cls):
        why = build_gtest()
        if why is not None:
            cls.skip_reason = why
            return
        if not os.path.exists(GTEST_BIN):
            cls.skip_reason = "构建成功但没找到 %s" % GTEST_BIN
            return
        cls.rows = dump_table()

    def setUp(self):
        if self.skip_reason is not None:
            # 说出来。静默跳过的对拍等于没有对拍。
            self.skipTest("无法现场生成 C++ 对拍表，本次未做单元层对拍："
                          + self.skip_reason)

    def test_source_is_the_one_that_was_built(self):
        """源文件在、且构建产物比它新——否则比的是上一版 C++。"""
        self.assertTrue(os.path.exists(GTEST_SRC), GTEST_SRC)
        self.assertGreaterEqual(os.path.getmtime(GTEST_BIN), os.path.getmtime(GTEST_SRC),
                                "gtest 二进制比源文件旧，构建没生效")

    def test_grid_is_not_degenerate(self):
        """一张全是 false 或全是 true 的表比不出任何东西。"""
        self.assertGreater(len(self.rows), 1000, "网格太小")
        n_true = sum(1 for r in self.rows if r["feasible"] == "1")
        self.assertGreater(n_true, len(self.rows) // 10, "可行样本太少，判定逻辑没被覆盖")
        self.assertLess(n_true, len(self.rows) * 9 // 10, "不可行样本太少")

    def test_python_mirror_matches_cpp(self):
        mismatches = []
        for i, row in enumerate(self.rows):
            lim = arcnav.Limits(
                lean_angle_max_rad=float(row["lean_angle_max_rad"]),
                jerk_ne_msss=float(row["jerk_ne_msss"]),
                yaw_rate_max_rads=float(row["yaw_rate_max_rads"]),
                yaw_accel_max_radss=float(row["yaw_accel_max_radss"]),
                heading_lead_s=float(row["heading_lead_s"]),
            )
            r = float(row["radius_m"])
            v = float(row["speed_ms"])

            feasible, lead_in_m, _ = arcnav.plan_feasible(lim, r, v)
            plan = arcnav.plan_from_yaw_capability(
                v, arcnav.math.pi, lim.yaw_rate_max_rads * arcnav.YAW_RATE_FRACTION,
                lim.yaw_accel_max_radss)

            got = {
                "feasible": 1 if feasible else 0,
                "lead_in_m": lead_in_m,
                "required_spiral_len_m": arcnav.required_spiral_len_m(lim, r, v),
                "max_speed_for_radius_ms": arcnav.max_speed_for_radius_ms(lim, r),
                "min_radius_for_speed_m": arcnav.min_radius_for_speed_m(lim, v),
                "plan_radius_m": plan.radius_m,
                "plan_spiral_len_m": plan.spiral_len_m,
                "plan_duration_s": plan.duration_s,
                "plan_rate_limited": 1 if plan.rate_limited else 0,
            }

            for col in BOOL_COLS:
                if got[col] != int(row[col]):
                    mismatches.append("行%d %s: C++=%s Python=%s  (r=%s v=%s ψ̇=%s)"
                                      % (i, col, row[col], got[col],
                                         row["radius_m"], row["speed_ms"],
                                         row["yaw_rate_max_rads"]))
            for col in FLOAT_COLS:
                want = float(row[col])
                scale = max(abs(want), 1.0)
                if abs(got[col] - want) > REL_TOL * scale:
                    mismatches.append("行%d %s: C++=%.9g Python=%.9g  (r=%s v=%s)"
                                      % (i, col, want, got[col],
                                         row["radius_m"], row["speed_ms"]))

        self.assertEqual([], mismatches[:20],
                         "%d 处不一致（只列前 20 处）" % len(mismatches))


if __name__ == "__main__":
    unittest.main(verbosity=2)
