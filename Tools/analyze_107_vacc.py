#!/usr/bin/env python3
"""Analyze EK3_GPS_VACC_MAX effect on height estimation in log 107."""
import collections
import math
import statistics
import sys
from pymavlink import DFReader

LOG = r"C:\Users\Administrator\Desktop\00000107_decrypted.bin"


def stat(vals):
    if len(vals) < 2:
        return None
    return {
        "n": len(vals),
        "mean": statistics.mean(vals),
        "std": statistics.stdev(vals),
        "min": min(vals),
        "max": max(vals),
        "range": max(vals) - min(vals),
    }


def nearest(series, t, tol=0.15):
    best = None
    best_dt = tol
    for ts, v in series:
        dt = abs(ts - t)
        if dt < best_dt:
            best_dt = dt
            best = v
    return best


def main():
    df = DFReader.DFReader_binary(LOG)
    msgs = []
    while True:
        m = df.recv_msg()
        if m is None:
            break
        msgs.append(m)

    params = {}
    gps = []
    baro = []
    ebfh = []
    mode_arm = []  # (t, mode_num, armed)
    gps_status_changes = []

    for m in msgs:
        t = getattr(m, "TimeUS", None)
        if t is not None:
            t = t / 1e6
        mt = m.get_type()

        if mt == "PARM":
            params[m.Name] = m.Value

        elif mt == "GPS":
            vacc = getattr(m, "VAcc", None)
            if vacc is not None and vacc > 50:
                vacc *= 0.01
            status = getattr(m, "Status", None)
            alt = m.Alt
            if abs(alt) > 5000:
                alt *= 0.01
            gps.append({
                "t": t,
                "alt": alt,
                "vacc": vacc,
                "status": status,
                "hdop": getattr(m, "HDop", None),
                "nsats": getattr(m, "NSats", None),
            })

        elif mt == "BARO":
            baro.append((t, m.Alt))

        elif mt == "EBFH":
            ebfh.append({
                "t": t,
                "pd": -m.PD,           # up positive
                "pd_nb": -m.PDnb,
                "pd_wb": -m.PDwb,
                "baro_h": -m.BH,
                "gps_h": -m.GH,
                "flags": m.Flg,
            })

        elif mt == "MODE":
            mode_arm.append((t, m.Mode, getattr(m, "ModeNum", None)))

        elif mt == "STAT":
            # armed state
            mode_arm.append((t, "STAT", m.Armed))

    print("=" * 60)
    print("LOG 00000107 — EK3_GPS_VACC_MAX 高度估计分析")
    print("=" * 60)
    print(f"总消息数: {len(msgs)}")
    print(f"GPS 样本: {len(gps)}, BARO: {len(baro)}, EBFH: {len(ebfh)}")

    print("\n--- 关键参数 ---")
    for k in [
        "EK3_GPS_VACC_MAX", "EK3_BARO_HDOP", "EK3_SRC1_POSZ",
        "EK3_SRC1_POSXY", "EK3_SRC1_YAW", "EK3_ENABLE", "AHRS_EKF_TYPE",
    ]:
        print(f"  {k} = {params.get(k, 'N/A')}")

    vacc_vals = [g["vacc"] for g in gps if g["vacc"] is not None]
    if vacc_vals:
        s = stat(vacc_vals)
        below = sum(1 for v in vacc_vals if v < 0.5) / len(vacc_vals) * 100
        above = sum(1 for v in vacc_vals if v >= 0.5) / len(vacc_vals) * 100
        print("\n--- GPS 垂直精度 VAcc ---")
        print(f"  均值={s['mean']:.2f}m  标准差={s['std']:.2f}m  最大={s['max']:.2f}m")
        print(f"  VAcc < 0.5m (GPS高度可用): {below:.1f}%")
        print(f"  VAcc >= 0.5m (应回落baro): {above:.1f}%")

    # GPS status distribution
    status_cnt = collections.Counter(g["status"] for g in gps if g["status"] is not None)
    status_names = {0: "NoGPS", 1: "NoFix", 2: "2D", 3: "3D", 4: "DGPS", 5: "RTK_Float", 6: "RTK_Fixed"}
    print("\n--- GPS 定位状态 ---")
    for st, cnt in sorted(status_cnt.items()):
        pct = cnt / len(gps) * 100
        print(f"  {status_names.get(st, st)}: {cnt} ({pct:.1f}%)")

    # Time segments by VAcc threshold
    segments = {"vacc_ok": [], "vacc_bad": []}
    for g in gps:
        if g["vacc"] is None:
            continue
        key = "vacc_ok" if g["vacc"] < 0.5 else "vacc_bad"
        segments[key].append(g)

    print("\n--- 按 VAcc 分段 (阈值 0.5m) ---")
    for name, label in [("vacc_ok", "VAcc<0.5 (应用GPS高度)"), ("vacc_bad", "VAcc>=0.5 (应拒绝GPS高度)")]:
        seg = segments[name]
        if not seg:
            print(f"  {label}: 无数据")
            continue
        alts = [g["alt"] for g in seg]
        s = stat(alts)
        t0, t1 = seg[0]["t"], seg[-1]["t"]
        print(f"  {label}: {len(seg)}点, 时长≈{t1-t0:.0f}s")
        print(f"    GPS Alt 波动: range={s['range']:.2f}m std={s['std']:.2f}m")

    # EBFH analysis - the key custom log
    if ebfh:
        print("\n--- EBFH 高度对比 (核心) ---")
        baro_fused = sum(1 for e in ebfh if e["flags"] & 1)
        baro_blocked = sum(1 for e in ebfh if e["flags"] & 2)
        baro_latched = sum(1 for e in ebfh if e["flags"] & 4)
        print(f"  baro已融合(bit0): {baro_fused}/{len(ebfh)} ({baro_fused/len(ebfh)*100:.1f}%)")
        print(f"  baro被HDOP门控挡(bit1): {baro_blocked}/{len(ebfh)} ({baro_blocked/len(ebfh)*100:.1f}%)")
        print(f"  baro锁定(bit2): {baro_latched}/{len(ebfh)} ({baro_latched/len(ebfh)*100:.1f}%)")

        # Split EBFH by corresponding GPS VAcc
        ekf_gps_diff_ok = []
        ekf_gps_diff_bad = []
        ekf_baro_diff_ok = []
        ekf_baro_diff_bad = []
        ekf_drift_ok = []
        ekf_drift_bad = []

        gps_by_t = [(g["t"], g) for g in gps]

        def gps_at(t):
            best = None
            bd = 0.5
            for gt, g in gps_by_t:
                d = abs(gt - t)
                if d < bd:
                    bd = d
                    best = g
            return best

        for e in ebfh:
            g = gps_at(e["t"])
            if g is None:
                continue
            ekf_gps = e["pd"] - e["gps_h"]
            ekf_baro = e["pd"] - e["baro_h"]
            if g["vacc"] is not None and g["vacc"] < 0.5:
                ekf_gps_diff_ok.append(abs(ekf_gps))
                ekf_baro_diff_ok.append(abs(ekf_baro))
                ekf_drift_ok.append(e["pd"])
            elif g["vacc"] is not None:
                ekf_gps_diff_bad.append(abs(ekf_gps))
                ekf_baro_diff_bad.append(abs(ekf_baro))
                ekf_drift_bad.append(e["pd"])

        print("\n  VAcc>=0.5 时段 (GPS高度被拒绝，EKF应跟baro):")
        if ekf_baro_diff_bad:
            s = stat(ekf_baro_diff_bad)
            print(f"    |EKF - Baro| 均值={s['mean']:.3f}m  最大={s['max']:.3f}m")
        if ekf_gps_diff_bad:
            s = stat(ekf_gps_diff_bad)
            print(f"    |EKF - GPS|  均值={s['mean']:.3f}m  最大={s['max']:.3f}m  (应较大，说明未跟GPS)")
        if ekf_drift_bad and len(ekf_drift_bad) > 10:
            s = stat(ekf_drift_bad)
            print(f"    EKF高度波动 range={s['range']:.2f}m std={s['std']:.3f}m")

        print("\n  VAcc<0.5 时段 (GPS高度可用，EKF应跟GPS):")
        if ekf_gps_diff_ok:
            s = stat(ekf_gps_diff_ok)
            print(f"    |EKF - GPS|  均值={s['mean']:.3f}m  最大={s['max']:.3f}m")
        if ekf_baro_diff_ok:
            s = stat(ekf_baro_diff_ok)
            print(f"    |EKF - Baro| 均值={s['mean']:.3f}m  最大={s['max']:.3f}m")
        if ekf_drift_ok and len(ekf_drift_ok) > 10:
            s = stat(ekf_drift_ok)
            print(f"    EKF高度波动 range={s['range']:.2f}m std={s['std']:.3f}m")

        # First 5 min stationary analysis (before any flight)
        early = [e for e in ebfh if e["t"] < 300]
        if early:
            print("\n  前5分钟 (上电静止阶段):")
            pd = [e["pd"] for e in early]
            gh = [e["gps_h"] for e in early]
            bh = [e["baro_h"] for e in early]
            sp = stat(pd)
            sg = stat(gh)
            sb = stat(bh)
            print(f"    EKF高度 range={sp['range']:.2f}m std={sp['std']:.3f}m")
            print(f"    GPS高度 range={sg['range']:.2f}m std={sg['std']:.3f}m")
            print(f"    Baro高度 range={sb['range']:.2f}m std={sb['std']:.3f}m")
            # correlation: does EKF follow baro or GPS?
            ekf_gps_r = statistics.mean(abs(p - g) for p, g in zip(pd, gh))
            ekf_baro_r = statistics.mean(abs(p - b) for p, b in zip(pd, bh))
            print(f"    平均|EKF-GPS|={ekf_gps_r:.2f}m  平均|EKF-Baro|={ekf_baro_r:.2f}m")
            if ekf_baro_r < ekf_gps_r:
                print("    → EKF 更接近 Baro，VAcc门控生效")
            else:
                print("    → EKF 更接近 GPS，可能仍在跟漂 GPS")

        # Full log stationary drift
        pd_all = [e["pd"] for e in ebfh]
        gh_all = [e["gps_h"] for e in ebfh]
        bh_all = [e["baro_h"] for e in ebfh]
        print("\n  全段 EBFH 汇总:")
        print(f"    EKF高度 range={stat(pd_all)['range']:.2f}m")
        print(f"    GPS高度 range={stat(gh_all)['range']:.2f}m")
        print(f"    Baro高度 range={stat(bh_all)['range']:.2f}m")
        print(f"    平均|EKF-GPS|={statistics.mean(abs(p-g) for p,g in zip(pd_all,gh_all)):.2f}m")
        print(f"    平均|EKF-Baro|={statistics.mean(abs(p-b) for p,b in zip(pd_all,bh_all)):.2f}m")

    # Compare with old problem signature: cumulative GPS alt drift while disarmed
    print("\n--- 与旧问题对比 (GPS高度累计漂移) ---")
    if len(gps) > 20:
        # first 10 min GPS alt drift
        early_gps = [g for g in gps if g["t"] < 600]
        if early_gps:
            alts = [g["alt"] for g in early_gps]
            s = stat(alts)
            print(f"  前10分钟 GPS Alt: 起={alts[0]:.1f}m 止={alts[-1]:.1f}m 累计变化={alts[-1]-alts[0]:.1f}m range={s['range']:.1f}m")
        if ebfh:
            early_e = [e for e in ebfh if e["t"] < 600]
            if early_e:
                pd = [e["pd"] for e in early_e]
                print(f"  前10分钟 EKF高度: 起={pd[0]:.1f}m 止={pd[-1]:.1f}m 累计变化={pd[-1]-pd[0]:.1f}m range={stat(pd)['range']:.1f}m")

    print("\n--- 结论 ---")
    vacc_max = params.get("EK3_GPS_VACC_MAX", 0)
    if vacc_max == 0.5 and vacc_vals:
        avg_vacc = statistics.mean(vacc_vals)
        pct_bad = sum(1 for v in vacc_vals if v >= 0.5) / len(vacc_vals) * 100
        if pct_bad > 50 and ebfh:
            ekf_baro = statistics.mean(abs(e["pd"] - e["baro_h"]) for e in ebfh)
            ekf_gps = statistics.mean(abs(e["pd"] - e["gps_h"]) for e in ebfh)
            baro_range = stat([e["baro_h"] for e in ebfh])["range"]
            ekf_range = stat([e["pd"] for e in ebfh])["range"]
            gps_range = stat([e["gps_h"] for e in ebfh])["range"]
            if ekf_range < gps_range * 0.5 and ekf_baro < ekf_gps:
                print("  ✅ 有明显改善：EKF高度波动远小于GPS，且更贴近Baro。")
                print(f"     EKF range={ekf_range:.1f}m vs GPS range={gps_range:.1f}m vs Baro range={baro_range:.1f}m")
            elif ekf_range < gps_range * 0.8:
                print("  ⚠️ 部分改善：EKF高度波动小于GPS，但仍有可优化空间。")
            else:
                print("  ❌ 改善不明显：EKF高度仍大幅跟随GPS漂移。")
        elif pct_bad < 20:
            print("  ℹ️ 本次测试 GPS VAcc 大部分 <0.5m，门控很少触发；需对比非RTK段。")
        else:
            print("  需结合 EBFH 数据进一步判断。")

    return 0


if __name__ == "__main__":
    sys.exit(main())
