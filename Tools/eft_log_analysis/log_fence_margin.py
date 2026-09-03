#!/usr/bin/env python3
"""从真机日志算电子围栏余量：逐模式分段、逐次逼近事件、速度-距离分布。

本 fork 专用，只读日志。用仓库自带的 `modules/mavlink/pymavlink`，路径相对脚本
自身解析，不依赖当前工作目录。

与 `~/UAV/tools/fence_margin.py` 的区别：那个是 SITL 专用（只认 `runs/` 目录、
多边形靠外部传入、按 `LOIT_SPEED` 变更分段）。本脚本面向真机架次——围栏从日志的
`FNCE` 消息自取，按飞行模式分段，并给出逐次逼近事件。

用法:
  log_fence_margin.py <log.bin>                  # 全部模式
  log_fence_margin.py <log.bin> --mode ALT_HOLD  # 只看某个模式
  log_fence_margin.py <log.bin> --csv out.csv    # 逼近事件导出

两个必须守住的口径
------------------

**一、位置只用 `POS`，不用 `GPS`。**

`GPS` 消息按实例分开记录（本机 RTK 主 + NMEA 副，实例 0 / 1）。不按实例过滤就会
把两台接收机的位置交织成一条序列，两者相隔数米，于是算出根本不存在的越界。
2026-09-02 架次 00000211 上就这样一度得出「ALT_HOLD 越界 1.2 m」——识破的线索是
距离在 0.2 s 内在 +1.8 与 −1.0 m 之间反复跳变，而飞机当时只有 0.4 m/s。
`POS` 是 EKF 融合后的单一位置，没有这个问题，也正是围栏限制器自己用的那个量。
本脚本因此**只读 `POS`**，速度也由 `POS` 中心差分得到，不碰 `GPS`。

**二、距离用到边界的法向距离，不是到中心的径向距离。**

对多边形，径向距离既算错余量也判不出越界：正 N 边形的边心距只有外接半径的
cos(pi/N) 倍，六边形是 0.866。内部为正、外部为负。
"""
import argparse
import bisect
import csv
import math
import os
import sys

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "..", "modules", "mavlink"))
from pymavlink import DFReader  # noqa: E402

# ArduCopter 模式号 -> 名字。只列本项目会用到的，未知的按号显示。
MODE_NAMES = {
    0: "STABILIZE", 1: "ACRO", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED",
    5: "LOITER", 6: "RTL", 7: "CIRCLE", 9: "LAND", 11: "DRIFT", 13: "SPORT",
    16: "POSHOLD", 17: "BRAKE", 21: "SMART_RTL", 22: "FLOWHOLD", 24: "ZIGZAG",
}

# MAV_CMD_NAV_FENCE_* 里本脚本认识的两种包含区
FENCE_POLY_INCLUSION = 98
FENCE_CIRCLE_INCLUSION = 97


def seg_dist(p, a, b):
    """点到线段的距离。"""
    (px, py), (ax, ay), (bx, by) = p, a, b
    dx, dy = bx - ax, by - ay
    L = dx * dx + dy * dy
    t = 0.0 if L == 0.0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def point_in_poly(p, verts):
    """射线法，顶点顺逆时针皆可。"""
    px, py = p
    inside = False
    n = len(verts)
    for i in range(n):
        (ax, ay), (bx, by) = verts[i], verts[(i + 1) % n]
        if (ay > py) != (by > py):
            if px < ax + (py - ay) * (bx - ax) / (by - ay):
                inside = not inside
    return inside


def signed_dist(p, verts):
    """到多边形边界的法向距离，内正外负。"""
    n = len(verts)
    d = min(seg_dist(p, verts[i], verts[(i + 1) % n]) for i in range(n))
    return d if point_in_poly(p, verts) else -d


def nearest_edge_index(p, verts):
    n = len(verts)
    ds = [seg_dist(p, verts[i], verts[(i + 1) % n]) for i in range(n)]
    return ds.index(min(ds))


def edge_label(verts, i):
    """用边中点相对多边形中心的方位给边起个名字，便于和现场口述对上。"""
    n = len(verts)
    (ax, ay), (bx, by) = verts[i], verts[(i + 1) % n]
    mx, my = (ax + bx) / 2.0, (ay + by) / 2.0
    cx = sum(v[0] for v in verts) / n
    cy = sum(v[1] for v in verts) / n
    brg = math.degrees(math.atan2(my - cy, mx - cx)) % 360.0
    return ["北", "东北", "东", "东南", "南", "西南", "西", "西北"][
        int((brg + 22.5) % 360.0 // 45.0)]


def read_log(path):
    """一次遍历取出围栏顶点、模式序列、位置序列与 FENCE_MARGIN。"""
    m = DFReader.DFReader_binary(path)
    fence_items = []
    modes = []          # (t, mode_num)
    pos = []            # (t, lat, lng, alt)
    thr = []            # (t, CTUN.ThO) 用于判在飞
    fence_margin = None
    gps_instances = set()
    while True:
        r = m.recv_msg()
        if r is None:
            break
        t = r.get_type()
        if t == "FNCE":
            fence_items.append({k: getattr(r, k) for k in r._fieldnames})
        elif t == "MODE":
            modes.append((r.TimeUS / 1e6, int(getattr(r, "ModeNum", -1))))
        elif t == "POS":
            pos.append((r.TimeUS / 1e6, r.Lat, r.Lng, getattr(r, "Alt", float("nan"))))
        elif t == "PARM" and getattr(r, "Name", "") == "FENCE_MARGIN":
            fence_margin = float(r.Value)
        elif t == "CTUN":
            tho = getattr(r, "ThO", None)
            if tho is not None:
                thr.append((r.TimeUS / 1e6, float(tho)))
        elif t == "GPS":
            gps_instances.add(getattr(r, "I", 0))
    return fence_items, modes, pos, thr, fence_margin, gps_instances


def build_fence(fence_items):
    """把 FNCE 消息拼成 (顶点列表[(lat,lng)], 说明)。只认包含多边形与包含圆。"""
    polys = [f for f in fence_items if int(f.get("Type", -1)) == FENCE_POLY_INCLUSION]
    if polys:
        polys.sort(key=lambda f: int(f.get("Seq", 0)))
        return [(f["Lat"], f["Lng"]) for f in polys], \
               "包含多边形 %d 顶点" % len(polys)
    circles = [f for f in fence_items
               if int(f.get("Type", -1)) == FENCE_CIRCLE_INCLUSION]
    if circles:
        c = circles[0]
        n = 64
        lat0, lng0, rad = c["Lat"], c["Lng"], float(c.get("Radius", 0.0))
        mpd_lat = 111320.0
        mpd_lng = 111320.0 * math.cos(math.radians(lat0))
        verts = [(lat0 + rad * math.cos(2 * math.pi * k / n) / mpd_lat,
                  lng0 + rad * math.sin(2 * math.pi * k / n) / mpd_lng)
                 for k in range(n)]
        return verts, "包含圆 半径 %.1f m（离散为 %d 边）" % (rad, n)
    return None, None


def to_ne(lat, lng, lat0, lng0):
    return ((lat - lat0) * 111320.0,
            (lng - lng0) * 111320.0 * math.cos(math.radians(lat0)))


def mode_at(modes, t):
    """modes 已按时间排序，返回 t 时刻生效的模式号。"""
    i = bisect.bisect_right([x[0] for x in modes], t) - 1
    return modes[i][1] if i >= 0 else None


def airborne_filter(thr, thresh=0.15):
    """按 CTUN.ThO 判在飞，与 log_control_metrics.py 同口径。

    不排除地面段的话，停机等待会被算进它当时所处的模式：00000211 上 LOITER
    因此从 122 s 变成 264 s、20–30 m 距离档的样本数翻一倍多。最小余量和逼近事件
    不受影响（都在空中），但时长与分布统计会失真。
    """
    ts = [x[0] for x in thr]
    if not ts:
        return lambda t: True          # 没有 CTUN 就不过滤，总比全丢掉强
    def ok(t):
        i = bisect.bisect_left(ts, t)
        i = min(max(i, 0), len(thr) - 1)
        return thr[i][1] >= thresh
    return ok


def build_samples(pos, modes, verts_ne, lat0, lng0, is_airborne):
    """(t, mode, 法向距离, 地速, 最近边序号, 高度)。速度由 POS 中心差分得到。"""
    pts = [(t, to_ne(la, ln, lat0, lng0), alt) for t, la, ln, alt in pos]
    out = []
    for i in range(2, len(pts) - 2):
        t, p, alt = pts[i]
        if not is_airborne(t):
            continue
        dt = pts[i + 2][0] - pts[i - 2][0]
        if dt <= 0:
            continue
        v = math.hypot(pts[i + 2][1][0] - pts[i - 2][1][0],
                       pts[i + 2][1][1] - pts[i - 2][1][1]) / dt
        out.append((t, mode_at(modes, t), signed_dist(p, verts_ne), v,
                    nearest_edge_index(p, verts_ne), alt))
    return out


def airborne_seconds(seg, max_gap_s=1.0):
    """在飞时长 = 相邻样本间隔之和，跳过大空档。

    不能用首末样本之差：同一个模式可能被别的模式打断成几段，00000211 的 LOITER
    就分布在 ALT_HOLD 两侧，取跨度会把中间那 73 s 一并算进去（202 s vs 实际 140 s）。
    """
    total = 0.0
    for a, b in zip(seg, seg[1:]):
        dt = b[0] - a[0]
        if 0.0 < dt <= max_gap_s:
            total += dt
    return total


def count_episodes(seg, max_gap_s=1.0):
    """该模式被打断成几段。"""
    n = 1
    for a, b in zip(seg, seg[1:]):
        if b[0] - a[0] > max_gap_s:
            n += 1
    return n


def find_events(seg, near_m, win=5, gap_s=5.0):
    """逼近事件 = 法向距离的局部极小且小于 near_m。返回样本下标列表。"""
    ev = []
    for i in range(win, len(seg) - win):
        if seg[i][2] != min(x[2] for x in seg[i - win:i + win + 1]):
            continue
        if seg[i][2] >= near_m:
            continue
        if ev and seg[i][0] - seg[ev[-1]][0] <= gap_s:
            continue
        ev.append(i)
    return ev


def main():
    ap = argparse.ArgumentParser(
        description="从真机日志算电子围栏余量与逐次逼近事件")
    ap.add_argument("log", help="解密后的 dataflash 日志（.bin）")
    ap.add_argument("--mode", action="append", default=None, metavar="NAME",
                    help="只统计这些模式，可重复。默认全部")
    ap.add_argument("--near", type=float, default=8.0, metavar="M",
                    help="逼近事件的距离门槛（默认 8 m）")
    ap.add_argument("--thr-min", type=float, default=0.15, metavar="X",
                    help="判在飞的 CTUN.ThO 门槛（默认 0.15，与 "
                         "log_control_metrics.py 同口径）。地面段不排除会把停机时间"
                         "算进当时的模式，时长与分布统计失真")
    ap.add_argument("--csv", default=None, metavar="FILE",
                    help="把逼近事件导出为 CSV")
    args = ap.parse_args()

    fence_items, modes, pos, thr, margin, gps_inst = read_log(args.log)
    if not modes:
        raise SystemExit("日志里没有 MODE 消息，无法分段")
    if not pos:
        raise SystemExit("日志里没有 POS 消息。本脚本刻意不回退到 GPS——"
                         "GPS 按实例分开记录，交织后会算出不存在的越界，见文件头")
    verts, fence_desc = build_fence(fence_items)
    if not verts:
        raise SystemExit("日志里没有可识别的包含区围栏（FNCE Type 97/98）")

    lat0 = sum(v[0] for v in verts) / len(verts)
    lng0 = sum(v[1] for v in verts) / len(verts)
    verts_ne = [to_ne(la, ln, lat0, lng0) for la, ln in verts]
    half_width = signed_dist((0.0, 0.0), verts_ne)

    print("围栏：%s，中心到边界最近 %.2f m" % (fence_desc, half_width))
    print("FENCE_MARGIN = %s" % ("%.1f m" % margin if margin is not None else "未记录"))
    if len(gps_inst) > 1:
        print("注意：日志含 %d 个 GPS 实例 %s；本脚本只用 POS，不受其影响"
              % (len(gps_inst), sorted(gps_inst)))

    samples = build_samples(pos, modes, verts_ne, lat0, lng0,
                            airborne_filter(thr, args.thr_min))
    if not samples:
        raise SystemExit("按 CTUN.ThO >= %.2f 判定，全程没有在飞样本；"
                         "如需放宽用 --thr-min" % args.thr_min)
    wanted = set(args.mode) if args.mode else None

    rows = []
    seen = []
    for _, mnum in modes:
        if mnum not in seen:
            seen.append(mnum)
    for mnum in seen:
        name = MODE_NAMES.get(mnum, "MODE_%d" % mnum)
        if wanted and name not in wanted:
            continue
        seg = [s for s in samples if s[1] == mnum]
        if len(seg) < 20:
            continue
        dmin = min(s[2] for s in seg)
        breach = sum(1 for s in seg if s[2] < 0.0)
        dur = airborne_seconds(seg)
        eps = count_episodes(seg)
        print("\n=== %s ===  在飞 %.0f s%s  采样 %d  最小余量 %.2f m  越界采样 %d%s"
              % (name, dur, "（分 %d 段）" % eps if eps > 1 else "",
                 len(seg), dmin, breach, "  ** 越界 **" if breach else ""))
        print("  最大地速 %.2f m/s   高度 %.1f~%.1f m"
              % (max(s[3] for s in seg),
                 min(s[5] for s in seg), max(s[5] for s in seg)))

        ev = find_events(seg, args.near)
        if ev:
            print("  逼近事件 %d 次（最近点 < %.0f m）" % (len(ev), args.near))
            print("    #   最近点t   最近余量   边   进场峰值速度  峰值处距边界  平均减速度")
            for k, i in enumerate(ev, 1):
                t0, _, d0, v0, ei, _ = seg[i]
                win = [s for s in seg if t0 - 10.0 <= s[0] <= t0]
                vmax = max(w[3] for w in win)
                wm = next(w for w in win if w[3] == vmax)
                dt = t0 - wm[0]
                dec = (vmax - v0) / dt if dt > 0 else 0.0
                lab = edge_label(verts_ne, ei)
                print("    %-3d %8.1f %9.2f %5s %13.2f %13.2f %12.2f"
                      % (k, t0, d0, lab, vmax, wm[2], dec))
                rows.append({"mode": name, "t_closest_s": round(t0, 1),
                             "margin_m": round(d0, 2), "edge": lab,
                             "peak_speed_ms": round(vmax, 2),
                             "dist_at_peak_m": round(wm[2], 2),
                             "mean_decel_mss": round(dec, 2),
                             "breached": d0 < 0.0})

        print("    距边界        样本   最大地速")
        for a, b in ((0, 5), (5, 10), (10, 15), (15, 20), (20, 30), (30, 60)):
            sub = [s for s in seg if a <= s[2] < b]
            if sub:
                print("    %2d–%2d m %9d   %6.2f m/s"
                      % (a, b, len(sub), max(x[3] for x in sub)))

    if args.csv and rows:
        with open(args.csv, "w", newline="", encoding="utf-8") as fh:
            w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
            w.writeheader()
            w.writerows(rows)
        print("\n逼近事件已写入 %s（%d 行）" % (args.csv, len(rows)))


if __name__ == "__main__":
    main()
