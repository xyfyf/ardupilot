#!/usr/bin/env python3
"""把 reproduce.py 的六旋翼 SITL 日志渲染成直观 MP4/GIF。

用法:
  render_video.py landing <coupled-result.json> <baseline-result.json> --output out
  render_video.py reverse <coupled-result.json> <baseline-result.json> --output out
"""

import argparse
import glob
import json
import math
import os
import sys

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
AP_ROOT = os.path.normpath(os.path.join(HERE, os.pardir, os.pardir))
sys.path.insert(0, os.path.join(AP_ROOT, "modules", "mavlink"))
from pymavlink import DFReader  # noqa: E402

W, H = 1280, 720
FPS = 30
# OpenCV drawing colors are BGR.  add_text() reverses them for Pillow/RGB.
BG = (35, 25, 18)
PANEL = (53, 39, 28)
GRID = (88, 69, 52)
WHITE = (247, 242, 236)
MUTED = (189, 171, 154)
CYAN = (232, 205, 48)
ORANGE = (64, 166, 255)
RED = (89, 77, 247)
GREEN = (137, 207, 73)
YELLOW = (79, 211, 245)
FONT = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
FONT_BOLD = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"
FONTS = {}


def font(size, bold=False):
    key = (size, bold)
    if key not in FONTS:
        FONTS[key] = ImageFont.truetype(FONT_BOLD if bold else FONT, size)
    return FONTS[key]


def add_text(frame, entries):
    image = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    draw = ImageDraw.Draw(image)
    for text, xy, size, color, bold in entries:
        draw.text(xy, text, font=font(size, bold), fill=tuple(reversed(color)))
    return cv2.cvtColor(np.asarray(image), cv2.COLOR_RGB2BGR)


def load_result(path):
    return json.load(open(path, encoding="utf-8"))


def read_log(path):
    series = {k: [] for k in ("pos", "att", "ctun", "pidp")}
    log = DFReader.DFReader_binary(path)
    while True:
        msg = log.recv_match(type=["XKF1", "ATT", "CTUN", "PIDP"])
        if msg is None:
            break
        typ = msg.get_type()
        t = msg.TimeUS * 1.0e-6
        if typ == "XKF1" and getattr(msg, "C", 0) == 0:
            series["pos"].append((t, msg.PN, msg.PE, -msg.PD, msg.VN, msg.VE, msg.VD))
        elif typ == "ATT":
            series["att"].append((t, msg.Roll, msg.Pitch, msg.Yaw,
                                  msg.DesRoll, msg.DesPitch, msg.DesYaw))
        elif typ == "CTUN":
            series["ctun"].append((t, msg.ThO, msg.CRt * 0.01, msg.DCRt * 0.01))
        elif typ == "PIDP":
            series["pidp"].append((t, msg.I))
    return {k: np.asarray(v, dtype=float) for k, v in series.items()}


def interp(arr, t, column, default=0.0):
    if len(arr) == 0:
        return default
    return float(np.interp(t, arr[:, 0], arr[:, column]))


def arrow(frame, p0, p1, color, thickness=4):
    cv2.arrowedLine(frame, tuple(map(int, p0)), tuple(map(int, p1)), color,
                    thickness, cv2.LINE_AA, tipLength=0.25)


def draw_hexa_top(frame, center, yaw_deg, scale=38, color=CYAN):
    cx, cy = center
    yaw = math.radians(yaw_deg)
    for i in range(6):
        a = yaw + math.radians(30 + i * 60)
        x = int(cx + scale * math.cos(a))
        y = int(cy - scale * math.sin(a))
        cv2.line(frame, (int(cx), int(cy)), (x, y), color, 4, cv2.LINE_AA)
        cv2.circle(frame, (x, y), 11, (14, 19, 27), -1, cv2.LINE_AA)
        cv2.circle(frame, (x, y), 11, color, 2, cv2.LINE_AA)
    cv2.circle(frame, (int(cx), int(cy)), 15, color, -1, cv2.LINE_AA)
    nose = (cx + scale * 0.75 * math.cos(yaw), cy - scale * 0.75 * math.sin(yaw))
    arrow(frame, (cx, cy), nose, WHITE, 3)


def draw_hexa_side(frame, center, pitch_deg, scale=1.0, color=CYAN,
                   ghost_pitch=None):
    cx, cy = center

    def transform(x, y, angle):
        a = math.radians(-angle)
        return (int(cx + scale * (x * math.cos(a) - y * math.sin(a))),
                int(cy + scale * (x * math.sin(a) + y * math.cos(a))))

    if ghost_pitch is not None:
        p0 = transform(-75, 0, ghost_pitch)
        p1 = transform(75, 0, ghost_pitch)
        cv2.line(frame, p0, p1, MUTED, 3, cv2.LINE_AA)
        for x in (-65, -38, -12, 12, 38, 65):
            p = transform(x, -10, ghost_pitch)
            cv2.ellipse(frame, p, (12, 4), 0, 0, 360, MUTED, 2, cv2.LINE_AA)

    p0 = transform(-78, 0, pitch_deg)
    p1 = transform(78, 0, pitch_deg)
    cv2.line(frame, p0, p1, color, 7, cv2.LINE_AA)
    body = transform(0, 5, pitch_deg)
    cv2.circle(frame, body, max(5, int(12 * scale)), color, -1, cv2.LINE_AA)
    for x in (-65, -38, -12, 12, 38, 65):
        p = transform(x, -10, pitch_deg)
        cv2.ellipse(frame, p, (max(5, int(12 * scale)), max(2, int(4 * scale))),
                    0, 0, 360, (12, 17, 24), -1, cv2.LINE_AA)
        cv2.ellipse(frame, p, (max(5, int(12 * scale)), max(2, int(4 * scale))),
                    0, 0, 360, color, 2, cv2.LINE_AA)
    for x in (-35, 35):
        a = transform(x, 10, pitch_deg)
        b = transform(x, 34, pitch_deg)
        cv2.line(frame, a, b, color, 4, cv2.LINE_AA)


def writer(path):
    # avc1 = H.264。原来用 mp4v（MPEG-4 Part 2）——容器是 .mp4，但浏览器、
    # QuickTime 和多数在线播放器都不解这个编码，文件能生成、打不开。
    # 本机 OpenCV 带 FFMPEG，avc1 可用；万一某天不可用再退回 mp4v。
    out = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"avc1"), FPS, (W, H))
    if not out.isOpened():
        out = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"mp4v"), FPS, (W, H))
    if not out.isOpened():
        raise RuntimeError("无法创建 MP4: %s" % path)
    return out


def result_time(result, text):
    for ms, msg in result.get("statustext", []):
        if text in msg:
            return ms * 0.001
    return None


def landing_video(coupled_result, baseline_result, outdir):
    result = load_result(coupled_result)
    baseline = load_result(baseline_result)
    data = read_log(result["dataflash_log"])
    arm_t = result_time(result, "Arming motors")
    land_t = result_time(result, "Mission: 4 Land")
    touch_t = result["touch_sim_ms"] * 0.001
    disarm_t = result_time(result, "Disarming motors") or result["disarm_sim_ms"] * 0.001
    end_t = disarm_t + 0.8
    duration = 18.0
    mp4 = os.path.join(outdir, "hexa-auto-hard-landing.mp4")
    out = writer(mp4)
    gif_frames = []
    story = []

    pos = data["pos"]
    pn_min, pn_max = np.min(pos[:, 1]), np.max(pos[:, 1])
    pe_min, pe_max = np.min(pos[:, 2]), np.max(pos[:, 2])
    margin = 8.0

    for frame_i in range(int(duration * FPS)):
        video_t = frame_i / FPS
        if video_t < 8.0:
            sim_t = arm_t + (land_t - arm_t) * video_t / 8.0
            phase = "AUTO 航点任务"
            top_view = True
        else:
            sim_t = land_t + (end_t - land_t) * (video_t - 8.0) / 10.0
            phase = "末端恒速下降"
            top_view = False

        frame = np.full((H, W, 3), BG, np.uint8)
        cv2.rectangle(frame, (28, 82), (1252, 672), PANEL, -1)
        entries = [
            ("植保六旋翼 · AUTO 航点结束后着陆砸地复现", (36, 20), 30, WHITE, True),
            (phase, (1030, 28), 22, CYAN if top_view else ORANGE, True),
        ]
        pn = interp(pos, sim_t, 1)
        pe = interp(pos, sim_t, 2)
        alt = max(0.0, interp(pos, sim_t, 3))
        vd = interp(pos, sim_t, 6)
        yaw = interp(data["att"], sim_t, 3)
        pitch = interp(data["att"], sim_t, 2)
        throttle = interp(data["ctun"], sim_t, 1)

        if top_view:
            x0, y0, x1, y1 = 85, 125, 900, 625
            for j in range(6):
                x = int(x0 + j * (x1 - x0) / 5)
                cv2.line(frame, (x, y0), (x, y1), GRID, 1)
                y = int(y0 + j * (y1 - y0) / 5)
                cv2.line(frame, (x0, y), (x1, y), GRID, 1)

            def xy(n, e):
                x = x0 + (e - pe_min + margin) / (pe_max - pe_min + 2 * margin) * (x1 - x0)
                y = y1 - (n - pn_min + margin) / (pn_max - pn_min + 2 * margin) * (y1 - y0)
                return int(x), int(y)

            mask = pos[:, 0] <= sim_t
            trail = np.array([xy(n, e) for n, e in pos[mask, 1:3]], np.int32)
            if len(trail) > 1:
                cv2.polylines(frame, [trail], False, CYAN, 3, cv2.LINE_AA)
            for n, e, label in ((0, 0, "起点/落点"), (25, 0, "航点1"), (25, 25, "航点2")):
                p = xy(n, e)
                cv2.circle(frame, p, 8, ORANGE, -1, cv2.LINE_AA)
                entries.append((label, (p[0] + 10, p[1] - 15), 18, WHITE, False))
            draw_hexa_top(frame, xy(pn, pe), yaw, 34)
            entries.extend([
                ("俯视航迹", (90, 92), 22, MUTED, False),
                ("高度  %.1f m" % alt, (955, 150), 26, WHITE, True),
                ("航速  %.1f m/s" % math.hypot(interp(pos, sim_t, 4), interp(pos, sim_t, 5)),
                 (955, 200), 26, WHITE, True),
                ("任务自动飞完后进入 LAND", (955, 285), 20, MUTED, False),
            ])
        else:
            ground_y = 610
            cv2.rectangle(frame, (60, ground_y), (1220, 665), (35, 58, 43), -1)
            cv2.line(frame, (60, ground_y), (1220, ground_y), GREEN, 3)
            zoom = alt < 2.0
            max_alt = 2.2 if zoom else 9.0
            y = ground_y - min(alt, max_alt) / max_alt * 440
            cx = 600
            draw_hexa_side(frame, (cx, int(y)), pitch, 1.05, CYAN)
            arrow_len = min(150, abs(vd) * 220)
            arrow(frame, (760, int(y - 45)), (760, int(y - 45 + arrow_len)), ORANGE, 5)

            # Near-ground aerodynamic region and its collapse zone.
            zone_top = ground_y - 0.35 / max_alt * 440
            collapse_top = ground_y - 0.06 / max_alt * 440
            cv2.rectangle(frame, (110, int(zone_top)), (1090, ground_y), (50, 62, 72), 2)
            cv2.rectangle(frame, (110, int(collapse_top)), (1090, ground_y), (65, 45, 52), -1)
            entries.extend([
                ("侧视落地（%s）" % ("近地放大" if zoom else "全高度"), (90, 96), 22, MUTED, False),
                ("下降速度  %.2f m/s" % vd, (900, 145), 27, ORANGE, True),
                ("离地高度  %.2f m" % alt, (900, 195), 27, WHITE, True),
                ("油门输出  %.3f" % throttle, (900, 245), 27, WHITE, True),
                ("近地增升区", (115, int(zone_top) - 28), 18, MUTED, False),
            ])
            if sim_t >= touch_t:
                age = sim_t - touch_t
                radius = int(25 + min(age, 0.8) * 120)
                cv2.circle(frame, (cx, ground_y), radius, RED, 5, cv2.LINE_AA)
                entries.append(("触地冲击  %.3f m/s" % result["touch_speed_m_s_down"],
                                (440, 510), 30, RED, True))
                if sim_t < disarm_t:
                    entries.append(("落地判定仍未完成  +%.2f s" % age,
                                    (430, 555), 23, YELLOW, True))
            elif touch_t - sim_t < 2.0:
                entries.append(("控制器仍保持约 0.5 m/s 下降，没有拉平", (335, 530), 24, YELLOW, True))

        # Compact A/B touchdown reference.
        entries.extend([
            ("本次触地", (985, 590), 17, MUTED, False),
            ("%.3f m/s" % result["touch_speed_m_s_down"], (1080, 584), 24, RED, True),
            ("无近地项", (985, 625), 17, MUTED, False),
            ("%.3f m/s" % baseline["touch_speed_m_s_down"], (1080, 619), 24, WHITE, True),
        ])
        frame = add_text(frame, entries)
        out.write(frame)
        if frame_i % 3 == 0:
            gif_frames.append(Image.fromarray(cv2.cvtColor(cv2.resize(frame, (640, 360)), cv2.COLOR_BGR2RGB)))
        if frame_i in (60, 240, 450, 525):
            story.append(frame.copy())
    out.release()
    gif = os.path.join(outdir, "hexa-auto-hard-landing.gif")
    gif_frames[0].save(gif, save_all=True, append_images=gif_frames[1:], duration=100, loop=0)
    storyboard = os.path.join(outdir, "hexa-auto-hard-landing-storyboard.png")
    cv2.imwrite(storyboard, np.vstack([cv2.resize(x, (640, 360)) for x in story]))
    return mp4, gif, storyboard


def draw_reverse_panel(frame, rect, data, sim_t, event_t, title, coupled):
    x0, y0, x1, y1 = rect
    cv2.rectangle(frame, (x0, y0), (x1, y1), PANEL, -1)
    pos = data["pos"]
    att = data["att"]
    pid = data["pidp"]
    pn0 = interp(pos, event_t, 1)
    pn = interp(pos, sim_t, 1) - pn0
    vn = interp(pos, sim_t, 4)
    pitch = interp(att, sim_t, 2)
    desired = interp(att, sim_t, 5)
    error = desired - pitch
    i_term = interp(pid, sim_t, 1)
    world_x = int(x0 + (pn + 28) / 56 * (x1 - x0))
    world_x = max(x0 + 90, min(x1 - 90, world_x))
    drone_y = y0 + 245
    draw_hexa_side(frame, (world_x, drone_y), pitch, 0.72,
                   ORANGE if coupled else CYAN, ghost_pitch=desired)
    arrow_len = int(min(135, abs(vn) * 25))
    direction = 1 if vn >= 0 else -1
    arrow(frame, (world_x, drone_y - 85),
          (world_x + direction * arrow_len, drone_y - 85), ORANGE, 5)
    cv2.line(frame, (x0 + 35, drone_y + 55), (x1 - 35, drone_y + 55), GRID, 2)

    # I term bar: this is the slow trim whose sign cannot reverse instantly.
    bar_x = x0 + 115
    bar_y = y1 - 120
    bar_w = x1 - x0 - 230
    cv2.line(frame, (bar_x, bar_y), (bar_x + bar_w, bar_y), GRID, 10)
    center = bar_x + bar_w // 2
    cv2.line(frame, (center, bar_y - 16), (center, bar_y + 16), WHITE, 2)
    end = int(center + np.clip(i_term / 0.07, -1, 1) * bar_w / 2)
    cv2.line(frame, (center, bar_y), (end, bar_y), RED if coupled else GREEN, 10)

    entries = [
        (title, (x0 + 28, y0 + 18), 25, ORANGE if coupled else CYAN, True),
        ("实线：实际姿态   灰线：目标姿态", (x0 + 28, y0 + 58), 17, MUTED, False),
        ("速度  %+.2f m/s" % vn, (x0 + 28, y0 + 100), 23, WHITE, True),
        ("俯仰误差  %+.2f°" % error, (x0 + 300, y0 + 100), 23,
         RED if abs(error) >= 2.5 else WHITE, True),
        ("速率环 I 项（旧配平会滞留）", (bar_x, bar_y - 48), 18, MUTED, False),
        ("%+.3f" % i_term, (x1 - 115, bar_y - 20), 22, RED if coupled else GREEN, True),
    ]
    if -0.15 <= sim_t - event_t <= 0.8:
        entries.append(("反向打满杆", (x0 + 230, y0 + 330), 31, YELLOW, True))
    if coupled:
        entries.append(("速度相关气动力矩已启用", (x0 + 28, y1 - 58), 18, ORANGE, False))
    else:
        entries.append(("默认模型：无速度相关力矩", (x0 + 28, y1 - 58), 18, CYAN, False))
    return entries


def reverse_video(coupled_result, baseline_result, outdir):
    coupled_r = load_result(coupled_result)
    baseline_r = load_result(baseline_result)
    coupled = read_log(coupled_r["dataflash_log"])
    baseline = read_log(baseline_r["dataflash_log"])
    tc = coupled_r["reversals"][0]["sim_ms"] * 0.001
    tb = baseline_r["reversals"][0]["sim_ms"] * 0.001
    duration = 12.0
    mp4 = os.path.join(outdir, "hexa-5ms-reversal-jerk-ab.mp4")
    out = writer(mp4)
    gif_frames = []
    story = []
    for frame_i in range(int(duration * FPS)):
        video_t = frame_i / FPS
        rel = -2.0 + video_t * 9.0 / duration
        frame = np.full((H, W, 3), BG, np.uint8)
        entries = [
            ("植保六旋翼 · 5 m/s 前进后突然反向：抽动复现 A/B", (34, 18), 30, WHITE, True),
            ("t = %+.2f s（0 为遥杆反向）" % rel, (985, 28), 19, YELLOW if abs(rel) < 0.5 else MUTED, True),
        ]
        entries += draw_reverse_panel(frame, (28, 82, 626, 680), baseline, tb + rel, tb,
                                      "A  默认物理模型", False)
        entries += draw_reverse_panel(frame, (654, 82, 1252, 680), coupled, tc + rel, tc,
                                      "B  大桨来流力矩模型", True)
        frame = add_text(frame, entries)
        out.write(frame)
        if frame_i % 3 == 0:
            gif_frames.append(Image.fromarray(cv2.cvtColor(cv2.resize(frame, (640, 360)), cv2.COLOR_BGR2RGB)))
        if frame_i in (45, 80, 110, 190):
            story.append(frame.copy())
    out.release()
    gif = os.path.join(outdir, "hexa-5ms-reversal-jerk-ab.gif")
    gif_frames[0].save(gif, save_all=True, append_images=gif_frames[1:], duration=100, loop=0)
    storyboard = os.path.join(outdir, "hexa-5ms-reversal-jerk-ab-storyboard.png")
    cv2.imwrite(storyboard, np.vstack([cv2.resize(x, (640, 360)) for x in story]))
    return mp4, gif, storyboard


def read_arc_log(path):
    """读协调转弯需要的三条序列：位置、姿态、以及 ARCN 自己的轨迹量。"""
    series = {k: [] for k in ("pos", "att", "arc")}
    log = DFReader.DFReader_binary(path)
    while True:
        msg = log.recv_match(type=["XKF1", "ATT", "ARCN"])
        if msg is None:
            break
        typ = msg.get_type()
        t = msg.TimeUS * 1.0e-6
        if typ == "XKF1" and getattr(msg, "C", 0) == 0:
            series["pos"].append((t, msg.PN, msg.PE, msg.VN, msg.VE))
        elif typ == "ATT":
            series["att"].append((t, msg.Yaw))
        elif typ == "ARCN":
            series["arc"].append((t, msg.Prog, msg.Gov, msg.Spd, msg.Tgt,
                                  msg.PErr, msg.HdgE, msg.Spir, msg.Alat, msg.HdgR))
    return {k: np.asarray(v, dtype=float) for k, v in series.items()}


def _bar(frame, rect, frac, color, bg=PANEL):
    x, y, w, h = rect
    cv2.rectangle(frame, (x, y), (x + w, y + h), bg, -1)
    fill = int(w * max(0.0, min(1.0, frac)))
    if fill > 0:
        cv2.rectangle(frame, (x, y), (x + fill, y + h), color, -1)
    cv2.rectangle(frame, (x, y), (x + w, y + h), GRID, 1)


def uturn_video(result_path, outdir, seconds_pad=2.0):
    """协调转弯的俯视动画。

    这一段最该被看见的不是轨迹本身，而是**机头与期望切线的夹角**：轨迹画得再
    圆，机头没跟上，喷幅方向就不对。所以画面上同时给出两个方向箭头——实际机头
    与期望切线——它们的夹角就是 ARCN.HdgE，肉眼可判。
    """
    result = load_result(result_path)
    data = read_arc_log(result["dataflash_log"])
    arc = data["arc"]
    if len(arc) == 0:
        raise RuntimeError("日志里没有 ARCN，这一架次没有跑协调转弯")
    pos = data["pos"]
    att = data["att"]

    t0 = arc[0, 0] - seconds_pad
    t1 = arc[-1, 0] + seconds_pad
    seg = pos[(pos[:, 0] >= t0) & (pos[:, 0] <= t1)]
    if len(seg) < 2:
        raise RuntimeError("位置数据不足")

    # 视图范围按这段轨迹自适应，留一成边距
    n_lo, n_hi = seg[:, 1].min(), seg[:, 1].max()
    e_lo, e_hi = seg[:, 2].min(), seg[:, 2].max()
    span = max(n_hi - n_lo, e_hi - e_lo, 1.0) * 1.20
    n_mid, e_mid = (n_lo + n_hi) / 2, (e_lo + e_hi) / 2

    MAP_W = 760
    px_per_m = (MAP_W - 80) / span

    def to_px(north, east):
        # 屏幕 x 向右为东，y 向下为南，与俯视图一致
        x = MAP_W / 2 + (east - e_mid) * px_per_m
        y = H / 2 - (north - n_mid) * px_per_m
        return int(x), int(y)

    target_speed = float(arc[0, 4])
    hdg_scale = max(20.0, float(np.abs(arc[:, 6]).max()))

    path = os.path.join(outdir, "uturn_coordinated.mp4")
    out = writer(path)
    trail = []
    for i in range(int((t1 - t0) * FPS)):
        t = t0 + i / FPS
        frame = np.full((H, W, 3), BG, np.uint8)

        north = interp(pos, t, 1)
        east = interp(pos, t, 2)
        yaw_deg = interp(att, t, 1)
        in_arc = arc[0, 0] <= t <= arc[-1, 0]
        prog = interp(arc, t, 1) if in_arc else (0.0 if t < arc[0, 0] else 1.0)
        spd = interp(arc, t, 3) if in_arc else math.hypot(interp(pos, t, 3), interp(pos, t, 4))
        hdg_err = interp(arc, t, 6) if in_arc else 0.0
        hdg_rate = interp(arc, t, 9) if in_arc else 0.0
        gov = interp(arc, t, 2) if in_arc else 1.0
        perr = interp(arc, t, 5) if in_arc else 0.0

        # --- 地图面板 ---
        cv2.rectangle(frame, (0, 0), (MAP_W, H), PANEL, -1)
        for g in range(-40, 41, 5):
            gx, _ = to_px(0, e_mid + g)
            _, gy = to_px(n_mid + g, 0)
            if 0 < gx < MAP_W:
                cv2.line(frame, (gx, 0), (gx, H), GRID, 1)
            if 0 < gy < H:
                cv2.line(frame, (0, gy), (MAP_W, gy), GRID, 1)

        trail.append(to_px(north, east))
        if len(trail) > 1:
            cv2.polylines(frame, [np.array(trail, np.int32)], False, GREEN, 3, cv2.LINE_AA)

        cx, cy = to_px(north, east)
        draw_hexa_top(frame, (cx, cy), yaw_deg, scale=26, color=CYAN)

        # 实际机头方向与期望切线方向；两者夹角就是 HdgE
        L = 92
        yaw = math.radians(yaw_deg)
        arrow(frame, (cx, cy), (cx + L * math.sin(yaw), cy - L * math.cos(yaw)), CYAN, 4)
        tangent = math.radians(yaw_deg + hdg_err)
        arrow(frame, (cx, cy),
              (cx + L * math.sin(tangent), cy - L * math.cos(tangent)), ORANGE, 4)

        # --- 右侧仪表 ---
        px = MAP_W + 30
        entries = [
            ("协调转弯 · 俯视", (24, 20), 30, WHITE, True),
            ("绿=实际航迹  青=机头  橙=期望切线", (24, 60), 20, MUTED, False),
            ("t = %+.2f s" % (t - arc[0, 0]), (px, 24), 24, MUTED, False),
        ]
        y = 76
        entries.append(("速度  %.2f / %.2f m/s" % (spd, target_speed), (px, y), 26, WHITE, True))
        _bar(frame, (px, y + 36, 400, 22), spd / max(target_speed, 0.1),
             GREEN if abs(spd - target_speed) < 0.15 * target_speed else YELLOW)
        y += 84

        entries.append(("航向误差  %+.1f°" % hdg_err, (px, y), 26, WHITE, True))
        entries.append(("机头与期望切线的夹角", (px, y + 30), 18, MUTED, False))
        _bar(frame, (px, y + 56, 400, 22), abs(hdg_err) / hdg_scale,
             GREEN if abs(hdg_err) < 5 else (YELLOW if abs(hdg_err) < 12 else RED))
        y += 104

        entries.append(("指令偏航速率  %+.1f °/s" % hdg_rate, (px, y), 26, WHITE, True))
        _bar(frame, (px, y + 36, 400, 22), abs(hdg_rate) / 90.0, ORANGE)
        y += 84

        entries.append(("参考推进  %.2f" % gov, (px, y), 26, WHITE, True))
        entries.append(("1=跟得上，0.05=参考已停住等飞机", (px, y + 30), 18, MUTED, False))
        _bar(frame, (px, y + 56, 400, 22), gov, GREEN if gov > 0.9 else RED)
        y += 104

        entries.append(("位置误差  %.2f m" % abs(perr), (px, y), 26, WHITE, True))
        y += 44
        entries.append(("掉头进度  %.0f%%" % (prog * 100), (px, y), 26, WHITE, True))
        _bar(frame, (px, y + 36, 400, 22), prog, CYAN)

        frame = add_text(frame, entries)
        out.write(frame)
    out.release()
    return [path]


def uturn_chart(result_path, outdir):
    """一张静态总览图：轨迹 + 速度 + 航向误差 + 指令偏航速率。"""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import font_manager

    for cand in ("Noto Sans CJK SC", "Noto Sans CJK JP", "WenQuanYi Zen Hei"):
        if any(f.name == cand for f in font_manager.fontManager.ttflist):
            plt.rcParams["font.family"] = cand
            break
    plt.rcParams["axes.unicode_minus"] = False

    result = load_result(result_path)
    data = read_arc_log(result["dataflash_log"])
    arc, pos = data["arc"], data["pos"]
    if len(arc) == 0:
        raise RuntimeError("日志里没有 ARCN")
    t0, t1 = arc[0, 0], arc[-1, 0]
    seg = pos[(pos[:, 0] >= t0 - 2) & (pos[:, 0] <= t1 + 2)]
    rel = arc[:, 0] - t0

    fig, ax = plt.subplots(2, 2, figsize=(13, 8.5))
    fig.suptitle("协调转弯 SITL 结果 — %s" % result.get("variant", ""), fontsize=15)

    a = ax[0][0]
    a.plot(seg[:, 2], seg[:, 1], color="#2e8b57", lw=2, label="实际航迹")
    a.set_aspect("equal"); a.grid(alpha=.3)
    a.set_xlabel("东 (m)"); a.set_ylabel("北 (m)"); a.set_title("俯视航迹")
    a.legend(fontsize=9)

    a = ax[0][1]
    a.plot(rel, arc[:, 3], color="#2e8b57", lw=1.8, label="实际速度")
    a.axhline(arc[0, 4], color="#d95f02", ls="--", lw=1.5, label="目标速度")
    a.set_ylim(0, max(arc[0, 4] * 1.4, arc[:, 3].max() * 1.1))
    a.grid(alpha=.3); a.set_xlabel("弧内时间 (s)"); a.set_ylabel("m/s")
    a.set_title("速度保持（掉速 %.1f%%）"
                % (100 * (1 - arc[:, 3].min() / max(arc[0, 4], 1e-6))))
    a.legend(fontsize=9)

    a = ax[1][0]
    a.plot(rel, arc[:, 6], color="#7570b3", lw=1.8)
    a.axhline(0, color="k", lw=.8)
    a.grid(alpha=.3); a.set_xlabel("弧内时间 (s)"); a.set_ylabel("度")
    a.set_title("机头相对期望切线的误差（均值 %.2f°，峰值 %.2f°）"
                % (np.abs(arc[:, 6]).mean(), np.abs(arc[:, 6]).max()))

    a = ax[1][1]
    a.plot(rel, arc[:, 9], color="#d95f02", lw=1.8, label="指令偏航速率")
    a.plot(rel, arc[:, 8] * 10, color="#1b9e77", lw=1.2, ls=":", label="指令横向加速度 ×10")
    a.grid(alpha=.3); a.set_xlabel("弧内时间 (s)")
    a.set_title("偏航速率与横向加速度剖面（螺线 %.2f m）" % arc[0, 7])
    a.legend(fontsize=9)

    fig.tight_layout(rect=[0, 0, 1, 0.96])
    path = os.path.join(outdir, "uturn_overview.png")
    fig.savefig(path, dpi=130)
    plt.close(fig)
    return [path]



# ── P04 单动力失效 ────────────────────────────────────────────────────
# 混控因子对应的臂角（AP_MotorsMatrix::setup_hexa_matrix，HEXA/DJI_X）。
# 角度按 AP 的约定：机头为 0，顺时针为正。输出通道 1..6 依次对应。
HEXA_ARM_DEG = [30.0, -30.0, -90.0, -150.0, 150.0, 90.0]
HEXA_SPIN = ["CCW", "CW", "CCW", "CW", "CCW", "CW"]
AMBER = (40, 150, 220)


def read_motor_fail_log(path):
    """取姿态与逐电机输出。RCOU 是**实际发出去的 PWM**，比推力指令更贴近现场看到的。"""
    att, rcou, malc = [], [], []
    log = DFReader.DFReader_binary(path)
    while True:
        m = log.recv_match(type=["ATT", "RCOU", "MALC"])
        if m is None:
            break
        t = m.TimeUS * 1.0e-6
        k = m.get_type()
        if k == "ATT":
            att.append((t, m.Roll, m.Pitch, m.Yaw, m.DesRoll, m.DesPitch))
        elif k == "RCOU":
            rcou.append((t,) + tuple(getattr(m, "C%d" % i, 0) for i in range(1, 7)))
        elif k == "MALC":
            malc.append((t, getattr(m, "Res", 0)))
    return {"att": np.asarray(att, float), "rcou": np.asarray(rcou, float),
            "malc": np.asarray(malc, float)}


def _unwrap_yaw(att, t_fail):
    """累计转动。wrap180 的航向误差看不出转圈，而机头扫过多少度才是飞手感受到的。"""
    if len(att) == 0:
        return np.zeros(0), np.zeros(0)
    ts, yaw = att[:, 0], att[:, 3]
    out = [0.0]
    for i in range(1, len(yaw)):
        d = yaw[i] - yaw[i - 1]
        while d > 180:
            d -= 360
        while d < -180:
            d += 360
        out.append(out[-1] + d)
    out = np.asarray(out)
    base = float(np.interp(t_fail, ts, out))
    return ts, out - base


def _draw_hexa_p04(frame, center, radius, yaw_deg, pwms, failed_idx, ceiling=1905.0):
    """俯视六旋翼：每台电机的圆盘大小与颜色随实际 PWM 变，失效那台画叉。

    尺寸用 (pwm-1050)/(ceiling-1050) —— 1050 是 MOT_PWM_MIN，1905 是有效上限
    （1050+(1950-1050)*0.95）。之所以不用 1950：那不是能达到的值，用它会让所有
    圆盘都偏小、看起来永远有余量。
    """
    cx, cy = center
    yaw = math.radians(yaw_deg)
    cv2.circle(frame, (int(cx), int(cy)), int(radius * 1.18), PANEL, -1, cv2.LINE_AA)
    for i, arm in enumerate(HEXA_ARM_DEG):
        a = yaw + math.radians(arm)          # 机头方向 + 臂角
        x = cx + radius * math.sin(a)
        y = cy - radius * math.cos(a)
        pwm = pwms[i] if i < len(pwms) else 1050.0
        frac = max(0.0, min(1.0, (pwm - 1050.0) / (ceiling - 1050.0)))
        dead = (failed_idx is not None and i == failed_idx)
        cv2.line(frame, (int(cx), int(cy)), (int(x), int(y)),
                 GRID if dead else WHITE, 3, cv2.LINE_AA)
        if dead:
            r = int(radius * 0.16)
            cv2.circle(frame, (int(x), int(y)), r, (60, 60, 70), -1, cv2.LINE_AA)
            cv2.circle(frame, (int(x), int(y)), r, RED, 2, cv2.LINE_AA)
            d = int(r * 0.6)
            cv2.line(frame, (int(x - d), int(y - d)), (int(x + d), int(y + d)), RED, 3, cv2.LINE_AA)
            cv2.line(frame, (int(x + d), int(y - d)), (int(x - d), int(y + d)), RED, 3, cv2.LINE_AA)
        else:
            r = int(radius * (0.11 + 0.16 * frac))
            col = GREEN if frac < 0.55 else (AMBER if frac < 0.8 else RED)
            cv2.circle(frame, (int(x), int(y)), r, col, -1, cv2.LINE_AA)
            cv2.circle(frame, (int(x), int(y)), r, WHITE, 1, cv2.LINE_AA)
    nose = (cx + radius * 0.62 * math.sin(yaw), cy - radius * 0.62 * math.cos(yaw))
    arrow(frame, (cx, cy), nose, CYAN, 4)
    cv2.circle(frame, (int(cx), int(cy)), 6, CYAN, -1, cv2.LINE_AA)


def motor_fail_video(off_result, on_result, outdir, watch_s=24.0):
    """左右对照：同一工况、同一时刻，唯一差别是降级重分配开不开。

    这是 P04 全部工作的一句话总结——摘掉一列之后前向混控的力矩不再平衡，
    而重分配把三个硬约束重新解回去。视频要让人不看数字也能看出这件事。
    """
    # **左右两栏由数据决定，不由参数顺序决定。** 上一版按位置贴标签，结果传参一颠倒
    # 就把"不降级"扣在了降级那一栏上——而画面本身完全正常，只有把累计转动跟
    # result.json 的总量对一下才会发现。标签错了的图比没有图更糟。
    runs = []
    for path in (off_result, on_result):
        res = load_result(path)
        ov = res.get("param_overrides") or {}
        alloc = ov.get("MOT_FAIL_ALLOC")
        if alloc is None:
            alloc = 1.0 if res.get("degraded_mixer") else 0.0
        on = float(alloc) > 0.5
        title = "降级重分配" if on else "不降级"
        sub = ("ALLOC = 1  YTRK = %g" % float(ov.get("MOT_FAIL_YTRK", 0))
               if on else "MOT_FAIL_ALLOC = 0  前向混控")
        log = os.path.join(os.path.dirname(path), "logs")
        cands = sorted(glob.glob(os.path.join(log, "*.EFT")) +
                       glob.glob(os.path.join(log, "*.BIN")))
        if not cands:
            raise SystemExit("找不到日志: %s" % log)
        d = read_motor_fail_log(cands[-1])
        t_fail = (res.get("fail_time_ms") or
                  res.get("metrics", {}).get("fail_time_ms") or 0) / 1000.0
        ts, unw = _unwrap_yaw(d["att"], t_fail)
        runs.append({"res": res, "d": d, "t_fail": t_fail, "ts": ts, "unw": unw,
                     "title": title, "sub": sub, "on": on,
                     "failed": (res.get("failed_motor") or 6) - 1})
    # 不降级在左、降级在右——阅读顺序是"问题→解决"。
    runs.sort(key=lambda r: r["on"])
    if len(runs) == 2 and runs[0]["on"] == runs[1]["on"]:
        raise SystemExit("两个架次的 MOT_FAIL_ALLOC 相同，构不成对照")

    path = os.path.join(outdir, "motor_fail_compare.mp4")
    out = writer(path)
    n = int(FPS * (watch_s + 3.0))
    for f in range(n):
        rel = f / float(FPS) - 3.0        # 失效前留 3 秒
        frame = np.full((H, W, 3), BG, np.uint8)
        texts = [("P04 单动力失效 — 摘掉一台电机之后", (40, 18), 30, WHITE, True),
                 ("六旋翼 HEXA/DJI_X · 6 号电机 · 4 m/s 侧风 · 同一工况对照",
                  (40, 56), 17, GRID, False)]
        for side, r in enumerate(runs):
            x0 = 40 + side * 620
            cv2.rectangle(frame, (x0, 92), (x0 + 580, 632), PANEL, -1)
            cv2.rectangle(frame, (x0, 92), (x0 + 580, 632), GRID, 1)
            t = r["t_fail"] + rel
            att, rcou = r["d"]["att"], r["d"]["rcou"]
            # DataFlash 的 ATT.Roll/Pitch/Yaw **本来就是度**——不要再乘 57.3。
            # （MAVLink 的 ATTITUDE 消息才是弧度，两者容易混。）
            roll = interp(att, t, 1)
            pitch = interp(att, t, 2)
            yawd = interp(att, t, 3)
            pwms = [interp(rcou, t, c) for c in range(1, 7)]
            fired = rel >= 0.0
            rot = float(np.interp(t, r["ts"], r["unw"])) if len(r["ts"]) else 0.0
            _draw_hexa_p04(frame, (x0 + 290, 268), 122, yawd, pwms,
                           r["failed"] if fired else None)
            bad = abs(roll) > 10.0
            texts += [
                (r["title"], (x0 + 22, 104), 26, GREEN if r["on"] else RED, True),
                (r["sub"], (x0 + 22, 138), 15, GRID, False),
                ("滚转", (x0 + 40, 470), 16, GRID, False),
                ("%+.1f°" % roll, (x0 + 40, 492), 34, RED if bad else WHITE, True),
                ("累计转动", (x0 + 210, 470), 16, GRID, False),
                ("%+.0f°" % rot, (x0 + 210, 492), 34, WHITE, True),
                ("俯仰", (x0 + 410, 470), 16, GRID, False),
                ("%+.1f°" % pitch, (x0 + 410, 492), 34, WHITE, True),
            ]
            # 逐电机输出条：一眼看出重分配把推力重新摊到了哪几台
            for i in range(6):
                bx = x0 + 40 + i * 88
                frac = max(0.0, min(1.0, (pwms[i] - 1050.0) / 855.0))
                dead = fired and i == r["failed"]
                _bar(frame, (bx, 560, 70, 16), 0.0 if dead else frac,
                     RED if dead else (GREEN if frac < 0.55 else AMBER))
                texts.append(("%d" % (i + 1), (bx + 28, 580), 14,
                              RED if dead else GRID, False))
            texts.append(("逐电机输出 (PWM 1050→1905)", (x0 + 40, 536), 15, GRID, False))
            # 滚转时间曲线。静止帧看不出差别——峰值只持续一瞬，而它正是问题本身。
            gx, gy, gw, gh = x0 + 40, 408, 500, 54
            cv2.rectangle(frame, (gx, gy), (gx + gw, gy + gh), (30, 22, 16), -1)
            cv2.line(frame, (gx, gy + gh // 2), (gx + gw, gy + gh // 2), GRID, 1)
            span, lim = watch_s, 18.0          # ±18° 铺满，10° 判据线画出来
            for sgn in (1, -1):
                yy = int(gy + gh / 2 - sgn * 10.0 / lim * gh / 2)
                cv2.line(frame, (gx, yy), (gx + gw, yy), (60, 48, 38), 1)
            pts = []
            for k in range(gw):
                tt = r["t_fail"] + (k / float(gw)) * span
                if tt > t:
                    break
                v = interp(att, tt, 1)
                pts.append((gx + k, int(gy + gh / 2 - max(-lim, min(lim, v)) / lim * gh / 2)))
            if len(pts) > 1:
                cv2.polylines(frame, [np.asarray(pts, np.int32)], False,
                              GREEN if r["on"] else RED, 2, cv2.LINE_AA)
            texts.append(("滚转 ±18°（细线 = 10° 判据）", (gx, gy - 20), 14, GRID, False))
        # 时间轴
        cv2.rectangle(frame, (40, 654), (1240, 686), PANEL, -1)
        prog = max(0.0, min(1.0, (rel + 3.0) / (watch_s + 3.0)))
        cv2.rectangle(frame, (40, 654), (40 + int(1200 * prog), 686), (90, 70, 55), -1)
        mark = 40 + int(1200 * (3.0 / (watch_s + 3.0)))
        cv2.line(frame, (mark, 648), (mark, 692), RED, 2, cv2.LINE_AA)
        texts += [("注入", (mark - 20, 694), 14, RED, True),
                  ("t = %+.1f s" % rel, (1100, 660), 20,
                   WHITE if rel >= 0 else GRID, True)]
        frame = add_text(frame, texts)
        out.write(frame)
    out.release()
    return [path]



# ── P04 三维姿态视频 ──────────────────────────────────────────────────
W3, H3 = 1600, 900


def _rot_body_to_world(roll_d, pitch_d, yaw_d):
    """机体→世界（NED）。航空标准 3-2-1：先滚转，再俯仰，最后偏航。"""
    r, p, y = (math.radians(v) for v in (roll_d, pitch_d, yaw_d))
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ])


class Cam:
    """固定机位的透视相机。

    NED 转成显示坐标：右=东(y)、上=-下(-z)、进屏=北(x)。相机绕方位角与俯仰角
    转动后做透视投影。机位固定不动，飞机才是动的——这样滚转、俯仰、偏航都能
    直接看出来，而地面网格提供参照，否则光看飞机分不清是它在转还是视角在转。
    """

    def __init__(self, az_deg=38.0, el_deg=24.0, dist=13.0, f=900.0,
                 center=(W3 // 4, 300)):
        self.az, self.el, self.d, self.f, self.c = (math.radians(az_deg),
                                                    math.radians(el_deg),
                                                    dist, f, center)

    def project(self, pts):
        p = np.atleast_2d(np.asarray(pts, float))
        disp = np.stack([p[:, 1], -p[:, 2], p[:, 0]], axis=1)   # 右, 上, 进屏
        ca, sa = math.cos(self.az), math.sin(self.az)
        x = disp[:, 0] * ca - disp[:, 2] * sa
        z = disp[:, 0] * sa + disp[:, 2] * ca
        ce, se = math.cos(self.el), math.sin(self.el)
        y = disp[:, 1] * ce - z * se
        z = disp[:, 1] * se + z * ce
        zc = np.maximum(z + self.d, 0.35)
        return np.stack([self.c[0] + self.f * x / zc,
                         self.c[1] - self.f * y / zc], axis=1)


def _draw_ground(frame, cam, half=6.0, step=1.0, z=1.9, clip_y=None):
    """地面网格：没有它，飞机在转还是视角在转分不出来。

    clip_y 把网格限制在三维区内。不限的话线会一路铺到下面的曲线区，
    读数时背景全是斜网格，比没有网格更糟。
    """
    sub = frame if clip_y is None else frame[:clip_y]
    n = int(half / step)
    for i in range(-n, n + 1):
        for seg in (cam.project([[i * step, -half, z], [i * step, half, z]]),
                    cam.project([[-half, i * step, z], [half, i * step, z]])):
            cv2.line(sub, tuple(seg[0].astype(int)), tuple(seg[1].astype(int)),
                     (58, 46, 36), 1, cv2.LINE_AA)


def _draw_craft3d(frame, cam, roll, pitch, yaw, pwms, failed, arm=1.5):
    R = _rot_body_to_world(roll, pitch, yaw)
    hub = np.zeros(3)
    order = []
    for i, a in enumerate(HEXA_ARM_DEG):
        th = math.radians(a)
        tip_b = np.array([arm * math.cos(th), arm * math.sin(th), 0.0])
        tip_w = R @ tip_b
        order.append((float((R @ tip_b)[0]), i, tip_b, tip_w))
    order.sort(key=lambda t: t[0])            # 远的先画，近的压在上面
    ph = cam.project([hub])[0]
    for _, i, tip_b, tip_w in order:
        pt = cam.project([tip_w])[0]
        dead = failed is not None and i == failed
        cv2.line(frame, tuple(ph.astype(int)), tuple(pt.astype(int)),
                 GRID if dead else WHITE, 3, cv2.LINE_AA)
        # 桨盘：机体系里的圆，跟着姿态一起转
        ring = []
        for k in range(0, 361, 12):
            t = math.radians(k)
            ring.append(tip_b + np.array([0.42 * math.cos(t), 0.42 * math.sin(t), 0.0]))
        rp = cam.project([R @ q for q in ring]).astype(np.int32)
        if dead:
            cv2.polylines(frame, [rp], True, RED, 2, cv2.LINE_AA)
            d = 12
            cv2.line(frame, (int(pt[0] - d), int(pt[1] - d)),
                     (int(pt[0] + d), int(pt[1] + d)), RED, 3, cv2.LINE_AA)
            cv2.line(frame, (int(pt[0] + d), int(pt[1] - d)),
                     (int(pt[0] - d), int(pt[1] + d)), RED, 3, cv2.LINE_AA)
        else:
            frac = max(0.0, min(1.0, (pwms[i] - 1050.0) / 855.0))
            col = GREEN if frac < 0.55 else (AMBER if frac < 0.8 else RED)
            cv2.fillPoly(frame, [rp], tuple(int(c * (0.25 + 0.75 * frac)) for c in col))
            cv2.polylines(frame, [rp], True, col, 2, cv2.LINE_AA)
    nose_w = R @ np.array([arm * 1.5, 0.0, 0.0])
    arrow(frame, ph, cam.project([nose_w])[0], CYAN, 3)
    cv2.circle(frame, tuple(ph.astype(int)), 7, CYAN, -1, cv2.LINE_AA)


def _chart(frame, rect, series, t_now, span, lo, hi, title, hint=None):
    """一张曲线图，两条轨迹叠在一起。series = [(数组, 颜色, 名字)]。"""
    x, y, w, h = rect
    cv2.rectangle(frame, (x, y), (x + w, y + h), (30, 22, 16), -1)
    cv2.rectangle(frame, (x, y), (x + w, y + h), GRID, 1)
    zero = int(y + h * (hi - 0.0) / (hi - lo)) if lo < 0 < hi else None
    if zero is not None:
        cv2.line(frame, (x, zero), (x + w, zero), (70, 56, 44), 1)
    if hint is not None:
        for sgn in (1, -1):
            yy = int(y + h * (hi - sgn * hint) / (hi - lo))
            if y < yy < y + h:
                cv2.line(frame, (x, yy), (x + w, yy), (64, 78, 96), 1)
    for arr, col, _ in series:
        pts = []
        for k in range(w):
            tt = (k / float(w)) * span
            if tt > t_now:
                break
            v = float(np.interp(tt, arr[:, 0], arr[:, 1]))
            v = max(lo, min(hi, v))
            pts.append((x + k, int(y + h * (hi - v) / (hi - lo))))
        if len(pts) > 1:
            cv2.polylines(frame, [np.asarray(pts, np.int32)], False, col, 2, cv2.LINE_AA)
    px = x + int(w * min(1.0, t_now / span))
    cv2.line(frame, (px, y), (px, y + h), (120, 100, 80), 1)
    return title


def motor_fail_3d(on_result, ref_result, outdir, watch_s=22.0):
    """三维姿态 + 同步曲线，主角是**我们的策略**：降级重分配。

    ref_result（不降级）只作为曲线里的淡色参照，不占画面——策略已经定了，
    要看的是它工作时飞机是什么状态，而不是再论证一遍该不该用它。
    """
    def load(path):
        res = load_result(path)
        ov = res.get("param_overrides") or {}
        logs = sorted(glob.glob(os.path.join(os.path.dirname(path), "logs", "*.EFT")) +
                      glob.glob(os.path.join(os.path.dirname(path), "logs", "*.BIN")))
        d = read_motor_fail_log(logs[-1])
        t_fail = (res.get("fail_time_ms") or
                  res.get("metrics", {}).get("fail_time_ms") or 0) / 1000.0
        ts, unw = _unwrap_yaw(d["att"], t_fail)
        att = d["att"]
        rel = att[:, 0] - t_fail
        return {"res": res, "ov": ov, "d": d, "t_fail": t_fail,
                "failed": (res.get("failed_motor") or 6) - 1,
                # 用**误差**（实际−期望）而不是原始角：侧风下飞机要压向风，
                # 原始滚转里含着配平量，与 result.json 的「滚转稳态」不是一回事，
                # 直接显示会出现"最好那栏数字更大"的自相矛盾。
                "roll": np.stack([rel, att[:, 1] - att[:, 4]], 1),
                "pitch": np.stack([rel, att[:, 2] - att[:, 5]], 1),
                "rot": np.stack([ts - t_fail, unw], 1)}

    m = load(on_result)
    ref = load(ref_result) if ref_result else None
    fi = m["failed"]

    path = os.path.join(outdir, "motor_fail_3d.mp4")
    out = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"avc1"), FPS, (W3, H3))
    if not out.isOpened():
        out = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"mp4v"), FPS, (W3, H3))
    pre, DIM = 3.0, (70, 58, 96)
    for f in range(int(FPS * (watch_s + pre))):
        rel = f / float(FPS) - pre
        fired = rel >= 0.0
        frame = np.full((H3, W3, 3), BG, np.uint8)
        t = m["t_fail"] + rel
        att, rcou = m["d"]["att"], m["d"]["rcou"]
        roll = interp(att, t, 1) - interp(att, t, 4)
        pitch = interp(att, t, 2) - interp(att, t, 5)
        yaw = interp(att, t, 3)
        pwms = [interp(rcou, t, c) for c in range(1, 7)]
        rot = float(np.interp(rel, m["rot"][:, 0], m["rot"][:, 1]))

        cam = Cam(az_deg=36.0, el_deg=22.0, dist=11.5, f=1050.0, center=(470, 300))
        _draw_ground(frame, cam, clip_y=520)
        _draw_craft3d(frame, cam, interp(att, t, 1), interp(att, t, 2), yaw, pwms,
                      fi if fired else None, arm=1.6)

        texts = [("P04 单动力失效 — 降级重分配", (44, 20), 34, WHITE, True),
                 ("HEXA/DJI_X · %d 号电机停转 · 4 m/s 侧风 · MOT_FAIL_ALLOC=1  YTRK=%g"
                  % (fi + 1, float(m["ov"].get("MOT_FAIL_YTRK", 0))), (44, 62), 18, GRID, False),
                 ("失效前" if not fired else "失效后 %+.1f s" % rel, (44, 96), 26,
                  GRID if not fired else GREEN, True)]

        # 右上：逐电机推力再分配——这是策略本身在做的事
        px, py = 960, 120
        cv2.rectangle(frame, (px, py), (px + 580, py + 330), PANEL, -1)
        cv2.rectangle(frame, (px, py), (px + 580, py + 330), GRID, 1)
        texts.append(("推力再分配（PWM 1050 → 1905 有效上限）", (px + 18, py + 12), 19, WHITE, True))
        for i in range(6):
            yy = py + 56 + i * 44
            dead = fired and i == fi
            frac = 0.0 if dead else max(0.0, min(1.0, (pwms[i] - 1050.0) / 855.0))
            col = RED if dead else (GREEN if frac < 0.55 else (AMBER if frac < 0.8 else RED))
            _bar(frame, (px + 92, yy, 380, 24), frac, col)
            texts += [("%d 号 %s" % (i + 1, HEXA_SPIN[i]), (px + 18, yy + 2), 17,
                       RED if dead else GRID, dead),
                      ("停转" if dead else "%d" % int(pwms[i]), (px + 486, yy + 2), 17,
                       RED if dead else WHITE, True)]
        texts.append(("正对失效那台（%d 号）被压向零推力——滚转平衡要求对侧减载"
                      % (((fi + 3) % 6) + 1), (px + 18, py + 300), 15, GRID, False))

        texts.append(("滚转误差 %+.1f°    俯仰误差 %+.1f°    累计转动 %+.0f°"
                      % (roll, pitch, rot), (44, 470), 26,
                      RED if abs(roll) > 10 else WHITE, True))

        cv2.rectangle(frame, (0, 524), (W3, H3), BG, -1)
        base = 566
        specs = [("滚转误差 °（细线 = ±10° 判据）", "roll", -20.0, 20.0, 10.0),
                 ("俯仰误差 °", "pitch", -20.0, 20.0, None),
                 ("累计转动 °", "rot", -60.0, 480.0, None)]
        for i, (title, key, lo, hi, hint) in enumerate(specs):
            rect = (60, base + i * 108, 1480, 88)
            series = ([(ref[key], DIM, "ref")] if ref else []) + [(m[key], GREEN, "on")]
            _chart(frame, rect, series, rel + pre, watch_s + pre, lo, hi, title, hint)
            texts.append((title, (66, base + i * 108 - 21), 17, GRID, False))
            mx = 60 + int(1480 * (pre / (watch_s + pre)))
            cv2.line(frame, (mx, base + i * 108), (mx, base + i * 108 + 88), RED, 1)
        texts += [("注入", (60 + int(1480 * (pre / (watch_s + pre))) - 18, base - 42),
                   15, RED, True),
                  ("绿 = 降级重分配（本策略）　　灰 = 不降级（仅作参照）",
                   (60, base + 3 * 108 - 16), 16, GRID, False)]
        out.write(add_text(frame, texts))
    out.release()
    return [path]



def motor_fail_3d_pair(a_result, b_result, outdir, watch_s=20.0,
                       labels=None, title=None, sub=None):
    """两个失效位置的三维对照。都开降级重分配，差别只在停的是哪一台。

    用来回答"最好与最差差多少"。左右由**滚转稳态**排序决定，不由参数顺序决定
    ——按位置贴标签的图一旦传参颠倒就会说反话，而画面看起来完全正常。
    """
    def load(path):
        res = load_result(path)
        ov = res.get("param_overrides") or {}
        logs = sorted(glob.glob(os.path.join(os.path.dirname(path), "logs", "*.EFT")) +
                      glob.glob(os.path.join(os.path.dirname(path), "logs", "*.BIN")))
        d = read_motor_fail_log(logs[-1])
        tf = (res.get("fail_time_ms") or
              res.get("metrics", {}).get("fail_time_ms") or 0) / 1000.0
        ts, unw = _unwrap_yaw(d["att"], tf)
        att = d["att"]
        rel = att[:, 0] - tf
        met = res.get("metrics", {})
        gm = lambda k: res.get(k, met.get(k))
        return {"res": res, "ov": ov, "d": d, "t_fail": tf,
                "failed": (res.get("failed_motor") or 6) - 1,
                "steady": abs(gm("roll_steady_deg") or 0.0),
                "peak": gm("roll_err_max_deg"), "rot": gm("yaw_total_rotation_deg"),
                "wind": float(ov.get("SIM_WIND_SPD", 0)),
                "roll": np.stack([rel, att[:, 1] - att[:, 4]], 1),
                "pitch": np.stack([rel, att[:, 2] - att[:, 5]], 1),
                "rotc": np.stack([ts - tf, unw], 1)}

    runs = [load(a_result), load(b_result)]
    if labels is None:
        runs.sort(key=lambda r: abs(r["peak"]))
    a0 = float(runs[0]["ov"].get("MOT_FAIL_ALLOC", 1))
    wind = runs[0]["wind"]
    global TITLE3, SUB3
    TITLE3 = title or ("P04 %s — 最好与最差的失效位置"
                       % ("降级重分配" if a0 > 0.5 else "前向混控（未降级）"))
    SUB3 = sub or ("%s、MOT_FAIL_ALLOC=%d，唯一差别是停的哪一台"
                   % ("无风" if wind < 0.5 else "%g m/s 侧风" % wind, int(a0)))
    if labels:
        for r, lab in zip(runs, labels):
            r["tag"] = lab
        runs[0]["col"], runs[1]["col"] = RED, GREEN
    else:
        runs[0]["tag"], runs[0]["col"] = "表现最好", GREEN
        runs[1]["tag"], runs[1]["col"] = "表现最差", RED
    # 按**滚转误差峰值**排序：稳态在无风下都接近 0，分不出来；峰值才是几何差异所在。

    path = os.path.join(outdir, "motor_fail_3d_best_worst.mp4")
    out = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"avc1"), FPS, (W3, H3))
    if not out.isOpened():
        out = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"mp4v"), FPS, (W3, H3))
    pre = 3.0
    for f in range(int(FPS * (watch_s + pre))):
        rel = f / float(FPS) - pre
        fired = rel >= 0.0
        frame = np.full((H3, W3, 3), BG, np.uint8)
        texts = [(TITLE3, (44, 18), 34, WHITE, True),
                 (SUB3, (44, 60), 18, GRID, False),
                 ("失效前" if not fired else "失效后 %+.1f s" % rel, (1360, 26), 26,
                  GRID if not fired else WHITE, True)]
        for side, r in enumerate(runs):
            cam = Cam(az_deg=36.0, el_deg=22.0, dist=12.5, f=980.0,
                      center=(410 + side * 790, 300))
            _draw_ground(frame, cam, clip_y=500)
            t = r["t_fail"] + rel
            att, rcou = r["d"]["att"], r["d"]["rcou"]
            roll = interp(att, t, 1) - interp(att, t, 4)
            pitch = interp(att, t, 2) - interp(att, t, 5)
            pwms = [interp(rcou, t, c) for c in range(1, 7)]
            _draw_craft3d(frame, cam, interp(att, t, 1), interp(att, t, 2), interp(att, t, 3), pwms,
                          r["failed"] if fired else None, arm=1.5)
            x0 = 70 + side * 790
            rot = float(np.interp(rel, r["rotc"][:, 0], r["rotc"][:, 1]))
            texts += [
                (r["tag"] if labels else "%s ─ 停 %d 号 (%s)"
                 % (r["tag"], r["failed"] + 1, HEXA_SPIN[r["failed"]]),
                 (x0, 96), 27, r["col"], True),
                ("滚转峰值 %.2f°   稳态 %.2f°   累计转 %.0f°"
                 % (r["peak"], r["steady"], r["rot"]), (x0, 132), 17, GRID, False),
                ("滚转误差 %+.1f°    俯仰误差 %+.1f°    累计转动 %+.0f°"
                 % (roll, pitch, rot), (x0, 452), 23,
                 RED if abs(roll) > 10 else WHITE, True),
            ]
            for i in range(6):
                bx = x0 + i * 110
                dead = fired and i == r["failed"]
                frac = 0.0 if dead else max(0.0, min(1.0, (pwms[i] - 1050.0) / 855.0))
                _bar(frame, (bx, 492, 88, 18), frac,
                     RED if dead else (GREEN if frac < 0.55 else AMBER))
                texts.append(("%d" % (i + 1), (bx + 38, 514), 15,
                              RED if dead else GRID, dead))
        cv2.line(frame, (790, 92), (790, 470), GRID, 1)
        cv2.rectangle(frame, (0, 540), (W3, H3), BG, -1)
        base = 578
        for i, (title, key, lo, hi, hint) in enumerate(
                [("滚转误差 °（细线 = ±10° 判据）", "roll", -20.0, 20.0, 10.0),
                 ("累计转动 °", "rotc", -60.0, 420.0, None)]):
            rect = (60, base + i * 150, 1480, 118)
            _chart(frame, rect, [(r[key], r["col"], r["tag"]) for r in runs],
                   rel + pre, watch_s + pre, lo, hi, title, hint)
            texts.append((title, (66, base + i * 150 - 22), 17, GRID, False))
            mx = 60 + int(1480 * (pre / (watch_s + pre)))
            cv2.line(frame, (mx, base + i * 150), (mx, base + i * 150 + 118), RED, 1)
        texts += [("注入", (60 + int(1480 * (pre / (watch_s + pre))) - 18, base - 44),
                   15, RED, True),
                  ("红 = %s    绿 = %s" % (runs[0]["tag"], runs[1]["tag"]),
                   (60, base + 2 * 150 - 18), 16, GRID, False)]
        out.write(add_text(frame, texts))
    out.release()
    return [path]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("case", choices=("landing", "reverse", "uturn", "motor-fail", "motor-fail-3d", "motor-fail-3d-pair"))
    parser.add_argument("coupled_result")
    parser.add_argument("baseline_result", nargs="?",
                        help="uturn 不需要基线架次")
    parser.add_argument("--output", required=True)
    parser.add_argument("--labels", help="两栏标签，用 | 分隔；给了就按参数顺序排")
    parser.add_argument("--title")
    parser.add_argument("--sub")
    args = parser.parse_args()
    os.makedirs(args.output, exist_ok=True)
    if args.case == "motor-fail-3d-pair":
        paths = motor_fail_3d_pair(
            args.coupled_result, args.baseline_result, args.output,
            labels=(args.labels.split("|") if args.labels else None),
            title=args.title, sub=args.sub)
    elif args.case == "motor-fail-3d":
        paths = motor_fail_3d(args.coupled_result, args.baseline_result, args.output)
    elif args.case == "motor-fail":
        paths = motor_fail_video(args.coupled_result, args.baseline_result, args.output)
    elif args.case == "landing":
        paths = landing_video(args.coupled_result, args.baseline_result, args.output)
    elif args.case == "uturn":
        paths = uturn_video(args.coupled_result, args.output)
        paths += uturn_chart(args.coupled_result, args.output)
    else:
        paths = reverse_video(args.coupled_result, args.baseline_result, args.output)
    for path in paths:
        print(path, os.path.getsize(path))


if __name__ == "__main__":
    main()
