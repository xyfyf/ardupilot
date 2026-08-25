#!/usr/bin/env python3
"""把 reproduce.py 的六旋翼 SITL 日志渲染成直观 MP4/GIF。

用法:
  render_video.py landing <coupled-result.json> <baseline-result.json> --output out
  render_video.py reverse <coupled-result.json> <baseline-result.json> --output out
"""

import argparse
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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("case", choices=("landing", "reverse"))
    parser.add_argument("coupled_result")
    parser.add_argument("baseline_result")
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    os.makedirs(args.output, exist_ok=True)
    if args.case == "landing":
        paths = landing_video(args.coupled_result, args.baseline_result, args.output)
    else:
        paths = reverse_video(args.coupled_result, args.baseline_result, args.output)
    for path in paths:
        print(path, os.path.getsize(path))


if __name__ == "__main__":
    main()
