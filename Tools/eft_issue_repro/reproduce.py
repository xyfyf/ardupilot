#!/usr/bin/env python3
"""EFT 植保六旋翼两个飞行问题的一键 SITL 闭环复现。

场景:
  landing  地面站式 AUTO 任务：起飞 -> 两个航点 -> NAV_LAND
  reverse  LOITER 手动打杆：加速到 5 m/s -> 突然反向，重复三次
  circle   CIRCLE 模式 2 m 半径绕圈：1.0/1.5/2.0/2.5 m/s 逐档，每档 2 圈
  loiter-circle  LOITER 杆量画圈：同上四档，但无 AC_Circle 自限速，可打到饱和
  fence    圆形围栏接近：逐档撞边界，量实际余量与冲出量
  route    AUTO 作业航线：直线段 + 180° 掉头 + 密集航点，量速度波动与回线误差

默认启用自定义机型中的近地增升与速度相关气动力矩。加 --baseline 可把这
两项归零，用同一场景做 A/B，确认现象来自物理项而不是脚本本身。
"""

import argparse
import datetime
import json
import math
import os
import re
import shutil
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
AP_ROOT = os.path.normpath(os.path.join(HERE, os.pardir, os.pardir))
PYMAVLINK = os.path.join(AP_ROOT, "modules", "mavlink")
if PYMAVLINK not in sys.path:
    sys.path.insert(0, PYMAVLINK)

from pymavlink import mavutil  # noqa: E402


SITL_BIN = os.path.join(AP_ROOT, "build", "sitl", "bin", "arducopter")
DEFAULTS = os.path.join(AP_ROOT, "Tools", "autotest", "default_params", "copter.parm")
PARAMS = os.path.join(HERE, "eft_hexa.parm")
MODEL = os.path.join(HERE, "eft_hexa.json")
HOME = (35.363261, 149.165230, 584.0, 0.0)
ON_GROUND = mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND
IN_AIR = mavutil.mavlink.MAV_LANDED_STATE_IN_AIR


def wrap_pi(v):
    return (v + math.pi) % (2.0 * math.pi) - math.pi


def quat_to_euler(q):
    """MAVLink quaternion [w,x,y,z] -> roll, pitch, yaw (rad)."""
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


class Monitor:
    def __init__(self, mav, process):
        self.mav = mav
        self.process = process
        self.sim_ms = 0
        self.att = None
        self.target = None
        self.local = None
        self.global_pos = None
        self.landed_state = None
        self.mission_seq = None
        self.ekf_flags = 0
        self.armed = False
        self.touch_ms = None
        self.touch_speed = None
        self.disarm_ms = None
        self.max_motor_spread_after_touch = 0
        self.messages = []

    def update(self, msg):
        if msg is None:
            return
        if hasattr(msg, "time_boot_ms"):
            self.sim_ms = max(self.sim_ms, int(msg.time_boot_ms))
        typ = msg.get_type()
        if typ == "ATTITUDE":
            self.att = msg
        elif typ == "ATTITUDE_TARGET":
            self.target = msg
        elif typ == "LOCAL_POSITION_NED":
            self.local = msg
        elif typ == "GLOBAL_POSITION_INT":
            self.global_pos = msg
        elif typ == "MISSION_CURRENT":
            self.mission_seq = int(msg.seq)
        elif typ == "EXTENDED_SYS_STATE":
            self.landed_state = msg.landed_state
        elif typ == "EKF_STATUS_REPORT":
            self.ekf_flags = int(msg.flags)
        elif typ == "HEARTBEAT":
            armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            if self.armed and not armed and self.disarm_ms is None:
                self.disarm_ms = self.sim_ms
            self.armed = armed
        elif typ == "STATUSTEXT":
            text = msg.text.rstrip("\x00")
            self.messages.append((self.sim_ms, text))
            if "Disarming motors" in text and self.disarm_ms is None:
                self.disarm_ms = self.sim_ms
            if "PreArm" in text or "Arm" in text or "Failsafe" in text:
                print("  FC:", text)
            hit = re.search(r"SIM Hit ground at ([+-]?[0-9.]+) m/s", text)
            if hit and self.touch_ms is None:
                self.touch_ms = self.sim_ms
                self.touch_speed = float(hit.group(1))
                print("  触地: %.3f m/s" % self.touch_speed)
        elif typ == "SERVO_OUTPUT_RAW" and self.touch_ms is not None:
            if self.sim_ms - self.touch_ms <= 1500:
                pwm = [getattr(msg, "servo%d_raw" % i) for i in range(1, 7)]
                self.max_motor_spread_after_touch = max(
                    self.max_motor_spread_after_touch, max(pwm) - min(pwm))

    def recv(self, timeout=1.0):
        msg = self.mav.recv_match(blocking=True, timeout=timeout)
        if msg is None and self.process.poll() is not None:
            raise RuntimeError("SITL 已退出，见 sitl_stdout.log")
        self.update(msg)
        return msg

    def body_velocity(self):
        if self.local is None or self.att is None:
            return None
        c = math.cos(self.att.yaw)
        s = math.sin(self.att.yaw)
        return (c * self.local.vx + s * self.local.vy,
                -s * self.local.vx + c * self.local.vy,
                self.local.vz)

    def attitude_error_deg(self):
        if self.att is None or self.target is None:
            return None
        tr, tp, ty = quat_to_euler(self.target.q)
        return (math.degrees(wrap_pi(tr - self.att.roll)),
                math.degrees(wrap_pi(tp - self.att.pitch)),
                math.degrees(wrap_pi(ty - self.att.yaw)))


def set_message_interval(mav, msg_id, hz):
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        msg_id, 1e6 / hz, 0, 0, 0, 0, 0)


def prepare_run(case, baseline, output, overrides=None, variant_name=None):
    stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    variant = variant_name or ("baseline" if baseline else "coupled")
    out = os.path.abspath(output or os.path.join(HERE, "runs", "%s-%s-%s" % (stamp, case, variant)))
    os.makedirs(out, exist_ok=True)
    model = json.load(open(MODEL, encoding="utf-8"))
    if baseline:
        model["ground_effect_height"] = 0.0
        model["ground_effect_collapse_height"] = 0.0
        model["ground_effect_gain"] = 0.0
        model["ground_effect_vspeed_gain"] = 0.0
        model["velocity_torque_gain"] = [0.0, 0.0, 0.0]
    model_path = os.path.join(out, "model.json")
    with open(model_path, "w", encoding="utf-8") as f:
        json.dump(model, f, indent=2)
        f.write("\n")
    shutil.copyfile(PARAMS, os.path.join(out, "params.parm"))
    # 算法侧的参数覆盖单独成档，使「物理项变体」与「算法变体」在结果目录里一眼分得开
    algo_path = os.path.join(out, "algo.parm")
    with open(algo_path, "w", encoding="utf-8") as f:
        for k, v in sorted((overrides or {}).items()):
            f.write("%s %g\n" % (k, v))
    return out, model_path, variant, algo_path


def start_sitl(out, model_path, speedup, algo_path=None):
    runtime = os.path.join(out, "runtime.parm")
    with open(runtime, "w", encoding="utf-8") as f:
        f.write("SIM_SPEEDUP %g\n" % speedup)
    # 顺序即优先级：算法覆盖放最后，压过机型参数
    chain = [DEFAULTS, PARAMS, runtime]
    if algo_path and os.path.getsize(algo_path) > 0:
        chain.append(algo_path)
    defaults = ",".join(chain)
    home = ",".join(str(x) for x in HOME)
    # AP_Filesystem's JSON loader is reliable with a path relative to the
    # process cwd; some SITL builds reject an otherwise valid absolute /tmp path.
    cmd = [SITL_BIN, "--model", "hexa-dji:" + os.path.basename(model_path),
           "--speedup", str(speedup), "--home", home,
           "--defaults", defaults, "--wipe"]
    stdout = open(os.path.join(out, "sitl_stdout.log"), "w", encoding="utf-8")
    print("SITL:", " ".join(cmd))
    proc = subprocess.Popen(cmd, cwd=out, stdout=stdout, stderr=subprocess.STDOUT)
    return proc, stdout


def connect(process):
    mav = mavutil.mavlink_connection("tcp:127.0.0.1:5760")
    mav.wait_heartbeat(timeout=30)
    mon = Monitor(mav, process)
    for msg_id, hz in ((30, 50), (32, 25), (33, 10), (36, 25),
                       (83, 50), (193, 5), (245, 5)):
        set_message_interval(mav, msg_id, hz)
    mav.mav.request_data_stream_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 10, 1)
    deadline = time.monotonic() + 45
    while time.monotonic() < deadline:
        mon.recv()
        # Attitude + horizontal/vertical velocity + absolute XY/Z position.
        ready = 1 | 2 | 4 | 16 | 32
        if (mon.global_pos is not None and mon.global_pos.lat != 0 and
                mon.ekf_flags & ready == ready):
            return mon
    raise RuntimeError("45 秒内未获得有效 GPS/EKF 位置")


def upload_mission(mon, items):
    mav = mon.mav
    mav.mav.mission_clear_all_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
    mav.mav.mission_count_send(
        mav.target_system, mav.target_component, len(items),
        mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
    sent = set()
    deadline = time.monotonic() + 20
    while time.monotonic() < deadline:
        msg = mon.recv()
        if msg is None:
            continue
        typ = msg.get_type()
        if typ in ("MISSION_REQUEST", "MISSION_REQUEST_INT"):
            seq = msg.seq
            command, lat, lon, alt = items[seq]
            mon.mav.mav.mission_item_int_send(
                mav.target_system, mav.target_component, seq,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                command, 0, 1, 0, 0, 0, float("nan"),
                int(round(lat * 1e7)), int(round(lon * 1e7)), alt,
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
            sent.add(seq)
        elif typ == "MISSION_ACK":
            # The first ACK can belong to MISSION_CLEAR_ALL.  The upload ACK
            # is only meaningful after at least one MISSION_REQUEST.
            if not sent:
                continue
            if msg.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
                raise RuntimeError("任务上传失败，MISSION_ACK=%d" % msg.type)
            if len(sent) != len(items):
                raise RuntimeError("任务只上传了 %d/%d 项" % (len(sent), len(items)))
            return
    raise RuntimeError("任务上传超时")


def command_takeoff(mon, altitude):
    mav = mon.mav
    set_mode_wait(mon, "GUIDED")
    prepare_for_arm(mon)
    mav.arducopter_arm()
    wait_armed(mon)
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
        0, 0, 0, 0, 0, 0, altitude)
    deadline = time.monotonic() + 45
    while time.monotonic() < deadline:
        mon.recv()
        if mon.global_pos is not None and mon.global_pos.relative_alt >= (altitude - 0.7) * 1000:
            return
    raise RuntimeError("起飞到 %.1f m 超时" % altitude)


def wait_disarmed(mon, wall_timeout=90):
    deadline = time.monotonic() + wall_timeout
    while time.monotonic() < deadline:
        mon.recv()
        if mon.disarm_ms is not None:
            return
    raise RuntimeError("等待落地上锁超时")


def prepare_for_arm(mon):
    # Match a real transmitter/GCS arming sequence: throttle must be low before
    # the command.  The AUTO/LOITER controllers take over after arming.
    for _ in range(20):
        mon.mav.mav.rc_channels_override_send(
            mon.mav.target_system, mon.mav.target_component,
            1500, 1500, 1000, 1500, 65535, 65535, 65535, 65535)
        mon.recv()


def set_mode_wait(mon, name, wall_timeout=10):
    mapping = mon.mav.mode_mapping()
    if name not in mapping:
        raise RuntimeError("固件没有模式 %s" % name)
    wanted = mapping[name]
    mon.mav.set_mode(wanted)
    deadline = time.monotonic() + wall_timeout
    while time.monotonic() < deadline:
        msg = mon.recv()
        if msg is not None and msg.get_type() == "HEARTBEAT" and msg.custom_mode == wanted:
            return
    raise RuntimeError("切换到 %s 超时" % name)


def wait_armed(mon, wall_timeout=20):
    deadline = time.monotonic() + wall_timeout
    while time.monotonic() < deadline:
        mon.recv()
        if mon.armed:
            return
    raise RuntimeError("等待解锁超时")


def run_landing(mon):
    lat, lon = HOME[0], HOME[1]
    dn = 25.0 / 111319.5
    de = 25.0 / (111319.5 * math.cos(math.radians(lat)))
    mission = [
        # ArduPilot stores seq=0 as the non-executed home item.
        (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, lat, lon, 0.0),
        (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, lat, lon, 8.0),
        (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, lat + dn, lon, 8.0),
        (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, lat + dn, lon + de, 8.0),
        (mavutil.mavlink.MAV_CMD_NAV_LAND, lat, lon, 0.0),
    ]
    upload_mission(mon, mission)
    print("AUTO 任务已上传：起飞 -> 航点 1 -> 航点 2 -> LAND")
    # AUTO_OPTIONS=1 makes the uploaded mission behave like the GCS "start
    # mission" action: enter AUTO, arm, then execute the takeoff item.
    set_mode_wait(mon, "AUTO")
    prepare_for_arm(mon)
    mon.mav.arducopter_arm()
    wait_armed(mon)
    wait_disarmed(mon, 120)
    delay = None
    if mon.touch_ms is not None and mon.disarm_ms is not None:
        delay = (mon.disarm_ms - mon.touch_ms) / 1000.0
    return {
        "touch_speed_m_s_down": mon.touch_speed,
        "touch_to_disarm_s": delay,
        "max_first6_motor_spread_after_touch_pwm": mon.max_motor_spread_after_touch,
        "touch_sim_ms": mon.touch_ms,
        "disarm_sim_ms": mon.disarm_ms,
    }


def rc_override(mon, roll=1500, pitch=1500, throttle=1500, yaw=1500):
    mon.mav.mav.rc_channels_override_send(
        mon.mav.target_system, mon.mav.target_component,
        roll, pitch, throttle, yaw, 65535, 65535, 65535, 65535)


def hold_stick_until(mon, pitch_pwm, predicate, max_sim_s, samples=None):
    start = mon.sim_ms
    while mon.sim_ms - start < max_sim_s * 1000:
        rc_override(mon, pitch=pitch_pwm)
        mon.recv(timeout=1)
        if samples is not None:
            err = mon.attitude_error_deg()
            vel = mon.body_velocity()
            if err is not None and vel is not None:
                samples.append((mon.sim_ms, vel[0], err[0], err[1],
                                math.degrees(mon.att.rollspeed),
                                math.degrees(mon.att.pitchspeed)))
        if predicate():
            return
    raise RuntimeError("打杆阶段 %.1f s 仿真时间内未达到目标" % max_sim_s)


def run_reverse(mon):
    command_takeoff(mon, 10.0)
    set_mode_wait(mon, "LOITER")
    for _ in range(50):
        rc_override(mon)
        mon.recv()
    print("LOITER 已稳定，开始 5 m/s 前进/急反向")

    samples = []
    events = []
    stick = 1000
    hold_stick_until(
        mon, stick,
        lambda: mon.body_velocity() is not None and abs(mon.body_velocity()[0]) >= 4.8,
        15, samples)

    # 到速后短暂维持，让 I 项形成稳定的速度相关配平，再突然反向。
    for index in range(3):
        sign = 1 if mon.body_velocity()[0] >= 0 else -1
        hold_start = mon.sim_ms
        hold_stick_until(mon, stick, lambda: mon.sim_ms - hold_start >= 1500, 3, samples)
        pre_speed = mon.body_velocity()[0]
        stick = 2000 if stick == 1000 else 1000
        event = {"index": index + 1, "sim_ms": mon.sim_ms,
                 "pre_speed_m_s": pre_speed, "pitch_pwm": stick}
        events.append(event)
        print("  反向 %d: t=%.2f s, 前速度=%+.2f m/s" %
              (index + 1, mon.sim_ms / 1000.0, pre_speed))
        hold_stick_until(
            mon, stick,
            lambda sign=sign: mon.body_velocity() is not None and
            mon.body_velocity()[0] * sign <= -4.8,
            15, samples)

    rc_override(mon)
    neutral_start = mon.sim_ms
    while mon.sim_ms - neutral_start < 1500:
        rc_override(mon)
        mon.recv()
    set_mode_wait(mon, "LAND")
    wait_disarmed(mon, 90)

    for event in events:
        window = [s for s in samples if event["sim_ms"] <= s[0] <= event["sim_ms"] + 6000]
        if not window:
            continue
        event["peak_roll_error_deg"] = max(abs(s[2]) for s in window)
        event["peak_pitch_error_deg"] = max(abs(s[3]) for s in window)
        event["peak_roll_rate_deg_s"] = max(abs(s[4]) for s in window)
        event["peak_pitch_rate_deg_s"] = max(abs(s[5]) for s in window)
        event["post_speed_m_s"] = window[-1][1]
    return {"reversals": events}


# P07：小半径绕圈。半径 2 m 是现场澄清的实际工况，它把问题从「轨迹平滑性」
# 变成「控制权限与带宽」——本机 ANGLE_MAX=15°，2 m 半径下 2.29 m/s 即顶满。
CIRCLE_RADIUS_M = 2.0
CIRCLE_SPEEDS_MS = (1.0, 1.5, 2.0, 2.5)
CIRCLE_LAPS = 2
# 满杆对应的倾角上限：ANGLE_MAX=1500 与 LOIT_ANG_MAX=15 取小者
LOITER_ANGLE_MAX_DEG = 15.0


def set_param(mon, name, value, timeout_ms=8000):
    """设参数并等飞控回读确认。CIRCLE_RATE 只在模式 init 时读取，所以调用方
    必须退出 CIRCLE 再重进，否则改了不生效。"""
    mon.mav.mav.param_set_send(mon.mav.target_system, mon.mav.target_component,
                               name.encode(), float(value),
                               mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    deadline = mon.sim_ms + timeout_ms
    while mon.sim_ms < deadline:
        msg = mon.recv()
        if msg is not None and msg.get_type() == "PARAM_VALUE" \
                and msg.param_id.rstrip("\x00") == name:
            return msg.param_value
    raise RuntimeError("参数 %s 设置超时" % name)


def target_tilt_deg(mon):
    """指令倾角(总倾角，不分轴)。顶在 ANGLE_MAX 上就是控制权限饱和的直接证据。"""
    if mon.target is None:
        return None
    tr, tp, _ = quat_to_euler(mon.target.q)
    return math.degrees(math.acos(
        constrain(math.cos(tr) * math.cos(tp), -1.0, 1.0)))


def constrain(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


def run_circle(mon):
    command_takeoff(mon, 15.0)
    set_mode_wait(mon, "LOITER")
    for _ in range(50):
        rc_override(mon)
        mon.recv()

    # 关掉杆量改半径/速率，否则脚本的中位杆量会干扰几何
    set_param(mon, "CIRCLE_OPTIONS", 0)
    set_param(mon, "CIRCLE_RADIUS", CIRCLE_RADIUS_M * 100.0)
    print("CIRCLE 半径 %.1f m，逐档加速" % CIRCLE_RADIUS_M)

    steps = []
    for speed in CIRCLE_SPEEDS_MS:
        rate_degs = math.degrees(speed / CIRCLE_RADIUS_M)
        set_param(mon, "CIRCLE_RATE", rate_degs)
        set_mode_wait(mon, "CIRCLE", 15)

        lap_s = 2.0 * math.pi * CIRCLE_RADIUS_M / speed
        hold_ms = int(CIRCLE_LAPS * lap_s * 1000)
        start = mon.sim_ms
        # 前 1/4 圈算进入段，不计入稳态统计
        settle_ms = int(lap_s * 250)
        samples = []
        while mon.sim_ms - start < hold_ms:
            rc_override(mon)
            mon.recv()
            err = mon.attitude_error_deg()
            tilt = target_tilt_deg(mon)
            vel = mon.body_velocity()
            if err is None or tilt is None or vel is None:
                continue
            samples.append((mon.sim_ms - start, tilt, err[0], err[1],
                            math.hypot(vel[0], vel[1])))

        steady = [s for s in samples if s[0] >= settle_ms] or samples
        tilts = [s[1] for s in steady]
        errs = sorted(math.hypot(s[2], s[3]) for s in steady)
        spds = [s[4] for s in steady]
        step = {
            "target_speed_m_s": speed,
            "circle_rate_deg_s": rate_degs,
            "required_tilt_deg": math.degrees(math.atan(
                speed * speed / (CIRCLE_RADIUS_M * 9.80665))),
            "laps": CIRCLE_LAPS,
            "samples": len(steady),
        }
        if tilts:
            step["target_tilt_max_deg"] = max(tilts)
            step["target_tilt_mean_deg"] = sum(tilts) / len(tilts)
            # 指令倾角贴住 ANGLE_MAX 的时间占比，即控制权限饱和程度
            step["tilt_saturated_frac"] = sum(t >= 14.7 for t in tilts) / len(tilts)
        if errs:
            step["att_err_mean_deg"] = sum(errs) / len(errs)
            step["att_err_p95_deg"] = errs[int(len(errs) * 0.95)]
            step["att_err_max_deg"] = errs[-1]
        if spds:
            step["actual_speed_mean_m_s"] = sum(spds) / len(spds)
        steps.append(step)
        print("  %.1f m/s (rate %.1f °/s): 需倾角 %.1f°，实测指令倾角 max %.1f°，"
              "饱和占比 %.0f%%，姿态误差均值 %.2f°"
              % (speed, rate_degs, step["required_tilt_deg"],
                 step.get("target_tilt_max_deg", float("nan")),
                 100 * step.get("tilt_saturated_frac", 0),
                 step.get("att_err_mean_deg", float("nan"))))

        set_mode_wait(mon, "LOITER", 15)
        settle = mon.sim_ms
        while mon.sim_ms - settle < 3000:
            rc_override(mon)
            mon.recv()

    set_mode_wait(mon, "LAND")
    wait_disarmed(mon, 120)
    return {"circle_radius_m": CIRCLE_RADIUS_M, "steps": steps}


def run_loiter_circle(mon):
    """LOITER 手动画圈：杆量按 A·cos(ωt) / A·sin(ωt) 旋转。

    与 CIRCLE 模式的区别是这里没有 AC_Circle 的自限速——杆量直接映射到倾角
    指令，所以能真的把指令顶到 ANGLE_MAX 上。协调圆周下 a = ω·v、R = v/ω，
    因此 R 固定时 ω = v/R、所需倾角 θ = atan(v²/(R·g))。
    """
    command_takeoff(mon, 15.0)
    set_mode_wait(mon, "LOITER")
    for _ in range(60):
        rc_override(mon)
        mon.recv()
    print("LOITER 已稳定，2 m 半径逐档画圈（杆量驱动）")

    steps = []
    for speed in CIRCLE_SPEEDS_MS:
        omega = speed / CIRCLE_RADIUS_M                      # rad/s
        need_deg = math.degrees(math.atan(
            speed * speed / (CIRCLE_RADIUS_M * 9.80665)))
        # 满杆对应 LOIT_ANG_MAX / ANGLE_MAX 的较小者，本机为 15°
        frac = min(need_deg / LOITER_ANGLE_MAX_DEG, 1.0)
        amp = int(round(frac * 500))
        saturated_cmd = need_deg > LOITER_ANGLE_MAX_DEG

        lap_s = 2.0 * math.pi / omega
        hold_ms = int(2 * lap_s * 1000)
        settle_ms = int(lap_s * 250)
        start = mon.sim_ms
        samples = []
        while mon.sim_ms - start < hold_ms:
            t = (mon.sim_ms - start) / 1000.0
            roll = 1500 + int(round(amp * math.cos(omega * t)))
            pitch = 1500 + int(round(amp * math.sin(omega * t)))
            rc_override(mon, roll=roll, pitch=pitch)
            mon.recv()
            err = mon.attitude_error_deg()
            tilt = target_tilt_deg(mon)
            vel = mon.body_velocity()
            if err is None or tilt is None or vel is None or mon.local is None:
                continue
            samples.append((mon.sim_ms - start, tilt, err[0], err[1],
                            math.hypot(vel[0], vel[1]),
                            mon.local.x, mon.local.y))

        steady = [s for s in samples if s[0] >= settle_ms] or samples
        tilts = [s[1] for s in steady]
        errs = sorted(math.hypot(s[2], s[3]) for s in steady)
        spds = [s[4] for s in steady]
        step = {
            "target_speed_m_s": speed,
            "omega_deg_s": math.degrees(omega),
            "required_tilt_deg": need_deg,
            "stick_frac": frac,
            "command_saturated_by_design": saturated_cmd,
            "samples": len(steady),
        }
        if tilts:
            step["target_tilt_max_deg"] = max(tilts)
            step["target_tilt_mean_deg"] = sum(tilts) / len(tilts)
            step["tilt_saturated_frac"] = sum(
                t >= LOITER_ANGLE_MAX_DEG - 0.3 for t in tilts) / len(tilts)
        if errs:
            step["att_err_mean_deg"] = sum(errs) / len(errs)
            step["att_err_p95_deg"] = errs[int(len(errs) * 0.95)]
            step["att_err_max_deg"] = errs[-1]
        if spds:
            step["actual_speed_mean_m_s"] = sum(spds) / len(spds)
        if len(steady) > 10:
            # 实际半径：稳态段位置的均值当圆心，到圆心距离的均值当半径。
            # 指令顶到限幅时半径会被撑大，这是饱和最直观的外部表现。
            cx = sum(s[5] for s in steady) / len(steady)
            cy = sum(s[6] for s in steady) / len(steady)
            radii = [math.hypot(s[5] - cx, s[6] - cy) for s in steady]
            step["actual_radius_mean_m"] = sum(radii) / len(radii)
        steps.append(step)
        print("  %.1f m/s: 需倾角 %.1f°%s，实测指令倾角 max %.1f°，饱和占比 %.0f%%，"
              "实测半径 %.2f m，姿态误差均值 %.2f°"
              % (speed, need_deg, "（满杆也给不出）" if saturated_cmd else "",
                 step.get("target_tilt_max_deg", float("nan")),
                 100 * step.get("tilt_saturated_frac", 0),
                 step.get("actual_radius_mean_m", float("nan")),
                 step.get("att_err_mean_deg", float("nan"))))

        rc_override(mon)
        settle = mon.sim_ms
        while mon.sim_ms - settle < 4000:
            rc_override(mon)
            mon.recv()

    set_mode_wait(mon, "LAND")
    wait_disarmed(mon, 120)
    return {"circle_radius_m": CIRCLE_RADIUS_M, "mode": "LOITER", "steps": steps}


# P05：圆形围栏接近。围栏圆心即 Home，从圆心向外加速接近边界，
# 看避障实际把飞机停在离边界多远——与 AVOID_MARGIN 的差值就是冲出量。
FENCE_RADIUS_M = 60.0
FENCE_APPROACH_SPEEDS_MS = (2.0, 5.0, 8.0, 12.0)


def horiz_radius(mon):
    if mon.local is None:
        return None
    return math.hypot(mon.local.x, mon.local.y)


def run_fence(mon):
    command_takeoff(mon, 15.0)
    set_mode_wait(mon, "LOITER")
    for _ in range(60):
        rc_override(mon)
        mon.recv()

    set_param(mon, "FENCE_TYPE", 2)          # 只留圆形围栏，隔离变量
    set_param(mon, "FENCE_RADIUS", FENCE_RADIUS_M)
    set_param(mon, "FENCE_ACTION", 0)        # 只报告，不触发 RTL，才能观察避障本身
    set_param(mon, "FENCE_ENABLE", 1)
    # 圆形围栏避障用的是 FENCE_MARGIN 而不是 AVOID_MARGIN
    # （AC_Avoid::adjust_velocity_circle_fence 取 _fence.get_margin()），
    # AVOID_MARGIN 管的是 proximity 传感器那条路。这里沿用实机的 FENCE_MARGIN=5。
    margin = 5.0
    set_param(mon, "FENCE_MARGIN", margin)
    set_param(mon, "AVOID_MARGIN", 10.0)
    set_param(mon, "AVOID_ENABLE", 7)        # 与实机 defaults.parm 一致
    set_param(mon, "AVOID_ACCEL_MAX", 4.0)   # 实机设的 4 m/s²，但 ANGLE_MAX=15° 只给得出 2.63
    print("圆形围栏 %.0f m，FENCE_MARGIN %.0f m，逐档接近" % (FENCE_RADIUS_M, margin))

    steps = []
    for speed in FENCE_APPROACH_SPEEDS_MS:
        # LOIT_SPEED 在 AC_Loiter::init() 读取，必须退出再进 LOITER 才生效
        set_param(mon, "LOIT_SPEED", speed * 100.0)
        set_mode_wait(mon, "ALT_HOLD", 15)
        set_mode_wait(mon, "LOITER", 15)

        # 固定时长跑满：2 m/s 走完 60 m 需 30 s，留足余量。不要用「半径不再增长」
        # 当停止判据——起飞点 r≈0、速度≈0 时它会立刻误触发。
        start = mon.sim_ms
        r_max, v_at_max, samples = 0.0, 0.0, []
        while mon.sim_ms - start < 70000:
            rc_override(mon, pitch=2000)     # 满杆向前（机头朝北，即 +X 向外）
            mon.recv()
            r = horiz_radius(mon)
            vel = mon.body_velocity()
            if r is None or vel is None:
                continue
            sp = math.hypot(vel[0], vel[1])
            samples.append((mon.sim_ms - start, r, sp))
            if r > r_max:
                r_max, v_at_max = r, sp

        # 末段持续顶住围栏，看是否在边界上振荡
        osc = [r for t, r, _ in samples if t >= 55000]

        margin_min = FENCE_RADIUS_M - r_max
        step = {
            "target_speed_m_s": speed,
            "fence_radius_m": FENCE_RADIUS_M,
            "fence_margin_m": margin,
            "closest_radius_m": r_max,
            "margin_achieved_m": margin_min,
            "margin_overshoot_m": margin - margin_min,
            "breached": r_max > FENCE_RADIUS_M,
            "speed_at_closest_m_s": v_at_max,
        }
        if osc:
            step["hold_radius_mean_m"] = sum(osc) / len(osc)
            step["hold_radius_pp_m"] = max(osc) - min(osc)
            step["hold_margin_mean_m"] = FENCE_RADIUS_M - step["hold_radius_mean_m"]
        # 制动峰值：接近段速度的最大下降率
        decel = 0.0
        for a, b in zip(samples, samples[1:]):
            dt = (b[0] - a[0]) / 1000.0
            if dt > 0:
                decel = max(decel, (a[2] - b[2]) / dt)
        step["peak_decel_m_s2"] = decel
        steps.append(step)
        print("  %.1f m/s: 最近半径 %.2f m，实际余量 %.2f m（设定 %.0f，冲出 %.2f）%s，"
              "制动峰值 %.2f m/s²，停稳后半径峰峰 %.2f m"
              % (speed, r_max, margin_min, margin, step["margin_overshoot_m"],
                 "  ** 越界 **" if step["breached"] else "",
                 decel, step.get("hold_radius_pp_m", float("nan"))))

        # 回飞到圆心附近，为下一档留出加速距离
        back = mon.sim_ms
        while mon.sim_ms - back < 45000:
            rc_override(mon, pitch=1000)
            mon.recv()
            r = horiz_radius(mon)
            if r is not None and r < 12.0:
                break
        rc_override(mon)
        settle = mon.sim_ms
        while mon.sim_ms - settle < 4000:
            rc_override(mon)
            mon.recv()

    set_mode_wait(mon, "LAND")
    wait_disarmed(mon, 120)
    return {"fence_radius_m": FENCE_RADIUS_M, "steps": steps}


# P06：作业航线骨架。优先覆盖两类最影响亩用量的场景——180° 掉头与密集航点切换。
# 航线相对 Home（米，北/东）：
#   起飞 → (60,0) → (60,12) → (0,12)        直线段 + 180° 掉头
#   → (0,24) → 每 10 m 一个点到 (50,24)     密集航点切换
# WPNAV_RADIUS=10 m 而点距 10 m，正是「间距小于制动距离」的应力工况。
ROUTE_ALT_M = 15.0
# 掉头连接段长度可调：它决定飞机在两个 90° 转角之间有没有距离重新加速。
# 掉头掉速若随该长度变化，说明主因是几何而不是限幅参数。
ROUTE_TURN_OFFSET_M = 12.0


def route_waypoints(offset, turn_deg=90.0):
    """turn_deg 是航段之间的转向角。SCurve 在转角处把前后两段混合，
    混合中点的速度是两段贡献的矢量和，因此掉速应随 cos(turn_deg/2) 走。
    用不同转角跑一遍就能把「几何必然」与「限幅不足」区分开。"""
    o = offset
    if abs(turn_deg - 90.0) < 1e-6:
        return [(60, 0), (60, o), (0, o), (0, o + 12),
                (10, o + 12), (20, o + 12), (30, o + 12), (40, o + 12), (50, o + 12)]
    # 指定角度的转角：向北 60 m 后偏转 turn_deg 再走两段，
    # 让转角落在航线内部——否则量到的是终点减速而不是转角掉速。
    th = math.radians(turn_deg)
    dn, de = 60 * math.cos(th), 60 * math.sin(th)
    return [(60, 0), (60 + dn, de), (60 + 2 * dn, 2 * de)]
TURN_SEQS = (2, 3)          # 掉头涉及的航点序号（1-based mission seq 见下）
DENSE_FROM_SEQ = 5


def ne_to_latlon(home_lat, home_lon, north_m, east_m):
    dlat = north_m / 111320.0
    dlon = east_m / (111320.0 * math.cos(math.radians(home_lat)))
    return home_lat + dlat, home_lon + dlon


def run_route(mon, turn_offset=ROUTE_TURN_OFFSET_M, turn_deg=90.0):
    global ROUTE_WPS
    ROUTE_WPS = route_waypoints(turn_offset, turn_deg)
    home_lat, home_lon = HOME[0], HOME[1]
    # seq=0 是 ArduPilot 不执行的 home 项，必须占位
    items = [(mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, home_lat, home_lon, 0.0),
             (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, home_lat, home_lon, ROUTE_ALT_M)]
    for n, e in ROUTE_WPS:
        lat, lon = ne_to_latlon(home_lat, home_lon, n, e)
        items.append((mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, lat, lon, ROUTE_ALT_M))
    items.append((mavutil.mavlink.MAV_CMD_NAV_LAND, home_lat, home_lon, 0.0))
    upload_mission(mon, items)

    # 顺序与 run_landing 一致：先进 AUTO，再解锁，靠 AUTO_OPTIONS=1 启动任务
    set_mode_wait(mon, "AUTO")
    prepare_for_arm(mon)
    mon.mav.arducopter_arm()
    wait_armed(mon)
    print("AUTO 航线已启动：%d 个航点" % len(ROUTE_WPS))

    samples = []
    start = mon.sim_ms
    while mon.sim_ms - start < 300000:
        mon.recv()
        if mon.local is None:
            continue
        sp = math.hypot(mon.local.vx, mon.local.vy)
        samples.append((mon.sim_ms, mon.mission_seq, mon.local.x, mon.local.y, sp))
        if mon.landed_state == ON_GROUND and mon.sim_ms - start > 30000:
            break
        if not mon.armed and mon.sim_ms - start > 30000:
            break

    wait_disarmed(mon, 60)
    return {"waypoints_ne_m": ROUTE_WPS,
            "turn_offset_m": turn_offset,
            "turn_deg": turn_deg,
            "segments": summarise_route(samples)}


def summarise_route(samples):
    """按 MISSION_CURRENT 切段，逐段统计速度与回线误差。

    作业段匀速是植保的核心诉求——喷洒量与速度直接相关，掉速就是重喷。
    所以主指标是速度波动（标准差/均值）与掉头处的最低速度，
    回线误差用点到航段直线的垂距。
    """
    by_seq = {}
    for t, seq, x, y, sp in samples:
        if seq is None or sp is None:
            continue
        by_seq.setdefault(seq, []).append((t, x, y, sp))

    out = []
    for seq in sorted(by_seq):
        seg = by_seq[seq]
        if len(seg) < 5:
            continue
        sps = [s[3] for s in seg]
        mean = sum(sps) / len(sps)
        var = sum((v - mean) ** 2 for v in sps) / len(sps)
        row = {
            "mission_seq": seq,
            "duration_s": (seg[-1][0] - seg[0][0]) / 1000.0,
            "speed_mean_m_s": mean,
            "speed_std_m_s": math.sqrt(var),
            "speed_min_m_s": min(sps),
            "speed_max_m_s": max(sps),
            "speed_cv": math.sqrt(var) / mean if mean > 0.05 else None,
            "samples": len(seg),
        }
        # 回线误差：点到该段起止连线的垂距
        x0, y0 = seg[0][1], seg[0][2]
        x1, y1 = seg[-1][1], seg[-1][2]
        dx, dy = x1 - x0, y1 - y0
        L = math.hypot(dx, dy)
        if L > 1.0:
            devs = [abs((s[1] - x0) * dy - (s[2] - y0) * dx) / L for s in seg]
            row["crosstrack_max_m"] = max(devs)
            row["crosstrack_rms_m"] = math.sqrt(sum(d * d for d in devs) / len(devs))
        out.append(row)
    return out


def summarise_log(path):
    """从日志算 P02 的验收量。

    峰值姿态误差由转动瞬态主导，测不出这个问题——真机现象是「卡住 1.4 s 而角速率≈0」，
    即持续误差。所以这里给的是 I 项摆动幅度与姿态误差的均值/P95，
    前者衡量「速率环还要自己配平多少」，后者衡量「卡住得有多久多深」。
    """
    if not path or not os.path.exists(path):
        return {}
    from pymavlink import DFReader
    axes = {"roll": ("PIDR", "DesRoll", "Roll"), "pitch": ("PIDP", "DesPitch", "Pitch")}
    i_terms = {k: [] for k in axes}
    errors = {k: [] for k in axes}
    m = DFReader.DFReader_binary(path)
    while True:
        msg = m.recv_match(type=["PIDR", "PIDP", "ATT"])
        if msg is None:
            break
        mt = msg.get_type()
        if mt == "ATT":
            for axis, (_, des, act) in axes.items():
                errors[axis].append(abs(getattr(msg, des) - getattr(msg, act)))
        else:
            for axis, (pid, _, _) in axes.items():
                if mt == pid:
                    i_terms[axis].append(msg.I)
    out = {}
    for axis in axes:
        vals, errs = i_terms[axis], sorted(errors[axis])
        if vals:
            out["%s_i_span" % axis] = max(vals) - min(vals)
            out["%s_i_rms" % axis] = math.sqrt(sum(v * v for v in vals) / len(vals))
        if errs:
            out["%s_att_err_mean_deg" % axis] = sum(errs) / len(errs)
            out["%s_att_err_p95_deg" % axis] = errs[int(len(errs) * 0.95)]
            out["%s_att_err_max_deg" % axis] = errs[-1]
    return out


def latest_log(out):
    candidates = []
    logdir = os.path.join(out, "logs")
    if os.path.isdir(logdir):
        for name in os.listdir(logdir):
            if name.upper().endswith((".BIN", ".EFT")):
                candidates.append(os.path.join(logdir, name))
    return max(candidates, key=os.path.getmtime) if candidates else None


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("case", choices=("landing", "reverse", "circle", "loiter-circle", "fence", "route"))
    parser.add_argument("--baseline", action="store_true",
                        help="关闭新增物理项，跑默认物理模型对照")
    parser.add_argument("--speedup", type=float, default=2.0)
    parser.add_argument("--output", help="结果目录（默认写入本目录 runs/）")
    parser.add_argument("--set", dest="overrides", action="append", default=[],
                        metavar="PARAM=VALUE",
                        help="覆盖飞控参数，可重复。用于算法 A/B（物理项保持不变，只切算法）")
    parser.add_argument("--variant", help="结果里记录的变体名，例如 baseline-algo / candidate-algo")
    parser.add_argument("--turn-offset", type=float, default=ROUTE_TURN_OFFSET_M,
                        help="route 场景的掉头连接段长度（米），用于分辨掉速是几何还是限幅所致")
    parser.add_argument("--turn-deg", type=float, default=90.0,
                        help="route 场景的转角度数；掉速若随 cos(角/2) 走即为 SCurve 混合的几何必然")
    args = parser.parse_args(argv)

    overrides = {}
    for item in args.overrides:
        if "=" not in item:
            raise SystemExit("--set 需要 PARAM=VALUE 形式，收到 %r" % item)
        k, v = item.split("=", 1)
        try:
            overrides[k.strip().upper()] = float(v)
        except ValueError:
            raise SystemExit("--set 的值必须是数字，收到 %r" % item)

    if not os.path.exists(SITL_BIN):
        raise SystemExit("缺少 %s；先在仓库根目录执行 ./waf configure --board sitl && ./waf copter" % SITL_BIN)
    out, model_path, variant, algo_path = prepare_run(
        args.case, args.baseline, args.output, overrides, args.variant)
    proc, stdout = start_sitl(out, model_path, args.speedup, algo_path)
    result = {"case": args.case, "variant": variant, "frame": "hexa-dji",
              "motor_count": 6, "output": out,
              "physics": "nominal" if args.baseline else "problem",
              "param_overrides": overrides}
    try:
        mon = connect(proc)
        if args.case == "landing":
            result.update(run_landing(mon))
        elif args.case == "circle":
            result.update(run_circle(mon))
        elif args.case == "loiter-circle":
            result.update(run_loiter_circle(mon))
        elif args.case == "fence":
            result.update(run_fence(mon))
        elif args.case == "route":
            result.update(run_route(mon, args.turn_offset, args.turn_deg))
        else:
            result.update(run_reverse(mon))
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()
        stdout.close()

    result["dataflash_log"] = latest_log(out)
    result["metrics"] = summarise_log(result["dataflash_log"])
    result["statustext"] = mon.messages
    result_path = os.path.join(out, "result.json")
    with open(result_path, "w", encoding="utf-8") as f:
        json.dump(result, f, ensure_ascii=False, indent=2)
        f.write("\n")
    print(json.dumps(result, ensure_ascii=False, indent=2))
    print("结果:", result_path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
