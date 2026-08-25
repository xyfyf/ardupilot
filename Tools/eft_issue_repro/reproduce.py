#!/usr/bin/env python3
"""EFT 植保六旋翼两个飞行问题的一键 SITL 闭环复现。

场景:
  landing  地面站式 AUTO 任务：起飞 -> 两个航点 -> NAV_LAND
  reverse  LOITER 手动打杆：加速到 5 m/s -> 突然反向，重复三次

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


def prepare_run(case, baseline, output):
    stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    variant = "baseline" if baseline else "coupled"
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
    return out, model_path, variant


def start_sitl(out, model_path, speedup):
    runtime = os.path.join(out, "runtime.parm")
    with open(runtime, "w", encoding="utf-8") as f:
        f.write("SIM_SPEEDUP %g\n" % speedup)
    defaults = ",".join((DEFAULTS, PARAMS, runtime))
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
    parser.add_argument("case", choices=("landing", "reverse"))
    parser.add_argument("--baseline", action="store_true",
                        help="关闭新增物理项，跑默认物理模型对照")
    parser.add_argument("--speedup", type=float, default=2.0)
    parser.add_argument("--output", help="结果目录（默认写入本目录 runs/）")
    args = parser.parse_args(argv)

    if not os.path.exists(SITL_BIN):
        raise SystemExit("缺少 %s；先在仓库根目录执行 ./waf configure --board sitl && ./waf copter" % SITL_BIN)
    out, model_path, variant = prepare_run(args.case, args.baseline, args.output)
    proc, stdout = start_sitl(out, model_path, args.speedup)
    result = {"case": args.case, "variant": variant, "frame": "hexa-dji",
              "motor_count": 6, "output": out}
    try:
        mon = connect(proc)
        if args.case == "landing":
            result.update(run_landing(mon))
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
