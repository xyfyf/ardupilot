#!/usr/bin/env python3
"""EFT 植保六旋翼两个飞行问题的一键 SITL 闭环复现。

场景:
  landing  地面站式 AUTO 任务：起飞 -> 两个航点 -> NAV_LAND
  reverse  LOITER 手动打杆：加速到 5 m/s -> 突然反向，重复三次
  circle   CIRCLE 模式 2 m 半径绕圈：1.0/1.5/2.0/2.5 m/s 逐档，每档 2 圈
  loiter-circle  LOITER 杆量画圈：同上四档，但无 AC_Circle 自限速，可打到饱和
  fence    圆形围栏接近：逐档撞边界，量实际余量与冲出量
  route    AUTO 作业航线：直线段 + 180° 掉头 + 密集航点，量速度波动与回线误差
  uturn    田间 U 型转弯：两条作业段 + 外场 U 转，量转弯耗时与进入下条线的建立距离
  uturn-guided  同上，但绕开 SCurve，用 GUIDED 三阶设定点喂解析匀速圆弧

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


def prepare_run(case, baseline, output, overrides=None, variant_name=None, model_overrides=None):
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
    # 物理模型参数的覆盖，与算法参数覆盖分开：前者改的是「被控对象」，
    # 后者改的是「控制器」，混在一起会让 A/B 结果无法归因。
    for k, v in (model_overrides or {}).items():
        model[k] = v
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


MISSION_TYPE_MISSION = 0     # MAV_MISSION_TYPE_MISSION


def upload_fence(mon, radius_m, lat, lon, sides=0):
    """上传 polyfence 包含区。sides=0 给包含圆，>=3 给外接半径 radius_m 的正多边形。

    两种形状不能混用，因为路径规划器对它们的支持不同：
      AP_OADijkstra    只认包含**多边形**（_inclusion_polygon_pts），不认包含圆
      AP_OABendyRuler  两者都认
      AC_Avoid         两者都认（LOITER/GUIDED 用的就是它）
    而三者**都不认** FENCE_RADIUS 那个参数式圆形围栏。
    """
    items = []
    if sides >= 3:
        for k in range(sides):
            th = 2.0 * math.pi * k / sides
            la, lo = ne_to_latlon(lat, lon,
                                  radius_m * math.cos(th), radius_m * math.sin(th))
            items.append((mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
                          float(sides), la, lo))
        label = "包含多边形 %d 边 外接半径 %.1f m" % (sides, radius_m)
    else:
        items.append((mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION,
                      float(radius_m), lat, lon))
        label = "包含圆 半径 %.1f m" % radius_m
    _upload_fence_items(mon, items, label)


def _upload_fence_items(mon, items, label):
    """上传一个 polyfence 包含圆（MAV_MISSION_TYPE_FENCE 通道）。

    为什么不能用 FENCE_RADIUS 那个参数式圆形围栏：三个路径规划器
    （AP_OADijkstra / AP_OABendyRuler / AC_Avoid 的多边形分支）读的都是
    polyfence 存储，**不认参数式圆形围栏**。也就是说想让 AUTO 下的围控
    真正生效，围栏必须走这条上传通道。
    """
    mav = mon.mav
    mav.mav.mission_clear_all_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
    mav.mav.mission_count_send(
        mav.target_system, mav.target_component, len(items),
        mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
    deadline = time.monotonic() + 20
    acked = False
    sent = set()
    while time.monotonic() < deadline and not acked:
        msg = mon.recv()
        if msg is None:
            continue
        typ = msg.get_type()
        if typ in ("MISSION_REQUEST", "MISSION_REQUEST_INT"):
            if getattr(msg, "mission_type", 0) != mavutil.mavlink.MAV_MISSION_TYPE_FENCE:
                continue
            sent.add(msg.seq)
            cmd, p1, ilat, ilon = items[msg.seq]
            mav.mav.mission_item_int_send(
                mav.target_system, mav.target_component, msg.seq,
                mavutil.mavlink.MAV_FRAME_GLOBAL,
                cmd, 0, 1,
                float(p1), 0.0, 0.0, 0.0,
                int(ilat * 1e7), int(ilon * 1e7), 0.0,
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
        elif typ == "MISSION_ACK":
            if getattr(msg, "mission_type", 0) != mavutil.mavlink.MAV_MISSION_TYPE_FENCE:
                continue
            # mission_clear_all 自己也会回一条 ACK。在发出任何一项之前收到的
            # ACK 是那一条，不是上传结果——照单全收就会「上传成功」却什么都没写，
            # 飞控侧随后打印 Fence upload timeout，而脚本这边毫无察觉。
            if not sent:
                continue
            if msg.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
                raise RuntimeError("围栏上传被拒: %s" % msg.type)
            acked = True
    if not acked:
        raise RuntimeError("围栏上传超时")
    print("  已上传 polyfence %s（飞控已确认 %d 项）" % (label, len(sent)))


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
        # 必须按 mission_type 过滤。任务、围栏、集结点共用同一套 MISSION_*
        # 消息，只靠 mission_type 区分；不过滤的话，前一次围栏上传留下的那条
        # FENCE 型 MISSION_ACK 会被当成本次任务的 ACK 收下，于是刚发出第一项
        # 就判定「上传完成」，报「只上传了 1/7 项」。
        if getattr(msg, "mission_type", MISSION_TYPE_MISSION) != MISSION_TYPE_MISSION:
            continue
        if typ in ("MISSION_REQUEST", "MISSION_REQUEST_INT"):
            seq = msg.seq
            item = items[seq]
            command, lat, lon, alt = item[:4]
            if len(item) >= 8:
                # 显式给 param1..4，用于 LOITER_TURNS 这类靠 param 表达语义的命令
                p1, p2, p3, p4 = item[4:8]
            else:
                p1 = p2 = p3 = 0.0
                # NAV_WAYPOINT 的 param4 是航向，NaN 表示沿用默认；但样条航点
                # 不接受 NaN（AP 会回 MISSION_ACK=9 INVALID_PARAM4），必须给 0。
                p4 = (0.0 if command == mavutil.mavlink.MAV_CMD_NAV_SPLINE_WAYPOINT
                      else float("nan"))
            mon.mav.mav.mission_item_int_send(
                mav.target_system, mav.target_component, seq,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                command, 0, 1, p1, p2, p3, p4,
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


def run_landing(mon, release_alt_m=None, release_speed_cms=200):
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

    if release_alt_m is None:
        wait_disarmed(mon, 120)
    else:
        # 复现 LNDS 缓降脚本的近地误判。
        #
        # 该脚本以「相对 Home 的 EKF 高度」判定近地：低于 LNDS_ALT_M 时把
        # LAND_SPEED 等压到 LNDS_SLOW_MS，低于 LNDS_REL_M(0.15 m) 则锁存并
        # **恢复基准速度**。问题在于 EKF 高度会漂——实机日志 220 中触地时
        # 读数为 -0.74 ~ -1.13 m，也就是说飞机穿过 0.15 m 这条线时，距真实
        # 地面还有约 1 m、还有约 2 秒才触地。限速于是在最后一米被提前放开。
        #
        # 这里直接按**真实离地高度**触发放开，等价地复现那一米的失速保护缺口。
        # 用高度自身判断是否已经起飞，而不是 landed_state：后者依赖
        # EXTENDED_SYS_STATE 的推送，实测在本场景下不足以作为触发门闩，
        # 结果是整段注入从未执行而参数纹丝不动。
        released = False
        airborne = False
        t0 = mon.sim_ms
        while mon.sim_ms - t0 < 250000:
            mon.recv()
            if not mon.armed and mon.sim_ms - t0 > 20000:
                break
            if mon.local is None:
                continue
            height = -mon.local.z
            if height > 3.0:
                airborne = True
            if airborne and not released and height < release_alt_m:
                set_param(mon, "LAND_SPEED", release_speed_cms)
                released = True
                vz = mon.local.vz if hasattr(mon.local, "vz") else 0.0
                print("  === 离地 %.2f m 放开限速 -> LAND_SPEED=%d cm/s（此刻下降率 %.2f m/s）==="
                      % (height, release_speed_cms, vz))
        if not released:
            print("  警告：注入未触发（airborne=%s）" % airborne)
        if mon.armed:
            wait_disarmed(mon, 180)

    delay = None
    if mon.touch_ms is not None and mon.disarm_ms is not None:
        delay = (mon.disarm_ms - mon.touch_ms) / 1000.0
    return {
        "release_alt_m": release_alt_m,
        "release_speed_cms": release_speed_cms if release_alt_m is not None else None,
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
    set_mode_wait(mon, mode if mode in ("LOITER", "ALT_HOLD") else "LOITER")
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


# 命令行 --set 指定过的参数名。场景内部的 set_param 会跳过它们。
#
# 不这样做的话，场景里硬编码的 set_param 会**静默覆盖**命令行的值，而结果看上去
# 完全正常——只是测的不是你以为的那个配置。围栏场景上已栽过两次：
# --set FENCE_ACTION=4 被场景内的 set_param(...,0) 覆盖，得出「开了刹车动作仍
# 冲出 842 m」的错误结论；--set FENCE_MARGIN=2 同样被覆盖，两组本该不同的数据
# 逐档相同。两次的表象都是「数字合理」，没有任何报错。
CLI_OVERRIDDEN = set()


def set_param(mon, name, value, timeout_ms=8000):
    """设参数并等飞控回读确认。CIRCLE_RATE 只在模式 init 时读取，所以调用方
    必须退出 CIRCLE 再重进，否则改了不生效。

    命令行 --set 指定过的参数不再被场景内部覆盖，见 CLI_OVERRIDDEN。"""
    if name in CLI_OVERRIDDEN:
        print("  [跳过] %s 由命令行 --set 指定，场景内部不覆盖" % name)
        return
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


def run_fence(mon, mode="LOITER", skip_param_fence=False):
    """满杆冲向围栏，量各模式的实际围控能力。

    mode 决定被测的是哪条链路——这正是问题所在，不同模式的水平围控**机制不同**：
      LOITER / POSHOLD / ZIGZAG   走 loiter_nav->update()，AC_Loiter 默认带避障
      GUIDED                       avoid.adjust_velocity + 目标点校验
      AUTO / RTL / SMART_RTL       走 wp_nav，**需要 OA_TYPE**，默认关
      CIRCLE / BRAKE               直驱 pos_control，**没有**水平避障
      ALT_HOLD / STABILIZE / SPORT 无位置控制，**原理上无法**主动围控
    对最后两类，唯一的防线是 FENCE_ACTION——越界之后才动作。

    skip_param_fence=True 时不设 FENCE_RADIUS，改用外部上传的 polyfence，
    因为路径规划器只认后者。
    """
    command_takeoff(mon, 15.0)
    set_mode_wait(mon, "LOITER")
    for _ in range(60):
        rc_override(mon)
        mon.recv()

    if not skip_param_fence:
        set_param(mon, "FENCE_TYPE", 2)      # 只留圆形围栏，隔离变量
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
        # 先退到 ALT_HOLD 再进被测模式：LOIT_SPEED 只在 AC_Loiter::init() 读取，
        # 不退出重进不生效。被测模式必须是**最后**切的那个——早先版本把它切在
        # 这之后又跟了一句切回 LOITER，结果四个模式测的全是 LOITER，
        # 四档结果精确相同（55.07 m），差点当成「各模式都拦得住」的结论。
        set_mode_wait(mon, "ALT_HOLD", 15)
        set_mode_wait(mon, mode, 15)

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
    # 距 Home 的最远距离。围栏场景要用：AUTO 的水平航迹**不过避障**
    # （mode_auto.cpp 里只有 get_avoidance_adjusted_climbrate_ms，管爬升率），
    # 所以围栏在 AUTO 下只是越界后的触发器，冲出量必须实测而不能假定为零。
    max_radius_m = max((math.hypot(sx, sy) for _, _, sx, sy, _ in samples),
                       default=0.0)
    return {"waypoints_ne_m": ROUTE_WPS,
            "turn_offset_m": turn_offset,
            "turn_deg": turn_deg,
            "max_radius_m": max_radius_m,
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


# P06/P07 交汇：田间 U 型转弯。直线段是作业喷洒段，U 型转弯在外场、不喷洒。
# 因此掉速本身不产生重喷，真正的代价是：
#   一、U 转耗时 → 作业效率（亩/小时）
#   二、退出 U 转后进入下一条作业线的建立距离 → 每条线开头有多长是废的
SPRAY_LEG_M = 60.0          # 作业段长度
MAG_DECLINATION_DEG = 0.0    # 兜底值；正常情况从日志的 COMPASS_DEC 读取
SWATH_M = 5.0               # 行距（喷幅），决定 U 转半径 = SWATH/2
# 跑道式 U 转的顶点正好在 leg + R（R = 行距/2），这才是半圆。
# 额外往外推会把两端只隔一个行距的转弯拉成又长又尖的发夹弯，
# 顶点曲率反而远小于 R —— 实测顶点半径会掉到 0.7 m，2 m/s 需要 30° 倾角。
UTURN_CLEAR_M = 0.0


def uturn_waypoints(leg, swath, style, clear=UTURN_CLEAR_M):
    """两条平行作业段 + 外场 U 型转弯。返回 [(北, 东, 是否样条点)]。

    三种做法对应 ArduPilot 原生的三条路：
      square  两个普通航点的直角转弯 —— SCurve 按转角混合，掉速 v·cos(θ/2)
      arc     一串普通航点近似半圆 —— 点密了更圆，但每个点仍是一个转角
      spline  MAV_CMD_NAV_SPLINE_WAYPOINT —— 真正的平滑曲线，跑道式 U 转
    半径一律取行距/2，即下一条作业线的横向间距。
    """
    R = swath / 2.0
    cx = leg + clear                        # 圆心北向坐标，已在外场
    wps = [(leg, 0.0, False)]               # 作业段 1 终点，普通航点
    if style == "arc":
        for k in range(1, 7):
            th = math.pi * k / 7.0
            wps.append((cx + R * math.sin(th), R - R * math.cos(th), False))
    elif style == "spline":
        # 一个样条顶点即可，样条会依据前后点自动定切线
        wps.append((cx + R, R, True))
        wps.append((leg, swath, True))
    elif style == "spline-arc":
        # 半圆上取多点，全部设为样条航点 —— 样条才会真正贴着设计半径走，
        # 只给一个顶点时曲率由点位决定，实测会收紧到 0.6 m。
        for k in range(1, 7):
            th = math.pi * k / 7.0
            wps.append((cx + R * math.sin(th), R - R * math.cos(th), True))
        wps.append((leg, swath, True))
    else:
        wps.append((cx + R, 0.0, False))
        wps.append((cx + R, swath, False))
    if style not in ("spline", "spline-arc"):
        wps.append((leg, swath, False))     # 作业段 2 起点
    wps.append((0.0, swath, False))         # 作业段 2 终点
    return wps


def run_uturn(mon, swath=SWATH_M, style="square", leg=SPRAY_LEG_M):
    home_lat, home_lon = HOME[0], HOME[1]
    wps = uturn_waypoints(leg, swath, style)
    items = [(mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, home_lat, home_lon, 0.0),
             (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, home_lat, home_lon, ROUTE_ALT_M)]
    for n, e, is_spline in wps:
        lat, lon = ne_to_latlon(home_lat, home_lon, n, e)
        cmd = (mavutil.mavlink.MAV_CMD_NAV_SPLINE_WAYPOINT if is_spline
               else mavutil.mavlink.MAV_CMD_NAV_WAYPOINT)
        items.append((cmd, lat, lon, ROUTE_ALT_M))
    items.append((mavutil.mavlink.MAV_CMD_NAV_LAND, home_lat, home_lon, 0.0))
    upload_mission(mon, items)

    set_mode_wait(mon, "AUTO")
    prepare_for_arm(mon)
    mon.mav.arducopter_arm()
    wait_armed(mon)
    # 作业段 1 = seq2（飞向 wps[0]），作业段 2 = 最后一个航点段
    spray1_seq = 2
    spray2_seq = 2 + len(wps) - 1
    print("U 型转弯：行距 %.1f m（半径 %.1f m），%s 型，作业段 %.0f m"
          % (swath, swath / 2.0, style, leg))

    samples = []
    start = mon.sim_ms
    while mon.sim_ms - start < 400000:
        mon.recv()
        if mon.local is None or mon.att is None:
            continue
        sp = math.hypot(mon.local.vx, mon.local.vy)
        err = mon.attitude_error_deg()
        tilt = target_tilt_deg(mon)
        samples.append((mon.sim_ms, mon.mission_seq, mon.local.x, mon.local.y, sp,
                        math.hypot(err[0], err[1]) if err else None, tilt))
        if not mon.armed and mon.sim_ms - start > 30000:
            break
    wait_disarmed(mon, 60)

    by = {}
    for t, seq, x, y, sp, e, tilt in samples:
        if seq is None:
            continue
        by.setdefault(seq, []).append((t, x, y, sp, e, tilt))

    res = {"swath_m": swath, "uturn_radius_m": swath / 2.0, "style": style,
           "spray_leg_m": leg,
           "waypoints_ne_m": [(n, e) for n, e, _ in wps]}

    # U 转段 = 两个作业段之间的全部航段
    turn = [s for q in range(spray1_seq + 1, spray2_seq) for s in by.get(q, [])]
    if turn:
        # 实际飞出来的曲率半径：三点定圆 R = abc/(4·面积)。
        # 若它明显小于设计半径，说明轨迹比设计更弯，限制在几何而不在限幅。
        radii = []
        step = max(1, len(turn) // 200)
        for i in range(step, len(turn) - step, step):
            (x1, y1), (x2, y2), (x3, y3) = ((turn[i - step][1], turn[i - step][2]),
                                            (turn[i][1], turn[i][2]),
                                            (turn[i + step][1], turn[i + step][2]))
            a = math.hypot(x2 - x1, y2 - y1)
            b = math.hypot(x3 - x2, y3 - y2)
            c = math.hypot(x3 - x1, y3 - y1)
            area2 = abs((x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1))
            if area2 > 1e-6 and a > 0.05 and b > 0.05:
                radii.append(a * b * c / (2 * area2))
        radii = [r for r in radii if r < 100.0]
        sps = [s[3] for s in turn]
        tilts = [s[5] for s in turn if s[5] is not None]
        errs = [s[4] for s in turn if s[4] is not None]
        res["uturn"] = {
            "duration_s": (turn[-1][0] - turn[0][0]) / 1000.0,
            "speed_min_m_s": min(sps),
            "speed_mean_m_s": sum(sps) / len(sps),
            "tilt_max_deg": max(tilts) if tilts else None,
            "tilt_saturated_frac": (sum(t >= 14.7 for t in tilts) / len(tilts)) if tilts else None,
            "att_err_mean_deg": (sum(errs) / len(errs)) if errs else None,
            "att_err_max_deg": max(errs) if errs else None,
            "flown_radius_min_m": min(radii) if radii else None,
            "flown_radius_median_m": sorted(radii)[len(radii) // 2] if radii else None,
        }

    # 作业段 2：进入后多久速度与姿态稳定下来 —— 每条作业线开头有多少是废的。
    # 必须排除末端 15 m 的停靠减速段，否则「其后一直达标」的判据永远不成立。
    seg2 = by.get(spray2_seq, [])
    if len(seg2) > 20:
        ex, ey = seg2[-1][1], seg2[-1][2]
        usable = [s for s in seg2 if math.hypot(s[1] - ex, s[2] - ey) > 15.0]
        if len(usable) > 20:
            target = max(s[3] for s in usable)
            t0, x0, y0 = usable[0][0], usable[0][1], usable[0][2]
            settle_t = settle_d = None
            for i, s in enumerate(usable):
                # 达标后需连续保持 3 s 才算稳定，避免瞬时穿越被误判
                hold = [u for u in usable[i:] if u[0] - s[0] <= 3000]
                if len(hold) < 5:
                    break
                if all(abs(u[3] - target) <= 0.05 * target for u in hold) and \
                   all((u[4] is None or u[4] <= 1.0) for u in hold):
                    settle_t = (s[0] - t0) / 1000.0
                    settle_d = math.hypot(s[1] - x0, s[2] - y0)
                    break
            sps = [s[3] for s in usable]
            mean = sum(sps) / len(sps)
            res["spray_leg_2"] = {
                "settle_time_s": settle_t,
                "settle_distance_m": settle_d,
                "usable_len_m": math.hypot(usable[-1][1] - x0, usable[-1][2] - y0),
                "speed_mean_m_s": mean,
                "speed_cv": math.sqrt(sum((v - mean) ** 2 for v in sps) / len(sps)) / mean,
            }

    res["segments"] = summarise_route(
        [(t, seq, x, y, sp) for t, seq, x, y, sp, _, _ in samples])
    return res


# 概念验证：绕开 SCurve 的航段混合，直接用解析的匀速圆弧喂 GUIDED 的
# pos+vel+accel 三阶设定点，看飞机能不能在 U 转里保住作业速度。
# 这一步不改固件——先证明概念成立，再决定要不要写成 AC_WPNav 的轨迹原语。
GUIDED_STREAM_HZ = 50.0


def stream_setpoint(mon, n, e, alt, vn, ve, an, ae):
    """发一帧 NED 的位置+速度+加速度设定点，偏航用角速率跟随航迹。"""
    TYPE_MASK = (1 << 10)          # 忽略偏航角，改用偏航角速率
    mon.mav.mav.set_position_target_local_ned_send(
        int(mon.sim_ms), mon.mav.target_system, mon.mav.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, TYPE_MASK,
        n, e, -alt,                # NED：z 向下为正
        vn, ve, 0.0,
        an, ae, 0.0,
        0.0, math.atan2(ve, vn) if (vn or ve) else 0.0)


def uturn_guided_path(leg, swath, speed, alt):
    """生成 直线段1 → 匀速半圆 → 直线段2 的解析参考。

    半圆：圆心 (leg, swath/2)，半径 R = swath/2，
          p(φ) = C + R·(sinφ, −cosφ)，φ 由 0 到 π
          v(φ) = speed·(cosφ, sinφ)          切向，模长恒为 speed
          a(φ) = (speed²/R)·(−sinφ, cosφ)    向心，指向圆心
    速度模长全程恒定 —— 这正是航点/样条做不到的那一点。
    """
    R = swath / 2.0
    t_leg = leg / speed
    t_arc = math.pi * R / speed
    total = t_leg + t_arc + t_leg

    def at(t):
        if t < t_leg:                                  # 直线段 1：向北
            return (speed * t, 0.0, speed, 0.0, 0.0, 0.0)
        t -= t_leg
        if t < t_arc:                                  # 匀速半圆
            phi = speed * t / R
            return (leg + R * math.sin(phi), R - R * math.cos(phi),
                    speed * math.cos(phi), speed * math.sin(phi),
                    -(speed * speed / R) * math.sin(phi),
                    (speed * speed / R) * math.cos(phi))
        t -= t_arc                                     # 直线段 2：向南
        return (leg - speed * t, swath, -speed, 0.0, 0.0, 0.0)

    return at, total, t_leg, t_leg + t_arc


def run_uturn_guided(mon, swath=SWATH_M, speed=2.0, leg=SPRAY_LEG_M):
    alt = ROUTE_ALT_M
    command_takeoff(mon, alt)
    set_mode_wait(mon, "GUIDED", 15)
    for _ in range(40):
        mon.recv()
    print("GUIDED 三阶设定点：行距 %.1f m（半径 %.1f m），速度 %.1f m/s"
          % (swath, swath / 2.0, speed))

    at, total, t_arc0, t_arc1 = uturn_guided_path(leg, swath, speed, alt)
    start = mon.sim_ms
    next_ms = 0.0
    samples = []
    while True:
        t = (mon.sim_ms - start) / 1000.0
        if t > total + 2.0:
            break
        if (mon.sim_ms - start) >= next_ms:
            n, e, vn, ve, an, ae = at(min(t, total))
            stream_setpoint(mon, n, e, alt, vn, ve, an, ae)
            next_ms += 1000.0 / GUIDED_STREAM_HZ
        mon.recv()
        if mon.local is None:
            continue
        sp = math.hypot(mon.local.vx, mon.local.vy)
        err = mon.attitude_error_deg()
        n, e, vn, ve, _, _ = at(min(t, total))
        # 跟踪误差：实际位置到参考位置的距离
        track = math.hypot(mon.local.x - n, mon.local.y - e)
        samples.append((t, sp, track, math.hypot(err[0], err[1]) if err else None,
                        target_tilt_deg(mon)))

    set_mode_wait(mon, "LAND")
    wait_disarmed(mon, 120)

    arc = [s for s in samples if t_arc0 + 0.3 <= s[0] <= t_arc1 - 0.3]
    legs = [s for s in samples if s[0] < t_arc0 - 0.3 or s[0] > t_arc1 + 0.3]
    res = {"swath_m": swath, "uturn_radius_m": swath / 2.0, "target_speed_m_s": speed,
           "mode": "GUIDED-posvelaccel"}
    for name, seg in (("arc", arc), ("legs", legs)):
        if not seg:
            continue
        sps = [s[1] for s in seg]
        tr = [s[2] for s in seg]
        errs = [s[3] for s in seg if s[3] is not None]
        tilts = [s[4] for s in seg if s[4] is not None]
        res[name] = {
            "speed_min_m_s": min(sps),
            "speed_mean_m_s": sum(sps) / len(sps),
            "speed_dip_pct": 100 * (1 - min(sps) / speed),
            "track_err_max_m": max(tr),
            "track_err_rms_m": math.sqrt(sum(x * x for x in tr) / len(tr)),
            "att_err_mean_deg": (sum(errs) / len(errs)) if errs else None,
            "tilt_max_deg": max(tilts) if tilts else None,
        }
    a = res.get("arc", {})
    print("  圆弧段：最低速 %.2f m/s（掉速 %.0f%%），跟踪误差 max %.2f m，指令倾角 max %.1f°"
          % (a.get("speed_min_m_s", 0), a.get("speed_dip_pct", 0),
             a.get("track_err_max_m", 0), a.get("tilt_max_deg") or 0))
    return res


def run_uturn_arcnav(mon, swath=SWATH_M, speed=2.0, leg=SPRAY_LEG_M):
    """固件内的 AC_ArcNav：用 MAV_CMD_NAV_LOITER_TURNS 触发匀速圆弧。

    与 uturn-guided 的区别是圆弧在飞控主循环里生成（400 Hz），不再依赖
    伴飞机以 50 Hz 流设定点。参数映射见 handle_command_int_nav_loiter_turns：
      param1 圈数(0.5=半圈)  param2 切向速度  param3 半径(负=顺时针)  x/y 圆心
    """
    alt = ROUTE_ALT_M
    R = swath / 2.0
    command_takeoff(mon, alt)
    set_mode_wait(mon, "GUIDED", 15)
    for _ in range(40):
        mon.recv()

    # 作业段用「位置+速度」的行进参考：位置以恒定速度前推，横向锁在 y=0。
    # 只给速度会让横向自由漂移，圆弧起点就不在圆上；只给位置又会在终点减速停住，
    # 而圆弧必须以切向速度切入。两者都要。
    lat0, lon0 = HOME[0], HOME[1]
    start = mon.sim_ms
    while mon.sim_ms - start < 120000:
        t = (mon.sim_ms - start) / 1000.0
        mon.mav.mav.set_position_target_local_ned_send(
            int(mon.sim_ms), mon.mav.target_system, mon.mav.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000110111000000,          # 位置 + 速度
            speed * t, 0.0, -alt,   # 参考点不钳位：钳住会让飞机在终点前减速，切入圆弧时就慢了
            speed, 0.0, 0.0, 0, 0, 0, 0, 0)
        mon.recv()
        if mon.local is not None and mon.local.x > leg - 0.8:
            break

    # 圆心在作业段终点向东半个行距处，扫 0.5 圈
    c_lat, c_lon = ne_to_latlon(lat0, lon0, leg, R)
    mon.mav.mav.command_int_send(
        mon.mav.target_system, mon.mav.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS, 0, 0,
        0.5, speed, R, 0,
        int(c_lat * 1e7), int(c_lon * 1e7), alt)
    ack = None
    t0 = mon.sim_ms
    while mon.sim_ms - t0 < 5000:
        msg = mon.recv()
        if msg is not None and msg.get_type() == "COMMAND_ACK" and \
                msg.command == mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS:
            ack = msg.result
            break
    print("  圆弧指令 ACK=%s（0=接受，4=被拒：所需倾角超预算）" % ack)
    if ack != 0:
        set_mode_wait(mon, "LAND")
        wait_disarmed(mon, 120)
        return {"accepted": False, "ack": ack, "swath_m": swath,
                "target_speed_m_s": speed, "uturn_radius_m": R}

    samples = []
    t0 = mon.sim_ms
    while mon.sim_ms - t0 < int(1.6 * math.pi * R / speed * 1000) + 4000:
        mon.recv()
        if mon.local is None:
            continue
        samples.append((math.hypot(mon.local.vx, mon.local.vy),
                        target_tilt_deg(mon),
                        math.hypot(mon.local.x - (leg), mon.local.y - R)))
    set_mode_wait(mon, "LAND")
    wait_disarmed(mon, 120)

    # 稳态段由 ARCN 的 progress 界定，不要靠采样窗口猜圆弧起止——
    # 窗口猜错会把圆弧结束后的悬停减速也算进来，量出来的「掉速」全是假的。
    n = len(samples)
    core = samples[int(n * 0.15):int(n * 0.85)] or samples
    sps = [s[0] for s in core]
    tilts = [s[1] for s in core if s[1] is not None]
    radii = [s[2] for s in core]
    res = {"accepted": True, "swath_m": swath, "uturn_radius_m": R,
           "target_speed_m_s": speed,
           "speed_min_m_s": min(sps), "speed_mean_m_s": sum(sps) / len(sps),
           "speed_dip_pct": 100 * (1 - min(sps) / speed),
           "tilt_max_deg": max(tilts) if tilts else None,
           "flown_radius_mean_m": sum(radii) / len(radii)}
    print("  固件圆弧：最低速 %.2f m/s（掉速 %.0f%%），实飞半径 %.2f m，指令倾角 max %.1f°"
          % (res["speed_min_m_s"], res["speed_dip_pct"],
             res["flown_radius_mean_m"], res["tilt_max_deg"] or 0))
    return res


def run_uturn_auto(mon, swath=SWATH_M, speed=None, leg=None, turns=0.5):
    """AUTO 任务里的协调转弯：LOITER_TURNS 带 param2=1。

    验证两件事，都是 GUIDED 版本测不到的：
      1) 掉头前的作业直线**不减速**——set_next_wp 必须走切线延长点分支。
         若走回「always stop」，飞机会在弧起点停住，入弧速度为零，
         匀速掉头就无从谈起，这正是接入 AUTO 的核心难点。
      2) 掉头段本身匀速、机头贴期望切线（看 ARCN 日志）。

    掉头速度不由任务给出，就是 WPNAV_SPEED——作业掉头与作业直线同速是需求
    本身，mission item 里也确实没有地方存第二个速度。
    """
    alt = ROUTE_ALT_M
    R = swath / 2.0
    leg = SPRAY_LEG_M if leg is None else leg
    lat0, lon0 = HOME[0], HOME[1]

    def ll(n, e):
        return ne_to_latlon(lat0, lon0, n, e)

    WP = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
    TO = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
    LT = mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS
    LAND = mavutil.mavlink.MAV_CMD_NAV_LAND

    a = ll(0.0, 0.0)            # 作业线起点
    b = ll(leg, 0.0)            # 作业线终点 = 弧起点
    c = ll(leg, R)              # 掉头圆心，在弧起点正东半个行距
    d = ll(0.0, 2.0 * R)        # 下一条作业线终点

    # param3 取正 = 顺时针（MAVLink 约定）。起点在圆心正西，顺时针半圈
    # 后出口在圆心正东，出口切线朝正南，正好接下一条作业线。
    items = [
        (WP, lat0, lon0, 0.0),
        (TO, lat0, lon0, alt),
        (WP, a[0], a[1], alt),
        (WP, b[0], b[1], alt),
        (LT, c[0], c[1], alt, turns, 1.0, R, 0.0),
        (WP, d[0], d[1], alt),
        # LAND 给 0/0 表示原地下降。给经纬度会走 do_land 的 FlyToLocation 分支，
        # 实测在本场景下不收敛（与协调转弯无关：turns=1.0 走原有盘旋路径同样卡住）。
        (LAND, 0.0, 0.0, 0.0),
    ]
    upload_mission(mon, items)
    set_mode_wait(mon, "AUTO")
    prepare_for_arm(mon)
    mon.mav.arducopter_arm()
    wait_armed(mon)

    samples = []
    start = mon.sim_ms
    while mon.sim_ms - start < 300000:
        mon.recv()
        if mon.local is None:
            continue
        samples.append((mon.sim_ms, mon.mission_seq,
                        math.hypot(mon.local.vx, mon.local.vy)))
        if not mon.armed and mon.sim_ms - start > 30000:
            break
    wait_disarmed(mon, 60)

    # 关键量：进弧前那一段（mission_seq 指向弧起点航点）的**最低速度**。
    # 若 set_next_wp 让飞机停在航点上，这个数会掉到接近零。
    ARC_SEQ = 4
    pre = [s[2] for s in samples if s[1] == ARC_SEQ - 1]
    arc = [s[2] for s in samples if s[1] == ARC_SEQ]
    res = {"swath_m": swath, "uturn_radius_m": R, "arc_seq": ARC_SEQ,
           "reached_arc": bool(arc)}
    if pre:
        # 只看这一段的后半，前半还在从上一个航点加速
        tail = pre[len(pre) // 2:]
        res["pre_arc_speed_min_m_s"] = min(tail)
        res["pre_arc_speed_mean_m_s"] = sum(tail) / len(tail)
    if arc:
        res["arc_speed_min_m_s"] = min(arc)
        res["arc_speed_mean_m_s"] = sum(arc) / len(arc)
    print("  进弧前尾段最低速 %.2f m/s（若接近 0 说明航点处停了），弧内最低速 %.2f m/s"
          % (res.get("pre_arc_speed_min_m_s", -1), res.get("arc_speed_min_m_s", -1)))
    return res


def run_mag_align(mon, leg=60.0, alt=25.0):
    """P03：磁罗盘相对 IMU 的安装未对准。

    `COMPASS_AUTO_ROT` 只辨识 24 种离散旋转；支架公差、安装面不平留下的几度
    残余偏差，现有链路既测不出也补不了，表现为一个恒定的航向偏差。它直接影响
    P06 的「机头贴期望切线」，也会让 EKF 的磁航向与 GPS 航迹长期不一致。

    这里用 EKF3 的 GSF 航向估计（`XKY0.YC`）作参考——它由 GPS 速度加 IMU 推出，
    完全不含磁罗盘，因此「EKF 航向 − GSF 航向」直接暴露磁罗盘带来的偏差。

    航线走正方形而非直线，有两个原因：GSF 需要水平加速度才可观测（匀速直线上
    它不收敛），四个转角提供了加减速；而且四个不同航向才能把**常值安装偏差**
    与**随航向变化的磁干扰**分开——前者是各航向一致的常数，后者是航向的一次
    或二次谐波，这正是航海罗盘自差分析的做法。
    """
    lat0, lon0 = HOME[0], HOME[1]

    def ll(n, e):
        return ne_to_latlon(lat0, lon0, n, e)

    WP = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
    TO = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
    LAND = mavutil.mavlink.MAV_CMD_NAV_LAND

    # 正方形四角，航向依次约为 N / E / S / W
    corners = [ll(0, 0), ll(leg, 0), ll(leg, leg), ll(0, leg), ll(0, 0)]
    items = [(WP, lat0, lon0, 0.0), (TO, lat0, lon0, alt)]
    items += [(WP, c[0], c[1], alt) for c in corners]
    items.append((LAND, 0.0, 0.0, 0.0))

    upload_mission(mon, items)
    set_mode_wait(mon, "AUTO")
    prepare_for_arm(mon)
    mon.mav.arducopter_arm()
    wait_armed(mon)

    start = mon.sim_ms
    while mon.sim_ms - start < 400000:
        mon.recv()
        if not mon.armed and mon.sim_ms - start > 30000:
            break
    wait_disarmed(mon, 90)
    return {"square_leg_m": leg, "alt_m": alt}


def summarise_arc_window(path):
    """按 ARCN 的 progress 界定弧内稳态段，重算掉速与航向误差。

    场景脚本里的 speed_dip_pct 用的是采样序列的 15%~85% 粗窗口，会把入弧、
    出弧过渡段乃至弧结束后的减速一并算进去——同一条 R=12 m / 5 m/s 的弧，
    粗窗口给 38%，而按 progress 界定只有 1.7%。**量错窗口比量错量更隐蔽**，
    因为得到的数字看上去总是「合理」的。

    这里用 ARCN 自己报的 Prog 作边界：那是生成器对自身进度的判断，不依赖
    任何对起止时刻的猜测。
    """
    if not path or not os.path.exists(path):
        return {}
    from pymavlink import DFReader
    m = DFReader.DFReader_binary(path)
    arc = []
    while True:
        msg = m.recv_match(type="ARCN")
        if msg is None:
            break
        arc.append(msg)
    if not arc:
        return {}
    win = [x for x in arc if 0.02 <= x.Prog <= 0.98]
    if len(win) < 20:
        return {}
    tgt = win[0].Tgt
    spd = [x.Spd for x in win]
    hdg = [abs(x.HdgE) for x in win]
    return {"arc_speed_dip_pct": round(100 * (1 - min(spd) / max(tgt, 1e-6)), 2),
            "arc_speed_min_m_s": round(min(spd), 3),
            "arc_hdg_err_mean_deg": round(sum(hdg) / len(hdg), 2),
            "arc_hdg_err_max_deg": round(max(hdg), 2),
            "arc_spiral_m": round(arc[0].Spir, 2),
            "arc_duration_s": round((arc[-1].TimeUS - arc[0].TimeUS) / 1e6, 2)}


def summarise_mag_align(path, declination_deg=None):
    """从日志分离磁罗盘的**安装偏差**与**磁干扰**。

    参考基准取 EKF3 的 GSF 航向（`XKY0.YC`）：它由 GPS 速度加 IMU 推出，完全
    不含磁罗盘，因此可以当作真航向。

    被测量必须是**磁罗盘自己算出的航向**，不能用 `ATT.Yaw`——后者是 EKF 融合
    的结果，EKF 会在磁罗盘与 GPS 之间自行权衡，注入 12 度偏差时它只泄漏出
    5 度，据此会严重低估安装偏差。所以这里从 `MAG` 的原始三轴磁场出发，用
    `ATT` 的横滚俯仰做倾斜补偿，自己算磁航向。

    只取**稳定直线段**：转弯与加减速时 GSF 与磁航向的动态响应不同步，会伪造
    出并不存在的谐波。判据是航向变化率足够小。

    误差来源已用 SITL 真值姿态（`SIM.Yaw`）分离清楚，结论很干净：

    * **磁航向这一侧算得精确**：注入 0 度时「磁航向 − 真航向」为 +0.01 度，
      即倾斜补偿与磁偏角处理都没有问题。
    * **全部误差来自 GSF 这个参考本身**：同一架次「GSF − 真航向」为 +3.24 度。
    * **GSF 偏置还与被测量耦合**：注入 0/8/16/30 度时其偏置为
      +2.60 / +1.57 / +0.57 / -1.64 度，随注入量单调下降，斜率约 -0.141/度。
      GSF 名义上完全不使用磁罗盘，但注入偏差会改变机体实际姿态与机动模式，
      从而改变它的估计条件。

    于是本函数的输出满足 `测出值 ≈ 1.14 x 真实偏差 - 2.6 度`：斜率约 +14%
    的系统性放大，外加一个约 ±3 度、架次间游走的偏置。**收紧 GSF 收敛判据
    没有用**——YCS 阈值从 8 收到 1，偏置从 +2.60 只变到 +2.69。

    因此：判断「装歪没有、大约多少度」足够可靠，直接拿去补偿也能把主要部分
    修掉（实测注入 30 度、补偿后残余等于零注入基线）；但要做到 1 度以内，
    需要换一个更好的真航向基准，例如地面上以已知朝向摆放后标定。

    区分安装偏差与磁干扰的判据仍不可靠：绕机体 Z 轴的偏差在有倾角时会随姿态
    投影变化，伪装成谐波。现场已确认本项目面对的是固定安装偏差，该判据优先级
    因此下调，输出仅供参考。
    """
    if not path or not os.path.exists(path):
        return {}
    from pymavlink import DFReader
    import math as _m
    import math as _math
    import bisect

    m = DFReader.DFReader_binary(path)
    att, mag, gsf = [], [], []
    dec_from_log = None
    while True:
        msg = m.recv_match(type=["ATT", "MAG", "XKY0", "PARM"])
        if msg is None:
            break
        if msg.get_type() == "PARM":
            # 磁偏角必须取自这架次实际用的值：磁航向是相对磁北的，GSF 是相对
            # 真北的，两者之差里天然含着磁偏角。拍脑袋填一个数会整体平移结果——
            # 实测填 11.6 而实际为 -5.43 时，偏差被平移了 17 度。
            if msg.Name == "COMPASS_DEC":
                dec_from_log = _math.degrees(msg.Value)
            continue
        t = msg.TimeUS * 1e-6
        typ = msg.get_type()
        if typ == "ATT":
            att.append((t, msg.Roll, msg.Pitch, msg.Yaw))
        elif typ == "MAG" and getattr(msg, "I", 0) == 0:
            mag.append((t, msg.MagX, msg.MagY, msg.MagZ))
        elif typ == "XKY0" and getattr(msg, "C", 0) == 0:
            gsf.append((t, msg.YC, msg.YCS))
    if not att or not mag or not gsf:
        return {"mag_align_note": "缺 ATT / MAG / XKY0"}

    if declination_deg is None:
        declination_deg = dec_from_log if dec_from_log is not None else MAG_DECLINATION_DEG

    att_t = [a[0] for a in att]
    gsf_t = [g[0] for g in gsf]

    def at(arr, tarr, t):
        i = min(bisect.bisect_left(tarr, t), len(arr) - 1)
        return arr[i]

    # 航向变化率，用来剔除转弯段
    yaw_rate = []
    for i in range(1, len(att)):
        dt = att[i][0] - att[i - 1][0]
        if dt <= 0:
            yaw_rate.append(0.0); continue
        d = att[i][3] - att[i - 1][3]
        while d > 180: d -= 360
        while d < -180: d += 360
        yaw_rate.append(d / dt)
    yaw_rate.insert(0, 0.0)

    pairs = []
    for t, mx, my, mz in mag:
        g = at(gsf, gsf_t, t)
        if abs(g[0] - t) > 0.5 or g[2] > 8.0:      # GSF 未收敛或时间对不上
            continue
        i = min(bisect.bisect_left(att_t, t), len(att) - 1)
        if abs(yaw_rate[i]) > 5.0:                  # deg/s，转弯段丢掉
            continue
        roll = _m.radians(att[i][1]); pitch = _m.radians(att[i][2])
        # 倾斜补偿：把机体磁场转到水平面
        cr, sr = _m.cos(roll), _m.sin(roll)
        cp, sp = _m.cos(pitch), _m.sin(pitch)
        hx = mx * cp + my * sr * sp + mz * cr * sp
        hy = my * cr - mz * sr
        mag_hdg = _m.degrees(_m.atan2(-hy, hx)) + declination_deg
        d = mag_hdg - g[1]
        while d > 180: d -= 360
        while d < -180: d += 360
        pairs.append((mag_hdg % 360.0, d))

    if len(pairs) < 100:
        return {"mag_align_note": "稳定直线段样本不足（%d）" % len(pairs)}

    n = len(pairs)
    const = sum(p[1] for p in pairs) / n
    c1 = sum(p[1] * _m.cos(_m.radians(p[0])) for p in pairs) * 2 / n
    s1 = sum(p[1] * _m.sin(_m.radians(p[0])) for p in pairs) * 2 / n
    c2 = sum(p[1] * _m.cos(_m.radians(2 * p[0])) for p in pairs) * 2 / n
    s2 = sum(p[1] * _m.sin(_m.radians(2 * p[0])) for p in pairs) * 2 / n
    h1, h2 = _m.hypot(c1, s1), _m.hypot(c2, s2)

    quad = {}
    for hdg, d in pairs:
        quad.setdefault(int(hdg // 90) % 4, []).append(d)
    per_quad = {("N", "E", "S", "W")[q]: round(sum(v) / len(v), 2)
                for q, v in sorted(quad.items())}
    spread = max(per_quad.values()) - min(per_quad.values()) if per_quad else 0.0

    return {"mag_yaw_bias_deg": round(const, 2),
            "mag_harmonic1_deg": round(h1, 2),
            "mag_harmonic2_deg": round(h2, 2),
            "mag_bias_per_quadrant_deg": per_quad,
            "mag_quadrant_spread_deg": round(spread, 2),
            "mag_align_samples": n,
            "mag_align_verdict": ("安装偏差为主，可用一个补偿角修掉"
                                  if spread < max(2.0, abs(const) * 0.4)
                                  else "随航向变化，属磁干扰，补偿角修不掉")}


def estimate_mag_yaw_offset_gps(path, declination_deg=None):
    """用 GPS 地速矢量估计磁罗盘的固定偏航安装偏差，并顺带解出风。

    比 GSF 法更准，因为它只依赖 GPS 速度（精度高），不继承 GSF 自身的偏差。

    模型：设磁罗盘装歪了 Δ，则真航向 = 磁航向 − Δ。无侧滑时空速矢量沿真航向，
    地速 = 空速矢量 + 风：

        vn = Va·cos(ψ−Δ) + Wn
        ve = Va·sin(ψ−Δ) + We

    展开并令 a = Va·cosΔ、b = Va·sinΔ，方程对 (a, b, Wn, We) **完全线性**：

        vn = a·cosψ + b·sinψ + Wn
        ve = a·sinψ − b·cosψ + We

    于是一次最小二乘就能同时解出偏差与风，无需迭代、无需初值：
    Δ = atan2(b, a)，Va = hypot(a, b)。

    要求航段覆盖至少两个差别足够大的航向（四个更好），且采样期间风基本恒定
    ——这正是正方形航线的用意。

    **对多旋翼不成立，仅作交叉参考。** 模型的前提是速度矢量沿机头方向（无
    侧滑），固定翼满足，多旋翼不满足：位置控制器会让机器沿航线飞，机头指哪
    与速度方向无关，缺的那一块靠侧飞补上——这正是 P06 里刻意利用的 crab
    特性。实测注入 0/8/16 度时本方法给出 -0.87 / 4.12 / 8.86 度，斜率只有
    0.55；同一批日志用 GSF 法斜率为 1.09。风与残差都拟合得很好（风解出接近
    零、残差 0.25 m/s），说明不是数值问题，而是前提不适用。

    留着它有两个用处：一是顺带解出风矢量与空速，二是它与 GSF 法的偏离量本身
    反映了航段上的平均侧滑角。
    """
    if not path or not os.path.exists(path):
        return {}
    from pymavlink import DFReader
    import math as _m
    import bisect

    m = DFReader.DFReader_binary(path)
    att, mag, vel = [], [], []
    dec_from_log = None
    while True:
        msg = m.recv_match(type=["ATT", "MAG", "XKF1", "PARM"])
        if msg is None:
            break
        typ = msg.get_type()
        if typ == "PARM":
            if msg.Name == "COMPASS_DEC":
                dec_from_log = _m.degrees(msg.Value)
            continue
        t = msg.TimeUS * 1e-6
        if typ == "ATT":
            att.append((t, msg.Roll, msg.Pitch, msg.Yaw))
        elif typ == "MAG" and getattr(msg, "I", 0) == 0:
            mag.append((t, msg.MagX, msg.MagY, msg.MagZ))
        elif typ == "XKF1" and getattr(msg, "C", 0) == 0:
            vel.append((t, msg.VN, msg.VE))
    if not att or not mag or not vel:
        return {"mag_gps_note": "缺 ATT / MAG / XKF1"}

    if declination_deg is None:
        declination_deg = dec_from_log if dec_from_log is not None else 0.0

    att_t = [a[0] for a in att]
    vel_t = [v[0] for v in vel]

    yaw_rate = [0.0]
    for i in range(1, len(att)):
        dt = att[i][0] - att[i - 1][0]
        d = att[i][3] - att[i - 1][3]
        while d > 180: d -= 360
        while d < -180: d += 360
        yaw_rate.append(d / dt if dt > 0 else 0.0)

    rows = []
    for t, mx, my, mz in mag:
        i = min(bisect.bisect_left(att_t, t), len(att) - 1)
        if abs(yaw_rate[i]) > 5.0:
            continue                      # 转弯段：航向在动，配对不可靠
        j = min(bisect.bisect_left(vel_t, t), len(vel) - 1)
        vn, ve = vel[j][1], vel[j][2]
        if _m.hypot(vn, ve) < 2.0:
            continue                      # 速度太低时航迹方向噪声大
        roll = _m.radians(att[i][1]); pitch = _m.radians(att[i][2])
        cr, sr = _m.cos(roll), _m.sin(roll)
        cp, sp = _m.cos(pitch), _m.sin(pitch)
        hx = mx * cp + my * sr * sp + mz * cr * sp
        hy = my * cr - mz * sr
        psi = _m.radians(_m.degrees(_m.atan2(-hy, hx)) + declination_deg)
        rows.append((psi, vn, ve))

    if len(rows) < 100:
        return {"mag_gps_note": "稳定直线段样本不足（%d）" % len(rows)}

    # 覆盖的航向必须够散，否则 a、b 与风无法分离
    hs = sorted(_m.degrees(r[0]) % 360.0 for r in rows)
    gaps = [hs[k + 1] - hs[k] for k in range(len(hs) - 1)] + [hs[0] + 360 - hs[-1]]
    if max(gaps) > 200.0:
        return {"mag_gps_note": "航向覆盖不足，最大空隙 %.0f°" % max(gaps)}

    # 正规方程：未知 x = [a, b, Wn, We]
    ATA = [[0.0] * 4 for _ in range(4)]
    ATb = [0.0] * 4
    for psi, vn, ve in rows:
        c, s = _m.cos(psi), _m.sin(psi)
        for row, obs in (([c, s, 1.0, 0.0], vn), ([s, -c, 0.0, 1.0], ve)):
            for p in range(4):
                ATb[p] += row[p] * obs
                for q in range(4):
                    ATA[p][q] += row[p] * row[q]

    # 高斯消元（4x4，直接写开比引入依赖更省事）
    M = [ATA[i][:] + [ATb[i]] for i in range(4)]
    for col in range(4):
        piv = max(range(col, 4), key=lambda r: abs(M[r][col]))
        if abs(M[piv][col]) < 1e-9:
            return {"mag_gps_note": "法方程奇异，航向覆盖或速度变化不足"}
        M[col], M[piv] = M[piv], M[col]
        pv = M[col][col]
        for k in range(col, 5):
            M[col][k] /= pv
        for r in range(4):
            if r == col:
                continue
            f = M[r][col]
            for k in range(col, 5):
                M[r][k] -= f * M[col][k]
    a, b, wn, we = (M[i][4] for i in range(4))

    delta = _m.degrees(_m.atan2(b, a))
    va = _m.hypot(a, b)
    resid = 0.0
    for psi, vn, ve in rows:
        c, s = _m.cos(psi), _m.sin(psi)
        resid += (a * c + b * s + wn - vn) ** 2 + (a * s - b * c + we - ve) ** 2
    rms = _m.sqrt(resid / (2 * len(rows)))

    return {"mag_gps_yaw_offset_deg_UNRELIABLE_FOR_MULTIROTOR": round(delta, 2),
            "mag_gps_airspeed_ms": round(va, 2),
            "mag_gps_wind_ms": [round(wn, 2), round(we, 2)],
            "mag_gps_resid_rms_ms": round(rms, 3),
            "mag_gps_samples": len(rows)}


def run_motor_fail(mon, motor=3, alt=None, watch_s=35.0, degrade=False, detect=False,
                   land_on_fail=False):
    """P04：单个电机停转后的可控性。

    注入用 `SIM_ENGINE_FAIL` + `SIM_ENGINE_MUL=0`，它缩放的是 servo PWM
    输入（`SITL_State.cpp`），因此该电机的推力**与转速一起归零**——正是本轮
    定义的「电机停转」，而不只是推力丢失。

    六旋翼掉一个电机，推力其实是够的：50 kg 机悬停油门约 0.16，剩下五台各
    出 0.16×6/5≈0.19，离饱和很远。真正丢掉的是**偏航配平**——失效电机的反
    扭矩没有了对手，六台电机的正反桨不再成对抵消。所以这里重点量的不是掉高，
    而是偏航是否失控、姿态还保不保得住。

    motor 为 1..6（对应 SERVO1..6），内部转成 SIM_ENGINE_FAIL 的位掩码。
    """
    alt = ROUTE_ALT_M if alt is None else alt
    if detect:
        # 只打开检测器，不告诉它哪台电机会失效——这才是真机上的情形
        set_param(mon, "MOT_FAIL_RPM", 300)
        set_param(mon, "MOT_FAIL_TIME", 200)
        set_param(mon, "MOT_FAIL_THST", 0.15)
    command_takeoff(mon, alt)
    set_mode_wait(mon, "GUIDED", 15)

    # 悬停稳定后再注入，否则量到的是爬升瞬态而非失效响应
    t0 = mon.sim_ms
    while mon.sim_ms - t0 < 12000:
        mon.recv()

    pre = []
    t0 = mon.sim_ms
    while mon.sim_ms - t0 < 3000:
        mon.recv()
        if mon.local is not None:
            pre.append(-mon.local.z)

    fail_ms = mon.sim_ms
    set_param(mon, "SIM_ENGINE_MUL", 0)
    set_param(mon, "SIM_ENGINE_FAIL", 1 << (motor - 1))
    if degrade:
        # 同时告诉飞控哪台电机失效。真机上这一步由检测器完成；这里直接给出
        # 答案，先把「降级混控本身管不管用」与「检测得准不准」两件事分开验证。
        set_param(mon, "MOT_FAIL_IDX", motor)
    print("  === 电机 %d 停转注入于 t=%.1fs ===" % (motor, fail_ms / 1000.0))

    if land_on_fail:
        # 切 LAND 让它尽快下来，而不是继续悬停等着。
        #
        # 这段的理由已经变了，记录一下免得后人照旧注释理解：早先没有降级重分配
        # 时，失效后维持定点悬停要靠倾斜抗风，而倾角权限本就不足，两者叠加会把
        # 姿态推到失控——无风勉强可控，2 m/s 风即坠毁，所以当时的说法是「放弃
        # 位置、随风漂移」。MOT_FAIL_ALLOC 的重分配伪逆做出来之后这条不再成立：
        # 8 m/s 风下悬停也能存活，且 LAND 本身是带水平位置控制的，实测从 15 m
        # 降到地面全程水平位移不超过 0.3 m。
        #
        # 现在切 LAND 的理由是另外两条：一是适航条款要的就是「受控应急着陆」，
        # 二是下降本身能压低偏航转速（总推力降下来，寄生偏航力矩随之下降，
        # 实测平均转速 -27%）。
        t_l = mon.sim_ms
        while mon.sim_ms - t_l < 1000:
            mon.recv()
        set_mode_wait(mon, "LAND", 10)

    # 失效瞬间的水平位置。适航条文两档都要求「不超出限制区域」，而判据落在
    # **从失效点算起的水平位移**上，不是落在速度上：飞机放弃位置保持后随风漂移，
    # 漂多远决定了围栏半径必须比限制区域边界内缩多少。
    fail_xy = (mon.local.x, mon.local.y) if mon.local is not None else (0.0, 0.0)

    # 要落地的科目必须看到触地。默认 35 s 只够看住悬停段的过渡与稳态，
    # 从数十米降到地面要更久，窗口不够会在半空截断，量出的水平位移偏小。
    if land_on_fail and watch_s < 150.0:
        watch_s = 150.0

    samples = []
    while mon.sim_ms - fail_ms < watch_s * 1000:
        mon.recv()
        if mon.local is None:
            continue
        samples.append(((mon.sim_ms - fail_ms) / 1000.0, -mon.local.z,
                        math.hypot(mon.local.vx, mon.local.vy),
                        mon.local.x, mon.local.y))
        if not mon.armed:
            break

    res = {"failed_motor": motor, "alt_m": alt, "degraded_mixer": bool(degrade), "detector_on": bool(detect), "land_on_fail": bool(land_on_fail),
           "fail_time_ms": fail_ms,
           "alt_before_m": sum(pre) / len(pre) if pre else None,
           "still_armed_after_watch": bool(mon.armed)}
    if samples:
        res["alt_min_after_fail_m"] = min(s[1] for s in samples)
        res["alt_end_m"] = samples[-1][1]
        res["horiz_drift_max_m_s"] = max(s[2] for s in samples)
        # 自失效点起的水平位移：峰值与末值。末值是落地/观察结束时的偏离，
        # 峰值覆盖中途荡出去又荡回来的情形——围栏要按峰值留裕度。
        res["horiz_excursion_max_m"] = max(
            math.hypot(s[3] - fail_xy[0], s[4] - fail_xy[1]) for s in samples)
        res["horiz_excursion_end_m"] = math.hypot(
            samples[-1][3] - fail_xy[0], samples[-1][4] - fail_xy[1])
        res["watch_end_alt_m"] = samples[-1][1]
        res["landed_within_watch"] = not mon.armed
    # 失效后不再尝试正常降落：此时的可控性正是被测对象，强行切模式会掩盖结果
    if mon.armed and not land_on_fail:
        set_mode_wait(mon, "LAND")
        wait_disarmed(mon, 120)
    print("  掉高 %.1f m，水平漂移峰值 %.1f m/s，%s"
          % ((res.get("alt_before_m") or 0) - (res.get("alt_min_after_fail_m") or 0),
             res.get("horiz_drift_max_m_s", -1),
             "仍在空中" if res["still_armed_after_watch"] else "已落地/坠毁"))
    return res


def summarise_motor_fail(path, fail_time_ms=None):
    """量化失效后的可控性：偏航是否失控、姿态保不保得住、剩余电机是否饱和。"""
    if not path or not os.path.exists(path):
        return {}
    from pymavlink import DFReader
    m = DFReader.DFReader_binary(path)
    att, rate, rcou, msgs = [], [], [], []
    while True:
        msg = m.recv_match(type=["ATT", "RATE", "RCOU", "MSG"])
        if msg is None:
            break
        t = msg.TimeUS * 1e-6
        typ = msg.get_type()
        if typ == "ATT":
            att.append((t, msg.Roll, msg.Pitch, msg.Yaw, msg.DesRoll, msg.DesPitch,
                        msg.DesYaw))
        elif typ == "RATE":
            rate.append((t, msg.Y))
        elif typ == "RCOU":
            rcou.append((t, [getattr(msg, "C%d" % i, 0) for i in range(1, 7)]))
        elif typ == "MSG":
            msgs.append((t, msg.Message))

    # 失效时刻：优先用注入时刻，否则从「某路输出跌到最低」推断
    t_fail = None
    if fail_time_ms is not None and rcou:
        # 日志时间与 mon.sim_ms 同源（都是启动后毫秒）
        t_fail = fail_time_ms / 1000.0
    if t_fail is None:
        for t, ch in rcou:
            if min(ch) < 1050 and max(ch) > 1200:
                t_fail = t
                break
    if t_fail is None:
        return {"motor_fail_note": "无法定位失效时刻"}

    win = [a for a in att if t_fail <= a[0] <= t_fail + 20]
    if len(win) < 50:
        return {"motor_fail_note": "失效后样本不足"}

    roll_err = [abs(a[1] - a[4]) for a in win]
    pitch_err = [abs(a[2] - a[5]) for a in win]

    # 航向**角**是否稳定，而不只是角速率是否小：1.6 deg/s 的残余自旋听起来
    # 微不足道，持续 20 秒就是 32 度的航向漂移，喷幅方向与前视避障早已失准。
    # 判据必须落在角度上。
    def _wrap180(d):
        while d > 180:
            d -= 360
        while d < -180:
            d += 360
        return d
    yaw_err = [abs(_wrap180(a[3] - a[6])) for a in win]
    # 相对失效瞬间的累计漂移，用来区分「稳在一个偏值上」与「一直在转」
    yaw0 = win[0][3]
    yaw_drift = [abs(_wrap180(a[3] - yaw0)) for a in win]

    # 切换到降级混控是一次控制结构的突变，必然有过渡过程。把它和稳态分开
    # 评价，否则一个数字同时背着「超调多大」和「最后稳在哪」两件事，两边
    # 都说不清：实测航向误差峰值 24 度而末值 12 度，前者是过渡超调，后者才
    # 是稳态偏差，用峰值判稳态会误判，用末值判过渡则会漏掉超调。
    def _profile(series, band, tail_s=5.0):
        t = [a[0] for a in win]
        n = len(series)
        overshoot = max(series)
        # 调节时间：误差首次进入 band 且此后不再越出
        settle = None
        for i in range(n):
            if all(v <= band for v in series[i:]):
                settle = t[i] - t[0]
                break
        tail = [series[i] for i in range(n) if t[i] >= t[-1] - tail_s] or series[-1:]
        return {"overshoot_deg": round(overshoot, 2),
                "settle_s": round(settle, 2) if settle is not None else None,
                "steady_deg": round(sum(tail) / len(tail), 2),
                "steady_osc_deg": round(max(tail) - min(tail), 2)}

    yaw_prof = _profile(yaw_err, band=15.0)
    roll_prof = _profile(roll_err, band=10.0)
    yr = [abs(r[1]) for r in rate if t_fail <= r[0] <= t_fail + 20]
    ch_win = [c for t, c in rcou if t_fail <= t <= t_fail + 20]
    sat = 0
    if ch_win:
        # 剩余电机是否顶到上限：饱和意味着推力已经不够，掉高不可避免
        sat = sum(1 for ch in ch_win if max(ch) >= 1990) / len(ch_win)

    # 内置推力丢失检测的触发延迟
    det = None
    for t, s in msgs:
        # 兼容内置检测与本项目检测器两种提示语
        if t >= t_fail and ("Thrust Loss" in s or
                            ("Motor" in s and ("stopped" in s or "degraded" in s))):
            det = t - t_fail
            break

    return {"fail_t_s": round(t_fail, 2),
            "detect_delay_s": round(det, 2) if det is not None else None,
            "roll_err_max_deg": round(max(roll_err), 2),
            "pitch_err_max_deg": round(max(pitch_err), 2),
            "yaw_err_max_deg": round(max(yaw_err), 2),
            "yaw_err_end_deg": round(yaw_err[-1], 2),
            "yaw_drift_max_deg": round(max(yaw_drift), 2),
            "yaw_overshoot_deg": yaw_prof["overshoot_deg"],
            "yaw_settle_s": yaw_prof["settle_s"],
            "yaw_steady_deg": yaw_prof["steady_deg"],
            "yaw_steady_osc_deg": yaw_prof["steady_osc_deg"],
            "roll_overshoot_deg": roll_prof["overshoot_deg"],
            "roll_settle_s": roll_prof["settle_s"],
            "roll_steady_deg": roll_prof["steady_deg"],
            "roll_steady_osc_deg": roll_prof["steady_osc_deg"],
            "yaw_rate_max_degs": round(max(yr), 1) if yr else None,
            "yaw_rate_mean_degs": round(sum(yr) / len(yr), 1) if yr else None,
            "motor_saturation_frac": round(sat, 3)}


def run_yaw_step(mon, alt=None):
    """偏航阶跃辨识：测这架飞机**实际可用**的偏航速率，而不是参数限幅值。

    这是数据需求清单 6.2 节现场科目的仿真版。之所以必须实测：ATC_SLEW_YAW
    是目标限幅，不是能力——把它设成 120 deg/s 不会让机体真的转得动 120。
    P06 的「机头贴期望切线」要求偏航速率 v/R，能不能相邻行掉头完全取决于
    这里量出来的可达值，所以这个数是 P06 的前置输入。

    做法与现场科目一致：定点悬停下给满量程偏航速率指令，等速率稳定后回中，
    左右各做一次；再做一组约 1/3 量程的小阶跃，用于判断线性度。
    """
    alt = ROUTE_ALT_M if alt is None else alt
    command_takeoff(mon, alt)
    set_mode_wait(mon, "GUIDED", 15)

    # 位置忽略、速度置零、加速度忽略、yaw 角忽略、只用 yaw_rate
    MASK = (0b111            # x y z 位置忽略
            | (0b111 << 6)   # 加速度忽略
            | (1 << 10))     # yaw 角忽略，保留 yaw_rate

    def hold(yaw_rate_rads, ms):
        t0 = mon.sim_ms
        while mon.sim_ms - t0 < ms:
            mon.mav.mav.set_position_target_local_ned_send(
                int(mon.sim_ms), mon.mav.target_system, mon.mav.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED, MASK,
                0.0, 0.0, -alt, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, yaw_rate_rads)
            mon.recv()

    # 指令给到远超能力的值，让机体自己饱和在它的真实上限上
    FULL = 3.0      # rad/s，约 172 deg/s，肯定超出六旋翼能力
    THIRD = 1.0     # rad/s，约 57 deg/s，用于判断线性度
    marks = []
    def mark(tag):
        marks.append((mon.sim_ms, tag))

    hold(0.0, 5000)                     # 先稳住
    mark("full_pos"); hold(+FULL, 4000)
    mark("settle");   hold(0.0, 4000)
    mark("full_neg"); hold(-FULL, 4000)
    mark("settle");   hold(0.0, 3000)
    mark("third_pos"); hold(+THIRD, 4000)
    mark("settle");    hold(0.0, 3000)

    set_mode_wait(mon, "LAND")
    wait_disarmed(mon, 120)
    return {"yaw_step_marks": marks, "full_cmd_rads": FULL, "third_cmd_rads": THIRD}


def summarise_yaw_step(path):
    """从日志读出可达偏航速率与一阶时间常数。

    时间常数取阶跃后速率爬到稳态 63.2% 所用的时间——只要响应近似一阶，
    这个数就是 AC_ArcNav 航向前视时间 τ 应该取的值。
    """
    if not path or not os.path.exists(path):
        return {}
    from pymavlink import DFReader
    m = DFReader.DFReader_binary(path)
    rate, pidy = [], []
    while True:
        msg = m.recv_match(type=["RATE", "PIDY"])
        if msg is None:
            break
        if msg.get_type() == "RATE":
            rate.append((msg.TimeUS / 1e6, msg.YDes, msg.Y, msg.YOut))
        else:
            pidy.append((msg.TimeUS / 1e6, msg.Tar, msg.Act, msg.P, msg.I, msg.D))
    if not rate:
        return {}

    ys = [r[2] for r in rate]
    out = {"yaw_rate_max_pos_degs": max(ys), "yaw_rate_max_neg_degs": min(ys),
           "yaw_out_peak": max(abs(r[3]) for r in rate)}
    if pidy:
        out["pidy_tar_peak_rads"] = max(abs(p[1]) for p in pidy)
        out["pidy_act_peak_rads"] = max(abs(p[2]) for p in pidy)
        # 目标远大于实际，说明是机体/增益权限不足而非限幅
        out["tar_over_act"] = (out["pidy_tar_peak_rads"] /
                               max(out["pidy_act_peak_rads"], 1e-6))

    # 用正向满阶跃段拟一阶时间常数：找速率从接近 0 单调爬升的最长一段
    best = None
    i = 0
    while i < len(rate):
        if rate[i][2] > 5.0:            # deg/s，离开噪声带
            j = i
            while j + 1 < len(rate) and rate[j + 1][2] >= rate[j][2] - 3.0:
                j += 1
            if best is None or (j - i) > (best[1] - best[0]):
                best = (i, j)
            i = j + 1
        else:
            i += 1
    if best:
        seg = rate[best[0]:best[1] + 1]
        steady = max(r[2] for r in seg)
        target = steady * 0.632
        for t, _, y, _ in seg:
            if y >= target:
                out["yaw_tau_s"] = t - seg[0][0]
                break
        out["yaw_steady_degs"] = steady
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
    parser.add_argument("case", choices=("landing", "reverse", "circle", "loiter-circle", "fence", "route", "uturn", "uturn-guided", "uturn-arcnav", "uturn-auto", "yaw-step", "mag-align", "motor-fail"))
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
    parser.add_argument("--land-on-fail", action="store_true",
                        help="失效后切 LAND，放弃定点——受控应急着陆不要求定点")
    parser.add_argument("--detect", action="store_true",
                        help="打开基于转速的停转检测器，由它自己发现失效电机")
    parser.add_argument("--degrade", action="store_true",
                        help="失效同时写 MOT_FAIL_IDX，启用降级混控（剔除失效电机并放弃偏航）")
    parser.add_argument("--polyfence-radius", type=float, default=None, metavar="M",
                        help="上传一个以 Home 为心的 polyfence 包含圆（米）。"
                             "路径规划器只认 polyfence，不认 FENCE_RADIUS 参数式围栏，"
                             "要验证 AUTO 下的围控必须走这条路")
    parser.add_argument("--fence-mode", default="LOITER", metavar="MODE",
                        help="fence 场景中被测的飞行模式（LOITER/POSHOLD/ALT_HOLD/SPORT/…）")
    parser.add_argument("--polyfence-sides", type=int, default=0, metavar="N",
                        help="0 给包含圆；>=3 给正多边形。Dijkstra 只认多边形")
    parser.add_argument("--fail-alt", type=float, default=None,
                        help="motor-fail 场景中注入失效时的高度 m（默认沿用航线高度）。"
                             "适航「不超出限制区域」的判据落在失效点起算的水平位移上，"
                             "而位移随下降耗时增长，故须按拟运行高度分别验证")
    parser.add_argument("--release-alt", type=float, default=None,
                        help="landing 场景：在离地这么高时放开限速，复现 LNDS 近地误判")
    parser.add_argument("--release-speed", type=int, default=200,
                        help="放开后的 LAND_SPEED，cm/s")
    parser.add_argument("--motor", type=int, default=3,
                        help="motor-fail 场景中停转的电机编号 1..6")
    parser.add_argument("--model-set", action="append", metavar="KEY=VALUE",
                        help="覆盖物理模型参数（eft_hexa.json 的键），可重复；"
                             "例如 --model-set vrs_gain=0.25")
    parser.add_argument("--turns", type=float, default=0.5,
                        help="LOITER_TURNS 圈数；<=0.5 走匀速协调转弯，>0.5 走原有盘旋")
    parser.add_argument("--swath", type=float, default=SWATH_M,
                        help="uturn 场景的作业行距（米），U 转半径 = 行距/2")
    parser.add_argument("--speed", type=float, default=2.0,
                        help="uturn-guided 的目标作业速度 m/s")
    parser.add_argument("--uturn-style", choices=("square", "arc", "spline", "spline-arc"), default="square",
                        help="U 型转弯形状：square 直角式 / arc 多点近似 / spline 样条平滑")
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

    # 命令行指定的参数不允许被场景内部的 set_param 覆盖。见 CLI_OVERRIDDEN。
    CLI_OVERRIDDEN.update(overrides.keys())

    if not os.path.exists(SITL_BIN):
        raise SystemExit("缺少 %s；先在仓库根目录执行 ./waf configure --board sitl && ./waf copter" % SITL_BIN)
    model_overrides = {}
    for item in args.model_set or []:
        if "=" not in item:
            raise SystemExit("--model-set 需要 KEY=VALUE 形式，收到 %r" % item)
        k, v = item.split("=", 1)
        try:
            model_overrides[k.strip()] = float(v)
        except ValueError:
            raise SystemExit("--model-set 的值必须是数字，收到 %r" % v)
    out, model_path, variant, algo_path = prepare_run(
        args.case, args.baseline, args.output, overrides, args.variant, model_overrides)
    proc, stdout = start_sitl(out, model_path, args.speedup, algo_path)
    result = {"case": args.case, "variant": variant, "frame": "hexa-dji",
              "motor_count": 6, "output": out, "model_overrides": model_overrides,
              "physics": "nominal" if args.baseline else "problem",
              "param_overrides": overrides}
    try:
        mon = connect(proc)
        if args.polyfence_radius:
            # 必须在任务之前上传：路径规划器在任务开始时读取围栏，
            # 中途上传不会重新规划已经在飞的航段。
            upload_fence(mon, args.polyfence_radius, HOME[0], HOME[1],
                         sides=args.polyfence_sides)
            result["polyfence_radius_m"] = args.polyfence_radius
            result["polyfence_sides"] = args.polyfence_sides
        if args.case == "landing":
            result.update(run_landing(mon, args.release_alt, args.release_speed))
        elif args.case == "circle":
            result.update(run_circle(mon))
        elif args.case == "loiter-circle":
            result.update(run_loiter_circle(mon))
        elif args.case == "fence":
            result.update(run_fence(mon, args.fence_mode,
                                    skip_param_fence=bool(args.polyfence_radius)))
        elif args.case == "uturn-auto":
            result.update(run_uturn_auto(mon, args.swath, turns=args.turns))
        elif args.case == "motor-fail":
            result.update(run_motor_fail(mon, args.motor, degrade=args.degrade, detect=args.detect,
                                         land_on_fail=args.land_on_fail,
                                         alt=args.fail_alt))
        elif args.case == "mag-align":
            result.update(run_mag_align(mon))
        elif args.case == "yaw-step":
            result.update(run_yaw_step(mon))
        elif args.case == "uturn-arcnav":
            result.update(run_uturn_arcnav(mon, args.swath, args.speed))
        elif args.case == "uturn-guided":
            result.update(run_uturn_guided(mon, args.swath, args.speed))
        elif args.case == "uturn":
            result.update(run_uturn(mon, args.swath, args.uturn_style))
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
    if args.case == "yaw-step":
        result["metrics"].update(summarise_yaw_step(result["dataflash_log"]))
    if args.case in ("uturn-arcnav", "uturn-auto", "uturn-guided"):
        result["metrics"].update(summarise_arc_window(result["dataflash_log"]))
    if args.case == "motor-fail":
        result["metrics"].update(summarise_motor_fail(result["dataflash_log"],
                                                     result.get("fail_time_ms")))
    if args.case == "mag-align":
        result["metrics"].update(summarise_mag_align(result["dataflash_log"]))
        result["metrics"].update(estimate_mag_yaw_offset_gps(result["dataflash_log"]))
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
