#!/usr/bin/env python3
"""把 log_to_sitl_scenario.py 产出的场景在 SITL 里闭环飞一遍。

用法:
    python3 fly_scenario.py <场景目录> [--speedup 4] [--max-t 秒] [--model hexa-dji]

做的事:
    1. 用场景目录里的 params.parm + sitl_overrides.parm 启动 build/sitl/bin/arducopter
       （无 GCS、无地图，纯命令行，适合批量跑）
    2. 等 EKF 拿到 GPS 位置后切 LOITER、指令解锁
    3. 按 timeline.csv 的时间戳，以仿真时钟为准逐条发 RC_CHANNELS_OVERRIDE 回放飞手杆量
    4. 时间线播完切 LAND，等上锁退出；SITL 日志在 <场景目录>/logs/

之后可以用 log_control_metrics.py 同时分析真机日志与 SITL 日志做横向对比。

解锁用指令而不是杆量手势：本机型 ARMING_RUDDER=0，真机就是 GCS 解锁的；
且真机日志 LOG_DISARMED=0，时间线从解锁那一刻开始。
"""
import sys
import os
import csv
import time
import argparse
import subprocess

_HERE = os.path.dirname(os.path.abspath(__file__))
_AP = os.path.normpath(os.path.join(_HERE, os.pardir, os.pardir))
if os.path.isdir(os.path.join(_AP, "modules", "mavlink")):
    sys.path.insert(0, os.path.join(_AP, "modules", "mavlink"))
from pymavlink import mavutil  # noqa: E402

SITL_BIN = os.path.join(_AP, "build", "sitl", "bin", "arducopter")
DEFAULTS = os.path.join(_AP, "Tools", "autotest", "default_params", "copter.parm")


def frame_from_runner(d):
    """run_sitl.sh 里已经按 FRAME_CLASS/FRAME_TYPE 选好了 SITL 模型，直接复用。"""
    try:
        for line in open(os.path.join(d, "run_sitl.sh"), encoding="utf-8"):
            if line.strip().startswith("-v ArduCopter -f"):
                return line.split("-f", 1)[1].split()[0]
    except OSError:
        pass
    return "hexa-dji"


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("scenario")
    ap.add_argument("--speedup", type=float, default=4)
    ap.add_argument("--max-t", type=float, default=None, help="只回放前 N 秒")
    ap.add_argument("--model", default=None)
    ap.add_argument("--arming-check", type=int, default=0,
                    help="SITL 里的 ARMING_CHECK，默认 0（真机的检查项多半依赖不存在的硬件）")
    a = ap.parse_args(argv)

    d = os.path.abspath(a.scenario)
    if not os.path.exists(SITL_BIN):
        raise SystemExit("没有 %s，先 ./waf configure --board sitl && ./waf copter" % SITL_BIN)
    home = open(os.path.join(d, "home.txt"), encoding="utf-8").read().strip()
    model = a.model or frame_from_runner(d)

    extra = os.path.join(d, "fly_extra.parm")
    with open(extra, "w", encoding="utf-8") as f:
        f.write("SIM_SPEEDUP %g\nARMING_CHECK %d\nLOG_DISARMED 0\n" % (a.speedup, a.arming_check))
    params = ",".join([DEFAULTS, os.path.join(d, "params.parm"),
                       os.path.join(d, "sitl_overrides.parm"), extra])

    rows = sorted((r for r in csv.DictReader(open(os.path.join(d, "timeline.csv"), encoding="utf-8"))
                   if r["kind"] == "rc"), key=lambda r: float(r["t_s"]))
    if not rows:
        raise SystemExit("timeline.csv 里没有杆量记录")
    t_end = float(rows[-1]["t_s"]) if a.max_t is None else a.max_t

    cmd = [SITL_BIN, "--model", model, "--speedup", str(a.speedup), "--home", home, "--defaults", params]
    print("SITL:", " ".join(cmd))
    sitl_log = open(os.path.join(d, "sitl_stdout.log"), "w")
    p = subprocess.Popen(cmd, cwd=d, stdout=sitl_log, stderr=subprocess.STDOUT)
    try:
        time.sleep(3)
        m = mavutil.mavlink_connection("tcp:127.0.0.1:5760")
        m.wait_heartbeat()

        def rc(r, pch, thr, yaw, mode=1800):
            m.mav.rc_channels_override_send(m.target_system, m.target_component,
                                            r, pch, thr, yaw, mode, 0, 0, 0)

        t0 = time.time()
        while time.time() - t0 < 120:
            x = m.recv_match(type="STATUSTEXT", blocking=True, timeout=1)
            if x and "is using GPS" in x.text:
                break
            rc(1500, 1500, 1000, 1500)
        else:
            raise SystemExit("120 s 内 EKF 没拿到 GPS 位置，看 sitl_stdout.log")

        for sid in (mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, mavutil.mavlink.MAV_DATA_STREAM_EXTRA1):
            m.mav.request_data_stream_send(m.target_system, m.target_component, sid, 10, 1)
        rc(1500, 1500, 1000, 1500)
        time.sleep(1)
        m.set_mode("LOITER")
        time.sleep(1)
        m.arducopter_arm()
        m.motors_armed_wait()
        print("已解锁，开始回放 %d 组杆量（%.0f s 仿真时间，speedup %g）" % (len(rows), t_end, a.speedup))

        def simt():
            x = m.recv_match(type=["SYSTEM_TIME", "ATTITUDE"], blocking=True, timeout=5)
            return x.time_boot_ms / 1000.0 if x else None

        base = simt()
        i, now, nxt = 0, 0.0, 10.0
        while i < len(rows):
            s = simt()
            if s is None:
                raise SystemExit("SITL 没有心跳了（通常是浮点异常崩溃），看 sitl_stdout.log")
            now = s - base
            if now > t_end:
                break
            while i < len(rows) and float(rows[i]["t_s"]) <= now:
                r = rows[i]
                rc(int(r["a"]), int(r["b"]), int(r["c"]), int(r["d"]))
                i += 1
            x = m.recv_match(type="STATUSTEXT", blocking=False)
            if x and ("Hit ground" in x.text or "Crash" in x.text or "EKF" in x.text):
                print("  %6.1f s  %s" % (now, x.text))
            if now >= nxt:
                print("  %6.1f s  已回放 %d/%d" % (now, i, len(rows)))
                nxt += 10.0
        print("时间线结束于仿真 %.1f s，切 LAND" % now)
        rc(1500, 1500, 1500, 1500)
        m.set_mode("LAND")
        m.motors_disarmed_wait()
        print("已上锁。SITL 日志在 %s/logs/" % d)
    finally:
        p.terminate()
        time.sleep(1)


if __name__ == "__main__":
    main(sys.argv[1:])
