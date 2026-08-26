#!/usr/bin/env python3
"""Quick fence/arm/mode timeline from ArduPilot .bin log."""
import sys
from pathlib import Path

try:
    from pymavlink import mavutil
except ImportError:
    import subprocess
    subprocess.check_call([sys.executable, "-m", "pip", "install", "pymavlink", "-q"])
    from pymavlink import mavutil

MODE_NAMES = {
    0: "STAB", 1: "ACRO", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED", 5: "LOITER",
    6: "RTL", 7: "CIRCLE", 9: "LAND", 11: "DRIFT", 13: "SPORT", 14: "FLIP",
    15: "AUTOTUNE", 16: "POSHOLD", 17: "BRAKE", 18: "THROW", 19: "AVOID_ADSB",
    20: "GUIDED_NOGPS", 21: "SMART_RTL", 22: "FLOWHOLD", 23: "FOLLOW", 24: "ZIGZAG",
    25: "SYSTEMID", 26: "AUTOROTATE", 27: "AUTO_RTL",
}

log_path = Path(sys.argv[1])
m = mavutil.mavlink_connection(str(log_path))
t0_us = None

def msg_time_s(msg):
    global t0_us
    d = msg.to_dict()
    tus = d.get("TimeUS")
    if tus is None:
        return None
    if t0_us is None:
        t0_us = tus
    return (tus - t0_us) / 1e6

keywords = (
    "fence", "prearm", "arm", "disarm", "mode", "auto", "loiter", "rtl",
    "althold", "breach", "failsafe", "mission", "guided", "takeoff",
)

events = []
modes = []
arms = []
cmds = []
fence_err = []

while True:
    msg = m.recv_match(blocking=False)
    if msg is None:
        break
    t = msg_time_s(msg)
    if t is None:
        continue
    mt = msg.get_type()
    if mt == "MSG":
        text = msg.Message
        if any(k in text.lower() for k in keywords):
            events.append((t, text))
    elif mt == "MODE":
        d = msg.to_dict()
        mode_num = d.get("Mode", d.get("ModeNum"))
        name = MODE_NAMES.get(mode_num, str(mode_num))
        modes.append((t, name, d))
    elif mt == "ARM":
        d = msg.to_dict()
        state = "ARM" if d.get("ArmState") else "DISARM"
        arms.append((t, state, d))
    elif mt == "CMD":
        cmds.append((t, msg.to_dict()))
    elif mt == "ERR" and getattr(msg, "Subsys", None) == 9:
        fence_err.append((t, int(msg.ECode)))

print(f"Log: {log_path.name}")
print("\n=== ARM / DISARM ===")
for t, state, d in arms:
    print(f"{t:9.1f}s {state}")

print("\n=== MODE CHANGES ===")
for t, name, d in modes:
    rsn = d.get("Rsn", "?")
    print(f"{t:9.1f}s -> {name:10s} Rsn={rsn}")

print("\n=== FAILSAFE_FENCE ERR ===")
for t, ecode in fence_err:
    print(f"{t:9.1f}s ECode={ecode}")

print("\n=== KEY MSG (filtered) ===")
for t, text in events:
    print(f"{t:9.1f}s {text}")

print("\n=== MAV CMD (first 20) ===")
for t, d in cmds[:20]:
    print(f"{t:9.1f}s CId={d.get('CId')} P1={d.get('Prm1')} frame={d.get('Frame')}")
