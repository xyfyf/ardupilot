import struct
import re
import math
from collections import defaultdict

LOG = r"c:\Users\Administrator\Desktop\long_text_03231322-AEE7-4F7C-827F-0265C0DBE96A.txt"

with open(LOG, "r", encoding="utf-8", errors="ignore") as f:
    text = f.read()

hex_str = re.sub(r"[^0-9A-Fa-f]", "", text)
data = bytes.fromhex(hex_str)

msgs = []
i = 0
while i < len(data):
    if data[i] != 0xFD:
        i += 1
        continue
    if i + 10 > len(data):
        break
    plen = data[i + 1]
    seq = data[i + 4]
    msgid = data[i + 7] | (data[i + 8] << 8) | (data[i + 9] << 16)
    total = 10 + plen + 2
    if i + total > len(data):
        break
    payload = data[i + 10 : i + 10 + plen]
    msgs.append((msgid, seq, plen, payload))
    i += total


def f32(b, o):
    return struct.unpack_from("<f", b, o)[0]


def i32(b, o):
    return struct.unpack_from("<i", b, o)[0]


def u32(b, o):
    return struct.unpack_from("<I", b, o)[0]


def u16(b, o):
    return struct.unpack_from("<H", b, o)[0]


def i16(b, o):
    return struct.unpack_from("<h", b, o)[0]


by_id = defaultdict(list)
for m, s, l, p in msgs:
    by_id[m].append(p)

print("=== Overview ===")
print(f"frames={len(msgs)}, duration~5.1s")
print(f"msg_ids={dict(sorted((k, len(v)) for k, v in by_id.items()))}")

modes = {0: "STABILIZE", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED", 5: "LOITER", 6: "RTL", 9: "LAND"}

if 0 in by_id:
    p = by_id[0][0]
    cm = u32(p, 0)
    print(
        f"\nHEARTBEAT: type=2(copter) autopilot=3 custom_mode={cm}({modes.get(cm, '?')}) "
        f"base_mode=0x{p[6]:02X} status={p[8]}"
    )

if 30 in by_id:
    atts = [(math.degrees(f32(p, 4)), math.degrees(f32(p, 8)), math.degrees(f32(p, 12))) for p in by_id[30]]
    rolls = [a[0] for a in atts]
    pitches = [a[1] for a in atts]
    yaws = [a[2] for a in atts]
    print(
        f"\nATTITUDE n={len(atts)} roll={min(rolls):.2f}~{max(rolls):.2f}deg "
        f"pitch={min(pitches):.2f}~{max(pitches):.2f} yaw={min(yaws):.1f}~{max(yaws):.1f}"
    )

if 33 in by_id:
    print(f"\nGLOBAL_POSITION n={len(by_id[33])}:")
    for idx, p in enumerate(by_id[33]):
        print(
            f"  [{idx}] lat={i32(p,4)/1e7:.7f} lon={i32(p,8)/1e7:.7f} "
            f"alt={i32(p,12)/1000:.1f}m rel={i32(p,16)/1000:.1f}m "
            f"vel=({i16(p,20)},{i16(p,22)},{i16(p,24)}) hdg={u16(p,26)/100:.2f}"
        )

if 24 in by_id:
    fix_names = {0: "NO_GPS", 1: "NO_FIX", 2: "2D", 3: "3D", 4: "DGPS", 5: "RTK_FLOAT", 6: "RTK_FIXED"}
    print(f"\nGPS_RAW_INT n={len(by_id[24])}:")
    for idx, p in enumerate(by_id[24]):
        print(
            f"  [{idx}] fix={fix_names.get(p[28], p[28])} sats={p[29]} "
            f"lat={i32(p,8)/1e7:.7f} lon={i32(p,12)/1e7:.7f} alt={i32(p,16)/1000:.1f}m "
            f"eph={u16(p,20)/100:.2f}m epv={u16(p,22)/100:.2f}m"
        )

if 1 in by_id:
    print(f"\nSYS_STATUS n={len(by_id[1])}:")
    for idx, p in enumerate(by_id[1]):
        print(
            f"  [{idx}] V={u16(p,14)/1000:.2f}V I={i16(p,16)/100:.2f}A remain={p[18]}% load={u16(p,12)/10:.0f}%"
        )

if 74 in by_id:
    print(f"\nVFR_HUD n={len(by_id[74])} payload_len={len(by_id[74][0])}:")
    for idx, p in enumerate(by_id[74]):
        if len(p) >= 20:
            print(
                f"  [{idx}] gs={f32(p,4):.2f}m/s hdg={i16(p,8)} thr={u16(p,10)} "
                f"alt={f32(p,12):.1f} climb={f32(p,16):.2f}"
            )

if 193 in by_id:
    print(f"\nEKF_STATUS n={len(by_id[193])}:")
    for idx, p in enumerate(by_id[193]):
        flags = u16(p, 10)
        print(
            f"  [{idx}] vel={f32(p,0):.4f} pos_h={f32(p,4):.4f} pos_v={f32(p,8):.4f} flags=0x{flags:04X}"
        )

if 147 in by_id:
    p = by_id[147][0]
    print(f"\nBATTERY_STATUS payload_len={len(p)}")

if 178 in by_id:
    print(f"\nAHRS2 n={len(by_id[178])}:")
    p = by_id[178][0]
    print(f"  lat={i32(p,8)/1e7:.7f} lon={i32(p,12)/1e7:.7f} alt={f32(p,16):.1f}m")

if 32 in by_id:
    p = by_id[32][-1]
    print(
        f"\nLOCAL_POSITION_NED: x={f32(p,0):.2f} y={f32(p,1):.2f} z={f32(p,2):.2f} "
        f"vx={f32(p,4):.2f} vy={f32(p,5):.2f} vz={f32(p,6):.2f}"
    )

if 65 in by_id:
    p = by_id[65][0]
    chans = [u16(p, 4 + i * 2) for i in range(8)]
    print(f"\nRC_CHANNELS ch1-8: {chans}")

if 27 in by_id:
    p = by_id[27][-1]
    print(
        f"\nRAW_IMU: acc=({i16(p,0)},{i16(p,2)},{i16(p,4)}) "
        f"gyro=({i16(p,6)},{i16(p,8)},{i16(p,10)})"
    )

seqs = [s for _, s, _, _ in msgs]
missing = []
for a, b in zip(seqs, seqs[1:]):
    d = (b - a) % 256
    if d not in (1, 0):
        missing.append((a, b, d))
print(f"\nseq_gaps={len(missing)}")
if missing:
    print("  samples:", missing[:8])

print("\n=== Anomaly checks ===")
issues = []

if 0 in by_id and by_id[0][0][8] != 4:
    issues.append(f"FC status={by_id[0][0][8]} (not ACTIVE=4)")

if 1 in by_id:
    for p in by_id[1]:
        v = u16(p, 14) / 1000
        if v < 10.5:
            issues.append(f"low voltage {v:.2f}V")
        if p[18] == 0:
            issues.append("battery_remaining=0% (uncalibrated or invalid)")

if 24 in by_id:
    for p in by_id[24]:
        if p[28] < 3:
            issues.append(f"GPS fix={p[28]} not 3D")
        if p[29] < 6:
            issues.append(f"GPS sats={p[29]} low")

if 30 in by_id:
    max_roll = max(abs(math.degrees(f32(p, 4))) for p in by_id[30])
    if max_roll > 45:
        issues.append(f"large roll max={max_roll:.1f}deg")

if len(by_id.get(33, [])) < 3:
    issues.append(f"GLOBAL_POSITION only {len(by_id.get(33, []))} msgs (low rate)")

if len(msgs) < 100:
    issues.append(f"only {len(msgs)} frames in ~5s (truncated stream or packet loss)")

# Check GPS vs GLOBAL position consistency
if 24 in by_id and 33 in by_id:
    g = by_id[24][-1]
    p = by_id[33][-1]
    dlat = abs(i32(g, 8) - i32(p, 4)) / 1e7 * 111320
    dlon = abs(i32(g, 12) - i32(p, 8)) / 1e7 * 111320 * math.cos(math.radians(i32(p, 4) / 1e7))
    dist = math.hypot(dlat, dlon)
    if dist > 5:
        issues.append(f"GPS vs GLOBAL position diff ~{dist:.1f}m")
    else:
        print(f"  GPS/GLOBAL consistency OK (~{dist:.2f}m)")

# EKF flags
if 193 in by_id:
    flags = u16(by_id[193][-1], 10)
    # common ArduPilot EKF flags bits
    ekf_ok = flags & 0x01
    if not ekf_ok:
        issues.append(f"EKF flags=0x{flags:04X} attitude unhealthy?")
    else:
        print(f"  EKF attitude OK flags=0x{flags:04X}")

for x in issues:
    print(" [!]", x)
if not issues:
    print("  no obvious numeric anomalies")
