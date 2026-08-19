# -*- coding: utf-8 -*-
"""Parse a serial hex dump of MAVLink v2 frames and flag anomalies."""
from __future__ import annotations

import math
import re
from collections import Counter
from pathlib import Path

from pymavlink.dialects.v20 import ardupilotmega as mavlink2

SRC = Path(r"c:\Users\Administrator\Desktop\long_text_03231322-AEE7-4F7C-827F-0265C0DBE96A.txt")
OUT = Path(r"c:\Users\Administrator\Desktop\WORK\ardupilot-ubuntu\_mavlink_analysis.txt")


def parse_dump(path: Path):
    text = path.read_bytes().decode("gbk", errors="replace")
    chunks = []
    for line in text.splitlines():
        m = re.match(r"\[(\d{2}:\d{2}:\d{2}\.\d+)\]", line)
        ts = m.group(1) if m else None
        hex_part = re.sub(r"^\[.*?\][^\s0-9A-Fa-f]*", "", line)
        hex_bytes = re.findall(r"(?<![0-9A-Fa-f])([0-9A-Fa-f]{2})(?![0-9A-Fa-f])", hex_part)
        if hex_bytes:
            chunks.append((ts, bytes(int(b, 16) for b in hex_bytes)))
    return chunks


def x25crc(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        tmp = b ^ (crc & 0xFF)
        tmp ^= (tmp << 4) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


def scan_frames(data: bytes):
    frames = []
    i = 0
    n = len(data)
    while i < n:
        if data[i] != 0xFD:
            i += 1
            continue
        if i + 12 > n:
            break
        plen = data[i + 1]
        incompat = data[i + 2]
        sig_len = 13 if (incompat & 0x01) else 0
        flen = 10 + plen + 2 + sig_len
        if i + flen > n:
            i += 1
            continue
        raw = data[i : i + flen]
        msgid = raw[7] | (raw[8] << 8) | (raw[9] << 16)
        cls = mavlink2.mavlink_map.get(msgid)
        extra = getattr(cls, "crc_extra", None) if cls else None
        if extra is None:
            i += 1
            continue
        crc_rx = raw[10 + plen] | (raw[11 + plen] << 8)
        crc_calc = x25crc(raw[1 : 10 + plen] + bytes([extra]))
        if crc_calc != crc_rx:
            i += 1
            continue
        msg = None
        err = None
        try:
            msg = mavlink2.MAVLink(None).decode(bytearray(raw))
        except Exception as e:
            err = str(e)
        frames.append(
            {
                "offset": i,
                "raw": raw,
                "len": plen,
                "seq": raw[4],
                "sysid": raw[5],
                "compid": raw[6],
                "msgid": msgid,
                "name": cls.msgname,
                "msg": msg,
                "err": err,
            }
        )
        i += flen
    return frames


def mode_name(custom_mode: int) -> str:
    copter = {
        0: "STABILIZE",
        1: "ACRO",
        2: "ALT_HOLD",
        3: "AUTO",
        4: "GUIDED",
        5: "LOITER",
        6: "RTL",
        7: "CIRCLE",
        9: "LAND",
        16: "POSHOLD",
        17: "BRAKE",
        21: "SMART_RTL",
    }
    return copter.get(custom_mode, f"MODE_{custom_mode}")


def sensor_bits(val: int):
    names = {
        1: "GYRO",
        2: "ACCEL",
        4: "MAG",
        8: "ABS_PRESSURE",
        16: "DIFF_PRESSURE",
        32: "GPS",
        64: "OPTICAL_FLOW",
        256: "LASER_POS",
        1024: "RATE_CTRL",
        2048: "ATT_STAB",
        4096: "YAW_POS",
        8192: "Z_ALT_CTRL",
        16384: "XY_POS_CTRL",
        32768: "MOTOR",
        65536: "RC",
        1048576: "GEOFENCE",
        2097152: "AHRS",
        4194304: "TERRAIN",
        16777216: "LOGGING",
        33554432: "BATTERY",
        67108864: "PROXIMITY",
        268435456: "PREARM",
        536870912: "OA",
        1073741824: "PROPULSION",
    }
    return [n for b, n in names.items() if val & b]


def ekf_flag_names(flags: int):
    names = {
        1: "ATTITUDE",
        2: "VEL_HORIZ",
        4: "VEL_VERT",
        8: "POS_HORIZ_REL",
        16: "POS_HORIZ_ABS",
        32: "POS_VERT_ABS",
        64: "POS_VERT_AGL",
        128: "CONST_POS_MODE",
        256: "PRED_POS_HORIZ_REL",
        512: "PRED_POS_HORIZ_ABS",
        1024: "UNINITIALIZED",
        2048: "GPS_GLITCHING",
    }
    return [n for b, n in names.items() if flags & b]


def stats(vals, fmt=".4g"):
    if not vals:
        return "n/a"
    return f"n={len(vals)} min={min(vals):{fmt}} max={max(vals):{fmt}} last={vals[-1]:{fmt}}"


def main():
    lines = []

    def out(s=""):
        lines.append(s)
        print(s)

    chunks = parse_dump(SRC)
    stream = b"".join(b for _, b in chunks)
    out(f"chunks={len(chunks)} bytes={len(stream)}")
    out(f"time {chunks[0][0]} -> {chunks[-1][0]}")
    out(f"chunk sizes: {[len(b) for _, b in chunks]}")
    frames = scan_frames(stream)
    decoded_n = sum(1 for f in frames if f["msg"] is not None)
    out(f"valid_crc_frames={len(frames)} pymavlink_ok={decoded_n}")
    names = Counter(f["name"] for f in frames)
    out("\n=== message counts ===")
    for k, v in names.most_common():
        out(f"  {k:28s} {v}")

    seqs = [f["seq"] for f in frames]
    gaps = []
    for a, b in zip(seqs, seqs[1:]):
        expect = (a + 1) & 0xFF
        if b != expect:
            lost = (b - expect) & 0xFF
            if 0 < lost < 128:
                gaps.append((a, b, lost))
    lost_total = sum(g[2] for g in gaps)
    out(f"\n=== seq first={seqs[0]} last={seqs[-1]} n={len(seqs)} gaps={len(gaps)} lost_est={lost_total}")
    for g in gaps:
        out(f"  gap {g[0]}->{g[1]} lost {g[2]}")
    out(f"sys/comp: {Counter((f['sysid'], f['compid']) for f in frames)}")

    def of(name):
        return [f for f in frames if f["name"] == name and f["msg"] is not None]

    findings = []

    out("\n========== HEARTBEAT ==========")
    for f in of("HEARTBEAT"):
        m = f["msg"]
        out(
            f"  seq={f['seq']} type={m.type} ap={m.autopilot} "
            f"base=0x{m.base_mode:02X} {mode_name(m.custom_mode)} "
            f"armed={bool(m.base_mode & 128)} status={m.system_status} ver={m.mavlink_version}"
        )
    if len(of("HEARTBEAT")) == 0:
        findings.append("约 5 秒窗口内没有 HEARTBEAT（正常约 1Hz）。地面站会判失联。")

    out("\n========== SYS_STATUS ==========")
    for f in of("SYS_STATUS"):
        m = f["msg"]
        unhealthy = m.onboard_control_sensors_present & m.onboard_control_sensors_enabled & ~m.onboard_control_sensors_health
        extra_health = m.onboard_control_sensors_health & ~m.onboard_control_sensors_present
        present_off = m.onboard_control_sensors_present & ~m.onboard_control_sensors_enabled
        out(
            f"  seq={f['seq']} load={m.load/10:.1f}% Vbat={m.voltage_battery/1000:.3f}V "
            f"I={m.current_battery/100:.2f}A remain={m.battery_remaining}% "
            f"drop={m.drop_rate_comm} err_comm={m.errors_comm} "
            f"e1={m.errors_count1} e2={m.errors_count2} e3={m.errors_count3} e4={m.errors_count4}"
        )
        if unhealthy:
            out(f"    UNHEALTHY {sensor_bits(unhealthy)} 0x{unhealthy:08X}")
            findings.append(f"SYS_STATUS 不健康: {sensor_bits(unhealthy)}")
        if present_off:
            out(f"    present-disabled {sensor_bits(present_off)}")
        if extra_health:
            out(f"    health-not-present {sensor_bits(extra_health)}")
    if of("SYS_STATUS"):
        m = of("SYS_STATUS")[0]["msg"]
        out(f"  present: {sensor_bits(m.onboard_control_sensors_present)}")
        out(f"  enabled: {sensor_bits(m.onboard_control_sensors_enabled)}")
        out(f"  health : {sensor_bits(m.onboard_control_sensors_health)}")
        vbats = [f["msg"].voltage_battery / 1000 for f in of("SYS_STATUS")]
        loads = [f["msg"].load / 10 for f in of("SYS_STATUS")]
        remains = [f["msg"].battery_remaining for f in of("SYS_STATUS")]
        out(f"  Vbat {stats(vbats)}")
        out(f"  load {stats(loads)}")
        out(f"  remain {stats(remains)}")
        if min(vbats) < 21:
            findings.append(f"SYS_STATUS 电池电压 {min(vbats):.2f}–{max(vbats):.2f}V，按 6S 偏低，按 3S 则接近空电")
        if any(r >= 0 and r < 20 for r in remains):
            findings.append(f"电池剩余电量上报 {remains[0]}%")

    out("\n========== GPS / POS ==========")
    for f in of("GPS_RAW_INT"):
        m = f["msg"]
        out(
            f"  GPS seq={f['seq']} fix={m.fix_type} sats={m.satellites_visible} "
            f"lat={m.lat/1e7:.7f} lon={m.lon/1e7:.7f} alt={m.alt/1000:.2f}m "
            f"eph={m.eph} epv={m.epv} vel={m.vel} cog={m.cog} "
            f"h_acc={m.h_acc}mm v_acc={m.v_acc}mm"
        )
        if m.fix_type < 3:
            findings.append(f"GPS fix={m.fix_type} 未 3D 定位")
        if m.satellites_visible < 10:
            findings.append(f"GPS 星数 {m.satellites_visible}")
        if m.eph not in (0, 65535) and m.eph > 250:
            findings.append(f"GPS HDOP 差 eph={m.eph}")
        if m.h_acc and m.h_acc > 2500:
            findings.append(f"GPS 水平精度 {m.h_acc}mm")
    for f in of("GLOBAL_POSITION_INT"):
        m = f["msg"]
        out(
            f"  GLOBAL seq={f['seq']} t={m.time_boot_ms} lat={m.lat/1e7:.7f} lon={m.lon/1e7:.7f} "
            f"alt={m.alt/1000:.2f} rel={m.relative_alt/1000:.2f} "
            f"v=({m.vx},{m.vy},{m.vz})cm/s hdg={m.hdg/100:.1f}"
        )
    for f in of("LOCAL_POSITION_NED"):
        m = f["msg"]
        out(
            f"  NED seq={f['seq']} t={m.time_boot_ms} "
            f"n={m.x:.3f} e={m.y:.3f} d={m.z:.3f} "
            f"v=({m.vx:.3f},{m.vy:.3f},{m.vz:.3f})"
        )

    out("\n========== ATTITUDE / HUD / AHRS ==========")
    for f in of("ATTITUDE"):
        m = f["msg"]
        out(
            f"  ATT seq={f['seq']} t={m.time_boot_ms} "
            f"r={m.roll*57.2958:.2f} p={m.pitch*57.2958:.2f} y={m.yaw*57.2958:.2f} "
            f"rates={m.rollspeed:.3f}/{m.pitchspeed:.3f}/{m.yawspeed:.3f}"
        )
    for f in of("AHRS2"):
        m = f["msg"]
        out(
            f"  AHRS2 seq={f['seq']} r={m.roll*57.2958:.2f} p={m.pitch*57.2958:.2f} y={m.yaw*57.2958:.2f} "
            f"alt={m.altitude:.2f} lat={m.lat/1e7:.7f} lon={m.lng/1e7:.7f}"
        )
    for f in of("VFR_HUD"):
        m = f["msg"]
        out(
            f"  HUD seq={f['seq']} as={m.airspeed:.2f} gs={m.groundspeed:.2f} "
            f"hdg={m.heading} thr={m.throttle} alt={m.alt:.2f} climb={m.climb:.3f}"
        )
    for f in of("AHRS"):
        m = f["msg"]
        out(
            f"  AHRS seq={f['seq']} omegaI={m.omegaIx:.5f}/{m.omegaIy:.5f}/{m.omegaIz:.5f} "
            f"err_rp={m.error_rp:.5f} err_yaw={m.error_yaw:.5f} renorm={m.renorm_val:.4f}"
        )

    out("\n========== EKF / VIBE / IMU ==========")
    for f in of("EKF_STATUS_REPORT"):
        m = f["msg"]
        flags = m.flags
        out(
            f"  EKF seq={f['seq']} flags=0x{flags:X} {ekf_flag_names(flags)} "
            f"vel={m.velocity_variance:.3f} posh={m.pos_horiz_variance:.3f} "
            f"posv={m.pos_vert_variance:.3f} mag={m.compass_variance:.3f} "
            f"terrain={m.terrain_alt_variance:.3f} aspd={getattr(m,'airspeed_variance',0):.3f}"
        )
        if flags & 2048:
            findings.append("EKF 报 GPS_GLITCHING")
        if flags & 1024:
            findings.append("EKF UNINITIALIZED")
        if flags & 128:
            findings.append("EKF CONST_POS_MODE（没用 GPS 位置）")
        needed = 1 | 2 | 4 | 16 | 32
        if (flags & needed) != needed:
            findings.append(f"EKF 关键标志不全 0x{flags:X} {ekf_flag_names(flags)}")
        if m.velocity_variance > 0.8:
            findings.append(f"EKF 速度方差高 {m.velocity_variance:.3f}")
        if m.pos_horiz_variance > 0.8:
            findings.append(f"EKF 水平位置方差高 {m.pos_horiz_variance:.3f}")
        if m.compass_variance > 0.8:
            findings.append(f"EKF 罗盘方差高 {m.compass_variance:.3f}")
    for f in of("VIBRATION"):
        m = f["msg"]
        out(
            f"  VIBE seq={f['seq']} {m.vibration_x:.3f}/{m.vibration_y:.3f}/{m.vibration_z:.3f} "
            f"clip={m.clipping_0}/{m.clipping_1}/{m.clipping_2}"
        )
        if max(m.vibration_x, m.vibration_y, m.vibration_z) > 30:
            findings.append(
                f"振动偏高 {m.vibration_x:.2f}/{m.vibration_y:.2f}/{m.vibration_z:.2f}"
            )
        if m.clipping_0 or m.clipping_1 or m.clipping_2:
            findings.append(f"IMU 削波 {m.clipping_0}/{m.clipping_1}/{m.clipping_2}")
    for f in of("RAW_IMU"):
        m = f["msg"]
        acc = math.sqrt(m.xacc**2 + m.yacc**2 + m.zacc**2)
        mag = math.sqrt(m.xmag**2 + m.ymag**2 + m.zmag**2)
        out(
            f"  IMU seq={f['seq']} acc=({m.xacc},{m.yacc},{m.zacc}) |a|={acc:.0f} "
            f"gyro=({m.xgyro},{m.ygyro},{m.zgyro}) mag=({m.xmag},{m.ymag},{m.zmag}) |m|={mag:.0f} "
            f"id={m.id} temp={m.temperature}"
        )

    out("\n========== POWER / BAT / MCU / MEM ==========")
    for f in of("POWER_STATUS"):
        m = f["msg"]
        out(f"  PWR seq={f['seq']} Vcc={m.Vcc}mV Vservo={m.Vservo}mV flags=0x{m.flags:X}")
        if m.Vcc == 0:
            findings.append("POWER_STATUS.Vcc=0（板载 5V 监测未配置或读数为 0）")
        elif m.Vcc < 4500:
            findings.append(f"板载 Vcc 偏低 {m.Vcc}mV")
    for f in of("BATTERY_STATUS"):
        m = f["msg"]
        cells = [v for v in m.voltages if v not in (0, 65535)]
        out(
            f"  BAT seq={f['seq']} id={m.id} func={m.battery_function} type={m.type} "
            f"temp={m.temperature} cells_mV={cells} I={m.current_battery} "
            f"consumed={m.current_consumed} remain={m.battery_remaining} "
            f"t_rem={m.time_remaining} state={m.charge_state}"
        )
        if cells and min(cells) < 3200:
            findings.append(f"单体电压偏低 {cells}")
    for f in of("MCU_STATUS"):
        m = f["msg"]
        out(
            f"  MCU seq={f['seq']} id={m.id} T={m.MCU_temperature/100:.2f}C "
            f"V={m.MCU_voltage} min={m.MCU_voltage_min} max={m.MCU_voltage_max}"
        )
        if m.MCU_temperature > 8000:
            findings.append(f"MCU 温度高 {m.MCU_temperature/100:.1f}C")
        if 0 < m.MCU_voltage < 3000:
            findings.append(f"MCU 电压低 {m.MCU_voltage}mV")
    for f in of("MEMINFO"):
        m = f["msg"]
        out(f"  MEM seq={f['seq']} brkval={m.brkval} free16={m.freemem} free32={m.freemem32}")

    out("\n========== RC / SERVO / NAV / BARO / TIME ==========")
    for f in of("RC_CHANNELS"):
        m = f["msg"]
        ch = [m.chan1_raw, m.chan2_raw, m.chan3_raw, m.chan4_raw, m.chan5_raw, m.chan6_raw,
              m.chan7_raw, m.chan8_raw, m.chan9_raw, m.chan10_raw, m.chan11_raw, m.chan12_raw]
        out(f"  RC seq={f['seq']} n={m.chancount} rssi={m.rssi} ch={ch}")
        if m.chancount == 0 or all(c == 0 for c in ch[:4]):
            findings.append("遥控通道无效/全 0")
        if any(c != 0 and (c < 800 or c > 2200) for c in ch[:8]):
            findings.append(f"遥控 PWM 超范围 {ch[:8]}")
    for f in of("SERVO_OUTPUT_RAW"):
        m = f["msg"]
        servos = [m.servo1_raw, m.servo2_raw, m.servo3_raw, m.servo4_raw,
                  m.servo5_raw, m.servo6_raw, m.servo7_raw, m.servo8_raw]
        out(f"  SERVO seq={f['seq']} port={m.port} {servos}")
    for f in of("NAV_CONTROLLER_OUTPUT"):
        m = f["msg"]
        out(
            f"  NAV seq={f['seq']} roll={m.nav_roll:.2f} pitch={m.nav_pitch:.2f} "
            f"bear={m.nav_bearing} tgt={m.target_bearing} dist={m.wp_dist} "
            f"alt_err={m.alt_error:.2f} aspd_err={m.aspd_error:.2f} xtrack={m.xtrack_error:.3f}"
        )
    for f in of("MISSION_CURRENT"):
        m = f["msg"]
        out(f"  MISSION seq={f['seq']} wp={m.seq} total={getattr(m,'total',None)} state={getattr(m,'mission_state',None)}")
    for f in of("SCALED_PRESSURE"):
        m = f["msg"]
        out(
            f"  BARO seq={f['seq']} t={m.time_boot_ms} P={m.press_abs:.2f}hPa "
            f"dP={m.press_diff:.3f} T={m.temperature/100:.2f}C"
        )
    for f in of("SYSTEM_TIME"):
        m = f["msg"]
        out(f"  TIME seq={f['seq']} unix_us={m.time_unix_usec} boot_ms={m.time_boot_ms}")
    for f in of("TERRAIN_REPORT"):
        m = f["msg"]
        out(
            f"  TERRAIN seq={f['seq']} lat={m.lat/1e7:.7f} lon={m.lon/1e7:.7f} "
            f"spacing={m.spacing} terrain={m.terrain_height:.2f} cur={m.current_height:.2f} "
            f"pending={m.pending} loaded={m.loaded}"
        )
    for f in of("WIND"):
        m = f["msg"]
        out(f"  WIND seq={f['seq']} dir={m.direction:.1f} spd={m.speed:.2f} z={m.speed_z:.2f}")

    # ranges
    if of("VFR_HUD"):
        out(f"\nHUD thr {stats([f['msg'].throttle for f in of('VFR_HUD')])}")
        out(f"HUD alt {stats([f['msg'].alt for f in of('VFR_HUD')])}")
        out(f"HUD climb {stats([f['msg'].climb for f in of('VFR_HUD')])}")
        out(f"HUD gs {stats([f['msg'].groundspeed for f in of('VFR_HUD')])}")
    if of("ATTITUDE"):
        rolls = [abs(f["msg"].roll) * 57.2958 for f in of("ATTITUDE")]
        pitches = [abs(f["msg"].pitch) * 57.2958 for f in of("ATTITUDE")]
        out(f"ATT |roll| {stats(rolls)}")
        out(f"ATT |pitch| {stats(pitches)}")
    if of("SYSTEM_TIME"):
        boots = [f["msg"].time_boot_ms for f in of("SYSTEM_TIME")]
        unixs = [f["msg"].time_unix_usec for f in of("SYSTEM_TIME")]
        out(f"boot_ms {stats(boots, '.0f')} span={boots[-1]-boots[0]}ms")
        if unixs[0] < 1e12:
            findings.append("SYSTEM_TIME 无有效 UTC（unix 过小）")

    if of("GPS_RAW_INT") and of("GLOBAL_POSITION_INT"):
        g = of("GPS_RAW_INT")[0]["msg"]
        p = of("GLOBAL_POSITION_INT")[0]["msg"]
        dlat = abs(g.lat - p.lat) / 1e7 * 1.11e5
        dlon = abs(g.lon - p.lon) / 1e7 * 1.11e5 * abs(math.cos(math.radians(p.lat / 1e7)))
        out(f"GPS vs GLOBAL ~ {dlat:.1f}m N, {dlon:.1f}m E")
        if dlat > 15 or dlon > 15:
            findings.append(f"GPS 与融合位置相差约北{dlat:.0f}m 东{dlon:.0f}m")

    if lost_total > 5:
        findings.append(f"序号缺口约丢 {lost_total} 包（{100*lost_total/(len(seqs)+lost_total):.1f}%）")

    # unique findings
    out("\n\n========== ANOMALY SUMMARY ==========")
    seen = set()
    uniq = []
    for x in findings:
        if x not in seen:
            seen.add(x)
            uniq.append(x)
    if not uniq:
        out("未发现明显协议/状态异常。")
    else:
        for i, x in enumerate(uniq, 1):
            out(f"{i}. {x}")

    OUT.write_text("\n".join(lines), encoding="utf-8")
    print(f"\nWrote {OUT}")


if __name__ == "__main__":
    main()
