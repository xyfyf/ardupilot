#!/usr/bin/env python3
"""把一份真实飞行日志拆成 SITL 可复现的场景，用于整机闭环重跑。

用法:
    python3 log_to_sitl_scenario.py <log.bin> [-o 输出目录]

产出:
    params.parm      当时的全部参数，SITL 启动时加载
    home.txt         EKF 原点，作为 sim_vehicle 的 --custom-location
    timeline.csv     模式切换与飞行员杆量的时间线
    mission.txt      任务航点（QGC WPL 格式，若日志里有）
    scenario.json    汇总，供驱动脚本读取
    run_sitl.sh      拼好的启动命令

这不是"重放传感器数据"。控制器无法离线在环重放——日志记的是当时那套
控制律产生的轨迹，换一套控制律飞机响应就变了，后续传感器读数不再成立。
这里做的是**场景重放**: 把输入侧(参数/起点/模式/杆量/航线)喂给 SITL，
让整个固件(EKF + 控制器 + 混控)在物理模型里真实闭环跑一遍。

仿真轨迹一定会和真实飞行发散。发散不是故障，是物理模型与实机的差异。
发散程度本身就是模型保真度的度量；要缩小它，用 SysID 模式扫频辨识出
本机型的模型，再配进 SITL 的 frame 参数或自定义 JSON 模型。
"""
import sys
import os
import json
import argparse

_AP = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   os.pardir, os.pardir, "modules", "mavlink")
_AP = os.path.normpath(_AP)
if os.path.isdir(_AP) and _AP not in sys.path:
    sys.path.insert(0, _AP)

from pymavlink import DFReader  # noqa: E402

# 这些参数描述的是这块飞控的硬件，搬进 SITL 只会让它去找不存在的设备
HW_PREFIXES = (
    "CAN_", "SERIAL", "BRD_", "INS_", "COMPASS_", "BARO_", "GPS_", "BATT",
    "RNGFND", "ESC_", "SERVO_BLH", "NET_", "MSP", "EFI", "OSD", "CAM_",
    "LOG_", "SCR_", "RC_PROTOCOLS", "SPRAY", "RELAY", "NTF_", "SIM_",
)

# 主要的驾驶通道，其余通道多为开关，采样一次即可
STICK_CH = ("C1", "C2", "C3", "C4")


def extract(path):
    log = DFReader.DFReader_binary(path)

    params = {}
    origin = None
    modes = []
    mission = {}
    rc = []
    pos0 = None
    yaw0 = None
    t_first = t_last = None

    while True:
        m = log.recv_match(type=["PARM", "ORGN", "MODE", "CMD", "RCIN",
                                 "POS", "ATT"])
        if m is None:
            break
        t = m.TimeUS
        t_first = t if t_first is None else t_first
        t_last = t
        typ = m.get_type()

        if typ == "PARM":
            params[m.Name] = m.Value
        elif typ == "ORGN" and origin is None:
            origin = (m.Lat, m.Lng, m.Alt)
        elif typ == "POS" and pos0 is None:
            pos0 = (m.Lat, m.Lng, m.Alt)
        elif typ == "ATT" and yaw0 is None:
            yaw0 = m.Yaw
        elif typ == "MODE":
            modes.append({"t": t, "mode": int(m.ModeNum), "reason": int(m.Rsn)})
        elif typ == "CMD":
            # 同一航点可能记录多次，按序号去重保留最后一条
            mission[int(m.CNum)] = {
                "num": int(m.CNum), "id": int(m.CId), "frame": int(m.Frame),
                "p1": m.Prm1, "p2": m.Prm2, "p3": m.Prm3, "p4": m.Prm4,
                "lat": m.Lat, "lng": m.Lng, "alt": m.Alt,
            }
        elif typ == "RCIN":
            rc.append([t] + [int(getattr(m, c, 0)) for c in STICK_CH])

    return {
        "path": path,
        "params": params,
        "origin": origin or pos0,
        "yaw0": yaw0,
        "modes": modes,
        "mission": [mission[k] for k in sorted(mission)],
        "rc": rc,
        "t_first": t_first,
        "t_last": t_last,
    }


def write_params(d, out):
    """只写与飞行行为相关的参数，硬件相关的会让 SITL 去找不存在的设备。"""
    kept = skipped = 0
    p = os.path.join(out, "params.parm")
    with open(p, "w", encoding="utf-8") as f:
        f.write("# 从 %s 提取\n" % os.path.basename(d["path"]))
        f.write("# 已剔除硬件相关参数（%s ...）\n" % ", ".join(HW_PREFIXES[:6]))
        for name in sorted(d["params"]):
            if name.startswith(HW_PREFIXES):
                skipped += 1
                continue
            f.write("%-18s %s\n" % (name, repr(d["params"][name])))
            kept += 1
    return p, kept, skipped


def write_home(d, out):
    if not d["origin"]:
        return None
    lat, lng, alt = d["origin"]
    yaw = d["yaw0"] if d["yaw0"] is not None else 0.0
    p = os.path.join(out, "home.txt")
    with open(p, "w", encoding="utf-8") as f:
        f.write("%.7f,%.7f,%.2f,%.1f\n" % (lat, lng, alt, yaw))
    return p


def write_timeline(d, out):
    p = os.path.join(out, "timeline.csv")
    t0 = d["t_first"] or 0
    with open(p, "w", encoding="utf-8") as f:
        f.write("t_s,kind,a,b,c,d\n")
        for m in d["modes"]:
            f.write("%.3f,mode,%d,%d,,\n"
                    % ((m["t"] - t0) / 1e6, m["mode"], m["reason"]))
        for r in d["rc"]:
            f.write("%.3f,rc,%d,%d,%d,%d\n"
                    % ((r[0] - t0) / 1e6, r[1], r[2], r[3], r[4]))
    return p


def write_mission(d, out):
    if not d["mission"]:
        return None
    p = os.path.join(out, "mission.txt")
    with open(p, "w", encoding="utf-8") as f:
        f.write("QGC WPL 110\n")
        for i, c in enumerate(d["mission"]):
            # seq cur frame cmd p1 p2 p3 p4 x y z autocontinue
            f.write("%d\t%d\t%d\t%d\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t1\n"
                    % (c["num"], 1 if i == 0 else 0, c["frame"], c["id"],
                       c["p1"], c["p2"], c["p3"], c["p4"],
                       c["lat"], c["lng"], c["alt"]))
    return p


def write_runner(d, out, param_file, home_file):
    p = os.path.join(out, "run_sitl.sh")
    home = open(home_file, encoding="utf-8").read().strip() if home_file else ""
    frame = "hexa" if int(d["params"].get("FRAME_CLASS", 0)) == 2 else "quad"
    with open(p, "w", encoding="utf-8") as f:
        f.write("#!/bin/bash\n")
        f.write("# 由 log_to_sitl_scenario.py 生成\n")
        f.write("# 机架按 FRAME_CLASS=%s 推断为 %s\n"
                % (int(d["params"].get("FRAME_CLASS", 0)), frame))
        f.write("cd \"$(dirname \"$0\")\"\n")
        f.write("AP=%s\n" % os.path.normpath(
            os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         os.pardir, os.pardir)))
        f.write("python3 \"$AP/Tools/autotest/sim_vehicle.py\" \\\n")
        f.write("    -v ArduCopter -f %s \\\n" % frame)
        if home:
            f.write("    --custom-location=%s \\\n" % home)
        f.write("    --add-param-file=\"$PWD/params.parm\" \\\n")
        f.write("    --console --map\n")
    os.chmod(p, 0o755)
    return p


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log")
    ap.add_argument("-o", "--out", default=None, help="输出目录")
    a = ap.parse_args(argv)

    if not os.path.exists(a.log):
        print("找不到日志: " + a.log, file=sys.stderr)
        return 1

    out = a.out or (os.path.splitext(os.path.basename(a.log))[0] + "_scenario")
    os.makedirs(out, exist_ok=True)

    d = extract(a.log)
    pf, kept, skipped = write_params(d, out)
    hf = write_home(d, out)
    tf = write_timeline(d, out)
    mf = write_mission(d, out)
    rf = write_runner(d, out, pf, hf)

    with open(os.path.join(out, "scenario.json"), "w", encoding="utf-8") as f:
        json.dump({
            "source": os.path.basename(a.log),
            "origin": d["origin"], "yaw0": d["yaw0"],
            "duration_s": ((d["t_last"] or 0) - (d["t_first"] or 0)) / 1e6,
            "modes": d["modes"], "mission": d["mission"],
            "rc_samples": len(d["rc"]),
            "frame_class": d["params"].get("FRAME_CLASS"),
            "frame_type": d["params"].get("FRAME_TYPE"),
        }, f, indent=2, ensure_ascii=False)

    print("输出目录: %s\n" % out)
    print("参数      %s   保留 %d 条，剔除硬件相关 %d 条" % (pf, kept, skipped))
    print("起点      %s   %s" % (hf or "(无 ORGN/POS)",
                                open(hf, encoding='utf-8').read().strip() if hf else ""))
    print("时间线    %s   %d 次模式切换 + %d 组杆量"
          % (tf, len(d["modes"]), len(d["rc"])))
    print("任务      %s" % (mf or "(日志中无航点)"))
    print("启动脚本  %s" % rf)

    print("\n模式序列:")
    t0 = d["t_first"] or 0
    for m in d["modes"]:
        print("  %7.1f s   模式 %d   (原因码 %d)"
              % ((m["t"] - t0) / 1e6, m["mode"], m["reason"]))

    print("\n下一步:")
    print("  cd %s && ./run_sitl.sh" % out)
    print("\n注意: 仿真轨迹会和真实飞行发散，那是物理模型与实机的差异，不是故障。")
    print("      要缩小差异，用 SysID 模式扫频辨识本机型模型再配进 SITL。")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
