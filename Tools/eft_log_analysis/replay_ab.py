#!/usr/bin/env python3
"""同一份飞行日志，用两组参数各重放一遍 EKF，逐点对比结果。

用法:
    replay_ab.py <log.bin> --a EK3_BARO_HDOP=3.0 --b EK3_BARO_HDOP=0
    replay_ab.py <log.bin> --a EK3_SRC1_POSZ=1 --b EK3_SRC1_POSZ=3 --field XKF1.PD

这是真正的反事实对比: 输入完全相同(同一份录下来的传感器时序)，只有
参数不同，所以差异百分之百来自那个参数。机上跑的影子滤波器做不到这点,
因为影子和主滤波器共享状态、且改不了"当时实际融合了什么"。

前提: 日志必须带 Replay 数据。先用 check_replay_ready.py 确认。

改了 EKF 代码想验证时同样适用: 分别用改前/改后的固件编译 Replay，
两次跑同一份日志。
"""
import sys
import os
import shutil
import subprocess
import argparse
import math

_HERE = os.path.dirname(os.path.abspath(__file__))
_AP = os.path.normpath(os.path.join(_HERE, os.pardir, os.pardir))
if os.path.isdir(os.path.join(_AP, "modules", "mavlink")):
    sys.path.insert(0, os.path.join(_AP, "modules", "mavlink"))

from pymavlink import DFReader  # noqa: E402

REPLAY_BIN = os.path.join(_AP, "build", "sitl", "tool", "Replay")

# 默认对比 EKF3 的位置与姿态输出
DEFAULT_FIELDS = ["XKF1.PN", "XKF1.PE", "XKF1.PD",
                  "XKF1.Roll", "XKF1.Pitch", "XKF1.Yaw"]

# 帧骨架，缺了就不是 replay 日志
CORE_MSGS = ("RFRH", "RFRF", "RISH", "RISI")


def check_ready(path):
    """确认日志真的带 replay 记录。按条数查，不是搜字符串——日志开头的
    FMT 表会声明固件认识的全部类型，grep 在任何日志上都会命中。"""
    log = DFReader.DFReader_binary(path)
    n = 0
    while n == 0:
        m = log.recv_match(type=list(CORE_MSGS))
        if m is None:
            break
        n += 1
    return n > 0


def run_replay(log, parms, workdir, label):
    os.makedirs(workdir, exist_ok=True)
    for stale in ("logs",):
        shutil.rmtree(os.path.join(workdir, stale), ignore_errors=True)

    cmd = [REPLAY_BIN]
    for p in parms:
        cmd += ["--parm", p]
    cmd.append(os.path.abspath(log))

    with open(os.path.join(workdir, "replay.log"), "w") as f:
        r = subprocess.run(cmd, cwd=workdir, stdout=f, stderr=subprocess.STDOUT)
    if r.returncode != 0:
        raise SystemExit("[%s] Replay 退出码 %d，详见 %s/replay.log"
                         % (label, r.returncode, workdir))

    out = []
    for root, _, files in os.walk(os.path.join(workdir, "logs")):
        for fn in files:
            if fn.endswith((".BIN", ".EFT")):
                out.append(os.path.join(root, fn))
    if not out:
        raise SystemExit("[%s] Replay 没有产出日志" % label)
    return sorted(out)[-1]


def series(path, fields):
    """按 (消息, 时间戳) 取值，便于两次重放逐点对齐。"""
    want = {}
    for f in fields:
        msg, fld = f.split(".", 1)
        want.setdefault(msg, []).append(fld)

    log = DFReader.DFReader_binary(path)
    data = {f: {} for f in fields}
    while True:
        m = log.recv_match(type=list(want))
        if m is None:
            break
        # 多核 EKF 只取 core 0，否则不同核会互相污染统计
        if getattr(m, "C", 0) != 0:
            continue
        t = m.TimeUS
        for fld in want[m.get_type()]:
            v = getattr(m, fld, None)
            if v is not None:
                data["%s.%s" % (m.get_type(), fld)][t] = v
    return data


def compare(a, b, fields):
    rows = []
    for f in fields:
        ta, tb = a.get(f, {}), b.get(f, {})
        common = sorted(set(ta) & set(tb))
        if not common:
            rows.append((f, 0, None, None, None))
            continue
        d = [ta[t] - tb[t] for t in common]
        n = len(d)
        mu = sum(d) / n
        rms = math.sqrt(sum(x * x for x in d) / n)
        mx = max(abs(x) for x in d)
        rows.append((f, n, rms, mx, mu))
    return rows


def main(argv):
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log")
    ap.add_argument("--a", action="append", default=[],
                    metavar="NAME=VALUE", help="A 组参数覆盖，可重复")
    ap.add_argument("--b", action="append", default=[],
                    metavar="NAME=VALUE", help="B 组参数覆盖，可重复")
    ap.add_argument("--field", action="append", default=None,
                    metavar="MSG.FIELD", help="要对比的字段，可重复")
    ap.add_argument("-o", "--out", default="replay_ab", help="工作目录")
    a = ap.parse_args(argv)

    if not os.path.exists(a.log):
        print("找不到日志: " + a.log, file=sys.stderr)
        return 1
    if not os.path.exists(REPLAY_BIN):
        print("Replay 未编译。先跑:\n"
              "  ./waf configure --board sitl && ./waf replay\n"
              "  ./waf configure --board EFT_CAAC   # 记得切回来", file=sys.stderr)
        return 1
    if not check_ready(a.log):
        print("这份日志没有 Replay 数据（RFRH/RFRF/RISH/RISI 一条都没有）。\n"
              "飞行时需要 LOG_REPLAY=1 且 LOG_DISARMED=1。\n"
              "详细诊断: python3 %s/check_replay_ready.py %s"
              % (_HERE, a.log), file=sys.stderr)
        return 1

    fields = a.field or DEFAULT_FIELDS
    print("日志: %s" % os.path.basename(a.log))
    print("A 组: %s" % (", ".join(a.a) or "(不改参数，即当时的设置)"))
    print("B 组: %s\n" % (", ".join(a.b) or "(不改参数，即当时的设置)"))

    la = run_replay(a.log, a.a, os.path.join(a.out, "A"), "A")
    lb = run_replay(a.log, a.b, os.path.join(a.out, "B"), "B")
    print("A 重放输出: %s" % la)
    print("B 重放输出: %s\n" % lb)

    rows = compare(series(la, fields), series(lb, fields), fields)

    print("%-14s %8s %12s %12s %12s" % ("字段", "样本", "RMS差", "最大差", "均值差"))
    print("-" * 62)
    for f, n, rms, mx, mu in rows:
        if not n:
            print("%-14s %8s   (两次重放没有共同时间点)" % (f, 0))
        else:
            print("%-14s %8d %12.5f %12.5f %12.5f" % (f, n, rms, mx, mu))

    if all(r[1] and r[2] == 0.0 for r in rows if r[1]):
        print("\n两组结果完全一致——参数没有生效，或对这份数据无影响。")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
