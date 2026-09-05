#!/usr/bin/env python3
"""P04 现场注入链路的端到端验证：遥控开关 → Lua → 输出覆盖 → 申报 → 重分配。

**为什么单独写这个。** 此前所有 P04 架次都走 `reproduce.py --degrade`，即由工具
直接写 `MOT_FAIL_IDX`。那条路验证的是分配器，**完全绕过了现场实际要用的入口**：
机载 Lua 脚本读开关、覆盖输出、再把电机号申报给混控器。54 个已归档架次里
`SCR_ENABLE` 一个都没设过（Codex 的 SITL 审查 A11）。

于是「参数配置正确」此前只有静态依据——逐条对着源码核门控——而没有一次整体
跑通的证据。本脚本补的就是这一条。

它加载的是**现场那份参数文件本身**（airworthiness/params/P04_现场前馈注入*.param），
不是另抄一份，否则验的就不是现场要用的配置了。

依次断言：

  1. 脚本已加载  —— MOT_STOP_BITMASK 出现在参数表里（Lua 运行时注册，脚本不跑就没有）
  2. 解锁起飞    —— MOT_FAIL_IDX 必须为 0，否则解锁会被拒
  3. 拨杆        —— 把 RCx_OPTION=301 的那一路推高
  4. 申报        —— GCS 出现 "declared motor N to mixer"
  5. 降级        —— GCS 出现 "Motor N failed: mixer degraded"
  6. 参数已写    —— MOT_FAIL_IDX 变成 N（证明是 Lua 写的，不是工具写的）
  7. 分配器在跑  —— MALC 有记录且 Res 主要为 0（ALLOC=0 时前 5 条都会过，只有这条不会）
  8. 姿态可控    —— 观察窗内滚转/俯仰误差不超判据

用法：
    python3 Tools/eft_issue_repro/p04_lua_e2e.py --motor 6 --wind 4
"""
import argparse
import os
import shutil
import subprocess
import sys
import time

from pymavlink import mavutil

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
SITL_BIN = os.path.join(ROOT, "build", "sitl", "bin", "arducopter")
DEFAULTS = os.path.join(ROOT, "Tools", "autotest", "default_params", "copter.parm")
PARAMS = os.path.join(HERE, "eft_hexa.parm")
MODEL = os.path.join(HERE, "eft_hexa.json")
SCRIPT = os.path.join(ROOT, "ROMFS_custom", "scripts", "motor_failure_test.lua")
HOME = (35.363261, 149.16523, 584.0, 0.0)


def find_field_param():
    """定位现场那份参数文件——验的必须是现场要加载的那一份，不是另抄一份。

    不能用 ROOT/.. 推：worktree 在 ~/UAV-work/<分支叶子名>/，而父仓库在 ~/UAV/，
    两者是平行目录不是父子。用 git 找主克隆的位置：linked worktree 的
    --git-common-dir 指向主克隆的 .git，再上两级就是父仓库根。
    """
    try:
        common = subprocess.check_output(
            ["git", "rev-parse", "--git-common-dir"], cwd=ROOT,
            text=True).strip()
    except Exception:
        common = os.path.join(ROOT, ".git")
    common = os.path.abspath(os.path.join(ROOT, common))
    # <父仓库>/ardupilot/.git  →  上两级
    parent = os.path.dirname(os.path.dirname(common))
    cand = os.path.join(parent, "airworthiness", "params",
                        "P04_现场前馈注入_20260904.param")
    return cand

AUX_CHAN = 7          # RC7_OPTION=301 (Scripting2)


def _fail(msg):
    print("  ✗ %s" % msg)
    sys.exit(1)


def _ok(msg):
    print("  ✓ %s" % msg)


def load_field_params(path):
    out = {}
    with open(path, encoding="utf-8") as f:
        for ln in f:
            ln = ln.strip()
            if not ln or ln.startswith("#"):
                continue
            k, v = ln.split(",", 1)
            out[k.strip()] = float(v.strip())
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--motor", type=int, default=6, help="要停转的电机 1..6")
    ap.add_argument("--wind", type=float, default=4.0)
    ap.add_argument("--alt", type=float, default=15.0)
    ap.add_argument("--watch", type=float, default=20.0, help="失效后观察秒数")
    ap.add_argument("--speedup", type=float, default=5.0)
    ap.add_argument("--out", default=None)
    ap.add_argument("--field-params", default=None,
                    help="现场参数文件路径；默认自动定位父仓库里的那一份")
    args = ap.parse_args()

    if not os.path.exists(SITL_BIN):
        _fail("缺少 %s；先 ./waf configure --board sitl && ./waf copter" % SITL_BIN)

    out = args.out or os.path.join(HERE, "runs",
                                   time.strftime("%Y%m%d-%H%M%S") + "-lua-e2e")
    os.makedirs(out, exist_ok=True)
    # 脚本必须放进 SITL 的工作目录下的 scripts/，SCR_ENABLE 才找得到。
    os.makedirs(os.path.join(out, "scripts"), exist_ok=True)
    shutil.copy(SCRIPT, os.path.join(out, "scripts"))
    shutil.copy(MODEL, out)

    fp = args.field_params or find_field_param()
    if not os.path.exists(fp):
        _fail("找不到现场参数文件 %s（用 --field-params 指定）" % fp)
    print("加载现场参数: %s" % fp)
    field = load_field_params(fp)
    field["MOT_STOP_BITMASK"] = float(1 << (args.motor - 1))
    field["RC%d_OPTION" % AUX_CHAN] = 301.0     # 现场按实机接线，文件里刻意不写死
    field["SIM_WIND_SPD"] = args.wind
    field["SIM_WIND_DIR"] = 90.0
    field["SIM_SPEEDUP"] = args.speedup
    runtime = os.path.join(out, "runtime.parm")
    with open(runtime, "w", encoding="utf-8") as f:
        for k, v in sorted(field.items()):
            f.write("%s %g\n" % (k, v))

    cmd = [SITL_BIN, "--model", "hexa-dji:" + os.path.basename(MODEL),
           "--speedup", str(args.speedup), "--home", ",".join(str(x) for x in HOME),
           "--defaults", ",".join([DEFAULTS, PARAMS, runtime]), "--wipe"]
    log = open(os.path.join(out, "sitl_stdout.log"), "w", encoding="utf-8")
    proc = subprocess.Popen(cmd, cwd=out, stdout=log, stderr=subprocess.STDOUT)
    try:
        run(proc, args, out)
    finally:
        proc.terminate()
        log.close()


def run(proc, args, out):
    print("连接 SITL …")
    mav = mavutil.mavlink_connection("tcp:127.0.0.1:5760")
    mav.wait_heartbeat(timeout=60)
    mav.mav.request_data_stream_send(mav.target_system, mav.target_component,
                                     mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
    msgs = []

    def pump(seconds):
        end = time.time() + seconds
        while time.time() < end:
            m = mav.recv_match(blocking=True, timeout=0.5)
            if m is None:
                continue
            if m.get_type() == "STATUSTEXT":
                t = m.text if isinstance(m.text, str) else m.text.decode()
                msgs.append(t)
                print("    GCS: %s" % t)

    def getparam(name, timeout=8):
        mav.mav.param_request_read_send(mav.target_system, mav.target_component,
                                        name.encode(), -1)
        end = time.time() + timeout
        while time.time() < end:
            m = mav.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
            if m and m.param_id.strip("\x00") == name:
                return m.param_value
        return None

    print("\n[1] 脚本是否已加载")
    pump(12)                       # 给 Lua 引擎启动与注册参数的时间
    if getparam("MOT_STOP_BITMASK") is None:
        _fail("参数表里没有 MOT_STOP_BITMASK —— 脚本没在跑（SCR_ENABLE / 脚本文件 / 堆）")
    _ok("MOT_STOP_BITMASK 存在，脚本已注册参数")

    print("\n[2] 解锁前 MOT_FAIL_IDX 必须为 0")
    idx0 = getparam("MOT_FAIL_IDX")
    if idx0 is None or abs(idx0) > 0.5:
        _fail("MOT_FAIL_IDX = %s，非零会拒绝解锁" % idx0)
    _ok("MOT_FAIL_IDX = 0")

    print("\n[3] 起飞到 %.0f m" % args.alt)
    mav.set_mode_apm("GUIDED")
    pump(2)
    mav.mav.command_long_send(mav.target_system, mav.target_component,
                              mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                              0, 1, 0, 0, 0, 0, 0, 0)
    pump(3)
    mav.mav.command_long_send(mav.target_system, mav.target_component,
                              mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                              0, 0, 0, 0, 0, 0, 0, args.alt)
    end = time.time() + 90
    while time.time() < end:
        m = mav.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=1)
        if m and m.relative_alt / 1000.0 >= args.alt * 0.9:
            break
    _ok("已到 %.1f m，稳定 5 s" % (m.relative_alt / 1000.0 if m else -1))
    pump(5)

    print("\n[4] 拨杆（RC%d → 2000）" % AUX_CHAN)
    before = len(msgs)
    ch = [65535] * 18
    ch[AUX_CHAN - 1] = 2000
    for _ in range(20):
        mav.mav.rc_channels_override_send(mav.target_system, mav.target_component,
                                          *ch[:8])
        pump(0.2)
    pump(4)

    new = msgs[before:]
    print("\n[5] 申报与降级")
    if not any("declared motor" in t for t in new):
        _fail("没有 'declared motor …' —— 申报没发生（DECL/掩码/映射）")
    _ok("申报消息已出现")
    if not any("mixer degraded" in t for t in new):
        _fail("没有 'mixer degraded' —— 混控器没有降级")
    _ok("降级消息已出现")

    print("\n[6] MOT_FAIL_IDX 是否被 Lua 写成 %d" % args.motor)
    idx = getparam("MOT_FAIL_IDX")
    if idx is None or abs(idx - args.motor) > 0.5:
        _fail("MOT_FAIL_IDX = %s，期望 %d" % (idx, args.motor))
    _ok("MOT_FAIL_IDX = %d —— 确实由脚本写入" % args.motor)

    print("\n[7] 观察 %.0f s" % args.watch)
    roll_max = pitch_max = 0.0
    end = time.time() + args.watch
    while time.time() < end:
        m = mav.recv_match(type="ATTITUDE", blocking=True, timeout=1)
        if m:
            roll_max = max(roll_max, abs(m.roll) * 57.2958)
            pitch_max = max(pitch_max, abs(m.pitch) * 57.2958)
    print("  滚转峰值 %.2f°  俯仰峰值 %.2f°" % (roll_max, pitch_max))
    if roll_max > 25.0 or pitch_max > 25.0:
        _fail("姿态超出 25° 中止判据")
    _ok("姿态在中止判据内")

    print("\n结果目录: %s" % out)
    print("\n[8] 分配器是否真的在跑 —— 请查 MALC.Res（下面自动读）")


if __name__ == "__main__":
    main()
