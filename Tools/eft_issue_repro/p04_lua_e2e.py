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
import io
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
    ap.add_argument("--mask", type=int, default=None,
                    help="直接指定 MOT_STOP_BITMASK，覆盖 --motor 推出的值。"
                         "用于验证非法掩码：多位时脚本必须**什么都不做**——"
                         "既不申报也不注入（A01）")
    ap.add_argument("--expect-blocked", action="store_true",
                    help="期望注入被拒：不得出现 declared/degraded，"
                         "MOT_FAIL_IDX 必须仍为 0，且必须出现 BLOCKED 告警")
    ap.add_argument("--debug-lua", action="store_true",
                    help="额外放一份带调试打印的脚本到 scripts/，用于定位链路断点。"
                         "注意它与 ROMFS 那份竞争 param 表，谁先注册谁赢——"
                         "是诊断手段，不是被测对象")
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
    # **默认不往 scripts/ 拷脚本。** ArduPilot 同时扫描文件系统的 scripts/ 与
    # @ROMFS/scripts，两处同名就会各加载一次；而脚本第 12 行的
    # param:add_table(75,...) 第二次调用会返回 false、assert 打死那一份。
    # 于是"哪一份在跑"取决于加载顺序，不可控。
    #
    # 真机上飞的是 ROMFS 那份，所以本测试就用它——测的必须是会飞的那份代码。
    #
    # 2026-09-05 实测这个坑：给拷贝注入了语法错误的调试代码，那份加载失败，
    # 而 ROMFS 那份照常注册了 MOT_STOP_BITMASK，于是第 1 步断言"通过"了。
    # 断言本身没错，错在它只能证明"有个脚本注册了参数"，不能证明是哪一份。
    # 真正的证据是第 6 步：MOT_FAIL_IDX 被写成 N。
    dst = os.path.join(out, "scripts", os.path.basename(SCRIPT))
    if args.debug_lua:
        shutil.copy(SCRIPT, dst)
        # 让脚本自己报它读到的开关位置。装在运行目录的那份拷贝上，不动 ROMFS，
        # 因此不需要重编固件。链路断在哪一环，靠它说话而不是靠推断。
        src = io.open(dst, encoding="utf-8").read()
        anchor = "  if switch:get_aux_switch_pos() == 2 then"
        assert src.count(anchor) == 1, "锚点不唯一，脚本改过了"
        dbg = ("  _dbg = (_dbg or 0) + 1\n"
               "  if _dbg % 100 == 0 then\n"
               "    gcs:send_text(6, string.format('DBG aux=%d mask=%d decl=%d chans=%d',\n"
               "      switch:get_aux_switch_pos(), stop_motor_bitmask:get(),\n"
               "      declare_to_mixer:get(), #(stop_motor_chan or {})))\n"
               "  end\n")
        io.open(dst, "w", encoding="utf-8").write(src.replace(anchor, dbg + anchor))
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
    _ok("MOT_STOP_BITMASK 存在——有脚本注册了它（哪一份见第 6 步）")

    # MOT_STOP_* 是 Lua 运行时注册的：启动时解析参数文件那一刻它们还不存在，
    # 文件里写了也会被静默忽略、随后由脚本按默认值 0 创建。所以必须在脚本起来
    # **之后**再设——现场从地面站设，正是这个道理。
    print("\n[1b] 脚本起来后再设 MOT_STOP_*（文件里设不进去）")
    def setparam(name, val):
        mav.mav.param_set_send(mav.target_system, mav.target_component,
                               name.encode(), float(val),
                               mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        end = time.time() + 5
        while time.time() < end:
            m = mav.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
            if m and m.param_id.strip("\x00") == name:
                return m.param_value
        return None
    mask = args.mask if args.mask is not None else (1 << (args.motor - 1))
    for k, v in (("MOT_STOP_DECL", 1), ("MOT_STOP_BITMASK", mask)):
        got = setparam(k, v)
        if got is None or abs(got - v) > 0.5:
            _fail("%s 设置失败：读回 %s，期望 %s" % (k, got, v))
        _ok("%s = %g" % (k, got))

    print("\n[2] 解锁前 MOT_FAIL_IDX 必须为 0")
    idx0 = getparam("MOT_FAIL_IDX")
    if idx0 is None or abs(idx0) > 0.5:
        _fail("MOT_FAIL_IDX = %s，非零会拒绝解锁" % idx0)
    _ok("MOT_FAIL_IDX = 0")

    print("\n[3] 起飞到 %.0f m" % args.alt)
    mav.set_mode_apm("GUIDED")
    pump(2)
    # 解锁前必须先送 RC 覆盖、且**油门压到最低**——与真实遥控器/地面站的解锁时序
    # 一致。少了这一步飞机会解锁后立刻上锁，而 GUIDED 起飞看起来只是"没上升"。
    for _ in range(20):
        mav.mav.rc_channels_override_send(mav.target_system, mav.target_component,
                                          1500, 1500, 1000, 1500,
                                          65535, 65535, 65535, 65535)
        pump(0.1)
    mav.mav.command_long_send(mav.target_system, mav.target_component,
                              mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                              0, 1, 0, 0, 0, 0, 0, 0)
    end = time.time() + 20
    armed = False
    while time.time() < end and not armed:
        m = mav.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if m and (m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            armed = True
    if not armed:
        _fail("解锁失败")
    _ok("已解锁")

    mav.mav.command_long_send(mav.target_system, mav.target_component,
                              mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                              0, 0, 0, 0, 0, 0, 0, args.alt)
    # 必须真的判到高度才算通过。上一版这里超时后拿最后一帧就 _ok()，于是飞机
    # 压根没起飞（高度 0.0 m）也报"✓ 已到 0.0 m"——正是本脚本要抓的那类
    # "判据永远不成立"的错误，我自己先犯了一次。
    end = time.time() + 90
    alt = -1.0
    while time.time() < end:
        m = mav.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=1)
        if m:
            alt = m.relative_alt / 1000.0
            if alt >= args.alt - 0.7:
                break
    if alt < args.alt - 0.7:
        _fail("起飞超时：只到 %.2f m，目标 %.1f m" % (alt, args.alt))
    _ok("已到 %.1f m，稳定 5 s" % alt)
    pump(5)

    print("\n[4] 拨杆（RC%d → 2000）" % AUX_CHAN)
    before = len(msgs)
    ch = [1500, 1500, 1000, 1500, 65535, 65535, 65535, 65535]
    ch[AUX_CHAN - 1] = 2000
    seen = {}
    for _ in range(30):
        mav.mav.rc_channels_override_send(mav.target_system, mav.target_component,
                                          *ch)
        m = mav.recv_match(type="RC_CHANNELS", blocking=False)
        if m:
            seen[AUX_CHAN] = getattr(m, "chan%d_raw" % AUX_CHAN, None)
        pump(0.2)
    pump(4)
    # 覆盖到底有没有落到飞控上——不落地就无从谈起脚本读不读得到。
    print("    RC%d 实测 = %s（期望 2000）" % (AUX_CHAN, seen.get(AUX_CHAN)))

    new = msgs[before:]
    if args.expect_blocked:
        print("\n[5] 非法掩码：必须什么都不发生（A01）")
        if any("declared motor" in t for t in new):
            _fail("出现了申报——非法掩码没有拦住申报")
        if any("mixer degraded" in t for t in new):
            _fail("混控器降级了——非法掩码没有拦住降级")
        if not any("BLOCKED" in t for t in new):
            _fail("没有 BLOCKED 告警——脚本没有识别出非法掩码")
        _ok("未申报、未降级，且给出了 BLOCKED 告警")
        idx = getparam("MOT_FAIL_IDX")
        if idx is None or abs(idx) > 0.5:
            _fail("MOT_FAIL_IDX = %s，应仍为 0" % idx)
        _ok("MOT_FAIL_IDX 仍为 0")
        # 关键：输出有没有真的被压下去。A01 的原始缺陷正是"拒绝申报但照常注入"，
        # 只查消息查不出来——必须看电机输出。
        lo = None
        end2 = time.time() + 5
        while time.time() < end2:
            m = mav.recv_match(type="SERVO_OUTPUT_RAW", blocking=True, timeout=1)
            if m:
                vals = [getattr(m, "servo%d_raw" % i) for i in range(1, 7)]
                lo = min(vals) if lo is None else min(lo, min(vals))
        print("    六路输出最低值 = %s" % lo)
        if lo is not None and lo < 1100:
            _fail("有电机被压到 %d —— 注入仍在执行，A01 未修复" % lo)
        _ok("没有电机被压到最低——注入确实被整体阻断")
        print("\n全部通过（非法掩码用例）。结果目录: %s" % out)
        return

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

    print("\n[8] 分配器是否真的在跑")
    # 前七步即便 MOT_FAIL_ALLOC=0 也会全过：set_motor_failed() 不读那个参数，
    # declared / degraded 两条消息照发，MOT_FAIL_IDX 照样被写。只有 MALC 能
    # 分开"摘列成功"与"重分配求解器在工作"。
    import glob as _glob
    logs = sorted(_glob.glob(os.path.join(out, "logs", "*.EFT")) +
                  _glob.glob(os.path.join(out, "logs", "*.BIN")))
    if not logs:
        _fail("没有 DataFlash 日志，无法确认分配器")
    from pymavlink import DFReader
    df = DFReader.DFReader_binary(logs[-1])
    res = {}
    n_malc = 0
    while True:
        x = df.recv_match(type="MALC")
        if x is None:
            break
        n_malc += 1
        res[int(x.Res)] = res.get(int(x.Res), 0) + 1
    if n_malc == 0:
        _fail("MALC 一条都没有——混控器没进降级路径，或 MOT_FAIL_ALLOC=0")
    ok_frac = res.get(0, 0) / float(n_malc)
    print("    MALC %d 条，Res 分布 %s" % (n_malc, res))
    if ok_frac < 0.5:
        _fail("求解成功率仅 %.1f%%，分配器多数周期在回退" % (ok_frac * 100))
    _ok("求解成功率 %.1f%%（失败 %.1f%% 会静默回退前向混控）"
        % (ok_frac * 100, (1 - ok_frac) * 100))

    print("\n全部通过。结果目录: %s" % out)


if __name__ == "__main__":
    main()
