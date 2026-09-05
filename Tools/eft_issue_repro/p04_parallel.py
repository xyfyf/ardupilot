#!/usr/bin/env python3
"""并行跑多个 P04 架次：每个任务占一个 SITL 实例号，端口互不重叠。

单个 SITL 只吃约 5% 的一核，而机器有 24 核——串行跑扫描时机器几乎是闲的。
瓶颈从来不是算力，是 reproduce.py 把端口写死在 5760、只能靠一把全局锁串行化。
加上 `-I N` 之后端口变成 5760+10*N（SITL_cmdline.cpp:428-437 把 base/rcin/
fg_view/simulator 四个端口全都偏移），实例之间彻底隔离。

**并行度要留余量，不能开满。** SITL 走仿真时钟，CPU 被抢时只是跑得慢、物理步进
不变，所以结果不受影响；但 reproduce.py 里的**墙钟**超时（--timeout、
wait_disarmed）会被拖垮。默认 4 路，24 核上每路即便吃满一核也只占 1/6。

用法：
    # 对称性验证：无风下 3/6 号 × YTRK 0/1
    p04_parallel.py --motors 3 6 --wind 0 --ytrk 0 1

    # 风向镜像
    p04_parallel.py --motors 3 6 --wind 8 --wind-dir 90 270 --ytrk 1
"""
import argparse
import json
import os
import queue
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPRO = os.path.join(HERE, "reproduce.py")


def run_case(case, inst, outdir, timeout_s):
    m, w, d, y = case
    variant = "par_m%d_w%d_d%d_y%d" % (m, w, d, y)
    cmd = [sys.executable, REPRO, "motor-fail", "--variant", variant,
           "--motor", str(m), "--degrade",          # 先验申报，与现场第一阶段一致
           "--instance", str(inst),
           "--set", "SIM_WIND_SPD=%d" % w,
           "--set", "SIM_WIND_DIR=%d" % d,
           "--set", "MOT_FAIL_YTRK=%d" % y]
    log = os.path.join(outdir, "%s.log" % variant)
    started = time.time()
    with open(log, "w", encoding="utf-8") as fh:
        try:
            p = subprocess.run(cmd, cwd=HERE, timeout=timeout_s,
                               stdout=fh, stderr=subprocess.STDOUT)
            rc = p.returncode
        except subprocess.TimeoutExpired:
            return variant, None, "超时"
    if rc != 0:
        return variant, None, "returncode=%d" % rc
    import glob
    hits = [h for h in glob.glob(os.path.join(HERE, "runs", "*%s" % variant,
                                              "result.json"))
            if os.path.getmtime(h) >= started - 5.0]
    if not hits:
        return variant, None, "无结果"
    return variant, json.load(open(sorted(hits)[-1], encoding="utf-8")), ""


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--motors", nargs="+", type=int, required=True)
    ap.add_argument("--wind", nargs="+", type=int, default=[0])
    ap.add_argument("--wind-dir", nargs="+", type=int, default=[90])
    ap.add_argument("--ytrk", nargs="+", type=int, default=[0, 1])
    ap.add_argument("--jobs", type=int, default=4, help="并行度，默认 4")
    ap.add_argument("--timeout", type=int, default=1200)
    args = ap.parse_args()

    cases = [(m, w, d, y) for m in args.motors for w in args.wind
             for d in args.wind_dir for y in args.ytrk]
    stamp = time.strftime("%Y%m%d-%H%M%S")
    outdir = os.path.join(HERE, "runs", "par-%s" % stamp)
    os.makedirs(outdir, exist_ok=True)
    print("并行扫描：%d 架次，%d 路，结果落在 %s\n" % (len(cases), args.jobs, outdir))

    work = queue.Queue()
    for c in cases:
        work.put(c)
    results = {}
    lock = threading.Lock()

    def worker(inst):
        while True:
            try:
                c = work.get_nowait()
            except queue.Empty:
                return
            v, r, note = run_case(c, inst, outdir, args.timeout)
            with lock:
                results[v] = (r, note)
                print("  [%d/%d] %-22s %s" % (len(results), len(cases), v,
                                              "ok" if r else "失败 " + note))

    # 实例号从 0 开始：0 就是原来的 5760，与既有归档架次同端口，便于对照。
    ts = [threading.Thread(target=worker, args=(i,)) for i in range(args.jobs)]
    t0 = time.time()
    for t in ts:
        t.start()
    for t in ts:
        t.join()
    print("\n耗时 %.1f 分钟\n" % ((time.time() - t0) / 60.0))

    def g(r, k):
        return r.get(k, r.get("metrics", {}).get(k)) if r else None

    print("%-22s %9s %9s %9s %9s %9s" % ("工况", "滚转峰", "滚转稳", "俯仰峰",
                                         "偏航均", "累计转"))
    for v in sorted(results):
        r, note = results[v]
        if not r:
            print("%-22s  %s" % (v, note))
            continue
        print("%-22s %9s %9s %9s %9s %9s" % (
            v.replace("par_", ""), g(r, "roll_err_max_deg"), g(r, "roll_steady_deg"),
            g(r, "pitch_err_max_deg"), g(r, "yaw_rate_mean_degs"),
            g(r, "yaw_total_rotation_deg")))
    with open(os.path.join(outdir, "summary.json"), "w", encoding="utf-8") as f:
        json.dump({k: v[0] for k, v in results.items()}, f,
                  ensure_ascii=False, indent=2)


if __name__ == "__main__":
    main()
