#!/usr/bin/env python3
"""刻画回归套件自身的噪声底：同一份二进制重复跑，逐指标算离散度。

为什么需要这个
--------------

20 条判据的每个门槛都是拍出来的，而**没有人量过同一份二进制重复跑会差多少**。
门槛离实测值 11% 和离 60%，在噪声未知时是同一句话「通过」；噪声一旦有几个百分点，
前者随时会翻。

2026-09-04 发现的诱因：`dev-planning` 那条线报「回归数值与改动前逐字一致」，
自查后推翻——两次运行都是改动后的，而且两次之间本来也不同（弧内掉速
2.1%→2.2%、滚转误差峰值 6.7°→6.9°）。**「逐字一致」这个判据对这套回归不成立**，
但当时没人能说出正确的判据该是多宽，因为噪声底从未被测量。

用法
----

  # 跑 N 轮（外部循环），然后
  regression_noise.py runs/regression-A runs/regression-B runs/regression-C
  regression_noise.py --latest 3          # 自动取最近三轮
  regression_noise.py ... --csv out.csv

口径
----

**只比较条目与指标名都相同的项。** 轮次之间条目集合若有出入（改了 SUITE），
不同的那些单独列出而不是悄悄跳过——那意味着这几轮不是同一个套件，合并统计
本身就是错的。

指标值形如 `"0.43 m/s"`，取前导数值；取不出数的（纯文本、布尔）不参与统计但会
计数，因为「有多少指标根本不是数」也是这套套件的一个事实。

离散度用**极差占均值的比例**（`(max-min)/|mean|`），不用标准差：三轮的标准差
自由度只有 2，极差更直白，且我们关心的正是「最坏能差多少」。
"""
import argparse
import glob
import json
import os
import re
import sys
from collections import OrderedDict

NUM_RE = re.compile(r'^\s*([+-]?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?)')


def parse_value(s):
    """从 '0.43 m/s' 取 0.43；取不出返回 None。"""
    if isinstance(s, (int, float)):
        return float(s)
    if not isinstance(s, str):
        return None
    m = NUM_RE.match(s)
    return float(m.group(1)) if m else None


def load_run(path):
    """读一轮回归，返回 {(pid, case, 指标名): (数值, 原文)}。"""
    jp = path if path.endswith('.json') else os.path.join(path, 'regression.json')
    with open(jp, encoding='utf-8') as f:
        rows = json.load(f)
    out = OrderedDict()
    for r in rows:
        key_head = (r.get('item_pid', '?'), r.get('case', r.get('item_name', '?')))
        for name, raw in (r.get('metrics') or {}).items():
            out[key_head + (name,)] = (parse_value(raw), raw)
    return out


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('runs', nargs='*', help='regression-* 目录或 regression.json')
    ap.add_argument('--latest', type=int, metavar='N',
                    help='自动取 runs/ 下最近 N 轮')
    ap.add_argument('--csv', help='把逐指标统计导出为 CSV')
    ap.add_argument('--warn-frac', type=float, default=0.02,
                    help='极差占均值超过此比例即标注（默认 0.02）')
    args = ap.parse_args(argv)

    paths = list(args.runs)
    if args.latest:
        here = os.path.dirname(os.path.abspath(__file__))
        # 只取已完成的轮次。正在跑的那一轮目录已经建好但还没有 regression.json，
        # 选中它只会让整条命令报 FileNotFound——而"最近 N 轮"的本意从来不包括
        # 还没跑完的那一轮。
        cand = [p for p in sorted(glob.glob(os.path.join(here, 'runs', 'regression-*')))
                if os.path.exists(os.path.join(p, 'regression.json'))]
        paths += cand[-args.latest:]
    if len(paths) < 2:
        ap.error('至少要两轮才能谈离散度')

    runs = []
    for p in paths:
        try:
            runs.append((os.path.basename(p.rstrip('/')), load_run(p)))
        except Exception as e:                       # noqa: BLE001
            print('读不了 %s：%s' % (p, e), file=sys.stderr)
            return 2

    print('=' * 78)
    print('回归噪声底：%d 轮，同一份二进制' % len(runs))
    for n, _ in runs:
        print('  ' + n)
    print()

    # 只统计所有轮次都有的键；不一致的单独说，不静默跳过
    keysets = [set(r.keys()) for _, r in runs]
    common = set.intersection(*keysets)
    union = set.union(*keysets)
    missing = union - common
    if missing:
        print('!! %d 个指标并非每轮都有——这几轮可能不是同一个套件，'
              '合并统计对它们无效：' % len(missing))
        for k in sorted(missing):
            have = [n for n, r in runs if k in r]
            print('   %-6s %-24s %-18s 仅见于 %s' % (k[0], k[1], k[2], '/'.join(have)))
        print()

    stats, non_numeric = [], []
    for k in sorted(common):
        vals = [r[k][0] for _, r in runs]
        if any(v is None for v in vals):
            non_numeric.append((k, [r[k][1] for _, r in runs]))
            continue
        lo, hi = min(vals), max(vals)
        mean = sum(vals) / len(vals)
        spread = hi - lo
        frac = spread / abs(mean) if abs(mean) > 1e-12 else (0.0 if spread == 0 else float('inf'))
        stats.append((frac, k, vals, mean, lo, hi, spread))

    stats.sort(reverse=True)
    print('%-6s %-24s %-18s %9s %9s %9s' % ('问题', '条目', '指标', '均值', '极差', '极差/均值'))
    print('-' * 78)
    for frac, k, vals, mean, lo, hi, spread in stats:
        flag = ''
        if frac == float('inf'):
            flag = '  ← 均值为零而有波动'
        elif frac >= args.warn_frac:
            flag = '  ← 波动 %.1f%%' % (frac * 100)
        print('%-6s %-24s %-18s %9.4f %9.4f %8.2f%%%s'
              % (k[0], k[1][:24], k[2][:18], mean, spread, frac * 100, flag))

    if non_numeric:
        print()
        print('%d 个指标不是数值，不参与统计：' % len(non_numeric))
        for k, raws in non_numeric:
            same = '一致' if len(set(map(str, raws))) == 1 else '**各轮不同**'
            print('   %-6s %-24s %-18s %s  %s' % (k[0], k[1][:24], k[2][:18], same, raws[0]))

    print()
    n_zero = sum(1 for f, *_ in stats if f == 0.0)
    print('小结：%d 个数值指标中 %d 个逐轮完全相同，%d 个波动 ≥ %.0f%%；'
          % (len(stats), n_zero, sum(1 for f, *_ in stats if f >= args.warn_frac),
             args.warn_frac * 100))
    if stats:
        worst = stats[0]
        print('      最大波动 %.2f%%（%s / %s）。'
              % (worst[0] * 100, worst[1][1], worst[1][2]))
    print()
    print('怎么用这张表：把某条判据的「实测距门槛的余量」除以该指标这里的波动，')
    print('得到的倍数就是它的安全系数。倍数接近 1 的判据随时会被噪声掀翻，')
    print('而它通过与否与代码对错无关。')

    if args.csv:
        import csv
        with open(args.csv, 'w', newline='', encoding='utf-8') as f:
            w = csv.writer(f)
            w.writerow(['问题', '条目', '指标'] + [n for n, _ in runs]
                       + ['均值', '最小', '最大', '极差', '极差占均值'])
            for frac, k, vals, mean, lo, hi, spread in stats:
                w.writerow(list(k) + vals + [mean, lo, hi, spread, frac])
        print('\nCSV: %s' % args.csv)
    return 0


if __name__ == '__main__':
    sys.exit(main())
