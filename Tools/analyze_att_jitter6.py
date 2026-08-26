"""Quantify lower-limit motor saturation (MOT_SPIN_MIN floor) during maneuvers."""
import sys
import numpy as np
from pymavlink import mavutil

path = sys.argv[1]
want = ["RCOU", "RATE", "ATT", "MOTB", "XKF1"]
data = {k: [] for k in want}
mlog = mavutil.mavlink_connection(path)
while True:
    m = mlog.recv_match(type=want)
    if m is None:
        break
    d = m.to_dict()
    d["_t"] = m._timestamp
    data[m.get_type()].append(d)
t0 = min(v[0]["_t"] for v in data.values() if v)


def tab(name):
    rows = data[name]
    return np.array([r["_t"] - t0 for r in rows]), rows


def col(rows, f):
    return np.array([r.get(f, np.nan) for r in rows], dtype=float)


PWM_MIN, PWM_MAX = 1050.0, 1950.0
SPIN_MIN, SPIN_MAX = 0.10, 0.95
floor = PWM_MIN + SPIN_MIN * (PWM_MAX - PWM_MIN)   # 1140
ceil_ = PWM_MIN + SPIN_MAX * (PWM_MAX - PWM_MIN)   # 1905
print(f"motor floor (MOT_SPIN_MIN) = {floor:.0f}us, ceiling (MOT_SPIN_MAX) = {ceil_:.0f}us")

tO, O = tab("RCOU")
mot = np.vstack([col(O, f"C{i}") for i in range(1, 7)])
tR, R = tab("RATE")
rr = np.interp(tO, tR, col(R, "R"))
pp = np.interp(tO, tR, col(R, "P"))
tX, X = tab("XKF1")
spd = np.interp(tO, tX, np.hypot(col(X, "VN"), col(X, "VE")))

fly = (tO > 20) & (tO < 285)
m = mot[:, fly]
print(f"\nin-flight ({fly.sum()} samples @10Hz):")
print(f"  mean output = {m.mean():.0f}us, i.e. {(m.mean()-floor):.0f}us above the floor")
print(f"  headroom down to floor (min motor): mean={np.mean(m.min(axis=0)-floor):.0f}us")
atfloor = (m <= floor + 5)
print(f"  frac samples with ANY motor at/below floor+5us : {np.mean(atfloor.any(axis=0)):.4f}")
print(f"  frac samples with >=2 motors at floor          : {np.mean(atfloor.sum(axis=0)>=2):.4f}")
print(f"  total time with any motor at floor            : {np.mean(atfloor.any(axis=0))*fly.sum()*0.1:.1f}s")

agg = fly & ((np.abs(rr) > 15) | (np.abs(pp) > 15))
calm = fly & (np.abs(rr) < 5) & (np.abs(pp) < 5)
for nm, msk in (("aggressive |rate|>15", agg), ("calm |rate|<5", calm)):
    if msk.sum() < 5:
        continue
    mm = mot[:, msk]
    af = (mm <= floor + 5)
    print(f"  [{nm}] n={msk.sum()} frac any motor at floor = {np.mean(af.any(axis=0)):.4f}, "
          f"min motor mean={mm.min(axis=0).mean():.0f}us")

# per-channel floor hits
print("\n  per-motor floor hits (in-flight):")
for i in range(6):
    print(f"    C{i+1}: {np.mean(mot[i, fly] <= floor+5):.4f}")

# how does spread behave
spread = m.max(axis=0) - m.min(axis=0)
print(f"\n  motor spread: mean={spread.mean():.0f}us p95={np.percentile(spread,95):.0f} max={spread.max():.0f}")

tM, M = tab("MOTB")
print(f"\n  MOTB.LiftMax mean={np.nanmean(col(M,'LiftMax')):.3f} min={np.nanmin(col(M,'LiftMax')):.3f}")
print(f"  MOTB.ThrOut mean={np.nanmean(col(M,'ThrOut')):.3f} min={np.nanmin(col(M,'ThrOut')):.3f} max={np.nanmax(col(M,'ThrOut')):.3f}")
print(f"  MOTB.ThrAvMx mean={np.nanmean(col(M,'ThrAvMx')):.3f}")
