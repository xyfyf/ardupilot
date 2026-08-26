"""Detailed multi-panel zoom on a maneuver+brake event."""
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from pymavlink import mavutil

path = sys.argv[1]
want = ["ATT", "RATE", "RCIN", "RCOU", "XKF1", "XKF3", "PIDR", "PIDP", "PIDY", "MOTB", "IMU"]
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


def tab(name, inst_field=None, inst=None):
    rows = data[name]
    if inst_field is not None:
        rows = [r for r in rows if r.get(inst_field) == inst]
    return np.array([r["_t"] - t0 for r in rows]), rows


def col(rows, f):
    return np.array([r.get(f, np.nan) for r in rows], dtype=float)


tA, A = tab("ATT")
tR, R = tab("RATE")
tC, C = tab("RCIN")
tO, O = tab("RCOU")
tX, X = tab("XKF1", "C", 0)
tPR, PR = tab("PIDR")
tPP, PP = tab("PIDP")
tPY, PY = tab("PIDY")
spd = np.hypot(col(X, "VN"), col(X, "VE"))

windows = [(47, 54), (108, 117), (148, 162), (238, 248), (273, 284)]
fig, axes = plt.subplots(len(windows), 4, figsize=(22, 3.0 * len(windows)))
for row, (s, e) in enumerate(windows):
    mA = (tA >= s) & (tA <= e)
    mR = (tR >= s) & (tR <= e)
    mC = (tC >= s) & (tC <= e)
    mO = (tO >= s) & (tO <= e)
    mX = (tX >= s) & (tX <= e)
    mPR = (tPR >= s) & (tPR <= e)
    mPP = (tPP >= s) & (tPP <= e)
    mPY = (tPY >= s) & (tPY <= e)

    a = axes[row, 0]
    a.plot(tA[mA], col(A, "DesRoll")[mA], label="DesRoll")
    a.plot(tA[mA], col(A, "Roll")[mA], label="Roll")
    a.plot(tA[mA], col(A, "DesPitch")[mA], label="DesPitch")
    a.plot(tA[mA], col(A, "Pitch")[mA], label="Pitch")
    a.axhline(20, color="r", ls=":", lw=0.8)
    a.axhline(-20, color="r", ls=":", lw=0.8)
    a.set_ylabel(f"{s}-{e}s\nangle deg")
    a.legend(fontsize=6, ncol=2)
    a.grid(alpha=0.3)

    a = axes[row, 1]
    a.plot(tR[mR], col(R, "RDes")[mR], label="RDes")
    a.plot(tR[mR], col(R, "R")[mR], label="R")
    a.plot(tR[mR], col(R, "PDes")[mR], label="PDes")
    a.plot(tR[mR], col(R, "P")[mR], label="P")
    a.set_ylabel("rate deg/s")
    a.legend(fontsize=6, ncol=2)
    a.grid(alpha=0.3)

    a = axes[row, 2]
    a.plot(tPR[mPR], col(PR, "P")[mPR], label="R.P")
    a.plot(tPR[mPR], col(PR, "I")[mPR], label="R.I")
    a.plot(tPR[mPR], col(PR, "D")[mPR], label="R.D")
    a.plot(tPY[mPY], col(PY, "I")[mPY], "--", label="Y.I")
    a.axhline(0.2, color="r", ls=":", lw=0.8)
    a.axhline(-0.2, color="r", ls=":", lw=0.8)
    a.set_ylabel("PID terms")
    a.legend(fontsize=6, ncol=2)
    a.grid(alpha=0.3)

    a = axes[row, 3]
    for i in range(1, 7):
        a.plot(tO[mO], col(O, f"C{i}")[mO], lw=0.8, label=f"C{i}")
    a2 = a.twinx()
    a2.plot(tX[mX], spd[mX], "k--", lw=1.2, alpha=0.6)
    a2.set_ylabel("spd m/s")
    a.set_ylabel("RCOU")
    a.legend(fontsize=5, ncol=3)
    a.grid(alpha=0.3)
fig.tight_layout()
fig.savefig("jitter_detail.png", dpi=100)

# yaw behaviour
tA_, A_ = tA, A
print("=== yaw ===")
dy, y = col(A, "DesYaw"), col(A, "Yaw")
ey = (dy - y + 180) % 360 - 180
print(f"yaw err rms={np.sqrt(np.nanmean(ey**2)):.2f} max={np.nanmax(np.abs(ey)):.2f} at t={tA[np.nanargmax(np.abs(ey))]:.1f}s")
yI = col(PY, "I")
print(f"yaw I: mean={np.nanmean(yI):.3f} p95={np.nanpercentile(yI,95):.3f} min={np.nanmin(yI):.3f} max={np.nanmax(yI):.3f} (IMAX=0.6)")
ydes, yact = col(R, "YDes"), col(R, "Y")
print(f"yaw rate err rms={np.sqrt(np.nanmean((ydes-yact)**2)):.2f} deg/s")

# roll I-term saturation vs IMAX=0.2
for nm, rows in (("roll", PR), ("pitch", PP)):
    Ii = col(rows, "I")
    print(f"{nm} rate-I: p95|I|={np.nanpercentile(np.abs(Ii),95):.4f} max|I|={np.nanmax(np.abs(Ii)):.4f} "
          f"frac|I|>0.19={np.mean(np.abs(Ii)>0.19):.4f} (IMAX=0.20)")

# how often is angle target at the 20deg limit
dr, dp = col(A, "DesRoll"), col(A, "DesPitch")
tot = np.hypot(dr, dp)
print(f"\nDesired lean angle: max={np.nanmax(tot):.1f}deg  frac>19.5deg={np.mean(tot>19.5):.4f}")
print(f"  DesRoll  frac|.|>19.5 = {np.mean(np.abs(dr)>19.5):.4f}")
print(f"  DesPitch frac|.|>19.5 = {np.mean(np.abs(dp)>19.5):.4f}")
print("saved jitter_detail.png")
