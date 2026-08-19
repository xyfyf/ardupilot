import re

LOG = r"c:\Users\Administrator\Desktop\long_text_03231322-AEE7-4F7C-827F-0265C0DBE96A.txt"

with open(LOG, encoding="utf-8", errors="ignore") as f:
    text = f.read()

hx = re.sub(r"[^0-9A-Fa-f]", "", text)
if len(hx) % 2:
    print("odd hex chars!", len(hx))
data = bytes.fromhex(hx)
print("total bytes", len(data))
print("last 40 hex:", data[-40:].hex(" "))

msgs = []
i = 0
while i < len(data):
    if data[i] != 0xFD:
        i += 1
        continue
    if i + 10 > len(data):
        break
    plen = data[i + 1]
    msgid = data[i + 7] | (data[i + 8] << 8) | (data[i + 9] << 16)
    need = 10 + plen + 2
    ok = i + need <= len(data)
    msgs.append(
        dict(off=i, seq=data[i + 4], msg=msgid, plen=plen, ok=ok, need=need, have=len(data) - i)
    )
    if ok:
        i += need
    else:
        break

names = {0: "HB", 1: "SYS", 24: "GPS", 30: "ATT", 33: "GPOS", 74: "VFR", 178: "AHRS2", 193: "EKF"}
print("frames parsed", len(msgs))
for m in msgs[-8:]:
    st = "OK" if m["ok"] else "TRUNC"
    print(f"  seq={m['seq']:3d} {names.get(m['msg'], str(m['msg'])):6s} plen={m['plen']:2d} {st}")

last = msgs[-1]
if not last["ok"]:
    print("\nLast frame TRUNCATED:")
    print(
        f"  msg={last['msg']} seq={last['seq']} need={last['need']} have={last['have']} "
        f"missing={last['need'] - last['have']} bytes"
    )
    print("  raw:", data[last["off"] :].hex(" "))
else:
    print("\nLast frame complete")

complete = [m for m in msgs if m["ok"]]
vfr_idx = [i for i, m in enumerate(complete) if m["msg"] == 74]
print("\nVFR complete count", len(vfr_idx))
for idx in vfr_idx[-3:]:
    if idx + 1 < len(complete):
        nxt = complete[idx + 1]
        print(
            f"  VFR seq={complete[idx]['seq']} -> next "
            f"{names.get(nxt['msg'], str(nxt['msg']))} seq={nxt['seq']}"
        )

hb = [m for m in complete if m["msg"] == 0]
print("\nHEARTBEAT count", len(hb), "last seq", hb[-1]["seq"] if hb else None)

ts = re.findall(r"\[(\d+:\d+:\d+\.\d+)\]", text)

def to_sec(t):
    h, m, s = t.split(":")
    return int(h) * 3600 + int(m) * 60 + float(s)

if ts:
    print("capture span", ts[0], "->", ts[-1], f"({to_sec(ts[-1]) - to_sec(ts[0]):.2f}s)")
    print("silence after last timestamp: unknown (no more lines in file)")

# Typical burst size: lines with ~787 bytes appear every ~1s
sizes = []
for line in text.splitlines():
    if "FD" not in line:
        continue
    hx_line = re.sub(r"[^0-9A-Fa-f]", "", line)
    if len(hx_line) % 2:
        hx_line = hx_line[:-1]
    sizes.append(len(bytes.fromhex(hx_line)) if hx_line else 0)
print("\nchunk sizes:", sizes)
print("pattern: small ~109B bursts + large ~787B bursts alternating")
