import re
from collections import Counter

LOG = r"c:\Users\Administrator\Desktop\long_text_03231322-AEE7-4F7C-827F-0265C0DBE96A.txt"

KNOWN = {
    0, 1, 2, 24, 27, 29, 30, 32, 33, 36, 42, 62, 65, 74, 125, 136, 147, 152, 163, 168, 178, 193, 241,
}

with open(LOG, encoding="utf-8", errors="ignore") as f:
    text = f.read()

hx = re.sub(r"[^0-9A-Fa-f]", "", text)
if len(hx) % 2:
    hx = hx[:-1]
data = bytes.fromhex(hx)

# Parse all plausible MAVLink2 frames
all_frames = []
i = 0
while i < len(data):
    if data[i] != 0xFD:
        i += 1
        continue
    if i + 10 > len(data):
        break
    plen = data[i + 1]
    seq = data[i + 4]
    msgid = data[i + 7] | (data[i + 8] << 8) | (data[i + 9] << 16)
    need = 10 + plen + 2
    if i + need > len(data):
        all_frames.append({"off": i, "seq": seq, "msg": msgid, "ok": False, "known": msgid in KNOWN})
        break
    all_frames.append({"off": i, "seq": seq, "msg": msgid, "ok": True, "known": msgid in KNOWN})
    i += need

valid = [f for f in all_frames if f["ok"] and f["known"]]
invalid = [f for f in all_frames if f["ok"] and not f["known"]]
trunc = [f for f in all_frames if not f["ok"]]

print("=== MAVLink seq analysis ===")
print(f"raw frames found : {len(all_frames)}")
print(f"valid known msgs : {len(valid)}")
print(f"sync-slip frames : {len(invalid)}")
print(f"truncated tail   : {len(trunc)}")

if len(valid) < 2:
    print("not enough valid frames")
    raise SystemExit

seqs = [f["seq"] for f in valid]
print(f"\nseq range: {seqs[0]} -> {seqs[-1]} (over {len(valid)} valid frames)")

# consecutive check with wrap 0-255
gaps = []
for a, b in zip(seqs, seqs[1:]):
    d = (b - a) % 256
    if d != 1:
        gaps.append({"from": a, "to": b, "delta": d, "lost_est": (d - 1) % 256})

print(f"\ncontinuity on valid frames:")
print(f"  expected step : +1 every frame")
print(f"  gap count     : {len(gaps)}")
print(f"  continuous    : {len(valid)-1-len(gaps)}/{len(valid)-1} transitions ({100*(len(valid)-1-len(gaps))/max(1,len(valid)-1):.1f}%)")
if gaps:
    lost_total = sum((g["delta"] - 1) % 256 for g in gaps)
    print(f"  est lost frames between valid ones: {lost_total}")

print("\ngap details (all):")
for g in gaps:
    lost = (g["delta"] - 1) % 256
    note = "wrap" if g["from"] == 255 and g["to"] == 0 else ""
    print(f"  {g['from']:3d} -> {g['to']:3d}  delta={g['delta']:3d}  lost~{lost:3d}  {note}")

# Also analyze ALL parsed frames including sync slips
if len(all_frames) >= 2:
    all_ok = [f for f in all_frames if f["ok"]]
    all_seqs = [f["seq"] for f in all_ok]
    all_gaps = []
    for a, b in zip(all_seqs, all_seqs[1:]):
        d = (b - a) % 256
        if d != 1:
            all_gaps.append((a, b, d))
    print(f"\nincluding sync-slip frames ({len(all_ok)} frames):")
    print(f"  gap count: {len(all_gaps)}")

# Longest continuous run
best = cur = 1
best_start = valid[0]["seq"]
run_start = valid[0]["seq"]
prev = valid[0]["seq"]
for f in valid[1:]:
    if (f["seq"] - prev) % 256 == 1:
        cur += 1
    else:
        if cur > best:
            best = cur
            best_start = run_start
        cur = 1
        run_start = f["seq"]
    prev = f["seq"]
if cur > best:
    best = cur
    best_start = run_start
print(f"\nlongest continuous run: {best} frames (starting seq={best_start})")

# Show last 20 valid seq
print("\nlast 20 valid seq:", seqs[-20:])

# Per-chunk line continuity (bytes as received by sniffer)
print("\n=== per display-chunk (24 lines) ===")
lines = [l for l in text.splitlines() if "FD" in l]
for n, line in enumerate(lines, 1):
    hxl = re.sub(r"[^0-9A-Fa-f]", "", line)
    if len(hxl) % 2:
        hxl = hxl[:-1]
    chunk = bytes.fromhex(hxl) if hxl else b""
    seq_in_chunk = []
    j = 0
    while j < len(chunk):
        if chunk[j] != 0xFD:
            j += 1
            continue
        if j + 10 > len(chunk):
            break
        plen = chunk[j + 1]
        msgid = chunk[j + 7] | (chunk[j + 8] << 8) | (chunk[j + 9] << 16)
        need = 10 + plen + 2
        if j + need > len(chunk):
            if msgid in KNOWN:
                seq_in_chunk.append(chunk[j + 4])
            break
        if msgid in KNOWN:
            seq_in_chunk.append(chunk[j + 4])
        j += need
    ts = re.search(r"\[(\d+:\d+:\d+\.\d+)\]", line)
    t = ts.group(1) if ts else "?"
    if len(seq_in_chunk) >= 2:
        local_gaps = sum(1 for a, b in zip(seq_in_chunk, seq_in_chunk[1:]) if (b - a) % 256 != 1)
    else:
        local_gaps = 0
    seq_show = ",".join(str(s) for s in seq_in_chunk[:8])
    if len(seq_in_chunk) > 8:
        seq_show += ",..."
    print(f"  L{n:2d} {t} seq=[{seq_show}] local_gaps={local_gaps}")

print("\n=== verdict ===")
if len(gaps) == 0:
    print("VALID: perfectly continuous on known-good frames")
elif len(gaps) <= 2 and all((g["delta"] - 1) % 256 <= 3 for g in gaps):
    print("OK: mostly continuous, minor loss")
else:
    pct = 100 * (len(valid) - 1 - len(gaps)) / max(1, len(valid) - 1)
    if pct >= 90:
        print("ACCEPTABLE: mostly continuous but with some loss/sync noise")
    elif pct >= 70:
        print("POOR: many gaps, capture/link quality issue")
    else:
        print("BAD: not sufficiently continuous")
