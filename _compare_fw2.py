"""Compare two ArduCopter .hex firmware images (bootloader + app) to help
diagnose why one version occasionally fails to boot.

Usage: python _compare_fw2.py <hex_v20> <hex_v22>
"""
import sys
import re
from intelhex import IntelHex

APP_START = 0x08020000  # FLASH_RESERVE_START_KB 128 -> app starts at 0x08000000+128K
FLASH_BASE = 0x08000000
FLASH_SIZE = 2048 * 1024


def load(path):
    ih = IntelHex(path)
    return ih


def dump_ranges(ih, label):
    print(f"\n[{label}] segments:")
    for start, end in ih.segments():
        print(f"  0x{start:08X} - 0x{end:08X}  ({end-start} bytes)")


def find_strings(data, base_addr, min_len=6):
    out = []
    cur = []
    cur_start = None
    for i, b in enumerate(data):
        if 32 <= b < 127:
            if cur_start is None:
                cur_start = i
            cur.append(b)
        else:
            if len(cur) >= min_len:
                out.append((base_addr + cur_start, bytes(cur).decode('ascii')))
            cur = []
            cur_start = None
    if len(cur) >= min_len:
        out.append((base_addr + cur_start, bytes(cur).decode('ascii')))
    return out


def find_version_strings(strings):
    pats = [re.compile(r'ArduCopter'), re.compile(r'\d+\.\d+\.\d+'), re.compile(r'APM:Copter')]
    hits = []
    for addr, s in strings:
        for p in pats:
            if p.search(s):
                hits.append((addr, s))
                break
    return hits


def diff_binaries(a, b, base, size, chunk=1):
    """Return list of (addr, len) differing byte ranges between two byte buffers."""
    diffs = []
    n = min(len(a), len(b))
    i = 0
    while i < n:
        if a[i] != b[i]:
            start = i
            while i < n and a[i] != b[i]:
                i += 1
            diffs.append((base + start, i - start))
        else:
            i += 1
    if len(a) != len(b):
        diffs.append((base + n, abs(len(a) - len(b))))
    return diffs


def main():
    if len(sys.argv) != 3:
        print("usage: _compare_fw2.py <hex_v20> <hex_v22>")
        sys.exit(1)

    path_a, path_b = sys.argv[1], sys.argv[2]
    ih_a = load(path_a)
    ih_b = load(path_b)

    dump_ranges(ih_a, "v20 (" + path_a + ")")
    dump_ranges(ih_b, "v22 (" + path_b + ")")

    max_end_a = max(end for _, end in ih_a.segments())
    max_end_b = max(end for _, end in ih_b.segments())
    top = max(max_end_a, max_end_b)

    buf_a = ih_a.tobinarray(start=FLASH_BASE, size=top - FLASH_BASE)
    buf_b = ih_b.tobinarray(start=FLASH_BASE, size=top - FLASH_BASE)

    bl_len = APP_START - FLASH_BASE
    bl_a, bl_b = buf_a[:bl_len], buf_b[:bl_len]
    app_a, app_b = buf_a[bl_len:], buf_b[bl_len:]

    print("\n=== Bootloader region (0x08000000 - 0x08020000) ===")
    if bytes(bl_a) == bytes(bl_b):
        print("  IDENTICAL")
    else:
        diffs = diff_binaries(bl_a, bl_b, FLASH_BASE, bl_len)
        print(f"  DIFFERS in {len(diffs)} contiguous range(s):")
        for addr, ln in diffs[:50]:
            print(f"    0x{addr:08X}, len={ln}")

    print("\n=== App region (0x08020000 onward) ===")
    diffs = diff_binaries(app_a, app_b, APP_START, len(app_a) if len(app_a) < len(app_b) else len(app_b))
    total_diff_bytes = sum(l for _, l in diffs)
    print(f"  {len(diffs)} contiguous differing ranges, {total_diff_bytes} bytes total")
    print(f"  app v20 size={len(app_a)} bytes, app v22 size={len(app_b)} bytes")

    # app descriptor search (AP_CheckFirmware app_descriptor_unsigned)
    import struct
    magic = bytes([0x40, 0xa2, 0xe4, 0xf1, 0x64, 0x68, 0x91, 0x06])
    for name, buf, base in (("v20 app", app_a, APP_START), ("v22 app", app_b, APP_START)):
        idx = bytes(buf).find(magic)
        print(f"\n  app_descriptor (unsigned) in {name}: {'0x%08X' % (base+idx) if idx >= 0 else 'NOT FOUND'}")
        if idx >= 0:
            raw = bytes(buf[idx:idx+28])
            sig, crc1, crc2, image_size, git_hash, ver_major, ver_minor, board_id = struct.unpack('<8sIIIIBBH', raw)
            print(f"    image_crc1=0x{crc1:08X} image_crc2=0x{crc2:08X} image_size={image_size} "
                  f"git_hash=0x{git_hash:08X} version={ver_major}.{ver_minor} board_id={board_id}")

    # version strings
    print("\n=== Version / identifying strings ===")
    for name, buf, base in (("v20 app", app_a, APP_START), ("v22 app", app_b, APP_START)):
        strs = find_strings(buf, base)
        hits = find_version_strings(strs)
        print(f"  {name}:")
        for addr, s in hits[:20]:
            print(f"    0x{addr:08X}: {s}")

    # board ID check (bootloader board_types)
    for name, buf, base in (("v20 bl", bl_a, FLASH_BASE), ("v22 bl", bl_b, FLASH_BASE)):
        strs = find_strings(buf, base)
        board_hits = [x for x in strs if 'EFT' in x[1] or 'CAAC' in x[1]]
        print(f"\n  {name} board-related strings:")
        for addr, s in board_hits[:20]:
            print(f"    0x{addr:08X}: {s}")

    print("\n=== Bootloader build/version strings (git hash, dates) ===")
    import re as _re
    date_pat = _re.compile(r'^(Jan|Feb|Mar|Apr|May|Jun|Jul|Aug|Sep|Oct|Nov|Dec) \d')
    for name, buf, base in (("v20 bl", bl_a, FLASH_BASE), ("v22 bl", bl_b, FLASH_BASE)):
        strs = find_strings(buf, base)
        hits = [x for x in strs if _re.match(r'^[0-9a-f]{8,10}$', x[1]) or date_pat.match(x[1])
                or 'ChibiOS' in x[1] or 'Bootloader' in x[1]]
        print(f"  {name}:")
        for addr, s in hits[:20]:
            print(f"    0x{addr:08X}: {s}")


if __name__ == "__main__":
    main()
