"""Deep compare bootloader regions of two with_bl hex files."""
import struct
import sys
from intelhex import IntelHex

FLASH = 0x08000000
APP = 0x08020000


def load(path):
    return IntelHex(path)


def tobin(ih, start, end):
    return bytes(ih.tobinarray(start=start, size=end - start))


def find_strings(data, base, min_len=4):
    out = []
    cur, start = [], None
    for i, b in enumerate(data):
        if 32 <= b < 127:
            if start is None:
                start = i
            cur.append(chr(b))
        else:
            if len(cur) >= min_len:
                out.append((base + start, "".join(cur)))
            cur, start = [], None
    return out


def main():
    a, b = sys.argv[1], sys.argv[2]
    ih_a, ih_b = load(a), load(b)
    bl_a = tobin(ih_a, FLASH, APP)
    bl_b = tobin(ih_b, FLASH, APP)

    # strip trailing 0xFF padding for size estimate
    def used(buf):
        i = len(buf)
        while i > 0 and buf[i - 1] == 0xFF:
            i -= 1
        return i

    ua, ub = used(bl_a), used(bl_b)
    print(f"BL used size: v20={ua} bytes, v22={ub} bytes, delta={ub-ua}")
    print(f"BL identical: {bl_a == bl_b}")
    print(f"BL used identical: {bl_a[:max(ua,ub)] == bl_b[:max(ua,ub)]}")

    # Vector table first 8 words
    print("\n=== Vector table (first 16 words) ===")
    for i in range(16):
        wa = struct.unpack_from("<I", bl_a, i * 4)[0]
        wb = struct.unpack_from("<I", bl_b, i * 4)[0]
        mark = "" if wa == wb else "  <<<"
        print(f"  [{i:2d}] v20=0x{wa:08X}  v22=0x{wb:08X}{mark}")

    # Diff density by 4KB page
    print("\n=== Diff density by 4KB ===")
    for off in range(0, 128 * 1024, 4096):
        chunk_a = bl_a[off : off + 4096]
        chunk_b = bl_b[off : off + 4096]
        nd = sum(1 for x, y in zip(chunk_a, chunk_b) if x != y)
        if nd:
            print(f"  0x{FLASH+off:08X}: {nd} bytes differ")

    # Unique / changed strings
    sa = {s for _, s in find_strings(bl_a, FLASH)}
    sb = {s for _, s in find_strings(bl_b, FLASH)}
    only_a = sorted(sa - sb)
    only_b = sorted(sb - sa)
    print(f"\n=== Strings only in v20 BL ({len(only_a)}) ===")
    for s in only_a[:40]:
        print(f"  {s!r}")
    print(f"\n=== Strings only in v22 BL ({len(only_b)}) ===")
    for s in only_b[:40]:
        print(f"  {s!r}")

    # USB manufacturer/product
    for label, buf in (("v20", bl_a), ("v22", bl_b)):
        for needle in (b"EFT_FC_T", b"EFT_CAAC", b"ArduPilot", b"Bootloader"):
            idx = buf.find(needle)
            print(f"  {label} find {needle!r}: {('0x%08X' % (FLASH+idx)) if idx>=0 else 'no'}")


if __name__ == "__main__":
    main()
