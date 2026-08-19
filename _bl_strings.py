"""Extract build fingerprints from bootloader region of a with_bl hex."""
import re
import sys
from intelhex import IntelHex

FLASH, APP = 0x08000000, 0x08020000


def strings(data, base, min_len=4):
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
    path = sys.argv[1]
    label = sys.argv[2] if len(sys.argv) > 2 else path
    ih = IntelHex(path)
    bl = bytes(ih.tobinarray(start=FLASH, size=APP - FLASH))
    used = len(bl)
    while used and bl[used - 1] == 0xFF:
        used -= 1
    print(f"=== {label} ===")
    print(f"used={used} bytes")

    ss = strings(bl, FLASH)
    # interesting patterns
    keys = re.compile(
        r"(Jan|Feb|Mar|Apr|May|Jun|Jul|Aug|Sep|Oct|Nov|Dec)\s+\d|"
        r"\d{4}-\d{2}-\d{2}|GCC|arm-|ChibiOS|ArduPilot|EFT|BOARD|Boot|git|"
        r"[0-9a-f]{7,12}|USB|STM32|APM|version|Version|BL",
        re.I,
    )
    print("\n-- interesting strings --")
    for addr, s in ss:
        if keys.search(s) or len(s) >= 8:
            # skip pure garbage of symbols
            if sum(c.isalnum() or c in " ._-+/%:" for c in s) < len(s) * 0.7:
                continue
            print(f"  0x{addr:08X}: {s!r}")

    # dump all printable strings longer than 5 for manual scan of dates
    print("\n-- all strings len>=5 (filtered ascii-ish) --")
    for addr, s in ss:
        if len(s) < 5:
            continue
        if not all(32 <= ord(c) < 127 for c in s):
            continue
        alnum = sum(c.isalnum() for c in s)
        if alnum < 3:
            continue
        print(f"  0x{addr:08X}: {s!r}")


if __name__ == "__main__":
    main()
