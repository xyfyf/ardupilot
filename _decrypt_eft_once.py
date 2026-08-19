#!/usr/bin/env python3
"""One-off EFT log decrypt (matches AP_Logger_EFT_Crypto)."""
import hashlib
import struct
import sys
from pathlib import Path

MASTER_KEY = bytes([
    0xEF, 0x54, 0x46, 0x4c, 0x4f, 0x47, 0x4d, 0x41, 0x53, 0x54, 0x45, 0x52,
    0x4b, 0x45, 0x59, 0x21, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
])
KDF_SALT = b"EFT-LOG-v1"
HEADER_SIZE = 17


def derive_key(fc_sn: str) -> bytes:
    h = hashlib.blake2b(digest_size=32)
    h.update(MASTER_KEY)
    h.update(KDF_SALT)
    if fc_sn:
        h.update(fc_sn.encode("ascii"))
    return h.digest()


def rotl32(v, n):
    return ((v << n) & 0xFFFFFFFF) | (v >> (32 - n))


def quarterround(a, b, c, d):
    a = (a + b) & 0xFFFFFFFF
    d ^= a
    d = rotl32(d, 16)
    c = (c + d) & 0xFFFFFFFF
    b ^= c
    b = rotl32(b, 12)
    a = (a + b) & 0xFFFFFFFF
    d ^= a
    d = rotl32(d, 8)
    c = (c + d) & 0xFFFFFFFF
    b ^= c
    b = rotl32(b, 7)
    return a, b, c, d


def chacha20_block(key, nonce8, ctr):
    constants = b"expand 32-byte k"
    state = list(struct.unpack("<4I", constants))
    state += list(struct.unpack("<8I", key))
    state += [ctr & 0xFFFFFFFF, (ctr >> 32) & 0xFFFFFFFF]
    state += list(struct.unpack("<2I", nonce8))
    working = state[:]
    for _ in range(10):
        working[0], working[4], working[8], working[12] = quarterround(
            working[0], working[4], working[8], working[12]
        )
        working[1], working[5], working[9], working[13] = quarterround(
            working[1], working[5], working[9], working[13]
        )
        working[2], working[6], working[10], working[14] = quarterround(
            working[2], working[6], working[10], working[14]
        )
        working[3], working[7], working[11], working[15] = quarterround(
            working[3], working[7], working[11], working[15]
        )
        working[0], working[5], working[10], working[15] = quarterround(
            working[0], working[5], working[10], working[15]
        )
        working[1], working[6], working[11], working[12] = quarterround(
            working[1], working[6], working[11], working[12]
        )
        working[2], working[7], working[8], working[13] = quarterround(
            working[2], working[7], working[8], working[13]
        )
        working[3], working[4], working[9], working[14] = quarterround(
            working[3], working[4], working[9], working[14]
        )
    out = [(working[i] + state[i]) & 0xFFFFFFFF for i in range(16)]
    return b"".join(struct.pack("<I", x) for x in out)


def ietf_keystream(key, nonce12, ctr, length):
    big_ctr = ctr + (struct.unpack("<I", nonce12[:4])[0] << 32)
    nonce8 = nonce12[4:12]
    out = bytearray()
    block_ctr = big_ctr
    while len(out) < length:
        out.extend(chacha20_block(key, nonce8, block_ctr))
        block_ctr += 1
    return bytes(out[:length])


def crypt_buffer(data: bytearray, key: bytes, nonce: bytes, plain_offset: int = 0):
    pos = 0
    n = len(data)
    while pos < n:
        abs_off = plain_offset + pos
        block_ctr = abs_off // 64
        skip = abs_off % 64
        chunk = min(n - pos, 64 - skip)
        ks = ietf_keystream(key, nonce, block_ctr, skip + chunk)
        for i in range(chunk):
            data[pos + i] ^= ks[skip + i]
        pos += chunk


def sn_from_filename(name: str) -> str:
    # 00000089_EFT9988776655_NOTIME.EFT -> EFT9988776655
    stem = Path(name).stem
    parts = stem.split("_")
    if len(parts) >= 2:
        return parts[1]
    return ""


def decrypt_eft(src: Path, dst: Path, fc_sn: str):
    raw = src.read_bytes()
    if raw[:4] != b"EFTL":
        raise ValueError(f"not EFTL: magic={raw[:4]!r}")
    version = raw[4]
    nonce = raw[5:17]
    cipher = bytearray(raw[17:])
    key = derive_key(fc_sn)
    crypt_buffer(cipher, key, nonce, 0)
    dst.write_bytes(cipher)
    return version, nonce, len(cipher)


def main():
    src = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(
        r"c:\Users\Administrator\xwechat_files\wxid_vp33z98okbyu21_4f0b\msg\file\2026-08\00000089_EFT9988776655_NOTIME.EFT"
    )
    fc_sn = sys.argv[2] if len(sys.argv) > 2 else sn_from_filename(src.name)
    dst = src.with_suffix(".bin")
    ver, nonce, plen = decrypt_eft(src, dst, fc_sn)
    plain = dst.read_bytes()[:64]
    print(f"fc_sn={fc_sn}")
    print(f"version={ver}")
    print(f"nonce={nonce.hex()}")
    print(f"plain_size={plen}")
    print(f"output={dst}")
    print(f"head_hex={plain[:32].hex()}")
    print(f"head_ascii={plain[:32]!r}")


if __name__ == "__main__":
    main()
