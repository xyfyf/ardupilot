#!/usr/bin/env python3
"""从 raw/dji_cn_restricted_raw.json 生成 sdcard/EFT_nfz/ 飞控离线包。"""
import json
import math
import os
import shutil

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
RAW = os.path.join(ROOT, "raw", "dji_cn_restricted_raw.json")
OUT = os.path.join(ROOT, "sdcard", "EFT_nfz")


def normalize_ring(pp):
    if not pp:
        return None
    if isinstance(pp[0], (list, tuple)) and pp[0] and isinstance(pp[0][0], (list, tuple)):
        ring = pp[0]
    else:
        ring = pp
    coords = []
    for p in ring:
        if not isinstance(p, (list, tuple)) or len(p) < 2:
            continue
        a, b = float(p[0]), float(p[1])
        if 15 <= a <= 55 and 70 <= b <= 140:
            lat, lon = a, b
        elif 15 <= b <= 55 and 70 <= a <= 140:
            lon, lat = a, b
        else:
            lat, lon = a, b
        coords.append((lon, lat))
    if len(coords) < 3:
        return None
    if coords[0] == coords[-1]:
        coords = coords[:-1]
    if len(coords) < 3:
        return None
    return coords


def bbox_center_radius(coords):
    lons = [c[0] for c in coords]
    lats = [c[1] for c in coords]
    clon = (min(lons) + max(lons)) / 2
    clat = (min(lats) + max(lats)) / 2
    r = 0.0
    for lon, lat in coords:
        dlat = (lat - clat) * 111320
        dlon = (lon - clon) * 111320 * math.cos(math.radians(clat))
        r = max(r, math.sqrt(dlat * dlat + dlon * dlon))
    return clat, clon, r


def main():
    if not os.path.isfile(RAW):
        raise SystemExit(f"missing raw: {RAW}")
    os.makedirs(OUT, exist_ok=True)
    areas = json.load(open(RAW, encoding="utf-8"))
    records = []
    for a in areas:
        if a.get("level") not in (2, 4):
            continue
        parent_id = int(a["area_id"])
        name = a.get("name") or str(parent_id)
        rings = []
        for sa in a.get("sub_areas") or []:
            ring = normalize_ring(sa.get("polygon_points"))
            if ring:
                rings.append(ring)
        top = normalize_ring(a.get("polygon_points"))
        if not rings and top:
            rings = [top]
        if rings:
            for i, ring in enumerate(rings):
                zid = parent_id * 100 + i if len(rings) > 1 else parent_id
                clat, clon, cr = bbox_center_radius(ring)
                records.append(("P", zid, clat, clon, int(round(cr)), ring, name))
        else:
            lat, lng, r = a.get("lat"), a.get("lng"), a.get("radius")
            if lat is None or lng is None or not r:
                continue
            records.append(("C", parent_id, float(lat), float(lng), int(round(float(r))), None, name))

    nC = nP = nV = 0
    with open(os.path.join(OUT, "circles.csv"), "w", encoding="ascii", newline="\n") as fc, open(
        os.path.join(OUT, "polys.csv"), "w", encoding="ascii", newline="\n"
    ) as fp, open(os.path.join(OUT, "index.csv"), "w", encoding="ascii", newline="\n") as fi:
        fc.write("id,lat,lng,r\n")
        fp.write("id,n,lon_lat_pairs\n")
        fi.write("id,kind,lat,lng,r\n")
        for kind, zid, lat, lng, r, ring, name in records:
            fi.write(f"{zid},{0 if kind == 'C' else 1},{lat:.7f},{lng:.7f},{r}\n")
            if kind == "C":
                fc.write(f"{zid},{lat:.7f},{lng:.7f},{r}\n")
                nC += 1
            else:
                parts = []
                for lon, la in ring:
                    parts.append(f"{lon:.7f}")
                    parts.append(f"{la:.7f}")
                fp.write(f"{zid},{len(ring)}," + ",".join(parts) + "\n")
                nP += 1
                nV += len(ring)

    # 保留已有 overlay/deleted；没有则建空表
    ov = os.path.join(OUT, "overlay.csv")
    if not os.path.isfile(ov):
        open(ov, "w", encoding="ascii", newline="\n").write("id,kind,lat,lng,r\n")
    de = os.path.join(OUT, "deleted.csv")
    if not os.path.isfile(de):
        open(de, "w", encoding="ascii", newline="\n").write("id\n")

    print(f"circles={nC} polys={nP} verts={nV} -> {OUT}")


if __name__ == "__main__":
    main()
