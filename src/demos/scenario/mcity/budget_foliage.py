#!/usr/bin/env python3
"""Pick as much foliage as a triangle budget allows, nearest the road first.

Why a budget rather than all of it
----------------------------------
Mcity's 2009 foliage placements draw 264.5M triangles. Measured on this machine the renderer
sustains roughly 95M triangles per second, so that is 2.7 s per frame -- not slow, unusable, and
enough to make the desktop stop responding. The same measurement puts a 30 FPS budget at about
3M triangles.

The assets are the reason, not the count: these are Omniverse scan-grade models at 80k-400k
triangles each, where a real-time engine would use 2-5k with alpha-mapped leaf cards. CARLA gets
away with placing all of them because Unreal supplies instanced draws, baked LODs and impostors;
Chrono has none of that yet. Until it does, the honest options are fewer trees or smaller trees,
and this script does the first.

Selection favours trees you will actually drive past: instances are sorted by distance to the
road surface and taken until the budget is spent, so the spend goes on what is visible from a
vehicle rather than on the far corners of the site.

Usage:  ./budget_foliage.py [--budget-mtris 3.0] [--out mcity_scene_trees.json]
"""

import argparse
import json
import math
import os


def obj_triangles(path, cache):
    if path not in cache:
        cache[path] = sum(1 for line in open(path) if line[:2] == "f ") if os.path.exists(path) else 0
    return cache[path]


def obj_xy(path, step=37):
    """A sparse sample of a mesh's vertices in local XY, enough to measure distance against."""
    pts = []
    for i, line in enumerate(open(path)):
        if line[:2] == "v " and i % step == 0:
            f = line.split()
            pts.append((float(f[1]), float(f[2])))
    return pts


def main():
    ap = argparse.ArgumentParser()
    here = os.path.dirname(os.path.abspath(__file__))
    data = os.path.abspath(os.path.join(here, "..", "..", "data", "mcity"))
    ap.add_argument("--dir", default=data)
    ap.add_argument("--budget-mtris", type=float, default=3.0,
                    help="million triangles of foliage to allow (default 3.0, about 30 FPS)")
    ap.add_argument("--in", dest="src", default=None)
    ap.add_argument("--out", dest="out", default=None)
    args = ap.parse_args()

    src = args.src or os.path.join(args.dir, "mcity_scene_foliage.json")
    out = args.out or os.path.join(args.dir, "mcity_scene_trees.json")
    man = json.load(open(src))
    assets = man["assets"]
    cache = {}

    def cost(inst):
        return sum(obj_triangles(os.path.join(args.dir, p["mesh"]), cache)
                   for p in assets[inst["asset"]]["parts"])

    foliage = [i for i in man["instances"] if i["group"] == "Foliage_Instanced"]
    other = [i for i in man["instances"] if i["group"] != "Foliage_Instanced"]
    if not foliage:
        raise SystemExit(f"{src} contains no foliage; convert with --exclude-groups \"\"")

    # Road surface, as a coarse occupancy grid. A grid rather than a nearest-neighbour structure
    # because a 2 m cell is finer than the question being asked and costs nothing to build.
    CELL = 4.0
    road = set()
    for inst in other:
        a = assets[inst["asset"]]
        if not a["name"].startswith(("SM_Roads", "SM_Sidewalks", "SM_Curbs")):
            continue
        for part in a["parts"]:
            p = os.path.join(args.dir, part["mesh"])
            if not os.path.exists(p):
                continue
            ox, oy = inst["pos"][0], inst["pos"][1]
            for x, y in obj_xy(p):
                road.add((int((x + ox) // CELL), int((y + oy) // CELL)))
    print(f"  road occupancy cells: {len(road)}")

    def distance_rank(inst):
        """Rings outward from the tree until a road cell is found; capped so far trees just sort last."""
        cx, cy = int(inst["pos"][0] // CELL), int(inst["pos"][1] // CELL)
        for r in range(0, 26):
            for dx in range(-r, r + 1):
                for dy in (-r, r) if r else (0,):
                    if (cx + dx, cy + dy) in road:
                        return r
            for dy in range(-r + 1, r):
                for dx in (-r, r):
                    if (cx + dx, cy + dy) in road:
                        return r
        return 99

    scored = []
    for inst in foliage:
        c = cost(inst)
        if c <= 0:
            continue
        scored.append((distance_rank(inst), c, inst))
    # Distance is bucketed rather than exact, then cheapest-first inside a bucket. Sorting on
    # exact distance instead spends the whole budget on whichever handful of trees happens to be
    # closest, and at 77k-400k triangles each that is a dozen shrubs. Coarsening the ordering
    # buys many more visible trees for the same triangles, which is what actually reads as a
    # planted site from a moving vehicle.
    scored.sort(key=lambda t: (t[0] // 4, t[1]))

    budget = args.budget_mtris * 1e6
    kept, spent = [], 0
    for rank, c, inst in scored:
        if spent + c > budget:
            continue
        kept.append(inst)
        spent += c

    by_species = {}
    for inst in kept:
        sp = assets[inst["asset"]]["name"].split("__")[0]
        by_species[sp] = by_species.get(sp, 0) + 1

    base = sum(cost(i) for i in other)
    man["instances"] = other + kept
    json.dump(man, open(out, "w"), indent=1)

    print(f"  foliage kept: {len(kept)}/{len(foliage)} instances, {spent/1e6:.2f} M triangles")
    for sp, n in sorted(by_species.items(), key=lambda kv: -kv[1]):
        print(f"    {n:5d}  {sp}")
    total = (base + spent) / 1e6
    print(f"  scene draws {total:.2f} M triangles total "
          f"(~{95.0/max(total,0.01):.0f} FPS at the measured 95 M tri/s)")
    print(f"  wrote {out}")


if __name__ == "__main__":
    main()
