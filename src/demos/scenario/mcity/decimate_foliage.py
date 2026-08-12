#!/usr/bin/env python3
"""Make Mcity's foliage cheap enough to render, without changing what it is.

The problem
-----------
The 18 vegetation species are Omniverse scan-grade models: 80k-620k triangles each, where a
real-time engine uses 2-5k with alpha-mapped leaf cards. Mcity places 2009 of them, which is 264M
triangles. Measured throughput here is ~95M triangles/second, so that is 2.7 s per frame.

Distance streaming does not rescue it. Sampling ego positions along the road, even a 40 m radius
leaves a 95th percentile of 104M triangles, because the cost is local density -- a single hedgerow
within 40 m -- rather than the size of the site.

So the geometry itself has to shrink, by roughly 20x.

Leaves only
-----------
Leaves, needles and flowers are thousands of disconnected cards sharing an atlas texture. Merging
their vertices would average texture coordinates across unrelated leaves and smear the atlas, so
those parts instead lose whole cards: every surviving leaf keeps its exact geometry and UVs, and
the canopy just gets thinner. Bark and trunks are connected tubes with continuous UVs, where
vertex clustering collapses detail cleanly.

Which treatment a part gets is decided by measuring it -- many small disconnected components means
cards -- rather than by trusting its name.

Usage:  ./decimate_foliage.py [--target-tris 6000] [--keep-fraction 1.0]
"""

import argparse
import json
import math
import os
import random



def chrono_data_dir():
    """<chrono>/data/mcity, found by walking up to the source root.

    Not a relative hop: these scripts have moved once already and a counted "../.." silently
    resolved to the wrong directory, writing the scene where nothing would look for it.
    """
    d = os.path.dirname(os.path.abspath(__file__))
    while d != "/" and not os.path.isdir(os.path.join(d, "src", "chrono")):
        d = os.path.dirname(d)
    if not os.path.isdir(os.path.join(d, "src", "chrono")):
        raise SystemExit("could not locate the Chrono source root above " + __file__)
    return os.path.join(d, "data", "mcity")

def read_obj(path):
    """Vertices, texcoords, normals and faces of corner triples (v, vt, vn), all 0-based."""
    V, VT, VN, F = [], [], [], []
    for line in open(path):
        if line.startswith("v "):
            f = line.split()
            V.append((float(f[1]), float(f[2]), float(f[3])))
        elif line.startswith("vt "):
            f = line.split()
            VT.append((float(f[1]), float(f[2])))
        elif line.startswith("vn "):
            f = line.split()
            VN.append((float(f[1]), float(f[2]), float(f[3])))
        elif line.startswith("f "):
            corners = []
            for tok in line.split()[1:4]:
                bits = tok.split("/")
                vi = int(bits[0]) - 1
                ti = int(bits[1]) - 1 if len(bits) > 1 and bits[1] else -1
                ni = int(bits[2]) - 1 if len(bits) > 2 and bits[2] else -1
                corners.append((vi, ti, ni))
            F.append(tuple(corners))
    return V, VT, VN, F


def write_obj(path, V, VT, VN, F, note):
    """Write only the vertices the surviving faces reference."""
    vmap, tmap, nmap = {}, {}, {}
    for tri in F:
        for vi, ti, ni in tri:
            if vi not in vmap:
                vmap[vi] = len(vmap)
            if ti >= 0 and ti not in tmap:
                tmap[ti] = len(tmap)
            if ni >= 0 and ni not in nmap:
                nmap[ni] = len(nmap)
    inv_v = sorted(vmap, key=vmap.get)
    inv_t = sorted(tmap, key=tmap.get)
    inv_n = sorted(nmap, key=nmap.get)
    with open(path, "w") as f:
        f.write(f"# {note}\n")
        for i in inv_v:
            f.write("v {:.5f} {:.5f} {:.5f}\n".format(*V[i]))
        for i in inv_t:
            f.write("vt {:.5f} {:.5f}\n".format(*VT[i]))
        for i in inv_n:
            f.write("vn {:.5f} {:.5f} {:.5f}\n".format(*VN[i]))
        for tri in F:
            out = []
            for vi, ti, ni in tri:
                a = vmap[vi] + 1
                b = str(tmap[ti] + 1) if ti >= 0 else ""
                c = str(nmap[ni] + 1) if ni >= 0 else ""
                out.append(f"{a}/{b}/{c}" if c else (f"{a}/{b}" if b else f"{a}"))
            f.write("f " + " ".join(out) + "\n")


def components(V, F):
    """Connected components of the face graph, by shared position index (union-find)."""
    parent = list(range(len(V)))

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    def union(a, b):
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[rb] = ra

    for tri in F:
        a, b, c = tri[0][0], tri[1][0], tri[2][0]
        union(a, b)
        union(a, c)

    groups = {}
    for tri in F:
        groups.setdefault(find(tri[0][0]), []).append(tri)
    return list(groups.values())


def drop_cards(V, F, target):
    """Keep whole connected components at random until the triangle target is reached.

    Random rather than spatial: foliage is meant to look unordered, and taking, say, the lowest
    cards would hollow out one side of the canopy.
    """
    comps = components(V, F)
    random.shuffle(comps)
    kept, n = [], 0
    for c in comps:
        if n + len(c) > target and kept:
            continue
        kept.extend(c)
        n += len(c)
    return kept, len(comps)


def cluster(V, VT, VN, F, target):
    """Vertex clustering on a uniform grid, sized by bisection to land near the target."""
    xs = [v[0] for v in V]; ys = [v[1] for v in V]; zs = [v[2] for v in V]
    diag = math.dist((min(xs), min(ys), min(zs)), (max(xs), max(ys), max(zs))) or 1.0

    def attempt(cell):
        rep = {}
        for i, v in enumerate(V):
            key = (int(v[0] // cell), int(v[1] // cell), int(v[2] // cell))
            rep.setdefault(key, []).append(i)
        # One representative vertex per occupied cell, at the centroid of what it absorbed.
        newV, of = [], {}
        for key, members in rep.items():
            cx = sum(V[i][0] for i in members) / len(members)
            cy = sum(V[i][1] for i in members) / len(members)
            cz = sum(V[i][2] for i in members) / len(members)
            idx = len(newV)
            newV.append((cx, cy, cz))
            for i in members:
                of[i] = idx
        out = []
        for tri in F:
            a, b, c = of[tri[0][0]], of[tri[1][0]], of[tri[2][0]]
            if a == b or b == c or a == c:
                continue  # collapsed to a sliver
            # Keep the original corner attributes of the first corner mapping to each new vertex.
            out.append(((a, tri[0][1], tri[0][2]), (b, tri[1][1], tri[1][2]), (c, tri[2][1], tri[2][2])))
        return newV, out

    lo, hi = diag / 2000.0, diag / 2.0
    best = None
    for _ in range(14):
        mid = math.sqrt(lo * hi)
        nv, nf = attempt(mid)
        if len(nf) > target:
            lo = mid
        else:
            hi = mid
            best = (nv, nf)
    return best if best else attempt(hi)


def main():
    ap = argparse.ArgumentParser()
    here = os.path.dirname(os.path.abspath(__file__))
    data = chrono_data_dir()
    ap.add_argument("--dir", default=data)
    ap.add_argument("--in", dest="src", default=None)
    ap.add_argument("--out", dest="out", default=None)
    # A fraction, not an absolute budget. An absolute target starves species whose bark alone
    # exceeds it -- Forsythia's branches are 61k triangles against a 6k target -- which leaves a
    # bare tree with a few leaves stranded in mid-air. A fraction thins every canopy evenly and
    # keeps the tree recognisable.
    ap.add_argument("--leaf-fraction", type=float, default=0.3,
                    help="fraction of leaf cards to keep (0 = bare branches)")
    # Conifers model their branches inside the needle mesh rather than the bark mesh -- White
    # Pine's bark is 878 triangles and Colorado Spruce's 4067, against 86k of limbs on a Hawthorn.
    # Stripping the canopy therefore leaves those species as bare trunks standing in the scene, so
    # with no branches to keep they are dropped instead. 0 keeps them, trunks and all.
    ap.add_argument("--min-branch-tris", type=int, default=8000,
                    help="in bare mode, drop species whose branches fall below this (0 = keep all)")
    ap.add_argument("--shrubs", default="Forsythia,Yew,Meadowlark,Holly,Blue_Berry_Elder,"
                                        "Grass_Trimmed_A,Grass_Trimmed_B",
                    help="species that --drop-shrubs removes")
    ap.add_argument("--drop-shrubs", action="store_true",
                    help="remove shrub and grass placements entirely, keeping only trees")
    ap.add_argument("--keep-fraction", type=float, default=1.0,
                    help="fraction of foliage placements to keep (density)")
    # Each variant needs its own directory. Sharing one means generating a leafy set silently
    # overwrites the bare set's meshes while its manifest still points at them, so a config that
    # was verified earlier quietly starts rendering something else.
    ap.add_argument("--lod-dir", default="assets_lod", help="output mesh directory, under --dir")
    ap.add_argument("--seed", type=int, default=5)
    args = ap.parse_args()
    random.seed(args.seed)

    src = args.src or os.path.join(args.dir, "mcity_scene_foliage.json")
    out = args.out or os.path.join(args.dir, "mcity_scene_lod.json")
    man = json.load(open(src))
    assets = man["assets"]

    lod_dir = os.path.join(args.dir, args.lod_dir)
    os.makedirs(lod_dir, exist_ok=True)

    foliage_assets = {i["asset"] for i in man["instances"] if i["group"] == "Foliage_Instanced"}
    print(f"  decimating {len(foliage_assets)} foliage assets, keeping {args.leaf_fraction:.0%} of leaf cards")

    before = after = 0
    report = []
    original_solid = {}   # species -> bark triangles before decimation
    shrubs = {x for x in args.shrubs.split(",") if x}
    for ai in sorted(foliage_assets):
        a = assets[ai]
        parts = a["parts"]
        sizes = []
        for p in parts:
            path = os.path.join(args.dir, p["mesh"])
            sizes.append(sum(1 for l in open(path) if l[:2] == "f ") if os.path.exists(path) else 0)
        total = sum(sizes) or 1

        # Classify parts first. Leaf cards can be thinned freely -- a sparser canopy still sits on
        # the branches -- but the bark must not move: clustering it collapses twigs and displaces
        # vertices, and every leaf that was attached to a vanished twig is left hanging in mid-air.
        # So bark is kept intact and the whole reduction is taken out of the cards.
        kinds = []
        for p, n in zip(parts, sizes):
            path = os.path.join(args.dir, p["mesh"])
            if not os.path.exists(path) or n == 0:
                kinds.append("skip")
                continue
            # Name first, geometry only as a fallback.
            #
            # The geometry heuristic on its own is wrong here: these trees model every branch as
            # a separate disconnected tube, so a bark mesh has hundreds of components and looks
            # exactly like a pile of leaf cards. Thinning it deletes branches and strands the
            # leaves that grew on them in mid-air. The part names are unambiguous, so use them.
            low = os.path.basename(p["mesh"]).lower()
            if any(w in low for w in ("bark", "trunk", "wood", "branch", "stem", "twig")):
                kinds.append("solid")
            elif any(w in low for w in ("leaf", "leaves", "needle", "flower", "frond",
                                        "blossom", "petal", "privet")):
                kinds.append("cards")
            else:
                V, VT, VN, F = read_obj(path)
                comps = components(V, F)
                kinds.append("cards" if (len(comps) > 50 and len(F) / len(comps) < 200) else "solid")

        # Record how much branch geometry this species has before anything is removed, so the
        # trunk-only test below is not confused by decimation.
        sp_name = a["name"].split("__")[0]
        original_solid[sp_name] = max(
            original_solid.get(sp_name, 0),
            sum(n for n, k in zip(sizes, kinds) if k == "solid"))

        for p, n, kind in zip(parts, sizes, kinds):
            if kind == "skip":
                continue
            path = os.path.join(args.dir, p["mesh"])
            V, VT, VN, F = read_obj(path)
            before += len(F)

            if kind == "cards" and args.leaf_fraction <= 0.0:
                # Bare branches: drop the canopy entirely. Useful as a winter/skeleton variant
                # and as the cheapest possible foliage.
                ncomp = 0
                F = []
            elif kind == "cards" and args.leaf_fraction < 1.0:
                F, ncomp = drop_cards(V, F, max(200, int(n * args.leaf_fraction)))
                report.append((a["name"].split("__")[0], os.path.basename(p["mesh"]), kind, n, len(F), ncomp))
            else:
                report.append((a["name"].split("__")[0], os.path.basename(p["mesh"]), kind, n, len(F), 0))
            after += len(F)

            name = os.path.basename(p["mesh"])
            dst = os.path.join(lod_dir, name)
            write_obj(dst, V, VT, VN, F, f"foliage LOD from {name}: {n} -> {len(F)} tris ({kind})")
            p["mesh"] = os.path.join(args.lod_dir, name)

    fol = [i for i in man["instances"] if i["group"] == "Foliage_Instanced"]
    other = [i for i in man["instances"] if i["group"] != "Foliage_Instanced"]

    # With the canopy gone, a species whose branches lived in that canopy is now a trunk.
    #
    # Measured on the bark as authored, so the test says something about the asset rather than
    # about whatever this run happened to remove.
    if args.leaf_fraction <= 0.0 and args.min_branch_tris > 0:
        poles = set()
        for sp, orig in original_solid.items():
            if orig < args.min_branch_tris:
                poles.add(sp)
        if poles:
            before_n = len(fol)
            fol = [i for i in fol if assets[i["asset"]]["name"].split("__")[0] not in poles]
            print(f"  dropped {before_n - len(fol)} trunk-only placements "
                  f"({', '.join(sorted(poles))}): their branches are modelled in the canopy")
    if args.drop_shrubs:
        before_n = len(fol)
        fol = [i for i in fol
               if assets[i["asset"]]["name"].split("__")[0] not in shrubs]
        print(f"  dropped {before_n - len(fol)} shrub/grass placements, {len(fol)} trees remain")
    if args.keep_fraction < 1.0:
        random.shuffle(fol)
        fol = fol[: int(len(fol) * args.keep_fraction)]
    man["instances"] = other + fol

    json.dump(man, open(out, "w"), indent=1)
    print(f"\n  {'species':18s} {'part':34s} {'kind':6s} {'from':>8} {'to':>8} {'comps':>7}")
    for sp, part, kind, n0, n1, nc in report:
        print(f"  {sp:18s} {part[:34]:34s} {kind:6s} {n0:>8} {n1:>8} {nc:>7}")
    print(f"\n  foliage geometry {before/1e6:.2f} M -> {after/1e6:.2f} M triangles "
          f"({before/max(after,1):.1f}x)")
    print(f"  foliage placements kept: {len(fol)}")
    print(f"  wrote {out}")


if __name__ == "__main__":
    main()
