#!/usr/bin/env python3
"""Build Mcity's vegetation at a triangle count a real-time renderer can draw.

The problem
-----------
The 18 species are film-grade models. Eight are plain meshes of 4k to 620k triangles. The other
ten are trees built the way a DCC tool builds them: a trunk mesh, plus point instancers that
place every branch with its leaves or needles, 52 to 454 branches a tree. Expanded, one Colorado
Spruce is 15.6 million triangles and the 2009 plants on the site are about 1.7 billion.

usd_to_chrono.py therefore does not expand anything. It writes each branch prototype once and
records where the instancer puts it, as "clumps" on the asset. This script does the expansion
and the reduction together, so nothing that large ever exists on disk or in memory.

How a plant is reduced
----------------------
Every plant gets a triangle budget, split between wood and foliage.

Wood, meaning trunks and branch tubes, is welded by vertex clustering. A tube survives that as
a coarser tube.

Foliage is thousands of separate leaf-shaped cards sharing one texture. Clustering would smear
texture coordinates across unrelated leaves, so cards are dropped whole instead and every
survivor keeps its exact shape. A canopy thinned to a hundredth of its leaves looks dead, so the
survivors are scaled up about their own centres until they cover a similar area. Up close that
reads as oversized leaves. From a road it reads as a tree.

Which treatment a part gets comes from its material name first and from its geometry only when
the name says nothing: these trees model every twig as its own disconnected tube, so bark looks
exactly like a pile of cards to a component count.

Usage:  ./decimate_foliage.py --lod-dir lod_full --out data/mcity/mcity_scene_full.json
"""

import argparse
import json
import math
import os
import zlib

try:
    import numpy as np
except ImportError:
    raise SystemExit("numpy is required:  python3 -m pip install numpy")


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


# --------------------------------------------------------------------------------------------
# Meshes
# --------------------------------------------------------------------------------------------


class Mesh:
    """Positions, texture coordinates and normals, one row per vertex, and triangles into them."""

    def __init__(self, V, VT, VN, F):
        self.V, self.VT, self.VN, self.F = V, VT, VN, F

    @property
    def tris(self):
        return len(self.F)


def read_obj(path):
    """Read an OBJ as written by usd_to_chrono.py into one aligned vertex table.

    That writer welds corners, so a face names the same index for position, texture coordinate
    and normal. Anything else is re-indexed corner by corner into the same shape.
    """
    V, VT, VN, corners = [], [], [], []
    with open(path) as f:
        for line in f:
            if line.startswith("v "):
                p = line.split()
                V.append((float(p[1]), float(p[2]), float(p[3])))
            elif line.startswith("vt "):
                p = line.split()
                VT.append((float(p[1]), float(p[2])))
            elif line.startswith("vn "):
                p = line.split()
                VN.append((float(p[1]), float(p[2]), float(p[3])))
            elif line.startswith("f "):
                for tok in line.split()[1:4]:
                    b = tok.split("/")
                    corners.append((int(b[0]) - 1,
                                    int(b[1]) - 1 if len(b) > 1 and b[1] else -1,
                                    int(b[2]) - 1 if len(b) > 2 and b[2] else -1))
    V = np.array(V, dtype=np.float64).reshape(-1, 3)
    VT = np.array(VT, dtype=np.float64).reshape(-1, 2)
    VN = np.array(VN, dtype=np.float64).reshape(-1, 3)
    C = np.array(corners, dtype=np.int64).reshape(-1, 3)
    if len(C) == 0:
        return Mesh(V[:0], None, None, np.zeros((0, 3), dtype=np.int64))
    aligned = ((C[:, 1] < 0) | (C[:, 1] == C[:, 0])).all() and ((C[:, 2] < 0) | (C[:, 2] == C[:, 0])).all()
    if not aligned:
        uniq, inv = np.unique(C, axis=0, return_inverse=True)
        V = V[uniq[:, 0]]
        VT = VT[uniq[:, 1]] if len(VT) and (uniq[:, 1] >= 0).all() else VT[:0]
        VN = VN[uniq[:, 2]] if len(VN) and (uniq[:, 2] >= 0).all() else VN[:0]
        C = np.stack([inv.ravel()] * 3, axis=1)
    return Mesh(V, VT if len(VT) == len(V) else None, VN if len(VN) == len(V) else None,
                C[:, 0].reshape(-1, 3))


def write_obj(path, mesh, note):
    with open(path, "w") as f:
        f.write(f"# {note}\n")
        f.write("".join("v {:.5f} {:.5f} {:.5f}\n".format(*v) for v in mesh.V))
        if mesh.VT is not None:
            f.write("".join("vt {:.5f} {:.5f}\n".format(*v) for v in mesh.VT))
        if mesh.VN is not None:
            f.write("".join("vn {:.5f} {:.5f} {:.5f}\n".format(*v) for v in mesh.VN))
        if mesh.VT is not None and mesh.VN is not None:
            fmt = "f {0}/{0}/{0} {1}/{1}/{1} {2}/{2}/{2}\n"
        elif mesh.VT is not None:
            fmt = "f {0}/{0} {1}/{1} {2}/{2}\n"
        else:
            fmt = "f {0} {1} {2}\n"
        f.write("".join(fmt.format(a + 1, b + 1, c + 1) for a, b, c in mesh.F))


def compact(mesh):
    """Drop vertices no triangle uses."""
    if len(mesh.F) == 0:
        return Mesh(mesh.V[:0], None if mesh.VT is None else mesh.VT[:0],
                    None if mesh.VN is None else mesh.VN[:0], mesh.F)
    used, inv = np.unique(mesh.F, return_inverse=True)
    return Mesh(mesh.V[used], None if mesh.VT is None else mesh.VT[used],
                None if mesh.VN is None else mesh.VN[used], inv.reshape(-1, 3))


def join(meshes):
    """Concatenate meshes that share a material."""
    meshes = [m for m in meshes if len(m.F)]
    if not meshes:
        return None
    has_vt = all(m.VT is not None for m in meshes)
    has_vn = all(m.VN is not None for m in meshes)
    offsets = np.cumsum([0] + [len(m.V) for m in meshes[:-1]])
    return Mesh(np.concatenate([m.V for m in meshes]),
                np.concatenate([m.VT for m in meshes]) if has_vt else None,
                np.concatenate([m.VN for m in meshes]) if has_vn else None,
                np.concatenate([m.F + o for m, o in zip(meshes, offsets)]))


def placed(mesh, xform):
    """A mesh carried through one instance matrix: 9 numbers row-major, then a translation."""
    lin = np.array(xform[:9], dtype=np.float64).reshape(3, 3)
    V = mesh.V @ lin + np.array(xform[9:12])
    VN = mesh.VN
    if VN is not None:
        # Normals go through the inverse transpose, which for a row vector is the plain inverse
        # applied on the other side.
        VN = VN @ np.linalg.inv(lin).T
        VN = VN / np.maximum(np.linalg.norm(VN, axis=1, keepdims=True), 1e-12)
    F = mesh.F if np.linalg.det(lin) > 0 else mesh.F[:, ::-1]  # a mirror flips the winding
    return Mesh(V, mesh.VT, VN, F)


IDENTITY = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]


def card_ids(mesh):
    """Which connected component each triangle belongs to, by shared vertex (union-find)."""
    parent = list(range(len(mesh.V)))

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    for a, b, c in mesh.F.tolist():
        ra, rb, rc = find(a), find(b), find(c)
        if rb != ra:
            parent[rb] = ra
        if rc != ra:
            parent[rc] = ra
    roots = np.array([find(a) for a in mesh.F[:, 0].tolist()], dtype=np.int64)
    _, ids = np.unique(roots, return_inverse=True)
    return ids


def cluster(mesh, target):
    """Vertex clustering on a uniform grid, sized by bisection to land at or under the target."""
    if len(mesh.F) <= target:
        return mesh
    lo_corner = mesh.V.min(axis=0)
    diag = float(np.linalg.norm(mesh.V.max(axis=0) - lo_corner)) or 1.0

    def attempt(cell):
        keys = np.floor((mesh.V - lo_corner) / cell).astype(np.int64)
        _, inv, counts = np.unique(keys, axis=0, return_inverse=True, return_counts=True)
        inv = inv.ravel()
        F = inv[mesh.F]
        F = F[(F[:, 0] != F[:, 1]) & (F[:, 1] != F[:, 2]) & (F[:, 0] != F[:, 2])]

        def mean(A):
            out = np.zeros((len(counts), A.shape[1]))
            np.add.at(out, inv, A)
            return out / counts[:, None]

        V = mean(mesh.V)
        VT = mean(mesh.VT) if mesh.VT is not None else None
        VN = None
        if mesh.VN is not None:
            VN = mean(mesh.VN)
            VN = VN / np.maximum(np.linalg.norm(VN, axis=1, keepdims=True), 1e-12)
        return Mesh(V, VT, VN, F)

    lo, hi = diag / 4000.0, diag
    best = None
    for _ in range(16):
        mid = math.sqrt(lo * hi)
        m = attempt(mid)
        if len(m.F) > target:
            lo = mid
        else:
            hi = mid
            if len(m.F) >= 4:
                best = m
    return compact(best) if best is not None else compact(attempt(lo))


# --------------------------------------------------------------------------------------------
# Plants
# --------------------------------------------------------------------------------------------

WOOD = ("bark", "trunk", "wood", "branch", "stem", "twig")
CARDS = ("leaf", "leaves", "needle", "flower", "frond", "blossom", "petal", "privet", "grass")

# Fewer triangles than this is not a branch any more.
MIN_BRANCH_TRIS = 12
# How large a surviving leaf may grow, and how much of the lost area it is asked to make up.
# Both were settled by eye against renders. At 6x and three quarters of the area a spruce came out
# as a trunk in a haze of wisps. Welding foliage the way wood is welded was tried too and left a
# skeleton of flat shards. Full coverage with leaves allowed to reach 20x gives a solid crown.
MAX_CARD_SCALE = 20.0
COVERAGE = 1.0
# A grown card is kept no more than this many times longer than it is wide. A needle is thirty
# times longer than wide, and grown evenly a spruce turns into a heap of metre-long sticks.
MAX_CARD_ASPECT = 3.0


def grow_cards(mesh, grow):
    """Scale every card of a mesh about its own centre so that it covers grow^2 times the area.

    Growth goes into width before length for anything long and thin, which is what turns a
    surviving needle into a tuft instead of a stick. Each card stays where its branch is.
    """
    ids = card_ids(mesh)
    vert_card = np.zeros(len(mesh.V), dtype=np.int64)
    vert_card[mesh.F.ravel()] = np.repeat(ids, 3)
    V = mesh.V.copy()
    for c in range(int(ids.max()) + 1):
        sel = np.nonzero(vert_card == c)[0]
        pts = mesh.V[sel]
        centre = pts.mean(axis=0)
        rel = pts - centre
        # Principal axes of the card: across its thickness, its width, then its length.
        _, axes = np.linalg.eigh(rel.T @ rel)
        local = rel @ axes
        width = np.ptp(local[:, 1]) or 1e-9
        length = np.ptp(local[:, 2]) or 1e-9
        aspect = length / width
        if aspect > MAX_CARD_ASPECT:
            g_width = math.sqrt(grow * grow * aspect / MAX_CARD_ASPECT)
            g_length = grow * grow / g_width
            if g_length < 1.0:
                g_length, g_width = 1.0, grow * grow
        else:
            g_width = g_length = grow
        local = local * np.array([min(g_width, g_length), g_width, g_length])
        V[sel] = centre + local @ axes.T
    return Mesh(V, mesh.VT, mesh.VN, mesh.F)


def kind_of(part, mesh):
    low = (part["name"] + " " + os.path.basename(part["mesh"])).lower()
    if any(w in low for w in WOOD):
        return "wood"
    if any(w in low for w in CARDS):
        return "cards"
    n = int(card_ids(mesh).max()) + 1 if len(mesh.F) else 0
    return "cards" if (n > 50 and len(mesh.F) / n < 200) else "wood"


def reduce_plant(asset, data_dir, wood_budget, card_budget, rng):
    """One plant, expanded and reduced. Returns {material name: (part template, Mesh)}."""
    items = []  # (part, mesh, kind, xforms, instanced)
    for part in asset["parts"]:
        path = os.path.join(data_dir, part["mesh"])
        if os.path.exists(path):
            mesh = read_obj(path)
            if len(mesh.F):
                items.append((part, mesh, kind_of(part, mesh), [IDENTITY], False))
    for clump in asset.get("clumps", []):
        for part in clump["parts"]:
            path = os.path.join(data_dir, part["mesh"])
            if os.path.exists(path):
                mesh = read_obj(path)
                if len(mesh.F):
                    items.append((part, mesh, kind_of(part, mesh), clump["xforms"], True))

    before = sum(len(m.F) * len(x) for _, m, _, x, _ in items)
    out = {}

    def emit(part, mesh):
        out.setdefault(part["name"], (part, []))[1].append(mesh)

    # Wood. The trunk is what the eye lands on, so where a plant has both a trunk and instanced
    # branches the trunk is given a fixed share of the budget instead of its share by triangle
    # count, which for an oak with 90 heavy branches would be a fraction of a percent.
    wood = [it for it in items if it[2] == "wood"]
    trunk = [it for it in wood if not it[4]]
    limbs = [it for it in wood if it[4]]
    trunk_budget = wood_budget * (0.35 if limbs else 1.0)
    limb_budget = wood_budget - trunk_budget if trunk else wood_budget
    for group, budget in ((trunk, trunk_budget), (limbs, limb_budget)):
        total = sum(len(m.F) * len(x) for _, m, _, x, _ in group)
        ratio = min(1.0, budget / total) if total else 0.0
        for part, mesh, _, xforms, _ in group:
            want = len(mesh.F) * ratio
            keep = 1.0
            if want < MIN_BRANCH_TRIS < len(mesh.F):
                # Too many branches for the budget: keep some whole rather than all as slivers.
                keep, want = want / MIN_BRANCH_TRIS, MIN_BRANCH_TRIS
            small = cluster(mesh, max(4, int(want))) if ratio < 1.0 else mesh
            for xform in xforms:
                if keep >= 1.0 or rng.random() < keep:
                    emit(part, placed(small, xform))

    # Foliage.
    cards = [it for it in items if it[2] == "cards"]
    total = sum(len(m.F) * len(x) for _, m, _, x, _ in cards)
    fraction = min(1.0, card_budget / total) if total else 0.0
    grow = min(MAX_CARD_SCALE, max(1.0, COVERAGE / math.sqrt(fraction))) if fraction > 0 else 1.0
    for part, mesh, _, xforms, _ in cards:
        if fraction <= 0.0:
            continue
        if fraction >= 1.0:
            for xform in xforms:
                emit(part, placed(mesh, xform))
            continue
        ids = card_ids(mesh)
        order = np.argsort(ids, kind="stable")
        bounds = np.searchsorted(ids[order], np.arange(ids.max() + 2))
        ncards = len(bounds) - 1
        want = fraction * ncards  # cards per instance, usually a fraction of one
        for xform in xforms:
            n = int(want) + (1 if rng.random() < want - int(want) else 0)
            if n == 0:
                continue
            chosen = rng.choice(ncards, size=min(n, ncards), replace=False)
            faces = np.concatenate([order[bounds[c]:bounds[c + 1]] for c in chosen])
            sub = compact(Mesh(mesh.V, mesh.VT, mesh.VN, mesh.F[faces]))
            if grow > 1.0:
                sub = grow_cards(sub, grow)
            emit(part, placed(sub, xform))

    merged = {}
    for name, (part, meshes) in out.items():
        mesh = join(meshes)
        if mesh is not None:
            merged[name] = (part, mesh)
    return merged, before


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--dir", default=chrono_data_dir())
    ap.add_argument("--in", dest="src", default=None)
    ap.add_argument("--out", dest="out", default=None)
    # Budgets are triangles per plant. They are the whole quality-against-cost dial: the totals
    # this script prints are what the renderer will be asked to draw every frame.
    #
    # The defaults are sized for stock Chrono::VSG, which draws every triangle of the scene every
    # frame and again for each shadow map. Measured on an M4 Pro with shadows on, a scene of up
    # to about 6 M triangles draws at 40 frames a second and one of 8 M at 12, so each level is
    # budgeted to land under 6 M including the 1.4 M of the scene itself.
    ap.add_argument("--tree-wood", type=int, default=2500, help="trunk and branches of a tree")
    ap.add_argument("--tree-leaves", type=int, default=4500, help="leaves or needles of a tree")
    ap.add_argument("--shrub-wood", type=int, default=300, help="stems of a shrub")
    ap.add_argument("--shrub-leaves", type=int, default=700, help="leaves or flowers of a shrub")
    ap.add_argument("--grass", type=int, default=300, help="one patch of grass")
    ap.add_argument("--no-leaves", action="store_true",
                    help="bare branches: trees and shrubs keep their wood only. Grass stays.")
    ap.add_argument("--shrubs", default="Forsythia,Yew,Meadowlark,Holly,Blue_Berry_Elder",
                    help="species counted as shrubs. Everything else that is not grass is a tree.")
    ap.add_argument("--drop-shrubs", action="store_true",
                    help="remove shrub and grass placements entirely, keeping only trees")
    # Each variant needs its own directory. Sharing one means generating a leafy set silently
    # overwrites the bare set's meshes while its manifest still points at them, so a config that
    # was verified earlier quietly starts rendering something else.
    ap.add_argument("--lod-dir", default="assets_lod", help="output mesh directory, under --dir")
    ap.add_argument("--seed", type=int, default=5)
    args = ap.parse_args()

    src = args.src or os.path.join(args.dir, "mcity_scene_foliage.json")
    out = args.out or os.path.join(args.dir, "mcity_scene_lod.json")
    with open(src) as f:
        man = json.load(f)
    assets = man["assets"]

    lod_dir = os.path.join(args.dir, args.lod_dir)
    os.makedirs(lod_dir, exist_ok=True)

    shrubs = {x for x in args.shrubs.split(",") if x}

    def species(asset):
        return asset["name"].split("__")[0]

    def klass(asset):
        sp = species(asset)
        return "grass" if sp.startswith("Grass") else ("shrub" if sp in shrubs else "tree")

    fol = [i for i in man["instances"] if i["group"] == "Foliage_Instanced"]
    other = [i for i in man["instances"] if i["group"] != "Foliage_Instanced"]
    if args.drop_shrubs:
        n = len(fol)
        fol = [i for i in fol if klass(assets[i["asset"]]) == "tree"]
        print(f"  dropped {n - len(fol)} shrub and grass placements, {len(fol)} trees remain")

    count = {}
    for i in fol:
        count[i["asset"]] = count.get(i["asset"], 0) + 1

    print(f"\n  {'species':18s} {'kind':6s} {'placed':>6s} {'source tris':>12s} {'now':>8s}  {'drawn':>10s}")
    drawn = source = 0
    for ai in sorted(count, key=lambda k: -count[k]):
        a = assets[ai]
        k = klass(a)
        if k == "grass":
            wood_budget, card_budget = 0, args.grass
        elif k == "shrub":
            wood_budget, card_budget = args.shrub_wood, 0 if args.no_leaves else args.shrub_leaves
        else:
            wood_budget, card_budget = args.tree_wood, 0 if args.no_leaves else args.tree_leaves
        # Seeded per species, so a plant comes out the same whatever else is in the run.
        rng = np.random.default_rng(args.seed + zlib.crc32(a["name"].encode()))
        merged, before = reduce_plant(a, args.dir, wood_budget, card_budget, rng)

        parts = []
        for name, (template, mesh) in merged.items():
            fname = f"{a['name']}__{''.join(c if c.isalnum() or c in '-_.' else '_' for c in name)}.obj"
            write_obj(os.path.join(lod_dir, fname), mesh,
                      f"Mcity vegetation, reduced from {before} triangles by decimate_foliage.py")
            part = dict(template)
            part["mesh"] = os.path.join(args.lod_dir, fname)
            part["tris"] = int(len(mesh.F))
            parts.append(part)
        a["parts"] = parts
        now = sum(p["tris"] for p in parts)
        drawn += now * count[ai]
        source += before * count[ai]
        print(f"  {species(a):18s} {k:6s} {count[ai]:6d} {before:12,d} {now:8,d}  {now * count[ai]:10,d}")

    # The instancer records have been baked into the parts above. No loader needs them.
    for a in assets:
        a.pop("clumps", None)
    man["instances"] = other + fol
    # A species this level leaves out keeps its slot, so asset indices stay put, but must stop
    # naming its source meshes: anything a manifest names gets packaged, and these are the
    # 300k-triangle originals.
    used = {i["asset"] for i in man["instances"]}
    for ai, a in enumerate(assets):
        if ai not in used:
            a["parts"] = []

    with open(out, "w") as f:
        json.dump(man, f, indent=1)
    print(f"\n  vegetation: {source / 1e6:,.0f} M triangles as authored, {drawn / 1e6:.2f} M to draw "
          f"({len(fol)} plants)")
    print(f"  wrote {out}")


if __name__ == "__main__":
    main()
