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

Wood is handled two ways. A trunk is welded by vertex clustering and survives as a coarser
trunk. An instanced branch is a thousand separate twig tubes and a few real stems, which no
welding reduces to a hundred triangles, so its largest tubes are kept whole and its stems are
redrawn as tapered sticks.

Foliage is thousands of separate leaf-shaped cards sharing one texture. Clustering would smear
texture coordinates across unrelated leaves, so cards are dropped whole instead and every
survivor stays a leaf: its convex outline, with its own texture coordinates. A canopy thinned to
a hundredth of its leaves looks dead, so the survivors are enlarged about their own centres to
win back cover, up to a limit set by the size of the plant.

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


def twig_thickness(mesh):
    """About how thick the thicker tubes of a wood mesh are, from its own triangles.

    A tube is rings of triangles that run long along its axis and short around it, so the
    shortest edge of a triangle is about the tube's radius. Three times the 90th percentile of
    that is a size no honest piece of this wood exceeds. The longest edges say nothing: they
    follow the length of a twig, not its girth.
    """
    if not len(mesh.F):
        return math.inf
    a, b, c = mesh.V[mesh.F[:, 0]], mesh.V[mesh.F[:, 1]], mesh.V[mesh.F[:, 2]]
    shortest = np.minimum(np.minimum(np.linalg.norm(b - a, axis=1), np.linalg.norm(c - b, axis=1)),
                          np.linalg.norm(a - c, axis=1))
    return 3.0 * float(np.percentile(shortest, 90))


def cluster(mesh, target, max_thickness=math.inf):
    """Vertex clustering on a uniform grid, sized by bisection to land at or under the target.

    max_thickness removes the shards. Welding a spray of thin twigs on a coarse grid collapses
    most of them to slivers, which is fine, but where three twigs fall in three neighbouring
    cells it also leaves a broad flat triangle strung between them. Through a canopy those read
    as dark shards. A triangle whose height is more than the wood it came from was thick cannot
    be part of a tube, so it is dropped.
    """
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
        if len(F) and math.isfinite(max_thickness):
            a, b, c = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]
            longest = np.maximum(np.maximum(np.linalg.norm(b - a, axis=1), np.linalg.norm(c - b, axis=1)),
                                 np.linalg.norm(a - c, axis=1))
            height = np.linalg.norm(np.cross(b - a, c - a), axis=1) / np.maximum(longest, 1e-12)
            F = F[height <= max_thickness]
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

# Bumped when the reduction changes what a plant looks like. 1 enlarged whole leaves without a
# limit tied to the plant. 2 draws leaves as outlines and caps their size.
VEGETATION_VERSION = 2

WOOD = ("bark", "trunk", "wood", "branch", "stem", "twig")
CARDS = ("leaf", "leaves", "needle", "flower", "frond", "blossom", "petal", "privet", "grass")

# Fewer triangles than this is not a branch any more.
MIN_BRANCH_TRIS = 12
# A surviving leaf is drawn as its convex outline, in at most this many triangles. The source
# spends 20 to 60 triangles on the lobes of one leaf. Spending 4 buys several times as many
# leaves for the same budget, and many modest leaves make a better crown than a few vast ones.
CARD_TRIS = 4
# How far a leaf may be enlarged to make up for the ones removed, and how much of the lost area
# it is asked to cover.
MAX_CARD_SCALE = 8.0
COVERAGE = 1.0
# A grown card is kept no more than this many times longer than it is wide. A needle is thirty
# times longer than wide, and grown evenly a spruce turns into a heap of metre-long sticks.
MAX_CARD_ASPECT = 3.0
# The largest a grown card may be, as a share of the height of its plant. Without a limit tied
# to the plant, a thin canopy asks for 20x and gets leaves half the size of the tree.
MAX_CARD_SHARE = {"tree": 0.05, "shrub": 0.09, "grass": 0.30}


def convex_outline(points):
    """Indices of the convex hull of 2D points, counter-clockwise (monotone chain)."""
    order = sorted(range(len(points)), key=lambda i: (points[i][0], points[i][1]))

    def cross(o, a, b):
        return ((points[a][0] - points[o][0]) * (points[b][1] - points[o][1]) -
                (points[a][1] - points[o][1]) * (points[b][0] - points[o][0]))

    lower, upper = [], []
    for i in order:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], i) <= 0:
            lower.pop()
        lower.append(i)
    for i in reversed(order):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], i) <= 0:
            upper.pop()
        upper.append(i)
    return lower[:-1] + upper[:-1]


def shape_card(mesh, faces, grow, max_size):
    """One leaf: reduced to its outline, then enlarged about its own centre.

    Growth goes into width before length for anything long and thin, which is what turns a
    surviving needle into a tuft instead of a stick, and stops at max_size in either direction.
    The card stays where its branch is. Returns a small Mesh of its own.
    """
    F = mesh.F[faces]
    used, inv = np.unique(F, return_inverse=True)
    F = inv.reshape(-1, 3)
    pts = mesh.V[used]
    centre = pts.mean(axis=0)
    rel = pts - centre
    # Principal axes of the card: across its thickness, its width, then its length.
    _, axes = np.linalg.eigh(rel.T @ rel)
    local = rel @ axes
    width = float(np.ptp(local[:, 1])) or 1e-9
    length = float(np.ptp(local[:, 2])) or 1e-9

    if len(F) > CARD_TRIS and width > 1e-6:
        hull = convex_outline(local[:, 1:3].tolist())
        if len(hull) > CARD_TRIS + 2:
            hull = [hull[int(round(k))] for k in np.linspace(0, len(hull), CARD_TRIS + 2, endpoint=False)]
        if len(hull) >= 3:
            F = np.array([(hull[0], hull[k], hull[k + 1]) for k in range(1, len(hull) - 1)], dtype=np.int64)

    aspect = length / width
    if aspect > MAX_CARD_ASPECT:
        g_width = math.sqrt(grow * grow * aspect / MAX_CARD_ASPECT)
        g_length = max(1.0, grow * grow / g_width)
    else:
        g_width = g_length = grow
    g_length = max(1.0, min(g_length, max_size / length))
    g_width = max(1.0, min(g_width, max_size / width))
    local = local * np.array([min(g_width, g_length), g_width, g_length])

    card = Mesh(centre + local @ axes.T,
                None if mesh.VT is None else mesh.VT[used],
                None if mesh.VN is None else mesh.VN[used], F)
    return compact(card)


def tubes(mesh, budget):
    """A branch reduced by keeping its largest tubes, in at most budget triangles.

    Measured, a branch prototype is about a thousand separate tubes. Nearly all are twigs a few
    millimetres thick that the source already draws in six triangles. A handful are real stems,
    a few hundred triangles each. Welding the lot is the wrong tool: the twigs are too thin to
    survive it and what is left are flat shards strung between them.

    So tubes are taken whole, largest surface first, until the budget is spent. A twig is kept
    exactly as it is. A stem is too dear to keep, so it is redrawn as a three-sided stick that
    follows its curve and taper, a ring every third of a metre.
    """
    empty = Mesh(mesh.V[:0], None, None, np.zeros((0, 3), dtype=np.int64))
    if not len(mesh.F) or budget < 6:
        return empty
    ids = card_ids(mesh)
    a, b, c = mesh.V[mesh.F[:, 0]], mesh.V[mesh.F[:, 1]], mesh.V[mesh.F[:, 2]]
    area = np.bincount(ids, weights=0.5 * np.linalg.norm(np.cross(b - a, c - a), axis=1))
    count = np.bincount(ids)
    face_order = np.argsort(ids, kind="stable")
    face_bounds = np.searchsorted(ids[face_order], np.arange(len(count) + 1))

    kept_faces, V, VT, VN, F = [], [], [], [], []
    spent = 0
    for t in np.argsort(-area):
        if spent + 6 > budget:
            break
        faces = face_order[face_bounds[t]:face_bounds[t + 1]]
        n = int(count[t])
        if n <= 12:
            if spent + n <= budget:
                kept_faces.append(faces)
                spent += n
            continue
        pts = mesh.V[np.unique(mesh.F[faces])]
        centre = pts.mean(axis=0)
        rel = pts - centre
        _, axes = np.linalg.eigh(rel.T @ rel)
        axis, u, w = axes[:, 2], axes[:, 1], axes[:, 0]
        along = rel @ axis
        lo, hi = float(along.min()), float(along.max())
        length = hi - lo
        if length < 1e-4:
            continue
        segments = int(min(10, max(1, round(length / 0.35))))
        while segments > 1 and spent + 6 * segments > budget:
            segments -= 1
        if spent + 6 * segments > budget:
            continue
        if 6 * segments >= n:
            kept_faces.append(faces)
            spent += n
            continue

        half = max(0.02, length / (2 * segments))
        mids, radii = [], []
        for k in range(segments + 1):
            at = lo + length * k / segments
            near = np.abs(along - at) <= half
            if near.sum() < 3:
                near = np.argsort(np.abs(along - at))[:6]
            mid = rel[near].mean(axis=0)
            mid = mid + (at - mid @ axis) * axis  # the middle of the tube here, at this station
            off = rel[near] - mid
            off = off - np.outer(off @ axis, axis)
            mids.append(mid)
            radii.append(float(np.median(np.linalg.norm(off, axis=1))))
        # A fork or a kink inside one window inflates that ring. Hold each ring near the stem's
        # own typical girth, and under what its surface area says a tube this long can be.
        typical = float(np.median(radii))
        ceiling = 1.5 * float(area[t]) / (2.0 * math.pi * length)
        radii = [max(0.002, min(r, 1.5 * typical, ceiling)) for r in radii]

        base = sum(len(v) for v in V)
        ring_v, ring_t, ring_n = [], [], []
        for k in range(segments + 1):
            for j in range(3):
                turn = 2.0 * math.pi * j / 3
                out = math.cos(turn) * u + math.sin(turn) * w
                ring_v.append(centre + mids[k] + radii[k] * out)
                ring_t.append((j / 3.0, lo + length * k / segments))
                ring_n.append(out)
        V.append(np.array(ring_v))
        VT.append(np.array(ring_t))
        VN.append(np.array(ring_n))
        for k in range(segments):
            for j in range(3):
                p, q = base + 3 * k + j, base + 3 * k + (j + 1) % 3
                F.append((p, q, p + 3))
                F.append((q, q + 3, p + 3))
        spent += 6 * segments

    parts = []
    if kept_faces:
        parts.append(compact(Mesh(mesh.V, mesh.VT, mesh.VN, mesh.F[np.concatenate(kept_faces)])))
    if F:
        parts.append(Mesh(np.concatenate(V), None if mesh.VT is None else np.concatenate(VT),
                          None if mesh.VN is None else np.concatenate(VN), np.array(F, dtype=np.int64)))
    return join(parts) or empty


def lit_from_above(mesh):
    """Give placed leaves normals that face the sky, and windings that agree with them.

    A source leaf is two-sided: a front and a back layer with opposite normals. Its outline
    picks vertices from either layer, so a rebuilt leaf can come out with the back's normal on
    its lit side and draw dark grey in full sun. Every leaf is therefore re-aimed: its normal is
    flipped to the upper side and leaned toward the vertical, which is also roughly how a canopy
    scatters light. The winding follows, so a renderer that tells front from back agrees.
    """
    if not len(mesh.F):
        return mesh
    tri = np.cross(mesh.V[mesh.F[:, 1]] - mesh.V[mesh.F[:, 0]], mesh.V[mesh.F[:, 2]] - mesh.V[mesh.F[:, 0]])
    down = tri[:, 2] < 0
    F = mesh.F.copy()
    F[down] = F[down][:, ::-1]
    tri[down] *= -1.0
    # One normal per vertex, from the faces around it, already on the upper side.
    VN = np.zeros_like(mesh.V)
    for k in range(3):
        np.add.at(VN, F[:, k], tri)
    VN = VN / np.maximum(np.linalg.norm(VN, axis=1, keepdims=True), 1e-12)
    VN = VN + np.array([0.0, 0.0, 0.6])
    VN = VN / np.maximum(np.linalg.norm(VN, axis=1, keepdims=True), 1e-12)
    return Mesh(mesh.V, mesh.VT, VN, F)


def kind_of(part, mesh):
    low = (part["name"] + " " + os.path.basename(part["mesh"])).lower()
    if any(w in low for w in WOOD):
        return "wood"
    if any(w in low for w in CARDS):
        return "cards"
    n = int(card_ids(mesh).max()) + 1 if len(mesh.F) else 0
    return "cards" if (n > 50 and len(mesh.F) / n < 200) else "wood"


def reduce_plant(asset, data_dir, wood_budget, card_budget, rng, card_share):
    """One plant, expanded and reduced. Returns {material name: (part template, Mesh)}.

    card_share caps a grown leaf at that share of the plant's height.
    """
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
    for group, budget, instanced in ((trunk, trunk_budget, False), (limbs, limb_budget, True)):
        total = sum(len(m.F) * len(x) for _, m, _, x, _ in group)
        ratio = min(1.0, budget / total) if total else 0.0
        for part, mesh, _, xforms, _ in group:
            want = len(mesh.F) * ratio
            keep = 1.0
            if want < MIN_BRANCH_TRIS < len(mesh.F):
                # Too many branches for the budget: keep some whole rather than all as slivers.
                keep, want = want / MIN_BRANCH_TRIS, MIN_BRANCH_TRIS
            if ratio >= 1.0:
                small = mesh
            elif instanced:
                small = tubes(mesh, int(want))
            else:
                small = cluster(mesh, max(4, int(want)), twig_thickness(mesh))
            for xform in xforms:
                if len(small.F) and (keep >= 1.0 or rng.random() < keep):
                    emit(part, placed(small, xform))

    # Foliage. First how tall the plant stands, since that is what a leaf is sized against.
    z_lo, z_hi = math.inf, -math.inf
    for _, mesh, _, xforms, _ in items:
        lo, hi = mesh.V.min(axis=0), mesh.V.max(axis=0)
        corners = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
        for xform in xforms:
            z = corners @ np.array(xform[:9]).reshape(3, 3)[:, 2] + xform[11]
            z_lo, z_hi = min(z_lo, float(z.min())), max(z_hi, float(z.max()))
    max_size = card_share * max(z_hi - z_lo, 0.3) if items else 0.3

    cards = []
    total = 0  # triangles if every leaf were kept, each drawn as its outline
    for part, mesh, kind, xforms, _ in items:
        if kind != "cards":
            continue
        ids = card_ids(mesh)
        order = np.argsort(ids, kind="stable")
        bounds = np.searchsorted(ids[order], np.arange(ids.max() + 2))
        total += len(xforms) * int(np.minimum(np.diff(bounds), CARD_TRIS).sum())
        cards.append((part, mesh, xforms, order, bounds))
    fraction = min(1.0, card_budget / total) if total else 0.0
    grow = min(MAX_CARD_SCALE, max(1.0, COVERAGE / math.sqrt(fraction))) if fraction > 0 else 1.0
    for part, mesh, xforms, order, bounds in cards:
        if fraction <= 0.0:
            continue
        ncards = len(bounds) - 1
        want = fraction * ncards  # leaves per instance, often a fraction of one
        for xform in xforms:
            n = int(want) + (1 if rng.random() < want - int(want) else 0)
            if n == 0:
                continue
            chosen = rng.choice(ncards, size=min(n, ncards), replace=False)
            leaves = join([shape_card(mesh, order[bounds[c]:bounds[c + 1]], grow, max_size) for c in chosen])
            if leaves is not None:
                emit(part, lit_from_above(placed(leaves, xform)))

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
        merged, before = reduce_plant(a, args.dir, wood_budget, card_budget, rng, MAX_CARD_SHARE[k])

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
    man["vegetation"] = VEGETATION_VERSION
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
