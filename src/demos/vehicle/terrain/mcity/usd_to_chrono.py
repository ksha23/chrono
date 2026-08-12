#!/usr/bin/env python3
"""Convert a Mcity USD scene into Wavefront OBJ assets plus a placement manifest for Chrono.

Why this shape of pipeline
--------------------------
The USD root stage is a *composition*: a few hundred prims, each referencing a per-asset mesh
file and carrying its own world transform. Mcity places 798 instances from 143 distinct assets --
a better than 5:1 reuse ratio. Flattening that into one large mesh would throw the reuse away,
along with any chance of culling or per-category control, and it would leave the driving surface
tessellated when Chrono can have it analytically from OpenDRIVE.

So the structure is preserved: each distinct asset becomes one OBJ, and the placements become a
manifest of transforms. At runtime Chrono loads each mesh once and instances it, which is what
makes the scene cheap.

Wavefront OBJ is the target because it is the common denominator. Chrono's own mesh loader reads
OBJ only (ChTriangleMeshConnected::LoadWavefrontMesh); VSG could read glTF through vsgXchange, but
going that way bypasses Chrono's visual shapes and the geometry would be invisible to the
ray-traced sensor pipeline -- the one that matters for perception work.

Units and axes
--------------
The stage declares metersPerUnit 0.01, so geometry and translations are scaled to metres on the
way out. USD is Z-up here, which already matches Chrono, so no axis swap is needed. Verified
against the OpenDRIVE network: the two share a coordinate frame, with no systematic offset
(dx mean -0.92 m against 2.56 m scatter, dy +1.06 against 3.28).

Usage:  ./usd_to_chrono.py [--in DIR] [--out DIR] [--groups A,B]
"""

import argparse
import json
import math
import os
import sys
from collections import OrderedDict

try:
    from pxr import Usd, UsdGeom, UsdShade, Gf
except ImportError:
    sys.exit("usd-core is required:  python3 -m pip install usd-core")


def safe_name(path):
    """A filesystem-safe asset name derived from its USD asset path."""
    base = os.path.basename(path)
    for ext in (".usdc", ".usda", ".usd"):
        if base.endswith(ext):
            base = base[: -len(ext)]
    return "".join(c if c.isalnum() or c in "-_" else "_" for c in base)


def triangulate(counts):
    """Fan-triangulate USD faces, returning *slot* indices into the flattened face-vertex array.

    Slots rather than point indices, because UVs and normals are almost always face-varying: two
    faces meeting at one point carry different texture coordinates there, and only the slot tells
    them apart. The caller maps a slot to a point through faceVertexIndices.
    """
    tris = []
    i = 0
    for c in counts:
        if c >= 3:
            for k in range(1, c - 1):
                tris.append((i, i + k, i + k + 1))
        i += c
    return tris


def primvar_lookup(values, indices, interpolation, face_vertex_indices):
    """A function from face-vertex slot to value, whatever interpolation the primvar uses.

    Mcity authors every st primvar as indexed faceVarying. Reading it as if it were per-point --
    which is the obvious thing to try, and what an earlier version of this script did -- silently
    fails the length check and leaves every texture coordinate at zero, so each texture collapses
    to a single texel and the whole scene renders as flat colour.
    """
    if values is None or len(values) == 0:
        return None
    n = len(values)

    def get(i):
        return values[i] if 0 <= i < n else values[0]

    if interpolation == UsdGeom.Tokens.constant:
        return lambda s: get(0)
    if interpolation == UsdGeom.Tokens.faceVarying:
        if indices:
            return lambda s: get(indices[s]) if s < len(indices) else get(0)
        return get
    # vertex, varying: indexed by point, so go through the face-vertex table first.
    if indices:
        return lambda s: get(indices[face_vertex_indices[s]])
    return lambda s: get(face_vertex_indices[s])


def load_materials(path):
    """Read the surface catalogue that assigns a flat colour per name pattern.

    This is the fallback for the 14 materials whose textures were never published (the crosswalk
    signals and the mailbox, mostly). Everything else gets its real texture; see
    resolve_textures.py for how that mapping is derived, and materials.json for how to edit this.
    """
    if not os.path.exists(path):
        return {"default": {"kd": [0.62, 0.62, 0.64]}, "rules": []}
    with open(path) as f:
        return json.load(f)


def match_material(catalog, *names):
    """First matching rule wins, so specific patterns must precede general ones.

    Names are tried in order, so a material name gets to answer before the asset name it belongs
    to -- MI_McityGrass_1 should be green even on an asset called SM_Ground.
    """
    for name in names:
        if not name:
            continue
        for rule in catalog.get("rules", []):
            if rule.get("match", "") in name:
                return rule
    return catalog.get("default", {"kd": [0.62, 0.62, 0.64]})


def write_mtl(path, asset, mats):
    """A Wavefront material library, one entry per material the asset actually uses.

    Chrono reads MTL through the OBJ, and vsgXchange's OBJ reader honours map_Kd, which is what
    puts lettering on signs and asphalt on the road.
    """
    with open(path, "w") as f:
        f.write(f"# surfaces for {asset}: {len(mats)} material(s)\n")
        f.write("# textures from resolve_textures.py; flat colours from materials.json\n")
        for name, mat in mats.items():
            kd = mat.get("kd", [0.62, 0.62, 0.64])
            ks = mat.get("ks", [0.05, 0.05, 0.05])
            ns = mat.get("ns", 10.0)
            f.write(f"\nnewmtl {name}\n")
            f.write(f"Ka {kd[0]*0.35:.4f} {kd[1]*0.35:.4f} {kd[2]*0.35:.4f}\n")
            f.write(f"Kd {kd[0]:.4f} {kd[1]:.4f} {kd[2]:.4f}\n")
            f.write(f"Ks {ks[0]:.4f} {ks[1]:.4f} {ks[2]:.4f}\n")
            f.write(f"Ns {ns:.2f}\nd 1.0\nillum 2\n")
            if mat.get("map_kd"):
                f.write(f"map_Kd {mat['map_kd']}\n")


def mtl_safe(name):
    """MTL material names are whitespace-delimited tokens."""
    return "".join(c if c.isalnum() or c in "-_." else "_" for c in name)


def material_signature(placement):
    """The bound-material names under a placement, in traversal order.

    Kept deliberately cheap and separate from gather_meshes: this runs for all 2869 placements,
    while the full gather runs only for the ~230 distinct (asset, signature) pairs that actually
    become meshes. Reading point arrays here -- which an earlier version did, just to test whether
    a mesh had geometry -- meant pulling tens of millions of points through the USD bindings and
    discarding them, and dominated the runtime.
    """
    out = []
    for prim in Usd.PrimRange(placement, Usd.TraverseInstanceProxies()):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        bound = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
        out.append(bound.GetPrim().GetName() if bound else "default")
    return tuple(out)


def gather_meshes(placement, xf):
    """Every mesh under one placement, with its transform relative to the placement and the
    material bound to it *in the composed stage*.

    Reading the composed stage rather than the referenced asset file is what makes signage work.
    Mcity builds its signs from a few blank plates and binds the legend as a per-placement
    override, so SM_Rect_24x30 opened alone reports Unreal's WorldGridMaterial placeholder on the
    face, while the same face in context reports MI_R2_1_SpeedLimit_45_24x30.
    """
    root_inv = xf.GetLocalToWorldTransform(placement).GetInverse()
    out = []
    # TraverseInstanceProxies, because the foliage prims are USD *native instances*: their
    # geometry lives in a shared prototype and a plain PrimRange reports zero meshes under them.
    for prim in Usd.PrimRange(placement, Usd.TraverseInstanceProxies()):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        bound = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
        mat = bound.GetPrim().GetName() if bound else "default"
        out.append((prim, xf.GetLocalToWorldTransform(prim) * root_inv, mat))
    return out


def export_variant(meshes, out_obj, scale, surface_for):
    """Write one placement's meshes into a single OBJ, in metres, in placement-local space.

    Faces are grouped by their bound material rather than emitted as one block. That grouping is
    the whole point: most Mcity assets carry more than one material, and collapsing them loses
    exactly the distinctions that read as detail -- a sign's lettered face against its blank back,
    brick against vinyl on a facade, asphalt against concrete on SM_Roads_v2.
    """
    # Corners are (position, uv, normal) tuples. Carrying the values rather than indices keeps
    # face-varying data intact; the per-part writer welds identical corners back together.
    by_material = OrderedDict()  # material name -> [(corner, corner, corner), ...]

    for prim, m, mat_name in meshes:
        mesh = UsdGeom.Mesh(prim)
        pts = mesh.GetPointsAttr().Get()
        counts = mesh.GetFaceVertexCountsAttr().Get()
        idx = mesh.GetFaceVertexIndicesAttr().Get()
        if not pts or not counts or not idx:
            continue

        positions = []
        for p in pts:
            w = m.Transform(Gf.Vec3d(p[0], p[1], p[2]))
            positions.append((round(w[0] * scale, 5), round(w[1] * scale, 5), round(w[2] * scale, 5)))

        stv = UsdGeom.PrimvarsAPI(prim).GetPrimvar("st")
        uv_at = primvar_lookup(stv.Get() if stv else None,
                               stv.GetIndices() if (stv and stv.IsIndexed()) else None,
                               stv.GetInterpolation() if stv else None, idx)

        rot = m.ExtractRotationMatrix()
        n_at = primvar_lookup(mesh.GetNormalsAttr().Get(), None,
                              mesh.GetNormalsInterpolation(), idx)

        def corner(slot):
            p = positions[idx[slot]]
            uv = (0.0, 0.0)
            if uv_at:
                u = uv_at(slot)
                uv = (round(float(u[0]), 5), round(float(u[1]), 5))
            nn = None
            if n_at:
                v = n_at(slot)
                d = Gf.Vec3d(v[0], v[1], v[2]) * rot
                ln = math.sqrt(d[0] ** 2 + d[1] ** 2 + d[2] ** 2) or 1.0
                nn = (round(d[0] / ln, 5), round(d[1] / ln, 5), round(d[2] / ln, 5))
            return (p, uv, nn)

        group = by_material.setdefault(mat_name, [])
        for a, b, c in triangulate(counts):
            group.append((corner(a), corner(b), corner(c)))

    num_faces = sum(len(g) for g in by_material.values())
    if not num_faces:
        return None

    stem = os.path.splitext(out_obj)[0]
    base = os.path.basename(stem)

    # One OBJ per material, rather than one OBJ with usemtl groups.
    #
    # The tempting alternative is a single multi-material OBJ, which Chrono can nearly consume:
    # ChTriangleMeshConnected keeps a per-face material index and the VSG backend splits draw
    # calls by it. But Chrono calls tinyobj::LoadObj without an mtl_basedir, so the mtllib is
    # resolved against the process working directory rather than the OBJ's own, and is simply not
    # found. The face indices then come back empty while the caller still supplies N materials,
    # and the renderer walks off the end of its per-material face counts. Splitting here keeps
    # every mesh single-material, which is well-defined no matter where the process is run from.
    parts = []
    for mat_name, faces in by_material.items():
        surf = surface_for(mat_name)

        # Weld identical corners. A point shared by faces with different texture coordinates has
        # to stay split, so the key is the whole corner rather than the position alone.
        weld, order = {}, []
        for tri in faces:
            for c in tri:
                if c not in weld:
                    weld[c] = len(order)
                    order.append(c)

        have_norms = all(c[2] is not None for c in order)

        part_name = f"{base}__{mtl_safe(mat_name)}"
        part_obj = os.path.join(os.path.dirname(out_obj), part_name + ".obj")
        with open(part_obj, "w") as f:
            f.write("# converted from the Mcity USD stage by usd_to_chrono.py\n")
            f.write(f"# material {mat_name}\n")
            for p, _, _ in order:
                f.write("v {:.5f} {:.5f} {:.5f}\n".format(*p))
            for _, uv, _ in order:
                f.write("vt {:.5f} {:.5f}\n".format(*uv))
            if have_norms:
                for _, _, n in order:
                    f.write("vn {:.5f} {:.5f} {:.5f}\n".format(*n))
            for tri in faces:
                i, j, k = (weld[c] + 1 for c in tri)
                if have_norms:
                    f.write(f"f {i}/{i}/{i} {j}/{j}/{j} {k}/{k}/{k}\n")
                else:
                    f.write(f"f {i}/{i} {j}/{j} {k}/{k}\n")

        parts.append({"name": mat_name,
                      "mesh": os.path.join("assets", part_name + ".obj"),
                      "texture": surf.get("texture"),
                      "normal": surf.get("normal"),
                      "roughness": surf.get("roughness"),
                      "metallic": surf.get("metallic"),
                      "colour": surf.get("kd", [0.62, 0.62, 0.64]),
                      "ks": surf.get("ks", [0.05, 0.05, 0.05]),
                      "ns": surf.get("ns", 10.0),
                      "tris": len(faces)})

    # Still written, for anything outside Chrono that reads the OBJ set.
    write_mtl(stem + ".mtl", base,
              OrderedDict((mtl_safe(m), surface_for(m)) for m in by_material))

    textured = sum(1 for p in parts if p["texture"])
    return {"tris": num_faces, "parts": parts, "textured": textured}


def decompose(m):
    """Split a USD matrix into translation, quaternion (w,x,y,z) and scale."""
    t = m.ExtractTranslation()
    # Row lengths give the scale; dividing them out leaves a pure rotation.
    rows = [Gf.Vec3d(m[i][0], m[i][1], m[i][2]) for i in range(3)]
    scale = [r.GetLength() for r in rows]

    r4 = Gf.Matrix4d(1.0)
    for i in range(3):
        s = scale[i] if scale[i] > 1e-12 else 1.0
        for j in range(3):
            r4[i, j] = m[i][j] / s

    # A negative determinant means the transform mirrors. A quaternion cannot express that, so
    # fold the flip into the scale and leave a proper rotation behind.
    if Gf.Matrix3d(r4.ExtractRotationMatrix()).GetDeterminant() < 0:
        scale[0] = -scale[0]
        for j in range(3):
            r4[0, j] = -r4[0, j]

    quat = r4.ExtractRotationQuat()
    im = quat.GetImaginary()
    return (t, (quat.GetReal(), im[0], im[1], im[2]), scale)


def main():
    ap = argparse.ArgumentParser()
    here = os.path.dirname(os.path.abspath(__file__))
    default_dir = os.path.abspath(os.path.join(here, "..", "..", "data", "mcity"))
    ap.add_argument("--in", dest="indir", default=default_dir)
    ap.add_argument("--out", dest="outdir", default=default_dir)
    ap.add_argument("--groups", default="", help="comma-separated /Root children to include")
    # Foliage is excluded by default, and not for download-size reasons. The 18 species are
    # scan-grade assets -- Hawthorn alone is 619k triangles, the set totals 2.9M -- and Mcity
    # places 2010 of them. Chrono's triangle-mesh path builds one GPU buffer per placement rather
    # than caching by asset, so including them means roughly 320M instanced triangles. Pass
    # --exclude-groups "" to include them anyway.
    ap.add_argument("--exclude-groups", dest="exclude", default="Foliage_Instanced",
                    help="comma-separated /Root children to skip (default: Foliage_Instanced)")
    args = ap.parse_args()

    root_usd = os.path.join(args.indir, "usd", "McityMap_Main.usdc")
    if not os.path.exists(root_usd):
        sys.exit(f"missing {root_usd} -- run fetch_mcity.sh first")

    assets_dir = os.path.join(args.outdir, "assets")
    os.makedirs(assets_dir, exist_ok=True)

    catalog = load_materials(os.path.join(here, "materials.json"))
    print(f"  surface catalogue: {len(catalog.get('rules', []))} rules")

    # Textures resolved by resolve_textures.py, keyed by material. Materials without one keep
    # the catalogue's flat colour.
    tex_map = {}
    tex_json = os.path.join(args.indir, "asset_textures.json")
    if os.path.exists(tex_json):
        with open(tex_json) as f:
            tex_map = json.load(f).get("materials", {})
        print(f"  texture assignments: {len(tex_map)} materials")

    tex_dir = os.path.join(args.outdir, "textures")

    def surface_for(material_name, asset_name=""):
        """Resolve one material to a Wavefront surface: its texture if we found one, else a
        catalogue colour."""
        surf = dict(match_material(catalog, material_name, asset_name))
        entry = tex_map.get(material_name)
        if entry and os.path.exists(os.path.join(tex_dir, entry["texture"])):
            # Two spellings of the same file. map_kd is relative to the MTL, which sits beside
            # the OBJ in assets/, for any external tool that reads the OBJ. "texture" is relative
            # to the manifest, which is what Chrono resolves against.
            surf["map_kd"] = os.path.join("..", "textures", entry["texture"])
            surf["texture"] = os.path.join("textures", entry["texture"])
            # Relief and gloss maps. The base colour on its own reads flat: these are what make
            # asphalt look like asphalt rather than grey paint.
            for kind in ("normal", "roughness", "metallic"):
                name = entry.get(kind)
                if name and os.path.exists(os.path.join(tex_dir, name)):
                    surf[kind] = os.path.join("textures", name)
            # A textured surface must not also be tinted, or the texture is multiplied by the
            # catalogue colour and comes out muddy.
            surf["kd"] = [1.0, 1.0, 1.0]
        return surf

    stage = Usd.Stage.Open(root_usd)
    mpu = UsdGeom.GetStageMetersPerUnit(stage)
    up = UsdGeom.GetStageUpAxis(stage)
    if up != "Z":
        print(f"  warning: stage is {up}-up; Chrono expects Z-up and no axis conversion is applied")

    wanted = [g for g in args.groups.split(",") if g] or None
    unwanted = set(g for g in args.exclude.split(",") if g)
    xf = UsdGeom.XformCache()

    placements = []  # (asset_path, placement_prim, matrix, group)
    for prim in stage.Traverse():
        # Both composition arcs matter. The props are brought in by reference, but the 2010 trees
        # and shrubs under /Root/Foliage_Instanced arrive by payload, and looking only at
        # references silently drops every one of them.
        arcs = prim.GetMetadata("references") or prim.GetMetadata("payload")
        if not arcs:
            continue
        items = list(arcs.prependedItems) or list(arcs.appendedItems) or list(arcs.addedItems)
        if not items:
            continue

        parts = str(prim.GetPath()).split("/")
        group = parts[2] if len(parts) > 2 else "Root"
        if wanted and group not in wanted:
            continue
        if group in unwanted:
            continue

        ap_ = items[0].assetPath
        resolved = ap_ if os.path.isabs(ap_) else os.path.join(args.indir, "usd", ap_.lstrip("./"))
        placements.append((resolved, prim, xf.GetLocalToWorldTransform(prim), group))

    # Keep only leaf-most placements. Composition arcs nest: /Root/Foliage_Instanced carries a
    # payload of its own *and* contains 2009 individually-payloaded trees. Accepting the parent
    # too would flatten the entire forest into one asset and emit every tree twice -- which in
    # practice meant a 13 GB process that never finished writing its first tree.
    #
    # Paths sort so that descendants immediately follow their ancestor, so one pass suffices.
    placements.sort(key=lambda pl: str(pl[1].GetPath()))
    pruned = []
    for i, pl in enumerate(placements):
        path = str(pl[1].GetPath())
        nxt = str(placements[i + 1][1].GetPath()) if i + 1 < len(placements) else ""
        if nxt.startswith(path + "/"):
            continue
        pruned.append(pl)
    if len(pruned) != len(placements):
        print(f"  dropped {len(placements) - len(pruned)} placements that contain other placements")
    placements = pruned

    print(f"  {len(placements)} placements referencing {len(set(p[0] for p in placements))} distinct assets")

    # One OBJ per (asset, material assignment). Usually that is one per asset, but Mcity reuses a
    # few blank plates across many signs and swaps the legend per placement, so those have to
    # become separate meshes -- the geometry is identical, the surface is not.
    variants = OrderedDict()  # (asset_path, signature) -> record
    per_placement = []        # variant key or None, parallel to placements
    missing = skipped = 0
    for path, prim, _, _ in placements:
        if not os.path.exists(path):
            per_placement.append(None)
            missing += 1
            continue

        key = (path, material_signature(prim))
        if key in variants:
            per_placement.append(key if variants[key] else None)
            continue
        meshes = gather_meshes(prim, xf)

        stem = safe_name(path)
        collisions = sum(1 for k in variants if k[0] == path)
        name = stem if collisions == 0 else f"{stem}__v{collisions}"
        info = export_variant(meshes, os.path.join(assets_dir, name + ".obj"), mpu,
                              lambda m, a=stem: surface_for(m, a))
        if info is None:
            variants[key] = None
            per_placement.append(None)
            skipped += 1
            continue
        # No manifest colour: the MTL now carries a surface per submesh, and a shape-level colour
        # would flatten all of them back into one.
        variants[key] = {"name": name, "info": info}
        per_placement.append(key)

    live = [v for v in variants.values() if v]
    index = {k: i for i, (k, v) in enumerate((k, v) for k, v in variants.items() if v)}
    distinct_assets = len({k[0] for k, v in variants.items() if v})
    print(f"  converted {len(live)} meshes from {distinct_assets} assets "
          f"({missing} not downloaded, {skipped} contained no geometry)")
    tot_mat = sum(len(v["info"]["parts"]) for v in live)
    tot_tex = sum(v["info"]["textured"] for v in live)
    print(f"  {tot_mat} material slots, {tot_tex} textured ({100*tot_tex//max(tot_mat,1)}%)")

    instances = []
    for (path, _, m, group), key in zip(placements, per_placement):
        if key is None or key not in index:
            continue
        t, q, s = decompose(m)
        instances.append({
            "asset": index[key],
            "group": group,
            "pos": [round(t[0] * mpu, 4), round(t[1] * mpu, 4), round(t[2] * mpu, 4)],
            "rot": [round(v, 6) for v in q],
            "scale": [round(v, 6) for v in s],
        })

    # The collision surface, as one merged Wavefront mesh.
    #
    # RigidTerrain::AddPatch takes a single OBJ and a single transform, while the scene is a few
    # hundred instanced meshes at different placements. Merging here rather than in C++ keeps the
    # runtime side to stock Chrono: the demo just points RigidTerrain at this file.
    #
    # Ground is everything except the categories a vehicle cannot drive on. Excluding is safer
    # than listing what to include -- a whitelist silently omits any surface nobody remembered to
    # name, and the failure mode is sinking through geometry you can see.
    SKIP_GROUPS = {"Foliage_Instanced", "TrafficPoles", "StreetLights", "TrafficLights",
                   "TrafficLightCables"}
    SKIP_NAMES = ("Pole", "Sign", "TrafficLight", "Cable", "Fence", "GuardRail", "Facade",
                  "Building", "StreetLight", "Hydrant", "Bollard", "Barrier", "Container",
                  "WaterTower", "Pavilion", "Basketball", "Dumpster", "BusStop", "Bench",
                  "Chair", "Table", "Meter", "Charger", "Barrel", "Rock", "Camera",
                  "GPSBlocker", "MailBox", "TrashCan", "BikeRack")

    ground_path = os.path.join(args.outdir, "mcity_ground.obj")
    n_tri = 0
    with open(ground_path, "w") as gf:
        gf.write("# Mcity ground, merged in world space for RigidTerrain::AddPatch\n")
        base = 0
        for inst in instances:
            asset = live[inst["asset"]]
            if inst["group"] in SKIP_GROUPS or any(w in asset["name"] for w in SKIP_NAMES):
                continue
            q = inst["rot"]
            w, x, y, z = q
            R = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                 [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                 [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
            sx, sy, sz = inst["scale"]
            tx, ty, tz = inst["pos"]
            for part in asset["info"]["parts"]:
                mesh_path = os.path.join(args.outdir, part["mesh"])
                if not os.path.exists(mesh_path):
                    continue
                verts, faces = [], []
                for line in open(mesh_path):
                    if line.startswith("v "):
                        f3 = line.split()
                        verts.append((float(f3[1]) * sx, float(f3[2]) * sy, float(f3[3]) * sz))
                    elif line.startswith("f "):
                        faces.append([int(t.split("/")[0]) - 1 for t in line.split()[1:4]])
                for vx, vy, vz in verts:
                    gf.write("v {:.4f} {:.4f} {:.4f}\n".format(
                        R[0][0] * vx + R[0][1] * vy + R[0][2] * vz + tx,
                        R[1][0] * vx + R[1][1] * vy + R[1][2] * vz + ty,
                        R[2][0] * vx + R[2][1] * vy + R[2][2] * vz + tz))
                for a, b, c in faces:
                    gf.write(f"f {base+a+1} {base+b+1} {base+c+1}\n")
                    n_tri += 1
                base += len(verts)
    print(f"  ground mesh: {n_tri} triangles -> {ground_path}")

    manifest = {
        "name": "Mcity",
        "source": "github.com/mcity/mcity-digital-twin (MIT)",
        "units": "metres, Z up; converted from a USD stage with metersPerUnit %.4g" % mpu,
        "assets": [{"name": v["name"], "parts": v["info"]["parts"]} for v in live],
        "instances": instances,
    }
    out_json = os.path.join(args.outdir, "mcity_scene.json")
    with open(out_json, "w") as f:
        json.dump(manifest, f, indent=1)

    from collections import Counter
    print(f"  wrote {out_json}: {len(live)} assets, {len(instances)} instances")
    for g, n in Counter(i["group"] for i in instances).most_common():
        print(f"    {n:5d}  {g}")


if __name__ == "__main__":
    main()
