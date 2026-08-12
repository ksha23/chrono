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


def triangulate(counts, indices):
    """Fan-triangulate USD face-vertex arrays. USD allows n-gons; OBJ here stays triangles."""
    tris = []
    i = 0
    for c in counts:
        if c >= 3:
            for k in range(1, c - 1):
                tris.append((indices[i], indices[i + k], indices[i + k + 1]))
        i += c
    return tris


def load_materials(path):
    """Read the surface catalogue that assigns a colour per asset-name pattern.

    The USD assets bind Omniverse MDL materials defined in an external Materials/*.mdl library,
    and only 3 of 139 carry a BaseColorMap inline, so nothing usable can be read out of the USD
    itself. The catalogue supplies plausible surfaces instead; see materials.json for the
    reasoning and for how to edit it.
    """
    if not os.path.exists(path):
        return {"default": {"kd": [0.62, 0.62, 0.64]}, "rules": []}
    with open(path) as f:
        return json.load(f)


def match_material(catalog, asset_name):
    """First matching rule wins, so specific patterns must precede general ones."""
    for rule in catalog.get("rules", []):
        if rule.get("match", "") in asset_name:
            return rule
    return catalog.get("default", {"kd": [0.62, 0.62, 0.64]})


def write_mtl(path, name, mat):
    """A minimal Wavefront material. Chrono reads MTL through the OBJ, which is how the shipped
    vehicle meshes get their colours."""
    kd = mat.get("kd", [0.62, 0.62, 0.64])
    ks = mat.get("ks", [0.05, 0.05, 0.05])
    ns = mat.get("ns", 10.0)
    with open(path, "w") as f:
        f.write(f"# surface for {name}, from materials.json\n")
        f.write(f"newmtl {name}\n")
        f.write(f"Ka {kd[0]*0.35:.4f} {kd[1]*0.35:.4f} {kd[2]*0.35:.4f}\n")
        f.write(f"Kd {kd[0]:.4f} {kd[1]:.4f} {kd[2]:.4f}\n")
        f.write(f"Ks {ks[0]:.4f} {ks[1]:.4f} {ks[2]:.4f}\n")
        f.write(f"Ns {ns:.2f}\nd 1.0\nillum 2\n")
        if mat.get("map_kd"):
            f.write(f"map_Kd {mat['map_kd']}\n")


def export_asset(asset_path, out_obj, scale, material):
    """Write every mesh in one USD asset file into a single OBJ, in metres.

    Sub-prim transforms inside the asset are baked in, so the OBJ is self-contained and the
    manifest only has to carry the placement.
    """
    stage = Usd.Stage.Open(asset_path)
    if not stage:
        return None

    xf = UsdGeom.XformCache()
    verts, norms, uvs, faces = [], [], [], []

    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Mesh):
            continue
        mesh = UsdGeom.Mesh(prim)
        pts = mesh.GetPointsAttr().Get()
        counts = mesh.GetFaceVertexCountsAttr().Get()
        idx = mesh.GetFaceVertexIndicesAttr().Get()
        if not pts or not counts or not idx:
            continue

        m = xf.GetLocalToWorldTransform(prim)
        base = len(verts)
        for p in pts:
            w = m.Transform(Gf.Vec3d(p[0], p[1], p[2]))
            verts.append((w[0] * scale, w[1] * scale, w[2] * scale))

        # Texture coordinates are carried through even though the catalogue currently supplies
        # flat colours: it costs little and is what a future texture pass would need.
        stv = UsdGeom.PrimvarsAPI(prim).GetPrimvar("st")
        vals = stv.Get() if stv else None
        if vals is not None and len(vals) == len(pts):
            uvs.extend((float(u[0]), float(u[1])) for u in vals)
        else:
            uvs.extend((0.0, 0.0) for _ in pts)

        n = mesh.GetNormalsAttr().Get()
        if n and len(n) == len(pts):
            rot = m.ExtractRotationMatrix()
            for v in n:
                d = Gf.Vec3d(v[0], v[1], v[2]) * rot
                ln = math.sqrt(d[0] ** 2 + d[1] ** 2 + d[2] ** 2) or 1.0
                norms.append((d[0] / ln, d[1] / ln, d[2] / ln))

        for t in triangulate(counts, idx):
            faces.append((t[0] + base, t[1] + base, t[2] + base))

    if not verts or not faces:
        return None

    name = os.path.splitext(os.path.basename(out_obj))[0]
    write_mtl(os.path.splitext(out_obj)[0] + ".mtl", name, material)

    have_norms = len(norms) == len(verts)
    have_uvs = len(uvs) == len(verts)
    with open(out_obj, "w") as f:
        f.write(f"# converted from {os.path.basename(asset_path)} by usd_to_chrono.py\n")
        f.write(f"mtllib {name}.mtl\n")
        for v in verts:
            f.write(f"v {v[0]:.5f} {v[1]:.5f} {v[2]:.5f}\n")
        if have_uvs:
            for t in uvs:
                f.write(f"vt {t[0]:.5f} {t[1]:.5f}\n")
        if have_norms:
            for n in norms:
                f.write(f"vn {n[0]:.5f} {n[1]:.5f}\n" if False else f"vn {n[0]:.5f} {n[1]:.5f} {n[2]:.5f}\n")
        f.write(f"usemtl {name}\n")
        for a, b, c in faces:
            if have_norms and have_uvs:
                f.write(f"f {a+1}/{a+1}/{a+1} {b+1}/{b+1}/{b+1} {c+1}/{c+1}/{c+1}\n")
            elif have_norms:
                f.write(f"f {a+1}//{a+1} {b+1}//{b+1} {c+1}//{c+1}\n")
            else:
                f.write(f"f {a+1} {b+1} {c+1}\n")

    return {"verts": len(verts), "tris": len(faces), "colour": material.get("kd")}


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
    args = ap.parse_args()

    root_usd = os.path.join(args.indir, "usd", "McityMap_Main.usdc")
    if not os.path.exists(root_usd):
        sys.exit(f"missing {root_usd} -- run fetch_mcity.sh first")

    assets_dir = os.path.join(args.outdir, "assets")
    os.makedirs(assets_dir, exist_ok=True)

    catalog = load_materials(os.path.join(here, "materials.json"))
    print(f"  surface catalogue: {len(catalog.get('rules', []))} rules")

    stage = Usd.Stage.Open(root_usd)
    mpu = UsdGeom.GetStageMetersPerUnit(stage)
    up = UsdGeom.GetStageUpAxis(stage)
    if up != "Z":
        print(f"  warning: stage is {up}-up; Chrono expects Z-up and no axis conversion is applied")

    wanted = [g for g in args.groups.split(",") if g] or None
    xf = UsdGeom.XformCache()

    placements = []  # (asset_path, matrix, group)
    for prim in stage.Traverse():
        refs = prim.GetMetadata("references")
        if not refs:
            continue
        parts = str(prim.GetPath()).split("/")
        group = parts[2] if len(parts) > 2 else "Root"
        if wanted and group not in wanted:
            continue
        for item in refs.prependedItems:
            ap_ = item.assetPath
            resolved = ap_ if os.path.isabs(ap_) else os.path.join(args.indir, "usd", ap_.lstrip("./"))
            placements.append((resolved, xf.GetLocalToWorldTransform(prim), group))
            break

    print(f"  {len(placements)} placements referencing {len(set(p[0] for p in placements))} distinct assets")

    # Convert each distinct asset once.
    assets = OrderedDict()
    missing = skipped = 0
    for path, _, _ in placements:
        if path in assets:
            continue
        if not os.path.exists(path):
            assets[path] = None
            missing += 1
            continue
        name = safe_name(path)
        obj = os.path.join(assets_dir, name + ".obj")
        material = match_material(catalog, name)
        info = export_asset(path, obj, mpu, material)
        if info is None:
            assets[path] = None
            skipped += 1
            continue
        assets[path] = {"name": name, "mesh": os.path.join("assets", name + ".obj"),
                        "colour": material.get("kd")}

    live = [a for a in assets.values() if a]
    index = {path: i for i, path in enumerate(p for p, a in assets.items() if a)}
    print(f"  converted {len(live)} assets ({missing} not downloaded, {skipped} contained no geometry)")

    instances = []
    for path, m, group in placements:
        if not assets.get(path):
            continue
        t, q, s = decompose(m)
        instances.append({
            "asset": index[path],
            "group": group,
            "pos": [round(t[0] * mpu, 4), round(t[1] * mpu, 4), round(t[2] * mpu, 4)],
            "rot": [round(v, 6) for v in q],
            "scale": [round(v, 6) for v in s],
        })

    manifest = {
        "name": "Mcity",
        "source": "github.com/mcity/mcity-digital-twin (MIT)",
        "units": "metres, Z up; converted from a USD stage with metersPerUnit %.4g" % mpu,
        "assets": [{"name": a["name"], "mesh": a["mesh"], "colour": a["colour"]} for a in live],
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
