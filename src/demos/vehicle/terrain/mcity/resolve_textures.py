#!/usr/bin/env python3
"""Work out which base-colour texture belongs to each Mcity *material*, and fetch those textures.

The join key is the material, not the asset
-------------------------------------------
An earlier version of this script matched texture names against *asset* names and did badly:
most assets carry several materials, so a single texture per asset is the wrong shape of answer,
and fuzzy name matching happily painted a road-sign texture across the entire road surface.

The USD binds each mesh to a material prim whose name follows a strict convention, and the
texture library follows the matching one:

    material  MI_Dumpster_s001_Signage_Atlas
    texture   T_Dumpster_s001_Signage_Atlas_BC.png

So resolution is a rename, not a guess: strip the MI_/M_ prefix, prepend T_, append _BC. That
alone resolves 125 of the 149 materials in the scene exactly. Note this is *not* what
info:mdl:sourceAsset says -- nearly every shader in the published USD points at
MI_McityFacades_s001_Vinyl.mdl regardless of what it actually is, an artefact of how the stage
was exported. The material prim name survived that export intact; the MDL reference did not.

What is left over
-----------------
The stragglers are the generic surfaces shared across the map -- asphalt, sidewalk concrete,
grass -- whose materials are named for their role while the textures are named for their
substance (MI_McityAsphaltDark against T_Asphalt_Mcity_BC.png). There are twelve of them and
they cover most of the ground you can see, so they get an explicit table below rather than a
heuristic. Materials that resolve to nothing fall back to a flat colour from materials.json.

Only the textures actually referenced get downloaded, which is a few hundred megabytes rather
than the 1.7 GB of maps in the repository.

Usage:  ./resolve_textures.py [--dir DATA] [--no-fetch]
"""

import argparse
import glob
import json
import os
import re
import subprocess
import sys
from collections import Counter


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

try:
    from pxr import Usd, UsdGeom, UsdShade
except ImportError:
    sys.exit("usd-core is required:  python3 -m pip install usd-core")

REPO = "mcity/mcity-digital-twin"
RAW = f"https://raw.githubusercontent.com/{REPO}/main"

# Materials whose name describes their role rather than the texture's subject. Verified by hand
# against the texture library; there is no rule that would produce these.
ALIASES = {
    "MI_McityAsphaltDark":   "T_Asphalt_Mcity_BC.png",
    "MI_McityPebbles":       "T_Asphalt_2_Mcity_BC.png",
    # Named for the master material it was derived from, not for what it is: this covers 85% of
    # SM_Roads_v2, i.e. the carriageway itself, which at Mcity is asphalt. Flip it back to
    # T_CrackedConcrete_Mcity_BC.png if you disagree -- it is one line and nothing else depends
    # on the choice.
    "MI_McityConcreteDark":  "T_Asphalt_Mcity_BC.png",
    "MI_McityConcreteWarm":  "T_CrackedConcrete_Mcity_BC.png",
    "MI_McityConcreteLight": "T_Sidewalk_Mcity_BC.png",
    "MI_McitySidewalks":     "T_Sidewalk_Mcity_BC.png",
    "MI_McityGrass_1":       "T_Grass_Mcity_BC.png",
    "MI_WaterTower_s001":    "T_WaterTower_s001_Mcity_BC.png",
    "LaneMarking1_Marking":  "LaneMarking1_Diff.png",
    "MI_Pavilion_s001_Metal_Sheet": "T_Pavilion_s001_CoatedMetal_Sheet_BC.png",
    # Deliberately left unmapped, so the colour catalogue keeps them yellow rather than
    # tinting a white paint texture:  LaneMarkingYellow1_Marking.
}

# Suffixes appearing on material names but never on the corresponding texture.
DROP_SUFFIX = ("_NSR",)

# Map-type tags, which appear immediately before the extension. Anchoring matters: as a bare
# substring "_met" also matches "_Metal_Atlas_BC", which quietly disqualified every metal
# material in the scene from having a texture at all.
MAP_TYPE = re.compile(r"_(nrm|orm|met|rgh|spec|ao|mask|opacity|alph|n|m)\.(png|jpg)$", re.I)

# Utility maps that are nobody's surface colour. MacroVariation in particular is a near-white
# overlay for breaking up tiling; as a diffuse map it washed the road and the ground out to pale
# grey. These are safe to match anywhere in the name.
NOT_BASE_COLOUR = ("macrovariation", "variation", "detail", "noise", "grid", "testmaterial")
PLACEHOLDER = "t_default"


def is_base_colour(name):
    low = name.lower()
    if MAP_TYPE.search(low):
        return False
    return not any(b in low for b in NOT_BASE_COLOUR) and PLACEHOLDER not in low


def repo_tree(cache):
    """The repository file listing, cached on disk.

    Fetched with curl rather than urllib: the GitHub API closes urllib connections here, and the
    listing is large enough that re-fetching it on every run is wasteful anyway.
    """
    if os.path.exists(cache):
        with open(cache) as f:
            return json.load(f)["tree"]

    url = f"https://api.github.com/repos/{REPO}/git/trees/main?recursive=1"
    out = subprocess.run(["curl", "-sfL", url], capture_output=True, text=True)
    if out.returncode != 0 or not out.stdout:
        sys.exit("could not fetch the repository listing")
    with open(cache, "w") as f:
        f.write(out.stdout)
    return json.loads(out.stdout)["tree"]


def collect_materials(root_usd):
    """Every material bound anywhere in the composed scene, plus any inline shader texture.

    This reads the *composed* root stage rather than the per-asset files, and the difference is
    not cosmetic. Mcity builds its signage from a handful of blank plates -- SM_Rect_24x30 and
    friends -- and binds the legend as a per-placement override in the root stage. Open the asset
    file on its own and the sign face reports Unreal's WorldGridMaterial placeholder; compose the
    stage and the same face reports MI_R2_1_SpeedLimit_45_24x30, which is the material that
    actually has a texture behind it.
    """
    stage = Usd.Stage.Open(root_usd)
    names, inline, inline_aux = set(), {}, {}
    # Instance proxies included: the foliage is natively instanced, and its materials are only
    # reachable through the prototype.
    for prim in stage.Traverse(Usd.TraverseInstanceProxies()):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        mat = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
        if not mat:
            continue
        mname = mat.GetPrim().GetName()
        names.add(mname)
        for shader in mat.GetPrim().GetChildren():
            # Two spellings, because the scene mixes two shader families. The props carry
            # BaseColorMap; the vegetation is OmniPBR and names it diffuse_texture. Reading only
            # the first left every tree and shrub untextured -- rendered flat grey, which for
            # geometry that is 70-85% leaves is most of what you see off-road.
            for attr in ("inputs:BaseColorMap", "inputs:diffuse_texture"):
                a = shader.GetAttribute(attr)
                if a and a.Get():
                    base = os.path.basename(str(a.Get().path))
                    if base and is_base_colour(base):
                        inline.setdefault(mname, base)
            for attr, kind in (("inputs:normalmap_texture", "normal"),
                               ("inputs:ORM_texture", "orm")):
                a = shader.GetAttribute(attr)
                if a and a.Get():
                    base = os.path.basename(str(a.Get().path))
                    if base:
                        inline_aux.setdefault(mname, {}).setdefault(kind, base)
    return sorted(names), inline, inline_aux


def companions(base_colour, tex_index):
    """The normal, roughness and metallic maps that sit beside a base-colour texture.

    Worth the extra download. Mcity's surface textures are fine-grained aggregate authored to be
    lit through a normal map -- on their own, at roughly one tile per metre, the speckle falls
    below a pixel at any normal viewing distance and averages out to flat grey. The relief is what
    reads as asphalt.
    """
    m = re.match(r"^(.*?)_(bc|diff|basecolor|albedo)\.(png|jpg)$", base_colour, re.I)
    if not m:
        return {}
    stem, ext = m.group(1), m.group(3)

    out = {}
    for kind, tags in (("normal", ("nrm", "normal", "n")),
                       ("roughness", ("rgh", "roughness")),
                       ("metallic", ("met", "metallic"))):
        for tag in tags:
            key = f"{stem}_{tag}.{ext}".lower()
            if key in tex_index:
                out[kind] = os.path.basename(tex_index[key])
                break
    return out


def squash(name):
    """Lowercase and drop separators, so R2_1 and R2-1 compare equal.

    The two conventions are mixed even within a single pair: the material is
    MI_R2_1_SpeedLimit_45_24x30 and its texture is T_R2-1_SpeedLimit_45_24x30.png.
    """
    return re.sub(r"[-_\s]", "", name.lower())


def mdl_texture(material, mdl_dir, tex_index):
    """Base-colour texture named by a collected MDL, if there is one.

    The vegetation shaders carry no inline inputs at all -- they point at an MDL beside the
    SubUSD, e.g. ./materials/TreeBark_10.mdl -- so without reading those files every tree and
    shrub resolves to nothing and renders flat grey.
    """
    path = os.path.join(mdl_dir, material + ".mdl")
    if not os.path.isfile(path):
        return None
    body = open(path, errors="ignore").read()
    found = [os.path.basename(m.group(1)) for m in re.finditer(r'"([^"]*\.(?:png|jpg))"', body)]
    return pick_base_colour([f for f in found if f.lower() in tex_index])


def pick_base_colour(candidates):
    """Prefer an explicit base-colour suffix, else the first usable map."""
    usable = [c for c in candidates if is_base_colour(c)]
    for c in usable:
        if any(g in c.lower() for g in ("_bc", "_diff", "basecolor", "albedo")):
            return c
    return usable[0] if usable else None


def resolve(material, tex_index, tex_squashed, inline, mdl_dir):
    """Return (texture_basename, how) for one material, or (None, 'none')."""
    if material in inline:
        return inline[material], "inline"

    hit = mdl_texture(material, mdl_dir, tex_index)
    if hit:
        return hit, "mdl"

    if material in ALIASES and ALIASES[material].lower() in tex_index:
        return ALIASES[material], "alias"

    stem = material
    for pre in ("MI_", "M_"):
        if stem.startswith(pre):
            stem = stem[len(pre):]
            break
    for suf in DROP_SUFFIX:
        if stem.endswith(suf):
            stem = stem[: -len(suf)]
    stem = stem.lower()

    for suffix in ("_bc", "_diff", "_basecolor", "_albedo", ""):
        for ext in (".png", ".jpg"):
            key = f"t_{stem}{suffix}{ext}"
            if key in tex_index and is_base_colour(key):
                return os.path.basename(tex_index[key]), "rule"

    # Same rule, but insensitive to how the two sides punctuate themselves.
    for suffix in ("_bc", "_diff", "_basecolor", "_albedo", ""):
        for ext in (".png", ".jpg"):
            key = squash(f"t_{stem}{suffix}{ext}")
            hit = tex_squashed.get(key)
            if hit and is_base_colour(hit):
                return os.path.basename(hit), "rule"
    return None, "none"


def main():
    ap = argparse.ArgumentParser()
    here = os.path.dirname(os.path.abspath(__file__))
    ap.add_argument("--dir", default=chrono_data_dir())
    ap.add_argument("--no-fetch", action="store_true")
    # Chrono's VSG backend uploads textures as uncompressed RGBA, so on-disk PNG size is not the
    # cost that matters: 568 maps at mostly 2048 square is 8.0 GB resident, which on its own put
    # the process near 9.5 GB before any foliage was loaded. Base colour keeps enough resolution
    # to read sign legends; normal, roughness and metallic are low-frequency and lose nothing
    # visible at half that. 0 disables resizing.
    ap.add_argument("--max-base", type=int, default=1024, help="max base-colour edge (0 = keep)")
    ap.add_argument("--max-aux", type=int, default=512, help="max normal/roughness/metallic edge")
    args = ap.parse_args()

    tree = repo_tree(os.path.join(args.dir, "repo_tree.json"))
    tex_index, tex_squashed = {}, {}
    for t in tree:
        p = t["path"]
        if p.lower().endswith((".png", ".jpg")):
            tex_index.setdefault(os.path.basename(p).lower(), p)
            tex_squashed.setdefault(squash(os.path.basename(p)), p)
    print(f"  textures available in the repository: {len(tex_index)}")

    mdl_dir = os.path.join(args.dir, "mdl")
    root_usd = os.path.join(args.dir, "usd", "McityMap_Main.usdc")
    if not os.path.exists(root_usd):
        sys.exit(f"missing {root_usd} -- run fetch_mcity.sh first")
    materials, inline, inline_aux = collect_materials(root_usd)
    print(f"  distinct materials bound in the composed scene: {len(materials)}")

    resolved, how, unresolved = {}, Counter(), []
    for m in materials:
        tex, layer = resolve(m, tex_index, tex_squashed, inline, mdl_dir)
        how[layer] += 1
        if tex:
            entry = {"texture": tex, "how": layer}
            entry.update(companions(tex, tex_index))
            # An inline normal map named by the shader beats one guessed from the base-colour name.
            aux = inline_aux.get(m, {})
            if aux.get("normal") and aux["normal"].lower() in tex_index:
                entry["normal"] = aux["normal"]
            resolved[m] = entry
        else:
            unresolved.append(m)

    extra = Counter(k for v in resolved.values() for k in ("normal", "roughness", "metallic")
                    if v.get(k))
    print("  companion maps: " + ", ".join(f"{extra[k]} {k}" for k in
                                           ("normal", "roughness", "metallic")))

    print(f"  materials resolved to a texture: {len(resolved)}/{len(materials)}")
    for layer in ("inline", "mdl", "rule", "alias", "none"):
        if how[layer]:
            print(f"    {how[layer]:4d}  {layer}")
    if unresolved:
        print(f"  falling back to a catalogue colour: {', '.join(unresolved)}")

    needed = sorted({t for v in resolved.values()
                     for t in (v.get("texture"), v.get("normal"), v.get("roughness"),
                               v.get("metallic")) if t})
    out_dir = os.path.join(args.dir, "textures")
    if not args.no_fetch and needed:
        os.makedirs(out_dir, exist_ok=True)
        todo = [t for t in needed if not os.path.exists(os.path.join(out_dir, t))]
        if todo:
            print(f"  fetching {len(todo)} of {len(needed)} textures")
            pairs = []
            for t in todo:
                pairs += [f"{RAW}/{tex_index[t.lower()]}", os.path.join(out_dir, t)]
            subprocess.run(
                "xargs -n2 -P8 sh -c 'curl -sfL --retry 3 -o \"$1\" \"$0\"'",
                input="\n".join(pairs), text=True, shell=True)
        have = [t for t in needed if os.path.exists(os.path.join(out_dir, t))]
        size = sum(os.path.getsize(os.path.join(out_dir, t)) for t in have)
        print(f"  have {len(have)}/{len(needed)} textures, {size/1e6:.0f} MB")

        if args.max_base or args.max_aux:
            aux = {t for v in resolved.values()
                   for t in (v.get("normal"), v.get("roughness"), v.get("metallic")) if t}
            shrunk, before, after = 0, 0, 0
            for t in have:
                limit = args.max_aux if t in aux else args.max_base
                path = os.path.join(out_dir, t)
                try:
                    from PIL import Image
                    im = Image.open(path)
                    w, h = im.size
                    before += w * h * 4
                    if limit and max(w, h) > limit:
                        k = limit / max(w, h)
                        im = im.resize((max(1, int(w * k)), max(1, int(h * k))), Image.LANCZOS)
                        im.save(path)
                        shrunk += 1
                    after += im.size[0] * im.size[1] * 4
                except Exception as e:
                    print(f"    could not resize {t}: {e}")
            print(f"  resized {shrunk} textures; resident RGBA "
                  f"{before/1e9:.1f} GB -> {after/1e9:.1f} GB")

    out = {"materials": resolved}
    with open(os.path.join(args.dir, "asset_textures.json"), "w") as f:
        json.dump(out, f, indent=1)
    print(f"  wrote {os.path.join(args.dir, 'asset_textures.json')}")


if __name__ == "__main__":
    main()
