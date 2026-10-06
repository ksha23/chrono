#!/usr/bin/env python3
"""Work out which textures and surface settings belong to each Mcity *material*, and fetch them.

The shader is the first source
------------------------------
Each material prim carries a shader whose inputs name its maps directly: BaseColor_Map,
Normal_Map, Roughness_Map, Metallic_Map, a packed AO_Rough_Metal_Map, an OpacityMap, an
emissive mask. Those inputs are read first and everything below is the fallback for the
materials whose shader names nothing. An earlier version read only two of the three spellings
of the base-colour input and guessed the rest from names, which left ten materials flat grey
that the stage does give a texture, and gave five ground materials the wrong one.

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
# The ground materials used to be here too. They are two-layer blends and the shader says exactly
# which two textures, so they are now resolved from it: see LAYERED.
ALIASES = {
    "MI_WaterTower_s001":    "T_WaterTower_s001_Mcity_BC.png",
    "LaneMarking1_Marking":  "LaneMarking1_Diff.png",
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

# Shader inputs that name a map, by role. The scene mixes three shader families and each spells
# its inputs differently: the props (BaseColor_Map), the signs (BaseColorMap) and the OmniPBR
# vegetation and signal lamps (diffuse_texture).
INPUTS = {
    "texture":   ("BaseColor_Map", "BaseColorMap", "diffuse_texture"),
    "normal":    ("Normal_Map", "NormalMap", "normalmap_texture"),
    "roughness": ("Roughness_Map", "RoughnessMap", "reflectionroughness_texture"),
    "metallic":  ("Metallic_Map", "MetallicMap"),
    "orm":       ("AO_Rough_Metal_Map", "ORM_texture"),
    "opacity":   ("OpacityMap", "Opacity_Map"),
    "emissive_texture": ("emissive_mask_texture",),
}

# Stand-ins the exporter left where a material has no map of that kind.
NOT_A_MAP = ("t_default", "t_empty", "worldgrid")


def real_map(name):
    return bool(name) and not any(p in name.lower() for p in NOT_A_MAP)


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
    """Every material bound anywhere in the composed scene, with what its shader says.

    This reads the *composed* root stage rather than the per-asset files, and the difference is
    not cosmetic. Mcity builds its signage from a handful of blank plates -- SM_Rect_24x30 and
    friends -- and binds the legend as a per-placement override in the root stage. Open the asset
    file on its own and the sign face reports Unreal's WorldGridMaterial placeholder; compose the
    stage and the same face reports MI_R2_1_SpeedLimit_45_24x30, which is the material that
    actually has a texture behind it.

    Returns the material names and, per material, {"maps": role -> file, "values": input -> value}.
    """
    stage = Usd.Stage.Open(root_usd)
    names, shader = set(), {}
    # Instance proxies included: the foliage is natively instanced, and its materials are only
    # reachable through the prototype.
    for prim in stage.Traverse(Usd.TraverseInstanceProxies()):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        mat = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
        if not mat:
            continue
        mname = mat.GetPrim().GetName()
        if mname in names:
            continue
        names.add(mname)
        maps, values = {}, {}
        for sh in mat.GetPrim().GetChildren():
            for attr in sh.GetAttributes():
                name = attr.GetName()
                if not name.startswith("inputs:"):
                    continue
                v = attr.Get()
                if v is None:
                    continue
                key = name[len("inputs:"):]
                if hasattr(v, "path"):
                    if v.path:
                        values[key] = os.path.basename(str(v.path))
                elif isinstance(v, (bool, int, float, str)):
                    values[key] = v
                else:
                    try:
                        values[key] = [float(x) for x in v]
                    except TypeError:
                        pass
        for role, spellings in INPUTS.items():
            for spelling in spellings:
                f = values.get(spelling)
                if isinstance(f, str) and real_map(f) and (role != "texture" or is_base_colour(f)):
                    maps[role] = f
                    break
        shader[mname] = {"maps": maps, "values": values}
    return sorted(names), shader


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


def resolve(material, tex_index, tex_squashed, shader, mdl_dir):
    """Return (texture_basename, how) for one material, or (None, 'none')."""
    named = shader.get(material, {}).get("maps", {}).get("texture")
    if named and named.lower() in tex_index:
        return named, "shader"

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


def settings(values, maps):
    """Shader constants that change how a material looks and that Chrono can represent."""
    out = {}

    def vec(key, n=3):
        v = values.get(key)
        return [float(x) for x in v[:n]] if isinstance(v, list) and len(v) >= n else None

    def num(key):
        v = values.get(key)
        return float(v) if isinstance(v, (int, float)) and not isinstance(v, bool) else None

    # Everything that multiplies the base colour, folded into one tint.
    tint = [1.0, 1.0, 1.0]
    for key in ("AlbedoTint", "BaseColorTint", "diffuse_tint"):
        v = vec(key)
        if v:
            tint = [a * b for a, b in zip(tint, v)]
    brightness = num("albedo_brightness")
    if brightness is not None:
        tint = [a * brightness for a in tint]
    if any(abs(a - 1.0) > 0.01 for a in tint):
        out["tint"] = [round(a, 4) for a in tint]

    # The props remap their roughness map into a range instead of using it as authored.
    lo = num("MinRoughness") if num("MinRoughness") is not None else num("RoughnessMIN")
    hi = num("MaxRoughness") if num("MaxRoughness") is not None else num("RoughnessMAX")
    if lo is not None and hi is not None and (abs(lo) > 0.005 or abs(hi - 1.0) > 0.005):
        out["roughness_range"] = [round(lo, 3), round(hi, 3)]

    # Constants that stand in for a map the material does not have.
    if "roughness" not in maps and "orm" not in maps:
        r = vec("Roughness", 1) or ([num("reflection_roughness_constant")] if num("reflection_roughness_constant") is not None else None)
        if r:
            out["roughness_value"] = round(r[0], 3)
    if "metallic" not in maps and "orm" not in maps:
        m = num("Metallic") if num("Metallic") is not None else num("Metallic_Value")
        if m is None:
            m = num("metallic_constant")
        if m is not None and m > 0.005:
            out["metallic_value"] = round(m, 3)

    scale = vec("texture_scale", 2)
    if scale and any(abs(a - 1.0) > 0.005 for a in scale):
        out["uv_scale"] = [round(a, 4) for a in scale]

    if values.get("enable_emission") is True and vec("emissive_color"):
        out["emissive"] = [round(a, 4) for a in vec("emissive_color")]
        if num("emissive_intensity") is not None:
            out["emissive_intensity"] = num("emissive_intensity")
    return out


def layered(values, tex_index):
    """The two-layer ground materials: roads, grass, sidewalks, gravel.

    These are Unreal terrain blends. Each mixes two tinted textures at their own tiling through a
    noise mask,

        mask   = clamp((noise * TerrainBlendAmount) ** TerrainBlendContrast, 0, 1)
        colour = lerp(T1 * T1BaseColorTint, T2 * T2BaseColorTint, mask)

    and Chrono has one texture per material. So the blend is baked: the layer the mask favours
    keeps its detail and its tiling, and the other one contributes its average colour in
    proportion to how much of the surface it covers. Returns the recipe, or None.
    """
    t1, t2, blend = values.get("T1BaseColorMap"), values.get("T2BaseColorMap"), values.get("TerrainBlendTexture")
    if not (isinstance(t1, str) and isinstance(t2, str) and isinstance(blend, str)):
        return None
    if not all(f.lower() in tex_index for f in (t1, t2, blend)):
        return None

    def num(key, default):
        v = values.get(key)
        return float(v) if isinstance(v, (int, float)) and not isinstance(v, bool) else default

    def tint(key):
        v = values.get(key)
        return [float(x) for x in v[:3]] if isinstance(v, list) else [1.0, 1.0, 1.0]

    return {
        "t1": t1, "t2": t2, "blend": blend,
        "tint1": tint("T1BaseColorTint"), "tint2": tint("T2BaseColorTint"),
        "scale1": num("T1ScaleNEAR", 1.0), "scale2": num("T2ScaleNEAR", 1.0),
        "amount": num("TerrainBlendAmount", 1.0), "contrast": num("TerrainBlendContrast", 1.0),
        "normal1": values.get("T1NormalMap"), "normal2": values.get("T2NormalMap"),
        "roughness1": values.get("T1RoughnessMap"), "roughness2": values.get("T2RoughnessMap"),
    }


def stem_of(name):
    return os.path.splitext(name)[0]


def derive(material, entry, out_dir):
    """Write the maps that have to be computed rather than downloaded. Returns the new files.

    Chrono's VSG backend ignores a material's colour once it has a texture, takes roughness and
    metalness as two separate maps, and has no remap or blend. So tints, packed ORM maps,
    roughness ranges and layer blends are all turned into plain image files here.
    """
    from PIL import Image, ImageChops, ImageStat
    made = []

    def path(name):
        return os.path.join(out_dir, name)

    def tinted(im, rgb):
        im = im.convert("RGBA") if im.mode in ("RGBA", "LA") else im.convert("RGB")
        bands = list(im.split())
        for i in range(3):
            k = max(0.0, rgb[i])
            bands[i] = bands[i].point(lambda v, k=k: min(255, int(v * k + 0.5)))
        return Image.merge(im.mode, bands)

    recipe = entry.get("layers")
    if recipe and all(os.path.exists(path(recipe[k])) for k in ("t1", "t2", "blend")):
        noise = Image.open(path(recipe["blend"])).convert("RGB").split()[0].resize((256, 256))
        a, c = recipe["amount"], recipe["contrast"]
        mask = noise.point(lambda v: int(255 * min(1.0, max(v / 255.0 * a, 1e-6) ** c) + 0.5))
        share2 = ImageStat.Stat(mask).mean[0] / 255.0  # how much of the surface is layer 2
        top, other = ("2", "1") if share2 >= 0.5 else ("1", "2")
        weight = share2 if top == "2" else 1.0 - share2
        base = tinted(Image.open(path(recipe["t" + top])), recipe["tint" + top]).convert("RGB")
        under = tinted(Image.open(path(recipe["t" + other])), recipe["tint" + other]).convert("RGB")
        mean = tuple(int(v + 0.5) for v in ImageStat.Stat(under).mean[:3])
        out = Image.blend(Image.new("RGB", base.size, mean), base, weight)
        name = f"{material}-blend.png"
        out.save(path(name))
        made.append(name)
        entry["texture"] = name
        recipe["share2"] = round(share2, 3)
        scale = abs(recipe["scale" + top])
        if abs(scale - 1.0) > 0.005:
            entry["uv_scale"] = [round(scale, 4), round(scale, 4)]
        for kind in ("normal", "roughness"):
            f = recipe.get(kind + top)
            if isinstance(f, str) and real_map(f) and os.path.exists(path(f)):
                entry[kind] = f
            else:
                entry.pop(kind, None)
        entry.pop("tint", None)  # already in the bake

    tint = entry.get("tint")
    if tint and entry.get("texture") and os.path.exists(path(entry["texture"])):
        tag = "".join(f"{min(255, int(v * 255 + 0.5)):02x}" for v in tint)
        name = f"{stem_of(entry['texture'])}-tint{tag}.png"
        tinted(Image.open(path(entry["texture"])), tint).save(path(name))
        made.append(name)
        entry["texture"] = name

    orm = entry.get("orm")
    if orm and os.path.exists(path(orm)):
        bands = Image.open(path(orm)).convert("RGB").split()
        for band, kind, tag in ((0, "ao", "ao"), (1, "roughness", "rough"), (2, "metallic", "metal")):
            if kind in entry and kind != "ao":
                continue
            name = f"{stem_of(orm)}-{tag}.png"
            bands[band].save(path(name))
            made.append(name)
            entry[kind] = name

    rng = entry.get("roughness_range")
    if rng and entry.get("roughness") and os.path.exists(path(entry["roughness"])):
        lo, hi = rng
        name = f"{stem_of(entry['roughness'])}-r{int(lo * 100 + 0.5):03d}-{int(hi * 100 + 0.5):03d}.png"
        Image.open(path(entry["roughness"])).convert("L").point(
            lambda v: min(255, max(0, int((lo + (hi - lo) * v / 255.0) * 255 + 0.5)))).save(path(name))
        made.append(name)
        entry["roughness"] = name
    return made


# The sky. The stage carries a sky material but binds it to nothing, so the dome texture is easy
# to miss: it is only ever named inside an MDL.
SKY = "T_Sky_Blue.png"


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
    materials, shader = collect_materials(root_usd)
    print(f"  distinct materials bound in the composed scene: {len(materials)}")

    AUX = ("normal", "roughness", "metallic", "orm", "opacity", "emissive_texture")
    resolved, how, unresolved = {}, Counter(), []
    for m in materials:
        info = shader.get(m, {"maps": {}, "values": {}})
        maps, values = info["maps"], info["values"]
        entry = {}
        recipe = layered(values, tex_index)
        if recipe:
            entry["layers"] = recipe
            layer = "layers"
        else:
            tex, layer = resolve(m, tex_index, tex_squashed, shader, mdl_dir)
            if tex:
                entry["texture"] = tex
                entry.update(companions(tex, tex_index))
        how[layer] += 1
        if layer == "none":
            unresolved.append(m)
        # Anything the shader names outright beats a companion guessed from the base-colour name.
        for kind in AUX:
            f = maps.get(kind)
            if f and f.lower() in tex_index:
                entry[kind] = f
        if values.get("NormalStrength") == 0:
            entry.pop("normal", None)  # authored, then switched off
        entry.update(settings(values, maps))
        if "emissive" not in entry:
            entry.pop("emissive_texture", None)
        if entry:
            entry["how"] = layer
            resolved[m] = entry

    extra = Counter(k for v in resolved.values() for k in AUX + ("layers", "tint", "emissive") if v.get(k))
    print("  maps and settings found: " + ", ".join(f"{extra[k]} {k}" for k in sorted(extra)))
    textured = sum(1 for v in resolved.values() if v.get("texture") or v.get("layers"))
    print(f"  materials resolved to a texture: {textured}/{len(materials)}")
    for layer in ("shader", "layers", "mdl", "rule", "alias", "none"):
        if how[layer]:
            print(f"    {how[layer]:4d}  {layer}")
    if unresolved:
        print(f"  falling back to a catalogue colour: {', '.join(unresolved)}")

    needed = set()
    for v in resolved.values():
        needed.update(v[k] for k in ("texture",) + AUX if v.get(k))
        recipe = v.get("layers")
        if recipe:
            needed.update(f for f in (recipe[k] for k in ("t1", "t2", "blend", "normal1", "normal2",
                                                           "roughness1", "roughness2"))
                          if isinstance(f, str) and real_map(f) and f.lower() in tex_index)
    if SKY.lower() in tex_index:
        needed.add(SKY)
    needed = sorted(needed)

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

        try:
            import PIL  # noqa: F401
        except ImportError:
            sys.exit("Pillow is required to build the derived maps:  python3 -m pip install pillow")

        made = []
        for m, entry in resolved.items():
            made += derive(m, entry, out_dir)
        if made:
            print(f"  derived {len(made)} maps: tints, split ORM, roughness ranges, layer blends")

        # The sky dome, as Chrono wants it: see sky_dome().
        sky = sky_dome(out_dir, args.dir)

        if args.max_base or args.max_aux:
            from PIL import Image
            base = {v["texture"] for v in resolved.values() if v.get("texture")}
            shrunk, before, after = 0, 0, 0
            for t in sorted(set(have) | set(made)):
                limit = args.max_base if t in base else args.max_aux
                path = os.path.join(out_dir, t)
                try:
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
    else:
        sky = None

    out = {"materials": resolved}
    if sky:
        out["sky"] = sky
    with open(os.path.join(args.dir, "asset_textures.json"), "w") as f:
        json.dump(out, f, indent=1)
    print(f"  wrote {os.path.join(args.dir, 'asset_textures.json')}")


def sky_dome(tex_dir, data_dir):
    """Turn Unreal's sky-sphere texture into the full-sphere panorama Chrono's sky dome takes.

    The source runs from the horizon at its bottom edge to the zenith at its top, once around.
    Chrono maps a dome texture over the whole sphere, so the sky goes in the upper half and the
    lower half, which sits below the ground and is never seen, repeats the horizon colour.
    """
    src = os.path.join(tex_dir, SKY)
    if not os.path.exists(src):
        return None
    from PIL import Image
    sky = Image.open(src).convert("RGB")
    w = 2048
    upper = sky.resize((w, w // 2), Image.LANCZOS)
    horizon = upper.crop((0, w // 2 - 2, w, w // 2 - 1)).resize((w, w // 2))
    pano = Image.new("RGB", (w, w))
    pano.paste(upper, (0, 0))
    pano.paste(horizon, (0, w // 2))
    os.makedirs(os.path.join(data_dir, "sky"), exist_ok=True)
    out = os.path.join("sky", "mcity_sky.jpg")
    pano.save(os.path.join(data_dir, out), quality=92)
    return out


if __name__ == "__main__":
    main()
