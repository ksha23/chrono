#!/bin/bash
# Package a converted Mcity scene into release archives, so consumers need not convert anything.
#
# The conversion is the expensive part of using this scene: a 3.2 GB clone, a USD toolchain, and a
# pass over a few hundred assets. None of it is per-user work -- the output is identical for
# everybody -- so it is worth doing once and publishing the result.
#
# The Mcity dataset is MIT licensed (Quantum Signal AI LLC and the Regents of the University of
# Michigan), which permits redistributing modified copies provided the notice travels with them.
# This script therefore writes the upstream LICENSE into each archive; keep it there.
#
# Two archives, because vegetation triples the download and the default demo does not load it:
#
#   mcity_scene_base.tar.gz      mcity_scene.json, the ground mesh, and everything they reference
#   mcity_scene_foliage.tar.gz   the vegetation levels and their meshes; extracts over the base
#
# Only files a manifest actually references are included, which is why the archives are a fraction
# of the working directory: unreferenced intermediates and the USD sources are left behind.
# mcity_scene_foliage.json is left out on purpose. It is the undecimated input to
# decimate_foliage.py, the demo never loads it, and it alone would add 400 MB of scan-grade meshes.
#
#   ./package_mcity.sh                      write the archives and SHA256SUMS to data/mcity/dist
#   ./package_mcity.sh --out-dir DIR        choose where
#   ./package_mcity.sh --upstream-rev SHA   the mcity-digital-twin commit this was converted from
#
# Publish the archives as release assets, then pin the URL and hashes at the top of setup_mcity.sh.
set -e

DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT="$DIR"
while [ "$ROOT" != "/" ] && [ ! -d "$ROOT/src/chrono" ]; do ROOT="$(dirname "$ROOT")"; done
DATA="$ROOT/data/mcity"
OUTDIR=""
UPSTREAM_REV=""

while [ $# -gt 0 ]; do
  case "$1" in
    --data) DATA="$2"; shift 2 ;;
    --out-dir) OUTDIR="$2"; shift 2 ;;
    --upstream-rev) UPSTREAM_REV="$2"; shift 2 ;;
    -h|--help) sed -n '2,26p' "$0"; exit 0 ;;
    *) echo "unknown option: $1" >&2; exit 1 ;;
  esac
done
[ -n "$OUTDIR" ] || OUTDIR="$DATA/dist"

[ -f "$DATA/mcity_scene.json" ] || { echo "no converted scene in $DATA -- run setup_mcity.sh --convert first" >&2; exit 1; }
mkdir -p "$OUTDIR"

# Provenance, so an archive can be traced back to what produced it. The HTTPS fetch leaves the
# upstream commit in repo_tree.json; a conversion from a local clone has to be told.
if [ -z "$UPSTREAM_REV" ] && [ -s "$DATA/repo_tree.json" ]; then
  UPSTREAM_REV="$(python3 -c "import json,sys; print(json.load(open(sys.argv[1])).get('sha',''))" "$DATA/repo_tree.json" 2>/dev/null || true)"
fi
CONVERTER_REV="$(git -C "$ROOT" log -1 --format=%H -- "$DIR/usd_to_chrono.py" "$DIR/resolve_textures.py" \
  "$DIR/decimate_foliage.py" "$DIR/build_configs.sh" "$DIR/materials.json" "$DIR/semantics.json" 2>/dev/null || true)"

# The upstream notice travels with the data, as MIT requires.
if [ ! -f "$DATA/LICENSE.mcity" ]; then
  curl -sfL -o "$DATA/LICENSE.mcity" \
    https://raw.githubusercontent.com/mcity/mcity-digital-twin/main/LICENSE || true
fi
[ -f "$DATA/LICENSE.mcity" ] || { echo "could not obtain the upstream LICENSE; refusing to package without it" >&2; exit 1; }

cat > "$DATA/README.txt" <<TXT
Mcity digital twin, converted for Chrono.

Source:     https://github.com/mcity/mcity-digital-twin  (MIT, see LICENSE.mcity)
Upstream:   ${UPSTREAM_REV:-not recorded}
Converter:  ${CONVERTER_REV:-not recorded}  (Chrono, src/demos/vehicle/terrain/mcity)

Derived by usd_to_chrono.py: USD meshes exported to Wavefront OBJ, materials read from
their shaders, instanced branches expanded and reduced, and the drivable surfaces
merged into mcity_ground.obj for RigidTerrain. Labels, signal lamps, the sky and the
road network ride along in the manifest.

Extract into <chrono>/data/mcity and run:  demo_VEH_McityDrive
TXT

python3 - "$DATA" "$OUTDIR" <<'PY'
import json, os, sys, glob
data, outdir = sys.argv[1], sys.argv[2]
REFS = ("mesh", "texture", "normal", "roughness", "metallic", "ao", "opacity", "emissive_texture")
NOTICE = {"LICENSE.mcity", "README.txt"}
SOURCE_ONLY = "mcity_scene_foliage.json"  # input to decimate_foliage.py, never loaded by the demo

def closure(manifests):
    need = set()
    for man in manifests:
        need.add(os.path.basename(man))
        doc = json.load(open(man))
        for a in doc.get("assets", []):
            for p in a.get("parts", []):
                need.update(p[k] for k in REFS if p.get(k))
        # The sky panorama and the road network ride with whichever manifest names them.
        need.update(doc[k] for k in ("sky", "road_network") if doc.get(k))
    return need

base = closure([os.path.join(data, "mcity_scene.json")]) | {"mcity_ground.obj"}
levels = sorted(m for m in glob.glob(os.path.join(data, "mcity_scene_*.json"))
                if os.path.basename(m) != SOURCE_ONLY)
archives = [("base", base)]
if levels:
    archives.append(("foliage", closure(levels) - base))

for name, need in archives:
    need = need | NOTICE
    missing = sorted(f for f in need if not os.path.exists(os.path.join(data, f)))
    if missing:
        # An archive with holes still extracts and still loads, minus whatever was absent, so
        # this has to stop here rather than be discovered by whoever downloads it.
        sys.exit(f"  {name}: {len(missing)} referenced files are missing, e.g. {missing[:3]}")
    with open(os.path.join(outdir, f".{name}.list"), "w") as f:
        f.write("\n".join(sorted(need)) + "\n")
    size = sum(os.path.getsize(os.path.join(data, f)) for f in need)
    print(f"  {name}: {len(need)} files, {size/1048576:.0f} MB uncompressed")
PY

sha256_of() {
  if command -v sha256sum >/dev/null 2>&1; then sha256sum "$1" | cut -d' ' -f1
  else shasum -a 256 "$1" | cut -d' ' -f1; fi
}

: > "$OUTDIR/SHA256SUMS"
for name in base foliage; do
  LIST="$OUTDIR/.$name.list"
  [ -f "$LIST" ] || continue
  ARCHIVE="mcity_scene_$name.tar.gz"
  echo "  writing $OUTDIR/$ARCHIVE"
  # No extended attributes and no AppleDouble entries: an archive made on macOS otherwise unpacks
  # elsewhere with a ._ twin beside every file.
  COPYFILE_DISABLE=1 tar --no-xattrs -czf "$OUTDIR/$ARCHIVE" -C "$DATA" -T "$LIST"
  rm -f "$LIST"
  echo "$(sha256_of "$OUTDIR/$ARCHIVE")  $ARCHIVE" >> "$OUTDIR/SHA256SUMS"
  echo "    $(du -h "$OUTDIR/$ARCHIVE" | cut -f1)"
done

echo
cat "$OUTDIR/SHA256SUMS"
echo
echo "publish these as release assets, then pin the URL and hashes at the top of setup_mcity.sh"
