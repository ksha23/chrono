#!/bin/bash
# Package a converted Mcity scene into one archive, so consumers need not convert anything.
#
# The conversion is the expensive part of using this scene: a 3.2 GB clone, a USD toolchain, and a
# pass over a few hundred assets. None of it is per-user work -- the output is identical for
# everybody -- so it is worth doing once and publishing the result.
#
# The Mcity dataset is MIT licensed (Quantum Signal AI LLC and the Regents of the University of
# Michigan), which permits redistributing modified copies provided the notice travels with them.
# This script therefore writes the upstream LICENSE into the archive; keep it there.
#
# Only files the manifests actually reference are included, which is why the archive is a fraction
# of the working directory: unreferenced intermediates and the USD sources are left behind.
#
#   ./package_mcity.sh                      package every configuration present
#   ./package_mcity.sh --out /tmp/x.tar.gz  choose the output path
#
# Publish the result as a release asset and point setup_mcity.sh --bundle at it.
set -e

DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT="$DIR"
while [ "$ROOT" != "/" ] && [ ! -d "$ROOT/src/chrono" ]; do ROOT="$(dirname "$ROOT")"; done
DATA="$ROOT/data/mcity"
OUT="$ROOT/mcity_scene_bundle.tar.gz"

while [ $# -gt 0 ]; do
  case "$1" in
    --data) DATA="$2"; shift 2 ;;
    --out) OUT="$2"; shift 2 ;;
    -h|--help) sed -n '2,18p' "$0"; exit 0 ;;
    *) echo "unknown option: $1" >&2; exit 1 ;;
  esac
done

[ -f "$DATA/mcity_scene.json" ] || { echo "no converted scene in $DATA -- run setup_mcity.sh first" >&2; exit 1; }

LIST="$(mktemp)"
trap 'rm -f "$LIST"' EXIT

python3 - "$DATA" "$LIST" <<'PY'
import json, os, sys, glob
data, listfile = sys.argv[1], sys.argv[2]
need = set()
for man in sorted(glob.glob(os.path.join(data, "mcity_scene*.json"))):
    doc = json.load(open(man))
    need.add(os.path.basename(man))
    for a in doc.get("assets", []):
        for p in a.get("parts", []):
            for key in ("mesh", "texture", "normal", "roughness", "metallic"):
                if p.get(key):
                    need.add(p[key])
for extra in ("mcity_ground.obj", "LICENSE.mcity", "README.txt"):
    if os.path.exists(os.path.join(data, extra)):
        need.add(extra)
present = sorted(f for f in need if os.path.exists(os.path.join(data, f)))
open(listfile, "w").write("\n".join(present) + "\n")
missing = len(need) - len(present)
size = sum(os.path.getsize(os.path.join(data, f)) for f in present)
print(f"  {len(present)} files, {size/1048576:.0f} MB uncompressed" + (f" ({missing} missing)" if missing else ""))
PY

# The upstream notice travels with the data, as MIT requires.
if [ ! -f "$DATA/LICENSE.mcity" ]; then
  curl -sfL -o "$DATA/LICENSE.mcity" \
    https://raw.githubusercontent.com/mcity/mcity-digital-twin/main/LICENSE || true
fi
cat > "$DATA/README.txt" <<'TXT'
Mcity digital twin, converted for Chrono.

Source:  https://github.com/mcity/mcity-digital-twin  (MIT, see LICENSE.mcity)
Derived by src/demos/vehicle/terrain/mcity/usd_to_chrono.py: USD meshes exported
to Wavefront OBJ, materials resolved to their published textures, and the
drivable surfaces merged into mcity_ground.obj for RigidTerrain.

Extract into <chrono>/data/mcity and run:  demo_SCEN_mcity
TXT

# Re-list so the notice and readme are included.
python3 - "$DATA" "$LIST" <<'PY'
import sys
data, listfile = sys.argv[1], sys.argv[2]
import os
lines = [l.strip() for l in open(listfile) if l.strip()]
for extra in ("LICENSE.mcity", "README.txt"):
    if extra not in lines and os.path.exists(os.path.join(data, extra)):
        lines.append(extra)
open(listfile, "w").write("\n".join(sorted(lines)) + "\n")
PY

echo "  writing $OUT"
tar -czf "$OUT" -C "$DATA" -T "$LIST"
echo "  done: $(du -h "$OUT" | cut -f1)"
echo
echo "publish it, then consumers need only:"
echo "  ./setup_mcity.sh --bundle <url-or-path>"
