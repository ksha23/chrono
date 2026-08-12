#!/bin/bash
# Build the four Mcity vegetation configurations.
#
# The source assets are Omniverse scan-grade: 80k-620k triangles per plant, and Mcity places 2009
# of them for 264M triangles a frame. A rasterizer here sustains roughly 220M triangles a second,
# so the full set is unusable and every configuration below is a different answer to what to give
# up. Each writes its meshes to its own directory, because sharing one silently rewrites the
# meshes another manifest still points at.
#
# Run after usd_to_chrono.py --exclude-groups "" has produced mcity_scene_foliage.json.
#
# Usage:  ./build_configs.sh
set -e
DIR="$(cd "$(dirname "$0")" && pwd)"
# Locate the Chrono root by walking up to the marker directory, rather than counting "..".
# These scripts have moved once already and the relative depth silently went wrong.
ROOT="$DIR"
while [ "$ROOT" != "/" ] && [ ! -d "$ROOT/src/chrono" ]; do ROOT="$(dirname "$ROOT")"; done
if [ ! -d "$ROOT/src/chrono" ]; then
  echo "could not locate the Chrono source root above $DIR" >&2
  exit 1
fi
cd "$ROOT"
DATA=data/mcity
DEC="python3 $DIR/decimate_foliage.py"

# The foliage manifest names meshes under assets/. A previous non-foliage conversion can leave
# that directory without them, and the decimator would then quietly skip every tree -- producing
# manifests that reference meshes which do not exist.
if [ -f "$DATA/mcity_scene_foliage.json" ]; then
  MISSING=$(python3 - "$DATA" <<'PY2'
import json, os, sys
data = sys.argv[1]
doc = json.load(open(os.path.join(data, "mcity_scene_foliage.json")))
missing = sum(1 for a in doc["assets"] for p in a["parts"]
              if not os.path.exists(os.path.join(data, p["mesh"])))
print(missing)
PY2
)
  if [ "$MISSING" -gt 0 ]; then
    echo "  $MISSING source meshes named by mcity_scene_foliage.json are missing from $DATA/assets" >&2
    echo "  re-run:  ./setup_mcity.sh --foliage --skip-fetch" >&2
    exit 1
  fi
fi

if [ ! -f "$DATA/mcity_scene_foliage.json" ]; then
  echo "missing $DATA/mcity_scene_foliage.json -- run:" >&2
  echo "  python3 $DIR/usd_to_chrono.py --exclude-groups \"\"" >&2
  exit 1
fi

echo "== 1/4  trees only, bare branches =="
$DEC --leaf-fraction 0 --drop-shrubs \
     --lod-dir lod_trees_bare --out $DATA/mcity_scene_trees_bare.json | tail -2

echo "== 2/4  trees and shrubs, bare branches =="
$DEC --leaf-fraction 0 \
     --lod-dir lod_all_bare --out $DATA/mcity_scene_all_bare.json | tail -2

echo "== 3/4  trees only, with leaves =="
$DEC --leaf-fraction 0.4 --drop-shrubs \
     --lod-dir lod_trees_leaf --out $DATA/mcity_scene_trees_leaf.json | tail -2

echo "== 4/4  everything: trees and shrubs, with leaves =="
$DEC --leaf-fraction 0.4 \
     --lod-dir lod_full --out $DATA/mcity_scene_full.json | tail -2

echo
echo "configurations (all take STREAM_RADIUS=90 TILE_SIZE=40):"
for m in mcity_scene mcity_scene_trees_bare mcity_scene_all_bare mcity_scene_trees_leaf mcity_scene_full; do
  [ -f "$DATA/$m.json" ] || continue
  printf "  %-28s %s\n" "$m.json" \
    "$("$DIR/probe_scenery.sh" "$DATA/$m.json" 0 6 2>/dev/null | grep -oE 'triangles, as drawn: +[0-9]+' | tr -s ' ')"
done
